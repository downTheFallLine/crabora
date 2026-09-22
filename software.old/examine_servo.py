#!/usr/bin/env python3
"""
check_overload.py -- scan every live CRABORA servo for overload, and
try to say *why* it's in overload (overcurrent, overheat, torque-limit
pinned, voltage, angle-limit, etc.)

Built on crabora_bus.py / MultiBus. Run it the same way as new_stand.py:

    python check_overload.py

Why this isn't just a one-liner on top of crabora_bus.py:
  - The Feetech error/status byte (overload, overheat, voltage, ...) rides
    back on EVERY reply packet at offset 4. Bus._send_and_receive() reads
    it and prints a warning if it's non-zero, but only returns the
    parameter bytes to its callers -- the error byte itself is thrown
    away. This script needs the actual bits, not just the print, so it
    re-does one raw read to get both.
  - crabora_bus.py's ADDR_* table doesn't (yet) include Present Load,
    Present Current, or Torque Limit -- only position/speed/voltage/temp.
    Those three registers are what let you tell overcurrent/stall apart
    from overheat apart from "torque limit set too low for the job".
    Their addresses below are the standard Feetech STS/SCS memory-table
    locations, NOT verified against your specific firmware rev -- if the
    numbers look implausible (e.g. current pegged at some huge value on
    every servo), that's the first thing to double check against your
    servo's datasheet.
"""

import sys

from crabora_bus import (
    MultiBus,
    describe_id,
    build_packet,
    HEADER,
    INST_READ,
    decode_error_byte,
    ERR_OVERLOAD,
    ERR_OVERHEAT,
    ERR_VOLTAGE,
    ERR_ANGLE_LIMIT,
    ADDR_PRESENT_VOLTAGE,
    ADDR_PRESENT_TEMP,
    ADDR_PRESENT_SPEED,
    ADDR_PRESENT_POSITION,
)

# -----------------------------------------------------------------------------
# Registers NOT currently in crabora_bus.py's ADDR_* table (see docstring).
# -----------------------------------------------------------------------------
ADDR_TORQUE_LIMIT    = 48  # 2 bytes -- 0..1000 = 0..100.0% of max torque
ADDR_PRESENT_LOAD    = 60  # 2 bytes -- bit15 = direction, bits0-9 = 0.1% units
ADDR_PRESENT_CURRENT = 69  # 2 bytes -- raw units, ~6.5 mA/LSB on STS3215 (approx)

# Temperature above which we'll flag "running hot" even if the error bit
# hasn't latched yet (STS3215 shutdown default is commonly ~70-85 C
# depending on firmware config -- adjust if you've set your own limit).
HOT_THRESHOLD_C = 70

# Load fraction (of torque limit) above which we call it "pinned" -- i.e.
# the servo is pushing at/near its ceiling, which is what a stall/bind
# looks like from the outside.
PINNED_LOAD_PCT = 90.0


def read_with_error(bus, servo_id, address, n):
    """Read n bytes AND return the error byte from the same reply.

    Reimplements Bus._send_and_receive()'s framing locally, because that
    method only hands back the parameter bytes -- not the error byte --
    to its callers.
    """
    packet = build_packet(servo_id, INST_READ, bytes([address, n]))
    urt = bus.urt
    urt.reset_input_buffer()
    urt.write(packet)
    reply_len = 6 + n
    reply = urt.read(reply_len)
    if len(reply) < reply_len or reply[:2] != HEADER or reply[2] != servo_id:
        raise IOError(f"bad/short reply from {servo_id}: {reply.hex()}")
    error_byte = reply[4]
    params = reply[5:5 + n]
    return params, error_byte


def safe_read_uint16(mb, sid, address, label):
    try:
        return mb.read_uint16(sid, address)
    except IOError as e:
        print(f"        {label}: (read failed -- {e})")
        return None


def diagnose(err, temp, load_pct, current_raw):
    """Best-effort single-line explanation for why overload tripped."""
    if err & ERR_OVERHEAT or temp >= HOT_THRESHOLD_C:
        return f"overheat -- {temp} C, sustained load/current for too long"
    if load_pct is not None and load_pct >= PINNED_LOAD_PCT:
        return f"load pinned at {load_pct:.1f}% of torque limit -- likely mechanical bind/stall"
    if err & ERR_VOLTAGE:
        return "bus voltage out of range -- check battery/UBEC sag under load"
    if err & ERR_ANGLE_LIMIT:
        return "commanded position outside firmware angle limits"
    if current_raw is not None and current_raw > 0:
        return f"current elevated (raw {current_raw}) -- no other flag set, worth trending over time"
    return "overload flagged but no single obvious cause -- check for mechanical binding at this joint"


def main():
    with MultiBus() as mb:
        ids = mb.live_ids
        if not ids:
            print("\nNo servos found on the bus -- nothing to check.")
            return

        print(f"\nChecking {len(ids)} servo(s) for overload...\n")
        overloaded = []

        for sid in sorted(ids):
            label = describe_id(sid)
            bus = mb._bus_for(sid)  # need the raw urt/error-byte path below

            try:
                _, err = read_with_error(bus, sid, ADDR_PRESENT_POSITION, 2)
            except IOError as e:
                print(f"  ?  {sid:3d}  {label:16s} -- could not read: {e}")
                continue

            temp = mb.read_uint8(sid, ADDR_PRESENT_TEMP)
            voltage = mb.read_uint8(sid, ADDR_PRESENT_VOLTAGE) / 10.0

            if not (err & ERR_OVERLOAD):
                print(f"  .  {sid:3d}  {label:16s} ok   "
                      f"(temp {temp:3d} C, volt {voltage:4.1f} V)")
                continue

            # --- overloaded: pull the extra context ---
            speed = safe_read_uint16(mb, sid, ADDR_PRESENT_SPEED, "speed")
            torque_limit_raw = safe_read_uint16(mb, sid, ADDR_TORQUE_LIMIT, "torque limit")
            load_raw = safe_read_uint16(mb, sid, ADDR_PRESENT_LOAD, "load")
            current_raw = safe_read_uint16(mb, sid, ADDR_PRESENT_CURRENT, "current")

            load_pct = None
            load_dir = ""
            if load_raw is not None:
                load_pct = (load_raw & 0x3FF) / 10.0
                load_dir = "-" if (load_raw & 0x400) else "+"

            torque_limit_pct = torque_limit_raw / 10.0 if torque_limit_raw is not None else None

            other_bits = err & ~ERR_OVERLOAD
            other_names = decode_error_byte(other_bits)

            print(f"  !  {sid:3d}  {label:16s} OVERLOAD")
            print(f"        temp:          {temp} C")
            print(f"        voltage:       {voltage:.1f} V")
            if speed is not None:
                print(f"        speed (raw):   {speed}")
            if load_pct is not None:
                print(f"        load:          {load_dir}{load_pct:.1f}% of torque limit")
            if torque_limit_pct is not None:
                print(f"        torque limit:  {torque_limit_pct:.1f}%")
            if current_raw is not None:
                print(f"        current (raw): {current_raw}  (unverified scale -- see script header)")
            if other_names:
                print(f"        also flagged:  {other_names}")

            cause = diagnose(err, temp, load_pct, current_raw)
            print(f"        likely cause:  {cause}")
            print()

            overloaded.append(sid)

        print()
        if overloaded:
            print(f"{len(overloaded)} servo(s) in overload: {overloaded}")
            sys.exit(1)
        else:
            print("No servos currently reporting overload.")


if __name__ == "__main__":
    main()
