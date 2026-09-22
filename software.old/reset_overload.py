#!/usr/bin/env python3
"""
reset_overload.py -- clear latched overload on CRABORA servos without a
power cycle.

STS3215 servos latch the overload (and most other) error bits until
Torque_Enable is toggled off then back on. This scans for servos
currently reporting ERR_OVERLOAD, toggles torque off -> pause -> on for
each, and re-checks. If a servo is still flagged afterward, the torque
toggle didn't clear it -- that servo needs an actual power cycle (some
firmware revs latch overheat/overcurrent harder than a torque toggle
can reach).

SAFETY NOTE: re-enabling torque drives the joint straight back to its
last goal position. If the overload was a mechanical bind (a leg jammed
against the frame, a linkage caught, etc.), turning torque back on can
immediately re-trip the fault -- or drive harder against whatever it hit.
Use --limp to clear the error and leave torque OFF afterward, so you can
hand-check that the joint moves freely before re-enabling it yourself.

Usage:
    python reset_overload.py                # scan, reset any in overload
    python reset_overload.py 23 31           # reset only these IDs
    python reset_overload.py --limp          # clear + leave torque off
    python reset_overload.py --limp 23       # same, for one servo
"""

import sys
import time

from crabora_bus import (
    MultiBus,
    describe_id,
    build_packet,
    HEADER,
    INST_READ,
    decode_error_byte,
    ERR_OVERLOAD,
    ADDR_TORQUE_ENABLE,
    ADDR_PRESENT_POSITION,
)

RESET_PAUSE_S = 0.3  # dwell with torque off before re-enabling


def read_with_error(bus, servo_id, address, n):
    """Read n bytes and return (params, error_byte).

    Reimplemented locally -- see check_overload.py for why: Bus's own
    read helpers print the error byte but don't return it.
    """
    packet = build_packet(servo_id, INST_READ, bytes([address, n]))
    urt = bus.urt
    urt.reset_input_buffer()
    urt.write(packet)
    reply_len = 6 + n
    reply = urt.read(reply_len)
    if len(reply) < reply_len or reply[:2] != HEADER or reply[2] != servo_id:
        raise IOError(f"bad/short reply from {servo_id}: {reply.hex()}")
    return reply[5:5 + n], reply[4]


def get_error(mb, sid):
    bus = mb._bus_for(sid)
    _, err = read_with_error(bus, sid, ADDR_PRESENT_POSITION, 2)
    return err


def reset_servo(mb, sid, leave_limp):
    label = describe_id(sid)

    try:
        before = get_error(mb, sid)
    except IOError as e:
        print(f"  ?  {sid:3d}  {label:16s} -- could not read before reset: {e}")
        return False

    print(f"  -> {sid:3d}  {label:16s} before: {decode_error_byte(before) or 'ok'}")

    try:
        mb.write_uint8(sid, ADDR_TORQUE_ENABLE, 0)
        time.sleep(RESET_PAUSE_S)
        if not leave_limp:
            mb.write_uint8(sid, ADDR_TORQUE_ENABLE, 1)
    except IOError as e:
        print(f"        reset write failed: {e}")
        return False

    time.sleep(0.05)
    try:
        after = get_error(mb, sid)
    except IOError as e:
        print(f"        could not verify after reset: {e}")
        return False

    if after == 0:
        state = "left limp (torque off) -- check it moves freely, then re-enable" if leave_limp else "cleared, torque re-enabled"
        print(f"        {state}.")
        return True

    print(f"        still flagged: {decode_error_byte(after)} "
          f"-- torque toggle didn't clear it, needs an actual power cycle.")
    return False


def main():
    args = sys.argv[1:]
    leave_limp = "--limp" in args
    explicit_ids = [int(a) for a in args if a != "--limp"]

    with MultiBus() as mb:
        if explicit_ids:
            targets = explicit_ids
            print(f"\nResetting {len(targets)} servo(s) by explicit ID: {targets}\n")
        else:
            print("\nScanning for overloaded servos...\n")
            targets = []
            for sid in sorted(mb.live_ids):
                try:
                    err = get_error(mb, sid)
                except IOError:
                    continue
                if err & ERR_OVERLOAD:
                    targets.append(sid)

            if not targets:
                print("No servos currently reporting overload. Nothing to reset.")
                return

            print(f"Found {len(targets)} servo(s) in overload: {targets}\n")

        if leave_limp:
            print("(--limp: clearing error, leaving torque OFF afterward)\n")

        cleared, still_bad = [], []
        for sid in targets:
            (cleared if reset_servo(mb, sid, leave_limp) else still_bad).append(sid)

        print()
        if cleared:
            print(f"OK: {cleared}")
        if still_bad:
            print(f"Needs a power cycle: {still_bad}")


if __name__ == "__main__":
    main()
