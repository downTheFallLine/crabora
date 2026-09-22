#!/usr/bin/env python3
"""
Set a servo's firmware angle limits (ADDR_MIN_ANGLE / ADDR_MAX_ANGLE) to
+/- a given number of degrees around center (position 2048).

This writes to the servo's locked EEPROM area, so it uses the
eeprom_unlocked() lock-dance from crabora_bus. The limit persists on the
servo across power cycles until changed again.

Usage:
    python3 set_servo_range.py --id 11 --degrees 45

    -> writes MIN_ANGLE = deg_to_pos(-45), MAX_ANGLE = deg_to_pos(+45)
       i.e. the servo will refuse (report ERR_ANGLE_LIMIT) any goal
       position outside [2048 - 45*11.378, 2048 + 45*11.378].
"""

import argparse
import sys

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE,
    ADDR_MAX_ANGLE,
    ADDR_PRESENT_POSITION,
    CENTER_POSITION,
    SIGN_BIT,
    MAX_SPEED_MAGNITUDE,
    deg_to_pos,
    describe_id,
    rpm_to_pos_speed,
)

# NOT defined in crabora_bus.py -- added here. Per the STS3215 register map,
# address 31 ("Position Offset", 2 bytes, EEPROM) is the calibration value
# that shifts what raw encoder reading gets reported as Present Position.
# This is what "one-key middle point setting" actually writes.
ADDR_POSITION_OFFSET = 31

CENTER_TIMEOUT = 3.0  # seconds to wait for the servo to reach center
MAX_OFFSET_MAGNITUDE = 4095  # sanity clamp; a full-turn's worth is more than any real calibration needs


def decode_signed(raw):
    """Sign-magnitude decode (bit 15 = sign), same convention as goal-speed."""
    magnitude = raw & MAX_SPEED_MAGNITUDE
    return -magnitude if (raw & SIGN_BIT) else magnitude


def encode_signed(value):
    """Inverse of decode_signed, clamped to a sane offset range."""
    value = max(-MAX_OFFSET_MAGNITUDE, min(value, MAX_OFFSET_MAGNITUDE))
    magnitude = abs(value)
    return (magnitude | SIGN_BIT) if value < 0 else magnitude


def main():
    parser = argparse.ArgumentParser(
        description="Move a servo to center (2048), then set its angle limits to +/- degrees around it"
    )
    parser.add_argument("--id", type=int, required=True, help="Servo ID")
    parser.add_argument("--degrees", type=float, required=True,
                         help="Allowed travel in degrees, +/- from center (e.g. 45 -> range is -45..+45)")
    parser.add_argument("--speed", type=float, default=10.0,
                         help="Speed (RPM) used to drive to center before setting limits (default: 10)")
    parser.add_argument("--ports", nargs="*", default=None,
                         help="Restrict discovery to specific URT ports (default: auto-discover all)")
    parser.add_argument("--set-center", action="store_true",
                         help="Also recalibrate the servo's EEPROM middle point (Position Offset, "
                              "register 31) so its CURRENT physical position becomes 2048. "
                              "Use this only when the horn is already physically where you want "
                              "'center' to be -- it redefines the reference point, it does not move the servo.")
    args = parser.parse_args()

    if args.degrees <= 0:
        print("--degrees must be a positive number (it's applied as both +/-)")
        sys.exit(1)

    min_pos = deg_to_pos(-args.degrees)
    max_pos = deg_to_pos(args.degrees)

    with MultiBus(ports=args.ports) as mb:
        if args.id not in mb.live_ids:
            print(f"Servo {args.id} ({describe_id(args.id)}) was not found during "
                  f"discovery. Live IDs: {mb.live_ids}. Check power/cable.")
            sys.exit(1)

        # --- Actually drive the servo to center (2048) first ---
        print(f"Moving servo {args.id} ({describe_id(args.id)}) to center ({CENTER_POSITION})...")
        mb.enable_torque(args.id, True)
        mb.write_goal_move(args.id, CENTER_POSITION, rpm_to_pos_speed(args.speed))
        if not mb.wait_until_stopped(args.id, timeout=CENTER_TIMEOUT):
            print(f"  ! did not report stopped within {CENTER_TIMEOUT}s "
                  f"(check for stall / overload) -- continuing anyway")

        actual_pos = mb.read_uint16(args.id, ADDR_PRESENT_POSITION)
        print(f"  present position: {actual_pos} (target was {CENTER_POSITION})")

        if args.set_center:
            # Recalibrate the EEPROM middle point so THIS physical position
            # becomes 2048 from now on (persists across power cycles).
            current_offset_raw = mb.read_uint16(args.id, ADDR_POSITION_OFFSET)
            current_offset = decode_signed(current_offset_raw)
            delta = CENTER_POSITION - actual_pos
            new_offset = current_offset + delta
            print(f"  calibrating center: current offset={current_offset}, "
                  f"delta={delta}, new offset={new_offset}")

            with mb.eeprom_unlocked(args.id):
                mb.write_uint16(args.id, ADDR_POSITION_OFFSET, encode_signed(new_offset))

            readback_offset = decode_signed(mb.read_uint16(args.id, ADDR_POSITION_OFFSET))
            reported_pos = mb.read_uint16(args.id, ADDR_PRESENT_POSITION)
            print(f"  offset now: {readback_offset} -- present position now reports: {reported_pos}")
            if readback_offset != new_offset:
                print("  ! offset readback does not match what was written -- verify manually")
                sys.exit(1)

        print(f"Setting angle limits: center={CENTER_POSITION}, +/-{args.degrees} deg "
              f"-> min={min_pos}, max={max_pos}")

        with mb.eeprom_unlocked(args.id):
            mb.write_uint16(args.id, ADDR_MIN_ANGLE, min_pos)
            mb.write_uint16(args.id, ADDR_MAX_ANGLE, max_pos)

        # Read back to confirm the write took.
        read_min = mb.read_uint16(args.id, ADDR_MIN_ANGLE)
        read_max = mb.read_uint16(args.id, ADDR_MAX_ANGLE)
        print(f"Confirmed on servo: min={read_min}, max={read_max}")

        if read_min != min_pos or read_max != max_pos:
            print("  ! readback does not match what was written -- retry or check the servo")
            sys.exit(1)

        print("Done.")


if __name__ == "__main__":
    main()
