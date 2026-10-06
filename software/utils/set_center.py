#!/usr/bin/env python3
"""
Set a servo's center point, then its left/right angle limits in degrees.

  1. Torque is released -- the joint goes limp.
  2. You move the joint by hand to the pose you want as CENTER and press
     Enter.  The servo records that pose as position 2048 (Feetech STS
     one-key middle calibration: writing 128 to the torque-enable
     register stores a position offset in EEPROM).
  3. The min/max angle limits are written relative to the new center:
         left  (min limit) = 2048 - left_deg  * 4096/360
         right (max limit) = 2048 + right_deg * 4096/360
     Left = min / right = max, the same convention swing_arm.py prints.

The limits are enforced in servo firmware, so every later goal position
is clamped to them -- a buggy gait command can't drive the joint into
the frame.  The center offset and limits live in EEPROM and survive a
power cycle; the writes are wrapped in an EEPROM unlock / re-lock.

Usage:
    python3 utils/set_center.py 41 45          # center, then ±45 deg
    python3 utils/set_center.py 41 60 30       # center, then 60 left / 30 right
    python3 utils/set_center.py 41 45 --no-center   # limits only, keep current center
    python3 utils/set_center.py 41 45 --ports /dev/ttyACM0

Torque is left OFF when done.  Check the result with swing_arm.py.

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys
import time

from scservo_sdk import PacketHandler

from swing_arm import (
    ADDR_MAX_ANGLE,
    ADDR_MIN_ANGLE,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    BAUDRATE,
    CENTER_POSITION,
    DEFAULT_PORTS,
    ServoError,
    find_servo,
    read2,
    to_degrees,
    write1,
    write2,
)

ADDR_LOCK         = 55    # 1 byte -- EEPROM write lock (0 unlocked, 1 locked)
CALIBRATE_MIDDLE  = 128   # written to torque-enable: "current pose = 2048"
CENTER_TOLERANCE  = 20    # counts (~1.8 deg) -- slack for the joint shifting
MAX_POSITION      = 4095


def to_counts(deg):
    return round(deg * 4096 / 360.0)


def write_eeprom(port, packet, sid, writes):
    """Unlock EEPROM, run each (fn, addr, val, what) write, re-lock."""
    write1(port, packet, sid, ADDR_LOCK, 0, "unlock EEPROM")
    try:
        for fn, addr, val, what in writes:
            fn(port, packet, sid, addr, val, what)
    finally:
        try:
            write1(port, packet, sid, ADDR_LOCK, 1, "re-lock EEPROM")
        except ServoError as exc:
            print(f"  ⚠ could not re-lock EEPROM: {exc}")


def set_center(port, packet, sid):
    print("\nReleasing torque -- the joint is now limp.")
    write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
    print("  -> Move the joint by hand to the pose you want as CENTER.")
    print("  -> Hold it steady...")
    input("  -> ...and press Enter to capture it. ")

    held = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
    print(f"\nCapturing raw position {held} ({to_degrees(held):+.1f} deg) as center {CENTER_POSITION}...")
    write_eeprom(port, packet, sid, [
        (write1, ADDR_TORQUE_ENABLE, CALIBRATE_MIDDLE, "calibrate middle"),
    ])
    time.sleep(0.2)  # let the correction settle
    write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, "release torque")

    after = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
    if abs(after - CENTER_POSITION) > CENTER_TOLERANCE:
        raise ServoError(f"position reads {after} after calibration, expected ~{CENTER_POSITION} "
                         f"-- the joint probably moved; hold it steadier and run again")
    print(f"✓ Center set: this pose now reads {after} (trimmed {held - CENTER_POSITION:+d} counts).")


def set_limits(port, packet, sid, lo, hi):
    print(f"\nWriting limits: left (min) {lo}, right (max) {hi}...")
    write_eeprom(port, packet, sid, [
        (write2, ADDR_MIN_ANGLE, lo, "write min limit"),
        (write2, ADDR_MAX_ANGLE, hi, "write max limit"),
    ])
    got_lo = read2(port, packet, sid, ADDR_MIN_ANGLE, "read min limit")
    got_hi = read2(port, packet, sid, ADDR_MAX_ANGLE, "read max limit")
    if (got_lo, got_hi) != (lo, hi):
        raise ServoError(f"readback mismatch: got {got_lo}..{got_hi}, expected {lo}..{hi}")
    print(f"✓ Limits set: left {got_lo} ({to_degrees(got_lo):+.1f} deg), "
          f"right {got_hi} ({to_degrees(got_hi):+.1f} deg).")


def main():
    parser = argparse.ArgumentParser(
        description="Set a servo's center point, then its left/right limits in degrees.")
    parser.add_argument("servo_id", type=int, help="bus ID of the servo, e.g. 41")
    parser.add_argument("left", type=float, help="degrees of travel left of center (min limit)")
    parser.add_argument("right", type=float, nargs="?",
                        help="degrees of travel right of center (max limit); defaults to LEFT")
    parser.add_argument("--no-center", action="store_true",
                        help="skip the center calibration; only write the limits")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    right = args.left if args.right is None else args.right
    if not 0 <= args.servo_id <= 253:
        parser.error("servo ID must be 0-253")
    for name, deg in (("left", args.left), ("right", right)):
        if not 0 < deg < 180:
            parser.error(f"{name} must be between 0 and 180 degrees")

    lo = CENTER_POSITION - to_counts(args.left)
    hi = CENTER_POSITION + to_counts(right)
    if lo < 0 or hi > MAX_POSITION:
        parser.error(f"limits {lo}..{hi} fall outside 0..{MAX_POSITION}")

    print(f"=== Servo {args.servo_id}: center + limits {args.left:g}° left / {right:g}° right ===")
    print(f"Looking for servo {args.servo_id} at {BAUDRATE} baud:")
    packet = PacketHandler(0)  # STS servos use protocol_end=0
    device, port = find_servo(args.ports, packet, args.servo_id)
    if port is None:
        print(f"\n✗ Servo {args.servo_id} not found on any port -- check wiring, power and ID.")
        return 1

    try:
        if not args.no_center:
            set_center(port, packet, args.servo_id)
        set_limits(port, packet, args.servo_id, lo, hi)
        print(f"\n✓ Done. Torque is OFF. Check it with:  python3 utils/swing_arm.py {args.servo_id // 10}")
        return 0
    except KeyboardInterrupt:
        print("\n\nCancelled.")
        return 130
    except ServoError as exc:
        print(f"\n✗ {exc}")
        return 1
    finally:
        port.closePort()


if __name__ == "__main__":
    sys.exit(main())
