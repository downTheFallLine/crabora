"""
CRABORA: set servo angle limits
===============================

Sets the min/max angle limits on a single Feetech STS servo, so the
servo itself clamps every goal position in firmware. This stops a buggy
gait command from swinging a leg into its neighbour or the chassis --
the guard holds even when the controlling code is wrong.

The limit is symmetric around the 2048 center, so run set_middle.py
first: once "straight out" has been calibrated to 2048, a symmetric
swing lines up with the leg's true neutral.

Arguments:
  servo_id   the bus ID of the servo (0-253)
  angle      swing in DEGREES to each side of center (e.g. 45 allows the
             leg to travel 45 deg either way -- a 90 deg total sweep)

How it maps to the servo:
  Position is 0..4095 over a full 360 deg turn, so 4096 counts = 360 deg
  and the center is 2048. For a given angle:
      half_counts = angle / 360 * 4096
      min limit   = 2048 - half_counts   (written to register 9)
      max limit   = 2048 + half_counts   (written to register 11)
  Registers 9 and 11 are in the write-protected EEPROM area, so the
  writes are wrapped in the unlock/re-lock dance.

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. The servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  python set_limits.py 11 45     # leg 1 coxa: +/-45 deg around center
"""

import argparse
import sys

from crabora_bus import (
    Bus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE,
    COUNTS_PER_REV, DEGREES_PER_REV, CENTER_POSITION, MAX_POSITION,
    MAX_VALID_ID,
    describe_id, find_feetech_ports, find_servo_port,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Set the symmetric angle limits of a Feetech STS servo.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "servo_id",
        type=int,
        help=f"bus ID of the servo (0-{MAX_VALID_ID})",
    )
    parser.add_argument(
        "angle",
        type=float,
        help="swing in degrees to each side of center (e.g. 45)",
    )
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    servo_id = args.servo_id
    angle = args.angle

    # --- validate --------------------------------------------------------
    if not 0 <= servo_id <= MAX_VALID_ID:
        print(f"✗ Servo ID {servo_id} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
        sys.exit(1)
    if angle <= 0:
        print("✗ Angle must be positive -- it is the swing in degrees each")
        print("  side of center. Use a small value like 30 or 45.")
        sys.exit(1)

    # --- convert the angle to min/max position counts --------------------
    half_counts = round(angle / DEGREES_PER_REV * COUNTS_PER_REV)
    raw_min = CENTER_POSITION - half_counts
    raw_max = CENTER_POSITION + half_counts
    min_pos = max(0, raw_min)
    max_pos = min(MAX_POSITION, raw_max)
    clamped = min_pos != raw_min or max_pos != raw_max

    print("=" * 60)
    print("CRABORA: set servo angle limits")
    print("=" * 60)
    print(f"  servo ID : {servo_id} ({describe_id(servo_id)})")
    print(f"  swing    : ±{angle:.1f}° off center  (±{half_counts} counts)")
    print(f"  limits   : {min_pos} .. {max_pos}  (center {CENTER_POSITION})")
    if clamped:
        print(
            f"⚠  ±{angle:.1f}° is wider than the servo's 0-{MAX_POSITION} range; "
            f"clamped to {min_pos}..{max_pos}."
        )
    print()

    # Find the servo on whatever URT(s) are actually plugged in.
    ports = find_feetech_ports()
    if not ports:
        print("✗ No FE-URT-2 found. Is it plugged in?")
        sys.exit(1)
    print(f"Looking for servo ID {servo_id} on {len(ports)} URT(s)...")
    port = find_servo_port(servo_id)
    if port is None:
        print(f"✗ No response at ID {servo_id} on any URT: {ports}")
        print("  Check the 7.4V bus power and the servo cable.")
        sys.exit(1)

    with Bus(port=port) as bus:
        print(f"✓ Servo {servo_id} answered on {bus.port_path}.")

        # --- write the limits, inside the EEPROM unlock dance ------------
        print("\nWriting angle limits...")
        with bus.eeprom_unlocked(servo_id):
            bus.write_uint16(servo_id, ADDR_MIN_ANGLE, min_pos)
            bus.write_uint16(servo_id, ADDR_MAX_ANGLE, max_pos)

        # --- verify ------------------------------------------------------
        print("Verifying...")
        got_min = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
        got_max = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
        if got_min != min_pos or got_max != max_pos:
            print(
                f"✗ Readback mismatch: got {got_min}..{got_max}, "
                f"expected {min_pos}..{max_pos}."
            )
            sys.exit(1)

    print(
        f"✓ Servo {servo_id} angle limits set to {got_min}..{got_max} "
        f"(±{angle:.1f}° around center)."
    )


if __name__ == "__main__":
    main()
