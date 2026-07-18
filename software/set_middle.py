"""
CRABORA: set servo center ("define middle")
===========================================

Calibrates a single Feetech STS servo so that its *current physical
pose* becomes the center of its range -- position 2048. Use this to
teach a leg joint where "neutral" is: e.g. the coxa with the leg
pointing straight out, after the servo horn has been mounted.

Why this is needed:
  The servo's output spline is discrete, so you cannot mount a leg
  perfectly straight at exactly 2048. This script trims out the leftover
  error -- you position the leg by hand and the servo records that pose
  as 2048 from then on. After this, symmetric angle limits around 2048
  line up with the leg's true neutral.

How it works:
  Writing the special value 128 to the Torque Enable register (40) is a
  documented Feetech STS one-key calibration: the servo computes an
  internal position-correction offset (stored in register 31) so the
  current pose reads as 2048. This script wraps that write in the EEPROM
  unlock/re-lock dance, since CRABORA's other scripts leave the EEPROM
  locked and the correction is stored there.

One argument:
  servo_id   the bus ID of the servo (0-253)

What happens when you run it:
  1. The script disables torque -- the servo goes limp.
  2. You rotate the leg by hand to the exact pose you want as center.
  3. You press Enter (keep holding the leg steady for a moment).
  4. The servo records that pose as 2048; the script verifies it.

Note: run this BEFORE setting min/max angle limits -- the limits are
measured relative to the 2048 center this establishes.

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. The servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  python set_middle.py 11
"""

import argparse
import sys
import time

from crabora_bus import (
    Bus,
    ADDR_TORQUE_ENABLE, ADDR_PRESENT_POSITION,
    CENTER_POSITION, MAX_VALID_ID,
    describe_id, find_feetech_ports, find_servo_port,
)


# Writing 128 to the Torque Enable register is the Feetech STS "set this
# pose as center 2048" one-key calibration -- not a value crabora_bus
# exposes as a normal API, so it's kept local here with a comment.
CALIBRATE_MIDDLE = 128

# After calibration the held pose should read 2048; allow a little slack
# for encoder noise and the leg shifting slightly during the write.
CENTER_TOLERANCE = 20


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate a Feetech STS servo's current pose as center (2048).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "servo_id",
        type=int,
        help=f"bus ID of the servo (0-{MAX_VALID_ID})",
    )
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    servo_id = args.servo_id

    if not 0 <= servo_id <= MAX_VALID_ID:
        print(f"✗ Servo ID {servo_id} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
        sys.exit(1)

    print("=" * 60)
    print("CRABORA: set servo center")
    print("=" * 60)
    print(f"  servo ID: {servo_id} ({describe_id(servo_id)})")
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

        try:
            # --- go limp so the leg can be positioned by hand ------------
            print("\nDisabling torque -- the servo is now limp.")
            bus.enable_torque(servo_id, on=False)

            print()
            print("  -> Rotate the leg by hand to the exact pose you want as")
            print("     CENTER (e.g. coxa: leg pointing straight out).")
            print("  -> Hold it steady...")
            input("  -> ...and press Enter to capture it. ")

            held = bus.read_uint16(servo_id, ADDR_PRESENT_POSITION)
            print(f"\nCapturing this pose (raw position {held}) as center 2048...")

            # --- one-key middle calibration, inside the EEPROM dance -----
            with bus.eeprom_unlocked(servo_id):
                bus.write_uint8(servo_id, ADDR_TORQUE_ENABLE, CALIBRATE_MIDDLE)
            time.sleep(0.2)  # let the correction settle

            # leave torque off in a clean, known state
            bus.enable_torque(servo_id, on=False)

            # --- verify --------------------------------------------------
            after = bus.read_uint16(servo_id, ADDR_PRESENT_POSITION)
            offset = after - CENTER_POSITION
            if abs(offset) <= CENTER_TOLERANCE:
                print(
                    f"✓ Servo {servo_id} now reads {after} at this pose "
                    f"(~{CENTER_POSITION} center; trimmed {held - CENTER_POSITION:+d} counts)."
                )
                print("\n✓ Done. Calibrate angle limits relative to 2048 next.")
            else:
                print(
                    f"⚠ Position reads {after}, expected ~{CENTER_POSITION} "
                    f"(off by {offset:+d})."
                )
                print("  The leg likely moved during calibration -- hold it")
                print("  steadier and run this again.")

        except KeyboardInterrupt:
            print("\n\nCancelled -- no calibration written.")


if __name__ == "__main__":
    main()
