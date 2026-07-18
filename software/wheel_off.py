"""
CRABORA: turn wheel mode off
============================

Returns a single Feetech STS servo to position (servo) mode -- the
factory default -- undoing what spin.py does. Use this when you are
finished spinning a servo and want position control back (e.g. pol.py).

One argument:
  servo_id   the bus ID of the servo (0-253)

What it does:
  1. Pings the servo and reads its current Operation Mode (register 33).
  2. If it is already in position mode, reports that and exits.
  3. Otherwise: commands a stop (Goal Speed 0), disables torque so the
     mode switch cannot cause a lurch, then unlocks EEPROM, writes mode 0
     (position), and re-locks.
  4. Verifies register 33 reads back as 0.

The servo is left limp (torque off) in position mode. Re-enable torque
and command a position with pol.py when you want to drive it again.

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. The servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  python wheel_off.py 1
"""

import argparse
import sys
import time

from crabora_bus import (
    Bus,
    ADDR_MODE, ADDR_TORQUE_ENABLE, ADDR_GOAL_SPEED,
    MODE_POSITION, MODE_NAMES,
    MAX_VALID_ID,
    describe_id, find_feetech_ports, find_servo_port,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Return a Feetech STS servo to position mode (turn wheel mode off).",
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
    print("CRABORA: turn wheel mode off")
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

        # --- check the current mode --------------------------------------
        mode = bus.read_uint8(servo_id, ADDR_MODE)
        mode_name = MODE_NAMES.get(mode, f"unknown ({mode})")
        print(f"Current mode: {mode} ({mode_name})")

        if mode == MODE_POSITION:
            print("\n✓ Servo is already in position mode -- nothing to do.")
            return

        # --- stop the servo, then make the switch safe -------------------
        # Command a stop while still in wheel mode so it decelerates
        # smoothly, then cut torque so flipping the mode register cannot
        # cause a lurch.
        print("\nCommanding stop (Goal Speed 0)...")
        bus.write_uint16(servo_id, ADDR_GOAL_SPEED, 0)
        time.sleep(0.5)  # let it decelerate while still in wheel mode

        print("Disabling torque...")
        bus.enable_torque(servo_id, on=False)

        # --- switch the EEPROM mode register back to position ------------
        print(f"Switching to position mode (reg {ADDR_MODE} <- {MODE_POSITION})...")
        with bus.eeprom_unlocked(servo_id):
            bus.write_uint8(servo_id, ADDR_MODE, MODE_POSITION)

        # --- verify ------------------------------------------------------
        print("\nVerifying...")
        confirmed = bus.read_uint8(servo_id, ADDR_MODE)
        if confirmed != MODE_POSITION:
            print(f"✗ Register {ADDR_MODE} reads {confirmed}, expected {MODE_POSITION}.")
            sys.exit(1)
        print(f"✓ Servo {servo_id} is back in position mode (torque off).")

    print("\n✓ Done. Use pol.py to enable torque and command positions.")


if __name__ == "__main__":
    main()
