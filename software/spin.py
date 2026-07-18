"""
CRABORA servo spin test
=======================

Puts a single Feetech STS servo into constant-speed ("wheel") mode and
spins it continuously at a requested RPM. Useful for bench-testing a
servo, checking direction, and confirming the bus is healthy under load.

Two arguments:
  servo_id   the bus ID of the servo to spin (0-253)
  rpm        target speed in revolutions per minute; a negative value
             reverses direction (e.g. -15 spins the other way)

How "wheel mode" works on the STS3215:
  The servo has an Operation Mode register (address 33):
      0 = position servo mode (the default -- goes to a goal angle)
      1 = constant-speed / wheel mode (spins at a goal speed)
  Register 33 lives in the write-protected EEPROM area, so it is
  unlocked, written, then re-locked -- the same dance set_id.py does for
  the ID. Once in wheel mode, the servo spins at whatever signed value
  sits in the Goal Speed register (address 46), with direction encoded
  in the sign bit.

Speed units (IMPORTANT -- verify empirically):
  Feetech's goal-speed value is in steps per second, and the STS3215 has
  4096 steps per revolution. So:
      register value = rpm * 4096 / 60
  This script prints the servo's *present* speed once per second so you
  can sanity-check it. If the real RPM (timed with a stopwatch) doesn't
  match what you asked for, adjust STEPS_PER_REV in crabora_bus.py.

The servo keeps spinning until you press Ctrl+C, at which point the
script commands speed 0 and switches torque off, so the servo coasts to
a safe stop. It is left in wheel mode -- see the note printed at exit.

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. The servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  python spin.py 1 20      # spin servo 1 forward at 20 rpm
  python spin.py 3 -12.5   # spin servo 3 in reverse at 12.5 rpm
"""

import argparse
import sys
import time

from crabora_bus import (
    Bus,
    ADDR_MODE, ADDR_GOAL_SPEED, ADDR_PRESENT_POSITION, ADDR_PRESENT_SPEED,
    MODE_POSITION, MODE_WHEEL,
    MAX_VALID_ID, TYPICAL_MAX_RPM,
    describe_id, find_feetech_ports, find_servo_port,
    rpm_to_raw, raw_to_rpm,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Spin a single Feetech STS servo in constant-speed mode.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "servo_id",
        type=int,
        help=f"bus ID of the servo to spin (0-{MAX_VALID_ID})",
    )
    parser.add_argument(
        "rpm",
        type=float,
        help="target speed in RPM; negative reverses direction",
    )
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    servo_id = args.servo_id
    rpm = args.rpm

    # --- validate --------------------------------------------------------
    if not 0 <= servo_id <= MAX_VALID_ID:
        print(f"✗ Servo ID {servo_id} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
        sys.exit(1)

    raw_speed = rpm_to_raw(rpm)
    if abs(rpm) > TYPICAL_MAX_RPM:
        print(
            f"⚠  {rpm:+.1f} rpm is above the STS3215's typical max "
            f"(~{TYPICAL_MAX_RPM} rpm). The servo will just run flat out."
        )

    print("=" * 60)
    print("CRABORA servo spin test")
    print("=" * 60)
    print(f"  servo ID    : {servo_id} ({describe_id(servo_id)})")
    print(f"  target speed: {rpm:+.1f} rpm  (speed register value 0x{raw_speed:04x})")
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
            # --- switch the servo into wheel mode ------------------------
            # The mode register is in the locked EEPROM area; Goal Speed
            # and Torque Enable are SRAM and need no unlock.
            print("\nSwitching to constant-speed (wheel) mode...")
            with bus.eeprom_unlocked(servo_id):
                bus.write_uint8(servo_id, ADDR_MODE, MODE_WHEEL)

            print("Enabling torque...")
            bus.enable_torque(servo_id, on=True)

            print(f"Commanding {rpm:+.1f} rpm...")
            bus.write_uint16(servo_id, ADDR_GOAL_SPEED, raw_speed)

            # --- spin until Ctrl+C, reporting actual speed ---------------
            print("\nSpinning. Press Ctrl+C to stop.\n")
            while True:
                time.sleep(1.0)
                try:
                    present_speed = bus.read_uint16(servo_id, ADDR_PRESENT_SPEED)
                    position = bus.read_uint16(servo_id, ADDR_PRESENT_POSITION)
                except IOError as e:
                    print(f"  (read glitch, retrying: {e})")
                    continue
                print(
                    f"  present speed: {raw_to_rpm(present_speed):+6.1f} rpm"
                    f"    position: {position}"
                )

        except KeyboardInterrupt:
            print("\n\nStopping...")
        finally:
            # Bring the servo to a safe stop no matter how we got here.
            try:
                bus.write_uint16(servo_id, ADDR_GOAL_SPEED, 0)
                time.sleep(0.3)
                bus.enable_torque(servo_id, on=False)
            except IOError:
                pass  # bus already gone -- nothing more we can do

    print("✓ Servo stopped (speed 0, torque off).")
    print(
        f"  Note: servo {servo_id} is still in wheel mode. To use position "
        f"control\n  again (e.g. pol.py), run: python wheel_off.py {servo_id}"
    )


if __name__ == "__main__":
    main()
