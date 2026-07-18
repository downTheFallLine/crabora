"""
CRABORA: coxa walk
==================

Oscillates one coxa servo back and forth between its configured angle
limits -- the protraction/retraction swing a leg makes during a walking
gait. A single coxa cannot walk the robot on its own (that needs the
femur/tibia lifting the foot too), but this exercises the coxa's part
of the motion and is a good bench test of range, speed, and the limits.

The script reads the servo's Min/Max Angle Limit registers and uses
them as the sweep endpoints, so it always "walks within the limits we
set" with set_limits.py. If no limits are set it refuses to run.

Arguments:
  servo_id   the bus ID of the coxa servo (0-253)
  speed      sweep speed in RPM -- the angular velocity of the swing
             (same unit as spin.py; higher = faster leg swing)

Requirements (the script checks these and tells you if not met):
  - The servo must be in position mode      -> run wheel_off.py
  - Angle limits must be set (not 0..4095)  -> run set_limits.py
  - The center should be calibrated         -> run set_middle.py

What it does each cycle:
  Commands the servo to the max limit, waits for it to arrive (polling
  the Moving register), then to the min limit, and repeats. Press Ctrl+C
  to stop -- the leg returns to center and torque is switched off.

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. The servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  python walk.py 11 15     # walk leg 1's coxa at 15 rpm
"""

import argparse
import sys

from crabora_bus import (
    Bus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE, ADDR_PRESENT_POSITION,
    MODE_POSITION, MODE_NAMES,
    COUNTS_PER_REV, DEGREES_PER_REV, CENTER_POSITION, MAX_POSITION,
    MAX_VALID_ID, TYPICAL_MAX_RPM,
    describe_id,
    find_feetech_ports,
    find_servo_port,
    rpm_to_pos_speed,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Walk a coxa servo back and forth between its angle limits.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "servo_id",
        type=int,
        help=f"bus ID of the coxa servo (0-{MAX_VALID_ID})",
    )
    parser.add_argument(
        "speed",
        type=float,
        help="sweep speed in RPM (the angular velocity of the swing)",
    )
    return parser.parse_args()


def _alternating(a, b):
    """Yield a, b, a, b, ... forever -- the back-and-forth walk targets."""
    while True:
        yield a
        yield b


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    servo_id = args.servo_id
    rpm = args.speed

    # --- validate --------------------------------------------------------
    if not 0 <= servo_id <= MAX_VALID_ID:
        print(f"✗ Servo ID {servo_id} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
        sys.exit(1)
    if rpm <= 0:
        print("✗ Speed must be positive (RPM). Try a modest value like 10 or 15.")
        sys.exit(1)

    raw_speed = rpm_to_pos_speed(rpm)
    if rpm > TYPICAL_MAX_RPM:
        print(
            f"⚠  {rpm:.1f} rpm is above the STS3215's typical max "
            f"(~{TYPICAL_MAX_RPM} rpm); the servo will just run flat out."
        )

    print("=" * 60)
    print("CRABORA: coxa walk")
    print("=" * 60)
    print(f"  servo ID: {servo_id} ({describe_id(servo_id)})")
    print(f"  speed   : {rpm:.1f} rpm")
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

        # --- preconditions: position mode + real angle limits ------------
        mode = bus.read_uint8(servo_id, ADDR_MODE)
        if mode != MODE_POSITION:
            name = MODE_NAMES.get(mode, f"mode {mode}")
            print(f"✗ Servo is in {name} mode; walking needs position mode.")
            print(f"  Run: python wheel_off.py {servo_id}")
            sys.exit(1)

        min_limit = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
        max_limit = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
        if min_limit >= max_limit or (min_limit == 0 and max_limit >= MAX_POSITION):
            print(f"✗ No usable angle limits set (servo reads {min_limit}..{max_limit}).")
            print(f"  Run set_limits.py first so the coxa walks within bounds.")
            sys.exit(1)

        half_deg = (max_limit - min_limit) / 2 / COUNTS_PER_REV * DEGREES_PER_REV
        print(f"Walk range: {min_limit}..{max_limit}  (±{half_deg:.1f}° around center)")

        # Size the per-sweep timeout from the actual speed: a full min..max
        # sweep at raw_speed counts/sec, doubled for safety, plus a margin.
        sweep_timeout = (max_limit - min_limit) / raw_speed * 2.0 + 1.0

        try:
            bus.enable_torque(servo_id, on=True)

            print("\nWalking. Press Ctrl+C to stop.\n")
            swing = 0
            for target in _alternating(max_limit, min_limit):
                bus.write_goal_move(servo_id, target, raw_speed)
                if not bus.wait_until_stopped(servo_id, sweep_timeout):
                    print("  (servo slow to report stopped -- continuing)")
                swing += 1
                pos = bus.read_uint16(servo_id, ADDR_PRESENT_POSITION)
                print(f"  swing {swing:3d}: reached {pos:4d}  (target {target})")

        except KeyboardInterrupt:
            print("\n\nStopping -- returning leg to center...")
        finally:
            try:
                bus.write_goal_move(servo_id, CENTER_POSITION, raw_speed)
                bus.wait_until_stopped(servo_id, sweep_timeout)
                bus.enable_torque(servo_id, on=False)
            except IOError:
                pass  # bus already gone -- nothing more we can do

    print("✓ Servo stopped at center, torque off.")


if __name__ == "__main__":
    main()
