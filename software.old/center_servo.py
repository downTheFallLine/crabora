"""
center_servo.py -- move one servo to CENTER_POSITION (2048)
=============================================================
Enables torque, commands a speed-controlled move to center, and waits
for arrival.

Usage:
    python center_servo.py <servo_id> [rpm] [--torque-off|--torque-on]

    python center_servo.py 23                  # leg 2 tibia, default 10 rpm, torque left ON
    python center_servo.py 11 5                # leg 1 coxa, slower 5 rpm
    python center_servo.py 23 10 --torque-off  # center then go limp
    python center_servo.py 23 --torque-off     # rpm still optional even with the flag
"""

import sys

from crabora_bus import (
    MultiBus,
    CENTER_POSITION,
    describe_id,
    rpm_to_pos_speed,
)

DEFAULT_RPM = 10


def main():
    args = sys.argv[1:]

    end_torque_on = True
    if "--torque-off" in args:
        end_torque_on = False
        args.remove("--torque-off")
    if "--torque-on" in args:
        end_torque_on = True
        args.remove("--torque-on")

    if len(args) < 1:
        print(f"Usage: python {sys.argv[0]} <servo_id> [rpm] [--torque-off|--torque-on]")
        sys.exit(1)

    servo_id = int(args[0])
    rpm = float(args[1]) if len(args) > 1 else DEFAULT_RPM

    with MultiBus() as mb:
        if servo_id not in mb.live_ids:
            print(f"Servo {servo_id} ({describe_id(servo_id)}) not found. "
                  f"Live IDs: {mb.live_ids}")
            sys.exit(1)

        print(f"Centering {servo_id} ({describe_id(servo_id)}) "
              f"at {rpm} rpm -> position {CENTER_POSITION}")

        mb.enable_torque(servo_id, True)
        speed = rpm_to_pos_speed(rpm)
        mb.write_goal_move(servo_id, CENTER_POSITION, speed)

        arrived = mb.wait_until_stopped(servo_id, timeout=5.0)
        if arrived:
            print("  done.")
        else:
            print("  timed out waiting for arrival (check for a stall/obstruction).")

        if not end_torque_on:
            mb.enable_torque(servo_id, False)
            print("  torque released (limp).")
        else:
            print("  torque left on.")


if __name__ == "__main__":
    main()
