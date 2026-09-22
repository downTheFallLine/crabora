#!/usr/bin/env python3
"""
Exercise a single CRABORA servo through:
    1. center
    2. extreme left
    3. extreme right
    4. back to center

Uses crabora_bus.MultiBus, which auto-discovers every FE-URT-2 on the
machine and routes the servo ID to whichever URT it actually answers on.
No need to know or pass a port -- handy on the bench when legs get moved
between URTs.

Usage:
    python3 exercise_servo.py --id 11 --speed 15
    python3 exercise_servo.py --id 23 --speed 10 --left-deg -60 --right-deg 60
"""

import argparse
import sys

from crabora_bus import (
    MultiBus,
    CENTER_POSITION,
    MIN_POSITION,
    MAX_POSITION,
    deg_to_pos,
    describe_id,
    rpm_to_pos_speed,
)

DWELL_TIMEOUT = 3.0  # seconds to wait for arrival at each stop


def move_and_wait(mb, servo_id, position, pos_speed_raw, label):
    print(f"  -> {label} (position {position})...")
    mb.write_goal_move(servo_id, position, pos_speed_raw)
    arrived = mb.wait_until_stopped(servo_id, timeout=DWELL_TIMEOUT)
    if not arrived:
        print(f"     ! did not report stopped within {DWELL_TIMEOUT}s "
              f"(check for stall / overload)")


def main():
    parser = argparse.ArgumentParser(description="Exercise a single CRABORA servo")
    parser.add_argument("--id", type=int, required=True, help="Servo ID (e.g. 11 = leg 1 coxa)")
    parser.add_argument("--speed", type=float, required=True, help="Move speed in RPM")
    parser.add_argument("--left-deg", type=float, default=None,
                         help="Extreme-left angle in degrees from center (default: full range / MIN_POSITION)")
    parser.add_argument("--right-deg", type=float, default=None,
                         help="Extreme-right angle in degrees from center (default: full range / MAX_POSITION)")
    parser.add_argument("--ports", nargs="*", default=None,
                         help="Restrict discovery to specific URT ports (default: auto-discover all)")
    parser.add_argument("--iterations", type=int, default=1,
                         help="Number of times to repeat the center/left/right/center cycle (default: 1)")
    args = parser.parse_args()

    if args.iterations < 1:
        print("--iterations must be at least 1")
        sys.exit(1)

    left_pos = deg_to_pos(args.left_deg) if args.left_deg is not None else MIN_POSITION
    right_pos = deg_to_pos(args.right_deg) if args.right_deg is not None else MAX_POSITION
    speed_raw = rpm_to_pos_speed(args.speed)

    with MultiBus(ports=args.ports) as mb:
        if args.id not in mb.live_ids:
            print(f"Servo {args.id} ({describe_id(args.id)}) was not found during "
                  f"discovery. Live IDs: {mb.live_ids}. Check power/cable.")
            sys.exit(1)

        print(f"Exercising servo {args.id} ({describe_id(args.id)}) at {args.speed} rpm "
              f"for {args.iterations} iteration(s)")
        mb.enable_torque(args.id, True)

        for i in range(1, args.iterations + 1):
            if args.iterations > 1:
                print(f"-- Iteration {i}/{args.iterations} --")
            move_and_wait(mb, args.id, CENTER_POSITION, speed_raw, "CENTER")
            move_and_wait(mb, args.id, left_pos, speed_raw, "EXTREME LEFT")
            move_and_wait(mb, args.id, right_pos, speed_raw, "EXTREME RIGHT")
            move_and_wait(mb, args.id, CENTER_POSITION, speed_raw, "CENTER")

        print("Exercise complete.")


if __name__ == "__main__":
    main()

