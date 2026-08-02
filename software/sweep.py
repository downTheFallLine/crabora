"""
limit_sweep.py -- read a servo's firmware angle limits, then sweep
center -> max -> min, using MultiBus so any leg/URT combination works.

Usage:
    python limit_sweep.py <servo_id> [rpm]

For the given servo_id:
  1. reads ADDR_MIN_ANGLE / ADDR_MAX_ANGLE (raw position counts -- the
     firmware limits, not the mechanical 0..4095 range)
  2. moves it to CENTER_POSITION
  3. moves it to its MAX firmware limit
  4. moves it to its MIN firmware limit
waiting for wait_until_stopped before advancing to the next move.

Torque is enabled at the start and released at the end so nothing is left
fighting against a stale goal position if you Ctrl-C mid-sweep.
"""

import sys

from crabora_bus import (
    MultiBus,
    CENTER_POSITION,
    ADDR_MIN_ANGLE,
    ADDR_MAX_ANGLE,
    rpm_to_pos_speed,
    describe_id,
)

DEFAULT_RPM = 10        # conservative sweep speed -- raise once this is proven safe
MOVE_TIMEOUT = 5.0      # seconds to wait for a stage to finish before warning


def read_limits(mb, servo_id):
    """Return (min_pos, max_pos) -- raw firmware limits, in counts."""
    lo = mb.read_uint16(servo_id, ADDR_MIN_ANGLE)
    hi = mb.read_uint16(servo_id, ADDR_MAX_ANGLE)
    return lo, hi


def move_to(mb, servo_id, position, speed_raw, label):
    """Command one servo to a position, then wait for it to arrive."""
    print(f"\n-- moving {servo_id} ({describe_id(servo_id)}) to {label} ({position}) --")
    mb.write_goal_move(servo_id, position, speed_raw)
    ok = mb.wait_until_stopped(servo_id, timeout=MOVE_TIMEOUT)
    if not ok:
        print("  ⚠ servo did not report stopped in time")


def sweep(mb, servo_id, limits, speed_raw):
    lo, hi = limits

    print(f"\n-- enabling torque on {servo_id} ({describe_id(servo_id)}) --")
    mb.enable_torque(servo_id, on=True)

    move_to(mb, servo_id, CENTER_POSITION, speed_raw, "CENTER")
    move_to(mb, servo_id, hi, speed_raw, "MAX limit")
    move_to(mb, servo_id, lo, speed_raw, "MIN limit")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <servo_id> [rpm]")
        sys.exit(1)

    servo_id = int(sys.argv[1])
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_RPM
    speed_raw = rpm_to_pos_speed(rpm)

    with MultiBus() as mb:
        if servo_id not in mb.live_ids:
            print(f"Servo {servo_id} not found. Live IDs: {mb.live_ids}")
            return

        limits = read_limits(mb, servo_id)
        lo, hi = limits
        print(f"\n{servo_id} ({describe_id(servo_id)}) firmware limits: min={lo}  max={hi}")

        sweep(mb, servo_id, limits, speed_raw)

        print(f"\n-- disabling torque on {servo_id} --")
        mb.enable_torque(servo_id, on=False)


if __name__ == "__main__":
    main()
