"""
stand.py -- quick standing-pose tuner for CRABORA

Holds all six legs at a given femur/tibia angle so you can see the effect
of each change immediately, without running a full gait cycle. Edit the
numbers and rerun -- much faster than tuning height/spread via walk_n_steps.py.

Run:
    python stand.py <femur_deg> <tibia_deg>

Example:
    python stand.py 25 -20
"""

import sys

from crabora_bus import (
    MultiBus,
    COXA, FEMUR, TIBIA,
    make_id,
    deg_to_pos, rpm_to_pos_speed,
)

ALL_LEGS = list(range(1, 7))
STAND_RPM = 10.0  # slow -- this is for careful tuning, not speed


def all_joint_ids(legs):
    return [make_id(l, j) for l in legs for j in (COXA, FEMUR, TIBIA)]


def stand_targets(femur_deg, tibia_deg, rpm=STAND_RPM):
    speed = rpm_to_pos_speed(rpm)
    targets = {}
    for leg in ALL_LEGS:
        targets[make_id(leg, COXA)] = (deg_to_pos(0.0), speed)
        targets[make_id(leg, FEMUR)] = (deg_to_pos(femur_deg), speed)
        targets[make_id(leg, TIBIA)] = (deg_to_pos(tibia_deg), speed)
    return targets


def main():
    if len(sys.argv) != 3:
        print("Usage: python stand.py <femur_deg> <tibia_deg>")
        sys.exit(1)

    femur_deg = float(sys.argv[1])
    tibia_deg = float(sys.argv[2])

    with MultiBus() as mb:
        ids = all_joint_ids(ALL_LEGS)
        mb.sync_enable_torque(ids, on=True)
        print(f"Standing at femur={femur_deg} deg, tibia={tibia_deg} deg...")
        mb.sync_goal_move(stand_targets(femur_deg, tibia_deg))
        mb.sync_wait_until_stopped(ids, timeout=2.0)
        print("Holding pose. Ctrl+C to release torque and exit.")
        try:
            while True:
                pass
        except KeyboardInterrupt:
            pass
        finally:
            mb.sync_enable_torque(ids, on=False)


if __name__ == "__main__":
    main()
