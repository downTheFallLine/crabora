"""
walk_n_steps.py -- tripod gait walker for CRABORA

Front is defined as the midpoint between legs 1 and 6 (not on top of leg 1).
Physical layout, counter-clockwise from top: 1, 4, 2, 5, 3, 6.

Run:
    python walk_n_steps.py [n_steps]

n_steps counts HALF-CYCLES (one tripod swap each). Two steps = one full
stride where both tripods have cycled through swing+stance once.
"""

import math
import sys

from crabora_bus import (
    MultiBus,
    COXA, FEMUR, TIBIA,
    TRIPOD_A, TRIPOD_B,
    LEG_LAYOUT_CW,
    make_id,
    deg_to_pos, rpm_to_pos_speed,
)

# =============================================================================
# Gait tuning -- these are starting guesses, not calibrated values.
# Tune stride/lift small at first and work up once you see it walk cleanly.
# =============================================================================
STRIDE_DEG    = 20.0   # max coxa swing half-angle (mid legs get the full amount)
LIFT_DEG      = 15.0   # femur/tibia lift during swing, degrees from neutral
SWING_RPM     = 20.0   # speed for the airborne (swing) phase
STANCE_RPM    = 12.0   # speed for the planted (push) phase
PHASE_TIMEOUT = 2.0    # seconds to wait for a phase before giving up

NEUTRAL_FEMUR_DEG = 0.0   # standing femur angle -- calibrate for your build
NEUTRAL_TIBIA_DEG = 0.0   # standing tibia angle -- calibrate for your build

# Per-leg sign flips for hardware mirroring. Start all at +1 and run one
# leg at a time; if a leg swings backward instead of forward, flip its sign
# here rather than changing the gait math.
COXA_SIGN  = {leg: 1 for leg in range(1, 7)}
FEMUR_SIGN = {leg: 1 for leg in range(1, 7)}
TIBIA_SIGN = {leg: 1 for leg in range(1, 7)}

ALL_LEGS = list(range(1, 7))


# =============================================================================
# Front-between-1-and-6 bearing
# =============================================================================
def leg_bearing_deg(leg):
    """Bearing clockwise from front, with front bisecting legs 1 and 6.

    LEG_LAYOUT_CW = [1, 6, 3, 5, 2, 4] already matches the physical order
    you described (its reverse is the CCW order 1,4,2,5,3,6). We only need
    to shift the reference angle by 30 deg so 0 sits between 1 and 6
    instead of on top of leg 1.
    """
    raw = LEG_LAYOUT_CW.index(leg) * 60.0
    shifted = raw - 30.0
    return ((shifted + 180) % 360) - 180  # normalize to (-180, 180]


def coxa_swing_delta(leg, stride_deg):
    """Coxa angle change to move this leg's foot in the global forward
    direction by an amount proportional to stride_deg.

    Mid legs (bearing near +/-90) get full amplitude: their coxa axis is
    perpendicular to travel, so coxa rotation directly drives fore-aft
    foot motion. Front/rear legs (bearing near +/-30, +/-150) get reduced
    amplitude, since their coxa axis is closer to the travel direction
    itself -- pure coxa yaw contributes less there. This is an accepted
    simplification for a 3-DOF-per-leg gait; it may leave a slight
    per-leg stride mismatch that shows up as gentle yaw drift, correctable
    by tuning STRIDE_DEG or adding a small steering trim later.
    """
    theta = math.radians(leg_bearing_deg(leg))
    return stride_deg * math.sin(theta) * COXA_SIGN[leg]


def leg_joint_ids(leg):
    return make_id(leg, COXA), make_id(leg, FEMUR), make_id(leg, TIBIA)


def all_joint_ids(legs):
    return [make_id(l, j) for l in legs for j in (COXA, FEMUR, TIBIA)]


# =============================================================================
# Pose builders -- each returns {servo_id: (position, pos_speed_raw)}
# =============================================================================
def neutral_targets(legs, rpm=STANCE_RPM):
    speed = rpm_to_pos_speed(rpm)
    targets = {}
    for leg in legs:
        cid, fid, tid = leg_joint_ids(leg)
        targets[cid] = (deg_to_pos(0.0), speed)
        targets[fid] = (deg_to_pos(NEUTRAL_FEMUR_DEG), speed)
        targets[tid] = (deg_to_pos(NEUTRAL_TIBIA_DEG), speed)
    return targets


def swing_lift_targets(legs, stride_deg, lift_deg, rpm=SWING_RPM):
    """Phase 1 for a swinging tripod: lift the foot and sweep coxa forward."""
    speed = rpm_to_pos_speed(rpm)
    targets = {}
    for leg in legs:
        cid, fid, tid = leg_joint_ids(leg)
        targets[cid] = (deg_to_pos(coxa_swing_delta(leg, +stride_deg)), speed)
        targets[fid] = (deg_to_pos(NEUTRAL_FEMUR_DEG + lift_deg * FEMUR_SIGN[leg]), speed)
        targets[tid] = (deg_to_pos(NEUTRAL_TIBIA_DEG + lift_deg * TIBIA_SIGN[leg]), speed)
    return targets


def swing_plant_targets(legs, stride_deg, rpm=SWING_RPM):
    """Phase 2 for the same tripod: lower back to the ground, coxa held forward."""
    speed = rpm_to_pos_speed(rpm)
    targets = {}
    for leg in legs:
        cid, fid, tid = leg_joint_ids(leg)
        targets[cid] = (deg_to_pos(coxa_swing_delta(leg, +stride_deg)), speed)
        targets[fid] = (deg_to_pos(NEUTRAL_FEMUR_DEG), speed)
        targets[tid] = (deg_to_pos(NEUTRAL_TIBIA_DEG), speed)
    return targets


def stance_push_targets(legs, stride_deg, rpm=STANCE_RPM):
    """Stance tripod: feet stay planted, coxa sweeps backward -- this is
    what actually drives the body forward."""
    speed = rpm_to_pos_speed(rpm)
    targets = {}
    for leg in legs:
        cid, fid, tid = leg_joint_ids(leg)
        targets[cid] = (deg_to_pos(coxa_swing_delta(leg, -stride_deg)), speed)
        targets[fid] = (deg_to_pos(NEUTRAL_FEMUR_DEG), speed)
        targets[tid] = (deg_to_pos(NEUTRAL_TIBIA_DEG), speed)
    return targets


# =============================================================================
# Gait loop
# =============================================================================
def walk(bus, n_steps, stride_deg=STRIDE_DEG, lift_deg=LIFT_DEG):
    """Walk n_steps half-cycles (one tripod swap per step)."""
    print("Standing to neutral...")
    bus.sync_goal_move(neutral_targets(ALL_LEGS))
    bus.sync_wait_until_stopped(all_joint_ids(ALL_LEGS), PHASE_TIMEOUT)

    swing, stance = TRIPOD_A, TRIPOD_B

    for step in range(n_steps):
        print(f"Step {step + 1}/{n_steps}: swinging {swing}, planting {stance}")

        # Phase 1: swing tripod lifts + sweeps forward, stance tripod pushes back
        targets = {}
        targets.update(swing_lift_targets(swing, stride_deg, lift_deg))
        targets.update(stance_push_targets(stance, stride_deg))
        bus.sync_goal_move(targets)
        if not bus.sync_wait_until_stopped(all_joint_ids(swing + stance), PHASE_TIMEOUT):
            print("  ! phase 1 timed out -- check for a stalled/overloaded servo")

        # Phase 2: swing tripod lowers back to the ground, coxa held forward
        bus.sync_goal_move(swing_plant_targets(swing, stride_deg))
        if not bus.sync_wait_until_stopped(all_joint_ids(swing), PHASE_TIMEOUT):
            print("  ! phase 2 (plant) timed out")

        swing, stance = stance, swing  # swap roles for the next half-cycle

    print("Returning to neutral...")
    bus.sync_goal_move(neutral_targets(ALL_LEGS))
    bus.sync_wait_until_stopped(all_joint_ids(ALL_LEGS), PHASE_TIMEOUT)


def main():
    n_steps = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    with MultiBus() as mb:
        ids = all_joint_ids(ALL_LEGS)
        mb.sync_enable_torque(ids, on=True)
        try:
            walk(mb, n_steps)
        finally:
            mb.sync_enable_torque(ids, on=False)


if __name__ == "__main__":
    main()
