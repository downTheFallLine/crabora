"""
CRABORA: 3-leg wave gait
========================

One leg lifts and steps forward while the other two push the body forward.
Legs take turns in the order 1 -> 2 -> 3 -> 1 -> ...

What happens each step:
  1. LIFT   -- swing leg femur+tibia go to the 'tuck' pose (foot off ground)
  2. STEP   -- swing leg coxa sweeps to FRONT
                stance leg coxas sweep to BACK simultaneously
               (stance legs pushing backward = body advances forward)
  3. PLANT  -- swing leg femur+tibia return to 'stand' pose (foot on ground)
  4. Next leg becomes the swing leg.

FRONT = coxa max_limit
BACK  = coxa min_limit

If the body goes BACKWARD instead of forward, one or more coxas are
mounted with reversed polarity.  Use --flip to invert specific legs:

  python walk_wave.py --flip 1        # reverse leg 1's coxa direction
  python walk_wave.py --flip 1 3      # reverse legs 1 and 3

Before the first step, all coxas are moved to FRONT so there is a full
backward sweep available for each stance phase.

Requirements (all 9 servos):
  - Position mode    -> run wheel_off.py <id>
  - Angle limits set -> run set_limits.py <id>
  - poses.json with 'tuck' and 'stand' -> run teach_pose.py

Hardware note:
  Start with the bot SUSPENDED from the bench hang-hole, feet clear of
  the ground. Confirm the leg motion looks right before setting it down.

Usage:
  python walk_wave.py                        # 3 cycles, 8 rpm swing
  python walk_wave.py --cycles 6             # more steps
  python walk_wave.py --flip 2               # reverse leg 2's coxa
  python walk_wave.py --speed 12 --no-pause  # faster, no prompts
"""

import argparse
import json
import os
import sys

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE, ADDR_PRESENT_POSITION,
    MODE_POSITION, MODE_NAMES,
    MAX_POSITION, TYPICAL_MAX_RPM,
    CENTER_POSITION,
    describe_id, rpm_to_pos_speed,
)

LEG_IDS   = {1: [11, 12, 13], 2: [21, 22, 23], 3: [31, 32, 33]}
POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

COXA  = 0
FEMUR = 1
TIBIA = 2


def parse_args():
    p = argparse.ArgumentParser(
        description="3-leg wave gait: one leg swings while two push the body forward.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--cycles", type=int, default=3,
                   help="number of full 3-leg gait cycles (default: 3)")
    p.add_argument("--speed", type=float, default=8.0,
                   help="coxa sweep speed in RPM (default: 8)")
    p.add_argument("--lift-speed", type=float, default=6.0,
                   help="femur/tibia lift and plant speed in RPM (default: 6)")
    p.add_argument("--flip", type=int, nargs="+", default=[], metavar="LEG",
                   choices=[1, 2, 3],
                   help="reverse FRONT/BACK coxa direction for these legs "
                        "(use if a leg steps backward instead of forward)")
    p.add_argument("--release", action="store_true",
                   help="relax all torque after the gait (default: hold)")
    p.add_argument("--no-pause", action="store_true",
                   help="skip the ready confirmation")
    return p.parse_args()


def load_poses():
    if not os.path.exists(POSES_PATH):
        print(f"✗ No poses.json at {POSES_PATH}.")
        print("  Capture poses first:  python teach_pose.py")
        sys.exit(1)
    with open(POSES_PATH) as f:
        poses = json.load(f)
    for needed in ("tuck", "stand"):
        if needed not in poses:
            print(f"✗ poses.json has no '{needed}' pose. Run: python teach_pose.py")
            sys.exit(1)
    return poses["tuck"], poses["stand"]


def preflight(bus, servo_id):
    label = describe_id(servo_id)
    if not bus.ping(servo_id):
        print(f"✗ No response at ID {servo_id} ({label}). Check power/cable.")
        sys.exit(1)
    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        print(f"✗ {label} (ID {servo_id}) in {MODE_NAMES.get(mode, mode)} mode. "
              f"Run: python wheel_off.py {servo_id}")
        sys.exit(1)
    mn = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
    mx = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
    if mn >= mx or (mn == 0 and mx >= MAX_POSITION):
        print(f"✗ {label} (ID {servo_id}) has no usable angle limits ({mn}..{mx}). "
              f"Run: python set_limits.py {servo_id}")
        sys.exit(1)
    print(f"✓ {label} (ID {servo_id}): limits {mn}..{mx}")
    return mn, mx


def clamp(counts, mn, mx):
    return max(mn, min(mx, counts))


def pose_targets(leg_ids, pose, limits, speed):
    """Build {servo_id: (counts, speed)} for one leg from a pose dict."""
    targets = {}
    for sid in leg_ids:
        counts = int(pose[str(sid)])
        mn, mx = limits[sid]
        targets[sid] = (clamp(counts, mn, mx), speed)
    return targets


def coxa_front_back(leg_num, limits, flipped_legs):
    """Return (front_counts, back_counts) for this leg's coxa.

    FRONT = where the foot lands ahead of the body (coxa max by default).
    BACK  = where the stance leg pushes to (coxa min by default).
    Flipped legs swap these.
    """
    coxa_id = LEG_IDS[leg_num][COXA]
    mn, mx = limits[coxa_id]
    if leg_num in flipped_legs:
        return mn, mx   # flipped: min is forward, max is backward
    return mx, mn       # default: max is forward, min is backward


def main():
    args = parse_args()

    if args.cycles < 1:
        print("✗ --cycles must be at least 1."); sys.exit(1)
    for rpm, name in ((args.speed, "--speed"), (args.lift_speed, "--lift-speed")):
        if rpm <= 0:
            print(f"✗ {name} must be positive."); sys.exit(1)
        if rpm > TYPICAL_MAX_RPM:
            print(f"⚠  {name} {rpm:.1f} rpm exceeds typical STS3215 max "
                  f"(~{TYPICAL_MAX_RPM} rpm).")

    tuck, stand  = load_poses()
    all_ids      = [sid for ids in LEG_IDS.values() for sid in ids]
    swing_speed  = rpm_to_pos_speed(args.speed)
    lift_speed   = rpm_to_pos_speed(args.lift_speed)
    flipped      = set(args.flip)

    print("=" * 62)
    print("CRABORA: 3-leg wave gait")
    print("=" * 62)
    print(f"  cycles    : {args.cycles}  (x3 steps each = {args.cycles * 3} steps)")
    print(f"  swing RPM : {args.speed:.1f}    lift RPM: {args.lift_speed:.1f}")
    if flipped:
        print(f"  flipped   : legs {sorted(flipped)}")
    print()
    print("  Tip: start with the bot SUSPENDED so you can check direction")
    print("  before setting it on the floor.")

    if not args.no_pause:
        try:
            ans = input("\nProceed? [y/N] ")
        except EOFError:
            ans = "n"
        if ans.strip().lower() not in ("y", "yes"):
            print("Aborted."); return

    with MultiBus() as bus:

        # --- preflight ---------------------------------------------------
        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}

        # Timeouts based on actual coxa range and speed
        widest_coxa = max(
            limits[LEG_IDS[n][COXA]][1] - limits[LEG_IDS[n][COXA]][0]
            for n in LEG_IDS
        )
        sweep_timeout = widest_coxa / swing_speed * 2.0 + 1.0
        lift_timeout  = 3.0

        try:
            bus.sync_enable_torque(all_ids, on=True)

            # --- initial position: stand pose, all coxas to FRONT -------
            print("\nMoving to start position (stand pose, coxas forward)...")
            start_targets = {}
            for leg_num, leg_ids in LEG_IDS.items():
                # femur + tibia from stand pose
                start_targets.update(
                    pose_targets([leg_ids[FEMUR], leg_ids[TIBIA]], stand, limits, lift_speed)
                )
                # coxa to FRONT
                front, _ = coxa_front_back(leg_num, limits, flipped)
                coxa_id = leg_ids[COXA]
                start_targets[coxa_id] = (front, swing_speed)
            bus.sync_goal_move(start_targets)
            bus.sync_wait_until_stopped(all_ids, sweep_timeout + lift_timeout)

            # --- gait loop -----------------------------------------------
            print(f"\nWalking. {args.cycles} cycle(s), "
                  f"leg order 1->2->3. Press Ctrl+C to stop.\n")

            step_count = 0
            for cycle in range(args.cycles):
                for swing_leg_num in [1, 2, 3]:
                    step_count += 1
                    swing_ids    = LEG_IDS[swing_leg_num]
                    stance_nums  = [n for n in LEG_IDS if n != swing_leg_num]
                    swing_front, swing_back = coxa_front_back(
                        swing_leg_num, limits, flipped)

                    print(f"  step {step_count:2d}  (cycle {cycle+1}, "
                          f"swing leg {swing_leg_num})")

                    # LIFT: raise swing leg foot
                    lift_targets = pose_targets(
                        [swing_ids[FEMUR], swing_ids[TIBIA]], tuck, limits, lift_speed)
                    bus.sync_goal_move(lift_targets)
                    bus.sync_wait_until_stopped(
                        [swing_ids[FEMUR], swing_ids[TIBIA]], lift_timeout)

                    # STEP: swing coxa FRONT, stance coxas BACK simultaneously
                    step_targets = {}
                    # swing leg coxa -> FRONT
                    step_targets[swing_ids[COXA]] = (swing_front, swing_speed)
                    # stance leg coxas -> BACK
                    for stance_num in stance_nums:
                        stance_ids = LEG_IDS[stance_num]
                        _, back = coxa_front_back(stance_num, limits, flipped)
                        step_targets[stance_ids[COXA]] = (back, swing_speed)
                    bus.sync_goal_move(step_targets)
                    bus.sync_wait_until_stopped(
                        [swing_ids[COXA]] +
                        [LEG_IDS[n][COXA] for n in stance_nums],
                        sweep_timeout)

                    # PLANT: lower swing leg foot
                    plant_targets = pose_targets(
                        [swing_ids[FEMUR], swing_ids[TIBIA]], stand, limits, lift_speed)
                    bus.sync_goal_move(plant_targets)
                    bus.sync_wait_until_stopped(
                        [swing_ids[FEMUR], swing_ids[TIBIA]], lift_timeout)

                    # Telemetry
                    telem = bus.read_telemetry(all_ids)
                    coxa_pos = {n: telem[LEG_IDS[n][COXA]]["position"]
                                for n in LEG_IDS if LEG_IDS[n][COXA] in telem}
                    voltages = [t["voltage_v"] for t in telem.values()]
                    temps    = [t["temp_c"]    for t in telem.values()]
                    cx_str = "  ".join(f"L{n}cx={coxa_pos[n]:4d}" for n in coxa_pos)
                    v_str  = f"{sum(voltages)/len(voltages):.1f}V" if voltages else "?V"
                    t_str  = f"{max(temps)}°C" if temps else "?°C"
                    print(f"         {cx_str}   [{v_str} {t_str}]")

            # --- return to stand -----------------------------------------
            print("\nReturning all legs to stand pose...")
            stand_targets = {}
            for leg_num, leg_ids in LEG_IDS.items():
                stand_targets.update(
                    pose_targets(leg_ids, stand, limits, lift_speed))
            bus.sync_goal_move(stand_targets)
            bus.sync_wait_until_stopped(all_ids, sweep_timeout + lift_timeout)
            print("\n✓ Wave gait complete.")

        except KeyboardInterrupt:
            print("\n\nCtrl+C -- holding current position.")
            return
        finally:
            if args.release:
                try:
                    bus.sync_enable_torque(all_ids, on=False)
                    print("Torque released.")
                except IOError:
                    pass
            else:
                print("Holding stance (torque on).")


if __name__ == "__main__":
    main()
