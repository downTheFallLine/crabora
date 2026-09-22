"""
CRABORA: wave hello
===================

Raises one leg into a lifted pose and sweeps its coxa back and forth
between the configured angle limits -- a friendly wave.  The other two
legs hold the stand pose throughout, keeping the body stable on its
two-legged tripod.

Motion sequence:
  1. All three legs move to the 'stand' pose (from poses.json).
  2. Wave leg: femur + tibia go to their 'tuck' positions so the foot
     lifts off the ground.  Coxa stays at the stand position.
  3. Coxa sweeps between min_limit and max_limit  N times  (the wave).
  4. Wave leg returns to 'stand'.  Torque stays on (use --release to drop it).

The foot-lift uses the captured 'tuck' pose, so the leg folds into the
same tucked geometry that stand.py starts from -- a known-safe lifted
position.  No separate 'wave' pose capture is needed.

Arguments:
  wave_leg      leg number to wave: 1, 2, or 3 (default: 1)
  --swings N    number of full back-and-forth sweeps (default: 3)
  --speed RPM   coxa sweep speed in RPM (default: 12)
  --lift-speed  RPM for the initial lift and final return (default: 8)
  --release     relax all torque after the wave (default: hold)
  --no-pause    skip the "ready to wave?" confirmation

Requirements (all 9 servos):
  - Position mode    -> run wheel_off.py <id>
  - Angle limits set -> run set_limits.py <id>
  - poses.json with 'tuck' and 'stand' -> run teach_pose.py

Usage:
  python wave.py          # leg 1 waves, 3 swings
  python wave.py 2        # leg 2 waves
  python wave.py 1 --swings 5 --speed 18
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
    describe_id, rpm_to_pos_speed,
)

LEG_IDS = {1: [11, 12, 13], 2: [21, 22, 23], 3: [31, 32, 33]}
POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

COXA  = 0   # index within a leg list
FEMUR = 1
TIBIA = 2


def parse_args():
    p = argparse.ArgumentParser(
        description="Wave one leg hello while the other two hold the stand pose.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("wave_leg", type=int, nargs="?", default=1,
                   choices=[1, 2, 3],
                   help="which leg to wave: 1, 2, or 3 (default: 1)")
    p.add_argument("--swings", type=int, default=3,
                   help="number of full back-and-forth coxa sweeps (default: 3)")
    p.add_argument("--speed", type=float, default=12.0,
                   help="coxa wave speed in RPM (default: 12)")
    p.add_argument("--lift-speed", type=float, default=8.0,
                   help="speed for lifting and lowering the wave leg (default: 8)")
    p.add_argument("--release", action="store_true",
                   help="relax all torque after the wave (default: hold stance)")
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
    """Ping + position-mode + limits check. Returns (min, max) or exits."""
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


def pose_targets(leg_ids, pose, limits, speed):
    """Build {servo_id: (counts, speed)} for a leg from a pose dict."""
    targets = {}
    for sid in leg_ids:
        key = str(sid)
        if key not in pose:
            print(f"✗ pose missing servo {sid}. Re-run teach_pose.py.")
            sys.exit(1)
        counts = int(pose[key])
        mn, mx = limits[sid]
        counts = max(mn, min(mx, counts))
        targets[sid] = (counts, speed)
    return targets


def main():
    args = parse_args()

    if args.swings < 1:
        print("✗ --swings must be at least 1."); sys.exit(1)
    for rpm, name in ((args.speed, "--speed"), (args.lift_speed, "--lift-speed")):
        if rpm <= 0:
            print(f"✗ {name} must be positive."); sys.exit(1)
        if rpm > TYPICAL_MAX_RPM:
            print(f"⚠  {name} {rpm:.1f} rpm exceeds the STS3215 typical max "
                  f"(~{TYPICAL_MAX_RPM} rpm).")

    tuck, stand = load_poses()
    wave_leg    = LEG_IDS[args.wave_leg]
    stance_legs = [ids for leg_num, ids in LEG_IDS.items() if leg_num != args.wave_leg]
    all_ids     = [sid for ids in LEG_IDS.values() for sid in ids]

    wave_speed  = rpm_to_pos_speed(args.speed)
    lift_speed  = rpm_to_pos_speed(args.lift_speed)

    print("=" * 60)
    print("CRABORA: wave hello")
    print("=" * 60)
    print(f"  waving   : leg {args.wave_leg}  "
          f"({' '.join(describe_id(s) for s in wave_leg)})")
    print(f"  stance   : legs "
          f"{', '.join(str(n) for n in LEG_IDS if n != args.wave_leg)}")
    print(f"  swings   : {args.swings}")
    print(f"  wave RPM : {args.speed:.1f}    lift RPM: {args.lift_speed:.1f}")

    if not args.no_pause:
        try:
            ans = input("\nProceed? Bot in stand pose, all legs grounded. [y/N] ")
        except EOFError:
            ans = "n"
        if ans.strip().lower() not in ("y", "yes"):
            print("Aborted."); return

    with MultiBus() as bus:

        # --- preflight all 9 servos --------------------------------------
        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}

        coxa_id  = wave_leg[COXA]
        coxa_min, coxa_max = limits[coxa_id]
        sweep_timeout = (coxa_max - coxa_min) / wave_speed * 2.0 + 1.0
        lift_timeout  = 3.0   # generous; lifting is a short move

        try:
            bus.sync_enable_torque(all_ids, on=True)

            # --- STEP 1: move all legs to stand --------------------------
            print("\nMoving to stand pose...")
            stand_targets = {}
            for ids in LEG_IDS.values():
                stand_targets.update(pose_targets(ids, stand, limits, lift_speed))
            bus.sync_goal_move(stand_targets)
            bus.sync_wait_until_stopped(all_ids, lift_timeout + 2.0)

            # --- STEP 2: lift the wave leg (femur + tibia to tuck) -------
            print(f"\nLifting leg {args.wave_leg}...")
            lift_targets = {}
            # Stance legs stay in stand; only wave leg femur + tibia change.
            for sid in [wave_leg[FEMUR], wave_leg[TIBIA]]:
                key = str(sid)
                counts = int(tuck[key])
                mn, mx = limits[sid]
                lift_targets[sid] = (max(mn, min(mx, counts)), lift_speed)
            bus.sync_goal_move(lift_targets)
            bus.sync_wait_until_stopped(
                [wave_leg[FEMUR], wave_leg[TIBIA]], lift_timeout)

            # --- STEP 3: wave the coxa -----------------------------------
            print(f"\nWaving! ({args.swings} swings, "
                  f"{coxa_min}..{coxa_max} counts)  Press Ctrl+C to stop.\n")
            endpoints = [coxa_max, coxa_min]
            for swing in range(args.swings * 2):
                target = endpoints[swing % 2]
                bus.sync_goal_move({coxa_id: (target, wave_speed)})
                bus.sync_wait_until_stopped([coxa_id], sweep_timeout)
                pos = bus.read_uint16(coxa_id, ADDR_PRESENT_POSITION)
                direction = "→" if swing % 2 == 0 else "←"
                print(f"  swing {swing // 2 + 1:2d}{direction}  coxa={pos:4d}")

            # --- STEP 4: return wave leg to stand ------------------------
            print(f"\nLowering leg {args.wave_leg} to stand...")
            return_targets = pose_targets(wave_leg, stand, limits, lift_speed)
            bus.sync_goal_move(return_targets)
            bus.sync_wait_until_stopped(wave_leg, lift_timeout)

            print("\n✓ Wave complete.")

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
