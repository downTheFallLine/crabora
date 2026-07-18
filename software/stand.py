"""
CRABORA: stand up from splayed -> standing  (3-phase, 5 A-budget friendly)
==========================================================================

Plays back hand-taught poses (poses.json, from teach_pose.py) to bring
CRABORA up off its belly onto its feet WITHOUT exceeding a ~5 A
bench-supply limit.

No fixed leg roster: MultiBus opens every FE-URT-2 it can see and discovers
which legs answer on which bus, so this works the same at 3 legs on two
URTs or 8 legs on three.  Every discovered leg must be complete (coxa,
femur, tibia) and present in both poses, or the script refuses to run --
standing on a half-connected leg is how robots faceplant.

Why three phases (from the power analysis):
  Lifting all legs at once from a splayed, near-horizontal pose is a
  simultaneous every-servo cantilever push -> blows past 5 A.  We dodge it
  by re-tucking the legs while the body still rests on the ground (legs
  carry no body weight, ~0.5 A each), THEN doing the coordinated push-up
  from a tucked, good-leverage start.

  PHASE 1  RE-TUCK, ONE LEG AT A TIME  (grounded, tiny simultaneous draw)
  PHASE 2  SETTLE                      (pause; read the PSU ammeter)
  PHASE 3  COORDINATED SLOW LIFT       (every servo to 'stand', slowly)

Then it HOLDS the stance (torque stays on).  Pass --release to relax.

Poses come from poses.json (absolute servo counts), captured with:
  python teach_pose.py            # teach 'tuck' and 'stand' by hand
The bot starts wherever it is (splayed); Phase 1 moves each leg to 'tuck'.

Requirements (every servo): position mode (wheel_off.py), angle limits
(set_limits.py).  Bench PSU at 7.4 V, current limit ~5 A.

Usage:
  python stand.py                 # full 3-phase stand, pausing between phases
  python stand.py --tuck-only     # just Phase 1 (grounded re-tuck), then stop
  python stand.py --skip-tuck     # assume already tucked; settle + lift only
  python stand.py --speed 5       # override speed (RPM)
  python stand.py --no-pause      # don't wait for Enter between phases
  python stand.py --release       # relax after standing (default: hold)
"""

import argparse
import json
import os
import sys

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE, ADDR_PRESENT_POSITION,
    MODE_POSITION, MODE_NAMES,
    MAX_POSITION, MAX_VALID_ID, TYPICAL_MAX_RPM,
    COXA, FEMUR, TIBIA, JOINT_NAMES, joint_of,
    describe_id, rpm_to_pos_speed,
)

# -----------------------------------------------------------------------------
POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

TUCK_RPM = 8.0     # Phase 1, per leg
LIFT_RPM = 6.0     # Phase 3, coordinated -- slowest (highest-load phase)

# Optional anti-drag: before tucking a leg, raise its femur this many COUNTS so
# the foot lifts off the bench instead of scraping.  Signed; 0 disables.
FOOT_LIFT_COUNTS = 0


def parse_args():
    p = argparse.ArgumentParser(
        description="Stand CRABORA from splayed to standing (3 phases, "
                    "legs discovered at runtime).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speed", type=float, default=None, help="override speed (RPM)")
    p.add_argument("--tuck-only", action="store_true", help="run only Phase 1")
    p.add_argument("--skip-tuck", action="store_true", help="skip Phase 1 (already tucked)")
    p.add_argument("--no-pause", action="store_true", help="don't wait for Enter between phases")
    p.add_argument("--release", action="store_true", help="relax after standing (default: hold)")
    p.add_argument("--yes", action="store_true", help="skip the initial confirmation")
    return p.parse_args()


def load_poses():
    if not os.path.exists(POSES_PATH):
        print(f"✗ No poses.json found at {POSES_PATH}.")
        print("  Capture poses first:  python teach_pose.py")
        sys.exit(1)
    with open(POSES_PATH) as f:
        poses = json.load(f)
    return poses


def preflight(bus, servo_id):
    """Ping + position-mode + angle-limit check. Exits on failure -> (min, max)."""
    label = describe_id(servo_id)
    if not bus.ping(servo_id):
        print(f"✗ No response at ID {servo_id} ({label}). Check power/cable.")
        sys.exit(1)
    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        print(f"✗ {label} (ID {servo_id}) in {MODE_NAMES.get(mode, mode)} mode; "
              f"needs position mode.  Run: python wheel_off.py {servo_id}")
        sys.exit(1)
    mn = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
    mx = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
    if mn >= mx or (mn == 0 and mx >= MAX_POSITION):
        print(f"✗ {label} (ID {servo_id}) has no usable angle limits ({mn}..{mx}). "
              f"Run: python set_limits.py {servo_id}")
        sys.exit(1)
    print(f"✓ {label} (ID {servo_id}): limits {mn}..{mx}")
    return mn, mx


def clamp(servo_id, counts, limits):
    mn, mx = limits[servo_id]
    c = max(mn, min(mx, counts))
    if c != counts:
        print(f"  ⚠  {describe_id(servo_id)} target {counts} clamped to {c} "
              f"({mn}..{mx})")
    return c


def leg_targets(leg_ids, pose, limits, raw_speed, femur_delta=0):
    """{servo_id: (counts, speed)} for one leg, from a captured pose dict."""
    targets = {}
    for sid in leg_ids:
        if str(sid) not in pose:
            print(f"✗ pose is missing servo {sid}. Re-run teach_pose.py.")
            sys.exit(1)
        counts = int(pose[str(sid)]) + (femur_delta if joint_of(sid) == FEMUR else 0)
        targets[sid] = (clamp(sid, counts, limits), raw_speed)
    return targets


def maybe_pause(msg, do_pause):
    if do_pause:
        try:
            input(f"\n{msg}  [Enter to continue, Ctrl+C to abort] ")
        except EOFError:
            pass
    else:
        print(f"\n{msg}")


def main():
    args = parse_args()
    poses = load_poses()
    for needed in ("tuck", "stand"):
        if needed not in poses:
            print(f"✗ poses.json has no '{needed}' pose. Run: python teach_pose.py")
            sys.exit(1)
    tuck, stand = poses["tuck"], poses["stand"]

    tuck_rpm = args.speed if args.speed is not None else TUCK_RPM
    lift_rpm = args.speed if args.speed is not None else LIFT_RPM
    for rpm in (tuck_rpm, lift_rpm):
        if rpm <= 0:
            print("✗ Speed must be positive (RPM)."); sys.exit(1)
        if rpm > TYPICAL_MAX_RPM:
            print(f"⚠  {rpm:.1f} rpm exceeds the STS3215 typical max (~{TYPICAL_MAX_RPM}).")
    tuck_speed = rpm_to_pos_speed(tuck_rpm)
    lift_speed = rpm_to_pos_speed(lift_rpm)

    print("=" * 62)
    print("CRABORA: stand up  (3-phase, 5 A budget, legs discovered)")
    print("=" * 62)
    print(f"  poses: tuck + stand from {os.path.basename(POSES_PATH)}")
    print(f"  speed: tuck {tuck_rpm:g} rpm, lift {lift_rpm:g} rpm")
    print()

    with MultiBus() as bus:
        legs = bus.legs()
        if not legs:
            print("✗ No servos answered on any URT. Bus power on? Cables seated?")
            sys.exit(1)

        # Refuse to stand on an incomplete leg -- a coxa with no femur/tibia
        # behind it can't carry its share of the body.
        for leg_num, ids in sorted(legs.items()):
            joints = {joint_of(sid) for sid in ids}
            missing = [JOINT_NAMES[j] for j in (COXA, FEMUR, TIBIA)
                       if j not in joints]
            if missing:
                print(f"✗ leg {leg_num} is missing its {'/'.join(missing)} "
                      f"(only {ids} answered). Fix power/cabling, or unplug "
                      f"the whole leg to stand without it.")
                sys.exit(1)

        all_ids = bus.live_ids

        # Both poses must cover every discovered servo, else it never got
        # taught (e.g. legs added since the last teach_pose run).
        for pose_name, pose in (("tuck", tuck), ("stand", stand)):
            untaught = [sid for sid in all_ids if str(sid) not in pose]
            if untaught:
                print(f"✗ pose '{pose_name}' has no positions for {untaught}. "
                      f"Re-run: python teach_pose.py")
                sys.exit(1)

        print("\n  legs : " + "  ".join(
            f"L{n}[{','.join(str(s) for s in ids)}]"
            for n, ids in sorted(legs.items())))

        if not args.yes:
            try:
                ans = input(f"\nProceed? {len(legs)} leg(s) / {len(all_ids)} "
                            "servos. Bot belly-down & splayed, PSU at 7.4 V / "
                            "~5 A limit. [y/N] ")
            except EOFError:
                ans = "n"
            if ans.strip().lower() not in ("y", "yes"):
                print("Aborted."); return

        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}

        # A taught pose was physically real, so a target outside a servo's
        # firmware limits means its calibration and limits DISAGREE (classic
        # cause: set_middle.py never ran, so the encoder reads ~90°+ off).
        # Clamping would march the joint far from the taught pose -- the
        # "leg folds under the body" failure -- so refuse instead.
        bad = []
        for pose_name, pose in (("tuck", tuck), ("stand", stand)):
            for sid in all_ids:
                counts = int(pose[str(sid)])
                mn, mx = limits[sid]
                if not mn <= counts <= mx:
                    bad.append((pose_name, sid, counts, mn, mx))
        if bad:
            print("\n✗ Taught targets OUTSIDE firmware angle limits -- refusing:")
            for pose_name, sid, counts, mn, mx in bad:
                print(f"    '{pose_name}':  {describe_id(sid)} (ID {sid})  "
                      f"target {counts}  vs limits {mn}..{mx}")
            print("  Fix: limp.py, hold the joint at its true center, "
                  "set_middle.py <id>,")
            print("  then re-teach BOTH poses: python teach_pose.py")
            sys.exit(1)

        widest = max(mx - mn for mn, mx in limits.values())
        move_timeout = widest / min(tuck_speed, lift_speed) * 2.0 + 1.0

        def report(ids):
            pos = {sid: bus.read_uint16(sid, ADDR_PRESENT_POSITION) for sid in ids}
            print("    " + "  ".join(f"{sid}={p:4d}" for sid, p in pos.items()))

        try:
            bus.sync_enable_torque(all_ids, on=True)

            if not args.skip_tuck:
                print("\n" + "-" * 62 + "\nPHASE 1: re-tuck, one leg at a time (grounded)\n" + "-" * 62)
                for leg_num, leg_ids in sorted(legs.items()):
                    maybe_pause(f"Phase 1 -- tuck leg {leg_num}.", not args.no_pause)
                    if FOOT_LIFT_COUNTS:
                        bus.sync_goal_move(leg_targets(leg_ids, tuck, limits, tuck_speed,
                                                       femur_delta=FOOT_LIFT_COUNTS))
                        bus.sync_wait_until_stopped(leg_ids, move_timeout)
                    bus.sync_goal_move(leg_targets(leg_ids, tuck, limits, tuck_speed))
                    bus.sync_wait_until_stopped(leg_ids, move_timeout)
                    report(leg_ids)

            if args.tuck_only:
                print("\n--tuck-only: done after Phase 1."); return

            print("\n" + "-" * 62 + "\nPHASE 2: settle -- all feet tucked under the body\n" + "-" * 62)
            maybe_pause("Phase 2 -- check stance + PSU current before the lift.",
                        not args.no_pause)

            print("\n" + "-" * 62 + "\nPHASE 3: coordinated lift (all legs, slow)\n" + "-" * 62)
            maybe_pause(f"Phase 3 -- about to push up on all {len(legs)} legs "
                        f"({len(all_ids)} servos at once). Watch the "
                        "ammeter; Ctrl+C aborts.", not args.no_pause)
            stand_targets = {}
            for leg_ids in legs.values():
                stand_targets.update(leg_targets(leg_ids, stand, limits, lift_speed))
            bus.sync_goal_move(stand_targets)
            if not bus.sync_wait_until_stopped(all_ids, move_timeout):
                print("  (servos slow to report stopped -- continuing)")
            report(all_ids)
            print("\n✓ Standing.")

        except KeyboardInterrupt:
            print("\n\nCtrl+C -- holding current position (servos stay driven).")
            return
        finally:
            if args.release:
                try:
                    bus.sync_enable_torque(all_ids, on=False)
                    print("Torque released -- bot will settle.")
                except IOError:
                    pass
            else:
                print("Holding stance (torque stays on after exit).")


if __name__ == "__main__":
    main()
