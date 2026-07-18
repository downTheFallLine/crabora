"""
CRABORA: tripod_walk -- first walking gait, alternating tripods
===============================================================

The classic hexapod gait: legs split into two tripods that take turns
being planted (stable, propelling) and swinging (in the air, reaching
forward).  One tripod is always on the ground, so the bot is statically
stable at every instant -- no balance code.

Leg layout (top view, CLOCKWISE from the front): 1, 6, 3, 5, 2, 4
  Tripod A = legs 1, 3, 2        Tripod B = legs 6, 5, 4

Half-cycle, with S = swing tripod and T = stance tripod:
  1. LIFT   S femurs rise LIFT_DEG          -- S feet leave the ground
  2. SWING  S coxas rotate to their forward pose while T coxas sweep to
            their backward pose             -- T pushes: the bot MOVES here
  3. PLANT  S femurs return to stance       -- S feet down
Then S and T swap.  One full cycle = both tripods have stepped.

Straight-line travel with radially-mounted legs: each leg's coxa swing is
scaled by sin(bearing - heading).  Side legs push hardest; legs pointing
along the travel direction lift and replant in place.  --heading 0 walks
toward leg 1 (the front); --heading 90 crab-walks toward the leg-3 side.

DIRECTION UNVERIFIED: the coxa +count rotation sense (CW vs CCW from the
top) hasn't been established on hardware.  If the first try walks
backward, add --reverse -- then we hardcode the flip and delete the flag.

Stance matches stand2.py (coxa center, femur +45, tibia +90).  The script
moves to that stance first (one tripod at a time, 5 A budget), walks, then
recenters the coxas and HOLDS.  Tibias never move during the gait.

Usage:
  python tripod_walk.py                  # 4 full cycles toward the front
  python tripod_walk.py --steps 10
  python tripod_walk.py --heading 90     # crab-walk toward the leg-3 side
  python tripod_walk.py --swing-deg 8 --lift-deg 10 --speed 8
  python tripod_walk.py --reverse        # if the first try walked backward
"""

import argparse
import math
import sys

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE,
    MODE_POSITION, MODE_NAMES,
    COUNTS_PER_REV, DEGREES_PER_REV, CENTER_POSITION, MAX_POSITION,
    TYPICAL_MAX_RPM,
    COXA, FEMUR, TIBIA, JOINT_NAMES, joint_of, make_id,
    TRIPOD_A, TRIPOD_B, leg_bearing_deg,
    describe_id, rpm_to_pos_speed,
)

# -----------------------------------------------------------------------------
# Stance (same as stand2.py) and gait geometry, degrees off center.
STANCE_DEG = {COXA: 0.0, FEMUR: 45.0, TIBIA: 90.0}

SWING_DEG = 12.0   # coxa swing amplitude (coxa limits are only ±20°)
LIFT_DEG  = 15.0   # how far the femur rises to unweight a swinging foot
SPEED_RPM = 10.0


def deg_to_counts(deg):
    """Absolute position counts for an angle in degrees off center."""
    return CENTER_POSITION + round(deg / DEGREES_PER_REV * COUNTS_PER_REV)


def parse_args():
    p = argparse.ArgumentParser(
        description="Alternating-tripod walking gait (legs discovered at "
                    "runtime).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--steps", type=int, default=4,
                   help="full gait cycles to walk (default 4)")
    p.add_argument("--heading", type=float, default=0.0,
                   help="travel direction, degrees clockwise from the front "
                        "(default 0 = toward leg 1)")
    p.add_argument("--swing-deg", type=float, default=SWING_DEG,
                   help=f"coxa swing amplitude in degrees (default {SWING_DEG:g})")
    p.add_argument("--lift-deg", type=float, default=LIFT_DEG,
                   help=f"femur lift in degrees (default {LIFT_DEG:g})")
    p.add_argument("--speed", type=float, default=SPEED_RPM,
                   help=f"joint speed in RPM (default {SPEED_RPM:g})")
    p.add_argument("--reverse", action="store_true",
                   help="flip all coxa swing directions (first-run direction "
                        "test)")
    p.add_argument("--release", action="store_true",
                   help="relax when done (default: hold the stance)")
    p.add_argument("--yes", action="store_true",
                   help="skip the initial confirmation")
    return p.parse_args()


def preflight(bus, servo_id):
    """Position-mode + angle-limit check. Exits on failure -> (min, max)."""
    label = describe_id(servo_id)
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
    return mn, mx


def main():
    args = parse_args()
    if args.steps < 1:
        print("✗ --steps must be at least 1."); sys.exit(1)
    if args.speed <= 0:
        print("✗ Speed must be positive (RPM)."); sys.exit(1)
    if args.speed > TYPICAL_MAX_RPM:
        print(f"⚠  {args.speed:.1f} rpm exceeds the STS3215 typical max "
              f"(~{TYPICAL_MAX_RPM}).")
    if args.lift_deg <= 0 or args.swing_deg <= 0:
        print("✗ --swing-deg and --lift-deg must be positive."); sys.exit(1)
    raw_speed = rpm_to_pos_speed(args.speed)

    print("=" * 62)
    print("CRABORA: tripod walk")
    print("=" * 62)
    print(f"  steps  : {args.steps} full cycle(s)")
    print(f"  heading: {args.heading:g}° clockwise from front"
          f"{'  (REVERSED)' if args.reverse else ''}")
    print(f"  swing  : ±{args.swing_deg:g}° coxa   lift: {args.lift_deg:g}° femur")
    print(f"  speed  : {args.speed:g} rpm")
    print()

    with MultiBus() as bus:
        legs = bus.legs()
        if not legs:
            print("✗ No servos answered on any URT. Bus power on? Cables seated?")
            sys.exit(1)

        # Complete legs only, and every leg must have a known bearing.
        for leg_num, ids in sorted(legs.items()):
            joints = {joint_of(sid) for sid in ids}
            missing = [JOINT_NAMES[j] for j in (COXA, FEMUR, TIBIA)
                       if j not in joints]
            if missing:
                print(f"✗ leg {leg_num} is missing its {'/'.join(missing)} "
                      f"(only {ids} answered). Fix power/cabling, or unplug "
                      f"the whole leg to walk without it.")
                sys.exit(1)
            if leg_num not in TRIPOD_A + TRIPOD_B:
                print(f"✗ leg {leg_num} is not in LEG_LAYOUT_CW -- can't "
                      f"place it in a tripod. Update crabora_bus.py.")
                sys.exit(1)

        all_ids = bus.live_ids
        tripods = [
            ("A", [n for n in TRIPOD_A if n in legs]),
            ("B", [n for n in TRIPOD_B if n in legs]),
        ]
        for name, nums in tripods:
            note = "" if len(nums) >= 3 else "  ⚠  fewer than 3 legs -- UNSTABLE"
            print(f"  tripod {name}: legs {nums}{note}")

        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}

        # --- compute every target this gait will ever command -------------
        stance = {sid: deg_to_counts(STANCE_DEG[joint_of(sid)])
                  for sid in all_ids}
        lift_counts = deg_to_counts(STANCE_DEG[FEMUR] - args.lift_deg)

        sign = -1.0 if args.reverse else 1.0
        coxa_fwd, coxa_back = {}, {}
        for leg_num in legs:
            scale = math.sin(math.radians(
                leg_bearing_deg(leg_num) - args.heading))
            off = round(sign * scale * args.swing_deg
                        / DEGREES_PER_REV * COUNTS_PER_REV)
            coxa_fwd[leg_num]  = CENTER_POSITION + off
            coxa_back[leg_num] = CENTER_POSITION - off

        # Refuse (don't clamp) any computed target outside firmware limits:
        # a clamped gait limps unpredictably.
        bad = []
        for sid in all_ids:
            targets_here = [stance[sid]]
            if joint_of(sid) == FEMUR:
                targets_here.append(lift_counts)
            if joint_of(sid) == COXA:
                targets_here += [coxa_fwd[sid // 10], coxa_back[sid // 10]]
            mn, mx = limits[sid]
            bad += [(sid, t, mn, mx) for t in targets_here if not mn <= t <= mx]
        if bad:
            print("\n✗ Gait targets OUTSIDE firmware angle limits -- refusing:")
            for sid, t, mn, mx in bad:
                print(f"    {describe_id(sid)} (ID {sid})  target {t}  "
                      f"vs limits {mn}..{mx}")
            print("  Reduce --swing-deg / --lift-deg, or widen limits "
                  "(set_limits.py).")
            sys.exit(1)

        if not args.yes:
            try:
                ans = input(f"\nProceed? {len(legs)} leg(s), {args.steps} "
                            "cycle(s). Clear floor space in the travel "
                            "direction; PSU ~5 A limit. [y/N] ")
            except EOFError:
                ans = "n"
            if ans.strip().lower() not in ("y", "yes"):
                print("Aborted."); return

        # Longest single move is a coxa going fwd->back (2x swing).
        longest = max(2 * abs(coxa_fwd[n] - CENTER_POSITION) for n in legs)
        longest = max(longest, abs(stance[make_id(min(legs), FEMUR)] - lift_counts))
        move_timeout = max(longest, 1) / raw_speed * 2.0 + 1.0

        def move(targets_by_sid):
            bus.sync_goal_move({sid: (t, raw_speed)
                                for sid, t in targets_by_sid.items()})
            if not bus.sync_wait_until_stopped(list(targets_by_sid),
                                               move_timeout):
                print("  (servos slow to report stopped -- continuing)")

        try:
            bus.sync_enable_torque(all_ids, on=True)

            # --- assume the stance, one tripod at a time (5 A budget) ------
            print("\nAssuming stance...")
            for name, nums in tripods:
                move({sid: stance[sid] for n in nums for sid in legs[n]})
            print("✓ standing, coxas centered")

            # --- walk ------------------------------------------------------
            print(f"\nWalking: {args.steps} cycle(s). Ctrl+C stops and holds.\n")
            for step in range(1, args.steps + 1):
                for (s_name, s_nums), (t_name, t_nums) in (
                        (tripods[0], tripods[1]), (tripods[1], tripods[0])):
                    print(f"  cycle {step}/{args.steps}: tripod {s_name} swings, "
                          f"tripod {t_name} pushes")
                    # 1. LIFT the swing tripod's feet
                    move({make_id(n, FEMUR): lift_counts for n in s_nums})
                    # 2. SWING fwd (in air) + PUSH back (on ground) together
                    combined = {make_id(n, COXA): coxa_fwd[n] for n in s_nums}
                    combined |= {make_id(n, COXA): coxa_back[n] for n in t_nums}
                    move(combined)
                    # 3. PLANT the swing tripod's feet
                    move({make_id(n, FEMUR): stance[make_id(n, FEMUR)]
                          for n in s_nums})

            # --- recenter coxas (feet planted; slight drag is fine) --------
            print("\nRecentering coxas...")
            for name, nums in tripods:
                move({make_id(n, COXA): CENTER_POSITION for n in nums})
            print("\n✓ Done walking.")

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
