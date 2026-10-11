#!/usr/bin/env python3
"""
CRABORA proto-walk tibia: extend and flex
=====================================

The tibia counterpart to proto_walk_coxa.py and proto_walk_tibia.py.
Tibias only -- the coxas and tibias are left wherever they are.

Sequence, with an operator check at each stop:

  1. Centre every tibia, then PAUSE -- you confirm
  2. EXTEND every tibia together (straighten the knee, foot pushed away
     from the body), then PAUSE -- you confirm
  3. FLEX every tibia together (fold the foot back toward the body),
     then PAUSE -- you confirm
  4. Return to centre and go limp

With --reps N it runs N extend/flex cycles continuously instead, with one
compact line per rep and a summary at the end.

THE POINT OF THIS SCRIPT is to settle TIBIA_EXTEND_SIGN, the last
unknown joint sign. Two predictions from the other two joints:

  - It should be UNIFORM across all six, not mirrored. Flexing a knee is
    the same physical motion on every leg, just as lifting is for the
    tibia -- which was confirmed uniform on 2026-10-11. The coxa's mirror
    comes from body geometry and has no analogue at the knee.
  - It may well be -1 rather than +1. The tibia's turned out to be, and
    if the tibia servos follow the same mounting convention this will too.

So the outcome to watch for is the two sides SPLITTING -- legs 1-3
extending while 4-6 flex. That would mean the tibias are mirrored after
all and the table needs per-side values like the coxa's. A clean
all-six-together motion in the wrong direction is just a sign flip.

Gravity is not symmetric here either: which direction it helps depends on
where the tibia happens to be sitting, since the knee hangs off it. If
the tibias are not level the two directions will settle differently.

Run --dry-run first. It prints every target without energising anything.

Usage:
  python3 proto_walk_tibia.py --dry-run
  python3 proto_walk_tibia.py --angle 20
  python3 proto_walk_tibia.py --reps 10 --no-pause
  python3 proto_walk_tibia.py --legs 1 4          # one leg per side: the mirror test

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys

from gait_lib import (
    ALL_LEGS, CENTER_POSITION, LEG_NAME, POSITION_TOLERANCE,
    TIBIA_EXTEND_SIGN, TIBIA_EXTEND_LIMIT_DEG, TIBIA_FLEX_LIMIT_DEG,
    TIBIA_JOINT_DIGIT,
    BusError, PacketHandler,
    arm, clamp_targets, confirm, find_joints, find_urt_devices, move_all,
    open_ports, read_limits, release, report, to_counts, to_degrees,
)

# Centre is the leg STRAIGHT, which is the edge of travel, not the middle:
# a knee only bends one way. So the two directions get separate defaults
# rather than one symmetric --angle.
DEFAULT_EXTEND = 5.0      # degrees past straight -- there is barely any room
DEFAULT_FLEX   = 30.0     # degrees of knee bend -- modest first pass
DEFAULT_SPEED = 300       # steps/s


def extend_sign(leg, invert=False):
    return TIBIA_EXTEND_SIGN[leg] * (-1 if invert else 1)


def plan_targets(tibias, limits, counts, direction, invert):
    """direction: +1 extend, -1 flex, 0 centre. `counts` is that
    direction's own travel -- they are not the same, see above."""
    want = {leg: CENTER_POSITION + direction * extend_sign(leg, invert) * counts
            for leg in tibias}
    return clamp_targets(want, limits)


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA proto-walk tibia: extend and flex the tibias.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=1,
                    help="extend/flex cycles to run (default 1). More than one "
                         "runs continuously, with no pause between stages.")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmations entirely")
    ap.add_argument("--extend", type=float, default=DEFAULT_EXTEND,
                    help=f"degrees PAST STRAIGHT to ask for (default "
                         f"{DEFAULT_EXTEND:g}; there is little room here, "
                         f"since centre IS straight)")
    ap.add_argument("--flex", type=float, default=DEFAULT_FLEX,
                    help=f"degrees of KNEE BEND to ask for (default "
                         f"{DEFAULT_FLEX:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--invert", action="store_true",
                    help="flip TIBIA_EXTEND_SIGN on all six legs at once")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 <= args.extend <= TIBIA_EXTEND_LIMIT_DEG:
        ap.error(f"--extend must be between 0 and {TIBIA_EXTEND_LIMIT_DEG:g} "
                 f"degrees past straight")
    if not 0 < args.flex <= TIBIA_FLEX_LIMIT_DEG:
        ap.error(f"--flex must be between 0 and {TIBIA_FLEX_LIMIT_DEG:g} degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    extend_counts = to_counts(args.extend)
    flex_counts = to_counts(args.flex)
    print("CRABORA proto-walk tibia: extend and flex")
    print(f"  extend {args.extend:g}° ({extend_counts} counts) / flex "
          f"{args.flex:g}° ({flex_counts} counts) at {args.speed} steps/s"
          f"{'  [INVERTED]' if args.invert else ''}")
    print("  centre 2048 = leg STRAIGHT (tibia collinear with femur), so "
          "travel is\n  one-sided: almost nothing to extend, all of it to "
          "flex.\n")

    print(f"  {'leg':>3} {'servo':>5} {'name':<12} {'extend is':>12}")
    for leg in legs:
        sid = leg * 10 + TIBIA_JOINT_DIGIT
        s = extend_sign(leg, args.invert)
        print(f"  {leg:>3} {sid:>5} {LEG_NAME[leg]:<12} "
              f"{('count + ' if s > 0 else 'count - '):>9}")

    sides = {l for l in legs if l <= 3}, {l for l in legs if l >= 4}
    if not (sides[0] and sides[1]):
        print("\n  ⚠ only one side present -- this run cannot tell you whether "
              "the tibias are mirrored. Include a leg from each side "
              "(e.g. --legs 1 4) for that.")

    if args.dry_run:
        print(f"\n  Nominal targets (before firmware limits clamp them):")
        for leg in legs:
            s = extend_sign(leg, args.invert)
            print(f"    leg {leg}: extend "
                  f"{CENTER_POSITION + s * extend_counts}, "
                  f"centre (straight) {CENTER_POSITION}, "
                  f"flex {CENTER_POSITION - s * flex_counts}")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("\nNo FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1

    print()
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    tibias = {}
    try:
        tibias = find_joints(packet, ports, legs, TIBIA_JOINT_DIGIT)
        if not tibias:
            print("\nNo tibia servos answered. Run tests/test_03_ping_servo.py.")
            return 1
        missing = [l for l in legs if l not in tibias]
        if missing:
            print(f"\n  Legs {', '.join(map(str, missing))} did not answer -- "
                  f"continuing with {len(tibias)} of {len(legs)}.")

        print()
        limits = read_limits(packet, tibias)
        arm(packet, tibias, args.speed)

        trouble = {}

        def go(label, direction, verbose=True):
            n = {+1: extend_counts, -1: flex_counts, 0: 0}[direction]
            targets, clamped = plan_targets(tibias, limits, n, direction,
                                            args.invert)
            if verbose:
                print(f"\n  -> {label}")
                for leg, want, got in clamped:
                    print(f"     leg {leg}: {want} clamped to {got} by its "
                          f"firmware limit ({to_degrees(got):+.1f}° not "
                          f"{to_degrees(want):+.1f}°)")
            results = move_all(packet, tibias, targets)
            arrived = True
            if verbose:
                arrived, _bad = report(tibias, targets, results)
                if not arrived:
                    print("\n  Not every joint reached its target -- "
                          "see the flags above.")
            else:
                for leg, _want, _got in clamped:
                    trouble.setdefault(leg, set()).add("clamped by travel limits")
                for leg, (pos, timed_out) in results.items():
                    if timed_out or abs(pos - targets[leg]) > POSITION_TOLERANCE:
                        arrived = False
                        trouble.setdefault(leg, set()).add(
                            "stalled" if timed_out else "off target")
            return results, arrived

        # --- centre, and the one pause worth keeping ---------------------
        go("CENTRE", 0)
        if args.pause and not confirm(
                "Is every tibia where you expect centre to be? Any that "
                "sits oddly needs set_center.py before going further."):
            print("\n  Stopped. Going limp.")
            return 1

        single = args.reps == 1 and args.pause
        if not single:
            print()
        for rep in range(1, args.reps + 1):
            if not single:
                print(f"  rep {rep}/{args.reps}:", end="", flush=True)
            for label, direction, question in (
                    ("EXTEND", +1, "Did EVERY tibia extend (knee straightening, "
                                   "foot pushed away)? If legs 1-3 went one way "
                                   "and 4-6 the other, the tibias are mirrored "
                                   "and TIBIA_EXTEND_SIGN needs per-side values."),
                    ("FLEX",   -1, "Did every tibia flex (foot folding back "
                                   "toward the body)?")):
                _results, arrived = go(label, direction, verbose=single)
                if not single:
                    print(f" {label.lower()}{'' if arrived else ' ✗'}",
                          end="", flush=True)
                elif not confirm(question):
                    print("\n  Stopped. Returning to centre and going limp.")
                    move_all(packet, tibias,
                             {leg: CENTER_POSITION for leg in tibias})
                    return 1
            if not single:
                print()

        print("\n  -> CENTRE (finishing)")
        centre = {leg: CENTER_POSITION for leg in tibias}
        report(tibias, centre, move_all(packet, tibias, centre))

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): "
                      f"{', '.join(sorted(trouble[leg]))}")
            print("\n  Which direction gravity helps depends on where the "
                  "femurs are sitting, so some asymmetry in settling is "
                  "expected. A stall or a one-sided failure is not.")
            return 1
        if args.reps > 1:
            print(f"\n  {args.reps} rep(s) completed cleanly on "
                  f"{len(tibias)} tibia(s).")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\n{e}")
        return 1
    finally:
        if tibias:
            print("\n  Releasing torque on all tibias.")
            release(packet, tibias)
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
