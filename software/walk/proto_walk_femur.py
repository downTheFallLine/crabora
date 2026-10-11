#!/usr/bin/env python3
"""
CRABORA proto-walk femur: lift and lower
=====================================

The femur equivalent of proto_walk_coxa.py. Femurs only -- the coxas are
left wherever they are, and nothing else moves.

Sequence, with an operator check at each stop:

  1. Centre every femur (= HORIZONTAL), then PAUSE -- you confirm
  2. Lift every femur UP together, then PAUSE -- you confirm
  3. Lower every femur DOWN together, then PAUSE -- you confirm
  4. Return to centre and go limp

With --reps N it runs N up/down cycles continuously instead, with one
compact line per rep and a summary at the end.

CENTRE IS THE FEMUR HORIZONTAL, envelope ±90° (see README.md). The
envelope is symmetric but the useful range is not: up is swing clearance
and tuck, down quickly straightens the leg and loses reach. Default
--angle here is deliberately well inside the envelope.

THE POINT OF THIS SCRIPT is to settle FEMUR_UP_SIGN. The coxa's mirror
came from body geometry; the femur has no equivalent, because lift is
the same physical motion on every leg. So gait_lib predicts ONE sign for
all six. If that is right, every femur rises together here. If the two
sides split -- legs 1-3 rising while 4-6 drop, or vice versa -- the
femurs are mirrored after all and FEMUR_UP_SIGN needs per-side values.
Watch for that specifically; it is the one outcome this script exists to
catch.

Gravity is not symmetric either: down is assisted, up works against the
leg's weight. Expect the two directions to settle differently, and a
larger position error going up. That is the mechanism, not a fault.

Run --dry-run first. It prints every target without energising anything.

Usage:
  python3 proto_walk_femur.py --dry-run
  python3 proto_walk_femur.py --angle 20
  python3 proto_walk_femur.py --reps 10 --no-pause
  python3 proto_walk_femur.py --legs 1 4          # one leg per side: the sign test

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys

from gait_lib import (
    ALL_LEGS, CENTER_POSITION, FEMUR_JOINT_DIGIT, FEMUR_MAX_DEFLECTION_DEG,
    FEMUR_UP_SIGN, LEG_NAME, POSITION_TOLERANCE,
    BusError, PacketHandler,
    arm, clamp_targets, confirm, find_joints, find_urt_devices, move_all,
    open_ports, read_limits, release, report, to_counts, to_degrees,
)

DEFAULT_ANGLE = 25.0      # degrees either side of horizontal
DEFAULT_SPEED = 300       # steps/s


def up_sign(leg, invert=False):
    return FEMUR_UP_SIGN[leg] * (-1 if invert else 1)


def plan_targets(femurs, limits, counts, direction, invert):
    """direction: +1 up, -1 down, 0 centre (horizontal)."""
    want = {leg: CENTER_POSITION + direction * up_sign(leg, invert) * counts
            for leg in femurs}
    return clamp_targets(want, limits)


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA proto-femur: lift and lower the femurs.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=1,
                    help="up/down cycles to run (default 1). More than one "
                         "runs continuously, with no pause between stages.")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmations entirely")
    ap.add_argument("--angle", type=float, default=DEFAULT_ANGLE,
                    help=f"degrees either side of horizontal "
                         f"(default {DEFAULT_ANGLE:g}, envelope "
                         f"±{FEMUR_MAX_DEFLECTION_DEG:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--invert", action="store_true",
                    help="flip FEMUR_UP_SIGN on all six legs at once")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 < args.angle <= FEMUR_MAX_DEFLECTION_DEG:
        ap.error(f"--angle must be between 0 and {FEMUR_MAX_DEFLECTION_DEG:g} "
                 f"degrees (the femur envelope)")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    counts = to_counts(args.angle)
    print("CRABORA proto-walk femur: lift and lower")
    print(f"  ±{args.angle:g}° (±{counts} counts) from HORIZONTAL at "
          f"{args.speed} steps/s{'  [INVERTED]' if args.invert else ''}")
    print(f"  centre 2048 = femur horizontal; envelope "
          f"±{FEMUR_MAX_DEFLECTION_DEG:g}°\n")

    print(f"  {'leg':>3} {'servo':>5} {'name':<12} {'up is':>9}")
    for leg in legs:
        sid = leg * 10 + FEMUR_JOINT_DIGIT
        s = up_sign(leg, args.invert)
        print(f"  {leg:>3} {sid:>5} {LEG_NAME[leg]:<12} "
              f"{('count + ' if s > 0 else 'count - '):>9}")

    sides = {l for l in legs if l <= 3}, {l for l in legs if l >= 4}
    if not (sides[0] and sides[1]):
        print("\n  ⚠ only one side present -- this run cannot tell you whether "
              "the femurs are mirrored. Include a leg from each side "
              "(e.g. --legs 1 4) for that.")

    if args.dry_run:
        print(f"\n  Nominal targets (before firmware limits clamp them):")
        for leg in legs:
            s = up_sign(leg, args.invert)
            print(f"    leg {leg}: up {CENTER_POSITION + s * counts}, "
                  f"horizontal {CENTER_POSITION}, "
                  f"down {CENTER_POSITION - s * counts}")
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
    femurs = {}
    try:
        femurs = find_joints(packet, ports, legs, FEMUR_JOINT_DIGIT)
        if not femurs:
            print("\nNo femur servos answered. Run tests/test_03_ping_servo.py.")
            return 1
        missing = [l for l in legs if l not in femurs]
        if missing:
            print(f"\n  Legs {', '.join(map(str, missing))} did not answer -- "
                  f"continuing with {len(femurs)} of {len(legs)}.")

        print()
        limits = read_limits(packet, femurs)
        arm(packet, femurs, args.speed)

        trouble = {}

        def go(label, direction, verbose=True):
            targets, clamped = plan_targets(femurs, limits, counts, direction,
                                            args.invert)
            if verbose:
                print(f"\n  -> {label}")
                for leg, want, got in clamped:
                    print(f"     leg {leg}: {want} clamped to {got} by its "
                          f"firmware limit ({to_degrees(got):+.1f}° not "
                          f"{to_degrees(want):+.1f}°)")
            results = move_all(packet, femurs, targets)
            arrived = True
            if verbose:
                arrived, _bad = report(femurs, targets, results)
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
        go("CENTRE (horizontal)", 0)
        if args.pause and not confirm(
                "Is every femur HORIZONTAL? Sight along the body -- any that "
                "sits high or low needs set_center.py before going further."):
            print("\n  Stopped. Going limp.")
            return 1

        single = args.reps == 1 and args.pause
        if not single:
            print()
        for rep in range(1, args.reps + 1):
            if not single:
                print(f"  rep {rep}/{args.reps}:", end="", flush=True)
            for label, direction, question in (
                    ("UP",   +1, "Did EVERY femur rise? If legs 1-3 went one "
                                 "way and 4-6 the other, the femurs are "
                                 "mirrored and FEMUR_UP_SIGN needs per-side "
                                 "values."),
                    ("DOWN", -1, "Did every femur lower?")):
                _results, arrived = go(label, direction, verbose=single)
                if not single:
                    print(f" {label.lower()}{'' if arrived else ' ✗'}",
                          end="", flush=True)
                elif not confirm(question):
                    print("\n  Stopped. Returning to horizontal and going limp.")
                    move_all(packet, femurs,
                             {leg: CENTER_POSITION for leg in femurs})
                    return 1
            if not single:
                print()

        print("\n  -> CENTRE (finishing)")
        centre = {leg: CENTER_POSITION for leg in femurs}
        report(femurs, centre, move_all(packet, femurs, centre))

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): "
                      f"{', '.join(sorted(trouble[leg]))}")
            print("\n  Note that up fights gravity and down is assisted, so a "
                  "little more error going up is the mechanism, not a fault. "
                  "A stall or a one-sided failure is not.")
            return 1
        if args.reps > 1:
            print(f"\n  {args.reps} rep(s) completed cleanly on "
                  f"{len(femurs)} femur(s).")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\n{e}")
        return 1
    finally:
        if femurs:
            print("\n  Releasing torque on all femurs.")
            release(packet, femurs)
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
