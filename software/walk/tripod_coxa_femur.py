#!/usr/bin/env python3
"""
CRABORA tripod gait -- coxa + femur
=====================================

The first script with the real shape of a walking gait. tripod_coxa.py
could only swing legs fore and aft; adding the femur gives LIFT, so the
swinging tripod clears while the stance tripod drives.

One stride is two half-strides, three moves each:

    half-stride 1          half-stride 2
    1. LIFT   A up         4. LIFT   B up
    2. SWING  A forward,   5. SWING  B forward,
              B rear                 A rear
    3. PLANT  A down       6. PLANT  B down

Tripod A is legs 1, 3, 5 and tripod B is 2, 4, 6. Each move touches only
one joint type -- the lift and plant phases move femurs, the swing phase
moves coxas -- which also means at most six servos start at once, not
twelve, so the current draw is no worse than the coxa-only gait.

STANCE FEMUR IS CENTRE (horizontal). On the test stand nothing bears
weight, so there is no real ride height to hold; "planted" just means
back to horizontal. When the robot eventually stands on its feet this
is the number that has to change -- stance femur becomes whatever angle
holds the body at ride height, and --lift is measured from there.

⚠ TEST STAND ONLY, legs hanging free. Nothing here is load-bearing and
the tibias are not involved, so this cannot support the robot. What it
does prove is the phase sequencing, that lift and swing compose
correctly, and that both tripods stay opposed across a long run.

Signs come from gait_lib and are both hardware-verified: the coxa is
mirrored by side, the femur is uniform across all six.

Usage:
  python3 tripod_coxa_femur.py --dry-run
  python3 tripod_coxa_femur.py --reps 10
  python3 tripod_coxa_femur.py --reps 50 --stride 15 --lift 15 --speed 200
  python3 tripod_coxa_femur.py --reps 4 --reverse
  python3 tripod_coxa_femur.py --legs 1 2 4 5

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys
import time

from gait_lib import (
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    FEMUR_UP_SIGN, LEG_NAME, TRIPOD_A, TRIPOD_B,
    BusError, PacketHandler,
    arm, check, clamp_targets, confirm, find_joints, find_urt_devices,
    forward_sign, move_all, open_ports, read_limits, release, report,
    to_counts, to_degrees,
)

DEFAULT_STRIDE = 20.0     # coxa degrees either side of centre
DEFAULT_LIFT   = 20.0     # femur degrees up from horizontal during swing
DEFAULT_SPEED  = 300
DEFAULT_REPS   = 4
DEFAULT_DWELL  = 0.0


def tripod_of(leg):
    return "A" if leg in TRIPOD_A else "B"


def femur_up_sign(leg, invert=False):
    return FEMUR_UP_SIGN[leg] * (-1 if invert else 1)


def femur_targets(legs, lift_counts, swinging, invert):
    """Femurs: the swinging tripod lifted, the stance tripod at centre."""
    return {leg: (CENTER_POSITION + femur_up_sign(leg, invert) * lift_counts
                  if tripod_of(leg) == swinging else CENTER_POSITION)
            for leg in legs}


def coxa_targets(legs, stride_counts, swinging, invert, reverse):
    """Coxas: the swinging tripod forward, the stance tripod rear.

    reverse flips the whole gait, so the robot would walk backwards.
    """
    targets = {}
    for leg in legs:
        go_forward = tripod_of(leg) == swinging
        if reverse:
            go_forward = not go_forward
        direction = +1 if go_forward else -1
        targets[leg] = (CENTER_POSITION
                        + direction * forward_sign(leg, invert) * stride_counts)
    return targets


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA tripod gait, coxa + femur.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=DEFAULT_REPS,
                    help=f"full strides to run (default {DEFAULT_REPS}); "
                         f"one stride = two half-strides = six moves")
    ap.add_argument("--stride", type=float, default=DEFAULT_STRIDE,
                    help=f"coxa degrees either side of centre "
                         f"(default {DEFAULT_STRIDE:g})")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT,
                    help=f"femur degrees up from horizontal during swing "
                         f"(default {DEFAULT_LIFT:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--dwell", type=float, default=DEFAULT_DWELL,
                    help=f"seconds to hold at each move (default {DEFAULT_DWELL:g})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--reverse", action="store_true", help="run the gait backwards")
    ap.add_argument("--invert-coxa", action="store_true",
                    help="flip the coxa forward sign on all six legs")
    ap.add_argument("--invert-femur", action="store_true",
                    help="flip the femur up sign on all six legs")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmation before moving")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the stride plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 < args.stride <= 90:
        ap.error("--stride must be between 0 and 90 degrees")
    if not 0 < args.lift <= 90:
        ap.error("--lift must be between 0 and 90 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    if args.dwell < 0:
        ap.error("--dwell cannot be negative")
    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    stride_counts = to_counts(args.stride)
    lift_counts = to_counts(args.lift)

    print("CRABORA tripod gait -- coxa + femur")
    print(f"  {args.reps} stride(s): stride ±{args.stride:g}° "
          f"(±{stride_counts}), lift {args.lift:g}° ({lift_counts}) "
          f"at {args.speed} steps/s"
          f"{'  [REVERSE]' if args.reverse else ''}")
    print("  stance femur = centre (horizontal) -- nothing bears weight on "
          "the stand")
    print("  ⚠ test stand only, legs hanging free; tibias not involved\n")

    in_a = [l for l in legs if l in TRIPOD_A]
    in_b = [l for l in legs if l in TRIPOD_B]
    print(f"  tripod A: {', '.join(f'{l} ({LEG_NAME[l]})' for l in in_a) or '-'}")
    print(f"  tripod B: {', '.join(f'{l} ({LEG_NAME[l]})' for l in in_b) or '-'}")
    if not in_a or not in_b:
        print("\n  ⚠ one tripod is empty -- this will cycle a single group "
              "rather than alternate. Not a gait, but a valid exercise.")

    if args.dry_run:
        # Raw counts differ between mirrored sides, so print what each
        # move does in the BODY frame -- that is what must look right.
        def words(targets, joint, invert):
            out = []
            for leg in legs:
                d = targets[leg] - CENTER_POSITION
                if d == 0:
                    out.append("--")
                elif joint == "femur":
                    out.append("UP" if d * femur_up_sign(leg, invert) > 0
                               else "dn")
                else:
                    out.append("fwd" if d * forward_sign(leg, invert) > 0
                               else "rear")
            return out

        hdr = "  ".join(f"{l:>4}" for l in legs)
        print(f"\n  {'move':<14} {'joint':<6} {hdr}")
        for half, swinging in ((1, "A"), (2, "B")):
            for label, joint, targets, inv in (
                    (f"{half}. LIFT {swinging}", "femur",
                     femur_targets(legs, lift_counts, swinging,
                                   args.invert_femur), args.invert_femur),
                    (f"{half}. SWING", "coxa",
                     coxa_targets(legs, stride_counts, swinging,
                                  args.invert_coxa, args.reverse),
                     args.invert_coxa),
                    (f"{half}. PLANT {swinging}", "femur",
                     femur_targets(legs, 0, swinging, args.invert_femur),
                     args.invert_femur)):
                cells = "  ".join(f"{w:>4}" for w in words(targets, joint, inv))
                print(f"  {label:<14} {joint:<6} {cells}")
        tag = "  ".join(f"{tripod_of(l):>4}" for l in legs)
        print(f"  {'':<14} {'tripod':<6} {tag}")
        print("\n  UP = femur lifted, -- = femur horizontal (planted).")
        print("  Counts differ between mirrored sides; judge it by these words.")
        print("  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1

    print()
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    coxas, femurs = {}, {}
    try:
        coxas = find_joints(packet, ports, legs, COXA_JOINT_DIGIT)
        femurs = find_joints(packet, ports, legs, FEMUR_JOINT_DIGIT)

        # A leg needs BOTH joints to take part in this gait.
        usable = sorted(set(coxas) & set(femurs))
        partial = sorted((set(coxas) | set(femurs)) - set(usable))
        if partial:
            print(f"\n  Legs {', '.join(map(str, partial))} answered on only "
                  f"one joint -- excluded, this gait needs coxa AND femur.")
            for leg in partial:
                coxas.pop(leg, None)
                femurs.pop(leg, None)
        if not usable:
            print("\nNo leg has both a coxa and a femur answering. "
                  "Run tests/test_03_ping_servo.py.")
            return 1
        legs = usable

        print()
        limits_c = read_limits(packet, coxas, verbose=False)
        limits_f = read_limits(packet, femurs, verbose=False)
        print(f"  {len(legs)} leg(s) with both joints: "
              f"{', '.join(map(str, legs))}")
        arm(packet, coxas, args.speed)
        arm(packet, femurs, args.speed)

        # --- centre everything, and confirm before cycling ---------------
        print("\n  -> CENTRE (coxas)")
        centre_c = {leg: CENTER_POSITION for leg in coxas}
        report(coxas, centre_c, move_all(packet, coxas, centre_c))
        print("\n  -> CENTRE (femurs, horizontal)")
        centre_f = {leg: CENTER_POSITION for leg in femurs}
        report(femurs, centre_f, move_all(packet, femurs, centre_f))

        if args.pause and not confirm(
                "On the test stand, legs hanging free, coxas centred and "
                "femurs horizontal? The next move starts the gait."):
            print("\n  Stopped. Going limp.")
            return 1

        # --- the gait ----------------------------------------------------
        trouble = {}

        def step(joints, limits, want, tag):
            targets, clamped = clamp_targets(want, limits)
            results = move_all(packet, joints, targets)
            ok, bad = check(targets, results)
            for leg, why in bad.items():
                trouble.setdefault(leg, set()).add(f"{why} ({tag})")
            for leg, _w, _g in clamped:
                trouble.setdefault(leg, set()).add(f"clamped ({tag})")
            if args.dwell:
                time.sleep(args.dwell)
            return ok

        print()
        for rep in range(1, args.reps + 1):
            print(f"  stride {rep}/{args.reps}:", end="", flush=True)
            for swinging in ("A", "B"):
                phases = (
                    ("lift", femurs, limits_f,
                     femur_targets(legs, lift_counts, swinging, args.invert_femur)),
                    ("swing", coxas, limits_c,
                     coxa_targets(legs, stride_counts, swinging,
                                  args.invert_coxa, args.reverse)),
                    ("plant", femurs, limits_f,
                     femur_targets(legs, 0, swinging, args.invert_femur)),
                )
                for name, joints, limits, want in phases:
                    ok = step(joints, limits, want, f"{name} {swinging}")
                    print(f" {swinging}-{name}{'' if ok else '✗'}",
                          end="", flush=True)
            print()

        print("\n  -> CENTRE (finishing)")
        report(femurs, centre_f, move_all(packet, femurs, centre_f))
        report(coxas, centre_c, move_all(packet, coxas, centre_c))

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): "
                      f"{', '.join(sorted(trouble[leg]))}")
            print("\n  Check for binding, a leg fouling its neighbour, travel "
                  "limits narrower than --stride or --lift, or the supply "
                  "sagging. The phase tag says which move it happened on.")
            return 1

        print(f"\n  {args.reps} stride(s) completed cleanly on "
              f"{len(legs)} leg(s).")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\n{e}")
        return 1
    finally:
        if coxas or femurs:
            print("\n  Releasing torque.")
            release(packet, coxas)
            release(packet, femurs)
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
