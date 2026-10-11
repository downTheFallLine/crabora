#!/usr/bin/env python3
"""
CRABORA turn -- rotate in place, clockwise
============================================

Turns the robot on the spot by 90, 180, 270 or 360 degrees. Always to
the RIGHT (clockwise seen from above). Assumes it is already standing;
run stand/stand.py first.

A turn is a tripod gait, but two things make it different from walking,
and both are easy to get wrong:

1. THERE IS NO LEFT/RIGHT MIRROR.
   Walking forward needs the two sides to swing opposite ways, because
   "forward" points differently for a leg on the left than one on the
   right (see walk/README.md). A turn is a pure rotation about the body
   axis, so EVERY leg rotates the same way. Turning clockwise means
   every stance coxa swings counter-clockwise, with no per-side sign.

2. THE BODY TURNS ABOUT HALF AS FAR AS THE COXA DOES.
   The coxa axis is 163 mm out from the body centre, and the foot sits
   178 mm beyond that. With the foot planted, swinging the coxa by a
   rotates the body by only

       atan2(d sin a, R + d cos a)    R = 163.2, d = 178

   which works out at 0.52 x a. Assume one-for-one and every turn lands
   at half the angle asked for. This module solves it properly, and
   also solves for the exact per-step angle so N whole steps land on
   the target rather than overshooting on the last one.

One step is a half-stride: the stance tripod sweeps from -a to +a,
pushing the body round, while the lifted tripod resets to -a ready for
its turn. Tripods alternate, so each is stance every other step.

Foot scrub: a planted foot's distance from the body centre changes
slightly through the sweep (about 3 mm at a=15, 12 mm at a=30), so the
feet do rub. That is inherent to turning with the coxa alone and is why
a larger --step is not free.

Usage:
  python3 turn.py 90 --dry-run
  python3 turn.py 180
  python3 turn.py 360 --step 25 --speed 400
  python3 turn.py 90 --height 90

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "walk"))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "stand"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, BODY_TO_COXA_AXIS, COXA_JOINT_DIGIT, COXA_RADIAL,
    FEMUR_JOINT_DIGIT, LEG_NAME, TIBIA_JOINT_DIGIT, TRIPOD_A, TRIPOD_B,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read2, to_counts,
    write1, write2,
)
from pose import Unreachable, leg_fk, pose_stand, stand_height_range  # noqa: E402
from stand import LOADED_TOLERANCE, clamp, move_group, pose_counts  # noqa: E402

TURNS = (90, 180, 270, 360)
DEFAULT_STEP = 18.0      # coxa degrees either side of centre, per step
DEFAULT_HEIGHT = 80.0
DEFAULT_SPEED = 300
DEFAULT_LIFT = 25.0      # femur degrees added to lift the swing tripod

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# TURN GEOMETRY
# =============================================================
def foot_radius(base):
    """Horizontal distance from the coxa axis out to the planted foot."""
    out, _down = leg_fk(base["femur"], base["tibia"])
    return COXA_RADIAL + out


def body_turn(coxa_deg, d):
    """Body rotation produced by swinging a planted leg's coxa.

    The hip is R from the body centre and the foot d beyond it; with the
    foot fixed, the body swings through the angle subtended at the body
    centre, not the angle at the hip.
    """
    a = math.radians(coxa_deg)
    return math.degrees(math.atan2(d * math.sin(a),
                                   BODY_TO_COXA_AXIS + d * math.cos(a)))


def scrub(coxa_deg, d):
    """How much a planted foot's radius changes over the sweep -- i.e.
    how far it has to rub. Inherent to a coxa-only turn."""
    a = math.radians(coxa_deg)
    return abs(math.hypot(d * math.sin(a), BODY_TO_COXA_AXIS + d * math.cos(a))
               - (BODY_TO_COXA_AXIS + d))


def solve_step(target, max_step, d):
    """Pick the number of half-strides and the exact coxa angle so that
    N whole steps land exactly on `target`.

    Each step sweeps the stance coxas from -a to +a, turning the body by
    2 * body_turn(a). Returns (n_steps, coxa_deg).
    """
    per_step_max = 2 * body_turn(max_step, d)
    n = max(1, math.ceil(target / per_step_max))
    want = target / n            # body degrees needed per step
    # Invert body_turn by bisection -- it is monotonic in a.
    lo, hi = 0.0, 90.0
    for _ in range(80):
        mid = (lo + hi) / 2
        if 2 * body_turn(mid, d) < want:
            lo = mid
        else:
            hi = mid
    return n, (lo + hi) / 2


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="CRABORA: turn in place, clockwise.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("degrees", type=int, choices=TURNS,
                    help="how far to turn, clockwise")
    ap.add_argument("--step", type=float, default=DEFAULT_STEP,
                    help=f"maximum coxa swing per step, degrees "
                         f"(default {DEFAULT_STEP:g}); the actual angle is "
                         f"solved down from this so whole steps hit the target")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT,
                    help=f"standing deck clearance, mm (default {DEFAULT_HEIGHT:g})")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT,
                    help=f"femur degrees to lift the swinging tripod "
                         f"(default {DEFAULT_LIFT:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")
    if not 0 < args.step <= 45:
        ap.error("--step must be between 0 and 45 degrees")
    if not 0 < args.lift <= 60:
        ap.error("--lift must be between 0 and 60 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")

    try:
        base = pose_stand(args.height)
    except Unreachable as e:
        lo, hi = stand_height_range()
        print(f"Cannot stand at {args.height:g} mm: {e}")
        print(f"Reachable range: {lo:.0f}..{hi:.0f} mm")
        return 1

    d = foot_radius(base)
    n_steps, coxa = solve_step(args.degrees, args.step, d)
    per_step = 2 * body_turn(coxa, d)

    print(f"CRABORA turn -- {args.degrees}° CLOCKWISE")
    print(f"  standing at {args.height:g} mm; foot {d:.0f} mm from the coxa "
          f"axis, {BODY_TO_COXA_AXIS + d:.0f} mm from the body centre")
    print(f"  coxa ±{coxa:.1f}° per step turns the body {per_step:.2f}° "
          f"(ratio {body_turn(coxa, d) / coxa:.3f} -- the hip offset)")
    print(f"  {n_steps} step(s) x {per_step:.2f}° = "
          f"{n_steps * per_step:.1f}°")
    print(f"  foot scrub {scrub(coxa, d):.1f} mm per sweep")
    print(f"  every coxa turns the SAME way -- a turn has no left/right "
          f"mirror\n")

    if args.dry_run:
        print(f"  {'step':>5} {'stance':>7} {'coxa -> ':>10} {'cumulative':>11}")
        total = 0.0
        for i in range(1, n_steps + 1):
            stance = "B" if i % 2 else "A"
            total += per_step
            print(f"  {i:>5} {stance:>7}   {-coxa:+.1f} -> {coxa:+.1f}°"
                  f" {total:>10.1f}°")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    joints, servos = {}, []
    try:
        joints = {dg: find_joints(packet, ports, legs, dg, quiet=True)
                  for dg, _ in JOINTS}
        servos = [sp for dg, _ in JOINTS for sp in joints[dg].values()]
        legs = [l for l in legs if all(l in joints[dg] for dg, _ in JOINTS)]
        if not legs:
            print("No leg has all three joints answering.")
            return 1
        in_a = [l for l in legs if l in TRIPOD_A]
        in_b = [l for l in legs if l in TRIPOD_B]
        print(f"  tripod A: {in_a}   tripod B: {in_b}")
        if not in_a or not in_b:
            print("  ⚠ one tripod is empty -- it cannot alternate, so it "
                  "cannot turn. Stopping.")
            return 1

        limits = {}
        for leg in legs:
            for dg, _ in JOINTS:
                sid, port = joints[dg][leg]
                limits[(leg, dg)] = (
                    read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                    read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))

        for sid, port in servos:
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

        if args.pause and not confirm(
                f"Standing, clear of obstacles, and ready to turn "
                f"{args.degrees}° clockwise?"):
            print("\n  Stopped.")
            return 1

        def send(frame, tag):
            group, targets = {}, {}
            for leg, pose in frame.items():
                counts = pose_counts(pose)
                for dg, _ in JOINTS:
                    group[(leg, dg)] = joints[dg][leg]
                    targets[(leg, dg)] = clamp(counts[dg], limits[(leg, dg)])
            results = move_group(packet, group, targets, LOADED_TOLERANCE)
            bad = [k for k, (p, to) in results.items()
                   if to or abs(p - targets[k]) > LOADED_TOLERANCE]
            return bad

        # Start with every coxa at -coxa, so the first stance sweep is a
        # full one rather than a half.
        print("\n  -> set up")
        send({l: dict(base, coxa=-coxa) for l in legs}, "setup")

        trouble = {}
        total = 0.0
        for i in range(1, n_steps + 1):
            stance = TRIPOD_B if i % 2 else TRIPOD_A
            swing = TRIPOD_A if i % 2 else TRIPOD_B
            tag = "B" if i % 2 else "A"
            print(f"  step {i}/{n_steps} (stance {tag}):", end="", flush=True)

            # lift the swing tripod
            frame = {l: (dict(base, coxa=-coxa, femur=base["femur"] + args.lift)
                         if l in swing else dict(base, coxa=-coxa))
                     for l in legs}
            for k in send(frame, "lift"):
                trouble.setdefault(k[0], set()).add("lift")
            print(" lift", end="", flush=True)

            # stance sweeps -coxa -> +coxa, pushing the body round;
            # the lifted tripod goes the other way, resetting itself
            frame = {}
            for l in legs:
                if l in stance:
                    frame[l] = dict(base, coxa=+coxa)
                else:
                    frame[l] = dict(base, coxa=-coxa,
                                    femur=base["femur"] + args.lift)
            for k in send(frame, "sweep"):
                trouble.setdefault(k[0], set()).add("sweep")
            print(" sweep", end="", flush=True)

            # plant the swing tripod at -coxa, ready to be stance next
            frame = {l: dict(base, coxa=(+coxa if l in stance else -coxa))
                     for l in legs}
            for k in send(frame, "plant"):
                trouble.setdefault(k[0], set()).add("plant")
            total += per_step
            print(f" plant   ({total:.1f}° of {args.degrees}°)")

        print("\n  -> back to neutral stance")
        send({l: base for l in legs}, "finish")

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): short on "
                      f"{', '.join(sorted(trouble[leg]))}")
        print(f"\n  Turned {total:.1f}° clockwise in {n_steps} step(s).")
        print(f"  Expect some drift: the feet scrub {scrub(coxa, d):.1f} mm "
              f"per sweep, and scrub is slip.")
        return 1 if trouble else 0

    except KeyboardInterrupt:
        print("\n\n  Interrupted -- returning to neutral stance.")
        try:
            send({l: base for l in legs}, "abort")
        except (BusError, NameError):
            pass
        print("  Torque left ON.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
