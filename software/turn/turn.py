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
    warn_if_service_running,
    ALL_LEGS, BODY_TO_COXA_AXIS, COXA_JOINT_DIGIT, COXA_RADIAL,
    COXA_VERTICAL, FEMUR_JOINT_DIGIT, LEG_BEARING, LEG_NAME,
    TIBIA_JOINT_DIGIT, TRIPOD_A, TRIPOD_B,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read2, to_counts,
    write1, write2,
)
from pose import (  # noqa: E402
    Unreachable, leg_fk, leg_ik, pose_stand, stand_height_range,
)
from stand import LOADED_TOLERANCE, clamp, move_group, pose_counts  # noqa: E402

TURNS = (90, 180, 270, 360)
DEFAULT_STEP = 30.0      # BODY degrees per step; the legs can manage
                         # more, but a bigger step reaches further
                         # round and loads the femur harder
DEFAULT_HEIGHT = 80.0
DEFAULT_SPEED = 300
DEFAULT_LIFT = 25.0      # femur degrees added to lift the swing tripod

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# TURN GEOMETRY
# =============================================================
def planted_radius(base):
    """Distance from the BODY CENTRE to a planted foot, legs neutral."""
    out, _down = leg_fk(base["femur"], base["tibia"])
    return BODY_TO_COXA_AXIS + COXA_RADIAL + out


def stance_pose(leg, rho, height, r_foot):
    """Pose that keeps this leg's foot planted while the body rotates.

    `rho` is how far the body has turned (degrees clockwise) from the
    point where the foot was planted. The foot is fixed in the world, so
    in the rotating body frame it swings the other way; the leg has to
    reach further around to hold it there.

    This is what makes the turn work. Holding femur and tibia fixed and
    swinging the coxa alone forces the foot to slide several millimetres
    radially on every sweep -- and a foot that is sliding radially has
    no tangential grip either, so most of the push is lost. Measured:
    90 degrees commanded gave 30-45.

    Raises Unreachable if the leg cannot stretch that far round.

    Approximation: COXA_TANGENT (21 mm) is ignored here. It shifts the
    coxa angle by under 2 degrees at these reaches, well inside the slip
    this is trying to eliminate.
    """
    phi = -math.radians(LEG_BEARING[leg])
    p = math.radians(rho)
    # Foot fixed in the world => it appears to rotate the other way in
    # the body frame.
    fx = r_foot * math.cos(phi + p)
    fy = r_foot * math.sin(phi + p)
    cx = BODY_TO_COXA_AXIS * math.cos(phi)
    cy = BODY_TO_COXA_AXIS * math.sin(phi)
    vx, vy = fx - cx, fy - cy
    reach = math.hypot(vx, vy)
    coxa = math.degrees(math.atan2(vy, vx) - phi)
    femur, tibia = leg_ik(out=reach - COXA_RADIAL,
                          down=height + COXA_VERTICAL)
    return {"coxa": coxa, "femur": femur, "tibia": tibia}


def max_rotation(height, r_foot, coxa_limit):
    """Largest body rotation per half-sweep the legs can actually hold,
    limited by coxa travel and by the leg running out of reach."""
    lo, hi = 0.0, 60.0
    for _ in range(60):
        mid = (lo + hi) / 2
        try:
            p = stance_pose(1, mid, height, r_foot)
            ok = abs(p["coxa"]) <= coxa_limit
        except Unreachable:
            ok = False
        if ok:
            lo = mid
        else:
            hi = mid
    return lo


# =============================================================
# MAIN
# =============================================================
def run_turn(packet, joints, legs, limits, base, *, half, per_step, n_steps,
             height, r_foot, lift, speed, substeps, degrees,
             should_cancel=None, log=print):
    """The turn itself: setup, alternating tripods, back to neutral.

    Lifted out of main() so the agent layer runs this exact sequence
    rather than a second copy of it. main() still owns argument handling
    and the reporting around it.

    should_cancel() is polled once per step. A turn is many seconds long,
    so without it a stop request would have to wait for the whole thing.
    Returns {"total", "steps_done", "trouble", "cancelled"}.
    """
    # Torque and speed live here, not in the caller, so the CLI and the
    # agent start from an identical servo state.
    for dg, _ in JOINTS:
        for leg in legs:
            sid, port = joints[dg][leg]
            write2(packet, port, sid, ADDR_GOAL_SPEED, speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

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

    # Every leg starts holding its foot with the body pre-rotated
    # back by half a step, so the first sweep is a full one.
    log("\n  -> set up")
    start = {l: stance_pose(l, -half, height, r_foot) for l in legs}
    send(start, "setup")

    trouble = {}
    total = 0.0
    for i in range(1, n_steps + 1):
        stance = TRIPOD_B if i % 2 else TRIPOD_A
        swing = TRIPOD_A if i % 2 else TRIPOD_B
        tag = "B" if i % 2 else "A"
        log(f"  step {i}/{n_steps} (stance {tag}):", end="", flush=True)
        if should_cancel is not None and should_cancel():
            return {"total": total, "steps_done": i - 1, "trouble": trouble,
                    "cancelled": True}

        # LIFT: femur only. Moving a coxa here would drag a foot that
        # is still on the floor and pull the body back round.
        frame = {l: (dict(start[l], femur=start[l]["femur"] + lift)
                     if l in swing else dict(start[l]))
                 for l in legs}
        for k in send(frame, "lift"):
            trouble.setdefault(k[0], set()).add("lift")
        log(" lift", end="", flush=True)

        # SWEEP: walk the body round in interpolated points. At each
        # one the stance legs are re-solved to hold their feet on the
        # SAME patch of floor -- that is the whole point, and why
        # there is no scrub to slip on.
        reset = {l: stance_pose(l, -half, height, r_foot)
                 for l in swing}
        for j in range(1, substeps + 1):
            frac = j / substeps
            rho = -half + (2 * half) * frac
            frame = {}
            for l in legs:
                if l in stance:
                    frame[l] = stance_pose(l, rho, height, r_foot)
                else:
                    # swinging: glide to the reset pose, staying up
                    a, b = start[l], reset[l]
                    frame[l] = {k: a[k] + (b[k] - a[k]) * frac
                                for k in ("coxa", "femur", "tibia")}
                    frame[l]["femur"] += lift
            for k in send(frame, "sweep"):
                trouble.setdefault(k[0], set()).add("sweep")
        log(" sweep", end="", flush=True)

        # PLANT: femur down, everything else held.
        landed = {}
        for l in legs:
            landed[l] = (stance_pose(l, +half, height, r_foot)
                         if l in stance else reset[l])
        for k in send(landed, "plant"):
            trouble.setdefault(k[0], set()).add("plant")
        total += per_step
        log(f" plant   ({total:.1f}° of {degrees}°)")

        # The stance legs are now at +half; next step they swing. The
        # legs just planted are at -half and become stance.
        start = landed

    log("\n  -> back to neutral stance")
    send({l: base for l in legs}, "finish")
    return {"total": total, "steps_done": n_steps, "trouble": trouble,
            "cancelled": False}


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA: turn in place, clockwise.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("degrees", type=int, choices=TURNS,
                    help="how far to turn, clockwise")
    ap.add_argument("--step", type=float, default=DEFAULT_STEP,
                    help=f"maximum BODY rotation per step, degrees "
                         f"(default {DEFAULT_STEP:g}); solved down so whole "
                         f"steps hit the target exactly")
    ap.add_argument("--substeps", type=int, default=5,
                    help="interpolation points per sweep (default 5). The "
                         "foot is held planted at each one, so more means a "
                         "truer arc and less slip.")
    ap.add_argument("--coxa-limit", type=float, default=50.0,
                    help="most coxa travel to use, degrees (default 50)")
    ap.add_argument("--efficiency", type=float, default=1.0,
                    help="fraction of the geometric turn actually achieved "
                         "(default 1.0 = no slip). Planted feet scrub and "
                         "slip, so the real turn falls short. Measure: turn "
                         "180, see what you got, pass that/180.")
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
    if not 0 < args.step <= 90:
        ap.error("--step must be between 0 and 90 degrees")
    if not 1 <= args.substeps <= 30:
        ap.error("--substeps must be 1-30")
    if not 0 < args.lift <= 60:
        ap.error("--lift must be between 0 and 60 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    if not 0.1 <= args.efficiency <= 1.0:
        ap.error("--efficiency must be between 0.1 and 1.0")

    try:
        base = pose_stand(args.height)
    except Unreachable as e:
        lo, hi = stand_height_range()
        print(f"Cannot stand at {args.height:g} mm: {e}")
        print(f"Reachable range: {lo:.0f}..{hi:.0f} mm")
        return 1

    r_foot = planted_radius(base)
    cap = max_rotation(args.height, r_foot, args.coxa_limit)
    per_step_max = min(args.step, 2 * cap) * args.efficiency
    n_steps = max(1, math.ceil(args.degrees / per_step_max))
    per_step = args.degrees / n_steps
    half = per_step / 2 / args.efficiency

    print(f"CRABORA turn -- {args.degrees}° CLOCKWISE")
    try:
        ex = stance_pose(1, half, args.height, r_foot)
    except Unreachable as e:
        print(f"  Cannot hold a foot planted through ±{half:.1f}°: {e}")
        return 1
    print(f"  standing at {args.height:g} mm; feet planted {r_foot:.0f} mm "
          f"from the body centre")
    print(f"  {n_steps} step(s) x {per_step:.1f}° = {args.degrees}°")
    print(f"  each sweep rotates the body ±{half:.1f}° about the planted "
          f"feet, in {args.substeps} interpolated points")
    print(f"  at the extreme: coxa {ex['coxa']:+.1f}°, femur "
          f"{ex['femur']:+.1f}°, flex {ex['tibia']:.1f}°  "
          f"(limit ±{args.coxa_limit:g}°, max half-sweep {cap:.1f}°)")
    print(f"  feet are held PLANTED throughout -- no scrub, so no forced "
          f"slip")
    print(f"  every coxa turns the SAME way -- a turn has no left/right "
          f"mirror\n")

    if args.dry_run:
        print(f"\n  {'step':>5} {'stance':>7} {'body':>16} {'coxa sweep':>16}"
              f" {'cumulative':>11}")
        total = 0.0
        lo = stance_pose(1, -half, args.height, r_foot)
        hi = stance_pose(1, +half, args.height, r_foot)
        for i in range(1, n_steps + 1):
            total += per_step
            print(f"  {i:>5} {('B' if i % 2 else 'A'):>7}"
                  f" {-half:+6.1f}..{half:+6.1f}°"
                  f" {lo['coxa']:+7.1f}..{hi['coxa']:+6.1f}°"
                  f" {total:>10.1f}°")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    warn_if_service_running()
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

        if args.pause and not confirm(
                f"Standing, clear of obstacles, and ready to turn "
                f"{args.degrees}° clockwise?"):
            print("\n  Stopped.")
            return 1

        outcome = run_turn(
            packet, joints, legs, limits, base,
            half=half, per_step=per_step, n_steps=n_steps,
            height=args.height, r_foot=r_foot, lift=args.lift,
            speed=args.speed, substeps=args.substeps, degrees=args.degrees)
        trouble = outcome["trouble"]
        total = outcome["total"]

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): short on "
                      f"{', '.join(sorted(trouble[leg]))}")
        print(f"\n  Turned {total:.1f}° clockwise in {n_steps} step(s).")
        print(f"  Feet were held planted throughout. If it still falls "
              f"short, that is")
        print(f"  real slip rather than forced scrub -- measure it and pass "
              f"--efficiency.")
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
