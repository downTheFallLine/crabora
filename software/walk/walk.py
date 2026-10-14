#!/usr/bin/env python3
"""
walk -- translate the body across the floor, feet planted.

THE IDEA IS TURN.PY'S, WITH TRANSLATION IN PLACE OF ROTATION.

turn.py established the thing that matters: a stance foot must stay on
the same patch of floor while the body moves. Swinging the coxa and
hoping is what lost two thirds of the first turn -- the foot slid
radially, and a foot that is sliding radially has no grip to push with
either. Measured: 90 degrees commanded gave 30-45.

So the same discipline here. At every interpolated point the stance legs
are re-solved from where their feet already are, and the body is what
moves. The only change is what the body does between solves:

    turn:  the foot appears to rotate about the body centre
    walk:  the foot appears to translate, opposite the heading

Both are "foot fixed in the world, re-solve the leg". The code below is
deliberately the same shape as turn.py's so the two read alike.

A hexagonal robot with radial legs has no front in the mechanical sense,
so this walks in any direction without turning first -- heading is just
a vector. Forward (the hexagon point between legs 1 and 6) is 0.

NOT YET RUN ON HARDWARE. The geometry is checked in --dry-run, which
reports reach and coxa angle at the extremes and refuses a stride the
legs cannot hold. That is not the same as having walked.

Usage:
  python3 walk.py 300 --dry-run        # 300 mm forward, geometry only
  python3 walk.py 300                  # 300 mm forward
  python3 walk.py 200 --heading 180    # backwards
  python3 walk.py 200 --heading 90     # sideways, left
  python3 walk.py 300 --stride 40 --height 90

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "stand"))

from gait_lib import (  # noqa: E402
    require_bus_free,
    ALL_LEGS, BODY_TO_COXA_AXIS, COXA_JOINT_DIGIT, COXA_RADIAL,
    COXA_VERTICAL, FEMUR_JOINT_DIGIT, LEG_BEARING, LEG_NAME,
    TIBIA_JOINT_DIGIT, TRIPOD_A, TRIPOD_B,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read2,
    write1, write2,
)
from pose import (  # noqa: E402
    Unreachable, leg_fk, leg_ik, pose_stand, stand_height_range,
)
from gait_lib import (  # noqa: E402
    ADDR_PRESENT_POSITION, ground_clearance, tripod_gap,
)

# Optional: the walk works without it, but the joint angles are
# body-relative and cannot see the body move. This is the only sensor
# that can.
try:
    from imu_lib import Imu, attitude, tilt_between
except Exception:                                      # noqa: BLE001
    Imu = None

    def attitude(_imu, seconds=0.6):
        return None

    def tilt_between(_a, _b):
        return None
from stand import LOADED_TOLERANCE, clamp, move_group, pose_counts  # noqa: E402

DEFAULT_STRIDE = 50.0    # mm of body travel per step. Bigger covers
                         # ground faster and reaches further out, which
                         # loads the femur harder at the extremes.
# 100, not 80. The femur caps hang 21 mm below the deck and the body
# drops another ~13 mm when a tripod lifts, so 80 leaves about 46 mm of
# real gap under the lowest hardware and 67 at 100. Measured on hardware
# 2026-10-14: standing at 94 mm gave 73 mm under the caps and a clean
# tripod lift, where 75 mm did not.
DEFAULT_HEIGHT = 100.0
DEFAULT_SPEED = 300
# Femur degrees added to lift the swing tripod. Was 25, which computes
# to a 50 mm minimum through the swing -- ample on paper, and the feet
# were observed scraping anyway. The difference is sag: a stance leg
# being commanded to move under full load holds far worse than one
# standing still, and none of it reaches the encoders. 35 buys 24 mm
# more for nothing, since the femur has travel to spare here.
DEFAULT_LIFT = 35.0
DEFAULT_SUBSTEPS = 5
DEFAULT_COXA_LIMIT = 50.0

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# WALK GEOMETRY
# =============================================================
def planted_radius(base):
    """Distance from the BODY CENTRE to a planted foot, legs neutral."""
    out, _down = leg_fk(base["femur"], base["tibia"])
    return BODY_TO_COXA_AXIS + COXA_RADIAL + out


def stance_pose(leg, s, height, r_foot, heading=0.0):
    """Pose holding this leg's foot planted while the body advances `s`.

    `s` is how far the body has travelled (mm) along `heading` since the
    foot was planted. The foot is fixed in the world, so in the body
    frame it slides `s` the other way, and the leg has to reach to hold
    it there.

    Raises Unreachable if the leg cannot stretch that far.

    Same approximation as turn.py: COXA_TANGENT (21 mm) is ignored. It
    shifts the coxa angle by under 2 degrees at these reaches, well
    inside the slip this is trying to eliminate.
    """
    phi = -math.radians(LEG_BEARING[leg])
    psi = -math.radians(heading)      # body frame is +X fwd, +Y left;
                                      # bearings run clockwise, hence -

    # Foot where it was planted, in the body frame at s = 0 ...
    fx = r_foot * math.cos(phi)
    fy = r_foot * math.sin(phi)
    # ... then the body moves along the heading, so the foot appears to
    # move the opposite way by the same amount. This is the whole
    # difference from turn.py.
    fx -= s * math.cos(psi)
    fy -= s * math.sin(psi)

    cx = BODY_TO_COXA_AXIS * math.cos(phi)
    cy = BODY_TO_COXA_AXIS * math.sin(phi)
    vx, vy = fx - cx, fy - cy
    reach = math.hypot(vx, vy)
    coxa = math.degrees(math.atan2(vy, vx) - phi)
    femur, tibia = leg_ik(out=reach - COXA_RADIAL,
                          down=height + COXA_VERTICAL)
    return {"coxa": coxa, "femur": femur, "tibia": tibia}


def _normalise(deg):
    """Wrap to (-180, 180] so a coxa angle reads sensibly."""
    return (deg + 180.0) % 360.0 - 180.0


def max_half_stride(height, r_foot, coxa_limit, heading=0.0,
                    legs=ALL_LEGS, limit=200.0):
    """Largest half-stride (mm) every leg can hold at both extremes.

    Found by stepping outward until some leg either runs out of reach or
    needs more coxa than allowed. Done per leg because a heading that is
    easy for the beam legs can be hard for the corner ones -- walking
    sideways is not the same problem as walking forward.
    """
    best = 0.0
    step = 1.0
    s = step
    while s <= limit:
        for leg in legs:
            for sign in (-1.0, +1.0):
                try:
                    p = stance_pose(leg, sign * s, height, r_foot, heading)
                except Unreachable:
                    return best
                if abs(_normalise(p["coxa"])) > coxa_limit:
                    return best
        best = s
        s += step
    return best


def plan(distance, stride, height, foot_out, coxa_limit, heading, legs):
    """Work out the step schedule, or say why it cannot be done."""
    base = pose_stand(height, foot_out)
    r_foot = planted_radius(base)
    cap = max_half_stride(height, r_foot, coxa_limit, heading, legs)
    if cap <= 0:
        raise Unreachable(
            f"no usable stride at {height:g} mm with the feet "
            f"{r_foot:.0f} mm out -- the legs cannot hold a planted foot "
            f"through any body movement on heading {heading:g}")

    per_step_max = min(stride, 2 * cap)
    n_steps = max(1, math.ceil(distance / per_step_max))
    per_step = distance / n_steps
    half = per_step / 2.0
    if half > cap:
        raise Unreachable(f"half-stride {half:.0f} mm exceeds the "
                          f"{cap:.0f} mm the legs can hold")
    return {"base": base, "r_foot": r_foot, "cap": cap, "n_steps": n_steps,
            "per_step": per_step, "half": half}


def measure_stance(packet, joints, legs):
    """What height is the body ACTUALLY at, from the present angles?

    walk.py used to take --height on faith. If the robot is not up at
    that height -- never stood, or sagged under its own weight because
    the supply cannot hold it -- then the swing legs lift into a body
    that is too low and the feet scuff. From the outside that looks like
    a gait with no lift in it, which sends you hunting in the wrong
    place. Measuring costs one pass over the bus.

    Returns {leg: height_mm}, the deck clearance each leg implies.
    """
    from gait_lib import ADDR_PRESENT_POSITION, CENTER_POSITION, delta_degrees
    out = {}
    for leg in legs:
        try:
            pos = {}
            for dg, name in JOINTS:
                sid, port = joints[dg][leg]
                pos[name] = read2(packet, port, sid, ADDR_PRESENT_POSITION,
                                  "position")
        except (BusError, KeyError):
            continue
        # Counts -> pose degrees, matching pose_counts() in stand.py.
        # delta_degrees, NOT to_degrees: these are differences from
        # centre, and to_degrees would subtract centre a second time.
        femur = -delta_degrees(pos["femur"] - CENTER_POSITION)
        tibia = -delta_degrees(pos["tibia"] - CENTER_POSITION)
        _out, down = leg_fk(femur, tibia)
        out[leg] = down - COXA_VERTICAL
    return out


# =============================================================
# THE WALK
# =============================================================
def swing_clearance(packet, joints, swing, stance):
    """How far the swing feet are off the floor, measured, mid-gait.

    test_07 proved a tripod lifts 64 mm when the robot is standing still.
    That is not the same as it lifting during a walk, where the stance
    legs are moving under load at the same time. This is the same
    measurement taken in the real context, which is the only place the
    dragging actually happens.
    """
    from gait_lib import CENTER_POSITION, delta_degrees
    def pose_of(leg):
        out = {}
        for dg, name in JOINTS:
            sid, port = joints[dg][leg]
            out[name] = read2(packet, port, sid, ADDR_PRESENT_POSITION,
                              "position")
        return (-delta_degrees(out["femur"] - CENTER_POSITION),
                -delta_degrees(out["tibia"] - CENTER_POSITION))

    floors = []
    for leg in stance:
        f, t = pose_of(leg)
        floors.append(leg_fk(f, t)[1] - COXA_VERTICAL)
    if not floors:
        return {}
    body = sum(floors) / len(floors)
    out = {}
    for leg in swing:
        f, t = pose_of(leg)
        out[leg] = body - (leg_fk(f, t)[1] - COXA_VERTICAL)
    return out


def run_walk(packet, joints, legs, limits, base, *, half, per_step, n_steps,
             height, r_foot, heading, lift, speed, substeps,
             should_cancel=None, log=print, instrument=False, imu=None):
    """Setup, alternating tripods, then back to a neutral stance.

    Structured exactly like turn.run_turn, including the cancellation
    poll: a walk is many seconds long, and without it a stop request
    would have to wait for the whole thing.

    Returns {"travelled", "steps_done", "trouble", "cancelled"}.
    """
    for dg, _ in JOINTS:
        for leg in legs:
            sid, port = joints[dg][leg]
            write2(packet, port, sid, ADDR_GOAL_SPEED, speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

    def send(frame):
        group, targets = {}, {}
        for leg, pose in frame.items():
            counts = pose_counts(pose)
            for dg, _ in JOINTS:
                group[(leg, dg)] = joints[dg][leg]
                targets[(leg, dg)] = clamp(counts[dg], limits[(leg, dg)])
        results = move_group(packet, group, targets, LOADED_TOLERANCE)
        return [k for k, (p, to) in results.items()
                if to or abs(p - targets[k]) > LOADED_TOLERANCE]

    # Every leg starts holding its foot with the body pre-shifted back by
    # half a step, so the first sweep is a full one.
    log("\n  -> set up")
    start = {l: stance_pose(l, -half, height, r_foot, heading) for l in legs}
    send(start)

    att0 = attitude(imu) if instrument else None
    if att0:
        log(f"      level before walking: roll {att0['roll']:+.2f}deg, "
            f"pitch {att0['pitch']:+.2f}deg")
        if imu is not None:
            # The robot is standing still right here, so this is the one
            # moment the heading can be honestly zeroed.
            imu.mark_stationary()
            imu.heading_deg = 0.0
    worst_tilt = 0.0

    trouble = {}
    travelled = 0.0
    for i in range(1, n_steps + 1):
        if should_cancel is not None and should_cancel():
            return {"travelled": travelled, "steps_done": i - 1,
                    "trouble": trouble, "cancelled": True}
        stance = TRIPOD_B if i % 2 else TRIPOD_A
        swing = TRIPOD_A if i % 2 else TRIPOD_B
        log(f"  step {i}/{n_steps} (stance {'B' if i % 2 else 'A'}):",
            end="", flush=True)

        # LIFT: femur only. Moving a coxa here would drag a foot that is
        # still on the floor and pull the body back.
        frame = {l: (dict(start[l], femur=start[l]["femur"] + lift)
                     if l in swing else dict(start[l]))
                 for l in legs}
        for k in send(frame):
            trouble.setdefault(k[0], set()).add("lift")
        log(" lift", end="", flush=True)
        if instrument:
            clr = swing_clearance(packet, joints,
                                  [l for l in legs if l in swing],
                                  [l for l in legs if l in stance])
            log("\n      after LIFT  clearance: " +
                ", ".join(f"leg {l} {c:+.0f}mm" for l, c in sorted(clr.items())),
                end="", flush=True)
            t = tilt_between(att0, attitude(imu))
            if t:
                worst_tilt = max(worst_tilt, t[0])
                log(f"   tilt {t[0]:.2f}deg", end="", flush=True)

        # SWEEP: walk the body along in interpolated points. At each one
        # the stance legs are re-solved to hold their feet on the SAME
        # patch of floor -- that is the point, and why there is no scrub.
        reset = {l: stance_pose(l, -half, height, r_foot, heading)
                 for l in swing}
        for j in range(1, substeps + 1):
            frac = j / substeps
            s = -half + (2 * half) * frac
            frame = {}
            for l in legs:
                if l in stance:
                    frame[l] = stance_pose(l, s, height, r_foot, heading)
                else:
                    a, b = start[l], reset[l]
                    frame[l] = {k: a[k] + (b[k] - a[k]) * frac
                                for k in ("coxa", "femur", "tibia")}
                    frame[l]["femur"] += lift
            for k in send(frame):
                trouble.setdefault(k[0], set()).add("sweep")
        log(" sweep", end="", flush=True)

        # PLANT: femur down, everything else held.
        landed = {l: (stance_pose(l, +half, height, r_foot, heading)
                      if l in stance else reset[l])
                  for l in legs}
        if instrument:
            clr = swing_clearance(packet, joints,
                                  [l for l in legs if l in swing],
                                  [l for l in legs if l in stance])
            log("\n      after SWEEP clearance: " +
                ", ".join(f"leg {l} {c:+.0f}mm" for l, c in sorted(clr.items())),
                end="", flush=True)
            t = tilt_between(att0, attitude(imu))
            if t:
                worst_tilt = max(worst_tilt, t[0])
                log(f"   tilt {t[0]:.2f}deg", end="", flush=True)
        for k in send(landed):
            trouble.setdefault(k[0], set()).add("plant")
        travelled += per_step
        log(f" plant   ({travelled:.0f} mm of {per_step * n_steps:.0f})")

        start = landed

    log("\n  -> back to neutral stance")
    send({l: base for l in legs})

    heading = None
    if instrument and imu is not None:
        att1 = attitude(imu)
        heading = imu.heading_deg
        t = tilt_between(att0, att1)
        log(f"\n  body: worst tilt {worst_tilt:.2f}deg during the walk, "
            f"{t[0]:.2f}deg at the end" if t else "")
        log(f"  heading changed {heading:+.1f}deg over the walk "
            f"(straight line wants 0)")
        if abs(heading) > 5.0:
            log(f"  -> it veered. On a straight walk that is asymmetric "
                f"slip,")
            log(f"     one side gripping better than the other.")

    return {"travelled": travelled, "steps_done": n_steps,
            "trouble": trouble, "cancelled": False,
            "worst_tilt": worst_tilt if instrument else None,
            "heading": heading}


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="Walk the robot in a straight line, feet planted.")
    ap.add_argument("distance", type=float,
                    help="how far to travel, in mm")
    ap.add_argument("--heading", type=float, default=0.0,
                    help="direction in degrees: 0 forward, 180 back, "
                         "90 left, 270 right (default 0)")
    ap.add_argument("--stride", type=float, default=DEFAULT_STRIDE,
                    help=f"mm of body travel per step "
                         f"(default {DEFAULT_STRIDE:g})")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT,
                    help=f"deck clearance in mm (default {DEFAULT_HEIGHT:g})")
    ap.add_argument("--foot-out", type=float, default=120.0,
                    help="how far outboard the feet are planted (mm)")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT,
                    help=f"femur degrees to lift the swing tripod "
                         f"(default {DEFAULT_LIFT:g})")
    ap.add_argument("--substeps", type=int, default=DEFAULT_SUBSTEPS,
                    help="interpolated points per sweep")
    ap.add_argument("--coxa-limit", type=float, default=DEFAULT_COXA_LIMIT,
                    help="largest coxa angle to use, degrees")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="do not ask before moving")
    ap.add_argument("--instrument", action="store_true",
                    help="measure swing-foot clearance and body tilt "
                         "at each phase")
    ap.add_argument("--no-imu", dest="use_imu", action="store_false",
                    help="skip the body-attitude measurement")
    ap.add_argument("--force", action="store_true",
                    help="walk even if the measured stance is too low")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and the geometry, touch nothing")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = sorted(set(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("legs must be 1-6")
    if args.distance <= 0:
        ap.error("distance must be positive")
    if not 0 < args.stride <= 150:
        ap.error("--stride must be 0-150 mm")
    if not 0 <= args.lift <= 60:
        ap.error("--lift must be 0-60 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    if args.substeps < 1:
        ap.error("--substeps must be at least 1")

    try:
        p = plan(args.distance, args.stride, args.height, args.foot_out,
                 args.coxa_limit, args.heading, legs)
    except Unreachable as e:
        print(f"Cannot walk like that: {e}")
        try:
            lo, hi = stand_height_range(args.foot_out)
            print(f"  Reachable stand heights at foot_out "
                  f"{args.foot_out:g}: {lo:.0f}..{hi:.0f} mm")
        except Unreachable:
            pass
        return 1

    name = {0: "FORWARD", 180: "BACKWARD",
            90: "LEFT", 270: "RIGHT"}.get(args.heading % 360,
                                          f"on heading {args.heading:g}°")
    print(f"CRABORA walk -- {args.distance:g} mm {name}")
    gap = ground_clearance(args.height)
    walking_gap = tripod_gap(args.height)
    print(f"  standing at {args.height:g} mm deck (model) = {gap:.0f} mm under "
          f"the femur caps,")
    print(f"  dropping to about {walking_gap:.0f} mm while a tripod is lifted;")
    print(f"  feet planted {p['r_foot']:.0f} mm from the body centre")
    if walking_gap < 35:
        print(f"\n  ⚠ ONLY {walking_gap:.0f} mm UNDER THE CAPS WHILE WALKING.")
        print(f"    The femur caps are the lowest thing on the robot and the")
        print(f"    kinematics does not model them, so the feet can clear")
        print(f"    perfectly while the chassis scrapes -- which reads as")
        print(f"    'it drags its legs'. Try --height 100 for "
              f"{tripod_gap(100):.0f} mm.")
    print(f"  {p['n_steps']} step(s) x {p['per_step']:.0f} mm = "
          f"{args.distance:g} mm")
    print(f"  each sweep moves the body +/-{p['half']:.0f} mm about the "
          f"planted feet, in {args.substeps} interpolated points")
    print(f"  the legs can hold +/-{p['cap']:.0f} mm on this heading "
          f"(coxa limit +/-{args.coxa_limit:g}°)")
    print(f"  feet are held PLANTED throughout -- no scrub, so no forced "
          f"slip\n")

    # Per-leg extremes: the corner legs and the beam legs do very
    # different work, and which is worst depends on the heading.
    print(f"  {'leg':>4} {'name':<12} {'coxa at -half':>14} "
          f"{'coxa at +half':>14} {'worst':>8}")
    worst = 0.0
    for leg in legs:
        lo = stance_pose(leg, -p["half"], args.height, p["r_foot"],
                         args.heading)
        hi = stance_pose(leg, +p["half"], args.height, p["r_foot"],
                         args.heading)
        span = max(abs(_normalise(lo["coxa"])), abs(_normalise(hi["coxa"])))
        worst = max(worst, span)
        print(f"  {leg:>4} {LEG_NAME[leg]:<12} "
              f"{_normalise(lo['coxa']):>+13.1f}° "
              f"{_normalise(hi['coxa']):>+13.1f}° {span:>7.1f}°")
    print(f"\n  worst coxa angle {worst:.1f}° of {args.coxa_limit:g}° "
          f"allowed")

    if args.dry_run:
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    if require_bus_free(getattr(args, 'force', False)):
        return 1

    imu = None
    if args.instrument and args.use_imu and Imu is not None:
        try:
            imu = Imu().open()
            print("  IMU: open")
        except Exception as e:                          # noqa: BLE001
            print(f"  IMU: unavailable ({type(e).__name__}: {e}) -- "
                  f"continuing without it")

    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    try:
        joints = {dg: find_joints(packet, ports, legs, dg, quiet=True)
                  for dg, _ in JOINTS}
        legs = [l for l in legs if all(l in joints[dg] for dg, _ in JOINTS)]
        if not legs:
            print("No leg has all three joints answering.")
            return 1
        in_a = [l for l in legs if l in TRIPOD_A]
        in_b = [l for l in legs if l in TRIPOD_B]
        print(f"  tripod A: {in_a}   tripod B: {in_b}")
        if not in_a or not in_b:
            print("  ⚠ one tripod is empty -- it cannot alternate, so it "
                  "cannot walk. Stopping.")
            return 1

        limits = {}
        for leg in legs:
            for dg, _ in JOINTS:
                sid, port = joints[dg][leg]
                limits[(leg, dg)] = (
                    read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                    read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))

        measured = measure_stance(packet, joints, legs)
        if measured:
            lo = min(measured.values())
            hi = max(measured.values())
            print(f"\n  measured stance: {lo:.0f}..{hi:.0f} mm deck "
                  f"(commanded {args.height:g}), i.e. "
                  f"{ground_clearance(lo):.0f}..{ground_clearance(hi):.0f} mm "
                  f"of real gap under the caps")
            for leg in sorted(measured):
                if abs(measured[leg] - args.height) > 25.0:
                    print(f"    leg {leg} ({LEG_NAME[leg]}): "
                          f"{measured[leg]:.0f} mm")
            if lo < args.height - 25.0:
                print(f"\n  ⚠ THE BODY IS {args.height - lo:.0f} mm LOWER "
                      f"THAN THE GAIT ASSUMES.")
                print(f"    The swing legs lift about 50 mm, so at this "
                      f"height the feet will")
                print(f"    scuff instead of clearing -- which looks exactly "
                      f"like a gait with no")
                print(f"    lift in it. Stand first, or walk at "
                      f"--height {lo:.0f}.")
                if not args.force:
                    print("\n  Stopping. Pass --force to walk anyway.")
                    return 1

        if args.pause and not confirm(
                f"Standing, clear of obstacles, and ready to walk "
                f"{args.distance:g} mm {name.lower()}?"):
            print("\n  Stopped.")
            return 1

        outcome = run_walk(
            packet, joints, legs, limits, p["base"],
            half=p["half"], per_step=p["per_step"], n_steps=p["n_steps"],
            height=args.height, r_foot=p["r_foot"], heading=args.heading,
            lift=args.lift, speed=args.speed, substeps=args.substeps,
            instrument=args.instrument, imu=imu)

        if outcome["trouble"]:
            print()
            for leg in sorted(outcome["trouble"]):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): short on "
                      f"{', '.join(sorted(outcome['trouble'][leg]))}")
        print(f"\n  Commanded {outcome['travelled']:.0f} mm in "
              f"{outcome['steps_done']} step(s).")
        print(f"  Feet were held planted throughout, so any shortfall is "
              f"real slip")
        print(f"  rather than forced scrub. Measure it before reaching for "
              f"a bigger stride.")
        return 1 if outcome["trouble"] else 0

    except KeyboardInterrupt:
        # Torque stays ON deliberately: the body is up on six legs, and
        # releasing here drops it. Stopping where it stands is the safer
        # failure. Use utils/all_limp.py to let it down on purpose.
        print("\n\n  Interrupted -- stopped where it stands, torque ON.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        if imu is not None:
            try:
                imu.close()
            except Exception:                           # noqa: BLE001
                pass
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
