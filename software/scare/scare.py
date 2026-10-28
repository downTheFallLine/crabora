#!/usr/bin/env python3
"""
CRABORA scare! -- rear up on four legs and wave the front two
==============================================================

Assumes the robot is already up. Run `stand/stand.py` first.

WHY IT HAS TO STEP FIRST.

With legs 1 and 6 in the air the robot stands on 2, 3, 4 and 5. Legs 2
and 5 are the beam legs, straight out to the sides, so the line between
their feet runs right through the body centre -- the centre of mass
would sit ON the front edge of the support, and the robot would tip onto
its nose. So before anything lifts, legs 2 and 5 step forward
(`--brace`, 60 mm), one at a time with five feet down. That puts the
front edge 60 mm ahead of the centre of mass. --dry-run prints the
margin, and refuses below MIN_MARGIN.

THE SEQUENCE.

  brace    legs 2 then 5 step forward, one at a time
  rear     legs 1 and 6 lift high, claws curled, pointing forward
  menace   they wave, out of phase: coxas swing, femurs bob, tibias snap
  lower    legs 1 and 6 come back down to their standing spots
  settle   legs 5 then 2 step back; it ends in the standing pose

Built from the same pieces as everything else: stand.py's pose_stand,
walk.py's stance_pose for the planted-foot steps, dance.py's play() to
send each frame and wait for it.

SAFETY.

  - every step moves one leg with the other five down
  - supply voltage is read before each wave; a sag ends the menace and
    it puts the legs down (a sag here is the servo buck at its limit)
  - Ctrl-C does not drop torque: it lowers whatever is up and steps back
    to standing, one leg at a time
  - joint goals are clamped to each servo's firmware limits

Usage:
  python3 scare.py --dry-run
  python3 scare.py                  # 4 waves
  python3 scare.py --waves 8 --speed 1800

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "walk"))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "stand"))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "dance"))

from gait_lib import (  # noqa: E402
    require_bus_free,
    ALL_LEGS, FEMUR_JOINT_DIGIT, FORWARD_SIGN, LEG_BEARING,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read1, read2,
    write1, write2,
)
from pose import Unreachable, pose_stand  # noqa: E402
from walk import (  # noqa: E402
    DEFAULT_HEIGHT, measure_stance, planted_radius, stance_pose,
)
from dance import JOINTS, play  # noqa: E402

RAISED = (1, 6)          # the front pair: they wave
BRACED = (2, 5)          # the beam legs: they step forward to take the load
PLANTED = (2, 3, 4, 5)   # what it stands on while rearing

DEFAULT_BRACE = 60.0     # mm legs 2 and 5 step forward
MIN_MARGIN = 35.0        # mm the centre of mass must sit inside the
                         # four-foot support, or it will not rear up
STEP_LIFT = 25.0         # femur degrees a stepping foot lifts
DEFAULT_WAVES = 4
DEFAULT_SPEED = 1500     # steps/s while waving
STEP_SPEED = 500         # steps/s for the brace and settle steps -- these
                         # carry the body, so not quick
DEFAULT_MIN_VOLTS = 6.5  # as scurry: the buck is set to 7.4 V

# The raised pose: femur well up, knee curled, pointing a little forward.
# Angles in the pose convention: femur + is up from level, tibia is flex
# (0 straight), coxa + is counter-clockwise from above.
RAISE_FEMUR = 55.0
RAISE_FLEX = 70.0
RAISE_FORWARD = 20.0     # coxa degrees toward the front point
# The menace: around the raised pose.
WAVE_COXA = 15.0
WAVE_FEMUR = 12.0
SNAP_FLEX = (40.0, 100.0)   # claws open, claws shut

ADDR_PRESENT_VOLTAGE = 62   # 1 byte, 0.1 V


# =============================================================
# GEOMETRY
# =============================================================
def foot_xy(leg, r_foot, forward=0.0):
    """Planted foot in the body frame (X forward, Y left), mm."""
    phi = -math.radians(LEG_BEARING[leg])
    return (r_foot * math.cos(phi) + forward, r_foot * math.sin(phi))


def support_margin(points):
    """How far the body centre sits inside the convex polygon of
    `points`, in mm. Negative means outside: it would tip."""
    pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))
    best = float("inf")
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        # Signed distance from the origin to the edge; positive when the
        # origin is on the inside (left of a counter-clockwise edge).
        ex, ey = x2 - x1, y2 - y1
        best = min(best, (ex * -y1 - ey * -x1) / math.hypot(ex, ey))
    return best


def raised_pose(leg, coxa=0.0, femur=0.0, flex=None):
    """A front leg in the air, offsets from the raised pose. `coxa` is
    FORWARD-relative (+ swings toward the front point on either side),
    so equal and opposite offsets on legs 1 and 6 make a wave rather
    than a clap -- the raw angle is mirrored between the two sides."""
    return {"coxa": FORWARD_SIGN[leg] * (RAISE_FORWARD + coxa),
            "femur": RAISE_FEMUR + femur,
            "tibia": RAISE_FLEX if flex is None else flex}


# =============================================================
# THE PLAN
# =============================================================
def plan(height, foot_out, brace, waves):
    """Every frame of the routine, worked out before anything moves.

    Returns {"base", "braced", "margin", "phases"} where phases is
    [(name, speed, [(frame, note), ...]), ...] and every frame is a full
    six-leg {leg: pose}.
    """
    base = pose_stand(height, foot_out)
    r_foot = planted_radius(base)
    # Heading 180 means "the body moved back", so the planted foot sits
    # `brace` mm further forward in the body frame.
    braced = {leg: stance_pose(leg, brace, height, r_foot, 180.0)
              for leg in BRACED}
    margin = support_margin([foot_xy(l, r_foot, brace if l in BRACED else 0.0)
                             for l in PLANTED])

    cur = {l: dict(base) for l in ALL_LEGS}
    phases = []

    def snap(note):
        return {l: dict(p) for l, p in cur.items()}, note

    def step(leg, target, frames):
        cur[leg] = dict(cur[leg], femur=cur[leg]["femur"] + STEP_LIFT)
        frames.append(snap(f"leg {leg} up"))
        cur[leg] = dict(target, femur=target["femur"] + STEP_LIFT)
        frames.append(snap(f"leg {leg} across"))
        cur[leg] = dict(target)
        frames.append(snap(f"leg {leg} down"))

    frames = []
    for leg in BRACED:
        step(leg, braced[leg], frames)
    phases.append(("brace", STEP_SPEED, frames))

    frames = []
    for leg in RAISED:
        cur[leg] = dict(base, femur=base["femur"] + STEP_LIFT)
    frames.append(snap("front legs off the floor"))
    for leg in RAISED:
        cur[leg] = raised_pose(leg)
    frames.append(snap("reared up"))
    phases.append(("rear", STEP_SPEED, frames))

    frames = []
    for w in range(waves):
        for sign in (+1, -1):
            for leg in RAISED:
                # Out of phase: one swings in while the other swings out.
                s = sign if leg == RAISED[0] else -sign
                cur[leg] = raised_pose(
                    leg, coxa=s * WAVE_COXA, femur=s * WAVE_FEMUR,
                    flex=SNAP_FLEX[0] if s > 0 else SNAP_FLEX[1])
            frames.append(snap(f"wave {w + 1} {'left' if sign > 0 else 'right'}"))
    for leg in RAISED:
        cur[leg] = raised_pose(leg)
    frames.append(snap("hold"))
    phases.append(("menace", None, frames))

    phases.append(("lower", STEP_SPEED, lower_frames(cur, base)))
    phases.append(("settle", STEP_SPEED, settle_frames(cur, base)))
    return {"base": base, "braced": braced, "margin": margin,
            "phases": phases}


def lower_frames(cur, base):
    """Put legs 1 and 6 back on the floor at their standing spots.
    Mutates `cur`."""
    out = []
    up = [l for l in RAISED if cur[l] != base]
    if not up:
        return out
    for leg in up:
        cur[leg] = dict(base, femur=base["femur"] + STEP_LIFT)
    out.append(({l: dict(p) for l, p in cur.items()}, "front legs over their spots"))
    for leg in up:
        cur[leg] = dict(base)
    out.append(({l: dict(p) for l, p in cur.items()}, "front legs down"))
    return out


def settle_frames(cur, base):
    """Step any displaced leg back to standing, one at a time, last
    braced first. Mutates `cur`."""
    out = []
    for leg in sorted(cur, key=lambda l: (l not in BRACED, -l)):
        if cur[leg] == base:
            continue
        cur[leg] = dict(cur[leg], femur=cur[leg]["femur"] + STEP_LIFT)
        out.append(({l: dict(p) for l, p in cur.items()}, f"leg {leg} up"))
        cur[leg] = dict(base, femur=base["femur"] + STEP_LIFT)
        out.append(({l: dict(p) for l, p in cur.items()}, f"leg {leg} back"))
        cur[leg] = dict(base)
        out.append(({l: dict(p) for l, p in cur.items()}, f"leg {leg} down"))
    return out


# =============================================================
# THE RUN
# =============================================================
def set_speed(packet, joints, legs, speed):
    for leg in legs:
        for d, _ in JOINTS:
            sid, port = joints[d][leg]
            write2(packet, port, sid, ADDR_GOAL_SPEED, speed, "speed")


def supply_volts(packet, joints):
    """Lowest of the planted legs' femur voltages."""
    out = []
    for leg in PLANTED:
        sid, port = joints[FEMUR_JOINT_DIGIT][leg]
        out.append(read1(packet, port, sid, ADDR_PRESENT_VOLTAGE,
                         "voltage") / 10.0)
    return min(out)


def run_scare(packet, joints, limits, p, *, speed=DEFAULT_SPEED,
              min_volts=DEFAULT_MIN_VOLTS, should_cancel=None, log=print):
    """Play the plan. On a sag, a cancel or Ctrl-C, lower and settle from
    wherever it got to instead. Returns {"frames", "waves", "min_volts",
    "stopped"}; stopped is None for a clean finish."""
    legs = list(ALL_LEGS)
    base = p["base"]
    cur = {l: dict(base) for l in legs}
    played, waved, stopped, low = 0, 0, None, None

    try:
        for name, phase_speed, frames in p["phases"]:
            if name in ("lower", "settle"):
                break                    # done below, from wherever it is
            log(f"  -> {name}")
            set_speed(packet, joints, legs, phase_speed or speed)
            for frame, note in frames:
                if should_cancel is not None and should_cancel():
                    stopped = "cancelled"
                    break
                if name == "menace" and note.endswith("left"):
                    v = supply_volts(packet, joints)
                    low = v if low is None else min(low, v)
                    if v < min_volts:
                        stopped = f"supply fell to {v:.1f} V (limit {min_volts:g})"
                        break
                    waved += 1
                play(packet, joints, legs, limits, frame, 0)
                cur = frame
                played += 1
            if stopped:
                break
    except KeyboardInterrupt:
        stopped = "interrupted"

    if stopped:
        log(f"\n  STOPPING: {stopped} -- lowering and stepping back")
    if stopped == "cancelled":
        # STOP from the UI drops torque itself; moving now would fight it.
        return {"frames": played, "waves": waved, "min_volts": low,
                "stopped": stopped}
    log("  -> lower, settle")
    set_speed(packet, joints, legs, STEP_SPEED)
    cur = {l: dict(pose) for l, pose in cur.items()}
    for frame, _note in lower_frames(cur, base) + settle_frames(cur, base):
        play(packet, joints, legs, limits, frame, 0)
        played += 1
    return {"frames": played, "waves": waved, "min_volts": low,
            "stopped": stopped}


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="CRABORA scare! -- rear up on four legs and wave two.")
    ap.add_argument("--waves", type=int, default=DEFAULT_WAVES,
                    help=f"how many waves (default {DEFAULT_WAVES})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"steps/s while waving (default {DEFAULT_SPEED})")
    ap.add_argument("--brace", type=float, default=DEFAULT_BRACE,
                    help=f"mm legs 2 and 5 step forward first (default "
                         f"{DEFAULT_BRACE:g})")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT)
    ap.add_argument("--foot-out", type=float, default=120.0)
    ap.add_argument("--min-volts", type=float, default=DEFAULT_MIN_VOLTS)
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="run even if the agentix service holds the bus, "
                         "or the measured stance is low")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    if not 1 <= args.waves <= 20:
        ap.error("--waves must be 1-20")
    if not 100 <= args.speed <= 3000:
        ap.error("--speed must be 100-3000 steps/s")

    try:
        p = plan(args.height, args.foot_out, args.brace, args.waves)
    except Unreachable as e:
        print(f"Cannot scare like that: {e}")
        return 1

    print("CRABORA scare!")
    print(f"  standing at {args.height:g} mm; legs {BRACED} step "
          f"{args.brace:g} mm forward, then legs {RAISED} rear up")
    print(f"  centre of mass {p['margin']:.0f} mm inside the four-foot "
          f"support (needs {MIN_MARGIN:g})")
    for name, sp, frames in p["phases"]:
        print(f"  {name:<7} {len(frames):>2} frames at "
              f"{sp or args.speed} steps/s")
    if p["margin"] < MIN_MARGIN:
        print(f"\n  ⚠ not enough margin -- it would tip. Increase --brace.")
        return 1
    if args.dry_run:
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    if require_bus_free(args.force):
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    try:
        joints = {d: find_joints(packet, ports, list(ALL_LEGS), d, quiet=True)
                  for d, _ in JOINTS}
        missing = [l for l in ALL_LEGS
                   if not all(l in joints[d] for d, _ in JOINTS)]
        if missing:
            print(f"  scare! needs all six legs; {missing} did not answer.")
            return 1
        limits = {}
        for leg in ALL_LEGS:
            for d, _ in JOINTS:
                sid, port = joints[d][leg]
                limits[(leg, d)] = (
                    read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                    read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))
                write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

        measured = measure_stance(packet, joints, list(ALL_LEGS))
        if measured and min(measured.values()) < args.height - 25.0:
            print(f"\n  ⚠ the body is at {min(measured.values()):.0f} mm, "
                  f"expected {args.height:g}. Stand first.")
            if not args.force:
                return 1
        v0 = supply_volts(packet, joints)
        print(f"  supply {v0:.1f} V at rest")
        if v0 < args.min_volts + 0.3:
            print("  ⚠ too close to the low-voltage stop. Charge first.")
            return 1

        if args.pause and not confirm(
                "Standing, clear in front, and ready to scare?"):
            print("\n  Stopped.")
            return 1

        out = run_scare(packet, joints, limits, p, speed=args.speed,
                        min_volts=args.min_volts)
        print(f"\n  {out['waves']} wave(s), {out['frames']} frames"
              + (f", lowest supply {out['min_volts']:.1f} V"
                 if out["min_volts"] else "")
              + ". Torque left ON -- standing.")
        return 1 if out["stopped"] else 0
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
