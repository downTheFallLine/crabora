#!/usr/bin/env python3
"""
CRABORA dance -- once it is standing, make it move
====================================================

Assumes the robot is already up. Run `stand/stand.py` first; this goes
to the standing pose, checks it got there, and then dances.

Moves, all built from the same leg IK the stand uses:

  bob      body rises and falls, all six legs together
  twist    every coxa swings the same way at once, so the feet stay put
           and the BODY yaws about its centre
  bounce   tripod A extends while tripod B retracts, tilting the body
           one way then the other. Feet never leave the floor.
  wave     each leg lifts and sets down in turn around the hexagon, like
           a stadium wave. Only ever one foot up, so five stay planted.
  shimmy   tripods swing their coxas in OPPOSITE directions, twisting
           the body rapidly back and forth

Safety, such as it is for a dance:
  - the feet stay planted in every move except `wave`, which lifts
    exactly one leg at a time out of six
  - amplitudes are clamped to the servos' firmware travel limits
  - it refuses to start unless it can reach the standing pose
  - Ctrl-C returns to standing rather than dropping torque, so the
    robot does not collapse mid-move

Usage:
  python3 dance.py --dry-run
  python3 dance.py                      # the whole routine, twice
  python3 dance.py --moves twist wave
  python3 dance.py --reps 4 --speed 1600
  python3 dance.py --height 90 --amp 18

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
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    LEG_NAME, TIBIA_JOINT_DIGIT, TRIPOD_A,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read2, release,
    to_counts, write1, write2,
)
from pose import (  # noqa: E402
    Unreachable, leg_fk, pose_stand, stand_height_range,
)
from stand import (  # noqa: E402
    LOADED_TOLERANCE, clamp, move_group, pose_counts,
)

DEFAULT_HEIGHT = 80.0
DEFAULT_AMP    = 15.0     # degrees, or mm for the height moves
DEFAULT_SPEED  = 1400     # steps/s -- brisk; the stand runs at 200
DEFAULT_REPS   = 2
BEAT = 0.0                # extra pause per step; 0 means as fast as the
                          # servos will go

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# MOVES
# =============================================================
# Each move is a generator of frames. A frame is {leg: pose}, where a
# pose is the usual {"coxa", "femur", "tibia"} in degrees. The base
# standing pose is passed in so every move returns to it cleanly.

def move_bob(base, legs, amp, height):
    """Body rises and falls. Pure height change, feet planted."""
    for h in (height + amp, height, height - amp, height):
        try:
            p = pose_stand(h)
        except Unreachable:
            continue
        yield {leg: p for leg in legs}, f"body {h:.0f} mm"


def move_twist(base, legs, amp, height):
    """Every coxa the same way at once -- the feet are planted, so the
    body yaws about its own centre instead."""
    for a in (+amp, 0.0, -amp, 0.0):
        yield ({leg: dict(base, coxa=a) for leg in legs},
               f"body yaw {a:+.0f}°")


def move_bounce(base, legs, amp, height):
    """Tripod A extends while B retracts, rocking the body. Feet stay
    down -- this is a tilt, not a step."""
    for sign in (+1, -1, +1, -1):
        frame = {}
        for leg in legs:
            h = height + sign * amp * (1 if leg in TRIPOD_A else -1)
            try:
                frame[leg] = pose_stand(h)
            except Unreachable:
                frame[leg] = base
        yield frame, (f"A {height + sign * amp:.0f} mm / "
                      f"B {height - sign * amp:.0f} mm")


def move_wave(base, legs, amp, height):
    """Each leg lifts and sets down in turn around the hexagon. Exactly
    one foot off the floor at a time, so five always carry the body."""
    lift = amp * 1.5
    _o, base_down = leg_fk(base["femur"], base["tibia"])
    _o2, up_down = leg_fk(base["femur"] + lift, base["tibia"])
    rise = base_down - up_down
    for leg in sorted(legs):
        up = dict(base, femur=base["femur"] + lift)
        yield ({l: (up if l == leg else base) for l in legs},
               f"leg {leg} foot {rise:.0f} mm up")
    yield {l: base for l in legs}, "all down"


def move_shimmy(base, legs, amp, height):
    """Tripods swing their coxas opposite ways, twisting the body fast."""
    for sign in (+1, -1, +1, -1, +1, -1):
        yield ({leg: dict(base,
                          coxa=sign * amp * (1 if leg in TRIPOD_A else -1))
                for leg in legs},
               f"tripods ∓{amp:.0f}°")


MOVES = {
    "bob": move_bob,
    "twist": move_twist,
    "bounce": move_bounce,
    "wave": move_wave,
    "shimmy": move_shimmy,
}
ROUTINE = ["bob", "twist", "bounce", "shimmy", "wave"]


# =============================================================
# PLAYBACK
# =============================================================
def play(packet, joints, legs, limits, frame, beat):
    """Send one frame -- every joint of every leg, all at once."""
    group, targets = {}, {}
    for leg, pose in frame.items():
        counts = pose_counts(pose)
        for d, _ in JOINTS:
            if leg in joints[d] and (leg, d) in limits:
                group[(leg, d)] = joints[d][leg]
                targets[(leg, d)] = clamp(counts[d], limits[(leg, d)])
    if not group:
        return {}
    results = move_group(packet, group, targets, LOADED_TOLERANCE)
    if beat:
        time.sleep(beat)
    return {k: abs(results[k][0] - targets[k]) for k in results}


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA dance -- once standing, move to it.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--moves", nargs="+", choices=sorted(MOVES),
                    default=ROUTINE, help="which moves, in order")
    ap.add_argument("--reps", type=int, default=DEFAULT_REPS,
                    help=f"times through the routine (default {DEFAULT_REPS})")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT,
                    help=f"standing deck clearance, mm (default {DEFAULT_HEIGHT:g})")
    ap.add_argument("--amp", type=float, default=DEFAULT_AMP,
                    help=f"move amplitude, degrees or mm (default {DEFAULT_AMP:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"steps/s (default {DEFAULT_SPEED} -- quick)")
    ap.add_argument("--beat", type=float, default=BEAT,
                    help="extra pause per frame, seconds (default 0)")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")
    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 < args.amp <= 40:
        ap.error("--amp must be between 0 and 40")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")

    try:
        base = pose_stand(args.height)
    except Unreachable as e:
        lo, hi = stand_height_range()
        print(f"Cannot stand at {args.height:g} mm: {e}")
        print(f"Reachable range: {lo:.0f}..{hi:.0f} mm")
        return 1

    print("CRABORA dance")
    print(f"  standing at {args.height:g} mm (femur {base['femur']:+.1f}°, "
          f"flex {base['tibia']:.1f}°)")
    print(f"  {' -> '.join(args.moves)}  x{args.reps}, "
          f"amp {args.amp:g}, {args.speed} steps/s\n")

    if args.dry_run:
        for name in args.moves:
            frames = list(MOVES[name](base, legs, args.amp, args.height))
            print(f"  {name:<8} {len(frames)} frames")
            for i, (fr, note) in enumerate(frames, 1):
                uniq = {(round(p['coxa'], 1), round(p['femur'], 1),
                         round(p['tibia'], 1)) for p in fr.values()}
                print(f"      frame {i}: {note}"
                      f"   ({len(uniq)} distinct leg pose(s))")
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
        joints = {d: find_joints(packet, ports, legs, d, quiet=True)
                  for d, _ in JOINTS}
        servos = [sp for d, _ in JOINTS for sp in joints[d].values()]
        if not servos:
            print("No servos answered. Run tests/test_03_ping_servo.py.")
            return 1
        legs = [l for l in legs if all(l in joints[d] for d, _ in JOINTS)]
        if not legs:
            print("No leg has all three joints answering.")
            return 1
        print(f"  dancing on {len(legs)} leg(s): {', '.join(map(str, legs))}")

        limits = {}
        for leg in legs:
            for d, _ in JOINTS:
                sid, port = joints[d][leg]
                limits[(leg, d)] = (
                    read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                    read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))

        for sid, port in servos:
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

        if args.pause and not confirm(
                "Robot standing and clear of obstacles? It will move fast."):
            print("\n  Stopped.")
            return 1

        # Settle into the base pose before anything else, so every move
        # starts from a known place.
        print("\n  -> standing pose")
        sag = play(packet, joints, legs, limits, {l: base for l in legs},
                   args.beat)
        worst = max(sag.values(), default=0)
        if worst > LOADED_TOLERANCE:
            print(f"  Could not reach the standing pose "
                  f"({worst} counts short). Run stand/stand.py first.")
            return 1
        print(f"     settled, worst sag {worst} counts "
              f"({worst * 360 / 4096:.1f}°)")

        t0 = time.monotonic()
        frames = 0
        for rep in range(1, args.reps + 1):
            for name in args.moves:
                print(f"  rep {rep}/{args.reps}  {name}", end="", flush=True)
                for frame, _note in MOVES[name](base, legs, args.amp,
                                                args.height):
                    play(packet, joints, legs, limits, frame, args.beat)
                    frames += 1
                    print(".", end="", flush=True)
                print()

        print("\n  -> back to standing")
        play(packet, joints, legs, limits, {l: base for l in legs}, args.beat)
        elapsed = time.monotonic() - t0
        print(f"\n  {frames} frames in {elapsed:.1f} s "
              f"({frames / elapsed:.1f} frames/s). Torque left ON -- still "
              f"standing.")
        return 0

    except KeyboardInterrupt:
        # Do NOT drop torque: the robot is standing on these legs.
        print("\n\n  Interrupted -- returning to standing pose.")
        try:
            play(packet, joints, legs, limits, {l: base for l in legs}, 0)
        except BusError:
            pass
        print("  Torque left ON. Use utils/go_limp.py once it is supported.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
