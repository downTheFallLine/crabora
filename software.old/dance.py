"""
CRABORA: body sway and dance
=============================

Choreographed motion with all 3 feet planted.  No lifting, no balance
problem -- just expressive body movement driven by coxa and femur offsets
from the stand pose.

Moves
-----
  bob      All femurs oscillate together: body rises and falls.
  tilt     One leg extends while the other two retract: body rocks side to side.
  shimmy   Leg 1 coxa forward / legs 2+3 back, then swap -- body wiggles fast.
  spin     All coxas sweep in the same rotational direction: body rotates.
  routine  Full dance: bob -> tilt -> shimmy -> spin -> bow.

Usage
-----
  python dance.py                    # full routine
  python dance.py --move bob
  python dance.py --move tilt
  python dance.py --move shimmy
  python dance.py --move spin
  python dance.py --move routine
  python dance.py --reps 4           # repeat each move more times
  python dance.py --speed 20         # faster (more energetic)
"""

import argparse
import json
import os
import sys
import time

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE,
    MODE_POSITION, MODE_NAMES,
    MAX_POSITION, TYPICAL_MAX_RPM, CENTER_POSITION,
    describe_id, rpm_to_pos_speed,
)

LEG_IDS    = {1: [11, 12, 13], 2: [21, 22, 23], 3: [31, 32, 33]}
POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

COXA  = 0
FEMUR = 1
TIBIA = 2

# How far to move for each effect, in encoder counts.
# All offsets are from the stand pose and are clamped to servo limits.
BOB_COUNTS    = 250   # femur extension for the body-bob (up/down)
TILT_COUNTS   = 300   # femur delta for the body-tilt (one side vs. the other)
SHIMMY_FRAC   = 0.35  # fraction of coxa range to use for the shimmy
SPIN_FRAC     = 0.40  # fraction of coxa range to use for the spin

MOVES = ["bob", "tilt", "shimmy", "spin", "routine"]


# =============================================================================
# Helpers
# =============================================================================
def parse_args():
    p = argparse.ArgumentParser(
        description="CRABORA body sway and dance -- feet planted, body moving.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--move", choices=MOVES, default="routine",
                   help=f"which move to perform (default: routine). "
                        f"Choices: {', '.join(MOVES)}")
    p.add_argument("--reps", type=int, default=3,
                   help="repetitions per move (default: 3)")
    p.add_argument("--speed", type=float, default=15.0,
                   help="movement speed in RPM (default: 15)")
    p.add_argument("--release", action="store_true",
                   help="relax torque after the dance (default: hold)")
    p.add_argument("--no-pause", action="store_true",
                   help="skip the ready confirmation")
    return p.parse_args()


def load_stand():
    if not os.path.exists(POSES_PATH):
        print(f"✗ No poses.json at {POSES_PATH}. Run teach_pose.py first.")
        sys.exit(1)
    with open(POSES_PATH) as f:
        poses = json.load(f)
    if "stand" not in poses:
        print("✗ poses.json has no 'stand' pose. Run teach_pose.py.")
        sys.exit(1)
    return poses["stand"]


def preflight(bus, servo_id):
    label = describe_id(servo_id)
    if not bus.ping(servo_id):
        print(f"✗ No response at ID {servo_id} ({label}). Check power/cable.")
        sys.exit(1)
    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        print(f"✗ {label} (ID {servo_id}) in {MODE_NAMES.get(mode, mode)} mode. "
              f"Run: python wheel_off.py {servo_id}")
        sys.exit(1)
    mn = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
    mx = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
    if mn >= mx or (mn == 0 and mx >= MAX_POSITION):
        print(f"✗ {label} (ID {servo_id}) has no usable angle limits ({mn}..{mx}). "
              f"Run: python set_limits.py {servo_id}")
        sys.exit(1)
    print(f"✓ {label} (ID {servo_id}): limits {mn}..{mx}")
    return mn, mx


def clamp(counts, mn, mx):
    return max(mn, min(mx, counts))


def stand_pos(sid, stand):
    return int(stand[str(sid)])


def go(bus, targets, speed_raw, limits, timeout):
    """Move servos to targets (dict {sid: counts}), wait until stopped."""
    moves = {}
    for sid, counts in targets.items():
        mn, mx = limits[sid]
        moves[sid] = (clamp(counts, mn, mx), speed_raw)
    bus.sync_goal_move(moves)
    bus.sync_wait_until_stopped(list(targets.keys()), timeout)


def all_stand(bus, stand, limits, speed_raw, timeout):
    """Return all servos to the stand pose."""
    targets = {sid: stand_pos(sid, stand)
               for ids in LEG_IDS.values() for sid in ids}
    go(bus, targets, speed_raw, limits, timeout)


# =============================================================================
# Moves
# =============================================================================
def move_bob(bus, stand, limits, speed_raw, timeout, reps):
    """Body rises and falls: all femurs extend and retract together."""
    print("  ♪  bob")
    all_ids = [sid for ids in LEG_IDS.values() for sid in ids]
    for _ in range(reps):
        # Down (femurs extend -- moves body toward ground)
        targets = {}
        for ids in LEG_IDS.values():
            fid = ids[FEMUR]
            targets[fid] = stand_pos(fid, stand) + BOB_COUNTS
        go(bus, targets, speed_raw, limits, timeout)
        # Up (femurs retract -- body rises)
        targets = {}
        for ids in LEG_IDS.values():
            fid = ids[FEMUR]
            targets[fid] = stand_pos(fid, stand) - BOB_COUNTS
        go(bus, targets, speed_raw, limits, timeout)
    all_stand(bus, stand, limits, speed_raw, timeout)


def move_tilt(bus, stand, limits, speed_raw, timeout, reps):
    """Body rocks: each leg takes a turn being the low side."""
    print("  ♪  tilt")
    leg_nums = list(LEG_IDS.keys())
    for _ in range(reps):
        for low_leg in leg_nums:
            targets = {}
            for leg_num, ids in LEG_IDS.items():
                fid = ids[FEMUR]
                if leg_num == low_leg:
                    targets[fid] = stand_pos(fid, stand) + TILT_COUNTS
                else:
                    targets[fid] = stand_pos(fid, stand) - TILT_COUNTS // 2
            go(bus, targets, speed_raw, limits, timeout)
    all_stand(bus, stand, limits, speed_raw, timeout)


def move_shimmy(bus, stand, limits, speed_raw, timeout, reps):
    """Body wiggles: leg 1 coxa forward while legs 2+3 go back, then swap."""
    print("  ♪  shimmy")
    for leg_num, ids in LEG_IDS.items():
        cid = ids[COXA]
        mn, mx = limits[cid]
        half = int((mx - mn) * SHIMMY_FRAC)
        center = stand_pos(cid, stand)
        # pre-compute per-leg shimmy positions
        LEG_IDS[leg_num]._shimmy_fwd = clamp(center + half, mn, mx)
        LEG_IDS[leg_num]._shimmy_bck = clamp(center - half, mn, mx)

    for _ in range(reps):
        # Phase A: leg 1 forward, legs 2+3 back
        targets = {}
        for leg_num, ids in LEG_IDS.items():
            cid = ids[COXA]
            if leg_num == 1:
                targets[cid] = ids._shimmy_fwd
            else:
                targets[cid] = ids._shimmy_bck
        go(bus, targets, speed_raw, limits, timeout)
        # Phase B: leg 1 back, legs 2+3 forward
        targets = {}
        for leg_num, ids in LEG_IDS.items():
            cid = ids[COXA]
            if leg_num == 1:
                targets[cid] = ids._shimmy_bck
            else:
                targets[cid] = ids._shimmy_fwd
        go(bus, targets, speed_raw, limits, timeout)

    # clean up monkey-patched attrs
    for ids in LEG_IDS.values():
        for attr in ("_shimmy_fwd", "_shimmy_bck"):
            if hasattr(ids, attr):
                delattr(ids, attr)

    all_stand(bus, stand, limits, speed_raw, timeout)


def move_spin(bus, stand, limits, speed_raw, timeout, reps):
    """Body rotates: all coxas sweep in the same rotational direction."""
    print("  ♪  spin")
    # Compute per-leg spin endpoints from stand position
    spin_a = {}  # all coxas to one side
    spin_b = {}  # all coxas to the other side
    for leg_num, ids in LEG_IDS.items():
        cid = ids[COXA]
        mn, mx = limits[cid]
        half = int((mx - mn) * SPIN_FRAC)
        center = stand_pos(cid, stand)
        spin_a[cid] = clamp(center + half, mn, mx)
        spin_b[cid] = clamp(center - half, mn, mx)

    for _ in range(reps):
        go(bus, spin_a, speed_raw, limits, timeout)
        go(bus, spin_b, speed_raw, limits, timeout)

    all_stand(bus, stand, limits, speed_raw, timeout)


def move_bow(bus, stand, limits, speed_raw, timeout):
    """A single bow: body dips low and comes back up."""
    print("  ♪  bow")
    targets = {}
    for ids in LEG_IDS.values():
        fid = ids[FEMUR]
        targets[fid] = stand_pos(fid, stand) + BOB_COUNTS * 2
    go(bus, targets, speed_raw, limits, timeout)
    time.sleep(0.4)   # hold the bow for a moment
    all_stand(bus, stand, limits, speed_raw, timeout)


def move_routine(bus, stand, limits, speed_raw, timeout, reps):
    """Full dance: bob -> tilt -> shimmy -> spin -> bow."""
    print("  ♪  routine start")
    move_bob    (bus, stand, limits, speed_raw, timeout, reps)
    move_tilt   (bus, stand, limits, speed_raw, timeout, reps)
    move_shimmy (bus, stand, limits, speed_raw, timeout, reps)
    move_spin   (bus, stand, limits, speed_raw, timeout, reps)
    move_bow    (bus, stand, limits, speed_raw, timeout)
    print("  ♪  routine end")


# =============================================================================
# Main
# =============================================================================
def main():
    args = parse_args()

    if args.reps < 1:
        print("✗ --reps must be at least 1."); sys.exit(1)
    if args.speed <= 0:
        print("✗ --speed must be positive."); sys.exit(1)
    if args.speed > TYPICAL_MAX_RPM:
        print(f"⚠  {args.speed:.1f} rpm exceeds typical STS3215 max "
              f"(~{TYPICAL_MAX_RPM} rpm).")

    stand     = load_stand()
    all_ids   = [sid for ids in LEG_IDS.values() for sid in ids]
    speed_raw = rpm_to_pos_speed(args.speed)
    # Generous timeout: full coxa or femur range at given speed + margin
    timeout   = 4096 / speed_raw * 2.0 + 1.0

    print("=" * 60)
    print("CRABORA: body sway and dance")
    print("=" * 60)
    print(f"  move  : {args.move}")
    print(f"  reps  : {args.reps}")
    print(f"  speed : {args.speed:.1f} rpm")

    if not args.no_pause:
        try:
            ans = input("\nProceed? Bot standing, all feet grounded. [y/N] ")
        except EOFError:
            ans = "n"
        if ans.strip().lower() not in ("y", "yes"):
            print("Aborted."); return

    with MultiBus() as bus:

        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}

        try:
            bus.sync_enable_torque(all_ids, on=True)

            # Move to stand pose first
            print("\nMoving to stand pose...")
            all_stand(bus, stand, limits, speed_raw, timeout)
            time.sleep(0.3)

            print()
            dispatch = {
                "bob":     lambda: move_bob    (bus, stand, limits, speed_raw, timeout, args.reps),
                "tilt":    lambda: move_tilt   (bus, stand, limits, speed_raw, timeout, args.reps),
                "shimmy":  lambda: move_shimmy (bus, stand, limits, speed_raw, timeout, args.reps),
                "spin":    lambda: move_spin   (bus, stand, limits, speed_raw, timeout, args.reps),
                "routine": lambda: move_routine(bus, stand, limits, speed_raw, timeout, args.reps),
            }
            dispatch[args.move]()

            print("\n✓ Done.")

        except KeyboardInterrupt:
            print("\n\nCtrl+C -- holding position.")
            return
        finally:
            if args.release:
                try:
                    bus.sync_enable_torque(all_ids, on=False)
                    print("Torque released.")
                except IOError:
                    pass
            else:
                print("Holding stance (torque on).")


if __name__ == "__main__":
    main()
