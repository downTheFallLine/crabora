#!/usr/bin/env python3
"""
CRABORA stand -- recover from an arbitrary pose and stand up
==============================================================

Four phases, run in order (see README.md for why each is shaped the way
it is):

  SURVEY    read all 18 joints, move nothing, refuse to continue if
            anything is latched in overload
  COLLAPSE  femurs to horizontal and tibias straight, so the body
            settles onto its belly and every leg goes unloaded.
            Coxas are NOT touched here -- swinging a 440 mm extended
            leg is exactly what the untangle phase exists to avoid.
  UNTANGLE  one leg at a time: draw it compact and lifted, rotate its
            coxa to centre, and LEAVE IT LIFTED. Sequential, because
            the thing a leg can collide with is its neighbour. A leg
            whose travel limits cannot lift it clear is refused rather
            than dragged.
  STAND     all six together, interpolated in steps from lifted to the
            standing pose. The one phase where legs MUST move as one,
            or the body tips.

Safety, in layers:
  - firmware travel limits, already set, clamp every target
  - a reduced torque limit during recovery, raised only for the lift
  - bind detection from present load with no change in position
  - an escalating shake to free a bound joint, which gives up rather
    than forcing
  - operator confirmation between phases

FAILURE POLICY: continue with five legs and report. A leg that cannot
be freed is dropped from the remaining phases and named in the summary;
the exit code reflects that something was left behind.

⚠ THE TIBIA SIGN IS ASSUMED, NOT VERIFIED (see README.md). Every phase
here commands tibias. If the sign is backwards, UNTANGLE's first move
extends the knee into whatever the leg is caught on instead of pulling
it clear. Run on the test stand until that is settled.

Usage:
  python3 stand.py --dry-run
  python3 stand.py --phase survey
  python3 stand.py --phase collapse
  python3 stand.py --height 80
  python3 stand.py --height 80 --no-pause --speed 150

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    JOINT_NAME, LEG_NAME, POSITION_TOLERANCE, TIBIA_JOINT_DIGIT,
    ADDR_GOAL_POSITION, ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE,
    ADDR_MOVING, ADDR_PRESENT_POSITION, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    confirm, find_joints, find_urt_devices, open_ports, read1, read2,
    release, to_counts, to_degrees, write1, write2,
)
from pose import (  # noqa: E402
    GROUND_BELOW_FEMUR, Unreachable, floor_strikes, foot_height, pose_compact,
    pose_splay, pose_stand, pose_tuck, safe_sequence, stand_height_range,
)
from survey import ADDR_PRESENT_LOAD, ADDR_SERVO_STATUS, decode_errors, decode_load

# SRAM torque limit, 0..1000 in 0.1% units. Capping this is what makes a
# bind safe: a jammed servo simply fails to move instead of stripping
# gears or browning out the supply.
#
# ⚠ The address is from the common Feetech map and is NOT verified on
# this hardware. Every write is read back; if the readback disagrees the
# run continues WITHOUT torque limiting and says so, rather than
# believing in a protection that is not there.
ADDR_TORQUE_LIMIT = 48

RECOVERY_TORQUE_PCT = 35      # during collapse and untangle
LIFT_TORQUE_PCT     = 85      # the stand itself needs real force

# Thresholds recalibrated against a real run 2026-10-11. A femur holding
# an outstretched leg against gravity settles 2-5 degrees short at
# 18-43% load -- that is ordinary servo droop under a cantilever, not a
# bind, and the original 20-count / 30% thresholds called all of it a
# fault. A real bind is no movement at near-full load.
BIND_LOAD = 600               # 60% of scale, against an 85% torque limit
LOADED_TOLERANCE = 60         # counts (~5 deg) allowed when a joint is
                              # holding weight; the free-movement
                              # POSITION_TOLERANCE is far too tight there
SHAKE_AMPLITUDES = (3.0, 6.0, 12.0)   # degrees, escalating
SHAKE_CYCLES = 3
LIFT_STEPS = 6                # interpolation steps, lifted -> standing
MOVE_TIMEOUT = 6.0

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# POSE -> COUNTS
# =============================================================
def pose_counts(pose):
    """Convert a pose in degrees to servo counts, per joint digit.

    Sign conventions come from gait_lib's verified tables:
      coxa   +count = CCW                     -> count = centre + deg
      femur  +count LOWERS (FEMUR_UP_SIGN -1) -> count = centre - deg
      tibia  +count EXTENDS (ASSUMED +1)      -> count = centre - flex
    """
    return {
        COXA_JOINT_DIGIT:  CENTER_POSITION + to_counts(pose["coxa"]),
        FEMUR_JOINT_DIGIT: CENTER_POSITION - to_counts(pose["femur"]),
        TIBIA_JOINT_DIGIT: CENTER_POSITION - to_counts(pose["tibia"]),
    }


# =============================================================
# BUS HELPERS
# =============================================================
def set_torque_limit(packet, servos, pct):
    """Cap torque. Returns True only if every servo read the value back."""
    value = max(1, min(1000, int(pct * 10)))
    ok = True
    for (sid, port) in servos:
        try:
            write2(packet, port, sid, ADDR_TORQUE_LIMIT, value, "torque limit")
            if read2(packet, port, sid, ADDR_TORQUE_LIMIT, "torque limit") != value:
                ok = False
        except BusError:
            ok = False
    return ok


def move_group(packet, group, targets):
    """Write every goal first, then wait for all of them.

    `group` is {key: (sid, port)}; `targets` is {key: counts}. Writing
    all the goals before waiting is what makes a multi-joint move
    simultaneous rather than a sequence of twitches.
    """
    for key, (sid, port) in group.items():
        write2(packet, port, sid, ADDR_GOAL_POSITION, targets[key], "goal")
    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)
    results = {}
    pending = set(group)
    while pending:
        for key in list(pending):
            sid, port = group[key]
            pos = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
            moving = read1(packet, port, sid, ADDR_MOVING, "moving")
            results[key] = (pos, False)
            if not moving and abs(pos - targets[key]) <= POSITION_TOLERANCE:
                pending.discard(key)
        if pending and time.monotonic() > deadline:
            for key in pending:
                results[key] = (results[key][0], True)
            break
        if pending:
            time.sleep(0.02)
    return results


def diagnose(packet, sid, port, pos, target):
    """Why did this joint not arrive? Load tells bound from merely slow."""
    load = decode_load(read2(packet, port, sid, ADDR_PRESENT_LOAD, "load"))
    if abs(load) >= BIND_LOAD:
        return f"BOUND (load {load:+}, stopped {abs(pos - target)} counts short)"
    return f"did not arrive (load {load:+}, {abs(pos - target)} counts short)"


def shake(packet, sid, port, limits, verbose=True):
    """Escalating oscillation to break a joint loose. Returns True if it
    moved. Amplitude escalates; torque never does."""
    lo, hi = limits
    start = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
    for amp_deg in SHAKE_AMPLITUDES:
        amp = to_counts(amp_deg)
        for _ in range(SHAKE_CYCLES):
            for target in (start + amp, start - amp):
                write2(packet, port, sid, ADDR_GOAL_POSITION,
                       max(lo, min(hi, target)), "shake")
                time.sleep(0.12)
        write2(packet, port, sid, ADDR_GOAL_POSITION, start, "shake home")
        time.sleep(0.15)
        now = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
        if abs(now - start) > POSITION_TOLERANCE:
            if verbose:
                print(f"      freed after a ±{amp_deg:g}° shake")
            return True
        if verbose:
            print(f"      ±{amp_deg:g}° shake: no movement")
    return False


# =============================================================
# PHASES
# =============================================================
def phase_survey(packet, joints, legs):
    """Read-only. Returns (ok, [legs to exclude])."""
    print("\n=== SURVEY ===")
    bad = []
    for leg in legs:
        for digit, name in JOINTS:
            if leg not in joints[digit]:
                print(f"  leg {leg} {name}: NOT ANSWERING")
                if leg not in bad:
                    bad.append(leg)
                continue
            sid, port = joints[digit][leg]
            flags = decode_errors(
                read1(packet, port, sid, ADDR_SERVO_STATUS, "status"))
            if flags:
                print(f"  servo {sid} ({leg} {name}): LATCHED {', '.join(flags)}")
                if leg not in bad:
                    bad.append(leg)
    if bad:
        print(f"\n  Legs with faults or missing joints: "
              f"{', '.join(map(str, bad))}")
        print("  Latched errors have already dropped torque and ignore "
              "commands.\n  Clear them (power cycle) to include those legs.")
    else:
        print("  All joints answering, no latched faults.")
    return True, bad


SAFE_LIFT_DEG = 45.0    # femur angle held while tibias straighten
FLOOR_MARGIN  = 15.0    # a lifted leg must clear the floor by this much
                        # before its coxa is allowed to rotate


def phase_collapse(packet, joints, legs, limits, dry):
    """Body to belly, legs unloaded -- in an order that cannot drive a
    foot into the floor.

    The naive order (femurs to horizontal, then tibias straight) is what
    snapped a tibia on 2026-10-11: with the floor 62.6 mm below the
    femur axis, lowering a femur onto a tibia that is still flexed as
    little as 40 degrees buries the foot. So instead:

      1. raise each femur to AT LEAST SAFE_LIFT_DEG -- only ever
         raising, which can never drive a foot down
      2. straighten the tibias while the femurs are up there
      3. only now lower the femurs to horizontal

    Coxas are still deliberately untouched; rotating an extended leg is
    what UNTANGLE exists to handle.
    """
    print("\n=== COLLAPSE (body to belly, legs unloaded) ===")
    trouble = {}

    femurs = {l: joints[FEMUR_JOINT_DIGIT][l]
              for l in legs if l in joints[FEMUR_JOINT_DIGIT]}
    tibias = {l: joints[TIBIA_JOINT_DIGIT][l]
              for l in legs if l in joints[TIBIA_JOINT_DIGIT]}

    def run(label, group, targets, digit):
        print(f"  {label}")
        if dry:
            return
        results = move_group(packet, group, targets)
        for leg, (pos, timed_out) in results.items():
            if timed_out or abs(pos - targets[leg]) > POSITION_TOLERANCE:
                sid, port = group[leg]
                why = diagnose(packet, sid, port, pos, targets[leg])
                print(f"    leg {leg} {JOINT_NAME[digit]}: {why}")
                trouble.setdefault(leg, []).append(f"{JOINT_NAME[digit]} {why}")

    # 1. Raise every femur to at least SAFE_LIFT_DEG. Read where each one
    #    is first: commanding a flat SAFE_LIFT would LOWER any femur that
    #    is already higher, which is the dangerous direction.
    lift_counts = CENTER_POSITION - to_counts(SAFE_LIFT_DEG)
    targets = {}
    for leg, (sid, port) in femurs.items():
        now = (read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
               if not dry else CENTER_POSITION)
        # femur: fewer counts = higher, so "at least this high" is a min()
        targets[leg] = clamp(min(now, lift_counts), limits[(leg, FEMUR_JOINT_DIGIT)])
    run(f"1. femurs up to at least +{SAFE_LIFT_DEG:g}° (never lowering one)",
        femurs, targets, FEMUR_JOINT_DIGIT)

    # 2. Straighten the tibias while the femurs are clear of the floor.
    run("2. tibias -> straight (femurs still up)", tibias,
        {l: clamp(CENTER_POSITION, limits[(l, TIBIA_JOINT_DIGIT)])
         for l in tibias}, TIBIA_JOINT_DIGIT)

    # 3. Only now bring the femurs down.
    run("3. femurs -> horizontal", femurs,
        {l: clamp(CENTER_POSITION, limits[(l, FEMUR_JOINT_DIGIT)])
         for l in femurs}, FEMUR_JOINT_DIGIT)
    return trouble


def phase_untangle(packet, joints, legs, limits, dry, shake_enabled=True,
                   compact_out=140.0, compact_clear=70.0):
    """One leg at a time: compact, rotate coxa to centre, set down."""
    print("\n=== UNTANGLE (one leg at a time) ===")
    compact_pose = pose_compact(compact_out, compact_clear)
    compact = pose_counts(compact_pose)
    compact_need = compact_pose["femur"]
    print(f"  (each leg ends lifted at femur {compact_pose['femur']:+.0f}°, "
          f"flex {compact_pose['tibia']:.0f}° -- "
          f"{foot_height(compact_pose['femur'], compact_pose['tibia']):.0f} mm "
          f"of floor clearance; it is NOT set down until the stand phase)")
    failed = []
    for leg in legs:
        print(f"  leg {leg} ({LEG_NAME[leg]}):", end="", flush=True)
        if dry:
            print(f"  compact femur {compact_pose['femur']:+.0f}°"
                  f" flex {compact_pose['tibia']:.0f}°  -> coxa 0  (stays lifted)")
            continue
        # What the travel limits will ACTUALLY allow, not what we asked
        # for. A femur that clamps short leaves the foot hanging low, and
        # rotating the coxa then drags it across the floor -- which is
        # how legs dug in and stuck on 2026-10-11.
        fem_lim = limits.get((leg, FEMUR_JOINT_DIGIT))
        tib_lim = limits.get((leg, TIBIA_JOINT_DIGIT))
        if fem_lim and tib_lim:
            got_f = -to_degrees(clamp(compact[FEMUR_JOINT_DIGIT], fem_lim))
            got_t = -to_degrees(clamp(compact[TIBIA_JOINT_DIGIT], tib_lim))
            clear = foot_height(got_f, got_t)
            if clear < FLOOR_MARGIN:
                print(f"  REFUSED: travel limits only reach femur "
                      f"{got_f:+.0f}°/flex {got_t:.0f}°, leaving the foot "
                      f"{clear:+.0f} mm vs the floor.")
                print(f"    Rotating the coxa would drag it. Widen the femur "
                      f"limit (needs about +{compact_need:.0f}°) and retry.")
                failed.append(leg)
                continue

        steps = (
            ("compact", {d: compact[d] for d in
                         (FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT)}),
            ("rotate",  {COXA_JOINT_DIGIT: CENTER_POSITION}),
        )
        leg_ok = True
        for name, want in steps:
            # Abort this leg the moment a step fails. In particular, if
            # COMPACT did not complete, the leg is still extended, and
            # rotating its coxa would sweep a 440 mm lever through its
            # neighbours -- the exact thing this phase exists to prevent.
            if not leg_ok:
                print(f" [skipping {name}: leg not compact]", end="", flush=True)
                continue
            group = {d: joints[d][leg] for d in want if leg in joints[d]}
            if not group:
                continue
            targets = {d: clamp(want[d], limits[(leg, d)]) for d in group}
            results = move_group(packet, group, targets)
            stuck = [d for d, (pos, to) in results.items()
                     if to or abs(pos - targets[d]) > POSITION_TOLERANCE]
            if stuck and shake_enabled:
                print(f" {name}✗", end="", flush=True)
                for d in stuck:
                    sid, port = group[d]
                    print(f"\n    {JOINT_NAME[d]} stuck, shaking:")
                    if shake(packet, sid, port, limits[(leg, d)]):
                        r = move_group(packet, {d: group[d]}, {d: targets[d]})
                        pos, to = r[d]
                        if not to and abs(pos - targets[d]) <= POSITION_TOLERANCE:
                            continue
                    leg_ok = False
                    print(f"    could not free leg {leg} {JOINT_NAME[d]}")
                print("   ", end="", flush=True)
                if not leg_ok:
                    continue
            elif stuck:
                leg_ok = False
                print(f" {name}✗", end="", flush=True)
            else:
                print(f" {name}", end="", flush=True)
        print()
        if not leg_ok:
            failed.append(leg)
    return failed


def phase_stand(packet, joints, legs, limits, height, foot_out, dry,
                start_pose=None):
    """All six together, interpolated from tuck to the standing pose."""
    print(f"\n=== STAND (to {height:g} mm deck clearance) ===")
    try:
        target_pose = pose_stand(height, foot_out)
    except Unreachable as e:
        lo, hi = stand_height_range(foot_out)
        print(f"  Cannot stand at {height:g} mm: {e}")
        print(f"  Reachable range with foot_out={foot_out:g}: "
              f"{lo:.0f}..{hi:.0f} mm")
        return {leg: ["unreachable height"] for leg in legs}

    # Legs are left LIFTED by untangle, so that is where the lift starts.
    # Going via tuck would set the feet down at only 10 mm of modelled
    # clearance -- no margin at all for an uneven floor or model error.
    start, end = (start_pose or pose_compact()), target_pose
    print(f"  femur {start['femur']:+.1f}° -> {end['femur']:+.1f}°, "
          f"flex {start['tibia']:.1f}° -> {end['tibia']:.1f}°, "
          f"in {LIFT_STEPS} steps")
    trouble = {}
    if dry:
        return trouble

    for step in range(1, LIFT_STEPS + 1):
        frac = step / LIFT_STEPS
        mid = {k: start[k] + (end[k] - start[k]) * frac for k in start}
        counts = pose_counts(mid)
        # Femurs and tibias must move AS ONE, so build a single group
        # keyed by (leg, joint) and write every goal before waiting.
        group, targets = {}, {}
        for leg in legs:
            for d in (FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT):
                if leg in joints[d]:
                    group[(leg, d)] = joints[d][leg]
                    targets[(leg, d)] = clamp(counts[d], limits[(leg, d)])
        results = move_group(packet, group, targets)
        bad = [k for k, (pos, to) in results.items()
               if to or abs(pos - targets[k]) > LOADED_TOLERANCE]
        droop = max((abs(results[k][0] - targets[k]) for k in results),
                    default=0)
        print(f"  step {step}/{LIFT_STEPS}  femur {mid['femur']:+6.1f}°  "
              f"flex {mid['tibia']:5.1f}°   worst sag {droop:>2} counts "
              f"({droop * 360 / 4096:.1f}°)" + ("  ✗" if bad else ""))
        for (leg, d) in bad:
            sid, port = group[(leg, d)]
            pos, _timed_out = results[(leg, d)]
            why = diagnose(packet, sid, port, pos, targets[(leg, d)])
            trouble.setdefault(leg, []).append(f"{JOINT_NAME[d]} {why}")
    return trouble


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="CRABORA: recover from an arbitrary pose and stand up.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--phase", choices=["survey", "collapse", "untangle",
                                        "stand", "all"], default="all",
                    help="run one phase only (default: all, in order)")
    ap.add_argument("--height", type=float, default=80.0,
                    help="deck clearance to stand at, mm (default 80)")
    ap.add_argument("--foot-out", type=float, default=120.0,
                    help="how far outboard of the knee to plant the foot, mm "
                         "(default 120; wider is more stable, less range)")
    ap.add_argument("--compact-clear", type=float, default=60.0,
                    help="ground clearance for the lifted leg during untangle, "
                         "mm (default 60, which needs about +56 deg of "
                         "femur and so fits a +60 limit). Raise if a leg "
                         "does not swing "
                         "cleanly; the femur angle rises with it.")
    ap.add_argument("--compact-out", type=float, default=140.0,
                    help="how far out from the knee to hold the foot while "
                         "compact, mm (default 140). Smaller is more tucked "
                         "in but needs a steeper femur.")
    ap.add_argument("--speed", type=int, default=200,
                    help="move speed in steps/s (default 200 -- slow on purpose)")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--no-shake", dest="shake", action="store_false",
                    help="do not try to free a bound joint by oscillating it")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmation between phases")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan without touching the bus")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")

    print("CRABORA stand -- recover and stand up")
    print(f"  target {args.height:g} mm deck clearance, foot "
          f"{args.foot_out:g} mm out, {args.speed} steps/s")
    print("  ⚠ tibia sign is ASSUMED, not verified -- test stand only\n")

    if args.dry_run:
        for name, p in (("splay", pose_splay()),
                        ("compact", pose_compact(args.compact_out,
                                                 args.compact_clear)),
                        ("tuck", pose_tuck())):
            c = pose_counts(p)
            print(f"  {name:<8} coxa {c[COXA_JOINT_DIGIT]}  "
                  f"femur {c[FEMUR_JOINT_DIGIT]}  tibia {c[TIBIA_JOINT_DIGIT]}"
                  f"   (femur {p['femur']:+.1f}°, flex {p['tibia']:.1f}°)")
        try:
            p = pose_stand(args.height, args.foot_out)
            c = pose_counts(p)
            print(f"  {'stand':<8} coxa {c[COXA_JOINT_DIGIT]}  "
                  f"femur {c[FEMUR_JOINT_DIGIT]}  tibia {c[TIBIA_JOINT_DIGIT]}"
                  f"   (femur {p['femur']:+.1f}°, flex {p['tibia']:.1f}°)")
        except Unreachable as e:
            lo, hi = stand_height_range(args.foot_out)
            print(f"  stand    UNREACHABLE: {e}  (range {lo:.0f}..{hi:.0f} mm)")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    all_servos = []
    try:
        joints = {d: find_joints(packet, ports, legs, d, quiet=True)
                  for d, _ in JOINTS}
        all_servos = [sp for d, _ in JOINTS for sp in joints[d].values()]
        if not all_servos:
            print("No servos answered. Run tests/test_03_ping_servo.py.")
            return 1

        ok, excluded = phase_survey(packet, joints, legs)
        legs = [l for l in legs if l not in excluded]
        if not legs:
            print("\nNo usable legs. Stopping.")
            return 1

        limits = {}
        for leg in legs:
            for d, _ in JOINTS:
                if leg in joints[d]:
                    sid, port = joints[d][leg]
                    limits[(leg, d)] = (
                        read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                        read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))

        # Travel limits are the usual reason a recovery stalls, so report
        # them here rather than only inside the preflight -- --phase survey
        # is what people reach for when asking "can this even work?".
        need = pose_compact(args.compact_out, args.compact_clear)
        print(f"\n  Travel limits  (degrees from centre)")
        print(f"  {'leg':>3}  {'coxa':>16}  {'femur':>16}  {'tibia':>16}")
        tight = []
        for leg in legs:
            cells = []
            for d, jname in JOINTS:
                if (leg, d) not in limits:
                    cells.append(f"{'--':>16}")
                    continue
                lo, hi = limits[(leg, d)]
                cells.append(f"{to_degrees(lo):+7.0f}..{to_degrees(hi):+6.0f}")
                if d == FEMUR_JOINT_DIGIT:
                    # femur lift is NEGATIVE counts, so the min limit is
                    # what caps how high the leg can be raised
                    can_lift = -to_degrees(lo)
                    if can_lift < need["femur"] - 1.0:
                        tight.append((leg, "femur", can_lift, need["femur"]))
                if d == TIBIA_JOINT_DIGIT:
                    can_flex = -to_degrees(lo)
                    if can_flex < need["tibia"] - 1.0:
                        tight.append((leg, "tibia", can_flex, need["tibia"]))
            print(f"  {leg:>3}  " + "  ".join(cells))

        if tight:
            print(f"\n  ⚠ Not enough travel for the recovery poses:")
            for leg, jname, got, want in tight:
                print(f"      leg {leg} {jname}: reaches {got:+.0f}°, "
                      f"needs {want:+.0f}°")
            print(f"    A leg that cannot lift clear will be REFUSED rather")
            print(f"    than dragged. Widen with utils/set_center.py.")
        else:
            print(f"\n  Travel is sufficient for the recovery poses "
                  f"(femur needs +{need['femur']:.0f}°, "
                  f"tibia flex {need['tibia']:.0f}°).")

        if args.phase == "survey":
            return 1 if (excluded or tight) else 0

        for sid, port in all_servos:
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

        # Preflight: the poses need generous travel, especially tibia
        # flex. If the firmware limits are tighter, every target clamps
        # and the robot quietly never reaches the pose. Say so first.
        print("\n  Preflight -- can the travel limits reach each pose?")
        blocked = False
        checks = [("splay", pose_splay()),
                  ("compact", pose_compact(args.compact_out,
                                           args.compact_clear)),
                  ("tuck", pose_tuck())]
        try:
            checks.append(("stand", pose_stand(args.height, args.foot_out)))
        except Unreachable:
            pass
        for name, p in checks:
            counts = pose_counts(p)
            short = []
            for leg in legs:
                for d, jname in JOINTS:
                    if (leg, d) not in limits:
                        continue
                    want = counts[d]
                    got = clamp(want, limits[(leg, d)])
                    if got != want:
                        short.append(
                            f"{leg}-{jname} wants {to_degrees(want):+.0f}° "
                            f"gets {to_degrees(got):+.0f}°")
            if short:
                blocked = True
                print(f"    {name}: CLAMPED -- " + "; ".join(short[:4])
                      + (f" (+{len(short)-4} more)" if len(short) > 4 else ""))
            else:
                print(f"    {name}: ok")
        if blocked:
            print("\n  Some poses cannot be reached within the travel limits.")
            print("  Widen them with utils/set_center.py, or the robot will")
            print("  stop short and the stand will not complete.")
            if args.pause and not confirm("Continue anyway?"):
                print("\n  Stopped.")
                return 1

        capped = set_torque_limit(packet, all_servos, RECOVERY_TORQUE_PCT)
        print(f"\n  torque limit {RECOVERY_TORQUE_PCT}%: "
              + ("confirmed by readback" if capped else
                 "NOT CONFIRMED -- proceeding without it. Watch the robot."))

        trouble, failed = {}, []
        run = args.phase

        if run in ("collapse", "all"):
            if args.pause and not confirm(
                    "Ready to COLLAPSE? Femurs go horizontal and tibias "
                    "straight; the body will settle onto its belly."):
                print("\n  Stopped.")
                return 1
            for leg, why in phase_collapse(
                    packet, joints, legs, limits, False).items():
                trouble.setdefault(leg, []).extend(why)

        if run in ("untangle", "all"):
            if args.pause and not confirm(
                    "Ready to UNTANGLE? Legs move one at a time."):
                print("\n  Stopped.")
                return 1
            failed = phase_untangle(packet, joints, legs, limits, False,
                                    args.shake, args.compact_out,
                                    args.compact_clear)
            for leg in failed:
                trouble.setdefault(leg, []).append("could not be untangled")
            legs = [l for l in legs if l not in failed]
            if not legs:
                print("\n  No legs survived untangling. Stopping.")
                return 1

        if run in ("stand", "all"):
            if args.pause and not confirm(
                    f"Ready to STAND on {len(legs)} leg(s)? This is the only "
                    f"phase that puts weight on the legs."):
                print("\n  Stopped.")
                return 1
            if not set_torque_limit(packet, all_servos, LIFT_TORQUE_PCT):
                print("  (torque limit for the lift not confirmed)")
            for leg, why in phase_stand(packet, joints, legs, limits,
                                        args.height, args.foot_out, False,
                                        pose_compact(args.compact_out,
                                                     args.compact_clear)).items():
                trouble.setdefault(leg, []).extend(why)

        print("\n=== SUMMARY ===")
        if excluded:
            print(f"  excluded before starting: {', '.join(map(str, excluded))}")
        if trouble:
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}):")
                for why in trouble[leg]:
                    print(f"      {why}")
        standing = [l for l in legs if l not in trouble]
        print(f"  {len(standing)} of 6 leg(s) completed: "
              f"{', '.join(map(str, standing)) or 'none'}")
        if len(standing) < 6:
            print("  Torque is left ON -- the robot is holding whatever pose")
            print("  it reached. Support it before releasing.")
        return 0 if len(standing) == 6 else 1

    except KeyboardInterrupt:
        print("\n\nInterrupted -- torque left ON so the robot does not drop.")
        print("Release it yourself with utils/go_limp.py once it is supported.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
