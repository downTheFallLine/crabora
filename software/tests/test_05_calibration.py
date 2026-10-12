#!/usr/bin/env python3
"""
CRABORA bring-up test 5: calibration consistency
==================================================

Answers one question: are all 18 joints set up the same way as each
other?

This exists because the same class of fault kept costing time. Leg 6
carried different travel limits from every other leg for several
sessions before anyone noticed. Centres were set to one convention,
the convention changed, and they had to be redone. The tibia direction
sign went unmeasured for two days while every phase of the stand
recovery depended on it.

None of that is visible from a single joint. It only shows up when you
put the six legs side by side, which is what this does.

Three passes, increasingly invasive:

  SURVEY   read-only. Compares every joint against its peers -- travel
           limits, whether the limits bracket centre, whether there is
           enough travel for the recovery poses -- plus temperature,
           voltage and latched faults. Moves nothing.

  --move   SYMMETRY. Swings each joint a small amount either side of
           centre and compares how it behaves in the two directions. A
           joint that is mis-centred, stiff, or dragging reads
           differently one way than the other, and that asymmetry is
           visible long before it becomes a failure.

  --signs  DIRECTION. Moves one leg per side and asks you which way it
           actually went, then checks that against the sign tables in
           walk/gait_lib.py. This is the only way to settle a sign
           without an IMU. All three were closed out this way; re-run it
           after any re-centring or servo swap.

Exit code is non-zero if anything disagrees, so it can gate a session.

Usage:
  python3 test_05_calibration.py
  python3 test_05_calibration.py --move
  python3 test_05_calibration.py --signs
  python3 test_05_calibration.py --move --signs --legs 1 4

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SW = os.path.dirname(_HERE)
sys.path.insert(0, _SW)
sys.path.insert(0, os.path.join(_SW, "walk"))
sys.path.insert(0, os.path.join(_SW, "stand"))

try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: F401
except ImportError:
    print("FAIL: scservo_sdk is not installed.")
    print("  Install it with:  python3 -m pip install feetech-servo-sdk")
    sys.exit(1)

from gait_lib import (  # noqa: E402
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    FEMUR_UP_SIGN, FORWARD_SIGN, JOINT_NAME, LEG_NAME, TIBIA_EXTEND_SIGN,
    TIBIA_JOINT_DIGIT,
    ADDR_GOAL_POSITION, ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE,
    ADDR_MOVING, ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMP, ADDR_TORQUE_ENABLE,
    BusError, confirm, find_joints, find_urt_devices, open_ports, read1, read2,
    release, to_counts, to_degrees, write1, write2,
)
from pose import pose_compact, pose_stand, pose_tuck  # noqa: E402

ADDR_PRESENT_VOLTAGE = 62
ADDR_SERVO_STATUS    = 65
ERROR_FLAGS = {0: "Voltage", 1: "Sensor", 2: "Temperature",
               3: "Current", 4: "Angle", 5: "Overload"}

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))

LIMIT_TOLERANCE = 30      # counts a leg may differ from its peers (~2.6 deg)
PROBE_DEGREES   = 10.0    # how far --move swings each joint
PROBE_SPEED     = 200
MOVE_TIMEOUT    = 5.0
ARRIVE_TOL      = 25      # counts
ASYM_TOL        = 25      # counts of difference between the two directions
TEMP_WARN_C     = 55
VOLT_LOW        = 60      # 0.1 V units -- 6.0 V
VOLT_HIGH       = 80      # 8.0 V, the STS3215 over-voltage trip


def decode_errors(status):
    return [n for b, n in ERROR_FLAGS.items() if status & (1 << b)]


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


# =============================================================
# PASS 1 -- SURVEY (read-only)
# =============================================================
def survey(packet, joints, legs):
    """Compare every joint against its peers. Returns a list of problems."""
    problems = []

    print("\n=== SURVEY (read-only) ===")
    missing = [(leg, name) for d, name in JOINTS
               for leg in legs if leg not in joints[d]]
    if missing:
        for leg, name in missing:
            problems.append(f"leg {leg} {name} did not answer")
        print("  not answering: " + ", ".join(f"{l}-{n}" for l, n in missing))

    # --- travel limits, compared across legs -------------------------
    limits = {}
    for d, name in JOINTS:
        for leg in legs:
            if leg not in joints[d]:
                continue
            sid, port = joints[d][leg]
            limits[(leg, d)] = (
                read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))

    print(f"\n  Travel limits, degrees from centre"
          f"   (flagged if a leg differs from its peers)")
    print(f"  {'leg':>3}  " + "  ".join(f"{n:^19}" for _d, n in JOINTS))
    outliers = set()
    for d, name in JOINTS:
        present = [l for l in legs if (l, d) in limits]
        if len(present) < 2:
            continue
        med_lo = median([limits[(l, d)][0] for l in present])
        med_hi = median([limits[(l, d)][1] for l in present])
        for leg in present:
            lo, hi = limits[(leg, d)]
            if abs(lo - med_lo) > LIMIT_TOLERANCE or \
               abs(hi - med_hi) > LIMIT_TOLERANCE:
                outliers.add((leg, d))
                problems.append(
                    f"leg {leg} {name} limits {to_degrees(lo):+.0f}.."
                    f"{to_degrees(hi):+.0f}° differ from the other legs' "
                    f"{to_degrees(med_lo):+.0f}..{to_degrees(med_hi):+.0f}°")
    for leg in legs:
        cells = []
        for d, _n in JOINTS:
            if (leg, d) not in limits:
                cells.append(f"{'--':^19}")
                continue
            lo, hi = limits[(leg, d)]
            mark = " <<" if (leg, d) in outliers else "   "
            cells.append(f"{to_degrees(lo):+7.0f}..{to_degrees(hi):+6.0f}{mark}")
        print(f"  {leg:>3}  " + "  ".join(cells))

    # --- centre must sit inside the travel ---------------------------
    for (leg, d), (lo, hi) in sorted(limits.items()):
        if lo >= hi:
            problems.append(f"leg {leg} {JOINT_NAME[d]} has no travel "
                            f"({lo}..{hi}) -- wheel mode?")
        elif not lo <= CENTER_POSITION <= hi:
            problems.append(f"leg {leg} {JOINT_NAME[d]} centre {CENTER_POSITION} "
                            f"is outside its limits {lo}..{hi}")

    # --- enough travel for the recovery poses ------------------------
    need_f = max(pose_compact()["femur"], pose_tuck()["femur"])
    need_t = max(pose_compact()["tibia"], pose_tuck()["tibia"])
    print(f"\n  Recovery poses need femur +{need_f:.0f}° lift, "
          f"tibia {need_t:.0f}° flex")
    for leg in legs:
        for d, need in ((FEMUR_JOINT_DIGIT, need_f), (TIBIA_JOINT_DIGIT, need_t)):
            if (leg, d) not in limits:
                continue
            can = -to_degrees(limits[(leg, d)][0])
            if can < need - 1.0:
                problems.append(
                    f"leg {leg} {JOINT_NAME[d]} reaches only {can:+.0f}°, "
                    f"the recovery needs {need:+.0f}°")

    # --- health ------------------------------------------------------
    print(f"\n  {'joint':<10} {'temp':>5} {'volts':>7}  status")
    for leg in legs:
        for d, name in JOINTS:
            if leg not in joints[d]:
                continue
            sid, port = joints[d][leg]
            temp = read1(packet, port, sid, ADDR_PRESENT_TEMP, "temp")
            volt = read1(packet, port, sid, ADDR_PRESENT_VOLTAGE, "volts")
            flags = decode_errors(
                read1(packet, port, sid, ADDR_SERVO_STATUS, "status"))
            notes = []
            if flags:
                notes.append("LATCHED " + ", ".join(flags))
                problems.append(f"leg {leg} {name}: latched "
                                + ", ".join(flags))
            if temp >= TEMP_WARN_C:
                notes.append(f"hot")
            if volt and not VOLT_LOW <= volt <= VOLT_HIGH:
                notes.append("voltage out of range")
                problems.append(f"leg {leg} {name}: {volt / 10:.1f} V")
            if notes:
                print(f"  {f'{leg}-{name}':<10} {temp:>4}° {volt / 10:>6.1f}V"
                      f"  {'; '.join(notes)}")
    print("  (only joints worth remarking on are listed)")
    return problems, limits


# =============================================================
# PASS 2 -- SYMMETRY (small moves)
# =============================================================
def move_to(packet, sid, port, target):
    write2(packet, port, sid, ADDR_GOAL_POSITION, target, "goal")
    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)
    while time.monotonic() < deadline:
        pos = read2(packet, port, sid, ADDR_PRESENT_POSITION, "pos")
        if not read1(packet, port, sid, ADDR_MOVING, "moving") and \
                abs(pos - target) <= ARRIVE_TOL:
            return pos
        time.sleep(0.02)
    return read2(packet, port, sid, ADDR_PRESENT_POSITION, "pos")


def symmetry(packet, joints, legs, limits):
    """Swing each joint both ways and compare. Asymmetry means something
    is mis-centred or dragging, well before it becomes a failure."""
    problems = []
    counts = to_counts(PROBE_DEGREES)
    print(f"\n=== SYMMETRY (±{PROBE_DEGREES:g}° on each joint) ===")
    print(f"  {'joint':<10} {'+err':>6} {'-err':>6} {'diff':>6}  verdict")
    for leg in legs:
        for d, name in JOINTS:
            if leg not in joints[d] or (leg, d) not in limits:
                continue
            sid, port = joints[d][leg]
            lo, hi = limits[(leg, d)]
            if CENTER_POSITION + counts > hi or CENTER_POSITION - counts < lo:
                print(f"  {f'{leg}-{name}':<10} {'(travel too narrow to probe)':>28}")
                continue
            write2(packet, port, sid, ADDR_GOAL_SPEED, PROBE_SPEED, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque")
            move_to(packet, sid, port, CENTER_POSITION)
            plus = move_to(packet, sid, port, CENTER_POSITION + counts)
            move_to(packet, sid, port, CENTER_POSITION)
            minus = move_to(packet, sid, port, CENTER_POSITION - counts)
            move_to(packet, sid, port, CENTER_POSITION)
            e_p = abs(plus - (CENTER_POSITION + counts))
            e_m = abs(minus - (CENTER_POSITION - counts))
            diff = abs(e_p - e_m)
            verdict = "ok"
            # Stiff BOTH ways needs both errors high -- test the smaller
            # one. Testing the larger would call a joint that is stiff in
            # only one direction "stiff both ways", which points at the
            # wrong fault: one-sided drag is a centring or binding
            # problem, two-sided is load or a tight joint.
            if min(e_p, e_m) > ARRIVE_TOL * 2:
                verdict = "STIFF both ways"
                problems.append(f"leg {leg} {name} is stiff in both directions")
            elif diff > ASYM_TOL:
                verdict = ("harder going +" if e_p > e_m else "harder going -")
                problems.append(
                    f"leg {leg} {name} is asymmetric ({e_p} vs {e_m} counts) "
                    f"-- mis-centred, or dragging one way")
            print(f"  {f'{leg}-{name}':<10} {e_p:>6} {e_m:>6} {diff:>6}  {verdict}")
    return problems


# =============================================================
# PASS 3 -- DIRECTION (interactive)
# =============================================================
SIGN_QUESTIONS = {
    COXA_JOINT_DIGIT:  ("FORWARD (toward the point between legs 1 and 6)",
                        "REARWARD", FORWARD_SIGN),
    FEMUR_JOINT_DIGIT: ("UP", "DOWN", FEMUR_UP_SIGN),
    TIBIA_JOINT_DIGIT: ("EXTEND (knee straightening, foot pushed away)",
                        "FLEX (foot folding back)", TIBIA_EXTEND_SIGN),
}


def directions(packet, joints, legs, limits):
    """Move a joint and ask which way it went. The only way to check a
    sign without an IMU."""
    problems = []
    counts = to_counts(PROBE_DEGREES * 1.5)
    print(f"\n=== DIRECTION (you confirm what you see) ===")
    print(f"  Each joint moves {PROBE_DEGREES * 1.5:g}° in the direction the")
    print(f"  sign tables in walk/gait_lib.py say is positive.\n")
    for leg in legs:
        for d, name in JOINTS:
            if leg not in joints[d] or (leg, d) not in limits:
                continue
            pos_name, neg_name, table = SIGN_QUESTIONS[d]
            sid, port = joints[d][leg]
            lo, hi = limits[(leg, d)]
            target = CENTER_POSITION + table[leg] * counts
            if not lo <= target <= hi:
                print(f"  leg {leg} {name}: travel too narrow, skipped")
                continue
            write2(packet, port, sid, ADDR_GOAL_SPEED, PROBE_SPEED, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque")
            move_to(packet, sid, port, CENTER_POSITION)
            move_to(packet, sid, port, target)
            ok = confirm(f"leg {leg} {name}: did it move {pos_name}?")
            move_to(packet, sid, port, CENTER_POSITION)
            if not ok:
                problems.append(
                    f"leg {leg} {name}: moved {neg_name} where the sign table "
                    f"says {pos_name} -- flip its entry in gait_lib.py")
                print(f"    ✗ sign table disagrees with the hardware")
            else:
                print(f"    ✓ matches the sign table")
    return problems


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="CRABORA: check all 18 joints agree with each other.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--move", action="store_true",
                    help="also swing each joint a little and compare the two "
                         "directions (the robot WILL move)")
    ap.add_argument("--signs", action="store_true",
                    help="also move each joint and ask which way it went, "
                         "checking the sign tables against reality")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    print("Crabora bring-up test 5: calibration consistency")
    if args.move or args.signs:
        print("  ⚠ the robot WILL move. Support it or put it on the stand.")

    devices = args.ports or find_urt_devices()
    if not devices:
        print("\nRESULT: FAIL -- no FE-URT-2 boards found. Run test 2.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("\nRESULT: FAIL -- no URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    joints = {}
    try:
        joints = {d: find_joints(packet, ports, legs, d, quiet=True)
                  for d, _ in JOINTS}
        if not any(joints.values()):
            print("\nRESULT: FAIL -- no servos answered. Run test 3.")
            return 1

        problems, limits = survey(packet, joints, legs)

        if args.move or args.signs:
            if not confirm("Ready for the robot to move?"):
                print("\n  Stopped after the read-only survey.")
                return 1 if problems else 0
        if args.move:
            problems += symmetry(packet, joints, legs, limits)
        if args.signs:
            problems += directions(packet, joints, legs, limits)

        print("\n=== RESULT ===")
        if not problems:
            checked = ["limits vs peers", "centre inside travel",
                       "travel vs recovery poses", "faults, temp, voltage"]
            if args.move:
                checked.append("direction symmetry")
            if args.signs:
                checked.append("sign tables vs hardware")
            print("  PASS -- all 18 joints agree. Checked: "
                  + "; ".join(checked) + ".")
            return 0
        print(f"  FAIL -- {len(problems)} problem(s):")
        for p in problems:
            print(f"    - {p}")
        print("\n  Most of these are fixed with utils/set_center.py.")
        return 1

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\nRESULT: FAIL -- bus error: {e}")
        return 1
    finally:
        if args.move or args.signs:
            for d, _ in JOINTS:
                release(packet, joints.get(d, {}))
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
