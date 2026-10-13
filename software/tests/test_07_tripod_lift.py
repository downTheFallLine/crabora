#!/usr/bin/env python3
"""
Crabora bring-up test 7: does a tripod actually leave the floor?

WHY THIS EXISTS

The walk drags its feet. The gait is not obviously wrong -- lift, swing,
plant, alternate, with the stance legs re-solved to hold their feet --
and computed against the real geometry a 25 degree femur lift gives
about 51 mm of clearance. Plenty.

But 51 mm of clearance is also 51 mm of permission to sag. During a
tripod gait THREE legs carry the whole robot, twice what each carried
standing on six. If the body drops by the lift amount when half the legs
come up, the raised feet never leave the floor, and from the outside
that looks exactly like a gait with no lift in it.

This separates the two. It does not walk. It stands, lifts one tripod,
and measures what actually happened:

  - did the swing femurs reach the angle they were told to?
  - did the stance legs hold, or did they give?
  - how far did the body actually drop?
  - is there any clearance left?

A gait problem and a power problem look identical in motion and quite
different here.

SAFETY: the robot must already be standing. It lifts one tripod and puts
it straight back. Nothing walks, nothing translates.

Usage:
  python3 test_07_tripod_lift.py
  python3 test_07_tripod_lift.py --lift 35
  python3 test_07_tripod_lift.py --tripod B
"""

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SOFTWARE = os.path.dirname(_HERE)
for _d in ("walk", "stand"):
    sys.path.insert(0, os.path.join(_SOFTWARE, _d))

from gait_lib import (  # noqa: E402
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, COXA_VERTICAL,
    FEMUR_JOINT_DIGIT, LEG_NAME, TIBIA_JOINT_DIGIT, TRIPOD_A, TRIPOD_B,
    ADDR_GOAL_POSITION, ADDR_GOAL_SPEED, ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    BusError, PacketHandler, bus_stats, confirm, delta_degrees,
    find_joints, find_urt_devices, open_ports, read2, reset_bus_stats,
    to_counts, write1, write2,
)
from pose import leg_fk  # noqa: E402

ADDR_PRESENT_LOAD    = 60
ADDR_PRESENT_VOLTAGE = 62

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


def decode_load(raw):
    return -(raw & 0x3FF) if raw & 0x400 else (raw & 0x3FF)


def read_volts(packet, joints, legs):
    """Lowest supply voltage any femur reports.

    Taken before and during the lift, because this is the measurement
    that separates the two explanations. A supply that holds while three
    legs take the whole robot is not the problem; one that dips under
    that load is exactly the problem, and it will not show up standing
    still on six.
    """
    lo = None
    for leg in legs:
        sid, port = joints[FEMUR_JOINT_DIGIT][leg]
        try:
            v = read2(packet, port, sid, ADDR_PRESENT_VOLTAGE, "voltage") / 10.0
        except BusError:
            continue
        lo = v if lo is None else min(lo, v)
    return lo


def read_pose(packet, joints, leg):
    """Present angles for one leg, in pose degrees."""
    out = {}
    for dg, name in JOINTS:
        sid, port = joints[dg][leg]
        out[name] = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
    # Counts -> pose degrees, matching pose_counts() in stand.py. Note
    # delta_degrees, not to_degrees: these are differences from centre.
    return {
        "coxa":  delta_degrees(out["coxa"] - CENTER_POSITION),
        "femur": -delta_degrees(out["femur"] - CENTER_POSITION),
        "tibia": -delta_degrees(out["tibia"] - CENTER_POSITION),
    }


def body_height(pose):
    """Deck clearance implied by this leg's angles, if its foot is down."""
    _out, down = leg_fk(pose["femur"], pose["tibia"])
    return down - COXA_VERTICAL


def survey(packet, joints, legs):
    return {leg: read_pose(packet, joints, leg) for leg in legs}


def main():
    ap = argparse.ArgumentParser(
        description="Lift one tripod and measure whether it leaves the floor.")
    ap.add_argument("--lift", type=float, default=25.0,
                    help="femur degrees to lift (default 25, matching walk.py)")
    ap.add_argument("--tripod", choices=["A", "B"], default="A")
    ap.add_argument("--speed", type=int, default=200)
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to let it settle before measuring")
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    if not 0 < args.lift <= 60:
        ap.error("--lift must be 0-60 degrees")

    print("Crabora bring-up test 7: does a tripod leave the floor?\n")
    print("  The robot must ALREADY BE STANDING. This lifts one tripod and")
    print("  puts it straight back. It does not walk.\n")

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1
    packet = PacketHandler(0)

    try:
        joints = {dg: find_joints(packet, ports, list(ALL_LEGS), dg, quiet=True)
                  for dg, _ in JOINTS}
        legs = [l for l in ALL_LEGS
                if all(l in joints[dg] for dg, _ in JOINTS)]
        if len(legs) < 6:
            print(f"Only {legs} have all three joints. Need all six.")
            return 1

        swing = sorted(l for l in legs if l in (TRIPOD_A if args.tripod == "A"
                                                else TRIPOD_B))
        stance = sorted(l for l in legs if l not in swing)
        print(f"  swing tripod {args.tripod}: {swing}")
        print(f"  stance tripod:            {stance}\n")

        # --- before --------------------------------------------------
        before = survey(packet, joints, legs)
        v_before = read_volts(packet, joints, legs)
        h_before = sum(body_height(before[l]) for l in legs) / len(legs)
        print(f"  standing at {h_before:.0f} mm deck clearance "
              f"(mean of six legs)")
        if h_before < 30:
            print(f"\n  ⚠ That is not standing. Run stand.py first.")
            return 1

        if args.pause and not confirm(
                f"Lift tripod {args.tripod} by {args.lift:g}°?"):
            print("\n  Stopped.")
            return 1

        # --- lift ----------------------------------------------------
        reset_bus_stats()
        targets = {}
        for leg in swing:
            sid, port = joints[FEMUR_JOINT_DIGIT][leg]
            want = before[leg]["femur"] + args.lift
            counts = CENTER_POSITION - to_counts(want)
            targets[leg] = (sid, port, want, counts)
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque")
        for leg, (sid, port, _want, counts) in targets.items():
            write2(packet, port, sid, ADDR_GOAL_POSITION, counts, "goal")

        # Voltage DURING the lift, while three legs hold everything --
        # the moment the supply is actually being asked a question.
        time.sleep(args.settle / 2)
        v_during = read_volts(packet, joints, legs)
        time.sleep(args.settle / 2)
        after = survey(packet, joints, legs)

        # --- what happened -------------------------------------------
        print(f"\n  === SWING LEGS (tripod {args.tripod}) ===")
        print(f"  {'leg':>4} {'commanded':>10} {'reached':>9} {'short by':>9} "
              f"{'load':>7}")
        short_worst = 0.0
        for leg in swing:
            want = targets[leg][2]
            got = after[leg]["femur"]
            short = want - got
            short_worst = max(short_worst, abs(short))
            sid, port = joints[FEMUR_JOINT_DIGIT][leg]
            load = decode_load(read2(packet, port, sid, ADDR_PRESENT_LOAD,
                                     "load"))
            print(f"  {leg:>4} {want:>+9.1f}° {got:>+8.1f}° {short:>+8.1f}° "
                  f"{load:>7}")

        print(f"\n  === STANCE LEGS (carrying everything) ===")
        print(f"  {'leg':>4} {'femur before':>13} {'after':>9} {'gave':>8} "
              f"{'load':>7}")
        sag_deg = 0.0
        for leg in stance:
            b, a = before[leg]["femur"], after[leg]["femur"]
            sag_deg = max(sag_deg, b - a)
            sid, port = joints[FEMUR_JOINT_DIGIT][leg]
            load = decode_load(read2(packet, port, sid, ADDR_PRESENT_LOAD,
                                     "load"))
            print(f"  {leg:>4} {b:>+12.1f}° {a:>+8.1f}° {b-a:>+7.1f}° "
                  f"{load:>7}")

        h_after = sum(body_height(after[l]) for l in stance) / len(stance)
        drop = h_before - h_after

        # Clearance: how far the swing feet ended up above the floor the
        # stance legs are standing on.
        print(f"\n  === RESULT ===")
        print(f"  body height before : {h_before:>6.0f} mm")
        print(f"  body height after  : {h_after:>6.0f} mm   "
              f"(dropped {drop:+.0f} mm)")

        clearance = []
        for leg in swing:
            foot_below = leg_fk(after[leg]["femur"],
                                after[leg]["tibia"])[1] - COXA_VERTICAL
            clearance.append(h_after - foot_below)
        worst = min(clearance)
        print(f"  swing foot clearance: {worst:>6.0f} mm   "
              f"(all: {', '.join(f'{c:.0f}' for c in clearance)})")

        if v_before is not None and v_during is not None:
            print(f"  supply              : {v_before:>6.1f} V standing, "
                  f"{v_during:.1f} V on three legs "
                  f"({v_during - v_before:+.1f} V)")
        retries, failures = bus_stats()
        if retries or failures:
            print(f"  bus                 : {retries} re-sends, "
                  f"{failures} failures")

        print()
        if worst > 10:
            print(f"  PASS -- the tripod is {worst:.0f} mm off the floor.")
            print(f"  The lift works, so dragging is coming from somewhere")
            print(f"  else: check the stance height during the walk itself.")
            verdict = 0
        elif drop > 15:
            print(f"  FAIL -- THE BODY SAGGED {drop:.0f} mm.")
            print(f"  The swing legs did lift, and the body came down to meet")
            print(f"  them. Three legs cannot hold what six were holding.")
            print(f"  That is torque or supply, not the gait:")
            if v_during is not None and v_before is not None and \
                    v_during < v_before - 0.3:
                print(f"    - the supply dipped {v_before - v_during:.1f} V "
                      f"under the load. That is the cause.")
            print(f"    - the LM2596 has ~1 V over its dropout and cannot do")
            print(f"      2 A continuously; this is the load that finds out")
            print(f"    - a bigger --lift masks it, it does not fix it")
            verdict = 1
        elif short_worst > 8:
            print(f"  FAIL -- the swing femurs did not reach their target")
            print(f"  (worst {short_worst:.0f}° short) and the body did not")
            print(f"  sag. The joints are being commanded but not arriving:")
            print(f"  check travel limits and torque on those three femurs.")
            verdict = 1
        else:
            print(f"  FAIL -- only {worst:.0f} mm of clearance, with no sag")
            print(f"  and the femurs arriving. The geometry expected ~51 mm")
            print(f"  at 25°, so the stance height or the pose model is off.")
            verdict = 1

        # --- put it back ---------------------------------------------
        print("\n  lowering the tripod back...")
        for leg in swing:
            sid, port = joints[FEMUR_JOINT_DIGIT][leg]
            counts = CENTER_POSITION - to_counts(before[leg]["femur"])
            write2(packet, port, sid, ADDR_GOAL_POSITION, counts, "goal")
        time.sleep(args.settle)
        print("  done. Torque left ON.")
        return verdict

    except KeyboardInterrupt:
        print("\n  Interrupted. Torque left ON.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
