#!/usr/bin/env python3
"""
Crabora bring-up test 7: lift a tripod and LOOK at it.

WHY THIS IS A HUMAN-IN-THE-LOOP TEST

The first version computed foot clearance from the joint angles and
reported 64 mm. Tom watched the same lift and the legs did not leave the
floor. Reality won.

That disagreement is the most useful thing this test can produce, so it
is now the point of it. The computed number is still shown, because when
the robot says "64 mm" and the operator says "still touching", the pose
model is wrong -- link lengths, an angle convention, an assumption about
where the floor is -- and that is a far more serious bug than any gait
tuning. Computing clearance from angles can only restate your
assumptions back to you. An eye on the actual robot cannot.

So: lift one tripod, ask, put it back down.

SAFETY: the robot must already be standing. Nothing walks or translates,
and the tripod goes back where it came from whatever the answer.

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
    BusError, PacketHandler, bus_stats, delta_degrees,
    find_joints, find_urt_devices, open_ports, read1, read2,
    ground_clearance, reset_bus_stats, to_counts, write1, write2,
)
from pose import leg_fk  # noqa: E402

ADDR_PRESENT_VOLTAGE = 62

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))

LIFT_INCREMENT = 15.0     # how much more to try when they are still down
LIFT_CEILING = 70.0


def read_pose(packet, joints, leg):
    """Present angles for one leg, in pose degrees."""
    raw = {}
    for dg, name in JOINTS:
        sid, port = joints[dg][leg]
        raw[name] = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
    # delta_degrees, not to_degrees: these are differences from centre and
    # to_degrees would subtract centre a second time.
    return {
        "coxa":  delta_degrees(raw["coxa"] - CENTER_POSITION),
        "femur": -delta_degrees(raw["femur"] - CENTER_POSITION),
        "tibia": -delta_degrees(raw["tibia"] - CENTER_POSITION),
    }


def body_height(pose):
    """Deck clearance this leg implies, IF its foot is on the floor."""
    _out, down = leg_fk(pose["femur"], pose["tibia"])
    return down - COXA_VERTICAL


def read_volts(packet, joints, legs):
    """Lowest supply voltage any femur reports. read1: it is ONE byte."""
    lo = None
    for leg in legs:
        sid, port = joints[FEMUR_JOINT_DIGIT][leg]
        try:
            v = read1(packet, port, sid, ADDR_PRESENT_VOLTAGE, "voltage") / 10.0
        except BusError:
            continue
        lo = v if lo is None else min(lo, v)
    return lo


def set_femurs(packet, joints, legs, want, speed):
    """Drive these legs' femurs to want[leg], in pose degrees."""
    for leg in legs:
        sid, port = joints[FEMUR_JOINT_DIGIT][leg]
        write2(packet, port, sid, ADDR_GOAL_SPEED, speed, "speed")
        write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque")
    for leg in legs:
        sid, port = joints[FEMUR_JOINT_DIGIT][leg]
        write2(packet, port, sid, ADDR_GOAL_POSITION,
               CENTER_POSITION - to_counts(want[leg]), "goal")


def ask(question):
    """y/n, defaulting to no. For 'are they off the floor?', anything
    other than a clear yes should count as no."""
    try:
        return input(f"  {question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def main():
    ap = argparse.ArgumentParser(
        description="Lift one tripod, ask whether it cleared, put it back.")
    ap.add_argument("--lift", type=float, default=25.0,
                    help="femur degrees to lift (default 25, as walk.py uses)")
    ap.add_argument("--tripod", choices=["A", "B"], default="A")
    ap.add_argument("--speed", type=int, default=200)
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to settle before asking")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    if not 0 < args.lift <= LIFT_CEILING:
        ap.error(f"--lift must be 0-{LIFT_CEILING:g} degrees")

    print("Crabora bring-up test 7: lift a tripod and look at it\n")
    print("  The robot must ALREADY BE STANDING.")
    print("  One tripod comes up, you say whether it cleared, it goes back.\n")

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
        legs = [l for l in ALL_LEGS if all(l in joints[dg] for dg, _ in JOINTS)]
        if len(legs) < 6:
            print(f"Only {legs} have all three joints answering. Need six.")
            return 1

        tri = TRIPOD_A if args.tripod == "A" else TRIPOD_B
        swing = sorted(l for l in legs if l in tri)
        stance = sorted(l for l in legs if l not in tri)
        print(f"  swing tripod {args.tripod}: {swing}  "
              f"({', '.join(LEG_NAME[l] for l in swing)})")
        print(f"  stance tripod:            {stance}\n")

        before = {l: read_pose(packet, joints, l) for l in legs}
        h_before = sum(body_height(before[l]) for l in legs) / len(legs)
        v_before = read_volts(packet, joints, legs)
        print(f"  standing at {h_before:.0f} mm deck (model) = "
              f"{ground_clearance(h_before):.0f} mm real gap under the caps"
              + (f", supply {v_before:.1f} V" if v_before else ""))
        if h_before < 30:
            print("\n  That is not standing. Run stand.py first.")
            return 1

        print()
        if not ask(f"Lift tripod {args.tripod} by {args.lift:g} degrees?"):
            print("\n  Stopped. Nothing moved.")
            return 1

        reset_bus_stats()
        lift = args.lift
        cleared = False
        attempts = []

        while True:
            want = {l: before[l]["femur"] + lift for l in swing}
            set_femurs(packet, joints, swing, want, args.speed)
            time.sleep(args.settle)

            after = {l: read_pose(packet, joints, l) for l in legs}
            v_now = read_volts(packet, joints, legs)
            floor = sum(body_height(after[l]) for l in stance) / len(stance)
            computed = {l: floor - body_height(after[l]) for l in swing}

            print(f"\n  --- femur +{lift:g} degrees ---")
            print(f"  {'leg':>4} {'commanded':>10} {'reached':>9} "
                  f"{'computed clearance':>19}")
            for l in swing:
                print(f"  {l:>4} {want[l]:>+9.1f} {after[l]['femur']:>+8.1f} "
                      f"{computed[l]:>15.0f} mm")
            if v_before and v_now:
                print(f"  supply {v_before:.1f} V -> {v_now:.1f} V")

            print()
            observed = ask(f"LOOK AT THE ROBOT. Are legs "
                           f"{', '.join(str(l) for l in swing)} "
                           f"off the floor?")
            attempts.append((lift, min(computed.values()), observed))
            if observed:
                cleared = True
                break
            if lift + LIFT_INCREMENT > LIFT_CEILING:
                print(f"\n  Reached the {LIFT_CEILING:g} degree ceiling.")
                break
            if not ask(f"Still down. Try +{lift + LIFT_INCREMENT:g} degrees?"):
                break
            lift += LIFT_INCREMENT

        # --- put it back, whatever happened --------------------------
        print("\n  lowering the tripod back...")
        set_femurs(packet, joints, swing,
                   {l: before[l]["femur"] for l in swing}, args.speed)
        time.sleep(args.settle)
        print("  done. Torque left ON.")

        # --- what it means -------------------------------------------
        print("\n  === RESULT ===")
        for lf, cc, ob in attempts:
            print(f"  femur +{lf:>4.0f}   computed {cc:>4.0f} mm   "
                  f"observed {'OFF THE FLOOR' if ob else 'still down'}")

        first_lift, first_computed, first_obs = attempts[0]
        retries, failures = bus_stats()
        if retries or failures:
            print(f"\n  bus: {retries} re-sends, {failures} failures")

        print()
        if cleared and first_obs:
            print(f"  PASS -- {first_lift:g} degrees lifts the tripod clear,")
            print(f"  and the computed {first_computed:.0f} mm agrees with what")
            print(f"  you saw. Model and robot tell the same story, so")
            print(f"  dragging during a walk is about the walk, not the lift:")
            print(f"      python3 ../walk/walk.py 150 --instrument")
            return 0

        if cleared:
            need = attempts[-1][0]
            print(f"  PARTIAL -- it took {need:g} degrees to clear, not "
                  f"{first_lift:g}.")
            print(f"  Set walk.py --lift to at least {need:g}. But note the")
            print(f"  computed figure claimed {first_computed:.0f} mm at "
                  f"{first_lift:g} degrees")
            print(f"  and that was wrong, so something in the pose model does")
            print(f"  not match the robot. Every other computed clearance in")
            print(f"  this codebase inherits the same error.")
            return 1

        print(f"  FAIL -- the tripod never left the floor, up to "
              f"{attempts[-1][0]:g} degrees.")
        print(f"  The femurs reached their commanded angles, so this is not a")
        print(f"  servo or supply problem. THE GEOMETRY IS LYING: the model")
        print(f"  computes {first_computed:.0f} mm of clearance that is not there.")
        print(f"  Worth checking, in order:")
        print(f"    - FEMUR_LEN and TIBIA_LEN against the built leg")
        print(f"    - that femur-up is really -1 and tibia-extend really +1")
        print(f"    - whether the feet are where leg_fk thinks they are")
        print(f"  Everything that computes a clearance shares this error.")
        return 1

    except KeyboardInterrupt:
        print("\n  Interrupted. Torque left ON -- the tripod may still be up.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
