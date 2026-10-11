#!/usr/bin/env python3
"""
CRABORA survey -- phase 0 of standing up
==========================================

Reads the robot's complete state without moving anything. This is the
first step of any recovery: you cannot plan a trajectory from an unknown
pose, and "unknown" overstates it anyway -- the STS3215 has an absolute
encoder, so every joint angle is readable for free.

For all 18 joints it reports:
  - present position, as counts and as an angle from that joint's centre
  - whether that position is inside the joint's firmware travel limits
  - present load, which is how a bound servo gives itself away
  - temperature, and any latched error flags (overload, over-current...)

and per leg it computes, by forward kinematics, **where the foot
actually is** in the body frame -- so "is this leg folded under the
body?" becomes a number rather than a guess.

NOTHING MOVES. Torque is not enabled, goal positions are not written.
Safe to run at any time, in any pose.

A joint latched in overload has already dropped its own torque and will
ignore commands until cleared, so those are called out separately: they
are the first thing to fix before attempting a recovery.

⚠ The tibia sign is ASSUMED, not verified (see README.md), so the foot
positions below inherit that assumption. If the tibia sign turns out
backwards, computed foot heights are wrong in the vertical axis. Joint
readings themselves are unaffected -- those are measured, not derived.

Usage:
  python3 survey.py
  python3 survey.py --legs 1 2 3
  python3 survey.py --ports /dev/ttyACM0

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, BODY_TO_COXA_AXIS, CENTER_POSITION, COXA_JOINT_DIGIT,
    DECK_APOTHEM,
    COXA_RADIAL, COXA_TANGENT, COXA_VERTICAL, FEMUR_JOINT_DIGIT, FEMUR_LEN,
    JOINT_NAME, LEG_BEARING, LEG_NAME, TIBIA_JOINT_DIGIT, TIBIA_LEN,
    ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMP,
    BusError, PacketHandler,
    find_joints, find_urt_devices, open_ports, read1, read2, to_degrees,
)

ADDR_PRESENT_LOAD = 60    # 2 bytes, signed-magnitude: bit 10 is direction
ADDR_SERVO_STATUS = 65    # 1 byte, error bitmask

ERROR_FLAGS = {
    0: "Voltage", 1: "Sensor", 2: "Temperature",
    3: "Current", 4: "Angle", 5: "Overload",
}

LOAD_BOUND_HINT = 300     # raw load above this, with no commanded motion,
                          # is worth remarking on
FOLDED_REACH    = 300.0   # foot radial below this means the leg is well
                          # folded in (nominal straight-out reach is 441)
NEIGHBOUR_CLEAR = 120.0   # feet closer than this are plausibly tangled;
                          # at nominal pose adjacent feet are ~440 mm apart
TEMP_WARN_C = 55

JOINTS = (
    (COXA_JOINT_DIGIT, "coxa"),
    (FEMUR_JOINT_DIGIT, "femur"),
    (TIBIA_JOINT_DIGIT, "tibia"),
)


def decode_load(raw):
    """STS load is magnitude in bits 0-9 with direction in bit 10."""
    magnitude = raw & 0x3FF
    return -magnitude if raw & 0x400 else magnitude


def decode_errors(status):
    return [name for bit, name in ERROR_FLAGS.items() if status & (1 << bit)]


def foot_position(leg, coxa_deg, femur_deg, tibia_deg):
    """Foot position in the body frame (mm), from joint angles in degrees.

    Frame: +X forward through the front point, +Y left, +Z up.

    Angle conventions, all derived from the sign tables in gait_lib:
      coxa_deg  -- rotation about body +Z, CCW positive
      femur_deg -- up from horizontal, positive is up
      tibia_deg -- knee flexion from straight, positive folds the foot down

    ⚠ The tibia term inherits the ASSUMED tibia sign. See README.md.
    """
    # Leg's nominal bearing, clockwise from the front point; the body frame
    # is right-handed, hence the negated Y.
    bearing = math.radians(LEG_BEARING[leg])
    # Coxa rotation turns the whole leg about the body's vertical axis.
    phi = -bearing + math.radians(coxa_deg)
    radial = (math.cos(phi), math.sin(phi), 0.0)
    tangent = (-math.sin(phi), math.cos(phi), 0.0)

    # Coxa axis sits on the deck at the leg's bearing, unrotated.
    b = -bearing
    px = BODY_TO_COXA_AXIS * math.cos(b)
    py = BODY_TO_COXA_AXIS * math.sin(b)
    pz = 0.0

    # Coxa axis -> femur axis. Note the tangent term: the femur axis is
    # offset sideways from the coxa axis, so this is NOT a planar chain.
    px += COXA_RADIAL * radial[0] + COXA_TANGENT * tangent[0]
    py += COXA_RADIAL * radial[1] + COXA_TANGENT * tangent[1]
    pz += COXA_VERTICAL

    # Femur, then tibia, in the vertical plane along the rotated radial.
    beta = math.radians(femur_deg)
    px += FEMUR_LEN * math.cos(beta) * radial[0]
    py += FEMUR_LEN * math.cos(beta) * radial[1]
    pz += FEMUR_LEN * math.sin(beta)

    # Flexion bends the tibia down relative to the femur's direction.
    gamma = beta - math.radians(tibia_deg)
    px += TIBIA_LEN * math.cos(gamma) * radial[0]
    py += TIBIA_LEN * math.cos(gamma) * radial[1]
    pz += TIBIA_LEN * math.sin(gamma)

    return px, py, pz


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA survey: read all 18 joints without moving.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to survey (default: all six)")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    print("CRABORA survey -- reading state, moving nothing\n")

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    try:
        found = {}
        for digit, name in JOINTS:
            found[digit] = find_joints(packet, ports, legs, digit, quiet=True)

        missing = [(leg, name) for digit, name in JOINTS
                   for leg in legs if leg not in found[digit]]
        answering = sum(len(v) for v in found.values())
        print(f"  {answering} of {len(legs) * 3} joints answering")
        if missing:
            print("  not answering: " + ", ".join(
                f"{leg}-{name}" for leg, name in missing))

        # --- per-joint table --------------------------------------------
        print(f"\n  {'joint':<10} {'servo':>5} {'pos':>6} {'angle':>8} "
              f"{'limits':>13} {'load':>6} {'temp':>5}  notes")
        angles = {}
        faulted, hot, outside, loaded = [], [], [], []
        for leg in legs:
            for digit, name in JOINTS:
                if leg not in found[digit]:
                    continue
                sid, port = found[digit][leg]
                pos = read2(packet, port, sid, ADDR_PRESENT_POSITION, "position")
                lo = read2(packet, port, sid, ADDR_MIN_ANGLE, "min limit")
                hi = read2(packet, port, sid, ADDR_MAX_ANGLE, "max limit")
                load = decode_load(
                    read2(packet, port, sid, ADDR_PRESENT_LOAD, "load"))
                temp = read1(packet, port, sid, ADDR_PRESENT_TEMP, "temperature")
                flags = decode_errors(
                    read1(packet, port, sid, ADDR_SERVO_STATUS, "status"))

                angles[(leg, digit)] = to_degrees(pos)
                notes = []
                if flags:
                    faulted.append((sid, flags))
                    notes.append(", ".join(flags))
                if not lo <= pos <= hi:
                    outside.append(sid)
                    notes.append("OUTSIDE LIMITS")
                if abs(load) >= LOAD_BOUND_HINT:
                    loaded.append((sid, load))
                    notes.append("under load")
                if temp >= TEMP_WARN_C:
                    hot.append((sid, temp))
                    notes.append(f"{temp}°C")
                print(f"  {f'{leg}-{name}':<10} {sid:>5} {pos:>6} "
                      f"{to_degrees(pos):>+7.1f}° {f'{lo}..{hi}':>13} "
                      f"{load:>+6} {temp:>4}°  {'; '.join(notes)}")

        # --- derived foot positions -------------------------------------
        # The foot CANNOT get under the deck: the knee alone sits at radius
        # 222 mm, outside even the deck's 210 mm circumradius, so nothing
        # distal to it reaches underneath. The real tangle risk is leg
        # against NEIGHBOUR, not leg under body -- hence the pair check.
        print(f"\n  Foot positions (body frame: +X forward, +Y left, +Z up)")
        print(f"  {'leg':<12} {'X':>8} {'Y':>8} {'Z':>8}  {'reach':>8}  note")
        feet = {}
        for leg in legs:
            try:
                c = angles[(leg, COXA_JOINT_DIGIT)]
                f = -angles[(leg, FEMUR_JOINT_DIGIT)]   # up is -count
                t = -angles[(leg, TIBIA_JOINT_DIGIT)]   # flex is -count (ASSUMED)
            except KeyError:
                print(f"  {f'{leg} {LEG_NAME[leg]}':<12} "
                      f"{'(incomplete leg -- cannot compute)':>36}")
                continue
            x, y, z = foot_position(leg, c, f, t)
            feet[leg] = (x, y, z)
            reach = math.hypot(x, y)
            note = ""
            if reach < DECK_APOTHEM:
                note = "under the deck -- should be geometrically impossible"
            elif reach < FOLDED_REACH:
                note = "tightly folded"
            if z > DECK_APOTHEM * 0.2:
                note = (note + "; " if note else "") + "foot ABOVE deck level"
            print(f"  {f'{leg} {LEG_NAME[leg]}':<12} {x:>8.1f} {y:>8.1f} "
                  f"{z:>8.1f}  {reach:>8.1f}  {note}")

        # --- neighbour clearance ----------------------------------------
        close = []
        order = [l for l in ALL_LEGS if l in feet]
        for a, b in zip(order, order[1:] + order[:1]):
            if len(order) < 2 or (a == b):
                continue
            d = math.dist(feet[a], feet[b])
            if d < NEIGHBOUR_CLEAR:
                close.append((a, b, d))
        if close:
            print(f"\n  Neighbouring feet closer than {NEIGHBOUR_CLEAR:.0f} mm:")
            for a, b, d in close:
                print(f"    legs {a} and {b}: {d:.0f} mm apart -- may be tangled")

        # --- verdict -----------------------------------------------------
        print()
        if faulted:
            print("  ⚠ LATCHED ERRORS -- these servos have already dropped")
            print("    torque and will ignore commands until cleared:")
            for sid, flags in faulted:
                print(f"      servo {sid}: {', '.join(flags)}")
            print("    Clear them (power cycle, or a torque-enable write)")
            print("    before attempting any recovery.")
        if outside:
            print(f"  ⚠ outside firmware limits: "
                  f"{', '.join(map(str, outside))} -- the joint was moved by")
            print("    hand past where set_center.py allows, or a limit changed.")
        if loaded:
            print(f"  ⚠ under load while idle: " + ", ".join(
                f"{sid} ({load:+})" for sid, load in loaded))
            print("    Something is pushing on these. Bound, or bearing weight.")
        if hot:
            print(f"  ⚠ warm: " + ", ".join(f"{sid} ({t}°C)" for sid, t in hot))
        if close:
            print("  ⚠ neighbouring feet are close -- untangle those legs "
                  "one at a time.")
        if not (faulted or outside or loaded or hot or close):
            print("  No faults, nothing outside limits, nothing loaded.")

        print("\n  Survey complete. Nothing was moved.")
        return 1 if (faulted or outside or missing) else 0

    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
