#!/usr/bin/env python3
"""
Exercise several CRABORA legs at once -- swing_arm.py for N legs.  Every
joint (coxa, femur, tibia) of every listed leg is swept center -> far right
-> far left -> center, pausing one second after each move.  Three modes:

  together (default): every joint of every leg moves at once, in lockstep.
  by-joint:           all legs' coxas together, then all femurs, then all
                      tibias.
  sequential:         one leg at a time, each leg's three joints in lockstep
                      (same as running swing_arm.py --mode together per leg).

Servo IDs follow the 2-digit scheme (digit 1 = leg, digit 2 = joint), e.g.
legs 1 2 -> servos 11 12 13 21 22 23.  "Far right"/"far left" are each
servo's firmware angle limits, as in swing_arm.py.

Each serial port is opened once and shared by every servo found on it.  If
any requested servo is missing the run aborts before anything moves, rather
than swinging a partial set.

Power: in together mode 3 x N servos can be loaded at the same moment --
keep an eye on the 5 A bench supply with more than two legs, or use by-joint.

Usage:
    python3 utils/swing_legs.py 1 2
    python3 utils/swing_legs.py 1 2 3 --mode by-joint
    python3 utils/swing_legs.py 1 2 --mode sequential --speed 300
    python3 utils/swing_legs.py 1 2 --ports /dev/ttyACM0 /dev/ttyACM1

Torque is left ON at center when the sweep finishes.
Ctrl-C releases torque on every servo.

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from swing_arm import (
    ADDR_TORQUE_ENABLE,
    BAUDRATE,
    DEFAULT_PORTS,
    DEFAULT_SPEED,
    JOINTS,
    ServoError,
    swing_together,
    write1,
)


def find_servos(ports, packet, servo_ids):
    """Open each port once and ping every still-unfound servo on it.
    Returns ({sid: PortHandler}, [opened PortHandlers])."""
    found = {}
    opened = []
    for device in ports:
        remaining = [sid for sid in servo_ids if sid not in found]
        if not remaining:
            break
        if not os.path.exists(device):
            print(f"  {device}: not present")
            continue
        port = PortHandler(device)
        port.baudrate = BAUDRATE
        try:
            if not port.openPort():
                print(f"  {device}: failed to open")
                continue
        except Exception as exc:
            print(f"  {device}: failed to open ({exc})")
            continue
        here = []
        for sid in remaining:
            _model, comm, _err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found[sid] = port
                here.append(sid)
        if here:
            print(f"  {device}: servos {' '.join(map(str, here))}")
            opened.append(port)
        else:
            print(f"  {device}: none of the requested servos")
            port.closePort()
    return found, opened


def release_all(packet, joints):
    for joint_name, sid, port in joints:
        try:
            write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, f"release torque ({joint_name})")
        except ServoError as exc:
            print(f"  ⚠ could not release torque on {joint_name}: {exc}")


def main():
    parser = argparse.ArgumentParser(
        description="Exercise all three joints of several CRABORA legs.")
    parser.add_argument("legs", type=int, nargs="+", help="leg numbers, 1-6 (e.g. 1 2 3)")
    parser.add_argument("--mode", choices=["together", "by-joint", "sequential"], default="together",
                        help="together: every joint of every leg at once (default). "
                             "by-joint: all coxas, then all femurs, then all tibias. "
                             "sequential: one leg at a time, its joints in lockstep.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    legs = list(dict.fromkeys(args.legs))  # de-dupe, keep order
    if any(not 1 <= leg <= 6 for leg in legs):
        parser.error("legs must be 1-6")
    if not 1 <= args.speed <= 3400:
        parser.error("speed must be 1-3400 steps/s")

    packet = PacketHandler(0)  # STS servos use protocol_end=0

    # (leg, joint_digit, joint_name, sid) for every requested servo
    wanted = [(leg, digit, name, leg * 10 + digit) for leg in legs for digit, name in JOINTS]

    print(f"=== Legs {' '.join(map(str, legs))} -- {args.mode} mode ===")
    print(f"Looking for servos {' '.join(str(w[3]) for w in wanted)} at {BAUDRATE} baud:")
    found, opened = find_servos(args.ports, packet, [w[3] for w in wanted])

    missing = [w for w in wanted if w[3] not in found]
    if missing:
        for leg, _digit, name, sid in missing:
            print(f"✗ Servo {sid} (leg {leg} {name}) not found on any port -- check wiring, power and ID.")
        for port in opened:
            port.closePort()
        print("\n✗ Not all requested servos were found -- aborting.")
        return 1

    # joints as swing_together wants them: (name, sid, port)
    joints = [(f"L{leg} {name}", sid, found[sid]) for leg, _digit, name, sid in wanted]

    if args.mode == "together":
        groups = [("all legs", joints)]
    elif args.mode == "by-joint":
        groups = [(f"all {name}s", [j for j, w in zip(joints, wanted) if w[1] == digit])
                  for digit, name in JOINTS]
    else:  # sequential
        groups = [(f"leg {leg}", [j for j, w in zip(joints, wanted) if w[0] == leg])
                  for leg in legs]

    try:
        for label, group in groups:
            print(f"\n=== {label}: {', '.join(f'{n}={s}' for n, s, _p in group)} ===")
            swing_together(packet, group, args.speed)
        print("\n✓ Sweep complete. Torque left ON at center on all joints.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted -- releasing torque on all joints.")
        release_all(packet, joints)
        return 130
    except ServoError as exc:
        print(f"\n✗ {exc}")
        return 1
    finally:
        for port in opened:
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
