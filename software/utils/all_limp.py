#!/usr/bin/env python3
"""
Release torque on EVERY servo at once -- the panic button.

Supersedes all_limp.sh, which drove go_limp.py once per servo. That had
two problems: it spawned a process per joint, each re-scanning the
serial ports before doing anything, so the robot stayed rigid for
several seconds; and it listed only 15 of the 18 servos -- leg 3 was
missing, so the one thing you reach for when the robot is misbehaving
left a whole leg clamped and loaded.

This opens each port once, finds every joint, and drops torque on all
of them in a single pass. It reports anything that did not release,
because a servo still holding is exactly what you need to know about.

⚠ THE ROBOT WILL DROP. If it is standing, support it first -- going
limp under its own weight is how legs get bent and gears get stripped.

Usage:
    python3 utils/all_limp.py
    python3 utils/all_limp.py --legs 1 2
    python3 utils/all_limp.py --yes          # no confirmation
    python3 utils/all_limp.py --ports /dev/ttyACM0

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, JOINT_NAME,
    TIBIA_JOINT_DIGIT, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    find_joints, find_urt_devices, open_ports, read1, write1,
)

JOINTS = (COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT)


def main():
    ap = argparse.ArgumentParser(
        description="Release torque on every servo at once.")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--yes", "-y", action="store_true",
                    help="skip the confirmation")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    print(f"Releasing torque on legs {', '.join(map(str, legs))}")
    if not args.yes:
        print("⚠ The robot WILL drop. Support it first if it is standing.")
        try:
            if input("  Continue? [y/N] ").strip().lower() != "y":
                print("  Stopped. Nothing released.")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n  Stopped. Nothing released.")
            return 1

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    released, failed, missing = [], [], []
    try:
        for digit in JOINTS:
            found = find_joints(packet, ports, legs, digit, quiet=True)
            for leg in legs:
                sid = leg * 10 + digit
                if leg not in found:
                    missing.append(sid)
                    continue
                _sid, port = found[leg]
                try:
                    write1(packet, port, sid, ADDR_TORQUE_ENABLE, 0, "release")
                    # Confirm rather than assume: a servo that ignored the
                    # write is the whole reason to run this.
                    if read1(packet, port, sid, ADDR_TORQUE_ENABLE,
                             "verify") == 0:
                        released.append(sid)
                    else:
                        failed.append((sid, "still reports torque ON"))
                except BusError as e:
                    failed.append((sid, str(e)))

        print(f"\n  released {len(released)} of {len(legs) * 3}: "
              + " ".join(map(str, sorted(released))))
        if missing:
            print(f"  did not answer: {' '.join(map(str, sorted(missing)))}")
        if failed:
            print(f"\n  ⚠ STILL HOLDING:")
            for sid, why in failed:
                print(f"      servo {sid} ({sid // 10}-"
                      f"{JOINT_NAME[sid % 10]}): {why}")
            print("  Cut the servo power if you need these released now.")
            return 1
        if missing:
            print("\n  Everything that answered is limp, but some servos were")
            print("  not found -- check they are powered before trusting this.")
            return 1
        print("\n  All limp.")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted -- some servos may still be holding.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
