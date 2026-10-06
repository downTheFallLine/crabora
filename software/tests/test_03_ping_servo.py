#!/usr/bin/env python3
"""
CRABORA bring-up test 3: ping servo(s) by ID
==============================================

Runs on the Pi, after test 2 has shown the FE-URT-2 boards are visible
and openable. Speaks Feetech protocol for the first time: pings each
requested servo ID on every URT and reports which board it answered on
and its model number. If test 2 passes but this one fails, the problem
is downstream of the URT -- servo power, bus wiring, or the servo's ID /
baud rate.

Servo IDs follow the 2-digit scheme (digit 1 = leg, digit 2 = joint:
1 coxa, 2 femur, 3 tibia), e.g. 41 = leg 4 coxa.

What it does:
  1. Finds the URT boards with ../urt_lib.py (or uses --ports).
  2. Opens each board once at the bus baud rate (1 Mbps).
  3. Pings every requested ID on every board.
  4. PASSes if every ID answered on exactly one board. FAILs if any ID
     didn't answer anywhere, or answered on more than one board (two
     servos sharing an ID, one on each bus).

Nothing moves and torque is not touched -- ping is read-only.

Usage:
  python3 test_03_ping_servo.py 41
  python3 test_03_ping_servo.py 11 12 13 21 22 23
  python3 test_03_ping_servo.py 41 --ports /dev/ttyACM0

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ImportError:
    print("FAIL: scservo_sdk is not installed.")
    print("  Install it with:  python3 -m pip install feetech-servo-sdk")
    sys.exit(1)

from urt_lib import DEFAULT_BAUDRATE, find_urt_devices  # noqa: E402


def open_port(device):
    """Open a URT at the bus baud rate. Returns a PortHandler or None."""
    port = PortHandler(device)
    port.baudrate = DEFAULT_BAUDRATE
    try:
        if port.openPort():
            return port
        print(f"  {device}: failed to open")
    except Exception as e:  # pyserial raises on permissions, busy port, etc.
        print(f"  {device}: failed to open ({e})")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", type=int, nargs="+", help="servo ID(s) to ping, 0-253")
    ap.add_argument("--ports", nargs="+",
                     help="serial ports to try (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    ids = list(dict.fromkeys(args.ids))  # de-dupe, keep order
    if any(not 0 <= sid <= 253 for sid in ids):
        ap.error("servo IDs must be 0-253")

    print("Crabora bring-up test 3: ping servo(s) by ID")
    print(f"  pinging {' '.join(map(str, ids))} at {DEFAULT_BAUDRATE} baud...\n")

    devices = args.ports or find_urt_devices()
    if not devices:
        print("RESULT: FAIL -- no FE-URT-2 boards found. Run test 2 first.")
        sys.exit(1)

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    answers = {sid: [] for sid in ids}  # sid -> [(device, model)]
    for device in devices:
        port = open_port(device)
        if port is None:
            continue
        try:
            for sid in ids:
                model, comm, _err = packet.ping(port, sid)
                if comm == COMM_SUCCESS:
                    answers[sid].append((device, model))
        finally:
            port.closePort()

    ok = True
    for sid in ids:
        hits = answers[sid]
        if len(hits) == 1:
            device, model = hits[0]
            print(f"  servo {sid:3d}: OK    on {device}  (model {model})")
        elif not hits:
            ok = False
            print(f"  servo {sid:3d}: FAIL  no answer on any board")
        else:
            ok = False
            print(f"  servo {sid:3d}: FAIL  answered on {len(hits)} boards -- duplicate ID:")
            for device, model in hits:
                print(f"               {device}  (model {model})")
    print()

    if ok:
        print(f"RESULT: PASS -- all {len(ids)} servo(s) answered.")
        sys.exit(0)
    print("RESULT: FAIL -- see above.")
    print("Checklist:")
    print("    -- is servo power on? (URT USB alone doesn't power the bus)")
    print("    -- bus cable seated at both ends?")
    print("    -- right ID? servos ship as ID 1 until re-IDed")
    print("    -- baud rate set to 1 Mbps on the servo?")
    sys.exit(1)


if __name__ == "__main__":
    main()
