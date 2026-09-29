#!/usr/bin/env python3
"""
Release torque on one Feetech STS servo (e.g. STS3215) so it can be moved
freely by hand.

Usage:
    python3 utils/go_limp.py 41
    python3 utils/go_limp.py 41 --ports /dev/ttyACM0

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
BAUDRATE = 1_000_000

# --- Register addresses (STS3215 / common Feetech map) ---
ADDR_TORQUE_ENABLE = 40  # 1 byte


class ServoError(Exception):
    pass


def check(packet, comm, err, what):
    if comm != COMM_SUCCESS:
        raise ServoError(f"{what}: {packet.getTxRxResult(comm)}")
    if err:
        print(f"  ⚠ {what}: servo reported {packet.getRxPacketError(err)}")


def write1(port, packet, sid, addr, val, what):
    comm, err = packet.write1ByteTxRx(port, sid, addr, val)
    check(packet, comm, err, what)


def find_servo(ports, packet, servo_id):
    """Open each port in turn and ping servo_id. Returns (device, PortHandler) or (None, None)."""
    for device in ports:
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
        _model, comm, _err = packet.ping(port, servo_id)
        if comm == COMM_SUCCESS:
            print(f"  {device}: servo {servo_id} found")
            return device, port
        print(f"  {device}: no answer from servo {servo_id}")
        port.closePort()
    return None, None


def main():
    parser = argparse.ArgumentParser(description="Release torque on one STS servo (go limp).")
    parser.add_argument("servo_id", type=int, help="servo ID, e.g. 41")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    if not 0 <= args.servo_id <= 253:
        parser.error("servo ID must be 0-253")

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    sid = args.servo_id

    print(f"Looking for servo {sid} at {BAUDRATE} baud:")
    device, port = find_servo(args.ports, packet, sid)
    if port is None:
        print(f"\n✗ Servo {sid} not found on any port -- check wiring, power and ID.")
        return 1

    try:
        write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
        print(f"\n✓ Servo {sid} on {device}: torque released, servo is limp.")
        return 0
    except ServoError as exc:
        print(f"\n✗ {exc}")
        return 1
    finally:
        port.closePort()


if __name__ == "__main__":
    sys.exit(main())
