#!/usr/bin/env python3
"""
Sweep one Feetech STS servo (e.g. STS3215): center -> far right -> far left
-> center, pausing one second after each move.

"Far right" and "far left" are the servo's firmware angle limits (max and
min position registers), not the raw 0..4095 range, so a joint that has had
its limits set with set_limits.py won't be driven into the frame.

Usage:
    python3 utils/sweep_servo.py 41
    python3 utils/sweep_servo.py 41 --speed 300
    python3 utils/sweep_servo.py 41 --ports /dev/ttyACM0

Torque is left ON at center when the sweep finishes. Ctrl-C releases torque.

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
BAUDRATE = 1_000_000

CENTER_POSITION = 2048
DEFAULT_SPEED = 500        # steps/s (~44 deg/s) -- conservative
DWELL_SECONDS = 1.0        # pause after each move
MOVE_TIMEOUT = 10.0        # give up waiting for arrival after this long
POSITION_TOLERANCE = 20    # counts (~1.8 deg)

# --- Register addresses (STS3215 / common Feetech map) ---
ADDR_MIN_ANGLE       = 9   # 2 bytes
ADDR_MAX_ANGLE       = 11  # 2 bytes
ADDR_TORQUE_ENABLE   = 40  # 1 byte
ADDR_ACCELERATION    = 41  # 1 byte
ADDR_GOAL_POSITION   = 42  # 2 bytes
ADDR_GOAL_SPEED      = 46  # 2 bytes
ADDR_PRESENT_POSITION = 56 # 2 bytes
ADDR_MOVING          = 66  # 1 byte


class ServoError(Exception):
    pass


def check(packet, comm, err, what):
    if comm != COMM_SUCCESS:
        raise ServoError(f"{what}: {packet.getTxRxResult(comm)}")
    if err:
        # Servo answered but flagged a status error (overload, voltage...)
        print(f"  ⚠ {what}: servo reported {packet.getRxPacketError(err)}")


def read1(port, packet, sid, addr, what):
    val, comm, err = packet.read1ByteTxRx(port, sid, addr)
    check(packet, comm, err, what)
    return val


def read2(port, packet, sid, addr, what):
    val, comm, err = packet.read2ByteTxRx(port, sid, addr)
    check(packet, comm, err, what)
    return val


def write1(port, packet, sid, addr, val, what):
    comm, err = packet.write1ByteTxRx(port, sid, addr, val)
    check(packet, comm, err, what)


def write2(port, packet, sid, addr, val, what):
    comm, err = packet.write2ByteTxRx(port, sid, addr, val)
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


def move_to(port, packet, sid, target, label):
    print(f"\n-> {label} ({target})")
    write2(port, packet, sid, ADDR_GOAL_POSITION, target, "write goal position")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)  # let the servo start moving before polling
    while True:
        pos = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
        moving = read1(port, packet, sid, ADDR_MOVING, "read moving flag")
        if not moving and abs(pos - target) <= POSITION_TOLERANCE:
            print(f"   arrived at {pos}")
            break
        if time.monotonic() > deadline:
            print(f"   ⚠ timed out at {pos} (target {target}) -- binding or stalled?")
            break
        time.sleep(0.02)

    time.sleep(DWELL_SECONDS)
    return pos


def to_degrees(pos):
    """Angle relative to center, in degrees (4096 counts per revolution)."""
    return (pos - CENTER_POSITION) * 360.0 / 4096


def main():
    parser = argparse.ArgumentParser(description="Sweep one STS servo center -> right -> left -> center.")
    parser.add_argument("servo_id", type=int, help="servo ID, e.g. 41")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    if not 0 <= args.servo_id <= 253:
        parser.error("servo ID must be 0-253")
    if not 1 <= args.speed <= 3400:
        parser.error("speed must be 1-3400 steps/s")

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    sid = args.servo_id

    print(f"Looking for servo {sid} at {BAUDRATE} baud:")
    device, port = find_servo(args.ports, packet, sid)
    if port is None:
        print(f"\n✗ Servo {sid} not found on any port -- check wiring, power and ID.")
        return 1

    try:
        lo = read2(port, packet, sid, ADDR_MIN_ANGLE, "read min limit")
        hi = read2(port, packet, sid, ADDR_MAX_ANGLE, "read max limit")
        start = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
        print(f"\nServo {sid} on {device}:")
        print(f"  Left  (min limit): {lo:5d}  ({to_degrees(lo):+7.1f} deg)")
        print(f"  Center           : {CENTER_POSITION:5d}  ({to_degrees(CENTER_POSITION):+7.1f} deg)")
        print(f"  Right (max limit): {hi:5d}  ({to_degrees(hi):+7.1f} deg)")
        print(f"  Current position : {start:5d}  ({to_degrees(start):+7.1f} deg)")
        if lo >= hi:
            print("✗ Limits are not a valid range (servo may be in wheel/multi-turn mode). Aborting.")
            return 1
        if not lo <= CENTER_POSITION <= hi:
            print(f"✗ Center {CENTER_POSITION} is outside the limits. Aborting.")
            return 1

        write2(port, packet, sid, ADDR_GOAL_SPEED, args.speed, "write speed")
        write1(port, packet, sid, ADDR_TORQUE_ENABLE, 1, "enable torque")

        moves = [
            ("Center", CENTER_POSITION),
            ("Right",  hi),
            ("Left",   lo),
            ("Center", CENTER_POSITION),
        ]
        results = []
        for label, target in moves:
            actual = move_to(port, packet, sid, target, label.lower())
            results.append((label, target, actual))

        print("\nSummary:")
        print(f"  {'Move':<7} {'Target':>6} {'Actual':>6} {'Error':>6} {'Angle':>9}")
        for label, target, actual in results:
            print(f"  {label:<7} {target:6d} {actual:6d} {actual - target:+6d} "
                  f"{to_degrees(actual):+8.1f}°")

        print("\n✓ Sweep complete. Torque left ON at center.")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted -- releasing torque.")
        try:
            write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
        except ServoError as exc:
            print(f"  ⚠ could not release torque: {exc}")
        return 130
    except ServoError as exc:
        print(f"\n✗ {exc}")
        return 1
    finally:
        port.closePort()


if __name__ == "__main__":
    sys.exit(main())
