#!/usr/bin/env python3
"""
Exercise all three joints of one CRABORA leg: coxa, femur, tibia -- each
swept center -> far right -> far left -> center in turn, pausing one second
after each move.

Servo IDs are derived from the leg number using the 2-digit ID scheme
(digit 1 = leg number, digit 2 = joint: 1 coxa, 2 femur, 3 tibia), e.g.
leg 4 -> servos 41 (coxa), 42 (femur), 43 (tibia).

"Far right" and "far left" are each servo's firmware angle limits (max and
min position registers), not the raw 0..4095 range, so a joint that has had
its limits set with set_limits.py won't be driven into the frame.

Usage:
    python3 utils/swing_arm.py 4
    python3 utils/swing_arm.py 4 --speed 300
    python3 utils/swing_arm.py 4 --ports /dev/ttyACM0

Torque is left ON at center on each joint when its sweep finishes.
Ctrl-C releases torque on whichever joint is currently moving.

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

JOINTS = [
    (1, "coxa"),
    (2, "femur"),
    (3, "tibia"),
]

# --- Register addresses (STS3215 / common Feetech map) ---
ADDR_MIN_ANGLE        = 9   # 2 bytes
ADDR_MAX_ANGLE        = 11  # 2 bytes
ADDR_TORQUE_ENABLE    = 40  # 1 byte
ADDR_ACCELERATION     = 41  # 1 byte
ADDR_GOAL_POSITION    = 42  # 2 bytes
ADDR_GOAL_SPEED       = 46  # 2 bytes
ADDR_PRESENT_POSITION = 56  # 2 bytes
ADDR_MOVING           = 66  # 1 byte


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
    print(f"\n  -> {label} ({target})")
    write2(port, packet, sid, ADDR_GOAL_POSITION, target, "write goal position")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)  # let the servo start moving before polling
    while True:
        pos = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
        moving = read1(port, packet, sid, ADDR_MOVING, "read moving flag")
        if not moving and abs(pos - target) <= POSITION_TOLERANCE:
            print(f"     arrived at {pos}")
            break
        if time.monotonic() > deadline:
            print(f"     ⚠ timed out at {pos} (target {target}) -- binding or stalled?")
            break
        time.sleep(0.02)

    time.sleep(DWELL_SECONDS)
    return pos


def to_degrees(pos):
    """Angle relative to center, in degrees (4096 counts per revolution)."""
    return (pos - CENTER_POSITION) * 360.0 / 4096


def swing_joint(port, packet, sid, speed, joint_name):
    lo = read2(port, packet, sid, ADDR_MIN_ANGLE, "read min limit")
    hi = read2(port, packet, sid, ADDR_MAX_ANGLE, "read max limit")
    start = read2(port, packet, sid, ADDR_PRESENT_POSITION, "read position")
    print(f"  Left  (min limit): {lo:5d}  ({to_degrees(lo):+7.1f} deg)")
    print(f"  Center           : {CENTER_POSITION:5d}  ({to_degrees(CENTER_POSITION):+7.1f} deg)")
    print(f"  Right (max limit): {hi:5d}  ({to_degrees(hi):+7.1f} deg)")
    print(f"  Current position : {start:5d}  ({to_degrees(start):+7.1f} deg)")
    if lo >= hi:
        raise ServoError(f"{joint_name}: limits are not a valid range (servo may be in wheel/multi-turn mode)")
    if not lo <= CENTER_POSITION <= hi:
        raise ServoError(f"{joint_name}: center {CENTER_POSITION} is outside the limits")

    write2(port, packet, sid, ADDR_GOAL_SPEED, speed, "write speed")
    write1(port, packet, sid, ADDR_TORQUE_ENABLE, 1, "enable torque")

    moves = [
        ("center", CENTER_POSITION),
        ("right",  hi),
        ("left",   lo),
        ("center", CENTER_POSITION),
    ]
    results = []
    for label, target in moves:
        actual = move_to(port, packet, sid, target, label)
        results.append((label, target, actual))

    print(f"\n  Summary ({joint_name}):")
    print(f"    {'Move':<7} {'Target':>6} {'Actual':>6} {'Error':>6} {'Angle':>9}")
    for label, target, actual in results:
        print(f"    {label:<7} {target:6d} {actual:6d} {actual - target:+6d} "
              f"{to_degrees(actual):+8.1f}°")


def main():
    parser = argparse.ArgumentParser(
        description="Exercise all three joints (coxa, femur, tibia) of one CRABORA leg.")
    parser.add_argument("leg", type=int, help="leg number, 1-6")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    if not 1 <= args.leg <= 6:
        parser.error("leg must be 1-6")
    if not 1 <= args.speed <= 3400:
        parser.error("speed must be 1-3400 steps/s")

    packet = PacketHandler(0)  # STS servos use protocol_end=0

    exit_code = 0
    for joint_digit, joint_name in JOINTS:
        sid = args.leg * 10 + joint_digit
        print(f"\n=== Leg {args.leg} {joint_name} (servo {sid}) ===")
        print(f"Looking for servo {sid} at {BAUDRATE} baud:")
        device, port = find_servo(args.ports, packet, sid)
        if port is None:
            print(f"\n✗ Servo {sid} not found on any port -- check wiring, power and ID.")
            exit_code = 1
            continue

        try:
            print(f"\nServo {sid} ({joint_name}) on {device}:")
            swing_joint(port, packet, sid, args.speed, joint_name)
            print(f"\n✓ {joint_name} sweep complete. Torque left ON at center.")
        except KeyboardInterrupt:
            print(f"\nInterrupted -- releasing torque on {joint_name}.")
            try:
                write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
            except ServoError as exc:
                print(f"  ⚠ could not release torque: {exc}")
            port.closePort()
            return 130
        except ServoError as exc:
            print(f"\n✗ {exc}")
            exit_code = 1
        finally:
            port.closePort()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
