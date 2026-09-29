#!/usr/bin/env python3
"""
Exercise all three joints of one CRABORA leg: coxa, femur, tibia -- each
swept center -> far right -> far left -> center, pausing one second after
each move.  Two modes:

  sequential (default): one joint at a time, full sweep before the next.
  together:              all three joints move at once, in lockstep --
                          center, then all three to their own "right" limit,
                          then all three to their own "left" limit, then
                          back to center.

Servo IDs are derived from the leg number using the 2-digit ID scheme
(digit 1 = leg number, digit 2 = joint: 1 coxa, 2 femur, 3 tibia), e.g.
leg 4 -> servos 41 (coxa), 42 (femur), 43 (tibia).

"Far right" and "far left" are each servo's firmware angle limits (max and
min position registers), not the raw 0..4095 range, so a joint that has had
its limits set with set_limits.py won't be driven into the frame.

Usage:
    python3 utils/swing_arm.py 4
    python3 utils/swing_arm.py 4 --mode together
    python3 utils/swing_arm.py 4 --speed 300
    python3 utils/swing_arm.py 4 --ports /dev/ttyACM0

Torque is left ON at center on each joint when its sweep finishes.
Ctrl-C releases torque on whichever joint(s) are currently moving.

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


def move_all(packet, joints, targets, label):
    """Move several servos to their own target at the same time, then wait
    for all of them to arrive (or time out). `joints` is a list of
    (joint_name, sid, port); `targets` maps joint_name -> target position."""
    print(f"\n  -> {label}")
    for joint_name, sid, port in joints:
        write2(port, packet, sid, ADDR_GOAL_POSITION, targets[joint_name],
               f"write goal position ({joint_name})")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)  # let the servos start moving before polling
    positions = {joint_name: None for joint_name, _sid, _port in joints}
    pending = set(positions)
    while pending:
        for joint_name, sid, port in joints:
            if joint_name not in pending:
                continue
            pos = read2(port, packet, sid, ADDR_PRESENT_POSITION, f"read position ({joint_name})")
            moving = read1(port, packet, sid, ADDR_MOVING, f"read moving flag ({joint_name})")
            positions[joint_name] = pos
            if not moving and abs(pos - targets[joint_name]) <= POSITION_TOLERANCE:
                print(f"     {joint_name} arrived at {pos}")
                pending.discard(joint_name)
        if pending and time.monotonic() > deadline:
            for joint_name in pending:
                print(f"     ⚠ {joint_name} timed out at {positions[joint_name]} "
                      f"(target {targets[joint_name]}) -- binding or stalled?")
            break
        if pending:
            time.sleep(0.02)

    time.sleep(DWELL_SECONDS)
    return positions


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


def swing_together(packet, joints, speed):
    """Sweep several already-opened servos in lockstep: center -> each to its
    own right limit -> each to its own left limit -> center. `joints` is a
    list of (joint_name, sid, port)."""
    limits = {}
    for joint_name, sid, port in joints:
        lo = read2(port, packet, sid, ADDR_MIN_ANGLE, f"read min limit ({joint_name})")
        hi = read2(port, packet, sid, ADDR_MAX_ANGLE, f"read max limit ({joint_name})")
        start = read2(port, packet, sid, ADDR_PRESENT_POSITION, f"read position ({joint_name})")
        print(f"  {joint_name:<6} left={lo:5d} ({to_degrees(lo):+7.1f} deg)  "
              f"center={CENTER_POSITION:5d}  right={hi:5d} ({to_degrees(hi):+7.1f} deg)  "
              f"now={start:5d} ({to_degrees(start):+7.1f} deg)")
        if lo >= hi:
            raise ServoError(f"{joint_name}: limits are not a valid range (servo may be in wheel/multi-turn mode)")
        if not lo <= CENTER_POSITION <= hi:
            raise ServoError(f"{joint_name}: center {CENTER_POSITION} is outside the limits")
        limits[joint_name] = (lo, hi)

    for joint_name, sid, port in joints:
        write2(port, packet, sid, ADDR_GOAL_SPEED, speed, f"write speed ({joint_name})")
        write1(port, packet, sid, ADDR_TORQUE_ENABLE, 1, f"enable torque ({joint_name})")

    steps = [
        ("center", {name: CENTER_POSITION for name in limits}),
        ("right",  {name: limits[name][1] for name in limits}),
        ("left",   {name: limits[name][0] for name in limits}),
        ("center", {name: CENTER_POSITION for name in limits}),
    ]
    history = []
    for label, targets in steps:
        actual = move_all(packet, joints, targets, label)
        history.append((label, targets, actual))

    print("\n  Summary (together):")
    print(f"    {'Move':<7} " + "  ".join(f"{name:>16}" for name, _sid, _port in joints))
    for label, targets, actual in history:
        cells = "  ".join(f"{actual[name]:6d} ({actual[name]-targets[name]:+4d})  "
                           for name, _sid, _port in joints)
        print(f"    {label:<7} {cells}")


def main():
    parser = argparse.ArgumentParser(
        description="Exercise all three joints (coxa, femur, tibia) of one CRABORA leg.")
    parser.add_argument("leg", type=int, help="leg number, 1-6")
    parser.add_argument("--mode", choices=["sequential", "together"], default="sequential",
                        help="sequential: one joint at a time (default). "
                             "together: all three joints move at once, in lockstep.")
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

    if args.mode == "sequential":
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

    # --mode together: find all three servos first; if any is missing, bail
    # out rather than swinging a partial leg.
    print(f"=== Leg {args.leg} -- together mode ===")
    joints = []
    found_ports = []
    exit_code = 0
    for joint_digit, joint_name in JOINTS:
        sid = args.leg * 10 + joint_digit
        print(f"Looking for servo {sid} ({joint_name}) at {BAUDRATE} baud:")
        device, port = find_servo(args.ports, packet, sid)
        if port is None:
            print(f"\n✗ Servo {sid} ({joint_name}) not found on any port -- check wiring, power and ID.")
            exit_code = 1
            continue
        joints.append((joint_name, sid, port))
        found_ports.append(port)

    if exit_code:
        for port in found_ports:
            port.closePort()
        print("\n✗ Not all three joints were found -- aborting (use --mode sequential to exercise the rest).")
        return exit_code

    try:
        print(f"\nLeg {args.leg}: {', '.join(f'{n}={s}' for n, s, _p in joints)}")
        swing_together(packet, joints, args.speed)
        print("\n✓ Together sweep complete. Torque left ON at center on all three joints.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted -- releasing torque on all joints.")
        for joint_name, sid, port in joints:
            try:
                write1(port, packet, sid, ADDR_TORQUE_ENABLE, 0, f"release torque ({joint_name})")
            except ServoError as exc:
                print(f"  ⚠ could not release torque on {joint_name}: {exc}")
        return 130
    except ServoError as exc:
        print(f"\n✗ {exc}")
        return 1
    finally:
        for port in found_ports:
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
