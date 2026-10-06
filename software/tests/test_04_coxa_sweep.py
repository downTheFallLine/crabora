#!/usr/bin/env python3
"""
CRABORA bring-up test 4: coxa sweep endurance
===============================================

Runs on the Pi, after test 3 has shown the servos answer a ping. Sweeps
every coxa joint (servo IDs x1 -- 11, 21, 31, ...) back and forth
between its firmware angle limits for --reps repetitions, then parks
them at center. All coxas move together, in lockstep.

This is the first test that makes the robot MOVE. The legs will swing
through their full travel -- clear the area, make sure the body is on a
stand or its side, and keep an eye on the bench supply. See the power
note below.

What it checks, per rep and per joint:
  - the joint actually arrives at the commanded limit (within tolerance)
  - it gets there without timing out (a timeout means binding or stall --
    e.g. a cap rubbing the horn, or a leg hitting its neighbour)
  - the servo's error register stays clear (no overload, over-temp,
    over-current or voltage faults)
It also reports each servo's temperature and worst position error at the
end, so a joint that is dragging shows up next to its healthy peers.

PASSes if every joint completed every rep cleanly. FAILs on a missing
servo, a timeout, or an error flag. WARNs on a high temperature.

Power: all N coxas accelerate at the same instant. On the 5 A bench
supply keep to 2-3 legs, or use --speed to slow them down; six coxas
starting together can trip the supply.

By default it sweeps whichever coxas answer on the bus, so it works with
a partly wired robot. Pass --legs to require specific legs and fail if
one is missing.

Usage:
  python3 test_04_coxa_sweep.py                    # all coxas found, 3 reps
  python3 test_04_coxa_sweep.py --reps 20          # endurance run
  python3 test_04_coxa_sweep.py --legs 1 2         # require legs 1 and 2
  python3 test_04_coxa_sweep.py --speed 200        # gentler on the supply

Torque is left ON at center on a pass, so the legs hold their pose.
Ctrl-C releases torque on every joint.

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ImportError:
    print("FAIL: scservo_sdk is not installed.")
    print("  Install it with:  python3 -m pip install feetech-servo-sdk")
    sys.exit(1)

from urt_lib import DEFAULT_BAUDRATE, find_urt_devices  # noqa: E402

COXA_JOINT_DIGIT = 1          # 2-digit ID scheme: digit 2 = 1 means coxa
ALL_LEGS = range(1, 7)

CENTER_POSITION = 2048
DEFAULT_SPEED = 500           # steps/s (~44 deg/s) -- conservative
DEFAULT_REPS = 3
DWELL_SECONDS = 0.5           # pause at each end of the sweep
MOVE_TIMEOUT = 10.0           # give up waiting for arrival after this long
POSITION_TOLERANCE = 20       # counts (~1.8 deg)
TEMP_WARN_C = 55              # STS3215 shuts down around 70 C

# --- Register addresses (STS3215 / common Feetech map) ---
ADDR_MIN_ANGLE        = 9     # 2 bytes
ADDR_MAX_ANGLE        = 11    # 2 bytes
ADDR_TORQUE_ENABLE    = 40    # 1 byte
ADDR_GOAL_POSITION    = 42    # 2 bytes
ADDR_GOAL_SPEED       = 46    # 2 bytes
ADDR_PRESENT_POSITION = 56    # 2 bytes
ADDR_PRESENT_TEMP     = 63    # 1 byte, degrees C
ADDR_SERVO_STATUS     = 65    # 1 byte, error bitmask
ADDR_MOVING           = 66    # 1 byte

ERROR_FLAGS = {
    0: "Voltage",
    1: "Sensor",
    2: "Temperature",
    3: "Current",
    4: "Angle",
    5: "Overload",
}


class BusError(Exception):
    pass


def to_degrees(pos):
    """Angle relative to center, in degrees (4096 counts per revolution)."""
    return (pos - CENTER_POSITION) * 360.0 / 4096


def decode_errors(status):
    return [name for bit, name in ERROR_FLAGS.items() if status & (1 << bit)]


def read1(packet, port, sid, addr, what):
    val, comm, _err = packet.read1ByteTxRx(port, sid, addr)
    if comm != COMM_SUCCESS:
        raise BusError(f"servo {sid}: {what}: {packet.getTxRxResult(comm)}")
    return val


def read2(packet, port, sid, addr, what):
    val, comm, _err = packet.read2ByteTxRx(port, sid, addr)
    if comm != COMM_SUCCESS:
        raise BusError(f"servo {sid}: {what}: {packet.getTxRxResult(comm)}")
    return val


def write1(packet, port, sid, addr, val, what):
    comm, _err = packet.write1ByteTxRx(port, sid, addr, val)
    if comm != COMM_SUCCESS:
        raise BusError(f"servo {sid}: {what}: {packet.getTxRxResult(comm)}")


def write2(packet, port, sid, addr, val, what):
    comm, _err = packet.write2ByteTxRx(port, sid, addr, val)
    if comm != COMM_SUCCESS:
        raise BusError(f"servo {sid}: {what}: {packet.getTxRxResult(comm)}")


def open_ports(devices):
    """Open each URT at the bus baud rate. Returns {device: PortHandler}."""
    ports = {}
    for device in devices:
        port = PortHandler(device)
        port.baudrate = DEFAULT_BAUDRATE
        try:
            if port.openPort():
                ports[device] = port
            else:
                print(f"  {device}: failed to open")
        except Exception as e:  # pyserial raises on permissions, busy port, etc.
            print(f"  {device}: failed to open ({e})")
    return ports


def find_coxas(packet, ports, legs):
    """Ping each leg's coxa on every open port. Returns ({sid: port}, [missing sids])."""
    found = {}
    for leg in legs:
        sid = leg * 10 + COXA_JOINT_DIGIT
        for device, port in ports.items():
            _model, comm, _err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found[sid] = port
                print(f"  servo {sid} (leg {leg} coxa): found on {device}")
                break
    missing = [leg * 10 + COXA_JOINT_DIGIT for leg in legs
               if leg * 10 + COXA_JOINT_DIGIT not in found]
    return found, missing


def move_all(packet, joints, targets):
    """Send every joint to its own target, wait for all to arrive.
    Returns {sid: (position, timed_out)}."""
    for sid, port in joints.items():
        write2(packet, port, sid, ADDR_GOAL_POSITION, targets[sid], "write goal position")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)  # let the servos start moving before polling
    results = {}
    pending = set(joints)
    while pending:
        for sid in list(pending):
            port = joints[sid]
            pos = read2(packet, port, sid, ADDR_PRESENT_POSITION, "read position")
            moving = read1(packet, port, sid, ADDR_MOVING, "read moving flag")
            if not moving and abs(pos - targets[sid]) <= POSITION_TOLERANCE:
                results[sid] = (pos, False)
                pending.discard(sid)
            else:
                results[sid] = (pos, False)
        if pending and time.monotonic() > deadline:
            for sid in pending:
                pos, _ = results[sid]
                results[sid] = (pos, True)
            break
        if pending:
            time.sleep(0.02)

    time.sleep(DWELL_SECONDS)
    return results


def release_all(packet, joints):
    for sid, port in joints.items():
        try:
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
        except BusError as e:
            print(f"  ⚠ {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=DEFAULT_REPS,
                     help=f"back-and-forth sweeps per joint (default: {DEFAULT_REPS})")
    ap.add_argument("--legs", type=int, nargs="+",
                     help="leg numbers whose coxas must be present, 1-6 "
                          "(default: sweep whichever coxas answer)")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                     help=f"move speed in steps/s (default: {DEFAULT_SPEED})")
    ap.add_argument("--ports", nargs="+",
                     help="serial ports to use (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    required = args.legs is not None
    legs = list(dict.fromkeys(args.legs)) if required else list(ALL_LEGS)
    if any(not 1 <= leg <= 6 for leg in legs):
        ap.error("--legs must be 1-6")

    print("Crabora bring-up test 4: coxa sweep endurance")
    print(f"  {args.reps} rep(s) at {args.speed} steps/s, "
          f"legs {'/'.join(map(str, legs))}"
          f"{'' if required else ' (whichever answer)'}\n")

    devices = args.ports or find_urt_devices()
    if not devices:
        print("RESULT: FAIL -- no FE-URT-2 boards found. Run test 2 first.")
        sys.exit(1)

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    ports = open_ports(devices)
    if not ports:
        print("RESULT: FAIL -- no URT could be opened.")
        sys.exit(1)

    joints = {}
    try:
        joints, missing = find_coxas(packet, ports, legs)
        if required and missing:
            for sid in missing:
                print(f"  servo {sid} (leg {sid // 10} coxa): NOT FOUND")
            print("\nRESULT: FAIL -- a requested coxa did not answer. Run test 3.")
            sys.exit(1)
        if not joints:
            print("\nRESULT: FAIL -- no coxa servos answered at all. Run test 3.")
            sys.exit(1)

        # Limits come from the servos themselves, so a joint whose travel has
        # been set with set_center.py is never driven into the frame.
        limits = {}
        print()
        for sid, port in joints.items():
            lo = read2(packet, port, sid, ADDR_MIN_ANGLE, "read min limit")
            hi = read2(packet, port, sid, ADDR_MAX_ANGLE, "read max limit")
            print(f"  servo {sid}: limits {lo}..{hi}  "
                  f"({to_degrees(lo):+.1f}° .. {to_degrees(hi):+.1f}°)")
            if lo >= hi:
                print(f"\nRESULT: FAIL -- servo {sid} has no valid travel range "
                      f"({lo}..{hi}); it may be in wheel mode. Run set_center.py.")
                sys.exit(1)
            if not lo <= CENTER_POSITION <= hi:
                print(f"\nRESULT: FAIL -- servo {sid} center {CENTER_POSITION} is "
                      f"outside its limits {lo}..{hi}. Run set_center.py.")
                sys.exit(1)
            limits[sid] = (lo, hi)

        for sid, port in joints.items():
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "write speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "enable torque")

        # Start from center so every rep covers the same travel.
        print("\n  centering...")
        move_all(packet, joints, {sid: CENTER_POSITION for sid in joints})

        worst_error = {sid: 0 for sid in joints}
        stalled = set()
        for rep in range(1, args.reps + 1):
            print(f"  rep {rep}/{args.reps}:", end="", flush=True)
            for label, pick in (("right", 1), ("left", 0)):
                targets = {sid: limits[sid][pick] for sid in joints}
                results = move_all(packet, joints, targets)
                bad = []
                for sid, (pos, timed_out) in results.items():
                    err = abs(pos - targets[sid])
                    worst_error[sid] = max(worst_error[sid], err)
                    if timed_out:
                        stalled.add(sid)
                        bad.append(f"{sid} stalled at {pos} (target {targets[sid]})")
                print(f" {label}{'' if not bad else ' ✗'}", end="", flush=True)
                for msg in bad:
                    print(f"\n    ⚠ servo {msg} -- binding or stalled?",
                          end="", flush=True)
            print()

        print("\n  parking at center...")
        move_all(packet, joints, {sid: CENTER_POSITION for sid in joints})

        # --- per-joint report ------------------------------------------
        print(f"\n  {'servo':>5}  {'worst err':>9}  {'temp':>5}  status")
        faulted = {}
        hot = []
        for sid, port in sorted(joints.items()):
            temp = read1(packet, port, sid, ADDR_PRESENT_TEMP, "read temperature")
            status = read1(packet, port, sid, ADDR_SERVO_STATUS, "read status")
            flags = decode_errors(status)
            if flags:
                faulted[sid] = flags
            if temp >= TEMP_WARN_C:
                hot.append((sid, temp))
            note = ", ".join(flags) if flags else "ok"
            print(f"  {sid:>5}  {worst_error[sid]:>6d} ct  {temp:>3d}°C  {note}")

        print()
        for sid, temp in hot:
            print(f"WARN: servo {sid} is at {temp}°C (warn at {TEMP_WARN_C}°C, "
                  f"shutdown near 70°C). Let it cool before another run.")
        if hot:
            print()

        ok = not stalled and not faulted
        if ok:
            print(f"RESULT: PASS -- {len(joints)} coxa(s) completed "
                  f"{args.reps} rep(s). Torque left ON at center.")
            return 0

        if stalled:
            print(f"RESULT: FAIL -- servo(s) {', '.join(map(str, sorted(stalled)))} "
                  f"did not reach a limit in {MOVE_TIMEOUT:.0f}s.")
        if faulted:
            for sid, flags in sorted(faulted.items()):
                print(f"RESULT: FAIL -- servo {sid} error flags: {', '.join(flags)}.")
        print("Checklist:")
        print("    -- does the joint turn freely by hand? (go_limp.py, then feel")
        print("       for a tight spot -- a cap rubbing the horn will stall it)")
        print("    -- is a leg hitting its neighbour or the frame? re-run")
        print("       set_center.py with a narrower swing")
        print("    -- supply sagging with all coxas starting at once? --speed 200")
        print("    -- Overload/Current flags latch: power-cycle or reset to clear")
        return 1

    except KeyboardInterrupt:
        print("\n\nInterrupted -- releasing torque on all joints.")
        release_all(packet, joints)
        return 130
    except BusError as e:
        print(f"\nRESULT: FAIL -- bus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
