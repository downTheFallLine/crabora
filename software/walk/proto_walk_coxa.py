#!/usr/bin/env python3
"""
CRABORA proto-walk 1: coxa sweep, forward and rear
====================================================

The first locomotion-shaped motion. Coxas only -- no femurs, no tibias,
no weight on the legs. The robot does not move; this is about proving
that "forward" means forward on all six legs before any gait depends on
it.

Sequence, with an operator check at each stop:

  1. Centre every coxa, then PAUSE -- you confirm they really are centred
  2. Sweep every coxa FORWARD together, then PAUSE -- you confirm
  3. Sweep every coxa REAR together, then PAUSE -- you confirm
  4. Return to centre and go limp

Each pause prints the measured position of every joint and waits for a
'y'. Anything else aborts and drops torque, so the safe reflex (Enter,
Ctrl-C, walking away) is the one that stops the robot.

WHY THE SIGNS MATTER
--------------------
Legs 1-3 are the right side, 4-6 the left (see README.md). They are the
same physical parts rotated into place, so a given servo direction
swings them OPPOSITE ways in the body frame.

Deriving it: with leg n at bearing θ = 30° + (n-1)x60° clockwise from
the front point, a counter-clockwise coxa rotation moves that foot
forward by sin θ --

    leg 1 (30°)   sin = +0.50     leg 4 (210°)  sin = -0.50
    leg 2 (90°)   sin = +1.00     leg 5 (270°)  sin = -1.00
    leg 3 (150°)  sin = +0.50     leg 6 (330°)  sin = -0.50

So the right side wants CCW for forward and the left side wants CW:
FORWARD_SIGN below. Note legs 2 and 5 are the beam legs -- their coxa
swing is pure fore/aft, while the corner legs only contribute half as
much per degree.

⚠ FORWARD_SIGN IS UNVERIFIED ON HARDWARE. It assumes every coxa servo
is mounted the same way round, so that a rising count means the same
rotational sense on all six legs. If the legs go the wrong way, do NOT
patch individual legs -- run with --invert, confirm that fixes all six,
and then flip the table here. If only SOME legs go the wrong way, the
servos are not mounted consistently; fix that in hardware or record it
here with a comment, because a half-corrected table will haunt the gait
code forever.

Run --dry-run first. It prints every target without energising anything.

Usage:
  python3 proto_walk_coxa.py --dry-run
  python3 proto_walk_coxa.py --angle 15
  python3 proto_walk_coxa.py --angle 25 --speed 200
  python3 proto_walk_coxa.py --legs 1 2 --invert

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
    print("scservo_sdk is not installed.")
    print("  Install it with:  python3 -m pip install feetech-servo-sdk")
    sys.exit(1)

from urt_lib import DEFAULT_BAUDRATE, find_urt_devices  # noqa: E402

COXA_JOINT_DIGIT = 1
ALL_LEGS = (1, 2, 3, 4, 5, 6)

# +1 = counter-clockwise about body +Z moves this foot FORWARD.
# Right side (1-3) and left side (4-6) are mirrored -- see the module
# docstring. UNVERIFIED on hardware; --invert flips all six at once.
FORWARD_SIGN = {1: +1, 2: +1, 3: +1, 4: -1, 5: -1, 6: -1}

LEG_NAME = {
    1: "front-right", 2: "right", 3: "rear-right",
    4: "rear-left",   5: "left",  6: "front-left",
}

CENTER_POSITION = 2048
COUNTS_PER_REV = 4096
DEFAULT_ANGLE = 20.0      # degrees either side of centre -- deliberately modest
DEFAULT_SPEED = 300       # steps/s -- slower than the sweep tests
MOVE_TIMEOUT = 10.0
POSITION_TOLERANCE = 20   # counts (~1.8 deg)

ADDR_MIN_ANGLE        = 9
ADDR_MAX_ANGLE        = 11
ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_GOAL_SPEED       = 46
ADDR_PRESENT_POSITION = 56
ADDR_MOVING           = 66


class BusError(Exception):
    pass


def to_counts(deg):
    return round(deg * COUNTS_PER_REV / 360.0)


def to_degrees(pos):
    return (pos - CENTER_POSITION) * 360.0 / COUNTS_PER_REV


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
    ports = {}
    for device in devices:
        port = PortHandler(device)
        port.baudrate = DEFAULT_BAUDRATE
        try:
            if port.openPort():
                ports[device] = port
            else:
                print(f"  {device}: failed to open")
        except Exception as e:
            print(f"  {device}: failed to open ({e})")
    return ports


def find_coxas(packet, ports, legs):
    """Returns {leg: (sid, port)} for every coxa that answers."""
    found = {}
    for leg in legs:
        sid = leg * 10 + COXA_JOINT_DIGIT
        for device, port in ports.items():
            _model, comm, _err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found[leg] = (sid, port)
                print(f"  leg {leg} coxa (servo {sid}): found on {device}")
                break
        else:
            print(f"  leg {leg} coxa (servo {sid}): no answer")
    return found


def move_all(packet, coxas, targets):
    """Send every coxa to its target and wait. Returns {leg: (pos, timed_out)}."""
    for leg, (sid, port) in coxas.items():
        write2(packet, port, sid, ADDR_GOAL_POSITION, targets[leg], "write goal")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)
    results = {}
    pending = set(coxas)
    while pending:
        for leg in list(pending):
            sid, port = coxas[leg]
            pos = read2(packet, port, sid, ADDR_PRESENT_POSITION, "read position")
            moving = read1(packet, port, sid, ADDR_MOVING, "read moving flag")
            results[leg] = (pos, False)
            if not moving and abs(pos - targets[leg]) <= POSITION_TOLERANCE:
                pending.discard(leg)
        if pending and time.monotonic() > deadline:
            for leg in pending:
                results[leg] = (results[leg][0], True)
            break
        if pending:
            time.sleep(0.02)
    return results


def report(coxas, targets, results, limits):
    """Print measured vs commanded for every leg. Returns True if all arrived."""
    print(f"\n    {'leg':>3} {'servo':>5} {'name':<12} {'target':>7} {'actual':>7} "
          f"{'err':>5} {'angle':>8}")
    ok = True
    for leg in sorted(coxas):
        sid, _port = coxas[leg]
        pos, timed_out = results[leg]
        err = pos - targets[leg]
        flag = ""
        if timed_out:
            ok = False
            flag = "  ⚠ STALLED"
        elif abs(err) > POSITION_TOLERANCE:
            ok = False
            flag = "  ⚠ off target"
        lo, hi = limits[leg]
        if targets[leg] in (lo, hi):
            flag += "  (at firmware limit)"
        print(f"    {leg:>3} {sid:>5} {LEG_NAME[leg]:<12} {targets[leg]:>7} "
              f"{pos:>7} {err:>+5} {to_degrees(pos):>+7.1f}°{flag}")
    return ok


def confirm(question):
    """Explicit 'y' to continue. Anything else -- including a bare Enter or
    Ctrl-C -- aborts, so the reflexive response is the safe one."""
    try:
        answer = input(f"\n  {question} [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer == "y"


def plan_targets(coxas, limits, counts, direction, invert):
    """direction: +1 forward, -1 rear, 0 centre. Clamped to firmware limits."""
    targets, clamped = {}, []
    for leg in coxas:
        sign = FORWARD_SIGN[leg] * (-1 if invert else 1)
        want = CENTER_POSITION + direction * sign * counts
        lo, hi = limits[leg]
        got = max(lo, min(hi, want))
        if got != want:
            clamped.append((leg, want, got))
        targets[leg] = got
    return targets, clamped


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA proto-walk 1: sweep the coxas forward and rear.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--angle", type=float, default=DEFAULT_ANGLE,
                    help=f"degrees either side of centre (default {DEFAULT_ANGLE:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--invert", action="store_true",
                    help="flip FORWARD_SIGN on all six legs at once")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if not 0 < args.angle <= 90:
        ap.error("--angle must be between 0 and 90 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    legs = [l for l in dict.fromkeys(args.legs)]
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    counts = to_counts(args.angle)
    print("CRABORA proto-walk 1: coxa forward/rear sweep")
    print(f"  ±{args.angle:g}° (±{counts} counts) at {args.speed} steps/s"
          f"{'  [INVERTED]' if args.invert else ''}\n")
    print(f"  {'leg':>3} {'name':<12} {'bearing':>8} {'forward is':>12}")
    for leg in legs:
        sign = FORWARD_SIGN[leg] * (-1 if args.invert else 1)
        bearing = 30 + (leg - 1) * 60
        print(f"  {leg:>3} {LEG_NAME[leg]:<12} {bearing:>7}° "
              f"{('CCW (+)' if sign > 0 else 'CW (-)'):>12}")

    if args.dry_run:
        print(f"\n  Nominal targets (before each servo's firmware limits clamp them):")
        for leg in legs:
            sign = FORWARD_SIGN[leg] * (-1 if args.invert else 1)
            print(f"    leg {leg}: forward {CENTER_POSITION + sign * counts}, "
                  f"centre {CENTER_POSITION}, rear {CENTER_POSITION - sign * counts}")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("\nNo FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1

    print()
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    coxas = {}
    try:
        coxas = find_coxas(packet, ports, legs)
        if not coxas:
            print("\nNo coxa servos answered. Run tests/test_03_ping_servo.py.")
            return 1
        missing = [l for l in legs if l not in coxas]
        if missing:
            print(f"\n  Legs {', '.join(map(str, missing))} did not answer -- "
                  f"continuing with {len(coxas)} of {len(legs)}.")

        # Firmware limits are the real guard: a joint whose travel was set
        # with set_center.py can never be driven past it from here.
        limits = {}
        print()
        for leg, (sid, port) in sorted(coxas.items()):
            lo = read2(packet, port, sid, ADDR_MIN_ANGLE, "read min limit")
            hi = read2(packet, port, sid, ADDR_MAX_ANGLE, "read max limit")
            print(f"  leg {leg} limits {lo}..{hi}  "
                  f"({to_degrees(lo):+.1f}° .. {to_degrees(hi):+.1f}°)")
            if lo >= hi or not lo <= CENTER_POSITION <= hi:
                print(f"\n  Leg {leg} has no usable travel around centre "
                      f"({lo}..{hi}). Run utils/set_center.py first.")
                return 1
            limits[leg] = (lo, hi)

        for leg, (sid, port) in coxas.items():
            write2(packet, port, sid, ADDR_GOAL_SPEED, args.speed, "write speed")
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "enable torque")

        stages = [
            ("CENTRE",  0, "Are ALL coxas truly centred? Sight along the body "
                           "-- each leg should point straight out from its flat."),
            ("FORWARD", +1, "Did every leg swing FORWARD (toward the point between "
                            "legs 1 and 6)?"),
            ("REAR",    -1, "Did every leg swing REAR?"),
        ]

        for label, direction, question in stages:
            targets, clamped = plan_targets(coxas, limits, counts, direction,
                                            args.invert)
            print(f"\n  -> {label}")
            for leg, want, got in clamped:
                print(f"     leg {leg}: {want} clamped to {got} by its firmware "
                      f"limit ({to_degrees(got):+.1f}° not {to_degrees(want):+.1f}°)")
            results = move_all(packet, coxas, targets)
            arrived = report(coxas, targets, results, limits)
            if not arrived:
                print("\n  Not every joint reached its target -- see the flags above.")
            if not confirm(question):
                print("\n  Stopped. Returning to centre and going limp.")
                move_all(packet, coxas,
                         {leg: CENTER_POSITION for leg in coxas})
                return 1

        print("\n  -> CENTRE (finishing)")
        results = move_all(packet, coxas, {leg: CENTER_POSITION for leg in coxas})
        report(coxas, {leg: CENTER_POSITION for leg in coxas}, results, limits)
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        # Always leave the robot limp, whatever happened.
        if coxas:
            print("\n  Releasing torque on all coxas.")
            for leg, (sid, port) in coxas.items():
                try:
                    write1(packet, port, sid, ADDR_TORQUE_ENABLE, 0,
                           "release torque")
                except BusError as e:
                    print(f"    could not release leg {leg}: {e}")
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
