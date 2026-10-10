"""
CRABORA gait_lib -- shared conventions and bus plumbing for software/walk
===========================================================================

Everything in software/walk/ imports its geometry constants and servo
access from here, so there is exactly one place where "which way is
forward" is defined. See README.md for the derivation.

Contents:
  - Geometry: ALL_LEGS, LEG_NAME, LEG_BEARING, FORWARD_SIGN, TRIPOD_A/B
  - Units: to_counts(), to_degrees()
  - Bus: read1/read2/write1/write2, open_ports(), find_coxas(), move_all()
  - Operator: confirm()

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: F401

from urt_lib import DEFAULT_BAUDRATE, find_urt_devices  # noqa: F401,E402


# =============================================================
# GEOMETRY  (see README.md)
# =============================================================
ALL_LEGS = (1, 2, 3, 4, 5, 6)
COXA_JOINT_DIGIT = 1

# Bearing clockwise from the front point (the hexagon vertex between
# legs 1 and 6): leg n sits at 30 + (n-1)*60 degrees.
LEG_BEARING = {n: 30 + (n - 1) * 60 for n in ALL_LEGS}

LEG_NAME = {
    1: "front-right", 2: "right", 3: "rear-right",
    4: "rear-left",   5: "left",  6: "front-left",
}

# +1 = counter-clockwise about body +Z moves this foot FORWARD.
# A CCW coxa rotation moves leg n's foot forward by sin(bearing), which
# is positive for legs 1-3 (right side) and negative for 4-6 (left).
#
# ⚠ UNVERIFIED ON HARDWARE. Assumes every coxa servo is mounted the same
# way round. If all six go the wrong way, use --invert and then flip this
# table. If only SOME go wrong, the servos are mounted inconsistently --
# fix that in hardware rather than patching individual legs here.
FORWARD_SIGN = {1: +1, 2: +1, 3: +1, 4: -1, 5: -1, 6: -1}

# Alternating tripods: each has two legs on one side and one on the
# other, straddling the centre of mass.
TRIPOD_A = (1, 3, 5)
TRIPOD_B = (2, 4, 6)


# =============================================================
# UNITS
# =============================================================
CENTER_POSITION = 2048
COUNTS_PER_REV = 4096
POSITION_TOLERANCE = 20    # counts (~1.8 deg)
MOVE_TIMEOUT = 10.0        # seconds before a move is called stalled


def to_counts(deg):
    return round(deg * COUNTS_PER_REV / 360.0)


def to_degrees(pos):
    return (pos - CENTER_POSITION) * 360.0 / COUNTS_PER_REV


def forward_sign(leg, invert=False):
    return FORWARD_SIGN[leg] * (-1 if invert else 1)


# =============================================================
# REGISTERS  (STS3215 / common Feetech map)
# =============================================================
ADDR_MIN_ANGLE        = 9
ADDR_MAX_ANGLE        = 11
ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_GOAL_SPEED       = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_TEMP     = 63
ADDR_MOVING           = 66


class BusError(Exception):
    pass


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


# =============================================================
# BUS
# =============================================================
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


def find_coxas(packet, ports, legs, quiet=False):
    """Ping each leg's coxa on every open port. Returns {leg: (sid, port)}."""
    found = {}
    for leg in legs:
        sid = leg * 10 + COXA_JOINT_DIGIT
        for device, port in ports.items():
            _model, comm, _err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found[leg] = (sid, port)
                if not quiet:
                    print(f"  leg {leg} coxa (servo {sid}): found on {device}")
                break
        else:
            if not quiet:
                print(f"  leg {leg} coxa (servo {sid}): no answer")
    return found


def read_limits(packet, coxas, verbose=True):
    """Read each coxa's firmware angle limits. Returns {leg: (lo, hi)}.

    Raises BusError if a joint has no usable travel around centre -- that
    means set_center.py hasn't been run on it, and nothing should move.
    """
    limits = {}
    for leg, (sid, port) in sorted(coxas.items()):
        lo = read2(packet, port, sid, ADDR_MIN_ANGLE, "read min limit")
        hi = read2(packet, port, sid, ADDR_MAX_ANGLE, "read max limit")
        if verbose:
            print(f"  leg {leg} limits {lo}..{hi}  "
                  f"({to_degrees(lo):+.1f}° .. {to_degrees(hi):+.1f}°)")
        if lo >= hi or not lo <= CENTER_POSITION <= hi:
            raise BusError(
                f"leg {leg} has no usable travel around centre ({lo}..{hi}) -- "
                f"run utils/set_center.py on servo {sid} first")
        limits[leg] = (lo, hi)
    return limits


def arm(packet, coxas, speed):
    """Set move speed and enable torque on every coxa."""
    for leg, (sid, port) in coxas.items():
        write2(packet, port, sid, ADDR_GOAL_SPEED, speed, "write speed")
        write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "enable torque")


def release(packet, coxas):
    """Drop torque on every coxa. Best effort -- never raises."""
    for leg, (sid, port) in coxas.items():
        try:
            write1(packet, port, sid, ADDR_TORQUE_ENABLE, 0, "release torque")
        except BusError as e:
            print(f"    could not release leg {leg}: {e}")


def move_all(packet, coxas, targets):
    """Send every coxa to its own target and wait for all to arrive.
    Returns {leg: (position, timed_out)}."""
    for leg, (sid, port) in coxas.items():
        write2(packet, port, sid, ADDR_GOAL_POSITION, targets[leg], "write goal")

    deadline = time.monotonic() + MOVE_TIMEOUT
    time.sleep(0.05)  # let the servos start moving before polling
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


def clamp_targets(targets, limits):
    """Clip each target to its servo's firmware limits.
    Returns (clamped_targets, [(leg, wanted, got), ...])."""
    out, clamped = {}, []
    for leg, want in targets.items():
        lo, hi = limits[leg]
        got = max(lo, min(hi, want))
        if got != want:
            clamped.append((leg, want, got))
        out[leg] = got
    return out, clamped


def report(coxas, targets, results, tolerance=POSITION_TOLERANCE):
    """Print measured vs commanded per leg. Returns (all_arrived, trouble)."""
    print(f"\n    {'leg':>3} {'servo':>5} {'name':<12} {'target':>7} "
          f"{'actual':>7} {'err':>5} {'angle':>8}")
    ok, trouble = True, {}
    for leg in sorted(coxas):
        sid, _port = coxas[leg]
        pos, timed_out = results[leg]
        err = pos - targets[leg]
        flag = ""
        if timed_out:
            ok = False
            trouble[leg] = "stalled"
            flag = "  ⚠ STALLED"
        elif abs(err) > tolerance:
            ok = False
            trouble[leg] = "off target"
            flag = "  ⚠ off target"
        print(f"    {leg:>3} {sid:>5} {LEG_NAME[leg]:<12} {targets[leg]:>7} "
              f"{pos:>7} {err:>+5} {to_degrees(pos):>+7.1f}°{flag}")
    return ok, trouble


def check(targets, results, tolerance=POSITION_TOLERANCE):
    """Quiet version of report(): returns (all_arrived, {leg: reason})."""
    ok, trouble = True, {}
    for leg, (pos, timed_out) in results.items():
        if timed_out:
            ok, trouble[leg] = False, "stalled"
        elif abs(pos - targets[leg]) > tolerance:
            ok, trouble[leg] = False, "off target"
    return ok, trouble


# =============================================================
# OPERATOR
# =============================================================
def confirm(question):
    """Explicit 'y' to continue. Anything else -- a bare Enter, Ctrl-C,
    closed stdin -- aborts, so the reflexive response is the safe one."""
    try:
        answer = input(f"\n  {question} [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer == "y"
