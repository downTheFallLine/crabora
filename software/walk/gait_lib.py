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

# The bus libraries are optional on purpose: the geometry constants and
# sign tables below are pure data, useful for kinematics and analysis on
# a machine with no servo SDK installed. Anything that actually touches
# the bus raises a clear error instead of failing at import time.
try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: F401
    from urt_lib import DEFAULT_BAUDRATE, find_urt_devices  # noqa: F401,E402
    HAVE_BUS = True
except ImportError as _exc:                                  # pragma: no cover
    HAVE_BUS = False
    _BUS_IMPORT_ERROR = _exc
    COMM_SUCCESS = 0
    DEFAULT_BAUDRATE = 1_000_000

    def _no_bus(*_a, **_k):
        raise ImportError(
            f"servo bus libraries are not available ({_BUS_IMPORT_ERROR}). "
            f"Install them with:  python3 -m pip install -r "
            f"software/requirements.txt")

    PacketHandler = PortHandler = find_urt_devices = _no_bus


# =============================================================
# GEOMETRY  (see README.md)
# =============================================================
ALL_LEGS = (1, 2, 3, 4, 5, 6)

# Second digit of the servo ID (see project README): 1 coxa, 2 femur, 3 tibia.
COXA_JOINT_DIGIT  = 1
FEMUR_JOINT_DIGIT = 2
TIBIA_JOINT_DIGIT = 3
JOINT_NAME = {1: "coxa", 2: "femur", 3: "tibia"}

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
# ✓ VERIFIED ON HARDWARE 2026-10-10: all six legs driven together by
# tripod_coxa.py swung correctly, the two tripods opposing as intended,
# with no --invert. The mirror is real and this table has it right.
#
# --invert survives as a diagnostic, not a fix: if a future run needs it,
# something changed in how the servos are mounted. If only SOME legs go
# the wrong way, they are mounted inconsistently -- fix that in hardware
# rather than patching individual legs here.
FORWARD_SIGN = {1: +1, 2: +1, 3: +1, 4: -1, 5: -1, 6: -1}

# Sign such that `CENTER + UP_SIGN * counts` raises the femur. A RISING
# servo count LOWERS this joint, so up is -1.
#
# ✓ VERIFIED ON HARDWARE 2026-10-11. Two findings from that run:
#   1. The femurs are NOT mirrored. All six moved together, confirming the
#      prediction: the coxa's mirror comes from body geometry (legs 1 and 6
#      point different ways, so forward is opposite rotations), whereas
#      lift is the same physical motion on every leg. One sign serves all.
#   2. The direction was inverted from the first guess -- the run raised
#      the femurs when the code said "down". Hence -1, not +1.
FEMUR_UP_SIGN = {leg: -1 for leg in ALL_LEGS}

# +1 = a rising servo count EXTENDS this tibia (straightens the knee,
# pushing the foot away from the body). -1 means a rising count flexes it.
#
# ✓ VERIFIED ON HARDWARE 2026-10-12 (test_05_calibration.py --signs
# --legs 1 4). Legs 1 and 4 are one per side, so this settles the mirror
# question too: both extended, so the tibia is NOT mirrored -- uniform
# across all six, like the femur. The first guess of +1 was right; unlike
# the femur, this one did not need inverting.
#
# This was the last unverified joint sign. All three tables now match
# hardware.
TIBIA_EXTEND_SIGN = {leg: +1 for leg in ALL_LEGS}

# Femur centre (2048) is the femur HORIZONTAL; envelope is ±90°, straight
# up to straight down. See README.md -- the envelope is symmetric but the
# useful range is not, and down is gravity-assisted while up is not.
FEMUR_MAX_DEFLECTION_DEG = 90.0

# Tibia centre (2048) is the tibia COLLINEAR WITH THE FEMUR -- the leg
# dead straight, zero knee bend. Decided 2026-10-11; see README.md for
# why (repeatable calibration against a straightedge, and zero = zero
# flexion for later IK).
#
# A knee only bends one way, so centre sits at the EDGE of useful travel,
# not the middle: there is almost nothing on the extend side and all of
# it on the flex side. Code that sweeps this joint symmetrically is the
# wrong shape -- see TIBIA_DEFAULT_EXTEND_DEG / _FLEX_DEG below.
TIBIA_EXTEND_LIMIT_DEG = 15.0    # how far past straight it is worth asking for
TIBIA_FLEX_LIMIT_DEG   = 135.0   # provisional; the real guard is the servo's
                                 # own firmware limits, set with set_center.py

# Alternating tripods: each has two legs on one side and one on the
# other, straddling the centre of mass.
TRIPOD_A = (1, 3, 5)
TRIPOD_B = (2, 4, 6)


# =============================================================
# LINK GEOMETRY  (mm) -- see README.md
# =============================================================
# Taken from the CAD and confirmed against the built leg with calipers
# on 2026-10-11. Distances are between JOINT AXES (the centre of each
# servo horn), which is the kinematic quantity, not part edges.
BODY_TO_COXA_AXIS = 163.22   # body centre -> coxa axis, horizontal, along
                             # the leg's bearing. Derived from the deck:
                             # apothem 181.825 - setback 5 - cap 52.2/2
                             # + horn offset 12.5.

# Coxa axis -> femur axis. NOT a simple outboard length: the femur axis is
# displaced sideways and upwards as well, so the femur/tibia plane is
# OFFSET from the coxa rotation axis rather than passing through it. The
# leg is therefore not a textbook planar 2-link chain on a yaw joint --
# any IK needs the tangent term or it will be quietly wrong.
COXA_RADIAL   = 58.0         # outboard, along the leg's bearing
COXA_TANGENT  = 21.0         # sideways, along the femur horn axis
COXA_VERTICAL = 17.6         # up, above the pan disc bottom

# The femur caps hang BELOW the deck plate, so the robot's real ground
# clearance is less than the deck height every pose calculation reports.
# Nothing in the kinematics knows about them: leg_fk measures to the
# femur AXIS, and the cap is a lump of plastic underneath it.
#
# MEASURED 2026-10-13: 52 mm under the caps while the model reported 73 mm
# of deck clearance. So the lowest hardware sits about this far below the
# number the geometry quotes, and it is the cap, not the deck, that hits
# the floor first.
CAP_DROP_BELOW_DECK = 21.0

# How much the body actually drops when one tripod comes off the floor,
# over and above anything the joint angles show.
#
# MEASURED 2026-10-13: the chassis went 46 mm -> 33 mm when tripod A
# lifted, while the stance femurs gave about one degree. Thirteen
# millimetres the encoders cannot see, so it is mechanical -- structural
# flex, horn and gear backlash, feet sliding outward under the doubled
# load, or all three. Every clearance computed from angles is optimistic
# by at least this much, and by more under a moving load.
TRIPOD_SAG = 13.0


FEMUR_LEN = 100.0            # femur axis -> tibia axis
TIBIA_LEN = 120.0            # tibia axis -> foot tip

# Hexagon deck, flat-to-flat / 2 (unibody/design_flanged_deck.py
# DECK_APOTHEM). A foot closer to the body centre than this is somewhere
# under the deck, which is the geometric test for "folded underneath".
DECK_APOTHEM = 181.825


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
    """An ABSOLUTE servo position, in degrees from centre.

    Note this is NOT the inverse of to_counts(): it subtracts
    CENTER_POSITION and to_counts() does not. Passing a difference to
    this function is wrong by 180 degrees, silently -- use
    delta_degrees() for that. The asymmetry is baked into too many
    callers to change now, so it is named and guarded instead.
    """
    return (pos - CENTER_POSITION) * 360.0 / COUNTS_PER_REV


def delta_degrees(counts):
    """A DIFFERENCE in counts, in degrees. The true inverse of to_counts().

    Exists because to_degrees() is not it, and the resulting 180-degree
    error is the quiet kind: positions still look plausible, and it only
    shows up as a measurement that is reflected about some arbitrary
    value rather than obviously broken.
    """
    return counts * 360.0 / COUNTS_PER_REV


def ground_clearance(deck_height):
    """Real gap under the robot, given a deck height from the pose model.

    Use this wherever a height is shown to a person. The pose model's
    own figure is a kinematic quantity measured to the femur axis, not a
    gap you could slide a ruler into, and quoting it as though it were
    one is how 73 mm turned out to be 52.
    """
    return deck_height - CAP_DROP_BELOW_DECK


def tripod_gap(deck_height):
    """Real gap under the femur caps while one tripod is lifted.

    The binding constraint when walking, and not an obvious one: the
    feet can be clearing beautifully while the chassis scrapes, because
    the caps hang lower than anything the kinematics models.
    """
    return ground_clearance(deck_height) - TRIPOD_SAG


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


# Retries. The bus is half-duplex and shared, and six servos switching
# current is an electrically noisy place, so an occasional lost reply is
# normal and not worth aborting a gait over. One lost packet used to kill
# the whole motion.
#
# But a retry that hides a real fault is worse than the abort was: a
# browning-out supply drops packets too, and that is a thing to fix, not
# to paper over. So retries are COUNTED, and callers are expected to
# report the count. A gait that completes with forty retries is telling
# you something a gait that completes with none is not.
#
# Safe for these registers specifically: goal, speed and torque writes
# are idempotent, so re-sending one the servo already applied is a no-op.
# Do not reuse this for anything that is not.
BUS_RETRIES = 2
RETRY_DELAY = 0.004      # a touch over one 1 Mbaud packet time

_retries = 0
_failures = 0


def bus_stats():
    """(retries, failures) since the last reset. Report these."""
    return _retries, _failures


def reset_bus_stats():
    global _retries, _failures
    _retries = _failures = 0


def _txrx(call, sid, what, tries=None):
    """Run a packet call, retrying a lost reply a couple of times."""
    global _retries, _failures
    tries = BUS_RETRIES + 1 if tries is None else tries
    last = None
    for attempt in range(tries):
        result = call()
        comm = result[-2] if len(result) == 3 else result[0]
        if comm == COMM_SUCCESS:
            return result
        last = comm
        if attempt + 1 < tries:
            _retries += 1
            time.sleep(RETRY_DELAY)
    _failures += 1
    raise BusError(f"servo {sid}: {what}: {PACKET.getTxRxResult(last)} "
                   f"(after {tries} attempts)")


PACKET = None            # set on first use, for getTxRxResult


def _remember(packet):
    global PACKET
    if PACKET is None:
        PACKET = packet
    return packet


def read1(packet, port, sid, addr, what):
    _remember(packet)
    val, _comm, _err = _txrx(
        lambda: packet.read1ByteTxRx(port, sid, addr), sid, what)
    return val


def read2(packet, port, sid, addr, what):
    _remember(packet)
    val, _comm, _err = _txrx(
        lambda: packet.read2ByteTxRx(port, sid, addr), sid, what)
    return val


def write1(packet, port, sid, addr, val, what):
    _remember(packet)
    _txrx(lambda: packet.write1ByteTxRx(port, sid, addr, val), sid, what)


def write2(packet, port, sid, addr, val, what):
    _remember(packet)
    _txrx(lambda: packet.write2ByteTxRx(port, sid, addr, val), sid, what)


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


def find_joints(packet, ports, legs, joint=COXA_JOINT_DIGIT, quiet=False):
    """Ping one joint of each leg on every open port. Returns {leg: (sid, port)}."""
    found = {}
    name = JOINT_NAME[joint]
    for leg in legs:
        sid = leg * 10 + joint
        for device, port in ports.items():
            _model, comm, _err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found[leg] = (sid, port)
                if not quiet:
                    print(f"  leg {leg} {name} (servo {sid}): found on {device}")
                break
        else:
            if not quiet:
                print(f"  leg {leg} {name} (servo {sid}): no answer")
    return found


def find_coxas(packet, ports, legs, quiet=False):
    """Back-compat wrapper: find_joints() restricted to the coxas."""
    return find_joints(packet, ports, legs, COXA_JOINT_DIGIT, quiet)


def read_limits(packet, coxas, verbose=True):
    """Read each joint's firmware angle limits. Returns {leg: (lo, hi)}.

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
