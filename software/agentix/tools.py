"""
tools -- what the agent is allowed to do.

THE READ-ONLY TOOLS. Nothing here moves a servo -- motion lives in
motion.py, behind the arming gate in safety.py.

That split is the point. These can be called freely, by the agent or by
the control strip, with no gate and no consequence beyond bus time. They
are also what the agent should reach for first: looking is free, moving
is not.

The emergency stop is deliberately NOT a tool, in either module. It
belongs to the human, on a button, reachable without the model being
involved at all.

Every function returns a plain dict: JSON for the API, and readable
when a model gets it back as an observation.
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "walk"))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "stand"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    JOINT_NAME, LEG_NAME, TIBIA_JOINT_DIGIT,
    ADDR_MAX_ANGLE, ADDR_MIN_ANGLE, ADDR_MOVING, ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMP, ADDR_TORQUE_ENABLE, to_degrees,
)
from arbiter import ArbiterError, describe  # noqa: E402

ADDR_PRESENT_LOAD    = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_SERVO_STATUS    = 65

ERROR_FLAGS = {0: "Voltage", 1: "Sensor", 2: "Temperature",
               3: "Current", 4: "Angle", 5: "Overload"}

JOINT_DIGITS = (COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT)
NAME_TO_DIGIT = {v: k for k, v in JOINT_NAME.items()}

TEMP_WARN_C = 55
LOAD_NOTABLE = 300


def _flags(v):
    return [n for b, n in ERROR_FLAGS.items() if v & (1 << b)]


def _load(raw):
    return -(raw & 0x3FF) if raw & 0x400 else (raw & 0x3FF)


# =============================================================
# TOOLS
# =============================================================
def robot_status(arbiter):
    """Overall health. The first thing to call -- it answers 'is the
    robot OK?' in one round trip, and says which legs to look at."""
    with arbiter.session("status") as s:
        joints = s.joints()
        answering, faults, hot, loaded, limp, driven = [], [], [], [], [], []
        for digit in JOINT_DIGITS:
            for leg in ALL_LEGS:
                sid = leg * 10 + digit
                if leg not in joints.get(digit, {}):
                    continue
                answering.append(sid)
                try:
                    st = s.read1(sid, ADDR_SERVO_STATUS, "status")
                    temp = s.read1(sid, ADDR_PRESENT_TEMP, "temp")
                    load = _load(s.read2(sid, ADDR_PRESENT_LOAD, "load"))
                    torque = s.read1(sid, ADDR_TORQUE_ENABLE, "torque")
                except ArbiterError:
                    continue
                if _flags(st):
                    faults.append({"servo": sid, "where": describe(sid),
                                   "flags": _flags(st)})
                if temp >= TEMP_WARN_C:
                    hot.append({"servo": sid, "where": describe(sid),
                                "temp_c": temp})
                if abs(load) >= LOAD_NOTABLE:
                    loaded.append({"servo": sid, "where": describe(sid),
                                   "load": load})
                (driven if torque else limp).append(sid)

    missing = [leg * 10 + d for d in JOINT_DIGITS for leg in ALL_LEGS
               if leg not in joints.get(d, {})]
    if faults:
        summary = (f"{len(faults)} servo(s) have latched faults and are "
                   f"ignoring commands until cleared")
    elif missing:
        summary = f"{len(missing)} of 18 servos are not answering"
    elif driven and limp:
        summary = (f"{len(driven)} servo(s) holding, {len(limp)} limp -- "
                   f"the robot is part-way through something")
    elif driven:
        summary = "all servos holding position"
    else:
        summary = "all servos limp"

    return {
        "summary": summary,
        "answering": len(answering),
        "expected": len(ALL_LEGS) * 3,
        "missing": missing,
        "faults": faults,
        "hot": hot,
        "under_load": loaded,
        "holding": len(driven),
        "limp": len(limp),
    }


def joint_detail(arbiter, leg: int, joint: str):
    """Everything about one joint.

    leg: 1-6. joint: 'coxa', 'femur' or 'tibia'.
    """
    if leg not in ALL_LEGS:
        return {"error": f"leg must be 1-6, got {leg}"}
    joint = str(joint).lower().strip()
    if joint not in NAME_TO_DIGIT:
        return {"error": f"joint must be coxa, femur or tibia, got {joint!r}"}
    sid = leg * 10 + NAME_TO_DIGIT[joint]

    with arbiter.session(f"detail {sid}") as s:
        if arbiter.locate(sid) is None:
            return {"error": f"servo {sid} ({describe(sid)}) is not answering"}
        pos = s.read2(sid, ADDR_PRESENT_POSITION, "position")
        lo = s.read2(sid, ADDR_MIN_ANGLE, "min")
        hi = s.read2(sid, ADDR_MAX_ANGLE, "max")
        load = _load(s.read2(sid, ADDR_PRESENT_LOAD, "load"))
        temp = s.read1(sid, ADDR_PRESENT_TEMP, "temp")
        volt = s.read1(sid, ADDR_PRESENT_VOLTAGE, "volts")
        st = s.read1(sid, ADDR_SERVO_STATUS, "status")
        torque = s.read1(sid, ADDR_TORQUE_ENABLE, "torque")
        moving = s.read1(sid, ADDR_MOVING, "moving")

    return {
        "servo": sid,
        "where": f"{describe(sid)} ({LEG_NAME[leg]})",
        "position_deg": round(to_degrees(pos), 1),
        "travel_deg": [round(to_degrees(lo), 1), round(to_degrees(hi), 1)],
        "at_limit": pos <= lo or pos >= hi,
        "load": load,
        "temp_c": temp,
        "volts": round(volt / 10.0, 1) if volt else None,
        "faults": _flags(st),
        "holding": bool(torque),
        "moving": bool(moving),
    }


def calibration_check(arbiter):
    """Are all 18 joints set up consistently with each other?

    The read-only half of tests/test_05_calibration.py: compares each
    joint's travel against the median of its peers. This is what catches
    one leg quietly configured differently from the rest.
    """
    with arbiter.session("calibration") as s:
        joints = s.joints()
        limits = {}
        for digit in JOINT_DIGITS:
            for leg in ALL_LEGS:
                if leg not in joints.get(digit, {}):
                    continue
                sid = leg * 10 + digit
                limits[(leg, digit)] = (
                    s.read2(sid, ADDR_MIN_ANGLE, "min"),
                    s.read2(sid, ADDR_MAX_ANGLE, "max"))

    def median(xs):
        xs = sorted(xs)
        n = len(xs)
        return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0

    problems, rows = [], []
    for digit in JOINT_DIGITS:
        present = [l for l in ALL_LEGS if (l, digit) in limits]
        if len(present) < 2:
            continue
        mlo = median([limits[(l, digit)][0] for l in present])
        mhi = median([limits[(l, digit)][1] for l in present])
        for leg in present:
            lo, hi = limits[(leg, digit)]
            odd = abs(lo - mlo) > 30 or abs(hi - mhi) > 30
            rows.append({"servo": leg * 10 + digit,
                         "where": describe(leg * 10 + digit),
                         "travel_deg": [round(to_degrees(lo), 0),
                                        round(to_degrees(hi), 0)],
                         "matches_peers": not odd})
            if odd:
                problems.append(
                    f"{describe(leg * 10 + digit)} travel "
                    f"{to_degrees(lo):+.0f}..{to_degrees(hi):+.0f}° differs "
                    f"from its peers' {to_degrees(mlo):+.0f}..{to_degrees(mhi):+.0f}°")
            if not lo <= CENTER_POSITION <= hi:
                problems.append(f"{describe(leg * 10 + digit)} centre is "
                                f"outside its own travel limits")

    return {
        "consistent": not problems,
        "summary": ("all joints agree" if not problems
                    else f"{len(problems)} joint(s) disagree with their peers"),
        "problems": problems,
        "joints": rows,
    }


def foot_positions(arbiter):
    """Where each foot actually is, in the body frame.

    Forward kinematics from the measured joint angles -- so 'is a leg
    folded under itself?' is a number rather than a guess. +X is forward
    through the point between legs 1 and 6, +Y left, +Z up.
    """
    try:
        from pose import foot_in_body_frame
    except ImportError as e:
        return {"error": f"kinematics unavailable: {e}"}

    with arbiter.session("feet") as s:
        joints = s.joints()
        angles = {}
        for digit in JOINT_DIGITS:
            for leg in ALL_LEGS:
                if leg in joints.get(digit, {}):
                    angles[(leg, digit)] = to_degrees(
                        s.read2(leg * 10 + digit, ADDR_PRESENT_POSITION, "pos"))

    feet, incomplete = [], []
    for leg in ALL_LEGS:
        try:
            c = angles[(leg, COXA_JOINT_DIGIT)]
            f = -angles[(leg, FEMUR_JOINT_DIGIT)]   # up is -count
            t = -angles[(leg, TIBIA_JOINT_DIGIT)]   # flex is -count (verified)
        except KeyError:
            incomplete.append(leg)
            continue
        x, y, z = foot_in_body_frame(leg, c, f, t)
        feet.append({"leg": leg, "name": LEG_NAME[leg],
                     "x_mm": round(x, 1), "y_mm": round(y, 1),
                     "z_mm": round(z, 1),
                     "reach_mm": round(math.hypot(x, y), 1)})

    close = []
    by_leg = {f["leg"]: f for f in feet}
    order = [l for l in ALL_LEGS if l in by_leg]
    for a, b in zip(order, order[1:] + order[:1]):
        if a == b:
            continue
        d = math.dist((by_leg[a]["x_mm"], by_leg[a]["y_mm"], by_leg[a]["z_mm"]),
                      (by_leg[b]["x_mm"], by_leg[b]["y_mm"], by_leg[b]["z_mm"]))
        if d < 120:
            close.append({"legs": [a, b], "apart_mm": round(d)})

    return {
        "feet": feet,
        "incomplete_legs": incomplete,
        "neighbours_close": close,
    }


def imu_state(arbiter=None):
    """Tilt and heading, if an IMU is fitted. Independent of the servo bus."""
    try:
        from imu_lib import Imu, ImuError
    except ImportError as e:
        return {"fitted": False, "error": f"imu_lib unavailable: {e}"}
    try:
        with Imu() as imu:
            r = imu.read()
            return {
                "fitted": True,
                "roll_deg": round(r.roll_deg, 1),
                "pitch_deg": round(r.pitch_deg, 1),
                "g_total": round(r.g_total, 3),
                "still": r.is_still,
                "temp_c": round(r.temp_c, 1),
                "note": ("heading needs zero_gyro() at startup and is not "
                         "tracked by this read-only call"),
            }
    except Exception as e:                            # noqa: BLE001
        return {"fitted": False, "error": f"{type(e).__name__}: {e}"}


# Commanded goals older than this are not "where the leg is going" any
# more -- the motion that wrote them is over.
GOAL_FRESH_S = 2.0


def leg_pose(arbiter, leg):
    """One leg's three joint angles, for the UI's leg animation. NOT a
    model tool: it is polled several times a second and never waits.

    Measured from the servos when the bus is free. While a motion holds
    the bus, the goals the gait last wrote instead (gait_lib.LAST_GOALS)
    -- 'commanded', which is where the leg was told to go. Angles use the
    kinematics' conventions: coxa_deg as foot_in_body_frame takes it
    (+ is counter-clockwise from above), femur_up_deg (+ up from level),
    tibia_flex_deg (0 straight, + folded).
    """
    import time                                       # noqa: PLC0415
    from gait_lib import LAST_GOALS, FORWARD_SIGN    # noqa: PLC0415

    sids = {d: leg * 10 + d for d in JOINT_DIGITS}
    counts = arbiter.try_positions(list(sids.values()))
    source, age = "measured", 0.0
    if not counts or len(counts) < 3:
        now = time.monotonic()
        goals = {sid: LAST_GOALS.get(sid) for sid in sids.values()}
        if all(goals.values()):
            age = now - min(t for _c, t in goals.values())
            counts = {sid: c for sid, (c, _t) in goals.items()}
            source = "commanded" if age < GOAL_FRESH_S else "last commanded"
        else:
            return {"leg": leg, "name": LEG_NAME[leg], "source": "none",
                    "reason": ("bus busy and nothing commanded yet"
                               if arbiter.available else
                               f"bus unavailable: {arbiter.reason}")}

    coxa = to_degrees(counts[sids[COXA_JOINT_DIGIT]])
    femur = -to_degrees(counts[sids[FEMUR_JOINT_DIGIT]])   # up is -count
    tibia = -to_degrees(counts[sids[TIBIA_JOINT_DIGIT]])   # flex is -count
    return {
        "leg": leg, "name": LEG_NAME[leg], "source": source,
        "age_s": round(age, 2),
        "coxa_deg": round(coxa, 1),
        # Forward-positive, for a label a person can read: the coxa is
        # mirrored, so raw + means forward on one side and back on the other.
        "coxa_forward_deg": round(coxa * FORWARD_SIGN[leg], 1) + 0.0,
        "femur_up_deg": round(femur, 1),
        "tibia_flex_deg": round(tibia, 1),
    }


# What the agent may call. Motion tools get added here, deliberately,
# one blast radius at a time.
READ_ONLY_TOOLS = {
    "robot_status": robot_status,
    "joint_detail": joint_detail,
    "calibration_check": calibration_check,
    "foot_positions": foot_positions,
    "imu_state": imu_state,
}

TOOL_HELP = {
    "robot_status": "Overall health: what is answering, faulted, hot, loaded, "
                    "holding or limp. Call this first.",
    "joint_detail": "Everything about one joint. Args: leg (1-6), "
                    "joint ('coxa'|'femur'|'tibia').",
    "calibration_check": "Whether all 18 joints are set up consistently with "
                         "each other.",
    "foot_positions": "Where each foot is in the body frame, by forward "
                      "kinematics.",
    "imu_state": "Tilt from the IMU, if one is fitted.",
}
