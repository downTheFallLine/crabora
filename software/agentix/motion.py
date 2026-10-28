"""
motion -- what the agent may actually do to the robot, and when.

Three gates stand between a sentence in the chat box and a servo moving,
deliberately different in kind:

  1. ARMED (safety.py). A human opened a time window. The agent cannot
     open it, and it closes on its own.
  2. PREFLIGHT (here). The robot is worth trusting right now: bus up,
     nothing latched, nothing cooking, supply not sagging.
  3. SHAPE (here, plus the schemas in tools.py). The move is one of a
     handful of known-good shapes with bounded arguments. The model
     picks between them; it does not compose joint angles.

Gate 3 matters most. Two tibias have snapped already, both from motions
that looked geometrically reasonable and were not, so NOTHING HERE
INVENTS A TRAJECTORY. Every sequence is the one that was debugged
against hardware -- stand.py's phase ordering, turn.py's planted-foot
sweep, dance.py's frames -- called as a library. Where those programs
had the sequence buried in main(), it was lifted out rather than copied,
so there is one implementation and the CLI exercises the same code the
agent does.

Long motions poll for cancellation between steps. A dance that cannot be
interrupted is worse than no dance.
"""

import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SOFTWARE = os.path.dirname(_HERE)
for _d in ("walk", "stand", "turn", "dance"):
    sys.path.insert(0, os.path.join(_SOFTWARE, _d))

from gait_lib import (  # noqa: E402
    bus_stats, reset_bus_stats,
    ALL_LEGS, CENTER_POSITION, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT,
    JOINT_NAME, TIBIA_JOINT_DIGIT, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE,
    ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMP, to_counts, to_degrees,
    delta_degrees,
)
from arbiter import ArbiterError, Cancelled, describe  # noqa: E402

ADDR_PRESENT_VOLTAGE = 62
ADDR_SERVO_STATUS    = 65
ERROR_FLAGS = {0: "Voltage", 1: "Sensor", 2: "Temperature",
               3: "Current", 4: "Angle", 5: "Overload"}

JOINT_DIGITS = (COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT)
NAME_TO_DIGIT = {v: k for k, v in JOINT_NAME.items()}

# Refuse above this. The servos trip their own protection higher up; past
# here, continuing just cooks them.
TEMP_REFUSE_C = 65

# The LM2596 regulates 7.4 V from the 3S pack but tops out around 2 A.
# Below this the servos are browning out, and a brownout reads exactly
# like a wiring fault -- which is how an evening gets lost. It no longer
# sees a draining pack: the buck holds 7.4 V until the pack collapses.
VOLTAGE_FLOOR = 6.2

NUDGE_MAX_DEG = 20.0      # a wrong sign should be a twitch, not a strike
MOVE_SPEED  = 400
NUDGE_SPEED = 300


class Unsafe(RuntimeError):
    """Refused before anything moved. The message is for the operator."""


# =============================================================
# SHARED
# =============================================================
def _limits(s, legs):
    """{(leg, digit): (min, max)} straight from servo firmware.

    The phases in stand.py, turn.py and dance.py all expect this shape.
    Firmware limits are the robot's own account of where it may go: set
    per joint on hardware, and they outlive any bug in this process.
    """
    found = s.joints()
    out = {}
    for leg in legs:
        for d in JOINT_DIGITS:
            if leg in found.get(d, {}):
                sid = leg * 10 + d
                out[(leg, d)] = (s.read2(sid, ADDR_MIN_ANGLE, "min"),
                                 s.read2(sid, ADDR_MAX_ANGLE, "max"))
    return out


def preflight(arbiter, legs=None):
    """Read the robot and decide whether it should move at all.

    Raises Unsafe on a blocker, returns what it found otherwise. The
    distinction matters: a warm servo is worth mentioning, a latched
    overload means the joint has already given up and commanding it
    achieves nothing but noise.
    """
    legs = sorted(legs or ALL_LEGS)
    faults, hot, flat, missing, volts = [], [], [], [], []

    with arbiter.session("preflight") as s:
        found = s.joints()
        for digit in JOINT_DIGITS:
            present = found.get(digit, {})
            for leg in legs:
                sid = leg * 10 + digit
                if leg not in present:
                    missing.append(sid)
                    continue
                try:
                    st = s.read1(sid, ADDR_SERVO_STATUS, "status")
                    if st:
                        faults.append({"servo": sid, "where": describe(sid),
                                       "flags": [n for b, n in ERROR_FLAGS.items()
                                                 if st & (1 << b)]})
                    t = s.read1(sid, ADDR_PRESENT_TEMP, "temp")
                    if t >= TEMP_REFUSE_C:
                        hot.append({"servo": sid, "where": describe(sid),
                                    "temp_c": t})
                    v = s.read1(sid, ADDR_PRESENT_VOLTAGE, "voltage") / 10.0
                    volts.append(v)
                    if v < VOLTAGE_FLOOR:
                        flat.append({"servo": sid, "where": describe(sid),
                                     "volts": round(v, 1)})
                except ArbiterError:
                    missing.append(sid)

    report = {"legs": legs, "faults": faults, "hot": hot, "undervolt": flat,
              "missing": sorted(set(missing)),
              "volts_min": round(min(volts), 1) if volts else None}

    if faults:
        raise Unsafe(f"{len(faults)} joint(s) latched in error -- clear them "
                     f"before moving: " +
                     ", ".join(f["where"] for f in faults))
    if hot:
        raise Unsafe("too hot to move: " +
                     ", ".join(f"{h['where']} {h['temp_c']}C" for h in hot))
    if flat:
        raise Unsafe(f"supply sagging below {VOLTAGE_FLOOR}V "
                     f"(min {report['volts_min']}V) -- the servos are "
                     f"browning out, which mimics a wiring fault. Check the "
                     f"buck converter.")
    if len(report["missing"]) > 3:
        raise Unsafe(f"{len(report['missing'])} servos are not answering; the "
                     f"bus is not healthy enough to move")
    return report


def _gate(arbiter, safety, legs=None):
    """Cheap permission check first, then the expensive bus read."""
    safety.require()                 # raises NotArmed
    arbiter.clear_stop()             # an old stop should not linger
    out = preflight(arbiter, legs)
    reset_bus_stats()                # count retries for THIS motion only
    return out


def _done(safety, result):
    """Attach bus health to every motion result.

    A motion that completed after forty retries is telling you something
    a clean one is not -- usually that the supply is sagging under load,
    since a browning-out servo drops packets exactly like a loose
    connector does. Without this the retries are invisible and the only
    symptom is the occasional hard failure.
    """
    safety.touch()
    retries, failures = bus_stats()
    if retries or failures:
        result["bus"] = {
            "retries": retries,
            "failures": failures,
            "note": ("the bus needed re-sends. Under load this is usually "
                     "the supply sagging rather than wiring -- check the "
                     "voltage with an inline meter while it moves."),
        }
    return result


def _usable_legs(s, legs):
    """Legs with all three joints answering. The phases assume this."""
    found = s.joints()
    return [l for l in legs if all(l in found.get(d, {})
                                   for d in JOINT_DIGITS)]


# =============================================================
# MOTIONS, in order of blast radius
# =============================================================
def relax(arbiter, safety, legs=None):
    """Drop torque.

    NOT harmless: if the robot is holding itself up it sits down under
    its own weight. It still does it, because "go limp" is also what you
    want when a leg is straining and the alternative is a snapped tibia
    -- but it reports whether it looked loaded rather than deciding.
    """
    safety.require()
    legs = sorted(legs or ALL_LEGS)
    sids = [leg * 10 + d for leg in legs for d in JOINT_DIGITS]

    with arbiter.session("relax") as s:
        before = s.positions(sids)
        femurs = [p for sid, p in before.items()
                  if sid % 10 == FEMUR_JOINT_DIGIT]
        loaded = sum(1 for p in femurs
                     if abs(p - CENTER_POSITION) > to_counts(25)) >= 4
        released = s.torque(sids, on=False)

    return _done(safety, {
        "action": "relax", "legs": legs, "released": sorted(released),
        "was_bearing_weight": loaded,
        "note": ("it was holding a pose, so it has just sat down" if loaded
                 else "it was not obviously bearing weight"),
    })


def nudge(arbiter, safety, leg: int, joint: str, degrees: float):
    """Move ONE joint a bounded amount, relative to where it is now.

    The lowest-blast-radius real motion: one joint, small angle, relative
    so it cannot fling anywhere. Firmware limits clamp underneath and the
    clamp is reported.
    """
    if leg not in ALL_LEGS:
        raise Unsafe(f"leg must be 1-6, got {leg}")
    digit = NAME_TO_DIGIT.get(str(joint).lower())
    if digit is None:
        raise Unsafe(f"joint must be coxa, femur or tibia, got {joint!r}")
    try:
        degrees = float(degrees)
    except (TypeError, ValueError):
        raise Unsafe(f"degrees must be a number, got {degrees!r}")
    if not -NUDGE_MAX_DEG <= degrees <= NUDGE_MAX_DEG:
        raise Unsafe(f"nudge is limited to +/-{NUDGE_MAX_DEG:g} degrees; "
                     f"asked {degrees:g}. Use a pose command for more.")

    _gate(arbiter, safety, legs=[leg])
    sid = leg * 10 + digit
    with arbiter.session(f"nudge {sid}") as s:
        start = s.read2(sid, ADDR_PRESENT_POSITION, "position")
        res = s.move({sid: start + to_counts(degrees)},
                     speed=NUDGE_SPEED, timeout=3.0)
        end = s.read2(sid, ADDR_PRESENT_POSITION, "position")

    return _done(safety, {
        "action": "nudge", "joint": describe(sid), "asked_deg": degrees,
        # delta_degrees: end-start is a difference, and to_degrees
        # would subtract centre again -- 180 degrees of nonsense.
        "moved_deg": round(delta_degrees(end - start), 1),
        "clamped": res["clamped"] or None,
        "note": "clamped by the joint's own travel limit"
                if res["clamped"] else None,
    })


def lay_down(arbiter, safety, legs=None):
    """Splay flat: every joint to its centre.

    pose_splay() is exactly all-centres -- that is how the conventions
    were chosen (coxa neutral, femur horizontal, tibia collinear). It is
    the safest pose to pass through, because nothing is folded, so
    nothing can be caught on anything.

    The ORDER is the lesson from the tibia that snapped: straighten the
    knees while the femurs are still up, and bring the femurs down last.
    The other way drives a flexed tibia into the floor.
    """
    legs = sorted(legs or ALL_LEGS)
    _gate(arbiter, safety, legs)

    steps = []
    with arbiter.session("lay down") as s:
        legs = _usable_legs(s, legs) or legs
        for digit in (COXA_JOINT_DIGIT, TIBIA_JOINT_DIGIT, FEMUR_JOINT_DIGIT):
            arbiter.check_cancel()
            res = s.move({leg * 10 + digit: CENTER_POSITION for leg in legs},
                         speed=MOVE_SPEED, timeout=6.0)
            steps.append({"joint": JOINT_NAME[digit],
                          "reached": len(res["reached"]),
                          "short": sorted(res["short"])})

    return _done(safety, {
        "action": "lay_down", "pose": "splay (all joints centred)",
        "legs": legs, "order": "coxa, tibia, femur -- femurs last",
        "steps": steps,
    })


def sit(arbiter, safety):
    """Fold to a low, stable pose and hold it under torque.

    Distinct from relax(): puts the body down deliberately rather than
    dropping it.
    """
    _gate(arbiter, safety)
    from pose import pose_compact                    # noqa: PLC0415

    p = pose_compact()
    with arbiter.session("sit") as s:
        legs = _usable_legs(s, sorted(ALL_LEGS))
        for digit, key in ((TIBIA_JOINT_DIGIT, "tibia"),
                           (FEMUR_JOINT_DIGIT, "femur")):
            arbiter.check_cancel()
            s.move({leg * 10 + digit: CENTER_POSITION - to_counts(p[key])
                    for leg in legs}, speed=MOVE_SPEED, timeout=6.0)

    return _done(safety, {"action": "sit",
                          "pose": {k: round(v, 1) for k, v in p.items()}})


def stand(arbiter, safety, height: float = 100.0):
    """Stand up, through stand.py's phases.

    Not reimplemented. Collapse, untangle and stand -- with the floor
    guards and the femur-last ordering -- are the part that was debugged
    against a robot that kept breaking, and they earn their reuse.
    """
    try:
        height = float(height)
    except (TypeError, ValueError):
        raise Unsafe(f"height must be a number, got {height!r}")
    if not 40.0 <= height <= 160.0:
        raise Unsafe(f"stand height must be 40-160 mm, got {height:g}")

    _gate(arbiter, safety)
    import stand as prog                             # noqa: PLC0415

    phases = []
    with arbiter.session(f"stand {height:g}mm") as s:
        packet, legs = s.packet, sorted(ALL_LEGS)
        joints = s.joints()
        _ok, excluded = prog.phase_survey(packet, joints, legs)
        legs = [l for l in legs if l not in excluded]
        if not legs:
            raise Unsafe("no usable legs -- survey excluded all six")
        limits = _limits(s, legs)

        arbiter.check_cancel()
        prog.phase_collapse(packet, joints, legs, limits, False)
        phases.append("collapse")

        arbiter.check_cancel()
        prog.phase_untangle(packet, joints, legs, limits, False)
        phases.append("untangle")

        arbiter.check_cancel()
        # stand.py has no module-level foot-out constant; 120 mm is its
        # own argparse default.
        prog.phase_stand(packet, joints, legs, limits, height, 120.0, False)
        phases.append("stand")

    return _done(safety, {"action": "stand", "height_mm": height,
                          "legs": legs, "excluded": sorted(excluded),
                          "phases": phases})


def turn(arbiter, safety, degrees: int):
    """Turn clockwise, via turn.py's planted-foot sweep.

    Constrained to the four angles that program was built for. Anything
    else is a different program, not a different argument.
    """
    try:
        degrees = int(degrees)
    except (TypeError, ValueError):
        raise Unsafe(f"degrees must be a number, got {degrees!r}")
    if degrees not in (90, 180, 270, 360):
        raise Unsafe(f"turn takes 90, 180, 270 or 360 degrees, got {degrees}")

    _gate(arbiter, safety)
    import turn as prog                              # noqa: PLC0415
    from pose import Unreachable, pose_stand         # noqa: PLC0415

    height = prog.DEFAULT_HEIGHT
    try:
        base = pose_stand(height)
    except Unreachable as e:
        raise Unsafe(f"cannot stand at {height:g} mm: {e}")

    # Same arithmetic as turn.py's main(), with its defaults.
    r_foot = prog.planted_radius(base)
    cap = prog.max_rotation(height, r_foot, 50.0)
    per_step_max = min(prog.DEFAULT_STEP, 2 * cap)
    n_steps = max(1, math.ceil(degrees / per_step_max))
    per_step = degrees / n_steps
    half = per_step / 2

    with arbiter.session(f"turn {degrees}") as s:
        legs = _usable_legs(s, sorted(ALL_LEGS))
        if len(legs) < 6:
            raise Unsafe(f"turning needs all six legs; {sorted(legs)} answered")
        outcome = prog.run_turn(
            s.packet, s.joints(), legs, _limits(s, legs), base,
            half=half, per_step=per_step, n_steps=n_steps, height=height,
            r_foot=r_foot, lift=prog.DEFAULT_LIFT, speed=prog.DEFAULT_SPEED,
            substeps=5, degrees=degrees,
            should_cancel=lambda: arbiter.stopping,
            log=lambda *a, **k: None)

    return _done(safety, {
        "action": "turn", "degrees": degrees, "clockwise": True,
        "turned_deg": round(outcome["total"], 1),
        "steps_done": outcome["steps_done"], "cancelled": outcome["cancelled"],
        "legs_short_on": {str(k): sorted(v)
                          for k, v in outcome["trouble"].items()} or None,
    })


def dance(arbiter, safety, seconds: float = 10.0):
    """Dance for a bounded time, one verified frame at a time.

    dance.py's MOVES are frame generators and play() sends one frame, so
    this drives them directly and checks for a stop between frames --
    which is what makes it interruptible.
    """
    try:
        seconds = float(seconds)
    except (TypeError, ValueError):
        raise Unsafe(f"seconds must be a number, got {seconds!r}")
    if not 2.0 <= seconds <= 30.0:
        raise Unsafe(f"dance runs 2-30 seconds, got {seconds:g}")

    _gate(arbiter, safety)
    import dance as prog                             # noqa: PLC0415
    from pose import Unreachable, pose_stand         # noqa: PLC0415

    height = getattr(prog, "DEFAULT_HEIGHT", 80.0)
    amp = getattr(prog, "DEFAULT_AMP", 12.0)
    beat = getattr(prog, "BEAT", 0.12)        # dance.py calls it BEAT
    try:
        base = pose_stand(height)
    except Unreachable as e:
        raise Unsafe(f"cannot stand at {height:g} mm to dance: {e}")

    played, moves_done = 0, []
    deadline = time.monotonic() + seconds
    cancelled = False

    with arbiter.session(f"dance {seconds:g}s") as s:
        legs = _usable_legs(s, sorted(ALL_LEGS))
        if len(legs) < 6:
            raise Unsafe(f"dancing needs all six legs; {sorted(legs)} answered")
        limits = _limits(s, legs)
        names = sorted(prog.MOVES)

        while time.monotonic() < deadline and not cancelled:
            for name in names:
                if time.monotonic() >= deadline:
                    break
                moves_done.append(name)
                for frame, _note in prog.MOVES[name](base, legs, amp, height):
                    if arbiter.stopping or time.monotonic() >= deadline:
                        cancelled = arbiter.stopping
                        break
                    prog.play(s.packet, s.joints(), legs, limits, frame, beat)
                    played += 1

        # Leave it standing rather than mid-flourish.
        if not arbiter.stopping:
            prog.play(s.packet, s.joints(), legs, limits,
                      {l: base for l in legs}, 0)

    return _done(safety, {
        "action": "dance", "seconds": seconds, "frames": played,
        "moves": sorted(set(moves_done)), "cancelled": cancelled,
    })



HEADINGS = {"forward": 0.0, "backward": 180.0, "left": 90.0, "right": 270.0}
WALK_MIN_MM, WALK_MAX_MM = 20.0, 1000.0


def walk(arbiter, safety, distance: float = 200.0, direction: str = "forward"):
    """Walk in a straight line, via walk.py's planted-foot gait.

    The robot must already be standing. A hexagon with radial legs has no
    mechanical front, so it walks sideways and backwards without turning
    first -- the heading is just a vector into the same maths.
    """
    try:
        distance = float(distance)
    except (TypeError, ValueError):
        raise Unsafe(f"distance must be a number, got {distance!r}")
    if not WALK_MIN_MM <= distance <= WALK_MAX_MM:
        raise Unsafe(f"walk distance must be {WALK_MIN_MM:g}-{WALK_MAX_MM:g} "
                     f"mm, got {distance:g}")
    heading = HEADINGS.get(str(direction).lower())
    if heading is None:
        raise Unsafe(f"direction must be one of "
                     f"{', '.join(sorted(HEADINGS))}; got {direction!r}")

    _gate(arbiter, safety)
    import walk as prog                              # noqa: PLC0415
    from pose import Unreachable                     # noqa: PLC0415

    try:
        p = prog.plan(distance, prog.DEFAULT_STRIDE, prog.DEFAULT_HEIGHT,
                      120.0, prog.DEFAULT_COXA_LIMIT, heading,
                      sorted(ALL_LEGS))
    except Unreachable as e:
        raise Unsafe(f"cannot walk {distance:g} mm {direction}: {e}")

    with arbiter.session(f"walk {distance:g}mm {direction}") as s:
        legs = _usable_legs(s, sorted(ALL_LEGS))
        if len(legs) < 6:
            raise Unsafe(f"walking needs all six legs; {sorted(legs)} answered")
        outcome = prog.run_walk(
            s.packet, s.joints(), legs, _limits(s, legs), p["base"],
            half=p["half"], per_step=p["per_step"], n_steps=p["n_steps"],
            height=prog.DEFAULT_HEIGHT, r_foot=p["r_foot"], heading=heading,
            lift=prog.DEFAULT_LIFT, speed=prog.DEFAULT_SPEED, substeps=5,
            should_cancel=lambda: arbiter.stopping,
            log=lambda *a, **k: None)

    return _done(safety, {
        "action": "walk", "direction": direction,
        "asked_mm": round(distance), "travelled_mm": round(outcome["travelled"]),
        "steps_done": outcome["steps_done"], "cancelled": outcome["cancelled"],
        "legs_short_on": {str(k): sorted(v)
                          for k, v in outcome["trouble"].items()} or None,
        "note": ("commanded distance -- feet were held planted, so any "
                 "shortfall on the floor is real slip, not scrub"),
    })


SCURRY_PERIOD_MIN, SCURRY_PERIOD_MAX = 2.0, 6.0


def scurry(arbiter, safety, distance: float = 300.0,
           direction: str = "forward", period: float = 3.0):
    """Walk quickly, via scurry.py's clock-streamed version of walk's gait.

    Reused, not reimplemented: scurry.plan() flies the whole run on paper
    first and refuses a cycle the servos cannot follow, and there is no
    --allow-fast from here. Open loop, so its own voltage guard is what
    stops it if the supply sags; a stop request ends it the same way,
    feet planted and torque on.
    """
    try:
        distance = float(distance)
        period = float(period)
    except (TypeError, ValueError):
        raise Unsafe(f"distance and period must be numbers, got "
                     f"{distance!r}, {period!r}")
    if not WALK_MIN_MM <= distance <= WALK_MAX_MM:
        raise Unsafe(f"scurry distance must be {WALK_MIN_MM:g}-"
                     f"{WALK_MAX_MM:g} mm, got {distance:g}")
    if not SCURRY_PERIOD_MIN <= period <= SCURRY_PERIOD_MAX:
        raise Unsafe(f"scurry period must be {SCURRY_PERIOD_MIN:g}-"
                     f"{SCURRY_PERIOD_MAX:g} s, got {period:g}")
    heading = HEADINGS.get(str(direction).lower())
    if heading is None:
        raise Unsafe(f"direction must be one of "
                     f"{', '.join(sorted(HEADINGS))}; got {direction!r}")

    _gate(arbiter, safety)
    import argparse                                  # noqa: PLC0415
    import scurry as prog                            # noqa: PLC0415
    from pose import Unreachable                     # noqa: PLC0415

    # Exactly scurry.py's command-line defaults, bar the three asked for.
    args = argparse.Namespace(
        distance=distance, heading=heading, period=period,
        stride=prog.DEFAULT_STRIDE, duty=prog.DEFAULT_DUTY,
        lift=prog.DEFAULT_LIFT, height=prog.DEFAULT_HEIGHT,
        foot_out=prog.DEFAULT_FOOT_OUT,
        coxa_limit=prog.DEFAULT_COXA_LIMIT, rate=prog.DEFAULT_RATE,
        min_volts=prog.DEFAULT_MIN_VOLTS)
    legs = sorted(ALL_LEGS)
    try:
        p = prog.plan(args, legs)
    except Unreachable as e:
        raise Unsafe(f"cannot scurry {distance:g} mm {direction}: {e}")
    if p["top_v"] > prog.MAX_FOLLOW_SPEED:
        raise Unsafe(f"a {period:g} s cycle needs {p['top_v']:.0f} steps/s "
                     f"from the fastest joint, over the "
                     f"{prog.MAX_FOLLOW_SPEED} the servos can be trusted to "
                     f"follow -- use a longer period")

    with arbiter.session(f"scurry {distance:g}mm {direction}") as s:
        legs = _usable_legs(s, legs)
        if len(legs) < 6:
            raise Unsafe(f"scurrying needs all six legs; {sorted(legs)} answered")
        joints = s.joints()
        measured = prog.measure_stance(s.packet, joints, legs)
        if measured and min(measured.values()) < args.height - 25.0:
            raise Unsafe(f"the body is at {min(measured.values()):.0f} mm and "
                         f"the gait assumes {args.height:g} -- stand up first")
        st = prog.Streamer(s.packet, joints, legs, _limits(s, legs))
        v0 = min(st.volts(l) for l in legs)
        if v0 < args.min_volts + 0.3:
            raise Unsafe(f"supply is {v0:.1f} V at rest, within 0.3 V of the "
                         f"{args.min_volts:g} V stop -- charge the battery")
        outcome = prog.run_scurry(st, p, args,
                                  should_cancel=lambda: arbiter.stopping,
                                  log=lambda *a, **k: None)

    stopped = outcome["stopped"]
    elapsed = outcome["elapsed"]
    return _done(safety, {
        "action": "scurry", "direction": direction, "period_s": period,
        "asked_mm": round(distance), "travelled_mm": round(outcome["travelled"]),
        "seconds": round(elapsed, 1),
        "speed_mm_s": round(outcome["travelled"] / elapsed) if elapsed else None,
        "volts_at_rest": round(v0, 1),
        "volts_lowest": outcome["min_volts"],
        "cancelled": stopped == "cancelled",
        "stopped": stopped if stopped != "cancelled" else None,
        "late_frames": outcome["late"] or None,
        "clamped_goals": st.clamped or None,
        "tracking_lag_ms": {j: r["lag_ms"] for j, r in outcome["lag"].items()}
                           or None,
        "note": ("commanded distance, open loop -- measure the floor. "
                 "Stopped early means feet planted and torque still on"
                 if stopped else
                 "commanded distance, open loop -- measure the floor"),
    })


MOTION_TOOLS = {
    "relax": relax,
    "nudge_joint": nudge,
    "lay_down": lay_down,
    "sit": sit,
    "stand_up": stand,
    "walk": walk,
    "scurry": scurry,
    "turn": turn,
    "dance": dance,
}


MOTION_HELP = {
    "relax": (
        "Drop torque so the legs go limp. If the robot is standing it "
        "WILL sit down under its own weight. Use it to stop a joint "
        "straining. Optional: legs, a list of 1-6; default all six."),
    "nudge_joint": (
        "Move one joint a small amount from where it is now: leg 1-6, "
        f"joint 'coxa'|'femur'|'tibia', degrees +/-{NUDGE_MAX_DEG:g}. "
        "Positive is forward for a coxa, up for a femur, extend for a "
        "tibia. The smallest real motion available -- prefer it when "
        "checking whether something moves at all."),
    "lay_down": (
        "Splay flat: every joint to centre, so the legs lie straight out "
        "and the body rests down. Moves coxa, then tibia, then femur, "
        "which is the order that cannot drive a foot into the floor. The "
        "safest pose to be in or pass through."),
    "sit": (
        "Fold to a low compact pose and HOLD it under torque. Unlike "
        "relax, the body is placed down deliberately rather than dropped."),
    "stand_up": (
        "Stand up from any pose, including a tangled one. Runs the full "
        "recovery: collapse, untangle leg by leg, then stand. Optional "
        "height, 40-160 mm of deck clearance, default 100. Takes tens of "
        "seconds and is the highest-current thing the robot does."),
    "walk": (
        "Walk in a straight line. distance in mm (20-1000, default 200), "
        "direction 'forward'|'backward'|'left'|'right'. The robot must "
        "already be standing. It walks sideways and backwards without "
        "turning first -- a hexagon with radial legs has no mechanical "
        "front. Feet are held planted, so a shortfall is real slip."),
    "scurry": (
        "Walk QUICKLY in a straight line: the same planted-foot gait as "
        "walk, streamed continuously instead of stopping between frames. "
        "distance in mm (20-1000, default 300), direction as for walk, "
        "period 2-6 s per gait cycle (default 3, about 67 mm/s; shorter "
        "is faster, and a period the servos cannot follow is refused). "
        "The robot must already be standing. Open loop: it stops itself "
        "if the supply sags, planting its feet and holding."),
    "turn": (
        "Turn clockwise in place by 90, 180, 270 or 360 degrees. The "
        "robot must already be standing. Always clockwise -- there is no "
        "anticlockwise turn."),
    "dance": (
        "Dance for 2-30 seconds, default 10, then return to standing. "
        "The robot must already be standing."),
}
