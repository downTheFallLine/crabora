"""
CRABORA pose -- leg kinematics and the named poses used when standing up
==========================================================================

Two-link inverse kinematics in a leg's vertical plane, plus the handful
of named poses the recovery sequence moves between.

The leg's vertical plane contains the femur and tibia. The coxa rotates
that whole plane about the body's vertical axis, and the plane is
OFFSET sideways from the coxa axis by COXA_TANGENT -- see walk/README.md.
That offset does not affect the two-link solve below, which works from
the femur axis outward; it matters only when converting to body-frame
coordinates.

Angle conventions, matching walk/gait_lib's sign tables:
    femur  -- degrees up from horizontal, positive up
    tibia  -- degrees of knee flexion from straight, positive folds the
              foot DOWN relative to the femur's direction
    coxa   -- degrees about the body's vertical axis, CCW positive

⚠ The tibia convention inherits the ASSUMED tibia sign. See README.md.

Everything here is pure geometry -- no bus, no servos, no side effects,
so it can be exercised without hardware. Run this file directly for a
self-check of the IK against poses whose answers are known by hand.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, BODY_TO_COXA_AXIS, COXA_RADIAL, COXA_TANGENT, COXA_VERTICAL,
    FEMUR_LEN, TIBIA_LEN,
)

# Radius from the body centre out to the femur axis (the knee), with the
# coxa at centre. The two-link solve works outward from here.
KNEE_RADIUS = BODY_TO_COXA_AXIS + COXA_RADIAL     # 221.22 mm

STRAIGHT_REACH = FEMUR_LEN + TIBIA_LEN            # 220.0 mm
MIN_REACH = abs(FEMUR_LEN - TIBIA_LEN)            # 20.0 mm

# How far below the deck plate the body actually rests when belly-down.
# The flanged deck hangs FLANGE_DEPTH below the plate underside.
BELLY_DROP = 25.0


class Unreachable(ValueError):
    """Asked for a foot position the leg cannot reach."""


def leg_ik(out, down):
    """Solve the femur/tibia angles that put the foot at (out, down),
    measured FROM THE FEMUR AXIS in the leg's vertical plane.

    `out`  -- horizontal distance outward from the femur axis (mm)
    `down` -- how far below the femur axis (mm, positive = below)

    Returns (femur_up_deg, tibia_flex_deg).

    Raises Unreachable if the point is outside the annulus the leg can
    reach, which is the useful failure: better to refuse than to clamp
    silently and produce a pose that is subtly wrong.
    """
    r = math.hypot(out, down)
    if r > STRAIGHT_REACH:
        raise Unreachable(
            f"{r:.1f} mm from the femur axis is beyond the leg's "
            f"{STRAIGHT_REACH:.0f} mm straight reach")
    if r < MIN_REACH:
        raise Unreachable(
            f"{r:.1f} mm from the femur axis is inside the leg's "
            f"{MIN_REACH:.0f} mm minimum fold")

    # Knee flexion, from the law of cosines on the femur/tibia/chord
    # triangle. The interior angle at the knee is (pi - flex), so the
    # usual minus sign becomes a plus.
    cos_flex = (r * r - FEMUR_LEN ** 2 - TIBIA_LEN ** 2) / (
        2 * FEMUR_LEN * TIBIA_LEN)
    flex = math.acos(max(-1.0, min(1.0, cos_flex)))

    # Femur angle: the chord's own elevation, plus the offset between the
    # chord and the femur.
    chord = math.atan2(-down, out)
    cos_off = (r * r + FEMUR_LEN ** 2 - TIBIA_LEN ** 2) / (2 * r * FEMUR_LEN)
    offset = math.acos(max(-1.0, min(1.0, cos_off)))

    return math.degrees(chord + offset), math.degrees(flex)


def leg_fk(femur_up_deg, tibia_flex_deg):
    """Inverse of leg_ik: foot position (out, down) from the femur axis."""
    beta = math.radians(femur_up_deg)
    gamma = beta - math.radians(tibia_flex_deg)
    out = FEMUR_LEN * math.cos(beta) + TIBIA_LEN * math.cos(gamma)
    down = -(FEMUR_LEN * math.sin(beta) + TIBIA_LEN * math.sin(gamma))
    return out, down


# =============================================================
# NAMED POSES
# =============================================================
# Each returns {joint: degrees} with keys "coxa", "femur", "tibia",
# in the conventions above. The coxa is at 0 (its nominal bearing) for
# every one of these -- the recovery never asks a leg to stand off-axis.

def pose_splay():
    """Legs straight out, horizontal. This is simply every joint at its
    own centre, which is exactly how the conventions were chosen: coxa
    neutral, femur horizontal, tibia collinear.

    With the body belly-down this is the natural resting splay, and it
    is the safest pose to pass through -- nothing is folded, so nothing
    can be caught on anything."""
    return {"coxa": 0.0, "femur": 0.0, "tibia": 0.0}


def pose_compact(out=140.0, ground_clear=70.0):
    """Leg drawn in and lifted clear: the foot pulled to `out` from the
    femur axis and held `ground_clear` mm above the floor. This is what
    a leg is put into BEFORE its coxa rotates, so a short stub sweeps
    past its neighbours instead of a 440 mm lever.

    Specified as a foot POSITION, not joint angles, for two reasons.
    First, what matters is clearance -- radial (shorter than the 220 mm
    straight leg) and vertical (off the floor). Second, picking angles
    by eye gets the vertical wrong: femur +35 with 90 of flex looks
    lifted but puts the foot on the floor, because flexion bends the
    foot DOWN from the femur's direction.

    ground_clear was raised 30 -> 70 mm on 2026-10-11: on hardware, 30 mm
    did not lift the femur enough for the leg to swing cleanly. At
    out=140 that moves the femur from +52 to +67, putting the foot 27 mm
    ABOVE the femur axis rather than 13 below. Tunable from stand.py.

    Do not push this much further at a small `out`: asking for a lot of
    height close in needs femur angles near +90, at the edge of the
    envelope and likely outside the firmware limits."""
    femur, tibia = leg_ik(out=out, down=BELLY_DROP + COXA_VERTICAL - ground_clear)
    return {"coxa": 0.0, "femur": femur, "tibia": tibia}


def pose_tuck(ground_clear=10.0):
    """Feet just off the ground with the body belly-down -- the known
    state the untangle phase drives every leg to, and the state the
    stand phase starts from.

    The body is resting, so the femur axis sits BELLY_DROP above the
    ground plus COXA_VERTICAL. The foot is placed a little outboard of
    the knee so the first push has somewhere to go."""
    down = BELLY_DROP + COXA_VERTICAL - ground_clear
    femur, tibia = leg_ik(out=90.0, down=down)
    return {"coxa": 0.0, "femur": femur, "tibia": tibia}


def pose_stand(height, foot_out=120.0):
    """Standing at `height` mm of ground clearance under the deck plate.

    `foot_out` is how far outboard of the femur axis the foot is planted.
    Bigger means a wider, more stable stance but less vertical range
    left; smaller stacks the leg up under the body and runs out of reach.
    """
    down = height + COXA_VERTICAL
    femur, tibia = leg_ik(out=foot_out, down=down)
    return {"coxa": 0.0, "femur": femur, "tibia": tibia}


def stand_height_range(foot_out=120.0):
    """(min, max) deck heights reachable with the foot `foot_out` from
    the femur axis. Useful for refusing a --height the leg cannot do."""
    lo_sq = MIN_REACH ** 2 - foot_out ** 2
    hi_sq = STRAIGHT_REACH ** 2 - foot_out ** 2
    if hi_sq <= 0:
        raise Unreachable(f"foot_out {foot_out} exceeds the leg's reach")
    lo = math.sqrt(lo_sq) if lo_sq > 0 else 0.0
    return max(0.0, lo - COXA_VERTICAL), math.sqrt(hi_sq) - COXA_VERTICAL


def foot_in_body_frame(leg, coxa_deg, femur_deg, tibia_flex_deg):
    """Foot position in the body frame, carrying the tangent offset.

    Frame: +X forward through the front point, +Y left, +Z up.
    """
    from gait_lib import LEG_BEARING
    bearing = math.radians(LEG_BEARING[leg])
    phi = -bearing + math.radians(coxa_deg)
    radial = (math.cos(phi), math.sin(phi))
    tangent = (-math.sin(phi), math.cos(phi))

    b = -bearing
    x = BODY_TO_COXA_AXIS * math.cos(b) + COXA_RADIAL * radial[0] + \
        COXA_TANGENT * tangent[0]
    y = BODY_TO_COXA_AXIS * math.sin(b) + COXA_RADIAL * radial[1] + \
        COXA_TANGENT * tangent[1]
    z = COXA_VERTICAL

    out, down = leg_fk(femur_deg, tibia_flex_deg)
    return x + out * radial[0], y + out * radial[1], z - down


# =============================================================
# SELF-CHECK
# =============================================================
def _check():
    """Exercise the IK against cases whose answers are known by hand."""
    ok = True

    def near(got, want, tol=0.05, what=""):
        nonlocal ok
        if abs(got - want) > tol:
            ok = False
            print(f"  FAIL {what}: got {got:.3f}, want {want:.3f}")
        return abs(got - want) <= tol

    print("Leg IK self-check")
    print(f"  femur {FEMUR_LEN:g}, tibia {TIBIA_LEN:g}, "
          f"reach {MIN_REACH:g}..{STRAIGHT_REACH:g} mm\n")

    # Straight out and level: no flexion, femur horizontal.
    f, t = leg_ik(out=STRAIGHT_REACH, down=0.0)
    near(f, 0.0, what="straight out, femur"); near(t, 0.0, what="straight out, flex")

    # Straight down: femur -90, still no flexion.
    f, t = leg_ik(out=0.0, down=STRAIGHT_REACH)
    near(f, -90.0, what="straight down, femur"); near(t, 0.0, what="straight down, flex")

    # Femur horizontal with the knee at a right angle puts the foot
    # FEMUR_LEN out and TIBIA_LEN down.
    f, t = leg_ik(out=FEMUR_LEN, down=TIBIA_LEN)
    near(f, 0.0, what="right-angle knee, femur"); near(t, 90.0, what="right-angle knee, flex")

    # FK must invert IK over a spread of reachable points.
    worst = 0.0
    for out in range(-150, 201, 25):
        for down in range(-150, 201, 25):
            r = math.hypot(out, down)
            if not MIN_REACH + 1 < r < STRAIGHT_REACH - 1:
                continue
            f, t = leg_ik(out, down)
            o2, d2 = leg_fk(f, t)
            worst = max(worst, abs(o2 - out), abs(d2 - down))
    if worst > 0.01:
        ok = False
        print(f"  FAIL round trip: worst error {worst:.4f} mm")
    else:
        print(f"  round trip IK->FK over the reachable annulus: "
              f"worst error {worst:.2e} mm")

    # Out-of-reach must raise rather than clamp.
    for out, down, why in ((300.0, 0.0, "beyond straight reach"),
                           (0.0, 5.0, "inside minimum fold")):
        try:
            leg_ik(out, down)
            ok = False
            print(f"  FAIL {why}: should have raised Unreachable")
        except Unreachable:
            pass
    print("  out-of-reach targets raise Unreachable")

    # Named poses must be self-consistent.
    print()
    ground = BELLY_DROP + COXA_VERTICAL   # below the femur axis, belly-down
    print(f"  (belly-down, the ground is {ground:.1f} mm below the femur axis)")
    for name, p in (("splay", pose_splay()), ("compact", pose_compact()),
                    ("tuck", pose_tuck())):
        out, down = leg_fk(p["femur"], p["tibia"])
        clear = ground - down
        print(f"  {name:<8} femur {p['femur']:+7.1f}°  flex {p['tibia']:6.1f}°"
              f"   -> foot {out:6.1f} out, {down:6.1f} down"
              f"   ({clear:+6.1f} mm ground clearance)")
        if clear < 0:
            ok = False
            print(f"    FAIL {name}: foot is {-clear:.1f} mm BELOW the ground")
    lo, hi = stand_height_range()
    print(f"\n  stand height range at foot_out=120: "
          f"{lo:.0f} .. {hi:.0f} mm of deck clearance")
    for h in (40, 80, 120, 160):
        try:
            p = pose_stand(h)
            print(f"    height {h:>3} mm -> femur {p['femur']:+6.1f}°, "
                  f"flex {p['tibia']:5.1f}°")
        except Unreachable as e:
            print(f"    height {h:>3} mm -> unreachable ({e})")

    print("\n" + ("Self-check PASSED" if ok else "Self-check FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_check())
