"""
CRABORA: 2-joint sync walk
==========================

The first multi-servo program. Drives two servos simultaneously back and
forth between their configured angle limits, using sync_goal_move so both
joints start moving on the same bus packet -- no staggering.

walk.py oscillates a single coxa; this oscillates a pair of joints, e.g.
leg 1's coxa (ID 11) and femur (ID 12) together. It is the bench test
that proves crabora_bus.sync_goal_move actually delivers coordinated
motion, and the stepping-stone toward a real walking gait.

Motion profile:
  IN-PHASE (default): both servos go to their own MAX limit together,
  then both to their own MIN, and repeat. Coordinated sweep, not a step.
  Safest first test -- proves sync motion without depending on which
  limit direction means "up" for the femur.

  ANTIPHASE (--antiphase): the second servo's endpoints are swapped, so
  when servo_a goes to MAX, servo_b goes to MIN, and vice versa. With a
  coxa + femur pair this produces a real one-leg STEPPING gait -- the
  coxa swings forward while the femur lifts, then the coxa swings back
  while the femur plants. ONLY enable this once you've eyeballed in-phase
  and you know which femur limit corresponds to "foot up" on your build,
  otherwise the leg can drive itself into the floor or a hard stop.

Arguments:
  servo_a       first servo ID (0-253)  -- typically a coxa, e.g. 11
  servo_b       second servo ID (0-253) -- typically a femur, e.g. 12
  speed         sweep speed in RPM, applied to both servos
  --antiphase   swap servo_b's endpoints (stepping motion)

Requirements (checked for BOTH servos before any motion):
  - Each must be in position mode      -> run wheel_off.py <id>
  - Each must have angle limits set    -> run set_limits.py <id>
  - The centers should be calibrated   -> run set_middle.py <id>

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. Both servos cabled to the FE-URT-2 bus (daisy-chained is fine)
  3. USB-C from the FE-URT-2 to your Mac
  4. The leg suspended on the bench so it can sweep freely

Usage:
  python walk2.py 11 12 15                # in-phase sweep, 15 rpm
  python walk2.py 11 12 15 --antiphase    # stepping motion (use with care)
"""

import argparse
import sys

from crabora_bus import (
    Bus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE, ADDR_PRESENT_POSITION,
    MODE_POSITION, MODE_NAMES,
    COUNTS_PER_REV, DEGREES_PER_REV, CENTER_POSITION, MAX_POSITION,
    MAX_VALID_ID, TYPICAL_MAX_RPM,
    describe_id,
    rpm_to_pos_speed,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Sync-walk two servos back and forth between their angle limits.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("servo_a", type=int,
                        help=f"first servo ID (0-{MAX_VALID_ID})")
    parser.add_argument("servo_b", type=int,
                        help=f"second servo ID (0-{MAX_VALID_ID})")
    parser.add_argument("speed", type=float,
                        help="sweep speed in RPM, applied to both servos")
    parser.add_argument("--antiphase", action="store_true",
                        help="swap servo_b's endpoints -- stepping motion "
                             "instead of in-phase sweep (see docstring warning)")
    return parser.parse_args()


def _alternating(a, b):
    """Yield a, b, a, b, ... forever -- the back-and-forth walk targets."""
    while True:
        yield a
        yield b


# -----------------------------------------------------------------------------
# Per-servo preflight
# -----------------------------------------------------------------------------
def preflight(bus, servo_id):
    """Ping + check position mode + read & validate angle limits.

    Prints status to the console, exits the program on any failure.
    Returns (min_limit, max_limit) on success.
    """
    label = describe_id(servo_id)

    if not bus.ping(servo_id):
        print(f"✗ No response at ID {servo_id} ({label}).")
        print("  Check the 7.4V bus power and the servo cable.")
        sys.exit(1)

    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        name = MODE_NAMES.get(mode, f"mode {mode}")
        print(f"✗ {label} (ID {servo_id}) is in {name} mode; walking needs position mode.")
        print(f"  Run: python wheel_off.py {servo_id}")
        sys.exit(1)

    mn = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
    mx = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
    if mn >= mx or (mn == 0 and mx >= MAX_POSITION):
        print(f"✗ {label} (ID {servo_id}) has no usable angle limits ({mn}..{mx}).")
        print(f"  Run: python set_limits.py {servo_id}")
        sys.exit(1)

    half_deg = (mx - mn) / 2 / COUNTS_PER_REV * DEGREES_PER_REV
    print(f"✓ {label} (ID {servo_id}): {mn}..{mx}  (±{half_deg:.1f}° around center)")
    return mn, mx


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    ids = [args.servo_a, args.servo_b]
    rpm = args.speed

    # --- validate args ---------------------------------------------------
    for sid in ids:
        if not 0 <= sid <= MAX_VALID_ID:
            print(f"✗ Servo ID {sid} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
            sys.exit(1)
    if ids[0] == ids[1]:
        print(f"✗ servo_a and servo_b must be different (both are {ids[0]}).")
        sys.exit(1)
    if rpm <= 0:
        print("✗ Speed must be positive (RPM). Try a modest value like 10 or 15.")
        sys.exit(1)

    raw_speed = rpm_to_pos_speed(rpm)
    if rpm > TYPICAL_MAX_RPM:
        print(
            f"⚠  {rpm:.1f} rpm is above the STS3215's typical max "
            f"(~{TYPICAL_MAX_RPM} rpm); both servos will just run flat out."
        )

    print("=" * 60)
    print("CRABORA: 2-joint sync walk")
    print("=" * 60)
    print(f"  servos: {ids[0]} ({describe_id(ids[0])}) + "
          f"{ids[1]} ({describe_id(ids[1])})")
    print(f"  speed : {rpm:.1f} rpm")
    print()

    with Bus() as bus:
        print(f"Opened serial port: {bus.port_path}\n")

        # --- preflight both servos ---------------------------------------
        limits = {sid: preflight(bus, sid) for sid in ids}

        # --- build the two sync target sets ------------------------------
        # The oscillation has two endpoints, A and B. In-phase: both
        # servos go to their max for A and their min for B. Antiphase:
        # servo_b's endpoints are swapped so it lifts while servo_a
        # swings forward -- a real one-leg stepping motion.
        a_id, b_id = ids
        a_lo, a_hi = limits[a_id]
        b_lo, b_hi = limits[b_id]
        if args.antiphase:
            targets_a = {a_id: (a_hi, raw_speed), b_id: (b_lo, raw_speed)}
            targets_b = {a_id: (a_lo, raw_speed), b_id: (b_hi, raw_speed)}
        else:
            targets_a = {a_id: (a_hi, raw_speed), b_id: (b_hi, raw_speed)}
            targets_b = {a_id: (a_lo, raw_speed), b_id: (b_lo, raw_speed)}
        targets_center = {sid: (CENTER_POSITION, raw_speed) for sid in ids}

        # Timeout: scale to the widest sweep, doubled, plus margin.
        widest = max(mx - mn for mn, mx in limits.values())
        sweep_timeout = widest / raw_speed * 2.0 + 1.0

        try:
            bus.sync_enable_torque(ids, on=True)

            mode_label = "ANTIPHASE — stepping" if args.antiphase else "in phase"
            print(f"\nWalking ({mode_label}). Press Ctrl+C to stop.\n")
            swing = 0
            for targets in _alternating(targets_a, targets_b):
                bus.sync_goal_move(targets)
                if not bus.sync_wait_until_stopped(ids, sweep_timeout):
                    print("  (servos slow to report stopped -- continuing)")
                swing += 1
                positions = {sid: bus.read_uint16(sid, ADDR_PRESENT_POSITION)
                             for sid in ids}
                pos_str = "   ".join(f"{sid}={p:4d}" for sid, p in positions.items())
                print(f"  swing {swing:3d}: {pos_str}")

        except KeyboardInterrupt:
            print("\n\nStopping -- returning both joints to center...")
        finally:
            try:
                bus.sync_goal_move(targets_center)
                bus.sync_wait_until_stopped(ids, sweep_timeout)
                bus.sync_enable_torque(ids, on=False)
            except IOError:
                pass  # bus already gone -- nothing more we can do

    print("✓ Both servos at center, torque off.")


if __name__ == "__main__":
    main()
