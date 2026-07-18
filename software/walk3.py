"""
CRABORA: 3-joint sync walk (full leg)
=====================================

Drives all three joints of one leg -- coxa, femur, tibia -- together via
sync_goal_move, so a single bus packet kicks off the move on every joint
at the same instant. The natural next step after walk2.py: same idea,
one more servo.

For leg 1 with the standard 2-digit ID scheme that is IDs 11 (coxa),
12 (femur), 13 (tibia). The script does not assume those IDs -- pass
whichever three you want.

Motion profile:
  IN-PHASE (default): all three servos go to their own MAX limit
  together, then all three to their own MIN, and repeat. Coordinated
  sweep across the whole leg. Safest first test -- proves the 3-way
  sync without depending on which direction is "foot up" on your build.

  ANTIPHASE (--antiphase): servo_b AND servo_c have their endpoints
  swapped relative to servo_a, so when servo_a goes to MAX the other
  two go to MIN, and vice versa. With a coxa-femur-tibia leg this
  produces a one-leg STEPPING motion -- the coxa swings forward while
  the femur and tibia both move toward their "lifted" side, then the
  coxa swings back while the femur and tibia move toward "planted".

  Caveat: whether "lifted" corresponds to a servo's HI limit or LO
  limit depends entirely on the joint's mounting orientation on your
  build. Always run in-phase first to confirm the sweep is mechanically
  sane, THEN turn on --antiphase. If the foot drives into the floor or
  the leg fights a hard stop, you've got the swap on the wrong side and
  need to flip the limits in set_limits.py for one of the followers.

Arguments:
  servo_a       lead servo ID -- typically the coxa, e.g. 11
  servo_b       second servo  -- typically the femur, e.g. 12
  servo_c       third servo   -- typically the tibia, e.g. 13
  speed         sweep speed in RPM, applied to all three servos
  --antiphase   swap servo_b AND servo_c relative to servo_a (stepping)

Requirements (checked for ALL three servos before any motion):
  - Each must be in position mode      -> run wheel_off.py <id>
  - Each must have angle limits set    -> run set_limits.py <id>
  - The centers should be calibrated   -> run set_middle.py <id>

Hardware setup (same as pol.py):
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. All three servos daisy-chained on the FE-URT-2 bus
  3. USB-C from the FE-URT-2 to your Mac
  4. The leg suspended on the bench so it can sweep freely

Usage:
  python walk3.py 11 12 13 15                # in-phase sweep, 15 rpm
  python walk3.py 11 12 13 15 --antiphase    # stepping motion (use with care)
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
        description="Sync-walk three servos back and forth between their angle limits.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("servo_a", type=int,
                        help=f"lead servo ID (0-{MAX_VALID_ID}) -- typically the coxa")
    parser.add_argument("servo_b", type=int,
                        help=f"second servo ID (0-{MAX_VALID_ID}) -- typically the femur")
    parser.add_argument("servo_c", type=int,
                        help=f"third servo ID (0-{MAX_VALID_ID}) -- typically the tibia")
    parser.add_argument("speed", type=float,
                        help="sweep speed in RPM, applied to all three servos")
    parser.add_argument("--antiphase", action="store_true",
                        help="swap servo_b AND servo_c endpoints relative to servo_a "
                             "-- stepping motion (see docstring warning)")
    return parser.parse_args()


def _alternating(a, b):
    """Yield a, b, a, b, ... forever -- the two endpoints of the cycle."""
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
    ids = [args.servo_a, args.servo_b, args.servo_c]
    rpm = args.speed

    # --- validate args ---------------------------------------------------
    for sid in ids:
        if not 0 <= sid <= MAX_VALID_ID:
            print(f"✗ Servo ID {sid} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
            sys.exit(1)
    if len(set(ids)) != 3:
        print(f"✗ servo_a, servo_b and servo_c must all be different (got {ids}).")
        sys.exit(1)
    if rpm <= 0:
        print("✗ Speed must be positive (RPM). Try a modest value like 10 or 15.")
        sys.exit(1)

    raw_speed = rpm_to_pos_speed(rpm)
    if rpm > TYPICAL_MAX_RPM:
        print(
            f"⚠  {rpm:.1f} rpm is above the STS3215's typical max "
            f"(~{TYPICAL_MAX_RPM} rpm); all servos will just run flat out."
        )

    print("=" * 60)
    print("CRABORA: 3-joint sync walk (full leg)")
    print("=" * 60)
    print(f"  servos: " + "  ".join(f"{sid} ({describe_id(sid)})" for sid in ids))
    print(f"  speed : {rpm:.1f} rpm")
    print()

    with Bus() as bus:
        print(f"Opened serial port: {bus.port_path}\n")

        # --- preflight all three servos ----------------------------------
        limits = {sid: preflight(bus, sid) for sid in ids}

        # --- build the two sync endpoint sets ----------------------------
        # The oscillation has two endpoints, A and B. In-phase: all three
        # joints go to their max for A, all to min for B. Antiphase: the
        # two follower servos (servo_b, servo_c) are swapped so they move
        # opposite to servo_a -- the coxa-forward-while-foot-lifts pattern.
        a_id, b_id, c_id = ids
        a_lo, a_hi = limits[a_id]
        b_lo, b_hi = limits[b_id]
        c_lo, c_hi = limits[c_id]
        if args.antiphase:
            targets_a = {a_id: (a_hi, raw_speed),
                         b_id: (b_lo, raw_speed),
                         c_id: (c_lo, raw_speed)}
            targets_b = {a_id: (a_lo, raw_speed),
                         b_id: (b_hi, raw_speed),
                         c_id: (c_hi, raw_speed)}
        else:
            targets_a = {sid: (limits[sid][1], raw_speed) for sid in ids}
            targets_b = {sid: (limits[sid][0], raw_speed) for sid in ids}
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
            print("\n\nStopping -- returning all three joints to center...")
        finally:
            try:
                bus.sync_goal_move(targets_center)
                bus.sync_wait_until_stopped(ids, sweep_timeout)
                bus.sync_enable_torque(ids, on=False)
            except IOError:
                pass  # bus already gone -- nothing more we can do

    print("✓ All three servos at center, torque off.")


if __name__ == "__main__":
    main()
