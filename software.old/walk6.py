"""
CRABORA: 6-joint sync walk (two legs, alternating gait)
========================================================

Drives two legs (six joints total) via sync_goal_move, with the two
legs locked 180 degrees out of phase: while leg 1 is at endpoint A,
leg 2 is at endpoint B, and vice versa.  When the legs are also in
stepping mode (--antiphase), this gives you the foundational 2-leg
walking gait: while one leg lifts its foot and swings its coxa
forward, the other leg keeps its foot planted and swings its coxa
backward to push the body forward.  Flip, repeat.

This is the natural next step after walk3.py: same idea, three more
servos, one more dimension of coordination (between legs as well as
within them).  Servos for leg 1 with the standard 2-digit ID scheme
are 11 (coxa), 12 (femur), 13 (tibia); leg 2 is 21/22/23.  The
script does not assume those IDs -- pass whichever six you want, in
(coxa, femur, tibia) order for each leg.

Motion profile:
  IN-PHASE (default): within each leg, all three joints go to their
  own MAX limit together, then all three to MIN.  The two LEGS are
  still 180 degrees out of phase with each other -- when leg 1 is
  at MAX, leg 2 is at MIN.  This is the safe first test: it
  confirms the 6-way sync, the two-leg phase relationship, and that
  each leg's sweep direction is sane, WITHOUT yet driving the
  stepping motion.

  ANTIPHASE (--antiphase): within each leg the femur and tibia are
  swapped relative to the coxa (exact same convention as walk3
  --antiphase) -- the foot lifts when the coxa swings forward, and
  plants when the coxa swings back.  Between the legs the gait is
  unchanged: leg 1 lifts and swings forward while leg 2 plants and
  pushes the body forward, then they swap.

  Caveat: whether the femur and tibia "lifted" direction maps to
  the HI or the LO firmware limit is build-specific (depends on each
  servo's mounting orientation).  Run IN-PHASE first to confirm both
  legs sweep sensibly, THEN turn on --antiphase.  If a foot drives
  into the floor or a leg fights a hard stop, flip the limits in
  set_limits.py for one of the followers.

  Caveat 2: real walking gaits care about TIMING within a step (a
  swing typically takes 1/3 of a cycle, stance the other 2/3).  This
  script just alternates between two endpoint sets and waits for
  arrival -- so SWING and STANCE take the same wall-clock time.
  Good enough to test the kinematics and the bus.  Real gait timing
  will come with the alternating-tripod gait.

Arguments:
  l1_coxa l1_femur l1_tibia     leg 1 servo IDs in CFT order
  l2_coxa l2_femur l2_tibia     leg 2 servo IDs in CFT order
  speed                          sweep speed in RPM, applied to all six
  --antiphase                    within each leg, swap femur+tibia
                                 relative to the coxa (stepping motion;
                                 see docstring warning)

Requirements (checked for ALL SIX servos before any motion):
  - Each must be in position mode      -> run wheel_off.py <id>
  - Each must have angle limits set    -> run set_limits.py <id>
  - The centers should be calibrated   -> run set_middle.py <id>

Hardware setup:
  1. 7.4-7.5V supply into the FE-URT-2 barrel jack
  2. All six servos daisy-chained on the FE-URT-2 bus
  3. USB-C from the FE-URT-2 to your Mac
  4. Both legs suspended on the bench so they can sweep freely
     (do NOT run this on the floor on day 1)

Usage:
  python walk6.py 11 12 13 21 22 23 12               # in-phase, 12 rpm
  python walk6.py 11 12 13 21 22 23 12 --antiphase   # 2-leg stepping
"""

import argparse
import sys

from crabora_bus import (
    Bus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE,
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
        description="Sync-walk two legs (six servos) with the legs 180 degrees "
                    "out of phase -- the foundation of a 2-leg walking gait.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("l1_coxa",  type=int, help="leg 1 coxa  servo ID (e.g. 11)")
    parser.add_argument("l1_femur", type=int, help="leg 1 femur servo ID (e.g. 12)")
    parser.add_argument("l1_tibia", type=int, help="leg 1 tibia servo ID (e.g. 13)")
    parser.add_argument("l2_coxa",  type=int, help="leg 2 coxa  servo ID (e.g. 21)")
    parser.add_argument("l2_femur", type=int, help="leg 2 femur servo ID (e.g. 22)")
    parser.add_argument("l2_tibia", type=int, help="leg 2 tibia servo ID (e.g. 23)")
    parser.add_argument("speed", type=float,
                        help="sweep speed in RPM, applied to all six servos")
    parser.add_argument("--antiphase", action="store_true",
                        help="within each leg, swap femur+tibia endpoints "
                             "relative to the coxa -- stepping motion "
                             "(see docstring warning)")
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
# Endpoint builder
# -----------------------------------------------------------------------------
def build_endpoints(ids, limits, raw_speed, antiphase):
    """Build the two sync target dicts that the legs alternate between.

    Naming: A and B are the two endpoints of the cycle.  Leg 1 starts
    at A, leg 2 starts at B; one swing later they've swapped.

    In-phase  : per leg, all three joints go to MAX or all to MIN
                together.  Leg 1 MAX <-> Leg 2 MIN (still alternating
                between legs).

    Antiphase : per leg, coxa goes one way while femur+tibia go the
                other (matches walk3 --antiphase).  Leg 1 and leg 2
                still mirrored.
    """
    l1c, l1f, l1t, l2c, l2f, l2t = ids

    def lo(sid): return limits[sid][0]
    def hi(sid): return limits[sid][1]

    if antiphase:
        # Endpoint A: leg 1 in "coxa fwd + foot lifted" stance,
        #             leg 2 in the opposite "coxa back + foot planted".
        a_pos = {
            l1c: hi(l1c), l1f: lo(l1f), l1t: lo(l1t),
            l2c: lo(l2c), l2f: hi(l2f), l2t: hi(l2t),
        }
        # Endpoint B: legs swap.
        b_pos = {
            l1c: lo(l1c), l1f: hi(l1f), l1t: hi(l1t),
            l2c: hi(l2c), l2f: lo(l2f), l2t: lo(l2t),
        }
    else:
        # Endpoint A: leg 1 all-MAX, leg 2 all-MIN.
        a_pos = {
            l1c: hi(l1c), l1f: hi(l1f), l1t: hi(l1t),
            l2c: lo(l2c), l2f: lo(l2f), l2t: lo(l2t),
        }
        # Endpoint B: legs swap.
        b_pos = {
            l1c: lo(l1c), l1f: lo(l1f), l1t: lo(l1t),
            l2c: hi(l2c), l2f: hi(l2f), l2t: hi(l2t),
        }

    targets_a = {sid: (pos, raw_speed) for sid, pos in a_pos.items()}
    targets_b = {sid: (pos, raw_speed) for sid, pos in b_pos.items()}
    return targets_a, targets_b


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    ids = [args.l1_coxa, args.l1_femur, args.l1_tibia,
           args.l2_coxa, args.l2_femur, args.l2_tibia]
    rpm = args.speed

    # --- validate args ---------------------------------------------------
    for sid in ids:
        if not 0 <= sid <= MAX_VALID_ID:
            print(f"✗ Servo ID {sid} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
            sys.exit(1)
    if len(set(ids)) != 6:
        print(f"✗ All six servo IDs must be different (got {ids}).")
        sys.exit(1)
    if rpm <= 0:
        print("✗ Speed must be positive (RPM). Try a modest value like 8 or 12.")
        sys.exit(1)

    raw_speed = rpm_to_pos_speed(rpm)
    if rpm > TYPICAL_MAX_RPM:
        print(
            f"⚠  {rpm:.1f} rpm is above the STS3215's typical max "
            f"(~{TYPICAL_MAX_RPM} rpm); all servos will just run flat out."
        )

    print("=" * 64)
    print("CRABORA: 6-joint sync walk (two legs, alternating gait)")
    print("=" * 64)
    print(f"  leg 1 : " + "  ".join(
        f"{sid} ({describe_id(sid)})" for sid in ids[:3]))
    print(f"  leg 2 : " + "  ".join(
        f"{sid} ({describe_id(sid)})" for sid in ids[3:]))
    print(f"  speed : {rpm:.1f} rpm")
    print()

    with Bus() as bus:
        print(f"Opened serial port: {bus.port_path}\n")

        # --- preflight all six servos ------------------------------------
        limits = {sid: preflight(bus, sid) for sid in ids}

        # --- build the two sync endpoint sets ----------------------------
        targets_a, targets_b = build_endpoints(ids, limits, raw_speed,
                                               antiphase=args.antiphase)
        targets_center = {sid: (CENTER_POSITION, raw_speed) for sid in ids}

        # Timeout: scale to the widest sweep, doubled, plus margin.
        widest = max(mx - mn for mn, mx in limits.values())
        sweep_timeout = widest / raw_speed * 2.0 + 1.0

        try:
            bus.sync_enable_torque(ids, on=True)

            mode_label = ("ANTIPHASE — 2-leg stepping" if args.antiphase
                          else "in-phase — legs only alternating")
            print(f"\nWalking ({mode_label}). Press Ctrl+C to stop.\n")
            swing = 0
            for targets in _alternating(targets_a, targets_b):
                bus.sync_goal_move(targets)
                if not bus.sync_wait_until_stopped(ids, sweep_timeout):
                    print("  (servos slow to report stopped -- continuing)")
                swing += 1
                telem = bus.read_telemetry(ids)
                l1_str = " ".join(f"{sid}={telem[sid]['position']:4d}" for sid in ids[:3] if sid in telem)
                l2_str = " ".join(f"{sid}={telem[sid]['position']:4d}" for sid in ids[3:] if sid in telem)
                # voltage and temp: average across responding servos
                voltages = [t["voltage_v"] for t in telem.values()]
                temps    = [t["temp_c"]    for t in telem.values()]
                v_str = f"{sum(voltages)/len(voltages):.1f}V" if voltages else "?V"
                t_str = f"{max(temps)}°C"                      if temps    else "?°C"
                print(f"  swing {swing:3d}:  L1 {l1_str}   L2 {l2_str}   [{v_str} {t_str}]")

        except KeyboardInterrupt:
            print("\n\nStopping -- returning all six joints to center...")
        finally:
            try:
                bus.sync_goal_move(targets_center)
                bus.sync_wait_until_stopped(ids, sweep_timeout)
                bus.sync_enable_torque(ids, on=False)
            except IOError:
                pass  # bus already gone -- nothing more we can do

    print("✓ All six servos at center, torque off.")


if __name__ == "__main__":
    main()
