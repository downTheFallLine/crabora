"""
CRABORA: 3-joint sync swing + fold-to-park (walk3 mod)
======================================================

A modification of walk3.py. Two differences:

  1. SWING IS BOUNDED, not limit-to-limit. Every joint oscillates
     +/- SWING degrees around its calibrated CENTER (default +/- 30),
     instead of slamming between its firmware MIN/MAX angle limits.
     This is the gentle "rock around center" motion.

  2. IT ENDS IN A PARK / FOLD POSE, not at center with torque off.
     After the swing cycles finish (or you press Ctrl+C), the leg moves
     to a tucked pose that pulls the centre of mass in tight against
     the 2x4. Torque is released once it arrives (pass --hold to keep
     it driven and locked in the fold):

       - coxa  (servo_a): back to CENTER -- "90 degrees, like it is now"
       - femur (servo_b): raised so the femur link stands VERTICAL
       - tibia (servo_c): folded ~90 deg back over the top of the arm

     The park targets are expressed as DEGREES FROM CENTER and are easy
     to tune (constants below, or --*-park overrides). Their SIGN is
     build-specific -- which way is "up" / "folded" depends on how each
     servo is mounted -- so verify on the bench and flip the sign if a
     joint goes the wrong way. Every park target is clamped to the
     servo's firmware angle limits, so a wrong number stalls at a limit
     rather than fighting a hard stop, but confirm direction at low speed.

Servo roles (leg 1, standard 2-digit IDs): 11 coxa, 12 femur, 13 tibia.
The script does not assume those -- pass whichever three you want, in
coxa, femur, tibia order.

Requirements (checked for ALL three before any motion):
  - position mode      -> wheel_off.py <id>
  - angle limits set   -> set_limits.py <id>
  - centers calibrated -> set_middle.py <id>      (CENTER must be true mid)

Usage:
  python walk4.py 11 12 13 15                 # +/-30 swing forever; Ctrl+C to fold
  python walk4.py 11 12 13 15 --swing 20      # gentler +/-20 swing
  python walk4.py 11 12 13 15 --cycles 3      # stop after 3 cycles, then fold
  python walk4.py 11 12 13 15 --femur-park -90 --tibia-park -90   # flip fold dir
  python walk4.py 11 12 13 15 --hold          # stay driven, locked in the fold
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
# Tunable defaults -- the whole point of this script. Edit here or override on
# the command line. All values are DEGREES; park values are degrees-from-center
# (signed -- positive raises counts, which may be "up" or "down" on your build).
# -----------------------------------------------------------------------------
DEFAULT_SWING_DEG = 30.0     # +/- this around center, all three joints
DEFAULT_CYCLES    = 0        # 0 = swing forever until Ctrl+C; >0 = that many cycles

COXA_PARK_DEG  = 0.0         # coxa ends at center ("90 deg, like it is now")
FEMUR_PARK_DEG = 90.0        # femur up to vertical
TIBIA_PARK_DEG = 90.0        # tibia folded back over the top of the arm


def deg_to_counts(deg):
    """Signed degrees -> signed encoder counts (4096 counts / 360 deg)."""
    return round(deg * COUNTS_PER_REV / DEGREES_PER_REV)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Swing three joints +/- N deg around center, then fold to a "
                    "tucked park pose and hold.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("servo_a", type=int, help="coxa servo ID")
    parser.add_argument("servo_b", type=int, help="femur servo ID")
    parser.add_argument("servo_c", type=int, help="tibia servo ID")
    parser.add_argument("speed", type=float, help="speed in RPM for all joints")
    parser.add_argument("--swing", type=float, default=DEFAULT_SWING_DEG,
                        help=f"swing amplitude in deg around center "
                             f"(default {DEFAULT_SWING_DEG:g})")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                        help=f"back-and-forth cycles then park; 0 = swing forever "
                             f"until Ctrl+C (default {DEFAULT_CYCLES})")
    parser.add_argument("--antiphase", action="store_true",
                        help="femur & tibia swing opposite the coxa (stepping)")
    parser.add_argument("--coxa-park", type=float, default=COXA_PARK_DEG,
                        help=f"coxa park, deg from center (default {COXA_PARK_DEG:g})")
    parser.add_argument("--femur-park", type=float, default=FEMUR_PARK_DEG,
                        help=f"femur park, deg from center (default {FEMUR_PARK_DEG:g})")
    parser.add_argument("--tibia-park", type=float, default=TIBIA_PARK_DEG,
                        help=f"tibia park, deg from center (default {TIBIA_PARK_DEG:g})")
    parser.add_argument("--hold", action="store_true",
                        help="keep torque on after parking to hold the fold "
                             "(default: release / go limp)")
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Per-servo preflight (same checks as walk3.py)
# -----------------------------------------------------------------------------
def preflight(bus, servo_id):
    """Ping + position-mode + angle-limit check. Exits on failure.

    Returns (min_limit, max_limit).
    """
    label = describe_id(servo_id)

    if not bus.ping(servo_id):
        print(f"✗ No response at ID {servo_id} ({label}).")
        print("  Check the 7.4V bus power and the servo cable.")
        sys.exit(1)

    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        name = MODE_NAMES.get(mode, f"mode {mode}")
        print(f"✗ {label} (ID {servo_id}) is in {name} mode; needs position mode.")
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


def clamp_target(servo_id, label, counts, limits):
    """Clamp an absolute count target to a servo's [min,max]; warn if clamped."""
    mn, mx = limits[servo_id]
    if counts < mn or counts > mx:
        clamped = max(mn, min(mx, counts))
        want_deg = (counts - CENTER_POSITION) / COUNTS_PER_REV * DEGREES_PER_REV
        got_deg = (clamped - CENTER_POSITION) / COUNTS_PER_REV * DEGREES_PER_REV
        print(f"  ⚠  {label} target {want_deg:+.1f}° is outside its limits; "
              f"clamping to {got_deg:+.1f}° ({clamped}).")
        return clamped
    return counts


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    ids = [args.servo_a, args.servo_b, args.servo_c]
    a_id, b_id, c_id = ids
    rpm = args.speed

    # --- validate args ---------------------------------------------------
    for sid in ids:
        if not 0 <= sid <= MAX_VALID_ID:
            print(f"✗ Servo ID {sid} out of range (0-{MAX_VALID_ID}).")
            sys.exit(1)
    if len(set(ids)) != 3:
        print(f"✗ The three servo IDs must all differ (got {ids}).")
        sys.exit(1)
    if rpm <= 0:
        print("✗ Speed must be positive (RPM). Try 10 or 15.")
        sys.exit(1)
    if args.swing <= 0:
        print("✗ --swing must be positive (degrees).")
        sys.exit(1)

    raw_speed = rpm_to_pos_speed(rpm)
    if rpm > TYPICAL_MAX_RPM:
        print(f"⚠  {rpm:.1f} rpm is above the STS3215 typical max "
              f"(~{TYPICAL_MAX_RPM} rpm); servos will just run flat out.")

    swing_counts = deg_to_counts(args.swing)

    print("=" * 60)
    print("CRABORA: 3-joint swing + fold-to-park (walk3 mod)")
    print("=" * 60)
    print("  servos: " + "  ".join(f"{sid} ({describe_id(sid)})" for sid in ids))
    print(f"  speed : {rpm:.1f} rpm")
    print(f"  swing : ±{args.swing:g}° around center"
          + ("  (antiphase)" if args.antiphase else "  (in phase)"))
    print(f"  cycles: {'until Ctrl+C' if args.cycles == 0 else args.cycles}")
    print(f"  park  : coxa {args.coxa_park:+g}°, femur {args.femur_park:+g}°, "
          f"tibia {args.tibia_park:+g}°  (deg from center)")
    print()

    with Bus() as bus:
        print(f"Opened serial port: {bus.port_path}\n")

        limits = {sid: preflight(bus, sid) for sid in ids}

        # --- swing endpoints: center +/- swing, clamped to limits --------
        hi = {sid: clamp_target(sid, describe_id(sid),
                                CENTER_POSITION + swing_counts, limits)
              for sid in ids}
        lo = {sid: clamp_target(sid, describe_id(sid),
                                CENTER_POSITION - swing_counts, limits)
              for sid in ids}

        if args.antiphase:
            # coxa leads; femur & tibia swing opposite.
            targets_a = {a_id: (hi[a_id], raw_speed),
                         b_id: (lo[b_id], raw_speed),
                         c_id: (lo[c_id], raw_speed)}
            targets_b = {a_id: (lo[a_id], raw_speed),
                         b_id: (hi[b_id], raw_speed),
                         c_id: (hi[c_id], raw_speed)}
        else:
            targets_a = {sid: (hi[sid], raw_speed) for sid in ids}
            targets_b = {sid: (lo[sid], raw_speed) for sid in ids}

        # --- park / fold pose, clamped to limits -------------------------
        park = {
            a_id: clamp_target(a_id, "coxa",
                               CENTER_POSITION + deg_to_counts(args.coxa_park),
                               limits),
            b_id: clamp_target(b_id, "femur",
                               CENTER_POSITION + deg_to_counts(args.femur_park),
                               limits),
            c_id: clamp_target(c_id, "tibia",
                               CENTER_POSITION + deg_to_counts(args.tibia_park),
                               limits),
        }
        targets_park = {sid: (park[sid], raw_speed) for sid in ids}

        # Timeout: scale to the widest move that can happen, doubled + margin.
        widest = max(mx - mn for mn, mx in limits.values())
        move_timeout = widest / raw_speed * 2.0 + 1.0

        def go_park():
            print("\nFolding to park pose...")
            bus.sync_goal_move(targets_park)
            bus.sync_wait_until_stopped(ids, move_timeout)
            positions = {sid: bus.read_uint16(sid, ADDR_PRESENT_POSITION)
                         for sid in ids}
            pos_str = "   ".join(f"{sid}={p:4d}" for sid, p in positions.items())
            print(f"  parked: {pos_str}")

        try:
            bus.sync_enable_torque(ids, on=True)
            mode_label = "antiphase" if args.antiphase else "in phase"
            print(f"\nSwinging ±{args.swing:g}° ({mode_label}). Ctrl+C to fold early.\n")

            swing = 0
            done = False
            # one cycle = A then B; run args.cycles cycles, or forever if 0.
            while not done:
                for targets in (targets_a, targets_b):
                    bus.sync_goal_move(targets)
                    if not bus.sync_wait_until_stopped(ids, move_timeout):
                        print("  (servos slow to report stopped -- continuing)")
                    swing += 1
                    positions = {sid: bus.read_uint16(sid, ADDR_PRESENT_POSITION)
                                 for sid in ids}
                    pos_str = "   ".join(f"{sid}={p:4d}"
                                         for sid, p in positions.items())
                    print(f"  swing {swing:3d}: {pos_str}")
                if args.cycles and swing >= args.cycles * 2:
                    done = True

            go_park()

        except KeyboardInterrupt:
            print("\n\nCtrl+C -- folding to park pose...")
            try:
                go_park()
            except IOError:
                pass
        finally:
            if args.hold:
                print("Holding park pose (torque stays on after exit).")
            else:
                try:
                    bus.sync_enable_torque(ids, on=False)
                    print("Torque released -- leg is limp at the park pose.")
                except IOError:
                    pass

    print("✓ Done.")


if __name__ == "__main__":
    main()
