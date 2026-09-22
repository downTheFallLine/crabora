"""
CRABORA: stand2 -- geometric stand, two tripods at a time
=========================================================

The simple, no-poses-file stand: every joint goes to a computed target
instead of a hand-taught one.

  coxa  (x1)  ->  center            (2048)
  femur (x2)  ->  +45 deg off center (2560)
  tibia (x3)  ->  +90 deg off center (3072)

Sign convention verified against the hand-taught 'stand' pose (poses.json):
positive counts = joint extends to stand.  The first cut used -45/-90 and
stood the bot "on its back".

Legs are discovered at runtime across every URT (see MultiBus), then moved
as two alternating tripods -- odd-numbered legs first, then even-numbered --
so only half the servos draw current at once (5 A bench-supply budget).
Torque comes on for ALL servos before anything moves, so the waiting tripod
holds its pose instead of sagging.

Every discovered leg must be complete (coxa/femur/tibia); targets are
clamped to each servo's firmware angle limits, with a warning when that
happens.  Ends HOLDING the stance (torque on); pass --release to relax.

Requirements (every servo): position mode (wheel_off.py), angle limits
(set_limits.py), center calibrated (set_middle.py) -- the targets are
absolute counts around 2048, so an uncalibrated center stands crooked.

Usage:
  python stand2.py                # both tripods, pausing between them
  python stand2.py --speed 4      # slower
  python stand2.py --no-pause     # don't wait for Enter between tripods
  python stand2.py --release      # relax after standing (default: hold)
"""

import argparse
import sys

from crabora_bus import (
    MultiBus,
    ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, ADDR_MODE, ADDR_PRESENT_POSITION,
    MODE_POSITION, MODE_NAMES,
    COUNTS_PER_REV, DEGREES_PER_REV, CENTER_POSITION, MAX_POSITION,
    TYPICAL_MAX_RPM,
    COXA, FEMUR, TIBIA, JOINT_NAMES, joint_of,
    TRIPOD_A, TRIPOD_B,
    describe_id, rpm_to_pos_speed,
)

# -----------------------------------------------------------------------------
# Joint targets, in degrees off center (positive = higher counts = extend
# to stand; matches the hand-taught 'stand' pose, which averages femur +34
# tibia +42 across all six legs).
JOINT_TARGET_DEG = {COXA: 0.0, FEMUR: 45.0, TIBIA: 90.0}

SPEED_RPM = 6.0    # slow: this is the load-bearing move


def deg_to_counts(deg):
    """Absolute position counts for an angle in degrees off center."""
    return CENTER_POSITION + round(deg / DEGREES_PER_REV * COUNTS_PER_REV)


def parse_args():
    p = argparse.ArgumentParser(
        description="Geometric stand: coxa center, femur -45 deg, tibia "
                    "-90 deg, moved as two alternating tripods.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speed", type=float, default=SPEED_RPM,
                   help=f"move speed in RPM (default {SPEED_RPM:g})")
    p.add_argument("--no-pause", action="store_true",
                   help="don't wait for Enter between tripods")
    p.add_argument("--release", action="store_true",
                   help="relax after standing (default: hold)")
    p.add_argument("--yes", action="store_true",
                   help="skip the initial confirmation")
    return p.parse_args()


def preflight(bus, servo_id):
    """Position-mode + angle-limit check. Exits on failure -> (min, max)."""
    label = describe_id(servo_id)
    mode = bus.read_uint8(servo_id, ADDR_MODE)
    if mode != MODE_POSITION:
        print(f"✗ {label} (ID {servo_id}) in {MODE_NAMES.get(mode, mode)} mode; "
              f"needs position mode.  Run: python wheel_off.py {servo_id}")
        sys.exit(1)
    mn = bus.read_uint16(servo_id, ADDR_MIN_ANGLE)
    mx = bus.read_uint16(servo_id, ADDR_MAX_ANGLE)
    if mn >= mx or (mn == 0 and mx >= MAX_POSITION):
        print(f"✗ {label} (ID {servo_id}) has no usable angle limits ({mn}..{mx}). "
              f"Run: python set_limits.py {servo_id}")
        sys.exit(1)
    return mn, mx


def clamp(servo_id, counts, limits):
    mn, mx = limits[servo_id]
    c = max(mn, min(mx, counts))
    if c != counts:
        print(f"  ⚠  {describe_id(servo_id)} target {counts} clamped to {c} "
              f"({mn}..{mx})")
    return c


def maybe_pause(msg, do_pause):
    if do_pause:
        try:
            input(f"\n{msg}  [Enter to continue, Ctrl+C to abort] ")
        except EOFError:
            pass
    else:
        print(f"\n{msg}")


def main():
    args = parse_args()
    if args.speed <= 0:
        print("✗ Speed must be positive (RPM)."); sys.exit(1)
    if args.speed > TYPICAL_MAX_RPM:
        print(f"⚠  {args.speed:.1f} rpm exceeds the STS3215 typical max "
              f"(~{TYPICAL_MAX_RPM}).")
    raw_speed = rpm_to_pos_speed(args.speed)

    print("=" * 62)
    print("CRABORA: stand2  (geometric, two tripods)")
    print("=" * 62)
    print("  targets: " + "  ".join(
        f"{JOINT_NAMES[j]} {JOINT_TARGET_DEG[j]:+g}° ({deg_to_counts(JOINT_TARGET_DEG[j])})"
        for j in (COXA, FEMUR, TIBIA)))
    print(f"  speed  : {args.speed:g} rpm")
    print()

    with MultiBus() as bus:
        legs = bus.legs()
        if not legs:
            print("✗ No servos answered on any URT. Bus power on? Cables seated?")
            sys.exit(1)

        # Refuse to stand on an incomplete leg.
        for leg_num, ids in sorted(legs.items()):
            joints = {joint_of(sid) for sid in ids}
            missing = [JOINT_NAMES[j] for j in (COXA, FEMUR, TIBIA)
                       if j not in joints]
            if missing:
                print(f"✗ leg {leg_num} is missing its {'/'.join(missing)} "
                      f"(only {ids} answered). Fix power/cabling, or unplug "
                      f"the whole leg to stand without it.")
                sys.exit(1)

        all_ids = bus.live_ids

        print()
        limits = {sid: preflight(bus, sid) for sid in all_ids}
        targets = {
            sid: (clamp(sid, deg_to_counts(JOINT_TARGET_DEG[joint_of(sid)]),
                        limits),
                  raw_speed)
            for sid in all_ids
        }

        # Physical alternating tripods (every other leg around the body).
        unmapped = sorted(n for n in legs if n not in TRIPOD_A + TRIPOD_B)
        tripods = [
            ("tripod A", [n for n in TRIPOD_A if n in legs]),
            ("tripod B", [n for n in TRIPOD_B if n in legs]),
            ("unmapped legs (not in LEG_LAYOUT_CW)", unmapped),
        ]
        tripods = [(name, nums) for name, nums in tripods if nums]

        widest = max(mx - mn for mn, mx in limits.values())
        move_timeout = widest / raw_speed * 2.0 + 1.0

        for name, nums in tripods:
            print(f"  {name}: legs {nums}")

        if not args.yes:
            try:
                ans = input(f"\nProceed? {len(legs)} leg(s) / {len(all_ids)} "
                            "servos. PSU at 7.4 V / ~5 A limit. [y/N] ")
            except EOFError:
                ans = "n"
            if ans.strip().lower() not in ("y", "yes"):
                print("Aborted."); return

        def report(ids):
            pos = {sid: bus.read_uint16(sid, ADDR_PRESENT_POSITION)
                   for sid in ids}
            print("    " + "  ".join(f"{sid}={p:4d}" for sid, p in pos.items()))

        try:
            # Torque on everywhere first: the tripod that isn't moving yet
            # holds its pose instead of sagging under the shifting body.
            bus.sync_enable_torque(all_ids, on=True)

            for name, nums in tripods:
                maybe_pause(f"About to move {name} -- watch the ammeter.",
                            not args.no_pause)
                ids = [sid for n in nums for sid in legs[n]]
                bus.sync_goal_move({sid: targets[sid] for sid in ids})
                if not bus.sync_wait_until_stopped(ids, move_timeout):
                    print("  (servos slow to report stopped -- continuing)")
                report(ids)

            print("\n✓ Standing.")

        except KeyboardInterrupt:
            print("\n\nCtrl+C -- holding current position (servos stay driven).")
            return
        finally:
            if args.release:
                try:
                    bus.sync_enable_torque(all_ids, on=False)
                    print("Torque released -- bot will settle.")
                except IOError:
                    pass
            else:
                print("Holding stance (torque stays on after exit).")


if __name__ == "__main__":
    main()
