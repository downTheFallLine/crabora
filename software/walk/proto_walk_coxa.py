#!/usr/bin/env python3
"""
CRABORA proto-walk 1: coxa sweep, forward and rear
====================================================

The first locomotion-shaped motion. Coxas only -- no femurs, no tibias,
no weight on the legs. The robot does not move; this is about proving
that "forward" means forward on all six legs before any gait depends on
it.

Sequence, with an operator check at each stop:

  1. Centre every coxa, then PAUSE -- you confirm they really are centred
  2. Sweep every coxa FORWARD together, then PAUSE -- you confirm
  3. Sweep every coxa REAR together, then PAUSE -- you confirm
  4. Return to centre and go limp

Each pause prints the measured position of every joint and waits for a
'y'. Anything else aborts and drops torque, so the safe reflex (Enter,
Ctrl-C, walking away) is the one that stops the robot.

WHY THE SIGNS MATTER
--------------------
Legs 1-3 are the right side, 4-6 the left (see README.md). They are the
same physical parts rotated into place, so a given servo direction
swings them OPPOSITE ways in the body frame.

Deriving it: with leg n at bearing θ = 30° + (n-1)x60° clockwise from
the front point, a counter-clockwise coxa rotation moves that foot
forward by sin θ --

    leg 1 (30°)   sin = +0.50     leg 4 (210°)  sin = -0.50
    leg 2 (90°)   sin = +1.00     leg 5 (270°)  sin = -1.00
    leg 3 (150°)  sin = +0.50     leg 6 (330°)  sin = -0.50

So the right side wants CCW for forward and the left side wants CW:
FORWARD_SIGN below. Note legs 2 and 5 are the beam legs -- their coxa
swing is pure fore/aft, while the corner legs only contribute half as
much per degree.

⚠ FORWARD_SIGN IS UNVERIFIED ON HARDWARE. It assumes every coxa servo
is mounted the same way round, so that a rising count means the same
rotational sense on all six legs. If the legs go the wrong way, do NOT
patch individual legs -- run with --invert, confirm that fixes all six,
and then flip the table here. If only SOME legs go the wrong way, the
servos are not mounted consistently; fix that in hardware or record it
here with a comment, because a half-corrected table will haunt the gait
code forever.

Run --dry-run first. It prints every target without energising anything.

Usage:
  python3 proto_walk_coxa.py --dry-run
  python3 proto_walk_coxa.py --angle 15
  python3 proto_walk_coxa.py --angle 25 --speed 200
  python3 proto_walk_coxa.py --legs 1 2 --invert

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys

from gait_lib import (
    ALL_LEGS, CENTER_POSITION, LEG_NAME, POSITION_TOLERANCE,
    BusError, PacketHandler,
    arm, clamp_targets, confirm, find_coxas, find_urt_devices, forward_sign,
    move_all, open_ports, read_limits, release, report, to_counts, to_degrees,
)


DEFAULT_ANGLE = 20.0      # degrees either side of centre -- deliberately modest
DEFAULT_SPEED = 300       # steps/s -- slower than the sweep tests


def plan_targets(coxas, limits, counts, direction, invert):
    """direction: +1 forward, -1 rear, 0 centre. Clamped to firmware limits."""
    want = {leg: CENTER_POSITION + direction * forward_sign(leg, invert) * counts
            for leg in coxas}
    return clamp_targets(want, limits)


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA proto-walk 1: sweep the coxas forward and rear.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=1,
                    help="forward/rear cycles to run (default 1). More than one "
                         "runs continuously, with no pause between stages.")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmations entirely")
    ap.add_argument("--angle", type=float, default=DEFAULT_ANGLE,
                    help=f"degrees either side of centre (default {DEFAULT_ANGLE:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--invert", action="store_true",
                    help="flip FORWARD_SIGN on all six legs at once")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 < args.angle <= 90:
        ap.error("--angle must be between 0 and 90 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    legs = [l for l in dict.fromkeys(args.legs)]
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    counts = to_counts(args.angle)
    print("CRABORA proto-walk 1: coxa forward/rear sweep")
    print(f"  ±{args.angle:g}° (±{counts} counts) at {args.speed} steps/s"
          f"{'  [INVERTED]' if args.invert else ''}\n")
    print(f"  {'leg':>3} {'name':<12} {'bearing':>8} {'forward is':>12}")
    for leg in legs:
        sign = forward_sign(leg, args.invert)
        bearing = 30 + (leg - 1) * 60
        print(f"  {leg:>3} {LEG_NAME[leg]:<12} {bearing:>7}° "
              f"{('CCW (+)' if sign > 0 else 'CW (-)'):>12}")

    if args.dry_run:
        print(f"\n  Nominal targets (before each servo's firmware limits clamp them):")
        for leg in legs:
            sign = forward_sign(leg, args.invert)
            print(f"    leg {leg}: forward {CENTER_POSITION + sign * counts}, "
                  f"centre {CENTER_POSITION}, rear {CENTER_POSITION - sign * counts}")
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("\nNo FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
        return 1

    print()
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    coxas = {}
    try:
        coxas = find_coxas(packet, ports, legs)
        if not coxas:
            print("\nNo coxa servos answered. Run tests/test_03_ping_servo.py.")
            return 1
        missing = [l for l in legs if l not in coxas]
        if missing:
            print(f"\n  Legs {', '.join(map(str, missing))} did not answer -- "
                  f"continuing with {len(coxas)} of {len(legs)}.")

        # Firmware limits are the real guard: a joint whose travel was set
        # with set_center.py can never be driven past it from here.
        print()
        limits = read_limits(packet, coxas)
        arm(packet, coxas, args.speed)

        trouble = {}   # leg -> {"stalled", "off target"} seen during the run

        def go(label, direction, verbose=True):
            """One move. Returns (results, arrived)."""
            targets, clamped = plan_targets(coxas, limits, counts, direction,
                                            args.invert)
            if verbose:
                print(f"\n  -> {label}")
                for leg, want, got in clamped:
                    print(f"     leg {leg}: {want} clamped to {got} by its firmware "
                          f"limit ({to_degrees(got):+.1f}° not "
                          f"{to_degrees(want):+.1f}°)")
            results = move_all(packet, coxas, targets)
            arrived = True
            if verbose:
                arrived, _bad = report(coxas, targets, results)
                if not arrived:
                    print("\n  Not every joint reached its target -- "
                          "see the flags above.")
            else:
                # Compact path: no per-move table, so fold everything worth
                # knowing into the end-of-run summary instead of losing it.
                for leg, _want, _got in clamped:
                    trouble.setdefault(leg, set()).add("clamped by travel limits")
                for leg, (pos, timed_out) in results.items():
                    if timed_out or abs(pos - targets[leg]) > POSITION_TOLERANCE:
                        arrived = False
                        trouble.setdefault(leg, set()).add(
                            "stalled" if timed_out else "off target")
            return results, arrived

        # --- centre, and the one pause worth keeping ---------------------
        _results, _ok = go("CENTRE", 0)
        if args.pause and not confirm(
                "Are ALL coxas truly centred? Sight along the body -- each leg "
                "should point straight out from its flat."):
            print("\n  Stopped. Returning to centre and going limp.")
            return 1

        # --- N forward/rear cycles ---------------------------------------
        single = args.reps == 1 and args.pause
        if not single:
            print()
        for rep in range(1, args.reps + 1):
            if not single:
                print(f"  rep {rep}/{args.reps}:", end="", flush=True)
            for label, direction, question in (
                    ("FORWARD", +1, "Did every leg swing FORWARD (toward the "
                                    "point between legs 1 and 6)?"),
                    ("REAR",    -1, "Did every leg swing REAR?")):
                _results, arrived = go(label, direction, verbose=single)
                if not single:
                    print(f" {label.lower()}{'' if arrived else ' ✗'}",
                          end="", flush=True)
                elif not confirm(question):
                    print("\n  Stopped. Returning to centre and going limp.")
                    move_all(packet, coxas,
                             {leg: CENTER_POSITION for leg in coxas})
                    return 1
            if not single:
                print()

        print("\n  -> CENTRE (finishing)")
        results = move_all(packet, coxas, {leg: CENTER_POSITION for leg in coxas})
        report(coxas, {leg: CENTER_POSITION for leg in coxas}, results)

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): "
                      f"{', '.join(sorted(trouble[leg]))} during the run")
            print("\n  Check for binding, a leg fouling its neighbour, or the "
                  "supply browning out with all coxas starting together.")
            return 1
        if args.reps > 1:
            print(f"\n  {args.reps} rep(s) completed cleanly on "
                  f"{len(coxas)} coxa(s).")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        # Always leave the robot limp, whatever happened.
        if coxas:
            print("\n  Releasing torque on all coxas.")
            release(packet, coxas)
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
