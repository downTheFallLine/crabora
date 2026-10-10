#!/usr/bin/env python3
"""
CRABORA tripod gait -- coxas only
===================================

The alternating tripod, reduced to the one joint that produces fore/aft
travel. Tripod A (legs 1, 3, 5) and tripod B (legs 2, 4, 6) swing in
opposite directions, swapping each half-stride:

    half-stride 1:   A swings FORWARD   |   B drives REAR
    half-stride 2:   A drives REAR      |   B swings FORWARD

One --reps is one full stride, i.e. both half-strides.

⚠ RUN THIS ON THE TEST STAND, WITH THE LEGS HANGING FREE.
A real tripod gait lifts the swing legs clear while the stance legs
push. With only the coxas there is no lift, so on the ground all six
feet stay at the same height and the "swing" legs would scrape backwards
against the floor, cancelling the stance legs and grinding the feet.
Hanging in mid air that does not arise -- and the legs are unloaded, so
current draw is well under the servos' rated figure. The femurs and
tibias are what turn this into locomotion, and they come later.

What this DOES prove, which is worth proving before adding two more
joints per leg:
  - the forward/rear sign is right on all six legs (see gait_lib)
  - the two tripods are correctly grouped and genuinely opposed
  - six coxas can reverse direction repeatedly without stalling, and
    the supply can take the current of two opposed tripods starting
    together -- the worst electrical case in the whole gait

Sequence: centre everything, pause for you to confirm, move to the
starting pose, then run --reps full strides, then recentre and go limp.

Usage:
  python3 tripod_coxa.py --dry-run
  python3 tripod_coxa.py --reps 10
  python3 tripod_coxa.py --reps 50 --angle 15 --speed 200 --no-pause
  python3 tripod_coxa.py --reps 4 --reverse        # walk backwards
  python3 tripod_coxa.py --legs 1 2                # partial robot

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import sys
import time

from gait_lib import (
    ALL_LEGS, CENTER_POSITION, LEG_BEARING, LEG_NAME, TRIPOD_A, TRIPOD_B,
    BusError, PacketHandler,
    arm, check, clamp_targets, confirm, find_coxas, find_urt_devices,
    forward_sign, move_all, open_ports, read_limits, release, report,
    to_counts, to_degrees,
)

DEFAULT_ANGLE = 20.0      # degrees either side of centre = half the stride
DEFAULT_SPEED = 300       # steps/s
DEFAULT_REPS = 4
DEFAULT_DWELL = 0.0       # seconds held at each half-stride


def tripod_of(leg):
    return "A" if leg in TRIPOD_A else "B"


def stride_targets(coxas, counts, a_forward, invert, reverse):
    """Targets for one half-stride.

    a_forward: True  -> tripod A forward, tripod B rear
               False -> tripod A rear,    tripod B forward
    reverse flips the whole gait, so the robot would walk backwards.
    """
    targets = {}
    for leg in coxas:
        a_side = leg in TRIPOD_A
        go_forward = a_forward if a_side else not a_forward
        if reverse:
            go_forward = not go_forward
        direction = +1 if go_forward else -1
        targets[leg] = CENTER_POSITION + direction * forward_sign(leg, invert) * counts
    return targets


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA tripod gait, coxas only.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=DEFAULT_REPS,
                    help=f"full strides to run (default {DEFAULT_REPS}); "
                         f"one stride = both half-strides")
    ap.add_argument("--angle", type=float, default=DEFAULT_ANGLE,
                    help=f"degrees either side of centre, i.e. half the stride "
                         f"(default {DEFAULT_ANGLE:g})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                    help=f"move speed in steps/s (default {DEFAULT_SPEED})")
    ap.add_argument("--dwell", type=float, default=DEFAULT_DWELL,
                    help=f"seconds to hold at each half-stride "
                         f"(default {DEFAULT_DWELL:g})")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS),
                    help="legs to include (default: all six, whichever answer)")
    ap.add_argument("--reverse", action="store_true",
                    help="run the gait backwards")
    ap.add_argument("--invert", action="store_true",
                    help="flip the forward sign on all six legs at once")
    ap.add_argument("--no-pause", dest="pause", action="store_false",
                    help="skip the operator confirmation before moving")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the stride plan and exit without touching the bus")
    ap.add_argument("--ports", nargs="+",
                    help="serial ports (default: auto-discover FE-URT-2 boards)")
    args = ap.parse_args()

    if args.reps < 1:
        ap.error("--reps must be at least 1")
    if not 0 < args.angle <= 90:
        ap.error("--angle must be between 0 and 90 degrees")
    if not 1 <= args.speed <= 3400:
        ap.error("--speed must be 1-3400 steps/s")
    if args.dwell < 0:
        ap.error("--dwell cannot be negative")
    legs = list(dict.fromkeys(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("--legs must be 1-6")

    counts = to_counts(args.angle)
    print("CRABORA tripod gait -- coxas only")
    print(f"  {args.reps} stride(s), ±{args.angle:g}° (±{counts} counts) "
          f"at {args.speed} steps/s"
          f"{'  [REVERSE]' if args.reverse else ''}"
          f"{'  [INVERTED]' if args.invert else ''}")
    print("  ⚠ no lift without the femurs -- test stand, legs hanging free\n")

    in_a = [l for l in legs if l in TRIPOD_A]
    in_b = [l for l in legs if l in TRIPOD_B]
    print(f"  tripod A: {', '.join(f'{l} ({LEG_NAME[l]})' for l in in_a) or '-'}")
    print(f"  tripod B: {', '.join(f'{l} ({LEG_NAME[l]})' for l in in_b) or '-'}")
    if not in_a or not in_b:
        print("\n  ⚠ one tripod is empty -- this will swing a single group back "
              "and forth, not alternate. Fine for a partial robot, but it is "
              "not a gait.")

    if args.dry_run:
        t1 = stride_targets({l: None for l in legs}, counts, True,
                            args.invert, args.reverse)
        t2 = stride_targets({l: None for l in legs}, counts, False,
                            args.invert, args.reverse)
        print(f"\n  {'leg':>3} {'name':<12} {'bearing':>8} {'tripod':>7} "
              f"{'half 1':>16} {'half 2':>16}")
        for leg in legs:
            # The raw counts differ between sides -- that is the mirror doing
            # its job. The body-frame words are what must look right.
            w1 = "forward" if t1[leg] > CENTER_POSITION else "rear"
            w2 = "forward" if t2[leg] > CENTER_POSITION else "rear"
            if forward_sign(leg, args.invert) < 0:
                w1, w2 = ("rear" if w1 == "forward" else "forward",
                          "rear" if w2 == "forward" else "forward")
            print(f"  {leg:>3} {LEG_NAME[leg]:<12} {LEG_BEARING[leg]:>7}° "
                  f"{tripod_of(leg):>7} {t1[leg]:>7} {w1:>8} "
                  f"{t2[leg]:>7} {w2:>8}")
        print("\n  (targets before each servo's firmware limits clamp them;")
        print("   counts differ between sides because the legs are mirrored,")
        print("   so judge it by the forward/rear words, not the numbers)")
        print("  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found. Run tests/test_02_urt_usb_visibility.py.")
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

        print()
        limits = read_limits(packet, coxas)
        arm(packet, coxas, args.speed)

        # --- centre, and confirm before anything cyclic starts -----------
        print("\n  -> CENTRE")
        centre = {leg: CENTER_POSITION for leg in coxas}
        report(coxas, centre, move_all(packet, coxas, centre))
        if args.pause and not confirm(
                "On the test stand with legs hanging free, and all coxas centred? "
                "The next move starts the gait."):
            print("\n  Stopped. Going limp.")
            return 1

        # --- the gait ----------------------------------------------------
        trouble = {}
        print()
        for rep in range(1, args.reps + 1):
            print(f"  stride {rep}/{args.reps}:", end="", flush=True)
            for half, a_forward in ((1, True), (2, False)):
                targets = stride_targets(coxas, counts, a_forward,
                                         args.invert, args.reverse)
                targets, clamped = clamp_targets(targets, limits)
                results = move_all(packet, coxas, targets)
                ok, bad = check(targets, results)
                for leg, why in bad.items():
                    trouble.setdefault(leg, set()).add(why)
                label = "A-fwd/B-rear" if a_forward else "A-rear/B-fwd"
                print(f"  {label}{'' if ok else ' ✗'}", end="", flush=True)
                if rep == 1 and half == 1 and clamped:
                    for leg, want, got in clamped:
                        trouble.setdefault(leg, set()).add("clamped by limits")
                if args.dwell:
                    time.sleep(args.dwell)
            print()

        print("\n  -> CENTRE (finishing)")
        report(coxas, centre, move_all(packet, coxas, centre))

        if trouble:
            print()
            for leg in sorted(trouble):
                print(f"  ⚠ leg {leg} ({LEG_NAME[leg]}): "
                      f"{', '.join(sorted(trouble[leg]))}")
            print("\n  Check for binding, a leg fouling its neighbour, travel "
                  "limits narrower than --angle, or the supply browning out "
                  "with two opposed tripods starting together.")
            return 1

        print(f"\n  {args.reps} stride(s) completed cleanly on "
              f"{len(coxas)} coxa(s).")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\n{e}")
        return 1
    finally:
        if coxas:
            print("\n  Releasing torque on all coxas.")
            release(packet, coxas)
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
