"""
exercise_legs.py -- sweep every servo through its full range of motion
=========================================================================
Two passes:
  1. INDIVIDUAL -- one servo at a time: center -> min -> max -> center.
     Good for visually confirming each joint moves freely and the
     firmware limits are sane before doing anything in unison.
  2. GROUP -- all servos together, same sweep (center -> min -> max ->
     center), via sync_goal_move so every joint moves in the same packet.
     This is closer to what a real gait asks of the bus, and will surface
     mechanical interference between legs that the individual pass can't.

Range per servo comes from its firmware angle limits (ADDR_MIN_ANGLE /
ADDR_MAX_ANGLE, registers 9-11). If a servo reports 0/0 (common in wheel
mode, or if limits were never set), this script falls back to the full
mechanical range (MIN_POSITION..MAX_POSITION) and prints a warning --
double check that's actually safe for that joint before proceeding.

Because a full sweep can drive a leg into a neighbor or the frame, this
prompts for confirmation before moving anything unless --yes is passed.

Usage:
    python exercise_legs.py [options]

Options:
    --rpm N            speed for all moves (default 8, deliberately slow)
    --pause N           seconds to pause between moves, for eyeballing (default 0.5)
    --timeout N         seconds to wait for arrival per move (default 5.0)
    --ids 11,12,13      restrict to these servo IDs (comma-separated); default = all discovered
    --skip-individual   go straight to the group pass
    --skip-group        stop after the individual pass
    --torque-off        release torque on all exercised servos at the end (default: leave on)
    --yes               skip the confirmation prompt
"""

import sys
import time

from crabora_bus import (
    MultiBus,
    CENTER_POSITION,
    MIN_POSITION,
    MAX_POSITION,
    ADDR_MIN_ANGLE,
    ADDR_MAX_ANGLE,
    describe_id,
    JOINT_NAMES,
    joint_of,
    leg_of,
    rpm_to_pos_speed,
)


def parse_args(argv):
    opts = {
        "rpm": 8.0,
        "pause": 0.5,
        "timeout": 5.0,
        "ids": None,
        "skip_individual": False,
        "skip_group": False,
        "torque_off": False,
        "yes": False,
    }
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--rpm":
            i += 1; opts["rpm"] = float(argv[i])
        elif a == "--pause":
            i += 1; opts["pause"] = float(argv[i])
        elif a == "--timeout":
            i += 1; opts["timeout"] = float(argv[i])
        elif a == "--ids":
            i += 1; opts["ids"] = [int(x) for x in argv[i].split(",")]
        elif a == "--skip-individual":
            opts["skip_individual"] = True
        elif a == "--skip-group":
            opts["skip_group"] = True
        elif a == "--torque-off":
            opts["torque_off"] = True
        elif a == "--yes":
            opts["yes"] = True
        else:
            print(f"Unknown option: {a}")
            sys.exit(1)
        i += 1
    return opts


def get_range(mb, sid):
    """Return (min_pos, max_pos) for a servo: firmware limits, or full
    mechanical range with a warning if the firmware limits are 0/0."""
    min_a = mb.read_uint16(sid, ADDR_MIN_ANGLE)
    max_a = mb.read_uint16(sid, ADDR_MAX_ANGLE)
    if min_a == 0 and max_a == 0:
        print(f"  ! {sid} ({describe_id(sid)}): firmware limits read 0/0 "
              f"(wheel mode, or limits unset). Falling back to full range "
              f"{MIN_POSITION}..{MAX_POSITION} -- verify this is safe for this joint.")
        return MIN_POSITION, MAX_POSITION
    return min_a, max_a


def move_and_wait(mb, sid, position, speed, timeout, label):
    mb.write_goal_move(sid, position, speed)
    ok = mb.wait_until_stopped(sid, timeout)
    tag = "ok" if ok else "TIMED OUT"
    print(f"    {label}: {position:4d}  [{tag}]")
    return ok


def sync_move_and_wait(mb, targets, ids, timeout, label):
    mb.sync_goal_move(targets)
    ok = mb.sync_wait_until_stopped(ids, timeout)
    tag = "ok" if ok else "TIMED OUT (one or more servos)"
    print(f"  {label}  [{tag}]")
    return ok


def main():
    opts = parse_args(sys.argv[1:])

    with MultiBus() as mb:
        ids = opts["ids"] if opts["ids"] is not None else mb.live_ids
        missing = [i for i in ids if i not in mb.live_ids]
        if missing:
            print(f"Not found on bus: {missing}. Live IDs: {mb.live_ids}")
            sys.exit(1)
        if not ids:
            print("No servos to exercise.")
            return

        # Order by leg, then joint, for readable output.
        ids = sorted(ids, key=lambda s: (leg_of(s), joint_of(s)))

        print(f"Servos to exercise: {ids}")
        print("Reading firmware angle limits...")
        ranges = {}
        for sid in ids:
            ranges[sid] = get_range(mb, sid)
            lo, hi = ranges[sid]
            print(f"  {sid:3d} {describe_id(sid):<16} range: {lo}..{hi}")

        if not opts["yes"]:
            resp = input(
                "\nThis will drive the above servos through their full range, "
                "one at a time and then all together. Clear the area. Proceed? [y/N] "
            )
            if resp.strip().lower() not in ("y", "yes"):
                print("Aborted.")
                return

        speed = rpm_to_pos_speed(opts["rpm"])
        print(f"\nSpeed: {opts['rpm']} rpm  |  pause: {opts['pause']}s  |  "
              f"per-move timeout: {opts['timeout']}s")

        print("\nEnabling torque on all exercised servos...")
        mb.sync_enable_torque(ids, True)
        time.sleep(0.2)

        # ------------------------------------------------------------------
        # Pass 1: one servo at a time
        # ------------------------------------------------------------------
        if not opts["skip_individual"]:
            print("\n" + "=" * 60)
            print("PASS 1: individual servo sweep (center -> min -> max -> center)")
            print("=" * 60)
            for sid in ids:
                lo, hi = ranges[sid]
                print(f"\n{sid} ({describe_id(sid)}):")
                move_and_wait(mb, sid, CENTER_POSITION, speed, opts["timeout"], "center")
                time.sleep(opts["pause"])
                move_and_wait(mb, sid, lo, speed, opts["timeout"], "min   ")
                time.sleep(opts["pause"])
                move_and_wait(mb, sid, hi, speed, opts["timeout"], "max   ")
                time.sleep(opts["pause"])
                move_and_wait(mb, sid, CENTER_POSITION, speed, opts["timeout"], "center")
                time.sleep(opts["pause"])
        else:
            print("\nSkipping individual pass (--skip-individual).")

        # ------------------------------------------------------------------
        # Pass 2: all servos together
        # ------------------------------------------------------------------
        if not opts["skip_group"]:
            print("\n" + "=" * 60)
            print("PASS 2: group sweep, all servos together (center -> min -> max -> center)")
            print("=" * 60)

            center_targets = {sid: (CENTER_POSITION, speed) for sid in ids}
            min_targets    = {sid: (ranges[sid][0], speed) for sid in ids}
            max_targets    = {sid: (ranges[sid][1], speed) for sid in ids}

            sync_move_and_wait(mb, center_targets, ids, opts["timeout"], "all -> center")
            time.sleep(opts["pause"])
            sync_move_and_wait(mb, min_targets, ids, opts["timeout"], "all -> min   ")
            time.sleep(opts["pause"])
            sync_move_and_wait(mb, max_targets, ids, opts["timeout"], "all -> max   ")
            time.sleep(opts["pause"])
            sync_move_and_wait(mb, center_targets, ids, opts["timeout"], "all -> center")
        else:
            print("\nSkipping group pass (--skip-group).")

        if opts["torque_off"]:
            print("\nReleasing torque on all exercised servos...")
            mb.sync_enable_torque(ids, False)
        else:
            print("\nTorque left on.")

        print("\nDone.")


if __name__ == "__main__":
    main()
