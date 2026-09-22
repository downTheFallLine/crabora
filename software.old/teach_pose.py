"""
CRABORA: teach_pose -- hand-pose the bot, record servo positions to poses.json
==============================================================================

The legs' joint zeros aren't characterized, so rather than guess servo angles
for TUCK / STAND, you physically move the bot into each pose (servos limp) and
this records where every servo actually is.  stand.py then plays them back.

No fixed leg roster: MultiBus opens every FE-URT-2 it can see and discovers
which servos answer on which bus, so this works the same at 3 legs on two
URTs or 8 legs on three.  Whatever answers gets captured.

How it works:
  1. Opens all URTs, discovers the live servos, turns TORQUE OFF (so you can
     move the legs by hand).  Warns if a leg is missing a joint.
  2. For each pose name, you pose the bot, hold it, press Enter -> it reads the
     present position of every discovered servo and stores it.
  3. Writes/updates poses.json next to this script.

Recommended poses to capture (the stand sequence uses 'tuck' and 'stand'):
  tuck   -- bot belly-down, each foot pulled in UNDER its hip (pre-stand)
  stand  -- bot held up in a stable, near-vertical stance (low holding torque)

Usage:
  python teach_pose.py                 # capture 'tuck' then 'stand'
  python teach_pose.py tuck            # re-capture just 'tuck'
  python teach_pose.py tuck stand splayed
"""

import json
import os
import sys

from crabora_bus import (
    MultiBus, ADDR_PRESENT_POSITION,
    COXA, FEMUR, TIBIA, JOINT_NAMES, joint_of,
)

POSES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")


def report_legs(legs):
    """Print the discovered legs; warn about any that are missing a joint."""
    for leg_num, ids in sorted(legs.items()):
        joints = {joint_of(sid) for sid in ids}
        missing = [JOINT_NAMES[j] for j in (COXA, FEMUR, TIBIA)
                   if j not in joints]
        line = f"  leg {leg_num}: {ids}"
        if missing:
            line += f"   ⚠  missing {'/'.join(missing)} -- power/cable?"
        print(line)


def main():
    names = sys.argv[1:] or ["tuck", "stand"]

    poses = {}
    if os.path.exists(POSES_PATH):
        with open(POSES_PATH) as f:
            poses = json.load(f)

    print("=" * 60)
    print("CRABORA: teach poses")
    print("=" * 60)
    print(f"  capturing: {names}")
    print()

    with MultiBus() as bus:
        ids = bus.live_ids
        if not ids:
            print("\n✗ No servos answered on any URT. Bus power on? "
                  "Cables seated?")
            sys.exit(1)

        legs = bus.legs()
        print(f"\n✓ {len(ids)} servo(s) on {len(legs)} leg(s):")
        report_legs(legs)

        bus.sync_enable_torque(ids, on=False)
        print("\nTORQUE OFF — you can move the legs by hand.\n")

        for name in names:
            try:
                input(f"Pose the bot into '{name}', hold it steady, press Enter "
                      f"to capture (Ctrl+C to abort)... ")
            except (EOFError, KeyboardInterrupt):
                print("\nAborted; nothing else captured.")
                break
            snap = {}
            for sid in ids:
                snap[str(sid)] = bus.read_uint16(sid, ADDR_PRESENT_POSITION)
            poses[name] = snap
            shown = "  ".join(f"{sid}={snap[str(sid)]:4d}" for sid in ids)
            print(f"  ✓ '{name}': {shown}\n")

        with open(POSES_PATH, "w") as f:
            json.dump(poses, f, indent=2)
        print(f"Saved poses {list(poses)} -> {POSES_PATH}")
        print("Servos left limp.  Run stand.py to play the stand sequence.")


if __name__ == "__main__":
    main()
