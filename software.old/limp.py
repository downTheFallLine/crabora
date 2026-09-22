"""
CRABORA: limp -- turn torque OFF on every servo (or just the ones named)
========================================================================

The universal "let go" script: discovers every servo on every URT and
switches Torque Enable off, so the legs can be moved by hand and nothing
is holding current.  Run it after stand.py / stand2.py (which exit still
holding the stance), before hand-posing with teach_pose.py, or any time
the PSU ammeter says something is still working hard.

If the bot is standing it WILL settle/collapse when torque drops, so it
asks first unless you pass --yes.

Usage:
  python limp.py              # all discovered servos
  python limp.py 41 42 43     # just these IDs
  python limp.py --yes        # no confirmation
"""

import argparse
import sys

from crabora_bus import MultiBus, describe_id


def parse_args():
    p = argparse.ArgumentParser(
        description="Turn torque off on every discovered servo (or the "
                    "listed IDs).")
    p.add_argument("servo_ids", type=int, nargs="*",
                   help="specific servo IDs (default: every servo found)")
    p.add_argument("--yes", action="store_true",
                   help="skip the confirmation")
    return p.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("CRABORA: limp -- torque off")
    print("=" * 60)

    with MultiBus() as bus:
        if args.servo_ids:
            ids = sorted(set(args.servo_ids))
            missing = [sid for sid in ids if sid not in bus.live_ids]
            if missing:
                print(f"✗ Not found on any URT: {missing} "
                      f"(live: {bus.live_ids})")
                sys.exit(1)
        else:
            ids = bus.live_ids
            if not ids:
                print("✗ No servos answered on any URT. Bus power on?")
                sys.exit(1)

        print(f"\nGoing limp: {len(ids)} servo(s): {ids}")

        if not args.yes:
            try:
                ans = input("\nIf the bot is standing it will settle/collapse. "
                            "Proceed? [y/N] ")
            except EOFError:
                ans = "n"
            if ans.strip().lower() not in ("y", "yes"):
                print("Aborted -- torque untouched."); return

        bus.sync_enable_torque(ids, on=False)
        print("\n✓ Torque off -- all listed servos are limp.")


if __name__ == "__main__":
    main()
