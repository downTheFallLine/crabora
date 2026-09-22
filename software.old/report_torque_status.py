"""
torque_status.py -- print torque-enable state for every servo on the bus
=========================================================================
Opens a MultiBus (discovers servos across however many FE-URT-2 adapters
are plugged in), reads ADDR_TORQUE_ENABLE on each live servo, and prints
a per-leg report.

Usage:
    python torque_status.py
"""

from crabora_bus import MultiBus, ADDR_TORQUE_ENABLE, describe_id, JOINT_NAMES, joint_of


def main():
    with MultiBus() as mb:
        if not mb.live_ids:
            print("No servos discovered.")
            return

        print()
        print("=" * 60)
        print("Torque status")
        print("=" * 60)

        readings = {}
        for sid in mb.live_ids:
            try:
                readings[sid] = mb.read_uint8(sid, ADDR_TORQUE_ENABLE)
            except IOError as e:
                readings[sid] = e

        legs = mb.legs()
        for leg in sorted(legs):
            print(f"leg {leg}:")
            for sid in legs[leg]:
                joint_name = JOINT_NAMES.get(joint_of(sid), "?")
                val = readings[sid]
                if isinstance(val, Exception):
                    state = f"ERR  ({val})"
                else:
                    state = "ON " if val else "off"
                print(f"  {sid:3d} {joint_name:<6} torque: {state}")

        on_count = sum(1 for v in readings.values() if not isinstance(v, Exception) and v)
        off_count = sum(1 for v in readings.values() if not isinstance(v, Exception) and not v)
        err_count = sum(1 for v in readings.values() if isinstance(v, Exception))

        print()
        print(f"Summary: {on_count} on, {off_count} off, {err_count} unreadable "
              f"(of {len(mb.live_ids)} servos)")


if __name__ == "__main__":
    main()
