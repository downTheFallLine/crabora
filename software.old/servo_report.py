"""
torque_status.py -- print torque, position, and angle-limit state for every
servo on the bus
=========================================================================
Opens a MultiBus (discovers servos across however many FE-URT-2 adapters
are plugged in). For each live servo, reads:
  - ADDR_TORQUE_ENABLE    -- on/off
  - ADDR_PRESENT_POSITION -- current position, and offset from CENTER_POSITION
  - ADDR_MIN_ANGLE / ADDR_MAX_ANGLE -- firmware angle limits (EEPROM)
and prints a per-leg report.

Usage:
    python torque_status.py
"""

from crabora_bus import (
    MultiBus,
    ADDR_TORQUE_ENABLE,
    ADDR_PRESENT_POSITION,
    ADDR_MIN_ANGLE,
    ADDR_MAX_ANGLE,
    CENTER_POSITION,
    describe_id,
    JOINT_NAMES,
    joint_of,
)


def main():
    with MultiBus() as mb:
        if not mb.live_ids:
            print("No servos discovered.")
            return

        print()
        print("=" * 60)
        print("Servo status: torque / position / angle limits")
        print("=" * 60)

        readings = {}
        for sid in mb.live_ids:
            try:
                torque   = mb.read_uint8(sid, ADDR_TORQUE_ENABLE)
                position = mb.read_uint16(sid, ADDR_PRESENT_POSITION)
                min_ang  = mb.read_uint16(sid, ADDR_MIN_ANGLE)
                max_ang  = mb.read_uint16(sid, ADDR_MAX_ANGLE)
                readings[sid] = {
                    "torque": torque,
                    "position": position,
                    "min_angle": min_ang,
                    "max_angle": max_ang,
                }
            except IOError as e:
                readings[sid] = e

        legs = mb.legs()
        for leg in sorted(legs):
            print(f"leg {leg}:")
            for sid in legs[leg]:
                joint_name = JOINT_NAMES.get(joint_of(sid), "?")
                val = readings[sid]
                if isinstance(val, Exception):
                    print(f"  {sid:3d} {joint_name:<6} ERR  ({val})")
                    continue
                state = "ON " if val["torque"] else "off"
                offset = val["position"] - CENTER_POSITION
                offset_str = f"{offset:+d}"
                # min/max of 0/0 means the firmware limit isn't set (wheel mode etc.)
                limits = f"{val['min_angle']}..{val['max_angle']}"
                print(
                    f"  {sid:3d} {joint_name:<6} torque: {state}  "
                    f"pos: {val['position']:4d} (center {offset_str})  "
                    f"limits: {limits}"
                )

        on_count  = sum(1 for v in readings.values() if not isinstance(v, Exception) and v["torque"])
        off_count = sum(1 for v in readings.values() if not isinstance(v, Exception) and not v["torque"])
        err_count = sum(1 for v in readings.values() if isinstance(v, Exception))

        print()
        print(f"Summary: {on_count} on, {off_count} off, {err_count} unreadable "
              f"(of {len(mb.live_ids)} servos)")


if __name__ == "__main__":
    main()
