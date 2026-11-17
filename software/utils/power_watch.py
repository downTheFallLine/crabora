#!/usr/bin/env python3
"""Watch the servos' current and supply voltage while the robot sits still.

The motion programs' --power-log covers moving. This covers HOLDING:
run stand.py (it leaves torque on), then this, and it reads every servo
round and round for --seconds, printing the total once a second, then
the same summary and CSV as --power-log. Press on the deck while it runs
to see what a load does to the draw.

It does not move anything or change torque.

    python3 utils/power_watch.py                # 10 s
    python3 utils/power_watch.py --seconds 30
"""
import argparse
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "walk"))

import power_log  # noqa: E402
from gait_lib import (  # noqa: E402
    ALL_LEGS, JOINT_NAME, BusError, LAST_CURRENT, PacketHandler,
    find_joints, find_urt_devices, open_ports, read_status, require_bus_free,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--force", action="store_true",
                    help="run even if the agentix service holds the bus")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    if require_bus_free(args.force):
        return 1
    ports = open_ports(args.ports or find_urt_devices())
    if not ports:
        print("No URT could be opened.")
        return 1
    packet = PacketHandler(0)
    power = None
    try:
        servos = []
        for digit in JOINT_NAME:
            for leg, (sid, port) in find_joints(packet, ports, args.legs,
                                                digit, quiet=True).items():
                servos.append((sid, port))
        if not servos:
            print("No servos answered.")
            return 1
        print(f"  watching {len(servos)} servos for {args.seconds:g} s "
              f"(Ctrl-C to stop early)")
        power = power_log.start("hold", True)
        t0 = time.monotonic()
        next_print = t0 + 1.0
        lowest = None
        while time.monotonic() - t0 < args.seconds:
            for sid, port in servos:
                try:
                    v = read_status(packet, port, sid, "power reading")["volts"]
                    lowest = v if lowest is None else min(lowest, v)
                except BusError as e:
                    print(f"  {e}")
            if time.monotonic() >= next_print:
                next_print += 1.0
                total = sum(LAST_CURRENT[s][0] for s, _ in servos
                            if s in LAST_CURRENT)
                low = f"{lowest:.1f} V" if lowest is not None else "--"
                print(f"  {time.monotonic() - t0:5.1f} s   total {total:5.2f} A"
                      f"   lowest supply {low}")
                lowest = None
    except KeyboardInterrupt:
        print()
    finally:
        power_log.finish(power)
        for port in ports.values():
            port.closePort()
    return 0


if __name__ == "__main__":
    sys.exit(main())
