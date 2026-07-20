#!/usr/bin/env python3
"""
urt_scan -- "can I see all my FE-URT-2 boards?" check for CRABORA
================================================================

You have three FE-URT-2 USB adapters plugged in -- one straight into the
Mac's USB-C, two through a hub. This script answers one question: does the
Mac actually enumerate all three, and if not, which one is missing?

How it identifies a URT
-----------------------
The FE-URT-2 presents as a WCH CH340-family USB serial device:

    VID:PID = 1A86:55D3   ("USB Single Serial")

So instead of guessing from /dev/tty.usb* globs (which also catch phones,
debug consoles, etc.), we ask pyserial for every serial port and keep the
ones whose USB vendor is WCH (0x1A86). Each real board has a unique USB
serial number, so we can tell them apart even though they're identical
hardware.

Direct vs. hub
--------------
macOS location IDs look like "1-1" (a root-hub port -- your direct USB-C
cable) or "2-1.2" (a dotted suffix -- downstream of an external hub, i.e.
your dongle). We print that so you can see at a glance which cable is which.

Usage
-----
    python urt_scan.py              # list boards, compare against 3 expected
    python urt_scan.py -n 2         # expect 2 instead of 3
    python urt_scan.py --ping       # also open each board at 1 Mbaud and
                                    #   scan servo IDs 1..30 to prove the
                                    #   Feetech bus behind it is alive
                                    #   (needs the servos powered)

Exit status is 0 when the expected number of boards is present, 1 otherwise,
so it can be dropped into a startup check.
"""

import argparse
import sys

from serial.tools import list_ports

from uart_lib import connection, is_urt, list_urt_ports

EXPECTED_DEFAULT = 3


def find_urts():
    """Return (urts, others): pyserial ListPortInfo lists, URTs sorted stably."""
    urts = list_urt_ports()
    others = [p for p in list_ports.comports() if not is_urt(p)]
    return urts, others


def ping_bus(device):
    """Open one URT and scan servo IDs 1..30. Returns a list of live IDs.

    Imported lazily so the plain listing works even if crabora_bus or the
    servo wiring isn't ready.
    """
    from crabora_bus import Bus  # local import: only needed for --ping

    live = []
    bus = Bus(port=device)
    bus.open()
    try:
        for sid in range(1, 31):
            if bus.ping(sid):
                live.append(sid)
    finally:
        bus.close()
    return live


def main():
    ap = argparse.ArgumentParser(description="Detect FE-URT-2 boards over USB.")
    ap.add_argument("-n", "--expected", type=int, default=EXPECTED_DEFAULT,
                    help=f"how many boards you expect (default {EXPECTED_DEFAULT})")
    ap.add_argument("--ping", action="store_true",
                    help="also open each board and scan for live servos "
                         "(servos must be powered)")
    args = ap.parse_args()

    urts, others = find_urts()

    print(f"Found {len(urts)} FE-URT-2 board(s) "
          f"(expected {args.expected}):\n")

    if urts:
        for i, p in enumerate(urts, 1):
            serial = p.serial_number or "?"
            loc = p.location or "?"
            print(f"  [{i}] {p.device}")
            print(f"      serial : {serial}")
            print(f"      usb    : {loc}  ({connection(p.location)})")
            if args.ping:
                try:
                    live = ping_bus(p.device)
                    if live:
                        print(f"      bus    : {len(live)} servo(s) answering "
                              f"-> IDs {live}")
                    else:
                        print("      bus    : opened OK, but NO servos answered "
                              "(powered? correct baud?)")
                except Exception as e:  # noqa: BLE001 - report, don't crash the scan
                    print(f"      bus    : could not talk to bus -- {e}")
            print()
    else:
        print("  (none)\n")

    if others:
        print("Other serial devices seen (not URTs):")
        for p in others:
            vidpid = (f"{p.vid:04X}:{p.pid:04X}"
                      if p.vid is not None and p.pid is not None else "----:----")
            print(f"  - {p.device}  [{vidpid}]  {p.description}")
        print()

    missing = args.expected - len(urts)
    if missing > 0:
        print(f"** {missing} board(s) missing. Things to check:")
        print("   - The board that WON'T appear: reseat its cable; try it")
        print("     directly in the Mac to rule out a bad port on the hub.")
        print("   - Bus-powered hub may not supply enough current -- try a")
        print("     powered hub, or move a board to a direct port.")
        print("   - Swap the suspect cable (some USB-C cables are charge-only).")
        print("   - If NOTHING shows, install the WCH CH340 macOS driver")
        print("     from wch-ic.com and re-plug.")
        return 1

    if len(urts) > args.expected:
        print(f"Note: found {len(urts)} boards, more than the {args.expected} "
              "expected. Extra URT plugged in?")
        return 0

    print("All expected boards present.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
