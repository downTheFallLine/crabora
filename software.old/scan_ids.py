#!/usr/bin/env python3
"""
scan_ids -- "what servo IDs are alive?" query for every URT on CRABORA
======================================================================

Opens each FE-URT-2 the Mac can see and pings every servo ID on the bus
behind it, then prints a per-URT roster using the CRABORA (leg, joint)
naming scheme. This is the tool to run after re-plugging cables, changing
an ID with set_id.py, or adding a leg, to confirm which servos answer
where.

How it differs from the neighbors:
  - urt_scan.py        -- "are all my URT *boards* enumerated over USB?"
                          (--ping only tries IDs 1..30)
  - crabora_bus.py     -- standalone scan of the FIRST URT only, IDs 1..30
  - scan_ids.py (this) -- every URT, full ID range by default, fast

A full 1..253 sweep is practical because a ping that nobody answers only
costs the read timeout, and at 1 Mbaud a real reply arrives in well under
5 ms -- so we scan with a short 0.05 s timeout instead of the driver's
0.5 s default. (~13 s for 253 IDs instead of ~2 minutes.)

Usage:
    python scan_ids.py                     # all URTs, IDs 1..253
    python scan_ids.py --low 11 --high 33  # just the CRABORA ID block
    python scan_ids.py --port /dev/tty.usbmodem5B790333691
    python scan_ids.py -v                  # also read position/voltage/temp

Exit status: 0 if at least one servo answered somewhere, 1 if none did
(or no URT was found), so it can gate a startup script.
"""

import argparse
import sys

from crabora_bus import (
    Bus,
    BAUDRATE,
    MAX_VALID_ID,
    describe_id,
    find_feetech_ports,
)

# Short read timeout just for scanning: a live servo at 1 Mbaud replies in
# a few ms, so 0.05 s is generous, and it's what makes a 253-ID sweep quick.
SCAN_TIMEOUT = 0.05


def scan_port(port, low, high, verbose):
    """Ping IDs low..high on one URT. Returns the list of IDs that answered."""
    alive = []
    with Bus(port=port, timeout=SCAN_TIMEOUT) as bus:
        for servo_id in range(low, high + 1):
            if not bus.ping(servo_id):
                continue
            alive.append(servo_id)
            line = f"    ✓ {servo_id:3d}   {describe_id(servo_id)}"
            if verbose:
                telem = bus.read_telemetry([servo_id]).get(servo_id)
                if telem:
                    line += (f"   pos {telem['position']:4d}"
                             f"   {telem['voltage_v']:.1f} V"
                             f"   {telem['temp_c']} °C")
                else:
                    line += "   (answered ping but telemetry read failed)"
            print(line)
    return alive


def main():
    ap = argparse.ArgumentParser(
        description="Ping every servo ID behind each FE-URT-2 and report "
                    "which respond.")
    ap.add_argument("--low", type=int, default=1,
                    help="first ID to try (default 1)")
    ap.add_argument("--high", type=int, default=MAX_VALID_ID,
                    help=f"last ID to try (default {MAX_VALID_ID})")
    ap.add_argument("--port", action="append",
                    help="scan only this serial device (repeatable); "
                         "default is every URT found")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="also read position / voltage / temperature "
                         "from each servo that answers")
    args = ap.parse_args()

    if not 0 <= args.low <= args.high <= MAX_VALID_ID:
        ap.error(f"need 0 <= low <= high <= {MAX_VALID_ID}")

    ports = args.port or find_feetech_ports()
    if not ports:
        print("No FE-URT-2 found. Is it plugged in? (urt_scan.py can help "
              "diagnose USB-level problems.)")
        return 1

    print("=" * 60)
    print(f"CRABORA servo-ID scan  (IDs {args.low}..{args.high}, "
          f"{len(ports)} URT(s), {BAUDRATE} baud)")
    print("=" * 60)

    total = []
    for port in ports:
        print(f"\n  {port}")
        try:
            alive = scan_port(port, args.low, args.high, args.verbose)
        except Exception as e:  # noqa: BLE001 - report this URT, scan the rest
            print(f"    !! could not scan this URT: {e}")
            continue
        if alive:
            print(f"    -> {len(alive)} servo(s): {alive}")
        else:
            print("    -> no servos answered (bus power on? cable seated?)")
        total.extend(alive)

    print()
    if total:
        print(f"Total: {len(total)} servo(s) across {len(ports)} URT(s): "
              f"{sorted(total)}")
        return 0
    print("No servos answered on any URT.")
    print("  - 7.4V bus power on? (never through the URT)")
    print("  - Servo cable seated at both ends?")
    return 1


if __name__ == "__main__":
    sys.exit(main())
