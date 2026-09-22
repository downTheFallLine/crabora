#!/usr/bin/env python3
"""
CRABORA bring-up test 2: FE-URT-2 USB visibility
==================================================

Runs on the Pi. Checks that the Feetech FE-URT-2 board(s) plugged into
the Pi's USB port(s) actually enumerate and can be opened at the bus
baud rate -- BEFORE test 3 tries to talk Feetech protocol to servos
through them. If this test fails, the problem is USB/driver/permissions
(cable, hub, dialout group); if this test passes but the next one
doesn't, the problem is downstream of the URT (servo wiring/power/IDs).

Each FE-URT-2 presents as a WCH CH340-family USB-serial adapter
(VID 0x1A86). See ../urt_lib.py for the discovery logic (pyserial VID
match, with a /dev glob fallback for headless images where VID/PID
aren't exposed) -- this test is a thin pass/fail wrapper around it.

What it does:
  1. Lists every USB-serial device that looks like an FE-URT-2.
  2. Opens each one at the Feetech bus baud rate (1 Mbps) just long
     enough to confirm the OS will actually hand it over (catches
     permission errors -- e.g. user not in `dialout` -- and devices
     that enumerate but are already claimed by another process).
  3. Compares the count found against --expected (default 3, matching
     the current 6-leg wiring in doc/servoNotes.md: 3x FE-URT-2, 2 legs
     each) -- a mismatch WARNs rather than fails, since you may be
     bringing the robot up with fewer than all 3 connected.
  4. FAILs only if zero boards are found, or if one enumerates but
     can't be opened.

This test does not speak Feetech protocol and does not require any
servos to be connected or powered -- it only proves the Pi can see and
claim the USB-serial adapters. Servo-level bus discovery is test 3.

Usage:
  python3 test_02_urt_usb_visibility.py
  python3 test_02_urt_usb_visibility.py --expected 1   # bringing up one URT at a time
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import serial  # noqa: F401  -- import check only; urt_lib does the real work
except ImportError:
    print("FAIL: pyserial is not installed.")
    print("  Install it with:  python3 -m pip install pyserial")
    print("  (or on Raspberry Pi OS:  sudo apt install python3-serial)")
    sys.exit(1)

from urt_lib import (  # noqa: E402
    DEFAULT_BAUDRATE,
    connection,
    find_urt_devices,
    list_urt_ports,
)


def try_open(device_path):
    """Open + immediately close the port at the bus baud rate.

    Returns (ok, error_message).
    """
    try:
        ser = serial.Serial(device_path, DEFAULT_BAUDRATE, timeout=0.5)
        ser.close()
        return True, None
    except Exception as e:  # serial.SerialException, PermissionError, etc.
        return False, str(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--expected", type=int, default=3,
                     help="expected number of FE-URT-2 boards (default: 3, "
                          "the full 6-leg wiring). Mismatch warns, doesn't fail.")
    args = ap.parse_args()

    print("Crabora bring-up test 2: FE-URT-2 USB visibility")
    print(f"  looking for WCH-VID (0x1A86) serial adapters, "
          f"expecting {args.expected}...\n")

    urts = list_urt_ports()
    devices = find_urt_devices()

    if not urts and devices:
        print("No boards matched by USB vendor ID -- falling back to /dev "
              "glob match (less reliable; can't confirm these are actually "
              "FE-URT-2 boards and not some other serial device):")
        for path in devices:
            print(f"    {path}")
        print()

    if not devices:
        print("RESULT: FAIL -- no FE-URT-2 / serial-adapter devices found at all.")
        print("Checklist:")
        print("    ls /dev/ttyUSB*        (or /dev/ttyACM*)")
        print("    lsusb | grep -i 1a86")
        print("    groups                  # should include 'dialout'")
        print("    -- is the URT's USB cable actually plugged into the Pi?")
        sys.exit(1)

    ok = True
    for i, path in enumerate(devices, 1):
        p = next((p for p in urts if p.device == path), None)
        if p is not None:
            print(f"  [{i}] {path}")
            print(f"      vid:pid : {p.vid:04X}:{p.pid:04X}"
                  if p.vid is not None else "      vid:pid : unknown")
            print(f"      serial  : {p.serial_number or '?'}")
            print(f"      usb     : {p.location or '?'} ({connection(p.location)})")
        else:
            print(f"  [{i}] {path}  (glob match, no USB VID/PID info available)")

        opened, err = try_open(path)
        if opened:
            print(f"      open    : OK at {DEFAULT_BAUDRATE} baud")
        else:
            ok = False
            print(f"      open    : FAILED -- {err}")
        print()

    found = len(devices)
    if found != args.expected:
        print(f"WARN: found {found} board(s), expected {args.expected}. "
              f"OK if you're bringing these up one at a time.\n")

    if ok:
        print(f"RESULT: PASS -- {found} FE-URT-2 board(s) visible and openable.")
        sys.exit(0)
    else:
        print("RESULT: FAIL -- at least one board enumerated but could not be "
              "opened (permissions or already in use by another process).")
        print("    sudo usermod -aG dialout $USER   # then log out/in")
        sys.exit(1)


if __name__ == "__main__":
    main()
