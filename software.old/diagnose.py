"""
CRABORA connection diagnostic
=============================

Tests the Mac <-> FE-URT-2 link WITHOUT needing the servo bus power supply.

What this proves when it passes:
  - macOS sees the FE-URT-2 as a serial device
  - Your USB-C cable carries data (not charge-only)
  - The CH340 driver is loaded
  - Python's pyserial can open the port at 1 Mbps
  - The TX path from your Mac through the FE-URT-2 is working

What this does NOT prove:
  - The 7.4V power supply works
  - The servo is alive
  - The bus is healthy end-to-end

For full end-to-end verification, run pol.py instead (which
requires the power supply and a connected servo). To scan an already-
powered bus for live servo IDs, `python crabora_bus.py` is the dedicated
tool for that and exercises the full read/reply path.

Usage:
  python diagnose.py
"""

import glob
import sys
import time

try:
    import serial
except ImportError:
    print("✗ pyserial is not installed.")
    print("  Run: pip install pyserial")
    sys.exit(1)

from crabora_bus import BAUDRATE


def step(n, total, msg):
    print(f"\n[{n}/{total}] {msg}")


def find_serial_devices():
    """Return all candidate serial paths that look like USB serial adapters.

    Broader than crabora_bus.find_feetech_port() -- this diagnostic also
    wants to see usbserial-* / wchusbserial-* devices and report all
    candidates, not silently pick the first one.
    """
    paths = []
    for pattern in [
        "/dev/tty.usbserial-*",
        "/dev/tty.wchusbserial-*",
        "/dev/tty.usbmodem*",
    ]:
        paths.extend(sorted(glob.glob(pattern)))
    return paths


def main():
    TOTAL = 4

    # ---------------------------------------------------------------------
    step(1, TOTAL, "Looking for USB-serial devices on macOS...")
    devices = find_serial_devices()
    if not devices:
        print("✗ No /dev/tty.usbserial-* or similar device found.")
        print()
        print("  Things to check:")
        print("    - Is the FE-URT-2 plugged into your Mac via USB-C?")
        print("    - Is the USB-C cable data-capable? (Not all are.)")
        print("      Try the cable that came with your Mac.")
        print("    - Try a different USB-C port on your Mac.")
        print("    - macOS may need the WCH CH340 driver from wch-ic.com")
        print("      (only on rare counterfeit boards; most work natively).")
        sys.exit(1)

    print(f"✓ Found {len(devices)} serial device(s):")
    for d in devices:
        print(f"    {d}")

    # ---------------------------------------------------------------------
    step(2, TOTAL, "Picking the most likely FE-URT-2 device...")
    if len(devices) == 1:
        port = devices[0]
        print(f"✓ One device found, using: {port}")
    else:
        # Prefer usbserial-* over usbmodem*, since CH340 reports as usbserial.
        usbserials = [d for d in devices if "usbserial" in d]
        if usbserials:
            port = usbserials[0]
            print(f"✓ Multiple devices found, using usbserial: {port}")
            other = [d for d in devices if d != port]
            if other:
                print(f"  Ignored other device(s): {other}")
        else:
            port = devices[0]
            print(f"  No usbserial-* match. Falling back to first: {port}")

    # ---------------------------------------------------------------------
    step(3, TOTAL, f"Opening serial port at {BAUDRATE} baud...")
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.5)
        time.sleep(0.1)  # let macOS settle after open
    except serial.SerialException as e:
        print(f"✗ Could not open port: {e}")
        print()
        print("  Things to check:")
        print("    - Is another program (Arduino IDE, screen, minicom)")
        print("      already using this port? Close it.")
        print("    - Try unplugging and replugging the USB-C cable.")
        sys.exit(1)
    print(f"✓ Port opened successfully.")

    # ---------------------------------------------------------------------
    step(4, TOTAL, "Sending a test byte to verify TX path...")
    # We send a harmless dummy packet that any servo would ignore if there
    # WAS one alive on the bus. With no bus power, this is just exercising
    # the TX line. We don't expect a reply -- that part needs servo power.
    try:
        # 8 zero bytes -- gibberish, but exercises the TX path
        bytes_written = ser.write(b"\x00" * 8)
        ser.flush()
        time.sleep(0.05)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"✗ Write failed: {e}")
        ser.close()
        sys.exit(1)
    print(f"✓ Wrote {bytes_written} bytes to {port} without errors.")
    print("  (No reply expected -- the servo needs the 7.4V supply to respond.)")

    ser.close()

    # ---------------------------------------------------------------------
    print()
    print("=" * 60)
    print("Diagnostic complete: Mac <-> FE-URT-2 link is functional.")
    print("=" * 60)
    print()
    print("Next steps:")
    print("  1. Wire your 7.4V/7.5V supply to the FE-URT-2 screw terminals.")
    print("     VERIFY POLARITY with a multimeter before connecting.")
    print("  2. Connect a servo to the FE-URT-2's bus port.")
    print("  3. Run: python pol.py            # talk to a factory servo")
    print("     or:  python crabora_bus.py    # scan IDs 1..30 for live servos")
    print()


if __name__ == "__main__":
    main()
