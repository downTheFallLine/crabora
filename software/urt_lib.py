"""
urt_lib -- USB-UART for CRABORA on Raspberry Pi (Ubuntu)
=========================================================

Primary target: Ubuntu on Pi hardware, FE-URT-2 plugged in over USB.
The Pi's onboard GPIO UART is not used here -- all bus traffic goes
through the WCH USB-serial adapter.

On Linux the adapter usually enumerates as /dev/ttyUSB* (ch341 driver)
or occasionally /dev/ttyACM*. pyserial's USB VID lookup (WCH 0x1A86) is
the preferred discovery path; /dev globs are a fallback when VID/PID
aren't exposed (headless images, odd udev setups).

Typical Pi bring-up:
  - Plug FE-URT-2 into a Pi USB port (powered hub if running several).
  - Confirm:  ls /dev/ttyUSB*   and/or   lsusb | grep -i 1a86
  - Permission: add the runtime user to the dialout group, then re-login:
        sudo usermod -aG dialout $USER
  - Pin a specific board across reboots (optional):
        ls -l /dev/serial/by-id/
        Uart(port="/dev/serial/by-id/usb-1A86_...")

Layout:
  - Constants (WCH_VID, DEFAULT_BAUDRATE)
  - is_urt(), connection()           -- identify FE-URT-2 boards
  - list_urt_ports()                 -- pyserial ListPortInfo, sorted stably
  - find_urt_devices()               -- device path strings
  - find_urt_device()                -- single device, raises if none
  - Uart class                       -- open / close / read / write

Standalone usage:
  python uart_lib.py                 -- list every URT the host can see

Programmatic usage:
  from uart_lib import Uart, find_urt_devices

  for device in find_urt_devices():
      with Uart(port=device) as uart:
          uart.write(b"\\xff\\xff\\x01\\x02\\x01\\xfb")
          reply = uart.read(6)
"""

import glob
import time

import serial
from serial.tools import list_ports


# The Feetech bus runs at 1 Mbps. Half-duplex; the FE-URT-2 handles
# direction switching.
DEFAULT_BAUDRATE = 1_000_000

# WCH CH340-family USB-serial -- what the FE-URT-2 presents as.
WCH_VID = 0x1A86
URT_PID = 0x55D3

# Glob patterns tried when pyserial can't match on VID. Linux/Pi first.
_DEVICE_GLOB_PATTERNS = (
    "/dev/ttyUSB*",           # ch341 on Ubuntu / Raspberry Pi OS
    "/dev/ttyACM*",           # CDC ACM, some adapters
    "/dev/serial/by-id/usb-*1A86*",  # stable Linux by-id symlinks
    "/dev/tty.wchusbserial-*",       # macOS WCH driver
    "/dev/tty.usb*",                   # macOS generic usbserial
)


def is_urt(port):
    """True if this pyserial port looks like an FE-URT-2 (WCH serial chip).

    Match on vendor first; PID can vary across board revisions, so treat any
    WCH device as a URT candidate even if the PID isn't exactly 0x55D3.
    """
    return port.vid == WCH_VID


def connection(location):
    """Human label for how a board is attached, inferred from USB location.

    Linux sysfs paths (e.g. "1-1.2:1.0") and macOS location IDs (e.g.
    "2-1.2") both use dotted segments for downstream hub ports.
    """
    if not location:
        return "unknown"
    return "via hub" if "." in location else "direct"


def list_urt_ports():
    """Return pyserial ListPortInfo for every FE-URT-2, sorted stably.

    Sorted by USB serial number so the same physical board keeps its row
    between runs regardless of enumeration order.
    """
    urts = [p for p in list_ports.comports() if is_urt(p)]
    urts.sort(key=lambda p: (p.serial_number or "", p.device))
    return urts


def _glob_fallback_devices():
    """Device paths from /dev globs when pyserial can't see VID/PID."""
    seen = set()
    devices = []
    for pattern in _DEVICE_GLOB_PATTERNS:
        for path in sorted(glob.glob(pattern)):
            if path not in seen:
                seen.add(path)
                devices.append(path)
    return devices


def find_urt_devices():
    """Return device path strings for every FE-URT-2 the host can open.

    Prefers pyserial USB enumeration; falls back to /dev globs if no WCH
    boards are reported.
    """
    urts = list_urt_ports()
    if urts:
        return [p.device for p in urts]
    return _glob_fallback_devices()


def find_urt_device():
    """Return one FE-URT-2 device path, raising if none are visible."""
    devices = find_urt_devices()
    if not devices:
        raise RuntimeError(
            "No FE-URT-2 serial device found. Is the USB adapter plugged "
            "into the Pi?\n"
            "  - Check:  ls /dev/ttyUSB*   and   lsusb | grep -i 1a86\n"
            "  - Permission denied?  sudo usermod -aG dialout $USER  "
            "(then re-login)\n"
            "  - Pin a board:  ls /dev/serial/by-id/"
        )
    if len(devices) > 1:
        print(f"Multiple serial devices found: {devices}")
        print(f"Using the first: {devices[0]}")
    return devices[0]


class Urt:
    """A context-managed serial port at the Feetech bus baud rate.

        with Urt() as urt:
            urt.write(packet)
            reply = urt.read(expected_len)

    Pass port=None to auto-discover the first FE-URT-2 on open().
    Pass an explicit path (e.g. /dev/ttyUSB0 or a /dev/serial/by-id/...
    symlink) when running headless on the Pi with a fixed wiring layout.
    """

    def __init__(self, port=None, baudrate=DEFAULT_BAUDRATE, timeout=0.5):
        self.port_path = port
        self.baudrate = baudrate
        self._timeout = timeout
        self.ser = None

    def open(self):
        if self.ser is not None:
            return self
        if self.port_path is None:
            self.port_path = find_urt_device()
        self.ser = serial.Serial(
            self.port_path, self.baudrate, timeout=self._timeout,
        )
        time.sleep(0.1)  # let the port settle after open
        return self

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _require_open(self):
        if self.ser is None:
            raise IOError("Urt is not open. Call .open() or use 'with Urt()'.")

    def reset_input_buffer(self):
        self._require_open()
        self.ser.reset_input_buffer()

    def write(self, data):
        self._require_open()
        self.ser.write(data)

    def read(self, n):
        self._require_open()
        return self.ser.read(n)

    @property
    def timeout(self):
        if self.ser is not None:
            return self.ser.timeout
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value
        if self.ser is not None:
            self.ser.timeout = value


# Backward-compatible names for callers migrating off crabora_bus glob helpers.
find_feetech_ports = find_urt_devices
find_feetech_port = find_urt_device


def _print_scan():
    """Standalone listing: every URT plus non-URT serial devices."""
    urts = list_urt_ports()
    others = [p for p in list_ports.comports() if not is_urt(p)]

    print(f"Found {len(urts)} FE-URT-2 board(s):\n")
    if urts:
        for i, p in enumerate(urts, 1):
            serial_no = p.serial_number or "?"
            loc = p.location or "?"
            print(f"  [{i}] {p.device}")
            print(f"      serial : {serial_no}")
            print(f"      usb    : {loc}  ({connection(p.location)})")
            print()
    else:
        fallback = _glob_fallback_devices()
        if fallback:
            print("  (no WCH boards via pyserial; glob fallback found:)")
            for path in fallback:
                print(f"    {path}")
            print()
        else:
            print("  (none)\n")
            print("  Pi checklist:")
            print("    ls /dev/ttyUSB*")
            print("    lsusb | grep -i 1a86")
            print("    groups   # should include 'dialout'")
            print()

    if others:
        print("Other serial devices seen (not URTs):")
        for p in others:
            vidpid = (
                f"{p.vid:04X}:{p.pid:04X}"
                if p.vid is not None and p.pid is not None
                else "----:----"
            )
            print(f"  - {p.device}  [{vidpid}]  {p.description}")
        print()


if __name__ == "__main__":
    _print_scan()
