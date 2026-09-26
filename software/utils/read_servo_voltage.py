#!/usr/bin/env python3
"""
Find one or more Feetech STS-series servos (e.g. STS3215) across the URT
serial ports, and report voltage, voltage limits, temperature and error status.

Usage:
    python3 utils/read_servo_voltage.py 41
    python3 utils/read_servo_voltage.py 41 42 43
    python3 utils/read_servo_voltage.py 41 --ports /dev/ttyACM0 /dev/ttyUSB0

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
BAUDRATE = 1_000_000

# --- Register addresses (STS3215 / common Feetech map) ---
ADDR_MAX_VOLTAGE_LIMIT = 14    # 1 byte, units of 0.1V
ADDR_MIN_VOLTAGE_LIMIT = 15    # 1 byte, units of 0.1V
ADDR_PRESENT_VOLTAGE   = 62    # 1 byte, units of 0.1V
ADDR_PRESENT_TEMP      = 63    # 1 byte, degrees C
ADDR_SERVO_STATUS      = 65    # 1 byte, error bitmask

ERROR_FLAGS = {
    0: "Voltage Error",
    1: "Sensor Error",
    2: "Temperature Error",
    3: "Current Error",
    4: "Angle Error",
    5: "Overload Error",
}


def open_port(device):
    """Open a URT port at the bus baud rate. Returns a PortHandler or None."""
    if not os.path.exists(device):
        print(f"  {device}: not present")
        return None
    port = PortHandler(device)
    port.baudrate = BAUDRATE
    try:
        if not port.openPort():
            print(f"  {device}: failed to open")
            return None
    except Exception as exc:  # pyserial raises on permissions, busy port, etc.
        print(f"  {device}: failed to open ({exc})")
        return None
    return port


def find_servo(ports, packet, servo_id):
    """Ping servo_id on every open port. Returns list of devices that answered."""
    found = []
    for device, port in ports.items():
        _model, comm, _err = packet.ping(port, servo_id)
        if comm == COMM_SUCCESS:
            found.append(device)
    return found


def read_byte(port, packet, servo_id, addr):
    value, comm, _err = packet.read1ByteTxRx(port, servo_id, addr)
    if comm != COMM_SUCCESS:
        raise IOError(f"read of register {addr} failed ({packet.getTxRxResult(comm)})")
    return value


def report_servo(device, port, packet, servo_id):
    print(f"\n--- Servo ID {servo_id} on {device} ---")
    try:
        max_v = read_byte(port, packet, servo_id, ADDR_MAX_VOLTAGE_LIMIT)
        min_v = read_byte(port, packet, servo_id, ADDR_MIN_VOLTAGE_LIMIT)
        present_v = read_byte(port, packet, servo_id, ADDR_PRESENT_VOLTAGE)
        temp = read_byte(port, packet, servo_id, ADDR_PRESENT_TEMP)
        status = read_byte(port, packet, servo_id, ADDR_SERVO_STATUS)
    except IOError as exc:
        print(f"  Failed to read servo: {exc}")
        return False

    print(f"  Configured min voltage: {min_v * 0.1:.1f} V")
    print(f"  Configured max voltage: {max_v * 0.1:.1f} V")
    print(f"  Present voltage:        {present_v * 0.1:.1f} V")
    print(f"  Present temperature:    {temp} C")

    ok = True
    if present_v < min_v:
        print(f"  ⚠ Present voltage is BELOW the configured minimum "
              f"({present_v * 0.1:.1f}V < {min_v * 0.1:.1f}V)")
        ok = False
    elif present_v > max_v:
        print(f"  ⚠ Present voltage is ABOVE the configured maximum "
              f"({present_v * 0.1:.1f}V > {max_v * 0.1:.1f}V)")
        ok = False
    else:
        print("  ✓ Present voltage is within configured range")

    print(f"  Status byte: {status} (binary: {status:08b})")
    if status == 0:
        print("  ✓ No active errors")
    else:
        ok = False
        print("  Active errors:")
        for bit, name in ERROR_FLAGS.items():
            if status & (1 << bit):
                print(f"    ✗ {name}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("servo_ids", nargs="+", type=int, metavar="ID",
                        help="servo ID(s) to look for, e.g. 41")
    parser.add_argument("--ports", nargs="+", default=DEFAULT_PORTS,
                        help=f"serial ports to scan (default: {' '.join(DEFAULT_PORTS)})")
    args = parser.parse_args()

    for sid in args.servo_ids:
        if not 0 <= sid <= 253:
            parser.error(f"servo ID {sid} out of range 0-253")

    packet = PacketHandler(0)  # STS servos use protocol_end=0

    print(f"Scanning ports at {BAUDRATE} baud:")
    ports = {}
    for device in args.ports:
        port = open_port(device)
        if port:
            print(f"  {device}: open")
            ports[device] = port
    if not ports:
        print("No usable ports -- check the URT USB cables, `ls /dev/ttyACM*`, "
              "and that you're in the dialout group.")
        return 2

    exit_code = 0
    try:
        for sid in args.servo_ids:
            found = find_servo(ports, packet, sid)
            if not found:
                print(f"\n--- Servo ID {sid} ---\n  ✗ Not found on any port "
                      f"({', '.join(ports)}) -- check wiring, power and ID.")
                exit_code = 1
                continue
            if len(found) > 1:
                print(f"\n  ⚠ Servo ID {sid} answered on multiple ports: "
                      f"{', '.join(found)} -- duplicate ID?")
            for device in found:
                if not report_servo(device, ports[device], packet, sid):
                    exit_code = 1
    finally:
        for port in ports.values():
            port.closePort()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
