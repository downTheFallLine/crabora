"""
CRABORA proof of life
=====================

The very first executable code in the CRABORA project. The goal is narrow:
prove that the Mac can talk to one Feetech STS3215 over the FE-URT-2 bus.

What it does:
  1. Opens the serial port
  2. Pings servo ID 1 (the factory default)
  3. Reads the present position
  4. Commands a move to 0, reads back
  5. Commands a move to 4095, reads back

What it does NOT do:
  - Assign IDs (the servo will have its factory ID of 1)
  - Tune any registers
  - Use any external SDK

This script is the smallest possible end-to-end exerciser of the bus.
It relies on the crabora_bus driver for the STS packet protocol -- read
that module if you want to see what bus servos actually do under the
hood (checksums, framing, register reads/writes).

Hardware setup before running:
  1. 7.5V power supply plugged into the FE-URT-2's barrel jack
  2. Servo cable from FE-URT-2's bus port to the servo
  3. USB-C cable from FE-URT-2 to your Mac
  4. The servo's red LED should briefly flash when bus power comes on

Usage:
  python pol.py
"""

import sys
import time

from crabora_bus import (
    Bus,
    ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION,
    find_feetech_ports,
)


SERVO_ID = 13  # factory default for a new servo


def move_and_report(bus, servo_id, goal):
    """Command a move, wait for it to settle, report the result."""
    print(f"  → commanding position {goal}")
    bus.write_uint16(servo_id, ADDR_GOAL_POSITION, goal)
    time.sleep(1.0)  # crude wait; later we'll poll for "moving" status
    actual = bus.read_uint16(servo_id, ADDR_PRESENT_POSITION)
    delta = actual - goal
    print(f"  ← reached position {actual} (off by {delta:+d})")


def main():
    # pol.py talks to a factory servo (ID 1) before it has a leg assignment.
    # Pass --bus 1 on the command line to use the second URT instead.
    bus_idx = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 0
    ports = find_feetech_ports()
    if not ports:
        print("✗ No URT devices found. Is the FE-URT-2 plugged in?")
        sys.exit(1)
    if bus_idx >= len(ports):
        print(f"✗ Bus index {bus_idx} requested but only {len(ports)} URT(s) found: {ports}")
        sys.exit(1)

    with Bus(port=ports[bus_idx]) as bus:
        print(f"Opened serial port: {bus.port_path}  (bus {bus_idx})")

        print(f"\nPinging servo ID {SERVO_ID}...")
        if not bus.ping(SERVO_ID):
            print(
                "✗ No response. Check:\n"
                "  - 7.5V supply plugged into the FE-URT-2 barrel jack and switched on\n"
                "  - Servo cable seated firmly at both ends\n"
                "  - Servo LED flashed briefly when bus power came up\n"
            )
            sys.exit(1)
        print("✓ Servo responded to ping.")

        print(f"\nEnabling torque (so the servo will hold position)...")
        bus.enable_torque(SERVO_ID, on=True)

        start = bus.read_uint16(SERVO_ID, ADDR_PRESENT_POSITION)
        print(f"Starting position: {start} (0=min, 4095=max, 2048=center)")

        print("\nRunning motion test:")
        move_and_report(bus, SERVO_ID, 0)
        time.sleep(1.0)
        move_and_report(bus, SERVO_ID, 4095)

        print("\nDisabling torque (servo goes limp)...")
        bus.enable_torque(SERVO_ID, on=False)

    print("\n✓ Proof of life complete. CRABORA is real.")


if __name__ == "__main__":
    main()
