"""
CRABORA servo ID setter
=======================

Assigns a new bus ID to a single Feetech STS servo (STS3215 et al.).

You do NOT need Feetech's Windows-only "FD" software for this. Changing a
servo's ID is just three register writes, and this script does them over
the same FE-URT-2 bus that pol.py uses.

How servo IDs work:
  - Every STS servo ships from the factory as ID 1.
  - Valid device IDs are 0..253. ID 254 (0xFE) is the broadcast address
    and must not be assigned. ID 255 (0xFF) is unusable -- it is the
    packet header byte.
  - The ID lives in register 5, inside the servo's write-protected EEPROM
    area. Register 55 ("Lock") gates EEPROM writes: 0 = unlocked, 1 =
    locked. So the sequence is: unlock -> write ID -> re-lock.

⚠  CONNECT ONLY ONE SERVO AT A TIME.
   Because fresh servos all share ID 1, two unconfigured servos on the bus
   are indistinguishable, and a write meant for one hits both. Bring servos
   up one at a time: connect, set ID, disconnect, repeat.

Hardware setup (same as pol.py):
  1. 7.4-7.5V power supply into the FE-URT-2 barrel jack
  2. ONE servo cabled to the FE-URT-2 bus port
  3. USB-C from the FE-URT-2 to your Mac

Usage:
  # Fresh servo (factory ID 1) -> ID 2
  python set_id.py 2

  # Servo currently at ID 5 -> ID 11
  python set_id.py 11 --current 5
"""

import argparse
import sys
import time

from crabora_bus import (
    Bus,
    ADDR_ID, ADDR_LOCK,
    BROADCAST_ID, MAX_VALID_ID,
    describe_id, find_feetech_ports,
)


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Set the bus ID of a single Feetech STS servo.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "new_id",
        type=int,
        help=f"the ID to assign (0-{MAX_VALID_ID})",
    )
    parser.add_argument(
        "-c",
        "--current",
        type=int,
        default=1,
        help="the servo's current ID (default: 1, the factory default)",
    )
    parser.add_argument(
        "--bus",
        type=int,
        default=0,
        help="which URT to use: 0 = first port (legs 1+2), 1 = second port (leg 3) "
             "(default: 0)",
    )
    return parser.parse_args()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    current_id = args.current
    new_id = args.new_id

    # --- validate the requested IDs --------------------------------------
    if not 0 <= new_id <= MAX_VALID_ID:
        print(f"✗ New ID {new_id} is out of range. Valid IDs are 0-{MAX_VALID_ID}.")
        print(f"  ({BROADCAST_ID} is the broadcast address; 255 is the header byte.)")
        sys.exit(1)
    if not 0 <= current_id <= MAX_VALID_ID:
        print(f"✗ Current ID {current_id} is out of range (0-{MAX_VALID_ID}).")
        sys.exit(1)

    print("=" * 60)
    print("CRABORA servo ID setter")
    print("=" * 60)
    print(f"  current ID : {current_id}")
    print(f"  new ID     : {new_id} ({describe_id(new_id)})")
    print()
    print("⚠  Make sure ONLY ONE servo is connected to the bus.")
    print()

    ports = find_feetech_ports()
    if args.bus >= len(ports):
        print(f"✗ --bus {args.bus} requested but only {len(ports)} URT(s) found: {ports}")
        sys.exit(1)

    with Bus(port=ports[args.bus]) as bus:
        print(f"Opened serial port: {bus.port_path}  (bus {args.bus})")

        # --- 1. confirm the servo is present at its current ID -----------
        print(f"\nPinging servo at current ID {current_id}...")
        if not bus.ping(current_id):
            print(f"✗ No response at ID {current_id}.")
            if current_id != new_id and bus.ping(new_id):
                print(f"  A servo DID answer at ID {new_id} -- it may already be set.")
            else:
                print("  Check the 7.4V bus power and the servo cable.")
                print("  If you don't know the current ID, pass it with --current.")
            sys.exit(1)
        print(f"✓ Servo responded at ID {current_id}.")

        if new_id == current_id:
            print(f"\nNothing to do -- servo is already at ID {new_id}.")
            return

        # --- 2. make sure the target ID isn't already taken --------------
        # With one servo connected this just times out, which is what we want.
        print(f"\nChecking that ID {new_id} is free on the bus...")
        if bus.ping(new_id):
            print(f"✗ A servo already answers at ID {new_id}. Aborting to avoid an")
            print("  ID collision. Disconnect that servo, or choose a different ID.")
            sys.exit(1)
        print(f"✓ ID {new_id} is free.")

        # --- 3. unlock EEPROM, write the new ID, re-lock -----------------
        # The ID register lives in the write-protected EEPROM area, so it
        # must be unlocked first. After the ID is written the servo
        # answers at the NEW id, so the re-lock has to be addressed
        # there -- the eeprom_unlocked() context manager would re-lock at
        # the wrong address, so the dance is written out explicitly.
        print(f"\nUnlocking EEPROM   (reg {ADDR_LOCK} <- 0, at ID {current_id})")
        bus.write_uint8(current_id, ADDR_LOCK, 0)

        print(f"Writing new ID     (reg {ADDR_ID} <- {new_id}, at ID {current_id})")
        bus.write_uint8(current_id, ADDR_ID, new_id)
        time.sleep(0.1)  # let the EEPROM write settle before re-addressing

        print(f"Re-locking EEPROM  (reg {ADDR_LOCK} <- 1, at ID {new_id})")
        bus.write_uint8(new_id, ADDR_LOCK, 1)

        # --- 4. verify ---------------------------------------------------
        print(f"\nVerifying...")
        if not bus.ping(new_id):
            print(f"✗ No response at the new ID {new_id}. Something went wrong.")
            sys.exit(1)
        stored = bus.read_uint8(new_id, ADDR_ID)
        if stored != new_id:
            print(f"✗ Servo answered at {new_id} but register {ADDR_ID} reads {stored}.")
            sys.exit(1)
        print(f"✓ Servo now responds at ID {new_id}, and register {ADDR_ID} confirms it.")

    print(f"\n✓ Done. Servo ID is now {new_id} ({describe_id(new_id)}).")


if __name__ == "__main__":
    main()
