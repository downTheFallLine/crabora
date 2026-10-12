#!/usr/bin/env python3
"""
servo -- one tool for every per-servo operation on the Feetech bus.

Replaces a drawer of single-purpose scripts (scan_ids, set_id,
reset_overload, servo_report, examine_servo) with one command that
understands the register map. The point is not tidiness: it is that the
next one-off need -- read this register, flip that bit -- stops
requiring a new script, which is how the last drawer filled up.

    servo.py scan                      what IDs are alive, on which board
    servo.py info 33                   the registers worth seeing, decoded
    servo.py get 33 present_load       read any register, by name
    servo.py set 33 torque_limit 500   write any register
    servo.py id 1 33                   change a servo's ID, safely
    servo.py reset 33                  clear a latched fault
    servo.py reset --all
    servo.py regs                      list the register map

Registers are addressed BY NAME. `get 33 present_load` is checkable;
`get 33 60` is a typo waiting to brick something.

Safety:
  - EEPROM registers are unlocked and re-locked around the write
  - registers that can make a servo unreachable (id, baud, mode, lock)
    need an explicit --force
  - `id` refuses outright unless exactly ONE servo is on the bus, since
    a broadcast ID write with two servos connected leaves you with two
    servos sharing an ID and no way to tell them apart

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "walk"))

from gait_lib import (  # noqa: E402
    COMM_SUCCESS, JOINT_NAME, LEG_NAME,
    BusError, PacketHandler,
    find_urt_devices, open_ports, read1, read2, to_degrees, write1, write2,
)

# Register map. Addresses marked (v) are verified by use elsewhere in
# this codebase or in software.old/crabora_bus.py; the rest are from the
# common Feetech STS map and are read-only here, so a wrong address
# shows up as a nonsense reading rather than damage.
#
#   name: (addr, size, area, writable, dangerous, description)
REGS = {
    "id":               (5,  1, "E", True,  True,  "bus ID (v)"),
    "baud":             (6,  1, "E", True,  True,  "baud rate index"),
    "min_angle":        (9,  2, "E", True,  False, "travel limit, low (v)"),
    "max_angle":        (11, 2, "E", True,  False, "travel limit, high (v)"),
    "max_temp":         (13, 1, "E", True,  False, "shutdown temperature"),
    "max_voltage":      (14, 1, "E", True,  False, "max voltage, 0.1 V"),
    "min_voltage":      (15, 1, "E", True,  False, "min voltage, 0.1 V"),
    "max_torque":       (16, 2, "E", True,  False, "startup torque limit"),
    "mode":             (33, 1, "E", True,  True,  "0 position, 1 wheel (v)"),
    "torque_enable":    (40, 1, "S", True,  False, "1 driven, 0 limp (v)"),
    "acceleration":     (41, 1, "S", True,  False, "move acceleration"),
    "goal_position":    (42, 2, "S", True,  False, "target position (v)"),
    "goal_time":        (44, 2, "S", True,  False, "0 = speed controlled (v)"),
    "goal_speed":       (46, 2, "S", True,  False, "target speed (v)"),
    "torque_limit":     (48, 2, "S", True,  False, "running torque cap, 0..1000"),
    "lock":             (55, 1, "E", True,  True,  "EEPROM lock (v)"),
    "present_position": (56, 2, "S", False, False, "where it is (v)"),
    "present_speed":    (58, 2, "S", False, False, "how fast (v)"),
    "present_load":     (60, 2, "S", False, False, "load, bit 10 = direction"),
    "present_voltage":  (62, 1, "S", False, False, "0.1 V units (v)"),
    "present_temp":     (63, 1, "S", False, False, "degrees C (v)"),
    "servo_status":     (65, 1, "S", False, False, "error bitmask"),
    "moving":           (66, 1, "S", False, False, "1 while travelling (v)"),
}

ERROR_FLAGS = {0: "Voltage", 1: "Sensor", 2: "Temperature",
               3: "Current", 4: "Angle", 5: "Overload"}

INFO_ORDER = ("present_position", "min_angle", "max_angle", "present_load",
              "present_speed", "moving", "present_voltage", "present_temp",
              "servo_status", "torque_enable", "torque_limit", "mode")


def decode_errors(v):
    return [n for b, n in ERROR_FLAGS.items() if v & (1 << b)]


def decode_load(raw):
    return -(raw & 0x3FF) if raw & 0x400 else (raw & 0x3FF)


def describe(sid):
    leg, joint = sid // 10, sid % 10
    if 1 <= leg <= 6 and joint in JOINT_NAME:
        return f"leg {leg} {JOINT_NAME[joint]} ({LEG_NAME[leg]})"
    return "not a CRABORA leg/joint ID"


def read_reg(packet, port, sid, name):
    addr, size, *_ = REGS[name]
    fn = read1 if size == 1 else read2
    return fn(packet, port, sid, addr, name)


def write_reg(packet, port, sid, name, value):
    """Write, unlocking EEPROM first if the register lives there."""
    addr, size, area, *_ = REGS[name]
    fn = write1 if size == 1 else write2
    if area == "E":
        write1(packet, port, sid, REGS["lock"][0], 0, "unlock EEPROM")
    try:
        fn(packet, port, sid, addr, value, name)
    finally:
        if area == "E":
            try:
                write1(packet, port, sid, REGS["lock"][0], 1, "re-lock EEPROM")
            except BusError:
                pass


def find_one(packet, ports, sid):
    """Which open port answers for this ID? (port, device) or (None, None)."""
    for device, port in ports.items():
        _m, comm, _e = packet.ping(port, sid)
        if comm == COMM_SUCCESS:
            return port, device
    return None, None


def scan_all(packet, ports, lo, hi):
    """{sid: [devices]} for every ID that answers anywhere."""
    found = {}
    for device, port in ports.items():
        for sid in range(lo, hi + 1):
            _m, comm, _e = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                found.setdefault(sid, []).append(device)
    return found


# =============================================================
# SUBCOMMANDS
# =============================================================
def cmd_scan(args, packet, ports):
    print(f"Scanning IDs {args.low}..{args.high} on {len(ports)} board(s)\n")
    found = scan_all(packet, ports, args.low, args.high)
    if not found:
        print("  nothing answered. Check power and wiring, and run "
              "tests/test_02_urt_usb_visibility.py")
        return 1
    print(f"  {'id':>4}  {'board':<22} what it should be")
    for sid in sorted(found):
        for dev in found[sid]:
            print(f"  {sid:>4}  {dev:<22} {describe(sid)}")
    dupes = {s: d for s, d in found.items() if len(d) > 1}
    if dupes:
        print(f"\n  ⚠ answering on more than one board -- two servos share "
              f"an ID:")
        for sid, devs in dupes.items():
            print(f"      {sid}: {', '.join(devs)}")
        return 1
    print(f"\n  {len(found)} servo(s).")
    return 0


def cmd_info(args, packet, ports):
    port, device = find_one(packet, ports, args.id)
    if port is None:
        print(f"servo {args.id} did not answer on any board.")
        return 1
    print(f"servo {args.id} on {device} -- {describe(args.id)}\n")
    for name in INFO_ORDER:
        try:
            v = read_reg(packet, port, args.id, name)
        except BusError as e:
            print(f"  {name:<18} <read failed: {e}>")
            continue
        extra = ""
        if name in ("present_position", "min_angle", "max_angle",
                    "goal_position"):
            extra = f"  ({to_degrees(v):+.1f}°)"
        elif name == "present_load":
            extra = f"  ({decode_load(v):+d} signed)"
        elif name in ("present_voltage",):
            extra = f"  ({v / 10:.1f} V)"
        elif name == "servo_status":
            flags = decode_errors(v)
            extra = f"  ({', '.join(flags) if flags else 'no errors'})"
        elif name == "mode":
            extra = "  (position)" if v == 0 else f"  (NOT position mode)"
        elif name == "torque_enable":
            extra = "  (driven)" if v else "  (limp)"
        print(f"  {name:<18} {v:>6}{extra}")
    return 0


def cmd_get(args, packet, ports):
    if args.reg not in REGS:
        print(f"unknown register {args.reg!r}. Try: servo.py regs")
        return 1
    port, _d = find_one(packet, ports, args.id)
    if port is None:
        print(f"servo {args.id} did not answer.")
        return 1
    v = read_reg(packet, port, args.id, args.reg)
    print(v)
    return 0


def cmd_set(args, packet, ports):
    if args.reg not in REGS:
        print(f"unknown register {args.reg!r}. Try: servo.py regs")
        return 1
    addr, size, area, writable, dangerous, desc = REGS[args.reg]
    if not writable:
        print(f"{args.reg} is read-only.")
        return 1
    if dangerous and not args.force:
        print(f"{args.reg} can make the servo unreachable ({desc}).")
        print(f"  Re-run with --force if you mean it.")
        return 1
    hi = 255 if size == 1 else 65535
    if not 0 <= args.value <= hi:
        print(f"{args.reg} is {size} byte(s): 0..{hi}")
        return 1
    port, _d = find_one(packet, ports, args.id)
    if port is None:
        print(f"servo {args.id} did not answer.")
        return 1
    before = read_reg(packet, port, args.id, args.reg)
    write_reg(packet, port, args.id, args.reg, args.value)
    after = read_reg(packet, port, args.id, args.reg)
    print(f"servo {args.id} {args.reg}: {before} -> {after}"
          + ("" if after == args.value else
             f"   ⚠ wanted {args.value}; the servo did not take it"))
    return 0 if after == args.value else 1


def cmd_id(args, packet, ports):
    """Change a servo's ID. The dangerous one, so it checks first."""
    found = scan_all(packet, ports, 0, 253)
    if args.old not in found:
        print(f"no servo answering at ID {args.old}.")
        return 1
    if len(found) != 1:
        print(f"⚠ {len(found)} servos are on the bus: "
              f"{', '.join(map(str, sorted(found)))}")
        print("  Connect ONE servo before changing an ID. With several")
        print("  attached you cannot tell afterwards which one you changed,")
        print("  and two servos sharing an ID are indistinguishable.")
        return 1
    if args.new in found and args.new != args.old:
        print(f"ID {args.new} is already taken.")
        return 1
    if not 0 <= args.new <= 253:
        print("new ID must be 0..253.")
        return 1

    port, device = find_one(packet, ports, args.old)
    print(f"servo {args.old} on {device} -> ID {args.new} "
          f"({describe(args.new)})")
    if not args.yes:
        try:
            if input("  Continue? [y/N] ").strip().lower() != "y":
                print("  Stopped.")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n  Stopped.")
            return 1

    write_reg(packet, port, args.old, "id", args.new)
    time.sleep(0.1)
    _m, comm, _e = packet.ping(port, args.new)
    if comm != COMM_SUCCESS:
        print(f"  ⚠ nothing answers at {args.new}. Power-cycle and rescan.")
        return 1
    print(f"  ✓ now answering at {args.new}.")
    return 0


def cmd_reset(args, packet, ports):
    """Clear a latched fault by toggling torque off and on."""
    targets = (sorted(scan_all(packet, ports, 0, 253)) if args.all
               else [args.id])
    if not targets or targets == [None]:
        print("nothing to reset.")
        return 1
    cleared, still = [], []
    for sid in targets:
        port, _d = find_one(packet, ports, sid)
        if port is None:
            continue
        before = decode_errors(read_reg(packet, port, sid, "servo_status"))
        if not before:
            continue
        write_reg(packet, port, sid, "torque_enable", 0)
        time.sleep(0.05)
        write_reg(packet, port, sid, "torque_enable", 1)
        time.sleep(0.05)
        write_reg(packet, port, sid, "torque_enable", 0)   # leave it limp
        after = decode_errors(read_reg(packet, port, sid, "servo_status"))
        print(f"  servo {sid}: {', '.join(before)} -> "
              f"{', '.join(after) if after else 'clear'}")
        (still if after else cleared).append(sid)
    if not cleared and not still:
        print("  no latched faults found.")
        return 0
    if still:
        print(f"\n  ⚠ still faulted: {' '.join(map(str, still))}")
        print("  An overload that will not clear usually means the joint is")
        print("  still jammed, or the supply is sagging. Fix the cause.")
        return 1
    print(f"\n  cleared {len(cleared)}. Torque left OFF.")
    return 0


def cmd_regs(args, packet, ports):
    print(f"  {'name':<18} {'addr':>4} {'size':>4} {'area':>4} "
          f"{'write':>5}  description")
    for name in sorted(REGS, key=lambda n: REGS[n][0]):
        addr, size, area, writable, dangerous, desc = REGS[name]
        w = "DANGER" if dangerous else ("yes" if writable else "no")
        print(f"  {name:<18} {addr:>4} {size:>4} {area:>4} {w:>5}  {desc}")
    print("\n  area: E = EEPROM (survives power-off, unlocked automatically")
    print("        for writes), S = SRAM (lost on power-off)")
    print("  (v) = address verified by use elsewhere in this codebase")
    return 0


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="One tool for every per-servo operation.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # --ports lives on each subcommand, not the top level: with
    # nargs="+" a global one swallows the subcommand name as a port.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--ports", nargs="+")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", parents=[common], help="what IDs are alive")
    s.add_argument("--low", type=int, default=0)
    s.add_argument("--high", type=int, default=253)
    s.set_defaults(fn=cmd_scan)

    s = sub.add_parser("info", parents=[common], help="the registers worth seeing, decoded")
    s.add_argument("id", type=int)
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("get", parents=[common], help="read a register by name")
    s.add_argument("id", type=int)
    s.add_argument("reg")
    s.set_defaults(fn=cmd_get)

    s = sub.add_parser("set", parents=[common], help="write a register by name")
    s.add_argument("id", type=int)
    s.add_argument("reg")
    s.add_argument("value", type=int)
    s.add_argument("--force", action="store_true",
                   help="allow a register that can make the servo unreachable")
    s.set_defaults(fn=cmd_set)

    s = sub.add_parser("id", parents=[common], help="change a servo's ID (one servo on the bus)")
    s.add_argument("old", type=int)
    s.add_argument("new", type=int)
    s.add_argument("--yes", "-y", action="store_true")
    s.set_defaults(fn=cmd_id)

    s = sub.add_parser("reset", parents=[common], help="clear a latched fault")
    s.add_argument("id", type=int, nargs="?")
    s.add_argument("--all", action="store_true")
    s.set_defaults(fn=cmd_reset)

    s = sub.add_parser("regs", parents=[common], help="list the register map")
    s.set_defaults(fn=cmd_regs)

    args = ap.parse_args()
    if args.cmd == "reset" and args.id is None and not args.all:
        ap.error("reset needs an ID, or --all")

    if args.cmd == "regs":          # no bus needed
        return cmd_regs(args, None, None)

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found. Run "
              "tests/test_02_urt_usb_visibility.py.")
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1
    packet = PacketHandler(0)  # STS servos use protocol_end=0
    try:
        return args.fn(args, packet, ports)
    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
