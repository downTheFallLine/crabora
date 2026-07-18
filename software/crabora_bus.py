"""
crabora_bus -- shared Feetech STS bus driver for CRABORA
========================================================

The Feetech protocol code used to live, copied, in every script: pol.py,
diagnose.py, walk.py, spin.py, set_id.py, set_limits.py, set_middle.py,
wheel_off.py. They all duplicated the same packet builder, the same
read/write helpers, the same port discovery. This module pulls all that
into one place so each script can stop reinventing it.

It also adds the one new capability gait work depends on:

  *** SYNC WRITE (instruction 0x83) ***
  Write the same register block on N servos in a SINGLE bus packet.
  Without this, "move three joints" means three sequential packets, and
  the joints stagger -- one starts moving while you're still mid-write to
  the next. With sync write, the bus broadcasts one frame and all the
  servos act on it simultaneously. This is how a leg takes a real step,
  and how multiple legs coordinate later.

Layout:
  - Constants (BAUDRATE, register ADDR_*, MODE_*, conversion magnitudes)
  - Servo-ID helpers (COXA/FEMUR/TIBIA, make_id, leg_of, joint_of)
  - find_feetech_port()             -- port discovery (same logic as before)
  - Speed conversion (rpm_to_raw / raw_to_rpm / rpm_to_pos_speed)
  - Bus class
      .open / .close / context manager
      .ping  .read_uint8/16  .write_uint8/16  .read_bytes
      .sync_write                   -- N servos, one packet  <-- the new bit
      .enable_torque / .sync_enable_torque
      .write_goal_move              -- the walk.py-style speed-controlled move
      .sync_goal_move               -- same idea, but for many servos at once
      .wait_until_stopped / .sync_wait_until_stopped
      .eeprom_unlocked(servo_id)    -- context manager for the LOCK dance

Standalone usage:
  python crabora_bus.py             -- opens the bus, scans IDs 1..30,
                                       reports which respond (a quick
                                       "what's alive on the bus" check)

Programmatic usage:
  from crabora_bus import Bus, CENTER_POSITION, rpm_to_pos_speed

  with Bus() as bus:
      bus.enable_torque(11)
      speed = rpm_to_pos_speed(15)             # 15 rpm in position-mode units
      bus.sync_goal_move({
          11: (CENTER_POSITION - 300, speed),  # coxa
          12: (CENTER_POSITION + 200, speed),  # femur
          13: (CENTER_POSITION - 100, speed),  # tibia
      })
      bus.sync_wait_until_stopped([11, 12, 13], timeout=2.0)

The existing scripts (walk.py, spin.py, etc.) continue to work unchanged.
They can be migrated to import from this module as a separate step.
"""

import contextlib
import glob
import time

import serial


# =============================================================================
# Protocol constants for the Feetech STS series (STS3215 et al.)
# =============================================================================
# The bus runs at 1 Mbps. Half-duplex; the FE-URT-2 handles direction switching.
BAUDRATE = 1_000_000

# Packet structure:
#   0xFF 0xFF  <id>  <length>  <instruction>  <param1>...<paramN>  <checksum>
HEADER = b"\xff\xff"

# Instruction codes (Feetech SCS/STS family)
INST_PING       = 0x01
INST_READ       = 0x02
INST_WRITE      = 0x03
INST_SYNC_WRITE = 0x83   # the new one: write to many IDs in one packet

# Special IDs: 254 is broadcast (no reply), 255 is the header byte.
BROADCAST_ID = 0xFE
MAX_VALID_ID = 253

# -----------------------------------------------------------------------------
# Memory-table addresses on the STS3215. Names match what the inline copies
# in walk.py and spin.py used, so migration is a straight import swap.
# -----------------------------------------------------------------------------
ADDR_ID               = 5   # 1 byte  -- bus ID (EEPROM, locked)
ADDR_MIN_ANGLE        = 9   # 2 bytes -- firmware angle limit (low)
ADDR_MAX_ANGLE        = 11  # 2 bytes -- firmware angle limit (high)
ADDR_MODE             = 33  # 1 byte  -- operation mode (EEPROM, locked)
ADDR_TORQUE_ENABLE    = 40  # 1 byte  -- 1 = driven, 0 = limp
ADDR_GOAL_POSITION    = 42  # 2 bytes -- start of the move block (see write_goal_move)
ADDR_GOAL_TIME        = 44  # 2 bytes -- 0 = speed-controlled, else time-based
ADDR_GOAL_SPEED       = 46  # 2 bytes -- target speed (sign-magnitude in wheel mode)
ADDR_LOCK             = 55  # 1 byte  -- EEPROM write lock (0 unlocked, 1 locked)
ADDR_PRESENT_POSITION = 56  # 2 bytes -- where the servo currently is
ADDR_PRESENT_SPEED    = 58  # 2 bytes -- how fast it's actually turning
ADDR_PRESENT_VOLTAGE  = 62  # 1 byte  -- bus voltage in 0.1 V units (e.g. 74 = 7.4 V)
ADDR_PRESENT_TEMP     = 63  # 1 byte  -- internal temperature in °C
ADDR_MOVING           = 66  # 1 byte  -- 1 while travelling to goal, 0 arrived

# Operation modes (register 33). Lives in the locked EEPROM area.
MODE_POSITION = 0
MODE_WHEEL    = 1   # constant-speed
MODE_PWM      = 2
MODE_STEP     = 3
MODE_NAMES = {
    MODE_POSITION: "position",
    MODE_WHEEL:    "wheel / constant-speed",
    MODE_PWM:      "PWM",
    MODE_STEP:     "step",
}

# Position is 0..4095 across one full revolution; 2048 is center.
COUNTS_PER_REV   = 4096
DEGREES_PER_REV  = 360
MIN_POSITION     = 0
CENTER_POSITION  = 2048
MAX_POSITION     = 4095

# Speed conversion. Goal-speed register is steps/second, 4096 steps/rev,
# so register_value = rpm * STEPS_PER_REV / 60.
STEPS_PER_REV = 4096

# Feetech encodes the signed 16-bit goal-speed as sign-magnitude:
# bit 15 = sign, bits 0..14 = magnitude.
SIGN_BIT             = 0x8000
MAX_SPEED_MAGNITUDE  = 0x7FFF

# STS3215's no-load top speed is ~45-60 rpm depending on bus voltage; past
# this the servo just runs flat out and the script can warn.
TYPICAL_MAX_RPM = 60

# -----------------------------------------------------------------------------
# Error byte bit-field (the byte at offset 4 of a reply packet).
# 0 means "no error".  Multiple bits can be set at once.
# -----------------------------------------------------------------------------
ERR_VOLTAGE     = 0x01   # bus voltage out of range (under- or over-volt)
ERR_ANGLE_LIMIT = 0x02   # commanded position outside the firmware limits
ERR_OVERHEAT    = 0x04   # internal temperature above threshold
ERR_RANGE       = 0x08   # received register value out of range
ERR_CHECKSUM    = 0x10   # received packet failed checksum
ERR_OVERLOAD    = 0x20   # load above threshold for too long (often a stall)
ERR_INSTRUCTION = 0x40   # unknown / malformed instruction
ERROR_BITS = [
    (ERR_VOLTAGE,     "voltage"),
    (ERR_ANGLE_LIMIT, "angle-limit"),
    (ERR_OVERHEAT,    "overheat"),
    (ERR_RANGE,       "range"),
    (ERR_CHECKSUM,    "checksum"),
    (ERR_OVERLOAD,    "overload"),
    (ERR_INSTRUCTION, "instruction"),
]


def decode_error_byte(error_byte):
    """Return a human label for an error byte. '' if 0; '|'-joined names otherwise.

    >>> decode_error_byte(0x00)
    ''
    >>> decode_error_byte(0x24)
    'overload|overheat'
    """
    if error_byte == 0:
        return ""
    names = [name for bit, name in ERROR_BITS if error_byte & bit]
    unknown = error_byte & ~sum(bit for bit, _ in ERROR_BITS)
    if unknown:
        names.append(f"unknown(0x{unknown:02x})")
    return "|".join(names) if names else f"0x{error_byte:02x}"


# =============================================================================
# Servo ID helpers -- CRABORA's 2-digit (leg, joint) scheme
# =============================================================================
# Digit 1 = leg number (1..9), digit 2 = joint within that leg.
# Joints, walking from body to foot:
COXA  = 1
FEMUR = 2
TIBIA = 3
JOINT_NAMES = {COXA: "coxa", FEMUR: "femur", TIBIA: "tibia"}


def make_id(leg, joint):
    """Compose a 2-digit servo ID, e.g. make_id(1, COXA) == 11."""
    if not 1 <= leg <= 9:
        raise ValueError(f"leg must be 1..9, got {leg}")
    if not 1 <= joint <= 9:
        raise ValueError(f"joint must be 1..9, got {joint}")
    return leg * 10 + joint


def leg_of(servo_id):
    """Return the leg number for a CRABORA-scheme servo ID (11 -> 1)."""
    return servo_id // 10


def joint_of(servo_id):
    """Return the joint number for a CRABORA-scheme servo ID (11 -> 1)."""
    return servo_id % 10


def describe_id(servo_id):
    """Human label, e.g. 23 -> 'leg 2 femur'. Falls back if not in scheme."""
    leg = leg_of(servo_id)
    joint = joint_of(servo_id)
    if 1 <= leg <= 9 and joint in JOINT_NAMES:
        return f"leg {leg} {JOINT_NAMES[joint]}"
    return f"servo {servo_id}"


# -----------------------------------------------------------------------------
# Physical leg layout (recorded 2026-07-05): leg numbers in CLOCKWISE order
# viewed from the top, starting at the front of the bot.
# -----------------------------------------------------------------------------
LEG_LAYOUT_CW = [1, 6, 3, 5, 2, 4]

# Alternating tripods -- every other position around the body. These are the
# stable groups for tripod gait (one tripod always planted) and the natural
# halves for power-budget staging.
TRIPOD_A = LEG_LAYOUT_CW[0::2]   # legs 1, 3, 2
TRIPOD_B = LEG_LAYOUT_CW[1::2]   # legs 6, 5, 4


def leg_bearing_deg(leg):
    """Direction the leg points, in degrees clockwise from the bot's front.

    leg 1 -> 0 (front), leg 6 -> 60, leg 3 -> 120, leg 5 -> 180 (rear), ...
    Raises ValueError for a leg not in the layout.
    """
    return LEG_LAYOUT_CW.index(leg) * 60.0


# =============================================================================
# Serial port discovery
# =============================================================================
def find_feetech_ports():
    """Return all /dev/tty.usb* paths that look like FE-URT-2 adapters, sorted.

    Returns a list; may be empty (no URTs), one element, or more.
    """
    candidates = sorted(glob.glob("/dev/tty.usb*"))
    if not candidates:
        candidates = sorted(glob.glob("/dev/tty.wchusbserial-*"))
    return candidates


def find_feetech_port():
    """Return the most likely single /dev/tty.usb* path for one FE-URT-2.

    Kept for backward compatibility with scripts that open a single bus.
    """
    candidates = find_feetech_ports()
    if not candidates:
        raise RuntimeError(
            "No /dev/tty.usb* device found. Is the FE-URT-2 plugged in? "
            "If you don't see one, you may need the WCH CH340 macOS driver "
            "from wch-ic.com."
        )
    if len(candidates) > 1:
        print(f"Multiple serial devices found: {candidates}")
        print(f"Using the first: {candidates[0]}")
    return candidates[0]


def find_servo_port(servo_id, timeout=0.5):
    """Return the port of whichever visible URT the servo answers on.

    No leg-to-URT mapping assumed: ping the servo on every FE-URT-2 the
    Mac can see and return the port that replies, or None if the servo
    answered nowhere.
    """
    for port in find_feetech_ports():
        with Bus(port=port, timeout=timeout) as probe:
            if probe.ping(servo_id):
                return port
    return None



# =============================================================================
# Packet construction
# =============================================================================
def checksum(packet_body):
    """Feetech checksum: bitwise NOT of the sum of all bytes after the header."""
    return (~sum(packet_body)) & 0xFF


def build_packet(servo_id, instruction, params=b""):
    """Build a full Feetech packet ready for the bus."""
    length = len(params) + 2  # instruction + params + checksum
    body = bytes([servo_id, length, instruction]) + params
    return HEADER + body + bytes([checksum(body)])


# =============================================================================
# Speed unit conversion
# =============================================================================
def rpm_to_raw(rpm):
    """Convert RPM to the wheel-mode goal-speed register (sign-magnitude).

    Used by spin.py-style wheel-mode commands, where direction is encoded
    in the sign bit.
    """
    magnitude = round(abs(rpm) * STEPS_PER_REV / 60.0)
    magnitude = min(magnitude, MAX_SPEED_MAGNITUDE)
    return (magnitude | SIGN_BIT) if rpm < 0 else magnitude


def raw_to_rpm(raw):
    """Inverse of rpm_to_raw."""
    magnitude = raw & MAX_SPEED_MAGNITUDE
    rpm = magnitude * 60.0 / STEPS_PER_REV
    return -rpm if (raw & SIGN_BIT) else rpm


def rpm_to_pos_speed(rpm):
    """Convert RPM to the position-mode goal-speed register (unsigned).

    Position-mode goal speed is unsigned. The catch: a value of 0 means
    "go at max speed" -- almost certainly NOT what you wanted -- so this
    floors to 1. Used by walk.py-style speed-controlled position moves.
    """
    raw = round(abs(rpm) * STEPS_PER_REV / 60.0)
    return max(1, min(raw, MAX_SPEED_MAGNITUDE))


# =============================================================================
# Bus class -- owns the serial port and offers every operation
# =============================================================================
class Bus:
    """A connection to the Feetech bus through the FE-URT-2.

    Use as a context manager so the port is always closed:

        with Bus() as bus:
            if bus.ping(11):
                bus.enable_torque(11)
                bus.write_goal_move(11, CENTER_POSITION, rpm_to_pos_speed(15))

    Or manage explicitly with .open() / .close().
    """

    def __init__(self, port=None, baudrate=BAUDRATE, timeout=0.5):
        self.port_path = port  # None = auto-discover on open
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    # ----- lifecycle -------------------------------------------------------
    def open(self):
        if self.ser is not None:
            return self
        if self.port_path is None:
            self.port_path = find_feetech_port()
        self.ser = serial.Serial(self.port_path, self.baudrate, timeout=self.timeout)
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

    # ----- low-level I/O ---------------------------------------------------
    def _send_and_receive(self, servo_id, packet, expected_params=0):
        """Send a packet, read the reply, return its parameter bytes.

        Reply framing: 0xFF 0xFF <id> <length> <error> <params...> <checksum>
        Raises IOError on short or malformed replies; prints a warning if
        the servo's error byte is non-zero but still returns the params.

        `servo_id` is passed in (rather than parsed back out of `packet`) so
        every error message can name the specific servo that's misbehaving
        -- "servo 23 (leg 2 tibia)" beats "some servo somewhere".
        """
        if self.ser is None:
            raise IOError("Bus is not open. Call .open() or use 'with Bus()'.")
        label = describe_id(servo_id)
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        # Reply length: 2 header + 1 id + 1 length + 1 error + N params + 1 checksum
        reply_len = 6 + expected_params
        reply = self.ser.read(reply_len)
        if len(reply) < reply_len:
            raise IOError(
                f"Short reply from ID {servo_id} ({label}): "
                f"{len(reply)}/{reply_len} bytes. "
                f"Bus power on? Servo connected? Got: {reply.hex()}"
            )
        if reply[:2] != HEADER:
            raise IOError(
                f"Bad reply header from ID {servo_id} ({label}): {reply.hex()}"
            )
        # Sanity-check the ID field of the reply too -- if it doesn't match,
        # we may have caught crosstalk from a different servo.
        reply_id = reply[2]
        if reply_id != servo_id:
            raise IOError(
                f"Reply ID mismatch: sent to {servo_id} ({label}), "
                f"got reply from {reply_id} ({describe_id(reply_id)}). "
                f"Bus contention? Stale bytes in the buffer? Raw: {reply.hex()}"
            )
        error_byte = reply[4]
        if error_byte != 0:
            decoded = decode_error_byte(error_byte)
            print(
                f"  ! ID {servo_id} ({label}) reported error byte "
                f"0x{error_byte:02x} [{decoded}]"
            )
        return reply[5 : 5 + expected_params]

    def _send_no_reply(self, packet):
        """Send a packet that will not be answered (broadcast / sync write)."""
        if self.ser is None:
            raise IOError("Bus is not open. Call .open() or use 'with Bus()'.")
        self.ser.reset_input_buffer()
        self.ser.write(packet)

    # ----- ping & reads ----------------------------------------------------
    def ping(self, servo_id):
        """True if a servo answers at this ID."""
        packet = build_packet(servo_id, INST_PING)
        try:
            self._send_and_receive(servo_id, packet, expected_params=0)
            return True
        except IOError:
            return False

    def read_bytes(self, servo_id, address, n):
        """Read n raw bytes from a register block."""
        packet = build_packet(servo_id, INST_READ, bytes([address, n]))
        return self._send_and_receive(servo_id, packet, expected_params=n)

    def read_uint8(self, servo_id, address):
        return self.read_bytes(servo_id, address, 1)[0]

    def read_uint16(self, servo_id, address):
        data = self.read_bytes(servo_id, address, 2)
        return data[0] | (data[1] << 8)

    # ----- writes (single servo) ------------------------------------------
    def write_bytes(self, servo_id, address, data):
        """Write raw bytes to a register block."""
        params = bytes([address]) + bytes(data)
        packet = build_packet(servo_id, INST_WRITE, params)
        self._send_and_receive(servo_id, packet, expected_params=0)

    def write_uint8(self, servo_id, address, value):
        self.write_bytes(servo_id, address, bytes([value & 0xFF]))

    def write_uint16(self, servo_id, address, value):
        self.write_bytes(
            servo_id, address,
            bytes([value & 0xFF, (value >> 8) & 0xFF]),
        )

    # ----- sync write (THE new capability) ---------------------------------
    def sync_write(self, address, payloads):
        """Write the same register block on many servos in one bus packet.

        payloads is a dict {servo_id: bytes}. Every value must be the same
        length (the bus protocol writes a fixed-size block per servo). No
        reply is sent -- this is a broadcast.

        Frame format (Feetech instruction 0x83):
            0xFF 0xFF 0xFE <length> 0x83 <addr> <data_len>
            <id1> <d1...dn>  <id2> <d1...dn>  ...  <checksum>

        Where length = N*(data_len + 1) + 4.
        """
        if not payloads:
            return
        sizes = {len(v) for v in payloads.values()}
        if len(sizes) != 1:
            raise ValueError(
                f"sync_write payloads must all be the same length, got {sizes}"
            )
        data_len = sizes.pop()

        params = bytearray([address, data_len])
        for servo_id, data in payloads.items():
            if not 0 <= servo_id <= MAX_VALID_ID:
                raise ValueError(f"servo_id {servo_id} out of range")
            params.append(servo_id)
            params.extend(data)

        packet = build_packet(BROADCAST_ID, INST_SYNC_WRITE, bytes(params))
        self._send_no_reply(packet)

    # ----- higher-level conveniences --------------------------------------
    def enable_torque(self, servo_id, on=True):
        """Drive the servo (on=True) or let it go limp (on=False)."""
        self.write_uint8(servo_id, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def sync_enable_torque(self, ids, on=True):
        """enable_torque for many servos in one packet."""
        value = bytes([1 if on else 0])
        self.sync_write(ADDR_TORQUE_ENABLE, {i: value for i in ids})

    def write_goal_move(self, servo_id, position, pos_speed_raw):
        """Speed-controlled position move.

        Writes Goal Position (42-43), Goal Time (44-45 = 0), and Goal Speed
        (46-47) as one contiguous block. This matters: if speed is not
        re-sent with every goal-position write, the servo reverts to full
        speed (the "speed doesn't vary" bug walk.py warns about). Goal
        Time is held at 0 so the move stays speed-controlled.

        pos_speed_raw is the unsigned register value -- use rpm_to_pos_speed
        to convert from RPM.
        """
        data = bytes([
            position & 0xFF, (position >> 8) & 0xFF,            # 42-43
            0x00, 0x00,                                         # 44-45 Goal Time
            pos_speed_raw & 0xFF, (pos_speed_raw >> 8) & 0xFF,  # 46-47 Goal Speed
        ])
        self.write_bytes(servo_id, ADDR_GOAL_POSITION, data)

    def sync_goal_move(self, targets):
        """Speed-controlled position move for many servos in one packet.

        targets is a dict {servo_id: (position, pos_speed_raw)}. Every
        servo gets a fresh Goal Speed alongside its Goal Position, so the
        single-write bug is avoided per-servo.
        """
        payloads = {}
        for servo_id, (position, speed) in targets.items():
            payloads[servo_id] = bytes([
                position & 0xFF, (position >> 8) & 0xFF,
                0x00, 0x00,
                speed & 0xFF, (speed >> 8) & 0xFF,
            ])
        self.sync_write(ADDR_GOAL_POSITION, payloads)

    # ----- telemetry -------------------------------------------------------
    def read_telemetry(self, servo_ids):
        """Read position, voltage, and temperature for each servo in servo_ids.

        Returns a dict {servo_id: {"position": int, "voltage_v": float, "temp_c": int}}.
        Voltage is converted from 0.1V raw units to volts.
        Skips servos that fail to respond (IOError) rather than raising.
        """
        result = {}
        for sid in servo_ids:
            try:
                position  = self.read_uint16(sid, ADDR_PRESENT_POSITION)
                voltage_v = self.read_uint8(sid, ADDR_PRESENT_VOLTAGE) / 10.0
                temp_c    = self.read_uint8(sid, ADDR_PRESENT_TEMP)
                result[sid] = {"position": position, "voltage_v": voltage_v, "temp_c": temp_c}
            except IOError:
                pass
        return result

    # ----- arrival polling -------------------------------------------------
    def wait_until_stopped(self, servo_id, timeout):
        """Block until ADDR_MOVING reads 0. Returns True if it did, else False."""
        time.sleep(0.1)  # give the move a moment to actually start
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self.read_uint8(servo_id, ADDR_MOVING) == 0:
                    return True
            except IOError:
                pass  # transient read glitch; keep polling
            time.sleep(0.05)
        return False

    def sync_wait_until_stopped(self, ids, timeout):
        """Block until ALL given servos report stopped. Returns True / False."""
        time.sleep(0.1)
        remaining = set(ids)
        deadline = time.time() + timeout
        while time.time() < deadline and remaining:
            for servo_id in list(remaining):
                try:
                    if self.read_uint8(servo_id, ADDR_MOVING) == 0:
                        remaining.discard(servo_id)
                except IOError:
                    pass
            if not remaining:
                return True
            time.sleep(0.05)
        return not remaining

    # ----- EEPROM lock dance ----------------------------------------------
    @contextlib.contextmanager
    def eeprom_unlocked(self, servo_id):
        """Context manager: unlock EEPROM, run the block, re-lock.

        Use around writes to the locked EEPROM area (ID, mode, baud, etc.):

            with bus.eeprom_unlocked(servo_id):
                bus.write_uint8(servo_id, ADDR_MODE, MODE_WHEEL)
        """
        self.write_uint8(servo_id, ADDR_LOCK, 0)
        try:
            yield
        finally:
            # Best-effort re-lock; don't mask a real exception with a relock error.
            try:
                self.write_uint8(servo_id, ADDR_LOCK, 1)
            except IOError:
                pass


# =============================================================================
# MultiBus -- N-URT router, drop-in replacement for Bus
# =============================================================================
# IDs the discovery scan tries: every (leg, joint) in the CRABORA 2-digit
# scheme -- legs 1..9, joints coxa/femur/tibia.
DISCOVERY_IDS = [make_id(leg, joint)
                 for leg in range(1, 10)
                 for joint in (COXA, FEMUR, TIBIA)]

# Read timeout used only while discovering. A live servo at 1 Mbaud answers
# in a few ms, so 0.05 s is generous; it's what keeps the scan of 27 IDs on
# each URT down to ~1 s instead of ~13 s at the driver's 0.5 s default.
DISCOVERY_TIMEOUT = 0.05


class MultiBus:
    """Routes commands to N Bus instances by discovering servos at open().

    Opens every FE-URT-2 the Mac can see (one, two, three, ...) and pings
    all CRABORA-scheme IDs on each bus to learn which servo lives where.
    No static leg-to-URT mapping: plug legs into whichever URT has a free
    connector and the routing follows.

        with MultiBus() as mb:
            legs = mb.legs()          # {leg_number: [ids, sorted by joint]}
            mb.sync_enable_torque(mb.live_ids, on=True)
            mb.sync_goal_move({11: (...), 31: (...)})

    Pass ports explicitly to restrict which URTs are used:

        MultiBus(ports=["/dev/tty.usbmodem5B790339841"])
    """

    def __init__(self, ports=None, baudrate=BAUDRATE, timeout=0.5):
        self._explicit_ports = ports
        self._baudrate = baudrate
        self._timeout = timeout
        self.buses = []           # populated in open()
        self._servo_to_bus = {}   # {servo_id: Bus}, built by _discover()

    # ----- lifecycle -------------------------------------------------------
    def open(self):
        if self.buses:
            return self
        ports = self._explicit_ports or find_feetech_ports()
        if not ports:
            raise RuntimeError(
                "No FE-URT-2 found. Is it plugged in? "
                "(urt_scan.py diagnoses USB-level problems.)"
            )
        self.buses = [
            Bus(port=p, baudrate=self._baudrate, timeout=self._timeout).open()
            for p in ports
        ]
        self._discover()
        print(f"MultiBus opened ({len(self.buses)} URT(s)):")
        for bus in self.buses:
            ids = sorted(s for s, b in self._servo_to_bus.items() if b is bus)
            print(f"  {bus.port_path}: "
                  f"{ids if ids else 'no servos answered'}")
        return self

    def _discover(self):
        """Ping every CRABORA-scheme ID on every bus; build servo->bus routing."""
        self._servo_to_bus = {}
        for bus in self.buses:
            saved_timeout = bus.ser.timeout
            bus.ser.timeout = DISCOVERY_TIMEOUT
            try:
                for sid in DISCOVERY_IDS:
                    if not bus.ping(sid):
                        continue
                    if sid in self._servo_to_bus:
                        print(f"  ⚠  servo {sid} ({describe_id(sid)}) answers "
                              f"on BOTH {self._servo_to_bus[sid].port_path} "
                              f"and {bus.port_path}; using the first.")
                        continue
                    self._servo_to_bus[sid] = bus
            finally:
                bus.ser.timeout = saved_timeout

    def close(self):
        for bus in self.buses:
            bus.close()
        self.buses = []
        self._servo_to_bus = {}

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # ----- discovery results ------------------------------------------------
    @property
    def live_ids(self):
        """Sorted list of every servo ID that answered during discovery."""
        return sorted(self._servo_to_bus)

    def legs(self):
        """{leg_number: [servo ids, sorted by joint]} for discovered servos.

        A leg appears if ANY of its joints answered; callers that need a
        complete coxa/femur/tibia set should check len == 3 themselves.
        """
        out = {}
        for sid in self.live_ids:
            out.setdefault(leg_of(sid), []).append(sid)
        return out

    # ----- routing helper --------------------------------------------------
    def _bus_for(self, servo_id):
        """Return the Bus instance responsible for this servo_id."""
        bus = self._servo_to_bus.get(servo_id)
        if bus is None:
            raise ValueError(
                f"Servo {servo_id} ({describe_id(servo_id)}) was not found on "
                f"any URT during discovery. Live IDs: {self.live_ids}. "
                "Check its power/cable and reopen the bus."
            )
        return bus

    def _split_by_bus(self, servo_ids):
        """Return a list of (bus, [ids_for_that_bus]) pairs, one per bus used."""
        groups = {}
        for sid in servo_ids:
            bus = self._bus_for(sid)
            groups.setdefault(id(bus), (bus, []))[1].append(sid)
        return list(groups.values())

    # ----- single-servo pass-throughs --------------------------------------
    def ping(self, servo_id):
        bus = self._servo_to_bus.get(servo_id)
        if bus is not None:
            return bus.ping(servo_id)
        # Not seen at discovery -- maybe powered up since open(). Try every
        # bus and remember where it answered so later commands route right.
        for bus in self.buses:
            if bus.ping(servo_id):
                self._servo_to_bus[servo_id] = bus
                return True
        return False

    def read_bytes(self, servo_id, address, n):
        return self._bus_for(servo_id).read_bytes(servo_id, address, n)

    def read_uint8(self, servo_id, address):
        return self._bus_for(servo_id).read_uint8(servo_id, address)

    def read_uint16(self, servo_id, address):
        return self._bus_for(servo_id).read_uint16(servo_id, address)

    def write_bytes(self, servo_id, address, data):
        self._bus_for(servo_id).write_bytes(servo_id, address, data)

    def write_uint8(self, servo_id, address, value):
        self._bus_for(servo_id).write_uint8(servo_id, address, value)

    def write_uint16(self, servo_id, address, value):
        self._bus_for(servo_id).write_uint16(servo_id, address, value)

    def enable_torque(self, servo_id, on=True):
        self._bus_for(servo_id).enable_torque(servo_id, on)

    def write_goal_move(self, servo_id, position, pos_speed_raw):
        self._bus_for(servo_id).write_goal_move(servo_id, position, pos_speed_raw)

    def wait_until_stopped(self, servo_id, timeout):
        return self._bus_for(servo_id).wait_until_stopped(servo_id, timeout)

    @contextlib.contextmanager
    def eeprom_unlocked(self, servo_id):
        with self._bus_for(servo_id).eeprom_unlocked(servo_id):
            yield

    # ----- multi-servo operations (split across buses) --------------------
    def sync_enable_torque(self, ids, on=True):
        for bus, group in self._split_by_bus(ids):
            bus.sync_enable_torque(group, on)

    def sync_write(self, address, payloads):
        groups = {}
        for sid, data in payloads.items():
            bus = self._bus_for(sid)
            groups.setdefault(id(bus), (bus, {}))[1][sid] = data
        for _, (bus, sub_payloads) in groups.items():
            bus.sync_write(address, sub_payloads)

    def sync_goal_move(self, targets):
        groups = {}
        for sid, target in targets.items():
            bus = self._bus_for(sid)
            groups.setdefault(id(bus), (bus, {}))[1][sid] = target
        for _, (bus, sub_targets) in groups.items():
            bus.sync_goal_move(sub_targets)

    def sync_wait_until_stopped(self, ids, timeout):
        results = []
        for bus, group in self._split_by_bus(ids):
            results.append(bus.sync_wait_until_stopped(group, timeout))
        return all(results)

    def read_telemetry(self, servo_ids):
        result = {}
        for bus, group in self._split_by_bus(servo_ids):
            result.update(bus.read_telemetry(group))
        return result


# =============================================================================
# Standalone: scan the bus for live servos
# =============================================================================
def scan(low=1, high=30):
    """Ping every ID from `low` to `high` inclusive; print which respond."""
    print("=" * 60)
    print(f"CRABORA bus scan  (IDs {low}..{high})")
    print("=" * 60)
    with Bus() as bus:
        print(f"Opened {bus.port_path} at {BAUDRATE} baud.\n")
        alive = []
        for servo_id in range(low, high + 1):
            if bus.ping(servo_id):
                alive.append(servo_id)
                print(f"  ✓ {servo_id:3d}   {describe_id(servo_id)}")
    print()
    if alive:
        print(f"Found {len(alive)} servo(s): {alive}")
    else:
        print("No servos responded.")
        print("  - 7.4V bus power on?")
        print("  - Servo cable seated at both ends?")
        print("  - Right ID range? Try scan(1, 253) to be exhaustive.")
    return alive


if __name__ == "__main__":
    scan()
