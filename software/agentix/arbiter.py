"""
arbiter -- the only thing allowed to touch the servo bus.

Everything above this (tools, the agent, the web API) goes through here,
and nothing bypasses it. It exists for three reasons:

  1. ONE AT A TIME. The Feetech bus is half-duplex and shared. Two HTTP
     requests reading servos concurrently interleave their packets and
     both get garbage. A single lock makes that impossible rather than
     unlikely.

  2. PORTS STAY OPEN. Opening and scanning the URTs costs seconds. A
     chat UI that takes four seconds to answer "is anything faulted?"
     is a chat UI nobody uses.

  3. STOP ALWAYS WINS. `emergency_stop()` takes the lock with priority
     and drops torque on everything. It is deliberately not something
     the agent can call -- it belongs to the human, on a button.

Nothing here decides anything. Policy -- whether motion is permitted
(safety.py), what an agent may ask for and with what arguments
(tools.py) -- lives above this. The writes below are mechanism: they
will move whatever they are told to move, which is exactly why nothing
calls them without passing through the layers that say no.

The one thing they do enforce is the servo's own travel limits, read
from firmware and applied to every goal. That is not policy, it is the
last line of a defence that starts three layers up, and it costs one
clamp.
"""

import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, JOINT_NAME,
    TIBIA_JOINT_DIGIT, ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE,
    ADDR_PRESENT_POSITION, POSITION_TOLERANCE,
    BusError, PacketHandler,
    find_joints, find_urt_devices, open_ports, read1, read2, read_status,
    write1, write2,
)

JOINT_DIGITS = (COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, TIBIA_JOINT_DIGIT)

# How stale a cached joint map may get before it is rebuilt. Servos do
# not usually move between boards, but a reboot or a replugged URT
# should not need a restart of the server.
DISCOVERY_TTL = 60.0


class ArbiterError(RuntimeError):
    pass


class Busy(ArbiterError):
    """Something else is already using the bus."""


class Cancelled(ArbiterError):
    """A motion was interrupted by a stop request."""


class Arbiter:
    def __init__(self, ports=None, busy_timeout=8.0):
        self._lock = threading.RLock()
        self._requested_ports = ports
        self._ports = {}
        self._packet = None
        self._joints = {}
        self._discovered_at = 0.0
        self._busy_timeout = busy_timeout
        self._last_action = None
        # Set without the lock, on purpose. A long motion holds the bus,
        # so STOP cannot wait for it -- it raises this flag first and the
        # motion loop drops out at its next step, then STOP takes the bus
        # and releases torque. Checking a flag between steps is what makes
        # a 30-second dance interruptible at all.
        self._cancel = threading.Event()
        self.available = False
        self.reason = "not started"

    # --- lifecycle ---------------------------------------------------
    def start(self):
        """Open the URTs. Failure is recorded, not raised: the web app
        should still come up and say the bus is unavailable."""
        with self._lock:
            try:
                devices = self._requested_ports or find_urt_devices()
                if not devices:
                    self.available = False
                    self.reason = ("no FE-URT-2 boards found -- check USB and "
                                   "run tests/test_02_urt_usb_visibility.py")
                    return self
                self._ports = open_ports(devices)
                if not self._ports:
                    self.available = False
                    self.reason = "found URT boards but none would open"
                    return self
                self._packet = PacketHandler(0)  # STS uses protocol_end=0
                self.available = True
                self.reason = f"{len(self._ports)} board(s) open"
            except Exception as e:                   # noqa: BLE001
                self.available = False
                self.reason = f"{type(e).__name__}: {e}"
            return self

    def stop(self):
        with self._lock:
            for port in self._ports.values():
                try:
                    port.closePort()
                except Exception:                     # noqa: BLE001
                    pass
            self._ports = {}
            self.available = False
            self.reason = "stopped"

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # --- access ------------------------------------------------------
    def _acquire(self, what):
        if not self._lock.acquire(timeout=self._busy_timeout):
            raise Busy(f"the bus is busy ({self._last_action or 'unknown'}); "
                       f"gave up after {self._busy_timeout:g}s")
        self._last_action = what
        return True

    def try_positions(self, sids):
        """{sid: counts} if the bus is free RIGHT NOW, else None.

        Never waits. For the leg animation, which polls several times a
        second: blocking would queue it behind a 30-second motion, and
        taking the lock even briefly mid-motion would put reads between a
        gait's writes. When a motion holds the bus the caller falls back
        to the goals it has been writing (gait_lib.LAST_GOALS).
        """
        if not self.available or not self._lock.acquire(blocking=False):
            return None
        try:
            if not self._joints:
                return None             # not discovered yet; not worth it here
            out = {}
            for sid in sids:
                loc = self.locate(sid)
                if loc is None:
                    continue
                try:
                    out[sid] = read2(self._packet, loc[1], sid,
                                     ADDR_PRESENT_POSITION, "leg view")
                except BusError:
                    pass
            return out
        finally:
            self._lock.release()

    def try_currents(self):
        """Read every known servo's current if the bus is free RIGHT NOW;
        returns how many were read, or None if it was busy.

        Same never-wait rule as try_positions, for the power view. The
        readings land in gait_lib.LAST_CURRENT (read_status does that),
        which is also where a running motion leaves its own, so the
        caller reads one place either way.
        """
        if not self.available or not self._lock.acquire(blocking=False):
            return None
        try:
            n = 0
            for by_leg in (self._joints or {}).values():
                for sid, port in by_leg.values():
                    try:
                        read_status(self._packet, port, sid, "power view")
                        n += 1
                    except BusError:
                        pass
            return n
        finally:
            self._lock.release()

    def session(self, what="read"):
        """Context manager giving exclusive bus access.

            with arb.session("status") as s:
                pos = s.read2(33, ADDR_PRESENT_POSITION, "position")
        """
        return _Session(self, what)

    # --- discovery ---------------------------------------------------
    def joints(self, refresh=False):
        """{digit: {leg: (sid, port)}}, cached. Call inside a session."""
        now = time.monotonic()
        if (refresh or not self._joints
                or now - self._discovered_at > DISCOVERY_TTL):
            self._joints = {
                d: find_joints(self._packet, self._ports, list(ALL_LEGS), d,
                               quiet=True)
                for d in JOINT_DIGITS
            }
            self._discovered_at = now
        return self._joints

    def locate(self, sid):
        """(sid, port) for a servo ID, or None."""
        leg, joint = sid // 10, sid % 10
        return self.joints().get(joint, {}).get(leg)

    # --- cancellation -------------------------------------------------
    def request_stop(self):
        """Ask any running motion to stop. Takes no lock, so it works
        while the bus is held."""
        self._cancel.set()

    def clear_stop(self):
        self._cancel.clear()

    @property
    def stopping(self):
        return self._cancel.is_set()

    def check_cancel(self):
        """Call between motion steps. Raises if a stop was requested."""
        if self._cancel.is_set():
            raise Cancelled("stopped by the operator")

    # --- the one write this module offers ----------------------------
    def emergency_stop(self):
        """Drop torque on everything. Belongs to the human, not the agent.

        Returns {"released": [...], "failed": [...], "missing": [...]}.
        Verifies rather than assumes: a servo that ignored the write is
        exactly what the operator needs told.
        """
        self.request_stop()      # first: unblocks a motion holding the bus
        if not self.available:
            raise ArbiterError(f"bus unavailable: {self.reason}")
        released, failed, missing = [], [], []
        with self.session("EMERGENCY STOP"):
            for digit in JOINT_DIGITS:
                found = self.joints().get(digit, {})
                for leg in ALL_LEGS:
                    sid = leg * 10 + digit
                    if leg not in found:
                        missing.append(sid)
                        continue
                    _s, port = found[leg]
                    try:
                        write1(self._packet, port, sid, ADDR_TORQUE_ENABLE, 0,
                               "release")
                        if read1(self._packet, port, sid, ADDR_TORQUE_ENABLE,
                                 "verify") == 0:
                            released.append(sid)
                        else:
                            failed.append(sid)
                    except BusError:
                        failed.append(sid)
        self.clear_stop()        # the flag has done its job
        return {"released": sorted(released), "failed": sorted(failed),
                "missing": sorted(missing)}


class _Session:
    """Exclusive bus access. Thin wrappers so callers never see packet."""

    def __init__(self, arbiter, what):
        self._a = arbiter
        self._what = what

    def __enter__(self):
        if not self._a.available:
            raise ArbiterError(f"bus unavailable: {self._a.reason}")
        self._a._acquire(self._what)
        return self

    def __exit__(self, *exc):
        self._a._last_action = None
        self._a._lock.release()
        return False

    @property
    def packet(self):
        return self._a._packet

    def joints(self, refresh=False):
        return self._a.joints(refresh)

    def read1(self, sid, addr, what="read"):
        loc = self._a.locate(sid)
        if loc is None:
            raise ArbiterError(f"servo {sid} is not on the bus")
        return read1(self._a._packet, loc[1], sid, addr, what)

    def read2(self, sid, addr, what="read"):
        return read2(self._a._packet, self._port(sid), sid, addr, what)

    def _port(self, sid):
        loc = self._a.locate(sid)
        if loc is None:
            raise ArbiterError(f"servo {sid} is not on the bus")
        return loc[1]

    # --- writes ------------------------------------------------------
    # Mechanism, not permission. Whether any of this may happen at all is
    # decided in safety.py and tools.py, before anything gets here.

    def write1(self, sid, addr, value, what="write"):
        return write1(self._a._packet, self._port(sid), sid, addr, value, what)

    def write2(self, sid, addr, value, what="write"):
        return write2(self._a._packet, self._port(sid), sid, addr, value, what)

    def limits(self, sid):
        """(min, max) travel in counts, straight from servo firmware.

        These were set on hardware per joint, so they are the robot's own
        account of where it may go -- better than anything this process
        can assert, and they outlive any bug up here.
        """
        return (self.read2(sid, ADDR_MIN_ANGLE, "min angle"),
                self.read2(sid, ADDR_MAX_ANGLE, "max angle"))

    def torque(self, sids, on, speed=None):
        """Enable or drop torque. With speed, sets it before enabling."""
        done = []
        for sid in sids:
            if on and speed is not None:
                self.write2(sid, ADDR_GOAL_SPEED, int(speed), "speed")
            self.write1(sid, ADDR_TORQUE_ENABLE, 1 if on else 0, "torque")
            done.append(sid)
        return done

    def positions(self, sids):
        """{sid: counts}, skipping any servo that will not answer."""
        out = {}
        for sid in sids:
            try:
                out[sid] = self.read2(sid, ADDR_PRESENT_POSITION, "position")
            except (BusError, ArbiterError):
                pass
        return out

    def move(self, targets, speed=None, timeout=4.0,
             tolerance=POSITION_TOLERANCE):
        """Drive {sid: counts} and wait for arrival.

        Every target is clamped to that servo's firmware limits first, and
        the clamp is reported rather than silently applied -- a caller
        asking for somewhere the joint cannot go has a bug worth seeing.

        Returns {"reached": {...}, "short": {...}, "clamped": {...}}.
        'short' is not an error here: a loaded joint that stops early is
        information for the layer above, which knows what it meant.
        """
        clamped = {}
        goals = {}
        for sid, want in targets.items():
            lo, hi = self.limits(sid)
            if lo > hi:                      # unset or wheel mode
                lo, hi = 0, 4095
            got = max(lo, min(hi, int(want)))
            if got != int(want):
                clamped[sid] = {"asked": int(want), "allowed": got,
                                "limits": [lo, hi]}
            goals[sid] = got

        for sid, goal in goals.items():
            if speed is not None:
                self.write2(sid, ADDR_GOAL_SPEED, int(speed), "speed")
            self.write1(sid, ADDR_TORQUE_ENABLE, 1, "torque")
            self.write2(sid, ADDR_GOAL_POSITION, goal, "goal")

        deadline = time.monotonic() + timeout
        reached, short = {}, {}
        while time.monotonic() < deadline:
            here = self.positions(list(goals))
            short = {s: {"at": p, "wanted": goals[s]}
                     for s, p in here.items()
                     if abs(p - goals[s]) > tolerance}
            if not short:
                reached = here
                break
            time.sleep(0.05)
        else:
            reached = {s: p for s, p in self.positions(list(goals)).items()
                       if s not in short}

        return {"reached": reached, "short": short, "clamped": clamped}


def describe(sid):
    leg, joint = sid // 10, sid % 10
    if 1 <= leg <= 6 and joint in JOINT_NAME:
        return f"leg {leg} {JOINT_NAME[joint]}"
    return f"servo {sid}"
