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
     and drops torque on everything. It is the one write this module
     offers, and it is deliberately not something the agent can call --
     it belongs to the human, on a button.

Nothing here decides anything. Policy -- what an agent may do, with what
arguments -- lives in tools.py, above this.
"""

import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "walk"))

from gait_lib import (  # noqa: E402
    ALL_LEGS, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, JOINT_NAME,
    TIBIA_JOINT_DIGIT, ADDR_TORQUE_ENABLE,
    BusError, PacketHandler,
    find_joints, find_urt_devices, open_ports, read1, write1,
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

    # --- the one write this module offers ----------------------------
    def emergency_stop(self):
        """Drop torque on everything. Belongs to the human, not the agent.

        Returns {"released": [...], "failed": [...], "missing": [...]}.
        Verifies rather than assumes: a servo that ignored the write is
        exactly what the operator needs told.
        """
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
        loc = self._a.locate(sid)
        if loc is None:
            raise ArbiterError(f"servo {sid} is not on the bus")
        from gait_lib import read2 as _read2
        return _read2(self._a._packet, loc[1], sid, addr, what)


def describe(sid):
    leg, joint = sid // 10, sid % 10
    if 1 <= leg <= 6 and joint in JOINT_NAME:
        return f"leg {leg} {JOINT_NAME[joint]}"
    return f"servo {sid}"
