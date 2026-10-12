"""
safety -- the arming gate between the model and the servos.

ONE RULE: the agent cannot arm itself. Arming comes from a human
pressing a button, over an HTTP route the tool layer cannot reach. The
model can ask to move; it cannot grant itself permission to.

This is a time window, not a latch. Arming expires, because the failure
mode worth designing against is not a model that goes berserk, it is a
robot left armed on a bench overnight that lurches when someone later
asks it an innocent question. Motion refreshes the window, so a working
session stays fluid; silence closes it.

Arming is also dropped, immediately and without waiting for the window,
whenever something says the robot is not in a state to be trusted: an
emergency stop, a latched fault, the bus going away. Re-arming is a
human action every time.

Nothing here touches the bus. It answers one question -- may motion
happen right now -- and records why not.
"""

import threading
import time

# How long an arm lasts with nothing happening. Long enough to think
# about what to ask for, short enough that walking away ends it.
DEFAULT_WINDOW = 300.0

# Ceiling on one continuous armed stretch, however much motion refreshes
# it. A session that has been live for half an hour should be a decision,
# not a side effect of staying busy.
MAX_SESSION = 1800.0


class NotArmed(RuntimeError):
    """Motion was attempted while disarmed. Carries why, for the UI."""


class Safety:
    def __init__(self, window=DEFAULT_WINDOW, max_session=MAX_SESSION):
        self._lock = threading.Lock()
        self._window = window
        self._max_session = max_session
        self._until = 0.0          # monotonic deadline
        self._started = 0.0        # when this armed stretch began
        self._by = None
        self._why_not = "never armed"

    # --- human side --------------------------------------------------
    def arm(self, who="operator", window=None):
        """Open the window. Only ever called from the human-facing route."""
        with self._lock:
            now = time.monotonic()
            self._window = window or self._window
            self._started = now
            self._until = now + self._window
            self._by = who
            self._why_not = None
            return self._state_locked(now)

    def disarm(self, why="disarmed"):
        with self._lock:
            self._until = 0.0
            self._started = 0.0
            self._by = None
            self._why_not = why
            return self._state_locked(time.monotonic())

    # --- motion side -------------------------------------------------
    def require(self):
        """Raise NotArmed unless motion is permitted right now."""
        with self._lock:
            now = time.monotonic()
            if self._until <= now:
                raise NotArmed(self._why_not or
                               "not armed -- press ARM on the control strip")
            if now - self._started > self._max_session:
                self._until = 0.0
                self._why_not = (f"armed for over {self._max_session/60:.0f} "
                                 f"minutes -- press ARM again")
                raise NotArmed(self._why_not)
            return True

    def touch(self):
        """Refresh the window after a successful motion. Never extends
        past the session ceiling -- require() enforces that separately."""
        with self._lock:
            if self._until > time.monotonic():
                self._until = time.monotonic() + self._window

    # --- reporting ---------------------------------------------------
    def state(self):
        with self._lock:
            return self._state_locked(time.monotonic())

    def _state_locked(self, now):
        armed = self._until > now
        return {
            "armed": armed,
            "seconds_left": round(max(0.0, self._until - now)) if armed else 0,
            "armed_by": self._by if armed else None,
            "reason": None if armed else (self._why_not or "not armed"),
            "window_seconds": round(self._window),
        }
