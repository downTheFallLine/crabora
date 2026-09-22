# Bring-up: getting Crabora off the ground

This is the checklist for taking a freshly-assembled (or freshly-repaired)
Crabora from "parts on a bench" to "standing and walking," in an order that
catches problems at the cheapest possible stage. Each step should pass
before moving to the next one -- don't skip ahead just because a later step
"probably" works.

Old bench-era scripts (Mac + USB FE-URT-2, no onboard Pi) live in
`software.old/` for reference while the new `software/` test suite below
gets built out. See `doc/servoNotes.md` for the servo/URT wiring and
`doc/uniBody.md` for how the Pi, battery, BEC, and URTs bolt to the deck.

## Power safety notes (read first)

- The bench PSU is a 5A supply -- that is the binding constraint during
  dev/test. Don't expect to run all 6 legs under full load off it.
- Never route servo bus power through the URT from the Pi's own supply --
  power goes into each FE-URT-2 directly; the URT just carries it
  downstream to its two legs' coxa cables.
- STS3215 servos stall at ~2.5A and trip an over-current condition above
  ~2A sustained for 2s -- if a joint binds mechanically, expect the servo
  (or the supply) to protect itself rather than cook something.

## Test sequence

### 1. Pi stability burn-in -- `software/tests/test_01_pi_stability.py` (done)

Run this first, on the Pi itself, before anything else. If the board
under-volts or thermal-throttles under load, every test after this one is
suspect -- a flaky brain will produce flaky everything downstream, and
you'll waste time debugging gait code for what's actually a power problem.

```bash
python3 software/tests/test_01_pi_stability.py --duration 120
```

No pip installs required (stdlib only), so it runs on a bare Raspberry Pi
OS image. It loads all cores for the given duration and watches
`vcgencmd get_throttled`, CPU temp, and CPU clock once a second. Exits 0
(PASS) if the board never under-volted or throttled; exits 1 (FAIL) with
the specific reason otherwise (see the script's docstring for what each
`vcgencmd` bit means).

**Common failure**: under-voltage under load almost always means the power
cable/connector into the Pi, not the Pi itself -- check gauge and
connector quality before suspecting the board or the BEC.

**If this fails**: fix the power delivery to the Pi and re-run before
touching anything else.

### 2. FE-URT-2 USB visibility -- `software/tests/test_02_urt_usb_visibility.py` (done)

Confirms the Pi's USB/OS layer can actually see and claim the FE-URT-2
board(s) before test 3 tries to speak Feetech protocol through them. If
this fails, the problem is USB/driver/permissions (cable, hub, dialout
group); if this passes but test 3 doesn't, the problem is downstream of
the URT (servo wiring/power/IDs) -- this split is what makes the two
tests worth keeping separate.

```bash
python3 software/tests/test_02_urt_usb_visibility.py
python3 software/tests/test_02_urt_usb_visibility.py --expected 1   # one URT at a time
```

Needs `pyserial` (`python3 -m pip install pyserial`, or
`sudo apt install python3-serial` on Raspberry Pi OS) -- the script
checks for it and tells you if it's missing rather than crashing.
Matches each board by its WCH USB vendor ID (0x1A86), falling back to a
`/dev/ttyUSB*` glob on headless images where VID/PID isn't exposed, then
opens each one at the bus baud rate (1 Mbps) to prove it's actually
usable, not just enumerated. Defaults to expecting 3 boards (the full
6-leg wiring in `doc/servoNotes.md`); a count mismatch only warns, since
you may be bringing URTs up one at a time. FAILs only on zero boards
found, or a board that enumerates but won't open.

**If this fails**: check `ls /dev/ttyUSB*`, `lsusb | grep -i 1a86`, and
that the Pi's user is in the `dialout` group, before suspecting the URT
board itself.

### 3. Servo bus discovery -- planned

Port `software.old/scan_ids.py` forward: every URT the Pi can see should
enumerate the expected servo IDs (see `doc/servoNotes.md`'s ID scheme --
digit 1 = leg, digit 2 = joint) with no missing or duplicate IDs.

**If this fails**: check bus wiring/daisy-chaining before assuming a dead
servo.

### 4. Per-servo range-of-motion check -- planned

Port `software.old/set_middle.py` / `set_limits.py` forward: every joint
sweeps its full commanded range without stalling, binding, or tripping
over-current, and returns to center cleanly.

**If this fails**: check for mechanical interference (printed part
clearance, cable snag) before assuming a bad servo.

### 5. Single-leg articulation test -- planned

One leg at a time, coxa -> femur -> tibia, through a few representative
poses (folded/tucked, standing height, full extension). Confirms the
kinematics for that leg are sane in isolation before combining all six.

### 6. Full stand-up sequence -- planned

Port `software.old/stand.py` (3-phase stand-up) forward. All six legs
together, off the bench, weight-bearing. This is the first test where
the 5A PSU constraint actually bites -- watch for brownout under
simultaneous load.

### 7. Tripod gait walk test -- planned

Port `software.old/tripod_walk.py` forward. First actual steps. Confirm
coxa swing direction and gait timing on hardware (this was still
unverified as of the last hardware session per `software.old/`'s gait
scripts).

## Notes for porting old scripts forward

`software.old/urt_lib.py` has already been ported forward unchanged to
`software/urt_lib.py` -- it was written Pi-first (WCH VID match with a
`/dev` glob fallback for headless images) and test 2 above just wraps it,
so no rewrite was needed.

The old `software.old/crabora_bus.py` `MultiBus` class already
auto-discovers which servo lives on which URT at runtime -- that logic is
worth carrying into the new `software/` tree largely as-is rather than
rewriting, per `doc/servoNotes.md`. The main change moving from bench to
onboard-Pi operation is that the bus is now local to the robot instead of
tethered over USB to a Mac.
