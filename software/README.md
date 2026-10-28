# CRABORA software

Everything that runs on the Pi. Organised below by **what you are
trying to do**, not by directory, because that is the question you
actually have.

Run from this directory (`~/crabora/software`) with the venv active.

---

## If the robot is misbehaving

```bash
python3 utils/all_limp.py --yes
```

Drops torque on all 18 servos in one pass and **verifies each one
actually released**, naming any that did not. The robot will fall, so
support it first if it is standing.

`all_limp.sh` does the same thing one servo at a time with no
verification. It is slower and was missing leg 3 entirely until
2026-10-12. Prefer the Python one.

Single joint: `python3 utils/go_limp.py 33`

---

## Cold start: proving the robot from nothing

The numbered tests run in order and each assumes the one before it
passed. If something is wrong, the lowest-numbered failure is the one
to fix.

| | what it proves | moves? |
|---|---|---|
| `tests/test_01_pi_stability.py` | the Pi holds up under load without under-volting or throttling | no |
| `tests/test_02_urt_usb_visibility.py` | the FE-URT-2 boards enumerate and can be opened | no |
| `tests/test_03_ping_servo.py` | servos answer at the IDs you expect, each on exactly one board | no |
| `tests/test_04_coxa_sweep.py` | the coxas move through their travel without binding | **yes** |
| `tests/test_05_calibration.py` | all 18 joints are set up consistently with each other | no by default |
| `tests/test_06_imu.py` | the IMU is wired, reading, and mounted in line with the body frame | no |

Test 5 is the one that catches the faults that waste days — a leg whose
travel limits differ from its peers, a centre set to a stale
convention, a sign table that disagrees with the hardware. Run it
whenever something feels off.

```bash
python3 tests/test_05_calibration.py            # read-only
python3 tests/test_05_calibration.py --move     # also probes each joint
python3 tests/test_05_calibration.py --signs    # asks you which way things moved
```

---

## Setting a joint up

```bash
python3 utils/set_center.py <id> <left°> <right°>
```

Hand-position the joint, press Enter to record it as centre, and the
travel limits are written either side. Both live in servo EEPROM and
survive a power cycle, so the firmware clamps anything the code gets
wrong.

The conventions these depend on — what "centre" means for each joint,
and which way is positive — are in [walk/README.md](walk/README.md).
Read that before setting anything; the centres have been redone twice
already because the convention changed underneath them.

### Talking to one servo

`utils/servo.py` is the general-purpose tool. Registers are addressed by
NAME, so the next one-off need does not require a new script:

```bash
python3 utils/servo.py scan                    # what IDs are alive, where
python3 utils/servo.py info 33                 # the registers worth seeing
python3 utils/servo.py get 33 present_load
python3 utils/servo.py set 33 torque_limit 500
python3 utils/servo.py id 1 33                 # change an ID, safely
python3 utils/servo.py reset --all             # clear latched faults
python3 utils/servo.py regs                    # the register map
```

EEPROM registers are unlocked and re-locked around the write. Registers
that can make a servo unreachable (`id`, `baud`, `mode`, `lock`) need
`--force`. `id` refuses unless exactly one servo is on the bus, because
two servos sharing an ID are indistinguishable afterwards.

Movement: `utils/sweep_servo.py`, `utils/swing_arm.py <leg>`,
`utils/swing_legs.py <legs...>`. Health at a glance:
`utils/read_servo_voltage.py <id>`.

---

## Getting it standing

```bash
python3 stand/stand.py --phase survey   # what state is it in?
python3 stand/stand.py                  # survey, collapse, untangle, stand
```

Recovers from an arbitrary pose. Four phases, each confirmed before it
runs. A leg that cannot be freed is reported and skipped rather than
aborting the whole thing. See [stand/README.md](stand/README.md) for
the strategy and why each phase is shaped the way it is.

`stand/survey.py` alone reads all 18 joints and computes where each
foot actually is, without moving anything.

---

## Once it is standing

```bash
python3 turn/turn.py 90            # turn in place, clockwise
python3 dance/dance.py             # five moves
python3 scare/scare.py             # rear up on legs 2-5, wave 1 and 6
./dance/dance_forever.sh           # loop until Ctrl-C
```

`software/walk/` holds the gait work: `proto_walk_*.py` exercise one
joint type at a time, `tripod_coxa.py` and `tripod_coxa_femur.py` build
up the tripod gait. `walk.py` walks in any direction with the feet held
planted, stop-and-wait; `scurry.py` is the same gait streamed on a clock
for speed (see [walk/README.md](walk/README.md)).

---

## Shared code

| | |
|---|---|
| `walk/gait_lib.py` | geometry, link lengths, sign tables, bus plumbing. "Which way is forward" is defined here and nowhere else |
| `stand/pose.py` | leg IK, named poses, floor-strike checking. Pure geometry — run it directly for a self-check |
| `imu_lib.py` | MPU-6050 with calibration and heading |
| `urt_lib.py` | FE-URT-2 discovery |

Bus imports in `gait_lib` are optional, so the geometry is usable on a
machine with no servo SDK installed.

---

## Known open items

- **Power is unmeasured.** The LM2596 passes 2–3 A on paper. It now
  regulates 7.4 V from a 3S pack, and the robot stands and turns on it.
  The 20 A replacement buck arrived defective and is being returned. An
  inline watt-meter would settle whether the LM2596 is a ceiling or a
  non-issue before walking leans on it.
- **Walking speed.** `walk.py` walks; `scurry.py` should walk faster but
  has not yet run on hardware, and quick tripod reversals are where the
  unmeasured power path is most likely to show.

## Legacy

`../software.old/` holds the pre-2026-09 bench scripts. The ones with
successors were **deleted** on 2026-10-12 — they are in git history if
ever needed, and leaving two versions around is how you end up editing
the wrong one:

| deleted | use instead |
|---|---|
| `set_middle.py`, `set_limits.py`, `center_servo.py` | `utils/set_center.py` |
| `limp.py` | `utils/go_limp.py`, `utils/all_limp.py` |
| `scan_ids.py`, `set_id.py`, `reset_overload.py`, `servo_report.py`, `examine_servo.py` | `utils/servo.py` |

What remains there is genuinely legacy: `crabora_bus.py` (the old
driver, superseded by `walk/gait_lib.py`), old gait experiments, and
one-off diagnostics. Nothing in `software/` imports any of it.
