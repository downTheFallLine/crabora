# Test log

Dated hardware sessions, newest first. What was run, what it showed, and
what it changed. Next steps live in `Crabora-next-steps.xlsx`.

## 2026-11-24: stand, scurry, skew, power, speed

Robot: 3S pack, LM2596 servo buck (the 20 A replacement went back
defective), all six legs, agentix service running on the Pi.

### Stand and first logged scurries (desk)

| Run | Peak current | Lowest supply | Verdict |
|---|---|---|---|
| `stand.py --power-log` | 1.90 A | 7.1 V (rests 7.6) | HITTING THE WALL |
| Standing still afterwards | ~1.3 A | | ~65% of the LM2596's continuous rating |
| `scurry.py 100`, 3.0 s cycle | 2.14 A (mean 0.96) | 7.2 V | COPING |
| `scurry.py 100 --period 2.0` | 2.29 A (mean 1.07) | 7.2 V | COPING |

Femurs do the work in stance (22, 62, 32 hardest). The 2026-11-18 scurry
log that showed only 0.2 A total almost certainly had the legs unloaded.

To run a CLI motion program on the Pi, stop the service first
(`sudo systemctl stop crabora-agentix`) and start it again afterwards; it
holds the bus.

### The "45° left" skew

Walk and scurry drifted 45-60° left of the heading on the desk.

- IMU yaw over the runs was only 1-1.5°: the body slides sideways, it
  does not steer.
- `--heading 60` (60° right by the maths) went right, not straight, so it
  is not a clean one-leg (60°) offset in the geometry.
- Code checked: one `pose_counts()` for all legs (coxa = centre + deg,
  CCW positive), no per-servo zero offsets, `LEG_BEARING` = 30 + (n-1)*60.
- **Feet in the air (robot on a box): all six stance strokes ran straight
  back.** The gait kinematics are right; the skew is ground contact.
- On a grippy mat leg 6's foot caught and bent left, the body yawed 27°,
  tibias 43/33/13 stalled at ~2.7 A, and the supply sagged to 6.3 V.
  `--lift 30` cut the yaw to 11° but tibias still stalled.
- **Root cause found by Tom: leg 4's tibia screws were very loose.** Later,
  leg 3's femur screws were loose too. Walking vibration loosens horn
  screws; check all 18 now and then.

Doc bug spotted: `walk.py`'s help says `--heading 90` is left, but
`stance_pose()`'s `psi = -heading` makes +90 go right.

### Speed on the test stand (feet in the air)

Tracking, not power, limits speed in the air (under 0.5 A throughout).
The tibia is the binding joint: about 2,900-3,000 steps/s unloaded.

| Cycle | Speed | Tibia mean / max error | Lag |
|---|---|---|---|
| 1.5 s | 100 mm/s | 7.7° / 34° | ~190 ms |
| 1.2 s | 125 mm/s | 12.7° / 50° | ~245 ms |
| 1.0 s | 150 mm/s | 13.9° / 46° | ~170-240 ms |

**New `--lead` option** (commit 32ef5c3): sends each joint's pose early,
one value or three (coxa femur tibia); tracking is judged against where
the joint should be, not the early goal. At 1.5 s, `--lead 0.10 0.15 0.18`
cut mean error coxa 2.7 → 0.6°, femur 4.8 → 2.2°, tibia 8.0 → 4.1°. A
single 0.20 s lead overshoots the coxa.

**Dry-run sweep of the tripod** (peak joint speed at equal body speed):
heading 0 is already the best direction; duty 0.5 cuts swing speeds ~17%;
lift 15 eases the femur; foot-out 90-100 eases the tibia; a longer stride
hurts. Wave and ripple gaits were considered and rejected: Tom wants top
speed.

**New defaults** (commit c9b87d0): stride 110, lift 15, foot-out 90,
duty 0.5, lead 0.16/0.15/0.18. Default period stays 3.0 s; top speed is
`--period 0.7 --allow-fast`.

| Cycle | Cruise | Mean error coxa / femur / tibia |
|---|---|---|
| 1.0 s | 200 mm/s | 4.8 / 3.0 / 7.7° |
| 0.8 s | 250 mm/s | 7.3 / 3.6 / 7.7° |
| 0.7 s | 286 mm/s | 5.9 / 4.0 / 8.1° (coxa lead 0.16) |

At 286 mm/s the coxa and tibia are both near their ceilings; past that
means trading them with `--foot-out` (~95), faster servos, or longer legs.
The web UI's Scurry button now uses the default lead too.

### Rug (low pile), 400 mm, 3 s cycle

| Run | Got to | Peak | Lowest supply | Mean error c/f/t | Yaw | Latched |
|---|---|---|---|---|---|---|
| New defaults | 265 mm | 4.77 A | 6.3 V | 1.1 / 1.1 / 1.4° | -8° | 33 |
| Old settings | 267 mm | 4.53 A | 5.5 V | 1.6 / 2.1 / 2.8° | -13° | 33 |
| New defaults, leg 3 femur tightened | 344 mm | 5.20 A | 5.0 V | 1.0 / 1.1 / 1.9° | -23° | 43 |

Every run was stopped by the low-voltage guard. The new defaults are not
the cause (the old set fails the same way and tracks worse). It is a
runaway: the rug needs more than the LM2596 gives, the voltage sags, the
servos weaken, loaded tibias stall, current rises, the voltage sags more.
The guard reads the supply every 0.2 s, so it fell from 6.5 V to 5.0 V
before stopping. After each overload `servo.py reset <id>` cleared it and
`stand.py` recovered, sometimes needing a second try.

### End of session

Leg 4's tibia broke off. A 100 mm floor run on five feet stopped at
89 mm with the supply at **3.9 V while drawing only ~0.5 A**: not a
current limit, most likely a flat pack after a long day (the servos can't
see pack voltage now that the buck regulates) or a hot buck. Five servos
latched Overload (23, 32, 33, 53, 63). Left with servo power to be
switched off and the pack checked.

Also: the camera stopped delivering frames after the day's service
restarts (reader running, 0 frames, 16 timeouts, `/api/camera.jpg` 503).
