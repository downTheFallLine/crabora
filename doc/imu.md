# IMU bring-up: MPU-6050 on Ubuntu

Getting the IMU wired, talking, calibrated, and -- the part that actually
matters -- **mounted the right way round**.

This is written for **Ubuntu on the Pi**, which is what Crabora runs.
Most MPU-6050 guides assume Raspberry Pi OS and tell you to use
`raspi-config`; that does not exist here, and the boot config is in a
different place. The Ubuntu steps are below.

See `doc/bringUp.md` for where this sits in the overall sequence, and
`software/walk/README.md` for the body frame the orientation check uses.

---

## 1. Wire it

**Power the Pi down first.** Four jumpers to the 40-pin header:

| MPU-6050 | Pi pin | GPIO | note |
|---|---|---|---|
| VCC | **1** | 3V3 | **3.3 V, not 5 V** |
| GND | **9** | GND | nearest ground to the I²C pins |
| SDA | **3** | GPIO 2 | |
| SCL | **5** | GPIO 3 | |

**Why 3.3 V and not 5 V.** The breakout has pull-up resistors tying SDA
and SCL to whatever VCC is. Most boards have an onboard regulator and
will *run* from 5 V, but then the pull-ups put 5 V on the Pi's GPIO,
which is 3.3 V only. The module works fine on 3.3 V.

Ground can be any of pins 6, 9, 14, 20, 25, 30, 34 or 39 -- they are all
the same plane. Pin 9 just keeps the run tidy, since 1, 3, 5 and 9 are
all on the same row.

**Keep SDA/SCL short.** I²C on flying leads gets unreliable past about
20-30 cm, especially routed near the servo harness where several amps
are switching. If the address flickers or appears intermittently in
`i2cdetect`, suspect wire length and routing before the sensor.

---

## 2. Mount it the right way round

The orientation is the thing that silently ruins everything downstream.
A sensor rotated 90° answers perfectly and quietly corrupts every
heading the robot computes.

The board's axes must line up with the body frame:

```
  +X  forward, out through the hexagon point between legs 1 and 6
  +Y  to the left
  +Z  up
```

The axes are normally silkscreened on the breakout. Match them to the
deck before you glue or bolt anything down -- `test_06` will tell you if
they are wrong, but it is easier to get right the first time.

---

## 3. Enable I²C (Ubuntu, not Raspberry Pi OS)

There is no `raspi-config` on Ubuntu. Edit the boot config directly:

```bash
sudo nano /boot/firmware/config.txt
```

Add, under `[all]`:

```
dtparam=i2c_arm=on
```

Note the path: **`/boot/firmware/config.txt`** on Ubuntu, not
`/boot/config.txt` as most Pi guides say. Then:

```bash
sudo reboot
```

After it comes back, confirm the bus exists:

```bash
ls -l /dev/i2c-*
```

You want `/dev/i2c-1`. If there is nothing, the dtparam line did not
take -- check you edited the file under `/boot/firmware/`.

---

## 4. Tools and permissions

```bash
sudo apt update
sudo apt install -y i2c-tools
sudo usermod -aG i2c $USER
```

The group change needs a **log out and back in** (or a reboot) before
your shell sees it. Until then `i2cdetect` needs `sudo`.

Python library, inside the project venv -- Ubuntu is PEP 668
externally-managed, so a bare `pip install` into the system Python is
refused:

```bash
cd ~/crabora/software
source .venv/bin/activate
pip install smbus2
```

---

## 5. Find the chip

```bash
i2cdetect -y 1
```

You are looking for **`68`** in the grid:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

| what you see | what it means |
|---|---|
| `68` | correct, carry on |
| `69` | AD0 is pulled high -- pass `--address 0x69`, or ground AD0 |
| nothing | wiring. Stay here until it appears; nothing downstream will work |
| `UU` | something else has claimed it (a kernel driver) |

If nothing shows: check VCC is on pin 1 and not pin 2 (5 V), check SDA
and SCL are not swapped, and check the jumper crimps. A multimeter on
VCC-to-GND at the module is worth thirty seconds.

---

## 6. Prove it is mounted correctly

Put the robot **still, on a level floor**, then:

```bash
cd ~/crabora/software
python3 tests/test_06_imu.py
```

The test uses gravity as a known reference. Sitting level and still, the
accelerometer must read about **+1 g on +Z** and about 0 on X and Y.
Anything else means the mount is rotated, and the test says which way.

> **+1 g, not -1 g.** An accelerometer at rest measures the reaction
> force holding it up, not gravity itself, so the axis pointing UP reads
> positive. Expecting -1 g is a classic sign error that inverts every
> tilt reading. This bit us once already; the test now encodes the
> correct sign.

Useful options:

```bash
python3 tests/test_06_imu.py --address 0x69     # AD0 high
python3 tests/test_06_imu.py --bus 0            # different I2C bus
python3 tests/test_06_imu.py --seconds 10       # longer sample
python3 tests/test_06_imu.py --no-orientation   # skip the axis check
```

---

## 7. Calibrate

```bash
python3 utils/calibrate_imu.py
```

This writes `software/imu_calibration.json`, which is **per-robot** and
deliberately not in git.

It calibrates the **accelerometer** only -- offsets and scale. Those are
stable, so once is enough unless the sensor is remounted.

**Gyro bias is not calibrated here on purpose.** A stationary gyro does
not read zero, and its bias drifts with temperature -- so a figure
stored last week is worse than useless once the servos have warmed the
deck. `imu_lib` therefore re-measures gyro bias fresh at every startup,
while the robot is known to be still.

---

## 8. Using it

```python
from imu_lib import Imu

with Imu() as imu:
    imu.zero_gyro()              # robot must be still
    while True:
        s = imu.read()
        print(s.roll_deg, s.pitch_deg, imu.heading_deg)
```

**Heading drifts.** It comes from integrating yaw rate, and integration
drifts no matter how good the calibration. `imu.mark_stationary()` lets
gait code say "the robot is definitely not turning now" and have the
drift re-zeroed. Use it -- without it, heading is good for seconds, not
minutes.

The agentix UI exposes the sensor read-only through the `imu_state`
tool and the **IMU** button on the control strip.

---

## Why this is worth having

Beyond heading, the IMU measures something the joint encoders
structurally **cannot**: how far the body actually moves.

Chasing a dragging gait (2026-10-13) came down to body sag that the
servo encoders could not see -- structural flex, gear backlash, feet
sliding. The joints reported arriving at their commanded angles while
the chassis sat 13 mm lower than the model believed. Every clearance
computed from joint angles inherited that error, and the only way to
catch it was a ruler and an eye.

Pitch and roll during a tripod lift answer that directly, and tell you
whether the body is dropping evenly or tilting.

`tests/test_07_tripod_lift.py` does this: it samples attitude with the
robot standing, lifts one tripod, samples again, and reports the tilt
plus what it costs at the feet. Useful anchor: **1 degree of tilt is
6 mm at a foot 341 mm from the body centre**, so the 13 mm of sag
measured with a ruler would be about 2.2 degrees if it were pure lean.

**An accelerometer measures tilt, not height.** A body that sinks evenly
on all three stance legs produces no tilt and the test reads zero --
which is why it still asks you to look, and why the ruler has not
retired. What the IMU catches is the asymmetric part, the body leaning
toward the side that just came up, which is the part that costs the
swing feet their clearance soonest.

It also reports mean |g| either side of the lift. Roll and pitch only
mean anything when gravity is the sole force, so a figure far from 1.000
says the robot was still moving and the tilt should not be believed.

---

## The bracket

`imu/design_imu_bracket.py` generates a C-section sled: a base that glues
flat to the deck, a web setting the wiring clearance, and a top plate the
board bolts to. The open side faces forward so a nut can be started on a
bolt dropped through the top plate, and so the wiring leaves underneath
without being pinched.

```bash
python imu/design_imu_bracket.py
python imu/design_imu_bracket.py --height 40 --nut-trap
python imu/design_imu_bracket.py --summary        # dimensions, no STL
```

As built: 35 mm gap, M3 holes 14.5 mm apart (from 17.5 mm measured
outside-to-outside on a 20 x 16 mm board), running ACROSS the plate so
the board's long axis sits athwart the robot. 41 mm overall.

The top plate stops 6 mm past the bolt centres rather than carrying the
whole board. The board is held at its holes, and plate beyond them is
only mass on the end of a 35 mm cantilever; the PCB overhangs the front,
where nothing bears on it.

`--nut-trap` adds a hex pocket under the top plate and thickens that
plate to 4.6 mm to suit. It has to: a 2.6 mm pocket in a 3 mm plate
leaves 0.4 mm of PLA carrying the entire clamp load, which crushes on
first tightening.

**Nothing on the bracket keys the orientation.** Two bolt holes cannot:
the board fits perfectly mounted 180 degrees out, +X aft instead of
forward, and nothing looks wrong. The sensor answers, the test passes if
you run it after mounting it backwards, and every heading is inverted.
Check the board's +X faces the hexagon vertex between legs 1 and 6 when
you fit it, and let `test_06` confirm it against gravity.

The web is 6 mm, which puts the first bending mode near 450 Hz. A foot
plant rings the structure at roughly 50-200 Hz, so the mount stays clear
of it -- a bracket that resonates reports its own wobble as body motion,
which is the one thing this sensor must not do. Dropping the web to 4 mm
would put it at ~310 Hz, which is closer than it needs to be.

Print base-down, no supports.

---

## Troubleshooting

| symptom | likely cause |
|---|---|
| `/dev/i2c-1` missing | `dtparam=i2c_arm=on` not in `/boot/firmware/config.txt`, or no reboot |
| `i2cdetect` permission denied | not in the `i2c` group yet, or not logged out and back in |
| address appears intermittently | SDA/SCL too long, or routed alongside the servo harness |
| `ModuleNotFoundError: smbus2` | installed outside the venv, or venv not activated |
| reads look fine, headings nonsense | mount rotated -- run `test_06` and read the orientation verdict |
| tilt signs inverted | the -1 g sign error; check against the test, not intuition |
