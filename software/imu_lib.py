"""
imu_lib -- MPU-6050 for CRABORA, with calibration and heading
===============================================================

Wraps the raw sensor in the three things that make it usable:

  1. GYRO RE-ZERO AT STARTUP. A stationary gyro does not read zero, and
     its bias drifts with temperature -- so a figure stored last week is
     worse than useless once the servos have warmed the deck. The bias
     is therefore measured fresh every time, while the robot is known to
     be still, not loaded from a file.

  2. ACCELEROMETER OFFSET AND SCALE, loaded from the file written by
     utils/calibrate_imu.py. Unlike gyro bias these are stable, so once
     is enough.

  3. HEADING, by integrating yaw rate. Integration drifts no matter how
     good the calibration, so `mark_stationary()` lets the gait code say
     "the robot is definitely not turning now" and have the drift
     re-zeroed. Use it; without it the heading is good for seconds, not
     minutes.

Axes are the body frame from walk/README.md: +X forward through the
point between legs 1 and 6, +Y left, +Z up. test_06_imu.py checks the
sensor is actually mounted that way.

Usage:
    from imu_lib import Imu

    with Imu() as imu:
        imu.zero_gyro()                 # robot must be still
        while True:
            s = imu.read()
            print(s.roll_deg, s.pitch_deg, imu.heading_deg)

Requires: pip install smbus2
"""

import json
import math
import os
import time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover
    SMBus = None

# --- registers -------------------------------------------------------
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

ACCEL_SCALE = 16384.0     # LSB/g at ±2 g
GYRO_SCALE  = 131.0       # LSB/(deg/s) at ±250 deg/s
TEMP_SCALE  = 340.0
TEMP_OFFSET = 36.53

DEFAULT_ADDRESS = 0x68
DEFAULT_BUS = 1

CALIBRATION_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "imu_calibration.json")

# Below this the robot is treated as stationary, so heading is not
# integrated. Stops bias noise from wandering the heading while standing.
STILL_RATE_DEG = 0.8

# A real MPU-6050's zero-rate offset is a few deg/s. Beyond this it is
# not bias, it is the robot moving -- and a bias measured while moving is
# worse than no calibration at all, because it is then subtracted from
# every future reading.
MAX_PLAUSIBLE_BIAS = 15.0


class ImuError(RuntimeError):
    pass


@dataclass
class Reading:
    """One sample, already corrected, in the body frame."""
    ax: float           # g, +X forward
    ay: float           # g, +Y left
    az: float           # g, +Z up
    gx: float           # deg/s, roll rate
    gy: float           # deg/s, pitch rate
    gz: float           # deg/s, yaw rate (+ve = counter-clockwise from above)
    temp_c: float
    t: float            # monotonic timestamp

    @property
    def roll_deg(self):
        """Rotation about +X. Zero when level.

        Note an accelerometer at rest measures the REACTION force, not
        gravity, so a level robot reads az = +1 g, not -1. Getting that
        backwards inverts roll and pitch.
        """
        return math.degrees(math.atan2(self.ay, self.az))

    @property
    def pitch_deg(self):
        """Rotation about +Y, nose-up positive. Zero when level."""
        return math.degrees(math.atan2(-self.ax,
                                       math.hypot(self.ay, self.az)))

    @property
    def g_total(self):
        """Magnitude of measured acceleration. ~1.0 when still -- a good
        test of whether tilt can be trusted, since roll and pitch are
        only meaningful if gravity is the sole force."""
        return math.sqrt(self.ax ** 2 + self.ay ** 2 + self.az ** 2)

    @property
    def is_still(self):
        return (abs(self.g_total - 1.0) < 0.08
                and max(abs(self.gx), abs(self.gy), abs(self.gz))
                < STILL_RATE_DEG * 3)


def load_calibration(path=CALIBRATION_PATH):
    """Accelerometer offset/scale from utils/calibrate_imu.py.

    Missing file is not an error -- identity calibration is fine for
    heading, which is what most of the robot cares about. It only costs
    accuracy in tilt.
    """
    default = {"accel_offset": [0.0, 0.0, 0.0],
               "accel_scale": [1.0, 1.0, 1.0],
               "source": "defaults (no calibration file)"}
    try:
        with open(path) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return default
    for key in ("accel_offset", "accel_scale"):
        v = data.get(key)
        if not (isinstance(v, list) and len(v) == 3):
            return default
    if any(s == 0 for s in data["accel_scale"]):
        return default
    data.setdefault("source", path)
    return data


class Imu:
    def __init__(self, bus=DEFAULT_BUS, address=DEFAULT_ADDRESS,
                 calibration=None):
        if SMBus is None:
            raise ImuError("smbus2 is not installed: "
                           "python3 -m pip install smbus2")
        self.address = address
        self._bus_no = bus
        self._bus = None
        self.calibration = calibration or load_calibration()
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.heading_deg = 0.0
        self._last_t = None

    # --- lifecycle ---------------------------------------------------
    def open(self):
        try:
            self._bus = SMBus(self._bus_no)
        except (FileNotFoundError, PermissionError) as e:
            raise ImuError(
                f"cannot open I2C bus {self._bus_no}: {e}. On Ubuntu add "
                f"dtparam=i2c_arm=on to /boot/firmware/config.txt, install "
                f"i2c-tools, and add yourself to the i2c group") from e
        try:
            self._bus.read_byte_data(self.address, REG_WHO_AM_I)
        except OSError as e:
            raise ImuError(
                f"nothing answered at 0x{self.address:02X}: {e}. Check the "
                f"wiring and run tests/test_06_imu.py") from e
        # The MPU-6050 boots asleep.
        self._bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self._bus.write_byte_data(self.address, REG_SMPLRT_DIV, 0x07)
        self._bus.write_byte_data(self.address, REG_CONFIG, 0x03)
        self._bus.write_byte_data(self.address, REG_GYRO_CONFIG, 0x00)
        self._bus.write_byte_data(self.address, REG_ACCEL_CONFIG, 0x00)
        time.sleep(0.05)
        return self

    def close(self):
        if self._bus is not None:
            try:
                self._bus.close()
            finally:
                self._bus = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # --- reading -----------------------------------------------------
    def _raw(self):
        d = self._bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)

        def word(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v >= 32768 else v

        return [word(i) for i in (0, 2, 4, 6, 8, 10, 12)]

    def read(self):
        """One corrected sample, and advance the heading."""
        if self._bus is None:
            raise ImuError("IMU is not open")
        ax, ay, az, t_raw, gx, gy, gz = self._raw()
        off = self.calibration["accel_offset"]
        sca = self.calibration["accel_scale"]
        acc = [((v / ACCEL_SCALE) - off[i]) / sca[i]
               for i, v in enumerate((ax, ay, az))]
        gyro = [(v / GYRO_SCALE) - self.gyro_bias[i]
                for i, v in enumerate((gx, gy, gz))]
        now = time.monotonic()
        r = Reading(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
                    t_raw / TEMP_SCALE + TEMP_OFFSET, now)

        # Integrate yaw, but only while actually turning: below the noise
        # floor the "rotation" is bias, and integrating it just walks the
        # heading away during a long stand.
        if self._last_t is not None:
            dt = now - self._last_t
            if 0 < dt < 0.5 and abs(r.gz) >= STILL_RATE_DEG:
                self.heading_deg += r.gz * dt
        self._last_t = now
        return r

    # --- calibration -------------------------------------------------
    def zero_gyro(self, seconds=2.0, progress=None):
        """Measure and subtract gyro bias. THE ROBOT MUST BE STILL.

        Done fresh at every startup rather than stored: bias moves with
        temperature, so yesterday's figure is wrong once the deck warms.
        Raises if the robot was moving, because a bias measured while
        moving is worse than none at all.
        """
        if self._bus is None:
            raise ImuError("IMU is not open")
        self.gyro_bias = [0.0, 0.0, 0.0]
        sums = [0.0, 0.0, 0.0]
        peaks = [0.0, 0.0, 0.0]
        n = 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            _ax, _ay, _az, _t, gx, gy, gz = self._raw()
            for i, v in enumerate((gx / GYRO_SCALE, gy / GYRO_SCALE,
                                   gz / GYRO_SCALE)):
                sums[i] += v
                peaks[i] = max(peaks[i], abs(v))
            n += 1
            if progress and n % 20 == 0:
                progress(min(1.0, 1 - (end - time.monotonic()) / seconds))
            time.sleep(0.005)
        if n == 0:
            raise ImuError("no samples read while zeroing the gyro")
        bias = [s / n for s in sums]
        # Two different ways the robot can be moving, and they look
        # nothing alike:
        #   - WOBBLE: the reading varies far more than its own mean
        #   - STEADY TURN: the reading is constant, which is
        #     indistinguishable from bias by variance alone. Caught
        #     instead by magnitude -- a real MPU-6050 zero-rate offset is
        #     a few deg/s, so anything past MAX_PLAUSIBLE_BIAS is motion.
        for i in range(3):
            if peaks[i] > abs(bias[i]) + 5.0:
                raise ImuError(
                    f"the robot moved while zeroing ({'XYZ'[i]} wobbled to "
                    f"{peaks[i]:.1f} deg/s). Hold it still and retry")
            if abs(bias[i]) > MAX_PLAUSIBLE_BIAS:
                raise ImuError(
                    f"{'XYZ'[i]} reads a steady {bias[i]:+.1f} deg/s. That is "
                    f"too large to be sensor bias, so the robot is turning. "
                    f"Hold it still and retry")
        self.gyro_bias = bias
        self.heading_deg = 0.0
        self._last_t = None
        return bias

    def mark_stationary(self):
        """Tell the IMU the robot is definitely not turning.

        Integrated heading drifts however well the gyro is calibrated.
        Call this whenever the gait code knows the robot is at rest --
        between strides, after a turn completes -- and the accumulated
        drift is discarded. Without it the heading is good for seconds,
        not minutes.
        """
        self._last_t = None

    def reset_heading(self, value=0.0):
        self.heading_deg = value
        self._last_t = None


def attitude(imu, seconds=0.6):
    """Mean roll and pitch over a short window, in degrees.

    Averaged rather than sampled once: a single reading catches whatever
    the robot happened to be doing at that instant.

    Also returns the mean |g|, which is the honesty check. Roll and pitch
    are only meaningful when gravity is the only force acting, so a
    figure far from 1.0 means the robot was still moving and the tilt
    should not be believed. A CONSTANT offset (the same |g| before and
    after) is calibration rather than motion, and cancels in a
    difference -- so a tilt CHANGE stays usable even uncalibrated.

    Needs no gyro and no zeroing: tilt comes from the accelerometer
    alone. Returns None if there is no IMU, so callers can treat the
    sensor as optional without branching everywhere.
    """
    if imu is None:
        return None
    rolls, pitches, gs = [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            r = imu.read()
        except ImuError:
            break
        rolls.append(r.roll_deg)
        pitches.append(r.pitch_deg)
        gs.append(r.g_total)
        time.sleep(0.01)
    if not rolls:
        return None
    n = len(rolls)
    return {"roll": sum(rolls) / n, "pitch": sum(pitches) / n,
            "g": sum(gs) / n, "n": n}


def tilt_between(before, after):
    """(magnitude, bearing) of the attitude change, in degrees.

    Bearing is measured in the body frame: 0 is nose-down, +90 is
    left-down. Returns None if either sample is missing.
    """
    if not before or not after:
        return None
    d_roll = after["roll"] - before["roll"]
    d_pitch = after["pitch"] - before["pitch"]
    return (math.hypot(d_roll, d_pitch),
            math.degrees(math.atan2(d_roll, d_pitch)))
