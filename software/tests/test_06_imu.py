#!/usr/bin/env python3
"""
CRABORA bring-up test 6: MPU-6050 IMU
=======================================

Checks the IMU is wired, talking, and -- the part that actually matters
-- MOUNTED THE RIGHT WAY ROUND.

A sensor that answers on the bus but is rotated 90 degrees on the deck
will pass every naive test and then quietly corrupt every heading the
robot ever computes. So this test does not stop at "the chip responds".
It uses gravity as a known reference: the robot is sitting still on a
level floor, so the accelerometer must read ~1 g on the body frame's
+Z and ~0 on X and Y. Anything else means the mount is rotated, and it
says which way.

(An accelerometer at rest measures the REACTION force holding it up,
not gravity itself, so the axis pointing UP reads +1 g. Expecting -1 g
is a classic sign error and inverts every tilt reading.)

Body frame, from walk/README.md:
    +X forward, out through the point between legs 1 and 6
    +Y to the left
    +Z up

Wiring (Pi 40-pin header):
    VCC -> pin 1  (3.3 V -- NOT 5 V; the breakout's pull-ups would put
                   5 V on the Pi's 3.3 V-only GPIO)
    GND -> pin 6
    SDA -> pin 3  (GPIO2 / SDA1)
    SCL -> pin 5  (GPIO3 / SCL1)
    AD0 -> floating for address 0x68, tied high for 0x69

On Ubuntu there is no raspi-config: add `dtparam=i2c_arm=on` to
/boot/firmware/config.txt (note the path differs from Raspberry Pi OS),
then `sudo apt install i2c-tools python3-smbus` and
`sudo usermod -aG i2c $USER`.

NOTHING MOVES. This is read-only and safe to run at any time, but the
orientation check needs the robot STILL and on a LEVEL surface.

Usage:
  python3 test_06_imu.py
  python3 test_06_imu.py --seconds 5
  python3 test_06_imu.py --address 0x69 --bus 1
  python3 test_06_imu.py --no-orientation     # skip the level-floor check

Requires: pip install smbus2
"""

import argparse
import math
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    print("FAIL: smbus2 is not installed.")
    print("  Install it with:  python3 -m pip install smbus2")
    sys.exit(1)

# --- MPU-6050 registers ----------------------------------------------
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B      # 14 bytes: accel(6), temp(2), gyro(6)
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

WHO_AM_I_EXPECT = 0x68       # some clones report 0x70 / 0x72 -- see below

ACCEL_SCALE = 16384.0        # LSB per g at ±2 g
GYRO_SCALE  = 131.0          # LSB per deg/s at ±250 deg/s
TEMP_SCALE  = 340.0
TEMP_OFFSET = 36.53

# --- what "mounted correctly" means ----------------------------------
# Sitting still and level, the sensor measures the reaction force
# holding it up: +1 g on the body frame's +Z, nothing on X and Y.
GRAVITY_TOL   = 0.15         # g, how far off 1.0 the down axis may read
LEVEL_TOL     = 0.15         # g, how much X/Y may read before it is tilted
GYRO_DRIFT_TOL = 2.0         # deg/s of bias while stationary


def read_word(bus, addr, reg):
    """Two bytes big-endian, as a signed 16-bit value."""
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg + 1)
    val = (hi << 8) | lo
    return val - 65536 if val >= 32768 else val


def read_all(bus, addr):
    """accel (g), temp (C), gyro (deg/s) in one burst."""
    d = bus.read_i2c_block_data(addr, REG_ACCEL_XOUT_H, 14)

    def word(i):
        v = (d[i] << 8) | d[i + 1]
        return v - 65536 if v >= 32768 else v

    ax, ay, az = (word(0) / ACCEL_SCALE, word(2) / ACCEL_SCALE,
                  word(4) / ACCEL_SCALE)
    temp = word(6) / TEMP_SCALE + TEMP_OFFSET
    gx, gy, gz = (word(8) / GYRO_SCALE, word(10) / GYRO_SCALE,
                  word(12) / GYRO_SCALE)
    return (ax, ay, az), temp, (gx, gy, gz)


def wake(bus, addr):
    """The MPU-6050 boots asleep; nothing reads until it is woken."""
    bus.write_byte_data(addr, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    bus.write_byte_data(addr, REG_SMPLRT_DIV, 0x07)     # 1 kHz / 8
    bus.write_byte_data(addr, REG_CONFIG, 0x03)          # 44 Hz DLPF
    bus.write_byte_data(addr, REG_GYRO_CONFIG, 0x00)     # ±250 deg/s
    bus.write_byte_data(addr, REG_ACCEL_CONFIG, 0x00)    # ±2 g
    time.sleep(0.05)


AXES = ("X", "Y", "Z")


def describe_mount(ax, ay, az):
    """Work out how the sensor is actually oriented, from gravity.

    Returns (ok, message). Correct means gravity on -Z and nothing much
    on X or Y -- i.e. the sensor's axes line up with the body frame.
    """
    g = (ax, ay, az)
    mag = math.sqrt(sum(v * v for v in g))
    if abs(mag - 1.0) > GRAVITY_TOL * 2:
        return False, (f"total acceleration is {mag:.2f} g, not ~1.00 -- the "
                       f"robot is not still, or the scale is wrong")

    # Which axis is carrying gravity?
    biggest = max(range(3), key=lambda i: abs(g[i]))
    sign = "-" if g[biggest] < 0 else "+"
    others = [i for i in range(3) if i != biggest]

    if biggest == 2 and g[2] > 0 and all(abs(g[i]) <= LEVEL_TOL for i in others):
        return True, "+1 g on +Z, X and Y flat: mount agrees with the body frame"

    if biggest == 2 and g[2] < 0:
        return False, ("-1 g on Z -- the sensor is UPSIDE DOWN. Flip it, "
                       "or negate Y and Z in software")

    if biggest != 2:
        return False, (f"gravity is on {sign}{AXES[biggest]}, not +Z -- the "
                       f"sensor is mounted on its {'side' if biggest else 'end'}. "
                       f"Rotate it so its Z points UP, or record the rotation "
                       f"as a constant")

    tilt = math.degrees(math.atan2(math.hypot(ax, ay), abs(az)))
    return False, (f"gravity is on +Z but X={ax:+.2f} Y={ay:+.2f} -- the robot "
                   f"is tilted about {tilt:.0f}°, or the floor is not level")


def main():
    ap = argparse.ArgumentParser(
        description="CRABORA bring-up test 6: MPU-6050 IMU.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=1, help="I2C bus (default 1)")
    ap.add_argument("--address", type=lambda s: int(s, 0), default=0x68,
                    help="I2C address, 0x68 or 0x69 (default 0x68)")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="how long to sample (default 3)")
    ap.add_argument("--no-orientation", dest="orientation",
                    action="store_false",
                    help="skip the gravity/mount check (use if the robot "
                           "cannot be put level and still)")
    args = ap.parse_args()

    print("Crabora bring-up test 6: MPU-6050 IMU")
    print(f"  bus {args.bus}, address 0x{args.address:02X}\n")

    try:
        bus = SMBus(args.bus)
    except (FileNotFoundError, PermissionError) as e:
        print(f"RESULT: FAIL -- cannot open I2C bus {args.bus}: {e}")
        print("Checklist:")
        print("    dtparam=i2c_arm=on  in /boot/firmware/config.txt, then reboot")
        print("    sudo apt install i2c-tools python3-smbus")
        print("    sudo usermod -aG i2c $USER   # then log out and back in")
        print(f"    i2cdetect -y {args.bus}      # should show 68")
        return 1

    problems = []
    try:
        # --- is it there? --------------------------------------------
        try:
            who = bus.read_byte_data(args.address, REG_WHO_AM_I)
        except OSError as e:
            print(f"RESULT: FAIL -- nothing answered at "
                  f"0x{args.address:02X}: {e}")
            print("Checklist:")
            print(f"    i2cdetect -y {args.bus}   -- is 68 (or 69) listed?")
            print("    VCC on pin 1 (3.3 V), GND pin 6, SDA pin 3, SCL pin 5")
            print("    AD0 floating = 0x68, tied high = 0x69")
            print("    keep the I2C run short and clear of the servo power bus")
            return 1

        # Genuine MPU-6050s report 0x68. Clones (MPU-6500/9250 dies sold
        # on GY-521 boards) report 0x70/0x71/0x73 and are register
        # compatible for everything used here, so warn rather than fail.
        if who == WHO_AM_I_EXPECT:
            print(f"  WHO_AM_I 0x{who:02X} -- genuine MPU-6050")
        else:
            print(f"  WHO_AM_I 0x{who:02X} -- not 0x{WHO_AM_I_EXPECT:02X}; "
                  f"probably a clone die, register-compatible for this test")

        wake(bus, args.address)

        # --- sample --------------------------------------------------
        n = 0
        acc_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        gyro_max = [0.0, 0.0, 0.0]
        temp_sum = 0.0
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            acc, temp, gyro = read_all(bus, args.address)
            for i in range(3):
                acc_sum[i] += acc[i]
                gyro_sum[i] += gyro[i]
                gyro_max[i] = max(gyro_max[i], abs(gyro[i]))
            temp_sum += temp
            n += 1
            time.sleep(0.02)

        if n == 0:
            print("RESULT: FAIL -- no samples read.")
            return 1

        acc = [v / n for v in acc_sum]
        gyro = [v / n for v in gyro_sum]
        temp = temp_sum / n

        print(f"\n  {n} samples over {args.seconds:g} s, {temp:.1f}°C\n")
        print(f"  {'axis':<6} {'accel (g)':>11} {'gyro (deg/s)':>14} {'peak':>8}")
        for i in range(3):
            print(f"  {AXES[i]:<6} {acc[i]:>+11.3f} {gyro[i]:>+14.2f} "
                  f"{gyro_max[i]:>8.2f}")

        # --- is it actually reading? ---------------------------------
        if all(abs(v) < 0.01 for v in acc):
            problems.append("accelerometer reads zero on every axis -- "
                            "still asleep, or SDA/SCL swapped")

        # --- gyro bias while stationary ------------------------------
        worst = max(abs(v) for v in gyro)
        if worst > GYRO_DRIFT_TOL:
            problems.append(
                f"gyro bias is {worst:.1f} deg/s while stationary (>"
                f"{GYRO_DRIFT_TOL:g}). Usable, but it must be calibrated out "
                f"or heading will drift {worst * 60:.0f}° per minute")
        else:
            print(f"\n  gyro bias {worst:.2f} deg/s -- would drift "
                  f"{worst * 60:.0f}°/minute uncorrected")

        # --- the check that matters ----------------------------------
        if args.orientation:
            ok, msg = describe_mount(*acc)
            print(f"\n  Mount: {msg}")
            if not ok:
                problems.append(f"orientation: {msg}")
        else:
            print("\n  Mount check skipped.")

        print("\n=== RESULT ===")
        if problems:
            print(f"  FAIL -- {len(problems)} problem(s):")
            for p in problems:
                print(f"    - {p}")
            return 1
        print("  PASS -- IMU present, reading, and mounted in line with the "
              "body frame.")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        return 130
    except OSError as e:
        print(f"\nRESULT: FAIL -- I2C error mid-read: {e}")
        print("  Usually a wiring problem: short the run, keep it away from")
        print("  the servo power bus, and check the connector is seated.")
        return 1
    finally:
        try:
            bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
