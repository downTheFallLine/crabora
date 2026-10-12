#!/usr/bin/env python3
"""
Calibrate the MPU-6050's accelerometer -- the six-position method.

Each axis has an offset (what it reads when it should read zero) and a
scale error (what it reads for 1 g when it should read exactly 1.00).
Resting the robot with each axis pointing first up, then down, measures
both at once:

    offset = (up + down) / 2        -- should be 0
    scale  = (up - down) / 2        -- should be 1 g

Six placements, a few seconds each. Unlike gyro bias -- which moves with
temperature and so is re-measured at every startup by imu_lib -- these
are stable, so once is enough. Results go to software/imu_calibration.json
and imu_lib picks them up automatically.

This affects TILT accuracy. Heading comes from the gyro and does not
need it, so if you only care about walking straight you can skip this
entirely.

WHAT "AXIS UP" MEANS, in the body frame (walk/README.md):
    +Z up      robot sitting normally, feet down       (default)
    -Z up      robot upside down
    +X up      nose up: front point toward the ceiling
    -X up      nose down
    +Y up      rolled right side down / left side up
    -Y up      rolled the other way

Rest it steadily in each pose -- against a wall or a box is fine. It
does not have to be exact: a few degrees out changes the result by well
under a percent, and the tool reports how square each pose actually was
so you can see whether it mattered.

Usage:
  python3 utils/calibrate_imu.py
  python3 utils/calibrate_imu.py --seconds 4
  python3 utils/calibrate_imu.py --show        # print the stored file

Requires: pip install smbus2
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from imu_lib import (  # noqa: E402
    ACCEL_SCALE, CALIBRATION_PATH, DEFAULT_ADDRESS, DEFAULT_BUS,
    GYRO_SCALE, Imu, ImuError, load_calibration,
)

AXES = ("X", "Y", "Z")

# (label, axis index, sign, how to place it)
POSES = (
    ("+Z up", 2, +1, "sitting normally, feet down"),
    ("-Z up", 2, -1, "upside down"),
    ("+X up", 0, +1, "nose up -- the front point toward the ceiling"),
    ("-X up", 0, -1, "nose down"),
    ("+Y up", 1, +1, "rolled with the LEFT side up"),
    ("-Y up", 1, -1, "rolled with the RIGHT side up"),
)

SQUARE_WARN_DEG = 12.0     # how far off-axis a pose may be before remarking


def sample(imu, seconds):
    """Average raw accelerometer g over `seconds`. Returns (avg, peak_dev)."""
    sums = [0.0, 0.0, 0.0]
    peak = 0.0
    n = 0
    end = time.monotonic() + seconds
    first = None
    while time.monotonic() < end:
        ax, ay, az, _t, _gx, _gy, _gz = imu._raw()
        g = [ax / ACCEL_SCALE, ay / ACCEL_SCALE, az / ACCEL_SCALE]
        if first is None:
            first = list(g)
        peak = max(peak, max(abs(g[i] - first[i]) for i in range(3)))
        for i in range(3):
            sums[i] += g[i]
        n += 1
        time.sleep(0.01)
    if n == 0:
        raise ImuError("no samples read")
    return [s / n for s in sums], peak


def main():
    ap = argparse.ArgumentParser(
        description="Six-position accelerometer calibration for the MPU-6050.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=DEFAULT_BUS)
    ap.add_argument("--address", type=lambda s: int(s, 0),
                    default=DEFAULT_ADDRESS)
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="sample time per pose (default 3)")
    ap.add_argument("--out", default=CALIBRATION_PATH)
    ap.add_argument("--show", action="store_true",
                    help="print the stored calibration and exit")
    args = ap.parse_args()

    if args.show:
        cal = load_calibration(args.out)
        print(json.dumps(cal, indent=2))
        return 0

    print("MPU-6050 accelerometer calibration (six-position)")
    print(f"  {args.seconds:g} s per pose, writing to {args.out}\n")
    print("  This corrects TILT. Heading comes from the gyro and is")
    print("  unaffected, so skip this if you only want to walk straight.\n")

    try:
        imu = Imu(bus=args.bus, address=args.address).open()
    except ImuError as e:
        print(f"✗ {e}")
        return 1

    readings = {}
    try:
        for label, axis, sign, how in POSES:
            print(f"  {label}: rest the robot {how}.")
            try:
                input("    Press Enter when it is steady (Ctrl-C to abort). ")
            except (EOFError, KeyboardInterrupt):
                print("\n\n  Aborted -- nothing written.")
                return 130
            avg, wobble = sample(imu, args.seconds)
            readings[(axis, sign)] = avg

            # How square was it? The chosen axis should carry all of the
            # gravity; whatever lands on the other two is the tilt error.
            others = math.hypot(*[avg[i] for i in range(3) if i != axis])
            off = math.degrees(math.atan2(others, abs(avg[axis])))
            note = ""
            if wobble > 0.05:
                note = f"  ⚠ moved during the sample ({wobble:.2f} g)"
            elif off > SQUARE_WARN_DEG:
                note = f"  ⚠ {off:.0f}° off square"
            print(f"    {AXES[axis]}={avg[axis]:+.3f} g, "
                  f"{off:.0f}° off square{note}\n")

        # --- solve ----------------------------------------------------
        offset, scale = [0.0] * 3, [1.0] * 3
        for axis in range(3):
            up = readings[(axis, +1)][axis]
            down = readings[(axis, -1)][axis]
            offset[axis] = (up + down) / 2.0
            scale[axis] = (up - down) / 2.0

        print("  Results:")
        print(f"  {'axis':<6} {'up':>8} {'down':>8} {'offset':>9} {'scale':>8}")
        bad = []
        for axis in range(3):
            up = readings[(axis, +1)][axis]
            down = readings[(axis, -1)][axis]
            print(f"  {AXES[axis]:<6} {up:>+8.3f} {down:>+8.3f} "
                  f"{offset[axis]:>+9.4f} {scale[axis]:>8.4f}")
            if not 0.85 <= abs(scale[axis]) <= 1.15:
                bad.append(f"{AXES[axis]} scale {scale[axis]:.3f} is far from "
                           f"1.0 -- poses swapped, or the sensor is faulty")
            if scale[axis] < 0:
                bad.append(f"{AXES[axis]} scale is negative -- the up and down "
                           f"poses for this axis were the wrong way round")
            if abs(offset[axis]) > 0.25:
                bad.append(f"{AXES[axis]} offset {offset[axis]:+.3f} g is "
                           f"large; plausible but worth re-running")

        if bad:
            print("\n  ✗ Not written:")
            for b in bad:
                print(f"      {b}")
            return 1

        data = {
            "accel_offset": [round(v, 5) for v in offset],
            "accel_scale": [round(v, 5) for v in scale],
            "source": "utils/calibrate_imu.py six-position",
            "written": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": ("Accelerometer only. Gyro bias is NOT stored: it drifts "
                     "with temperature, so imu_lib re-measures it at every "
                     "startup via zero_gyro()."),
        }
        with open(args.out, "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        print(f"\n  ✓ wrote {args.out}")
        print("  imu_lib picks this up automatically.")
        return 0

    except KeyboardInterrupt:
        print("\n\n  Aborted -- nothing written.")
        return 130
    except ImuError as e:
        print(f"\n✗ {e}")
        return 1
    finally:
        imu.close()


if __name__ == "__main__":
    sys.exit(main())
