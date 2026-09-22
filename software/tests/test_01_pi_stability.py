#!/usr/bin/env python3
"""
CRABORA bring-up test 1: Raspberry Pi stability burn-in
=========================================================

Runs ON the Raspberry Pi that lives on the unibody (see ../../doc/uniBody.md).
Before anything else is trusted -- servo bus, gait code, sensors -- the board
itself has to hold up under sustained load without under-volting or
thermal-throttling. This is the single most common Pi failure mode in a
robot: a marginal power supply or thin/long USB cable that's fine at idle
but sags the moment all 4 cores load up, causing silent slowdowns or
brownout resets right when the robot starts moving.

What it does:
  1. Loads all CPU cores for --duration seconds using a plain Python
     busy-loop (stdlib multiprocessing only -- no pip installs required,
     so this runs on a bare Raspberry Pi OS image).
  2. Samples, once per second, for as long as vcgencmd/thermal_zone/cpufreq
     are available on this machine:
       - CPU temperature      (/sys/class/thermal/thermal_zone0/temp)
       - CPU clock speed      (/sys/devices/.../cpufreq/scaling_cur_freq)
       - throttled/undervolt  (`vcgencmd get_throttled` bitmask)
       - 1-min load average   (os.getloadavg)
  3. Prints a summary and exits 0 (stable) or 1 (unstable).

`vcgencmd get_throttled` bit meanings (Raspberry Pi firmware):
    bit  0  under-voltage NOW
    bit  1  arm frequency capped NOW
    bit  2  currently throttled
    bit  3  soft temperature limit active NOW
    bit 16  under-voltage has occurred since boot
    bit 17  arm frequency capped has occurred since boot
    bit 18  throttling has occurred since boot
    bit 19  soft temperature limit has occurred since boot

Any "since boot" bit (16-19) means the board is not solid for this job,
even if it recovered -- the FAIL verdict below is driven off those, since a
transient dip is exactly the kind of thing you don't want happening in the
middle of a stand-up sequence.

Usage:
  python3 test_01_pi_stability.py                 # 60 s burn-in, all cores
  python3 test_01_pi_stability.py --duration 300   # 5 minute soak
  python3 test_01_pi_stability.py --workers 2       # only load 2 cores
"""

import argparse
import multiprocessing
import os
import subprocess
import sys
import time

THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
CPUFREQ_PATH = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"

# Above this, the Pi 3's firmware soft-throttles by default.
SOFT_TEMP_LIMIT_C = 80.0

THROTTLED_BITS = {
    0:  "under-voltage NOW",
    1:  "arm freq capped NOW",
    2:  "currently throttled",
    3:  "soft temp limit active NOW",
    16: "under-voltage occurred since boot",
    17: "arm freq capped occurred since boot",
    18: "throttling occurred since boot",
    19: "soft temp limit occurred since boot",
}
SINCE_BOOT_BITS = (16, 17, 18, 19)


# =============================================================
# Sampling
# =============================================================
def read_temp_c():
    try:
        with open(THERMAL_PATH) as f:
            return int(f.read().strip()) / 1000.0
    except OSError:
        return None


def read_cpu_freq_mhz():
    try:
        with open(CPUFREQ_PATH) as f:
            return int(f.read().strip()) / 1000.0
    except OSError:
        return None


def read_throttled():
    """Return the raw vcgencmd throttled bitmask as an int, or None if
    vcgencmd isn't available (not a Pi, or firmware tools not installed)."""
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=5, check=True,
        ).stdout.strip()
        # format: "throttled=0x50000"
        return int(out.split("=", 1)[1], 16)
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def sample():
    return {
        "t": time.time(),
        "temp_c": read_temp_c(),
        "freq_mhz": read_cpu_freq_mhz(),
        "throttled": read_throttled(),
        "load1": os.getloadavg()[0],
    }


# =============================================================
# CPU load generator (stdlib only, no numpy/etc.)
# =============================================================
def _burn(stop_at):
    x = 0.0001
    while time.time() < stop_at:
        for _ in range(50_000):
            x = (x * 1.0000001 + 1.0) % 1e6


def run_burn_in(duration_s, workers):
    stop_at = time.time() + duration_s
    procs = [multiprocessing.Process(target=_burn, args=(stop_at,))
             for _ in range(workers)]
    for p in procs:
        p.start()

    samples = []
    while time.time() < stop_at:
        samples.append(sample())
        time.sleep(1.0)

    for p in procs:
        p.join()

    return samples


# =============================================================
# Verdict
# =============================================================
def summarize(samples):
    temps = [s["temp_c"] for s in samples if s["temp_c"] is not None]
    freqs = [s["freq_mhz"] for s in samples if s["freq_mhz"] is not None]
    throttle_masks = [s["throttled"] for s in samples if s["throttled"] is not None]

    print(f"\nsamples collected : {len(samples)}")
    if temps:
        print(f"CPU temp           : min {min(temps):.1f}C  max {max(temps):.1f}C  "
              f"avg {sum(temps)/len(temps):.1f}C  (soft limit {SOFT_TEMP_LIMIT_C:.0f}C)")
    else:
        print("CPU temp           : unavailable (no thermal_zone0)")

    if freqs:
        print(f"CPU clock          : min {min(freqs):.0f}MHz  max {max(freqs):.0f}MHz")
    else:
        print("CPU clock          : unavailable (no scaling_cur_freq)")

    ok = True
    reasons = []

    if throttle_masks:
        union = 0
        for m in throttle_masks:
            union |= m
        active_bits = [bit for bit in sorted(THROTTLED_BITS) if union & (1 << bit)]
        if active_bits:
            print("vcgencmd throttled : flags seen during run:")
            for bit in active_bits:
                print(f"    bit {bit:<2d} - {THROTTLED_BITS[bit]}")
        else:
            print("vcgencmd throttled : clean (0x0 the whole run)")

        for bit in SINCE_BOOT_BITS:
            if union & (1 << bit):
                ok = False
                reasons.append(THROTTLED_BITS[bit])
    else:
        print("vcgencmd throttled : unavailable (not a Pi, or vcgencmd not installed)")

    if temps and max(temps) >= SOFT_TEMP_LIMIT_C:
        ok = False
        reasons.append(f"CPU temp hit {max(temps):.1f}C (>= {SOFT_TEMP_LIMIT_C:.0f}C soft limit)")

    return ok, reasons


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60.0,
                     help="burn-in duration in seconds (default: 60)")
    ap.add_argument("--workers", type=int, default=multiprocessing.cpu_count(),
                     help="number of CPU-loading worker processes "
                          "(default: all cores)")
    args = ap.parse_args()

    print(f"Crabora bring-up test 1: Pi stability burn-in")
    print(f"  duration : {args.duration:.0f} s")
    print(f"  workers  : {args.workers} (cpu_count={multiprocessing.cpu_count()})")
    print("Loading CPU and sampling temp/clock/throttle once per second...")

    samples = run_burn_in(args.duration, args.workers)
    ok, reasons = summarize(samples)

    print()
    if ok:
        print("RESULT: PASS -- board held steady under load, no under-voltage "
              "or throttle events since boot.")
        sys.exit(0)
    else:
        print("RESULT: FAIL -- instability detected:")
        for r in reasons:
            print(f"  - {r}")
        print("Check the power supply and USB/GPIO power cable/connector "
              "before trusting this board to run the robot.")
        sys.exit(1)


if __name__ == "__main__":
    main()
