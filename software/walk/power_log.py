"""Power log: is the servo supply (LOCOPOWER) the limit?

Every motion program takes --power-log. While it runs, every servo
status read -- the motion's own polling plus one extra read per wait
loop of whichever servo is stalest -- is kept with its current and the
supply voltage that servo sees. At the end you get a summary and a CSV
in software/logs/power/.

How to read it. The LM2596 holds 7.4 V right up to its current limit
(~2 A continuous, 3 A absolute), then the voltage falls. So:

  * total current climbing toward 2-3 A AND the voltage sagging at the
    same moments  -> the buck is the wall.
  * high current, no sag -> the buck is coping.
  * sag with low current -> not the buck's current limit: look at the
    pack, the wiring, or a hot buck.

Limits of the measurement, worth keeping in mind:
  * Servos are read one at a time, so the "total" is the sum of each
    servo's most recent reading -- an estimate, not a simultaneous
    measurement. Short spikes between reads are missed.
  * Amps per count (6.5 mA) is Feetech's figure, never checked against a
    meter on this robot, and it is motor current only -- not the servo
    electronics or the Pi.
  * Voltage is in 0.1 V steps.
An inline meter on the buck output is still the ground truth; this tells
you WHEN to look and how hard the robot was pulling at the time.

Re-read a saved log:
    python3 walk/power_log.py logs/power/scurry-20261115-142200.csv
"""
import csv
import os
import sys
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import gait_lib  # noqa: E402

LOG_DIR = os.path.join(os.path.dirname(_HERE), "logs", "power")

BUCK_CONTINUOUS = 2.0    # A, LM2596 comfortable continuous
BUCK_ABSOLUTE = 3.0      # A, LM2596 switch-current ceiling
SAG_VOLTS = 0.3          # a drop this far under the resting voltage is a sag
VOLT_WINDOW = 0.25       # s: supply voltage = lowest reading this recent
FRESH = 0.5              # s: a servo reading older than this is "stale"

JOINT = {1: "coxa", 2: "femur", 3: "tibia"}


# =============================================================
# RECORDING
# =============================================================
class Recording:
    def __init__(self, name):
        self.name = name
        self.started = datetime.now()
        self.t0 = time.monotonic()


def start(name, enabled=True, log=print):
    """Begin recording, or return None if not enabled."""
    if not enabled:
        return None
    gait_lib.power_log_start()
    log(f"  power log: recording ({name})")
    return Recording(name)


def finish(rec, log=print):
    """Stop, write the CSV, print the summary. Safe to call with None.
    Returns the summary dict (or None)."""
    if rec is None:
        return None
    samples = gait_lib.power_log_stop()
    rows = timeline(samples, rec.t0)
    path = None
    if rows:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(
            LOG_DIR, f"{rec.name}-{rec.started:%Y%m%d-%H%M%S}.csv")
        write_csv(path, rows)
    summary = analyse(rows)
    for line in report(summary, path):
        log(line)
    return summary


# =============================================================
# ANALYSIS
# =============================================================
def timeline(samples, t0=None):
    """Raw samples -> one row per sample with the running estimates.

    Each row: t, sid, amps, volts, total (sum of every servo's latest
    reading), fresh (servos read within FRESH s), supply (lowest voltage
    read in the last VOLT_WINDOW s), retries, failures.
    """
    if not samples:
        return []
    t0 = samples[0][0] if t0 is None else t0
    latest = {}                       # sid -> (t, amps)
    recent_v = []                     # [(t, volts)]
    rows = []
    for t, sid, amps, volts, retries, failures in sorted(samples):
        latest[sid] = (t, amps)
        if volts is not None:
            recent_v.append((t, volts))
        recent_v = [(tv, v) for tv, v in recent_v if t - tv <= VOLT_WINDOW]
        rows.append({
            "t": round(t - t0, 3),
            "sid": sid,
            "amps": round(amps, 3),
            "volts": volts,
            "total": round(sum(a for _, a in latest.values()), 3),
            "fresh": sum(1 for ts, _ in latest.values() if t - ts <= FRESH),
            "seen": len(latest),
            "supply": min((v for _, v in recent_v), default=None),
            "retries": retries,
            "failures": failures,
        })
    return rows


def _pct(values, p):
    s = sorted(values)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))] if s else None


def _time_above(rows, amps):
    """Seconds the total spent at or above `amps`."""
    out = 0.0
    for a, b in zip(rows, rows[1:]):
        if a["total"] >= amps:
            out += b["t"] - a["t"]
    return out


def analyse(rows):
    if not rows:
        return {"samples": 0}
    sids = sorted({r["sid"] for r in rows})
    duration = rows[-1]["t"] - rows[0]["t"]
    # Until every servo has reported once, the summed total leaves some
    # out and reads low. Judge only from the rows after that.
    n = len(sids)
    full = [r for r in rows if (r.get("seen") or n) >= n]
    rows = full or rows
    totals = [r["total"] for r in rows]
    volts = [r["volts"] for r in rows if r["volts"] is not None]
    peak = max(rows, key=lambda r: r["total"])

    # Resting voltage: what the buck holds when it is not struggling. A
    # buck cannot overshoot its setpoint, so the highest reading is it
    # (0.1 V steps, so a one-off high reading costs at most that much).
    rest = max(volts) if volts else None
    low = min(rows, key=lambda r: r["volts"] if r["volts"] is not None else 99)
    low_v = low["volts"]

    # The supply voltage while pulling hard vs. while not.
    # Hardest-pulling tenth of the run vs. the easiest third.
    hi_cut = _pct(totals, 90)
    lo_cut = _pct(totals, 30)
    v_hi = [r["supply"] for r in rows if r["total"] >= hi_cut and r["supply"]]
    v_lo = [r["supply"] for r in rows if r["total"] <= lo_cut and r["supply"]]

    per_joint = {}
    per_servo = {}
    for r in rows:
        per_servo.setdefault(r["sid"], []).append(r["amps"])
    for sid, amps in per_servo.items():
        j = per_joint.setdefault(JOINT.get(sid % 10, "?"), [])
        j.extend(amps)

    return {
        "samples": len(rows),
        "servos": len(sids),
        "duration": duration,
        "reads_per_servo_s": len(rows) / len(sids) / duration if duration else None,
        "peak": peak["total"], "peak_t": peak["t"],
        "peak_supply": peak["supply"],
        "mean": sum(totals) / len(totals),
        "p95": _pct(totals, 95),
        "above_cont": _time_above(rows, BUCK_CONTINUOUS),
        "above_abs": _time_above(rows, BUCK_ABSOLUTE),
        "rest_v": rest,
        "low_v": low_v, "low_t": low["t"], "low_total": low["total"],
        "v_hard": sum(v_hi) / len(v_hi) if v_hi else None,
        "v_easy": sum(v_lo) / len(v_lo) if v_lo else None,
        "hard_cut": hi_cut, "easy_cut": lo_cut,
        "retries": rows[-1]["retries"] - rows[0]["retries"],
        "failures": rows[-1]["failures"] - rows[0]["failures"],
        "joints": {j: (max(a), sum(a) / len(a)) for j, a in per_joint.items()},
        "top": sorted(((max(a), sid) for sid, a in per_servo.items()),
                      reverse=True)[:3],
    }


def verdict(s):
    if not s.get("rest_v") or s.get("low_v") is None:
        return "no voltage readings -- cannot judge"
    sag = s["rest_v"] - s["low_v"]
    # Does the voltage drop WITH the current? Compare the supply while
    # pulling hard against the supply while pulling little (or, if it
    # never pulled little, against resting).
    easy = s["v_easy"] if s["v_easy"] is not None else s["rest_v"]
    loaded_sag = (easy - s["v_hard"]) if s["v_hard"] is not None else 0.0
    if (sag >= SAG_VOLTS and s["easy_cut"] >= BUCK_CONTINUOUS
            and easy <= s["rest_v"] - SAG_VOLTS):
        return (f"HITTING THE WALL: the robot never pulled less than "
                f"~{s['easy_cut']:.1f} A, and the supply sat {sag:.1f} V under "
                f"its resting {s['rest_v']:.1f} V for most of the run. The "
                f"LM2596 is current-limiting throughout.")
    if sag >= SAG_VOLTS and loaded_sag >= SAG_VOLTS / 2 and s["peak"] >= 1.5:
        return (f"HITTING THE WALL: the supply sagged {sag:.1f} V, and it sags "
                f"when the robot pulls hardest (~{s['v_hard']:.1f} V above "
                f"{s['hard_cut']:.1f} A). The LM2596 is current-limiting.")
    if sag >= SAG_VOLTS:
        return (f"SAG WITHOUT HIGH CURRENT: the supply fell {sag:.1f} V but not "
                f"in step with the current. Suspect the pack, the wiring, or a "
                f"hot buck rather than its current limit.")
    if s["peak"] >= BUCK_CONTINUOUS:
        return (f"COPING: drew up to ~{s['peak']:.1f} A with no real sag "
                f"(lowest {s['low_v']:.1f} V). The buck held this one -- but it "
                f"is past its comfortable rating, so watch its temperature.")
    return (f"WELL INSIDE THE LIMIT: peak ~{s['peak']:.1f} A, supply steady "
            f"at {s['rest_v']:.1f} V.")


def report(s, path=None):
    out = ["", "  ---- power log " + "-" * 44]
    if not s.get("samples"):
        out.append("  no servo readings were taken -- nothing to report")
        return out
    rate = s["reads_per_servo_s"]
    out += [
        f"  {s['samples']} readings from {s['servos']} servos over "
        f"{s['duration']:.1f} s"
        + (f" (~{rate:.1f} per servo per second)" if rate else ""),
        f"  total current  peak {s['peak']:.2f} A at {s['peak_t']:.1f} s"
        + (f" (supply {s['peak_supply']:.1f} V)" if s["peak_supply"] else ""),
        f"                 mean {s['mean']:.2f} A, 95% of the time "
        f"under {s['p95']:.2f} A",
        f"                 {s['above_cont']:.1f} s at/above {BUCK_CONTINUOUS:g} A,"
        f" {s['above_abs']:.1f} s at/above {BUCK_ABSOLUTE:g} A",
    ]
    if s["rest_v"]:
        out.append(f"  supply         resting {s['rest_v']:.1f} V, lowest "
                   f"{s['low_v']:.1f} V at {s['low_t']:.1f} s "
                   f"(total then {s['low_total']:.2f} A)")
        if s["v_hard"] and s["v_easy"]:
            out.append(f"                 averages {s['v_hard']:.2f} V while "
                       f"pulling >= {s['hard_cut']:.1f} A, {s['v_easy']:.2f} V "
                       f"at <= {s['easy_cut']:.1f} A")
    joints = ", ".join(f"{j} peak {p:.2f} / mean {m:.2f} A"
                       for j, (p, m) in sorted(s["joints"].items()))
    out.append(f"  per servo      {joints}")
    out.append("  hardest        " + ", ".join(
        f"{sid} ({a:.2f} A)" for a, sid in s["top"]))
    out.append(f"  bus            {s['retries']} retries, {s['failures']} "
               f"failures during the log")
    out += ["", "  " + verdict(s)]
    if path:
        out.append(f"  log: {path}")
    out.append("  (servo-reported current: an estimate -- see walk/power_log.py)")
    return out


# =============================================================
# CSV
# =============================================================
FIELDS = ["t", "sid", "amps", "volts", "total", "fresh", "seen", "supply",
          "retries", "failures"]


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)


def read_csv(path):
    def num(v, kind=float):
        return None if v in ("", "None") else kind(v)
    with open(path, newline="") as f:
        return [{
            "t": num(r["t"]), "sid": num(r["sid"], int),
            "amps": num(r["amps"]), "volts": num(r["volts"]),
            "total": num(r["total"]), "fresh": num(r["fresh"], int),
            "seen": num(r.get("seen", ""), int),
            "supply": num(r["supply"]),
            "retries": num(r["retries"], int),
            "failures": num(r["failures"], int),
        } for r in csv.DictReader(f)]


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    for line in report(analyse(read_csv(argv[1])), argv[1]):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
