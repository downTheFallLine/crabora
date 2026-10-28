#!/usr/bin/env python3
"""
scurry -- walk.py's gait, streamed on a clock instead of stop-and-wait.

WHY WALK.PY IS SLOW.

walk.py moves in frames and waits for every one: write 18 goals one
packet at a time, sleep 50 ms, then poll position and the moving flag on
all 18 until the slowest joint has stopped. Seven of those per step, at
300 steps/s (about 26 deg/s). The robot comes to a dead stop seven times
a step, and the lift and plant alone are over a second each.

That was the right shape for finding out whether the gait worked at
all. It is the wrong shape for going anywhere.

WHAT THIS DOES INSTEAD.

A loop runs at a fixed rate (50 Hz by default). Each tick it works out
where every foot should be at THIS instant of the cycle and sends all of
them -- one sync-write packet per URT, no replies, no polling. A servo
is already heading for the next goal before it reaches the current one,
so the motion is continuous rather than a series of arrivals.

The geometry is walk.py's, unchanged: stance feet are re-solved with
walk.stance_pose() so they hold their patch of floor while the body
moves. What is new is only WHEN each pose is asked for:

  - each tripod spends `--duty` of the cycle on the floor (0.55 by
    default, so both tripods are briefly down together at each
    changeover -- a margin for servo lag, which is real and unmeasured)
  - the swing foot leaves the floor moving with it, arcs up, and comes
    down moving with it again (a Hermite curve on the stride, a sine on
    the femur), so there is no velocity jump at lift-off or touchdown
  - the cycle rate ramps up over the first period and down over the
    last, so the body does not start or stop with a jerk

WHAT IT CANNOT SEE.

This is open loop. walk.py knew, every frame, whether each joint got
there; this does not ask before moving on. What it does instead is
sample: every 0.05 s it reads one servo's real position and current,
round-robin, and the end-of-run report gives each joint's tracking error
and lag. The currents feed agentix's live power view.
That is a measurement, not a guard -- nothing stops on it. Two guards
stand in for walk.py's checks:

  - a supply-voltage read every 0.2 s, round-robin over the femurs. The
    LM2596 caps around 3 A and the power draw is unmeasured; quick
    tripod reversals are exactly what makes current spikes, and a
    brownout here would otherwise look like a gait that has gone wrong.
  - any bus error at all ends the run. In a stream that is far more
    likely to be a sagging supply or a URT dropping off USB than a
    lost packet worth retrying.

Either way it stops by planting whichever feet are up and holding, torque
ON -- same rule as walk.py's Ctrl-C.

GOING FASTER. Body speed comes from stride, and the femur's speed from
lift, so the defaults pair a long stride (120 mm asked) with a modest
lift (20°) and a slightly narrower stance (110 mm) -- about 67 mm/s at
the default 3.0 s period. Then shorten --period a step at a time (2.5,
2.0, 1.7, ...) and watch the voltage and tracking figures it prints at
the end. --dry-run gives the joint speeds each setting needs and refuses
one the servos cannot plausibly follow under load.

Past about a 1.6 s period the stream needs more acceleration than the
servo register can hold (25,400 steps/s^2). The plan then sets it
unlimited rather than capping it silently, and suggests --rate 100 to
keep each frame's step small.

NOT YET RUN ON HARDWARE.

Usage:
  python3 scurry.py 500 --dry-run            # plan only, touch nothing
  python3 scurry.py 500                      # 0.5 m forward, 3 s cycles
  python3 scurry.py 500 --period 2.0         # quicker
  python3 scurry.py 500 --period 1.7 --duty 0.5   # ~118 mm/s planned
  python3 scurry.py 300 --heading 90         # sideways, left

Requires: pip install feetech-servo-sdk   (provides the scservo_sdk module)
"""

import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "stand"))

from gait_lib import (  # noqa: E402
    require_bus_free,
    ALL_LEGS, COXA_JOINT_DIGIT, FEMUR_JOINT_DIGIT, LEG_NAME,
    TIBIA_JOINT_DIGIT, TRIPOD_A, TRIPOD_B, tripod_gap, ground_clearance,
    ADDR_GOAL_POSITION, ADDR_GOAL_SPEED, ADDR_MAX_ANGLE, ADDR_MIN_ANGLE,
    ADDR_TORQUE_ENABLE, BusError, PacketHandler,
    bus_stats, read_status, reset_bus_stats,
    confirm, find_joints, find_urt_devices, note_goal, open_ports, read1, read2,
    write1, write2,
)
from pose import Unreachable, pose_stand  # noqa: E402
from stand import LOADED_TOLERANCE, clamp, move_group, pose_counts  # noqa: E402
from walk import (  # noqa: E402
    DEFAULT_HEIGHT, DEFAULT_COXA_LIMIT, _normalise,
    max_half_stride, measure_stance, planted_radius, stance_pose,
)

try:
    from serial import SerialException
except ImportError:
    class SerialException(Exception):
        pass

DEFAULT_PERIOD = 3.0     # seconds per full cycle (both tripods swing once).
                         # Conservative on purpose: shorten it, not the rest.
# Stride is the cheap way to go faster and lift the expensive one. The
# femur does all the lifting, inside the swing, so its speed scales with
# lift and not stride; body speed scales with stride and not lift. So a
# long stride with a modest lift, rather than walk.py's 50 mm and 35°.
DEFAULT_STRIDE = 120.0   # mm each stance foot covers (asked; plan() may cap it)
DEFAULT_LIFT = 20.0      # femur degrees of swing lift. ~30 mm of foot rise,
                         # still clear of TRIPOD_SAG on a flat floor
DEFAULT_FOOT_OUT = 110.0 # mm outboard of the femur axis while scurrying. 10 mm
                         # in from the stand pose: it slows the femur and
                         # tibia and gives the stride more reach
STAND_FOOT_OUT = 120.0   # where stand.py leaves the feet, so where a run
                         # starts from and goes back to
DEFAULT_DUTY = 0.55      # fraction of the cycle each tripod is on the floor
DEFAULT_RATE = 50.0      # frames per second sent to the servos
DEFAULT_MIN_VOLTS = 6.5  # stop below this. The buck is set to 7.4 V; the
                         # servos' own under-voltage trip is lower, but by
                         # the time it trips the URTs may already be gone.
SETUP_SPEED = 400        # steps/s for the stop-and-wait moves into and
                         # out of the gait, which are walk.py-style

# What an STS3215 can be asked to follow under walking load. Unloaded at
# 7.4 V it manages about 0.22 s/60 deg, ~3000 steps/s; a femur holding a
# third of the robot is a different servo. Half of that, until a real run
# says otherwise. --allow-fast skips the check.
MAX_FOLLOW_SPEED = 1500  # steps/s

STEPS_PER_DEG = 4096 / 360.0

ADDR_ACCELERATION = 41  # 1 byte, units of 100 steps/s^2, 0 = unlimited
ACCEL_MAX = 254          # the register's ceiling: 25,400 steps/s^2
ADDR_PRESENT_VOLTAGE = 62  # 1 byte, 0.1 V
VOLT_EVERY = 0.2         # seconds between voltage reads
LAG_EVERY = 0.05         # seconds between tracking reads (position and
                         # current), one servo at a time, so each of 18 is
                         # read about once a second. Each is a round trip
                         # on the bus; if frames start going out late,
                         # this is the first thing to back off
LAG_MIN_SPEED = 300      # steps/s. Below this a joint is barely moving
                         # and error / speed says nothing about lag

INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

JOINTS = ((COXA_JOINT_DIGIT, "coxa"),
          (FEMUR_JOINT_DIGIT, "femur"),
          (TIBIA_JOINT_DIGIT, "tibia"))


# =============================================================
# THE CYCLE
# =============================================================
def leg_offset(leg):
    """Where in the cycle this leg's tripod starts. A at 0, B half a
    cycle on, so at phase 0 tripod A is starting stance and B is late in
    its own -- both feet down."""
    return 0.0 if leg in TRIPOD_A else 0.5


def swing_s(w, half, duty):
    """Body travel `s` for a swinging foot, w = 0..1 through the swing.

    Cubic Hermite from +half back to -half, with the end velocities set
    to the stance velocity. In the body frame a stance foot moves at
    +2*half per duty-fraction of cycle; matching that at both ends means
    the foot leaves the floor and lands with no velocity jump in the
    world, which is what keeps it from scuffing on the way off and
    stabbing on the way down.

    It overshoots +half a little at the start (the foot keeps going with
    the body while it lifts) -- that is intended, and plan() checks the
    legs can reach it.
    """
    m = 2.0 * half * (1.0 - duty) / duty      # stance velocity, in w units
    p0, p1 = half, -half
    h00 = 2 * w**3 - 3 * w**2 + 1
    h10 = w**3 - 2 * w**2 + w
    h01 = -2 * w**3 + 3 * w**2
    h11 = w**3 - w**2
    return h00 * p0 + h10 * m + h01 * p1 + h11 * m


def swing_overshoot(duty):
    """How far past the stance extremes the swing foot goes, as a
    multiple of `half`: max |swing_s| / half.

    swing_s is linear in `half`, so this depends on duty alone. The legs
    have to reach `half` times this, not just `half` -- a stride sized
    to the stance cap alone fails the moment the swing overshoots it.
    """
    return max(abs(swing_s(i / 400.0, 1.0, duty)) for i in range(401))


def gait_frame(phase, legs, *, half, duty, lift, height, r_foot, heading,
               planted=False):
    """Every leg's pose at `phase` (cycles, any real number).

    `planted=True` drops the lift from any swinging leg, i.e. "put the
    feet down where they are" -- the abort pose.

    Returns ({leg: pose}, {leg: "stance"|"swing"}).
    """
    frame, state = {}, {}
    for leg in legs:
        u = (phase + leg_offset(leg)) % 1.0
        if u < duty:
            s = -half + 2.0 * half * (u / duty)
            frame[leg] = stance_pose(leg, s, height, r_foot, heading)
            state[leg] = "stance"
        else:
            w = (u - duty) / (1.0 - duty)
            pose = stance_pose(leg, swing_s(w, half, duty), height, r_foot,
                               heading)
            if not planted:
                # (1 - cos 2πw)/2, not sin πw: same peak speed, but it
                # starts and ends at rest, so there is no kick at
                # lift-off or touchdown. Rising gently is safe here
                # because the Hermite above has the foot standing still
                # in the world at that moment -- nothing to scuff against.
                pose["femur"] += lift * 0.5 * (1.0 - math.cos(2 * math.pi * w))
            frame[leg] = pose
            state[leg] = "swing"
    return frame, state


def phase_at(t, period, n_cycles):
    """Cycle phase at time t: ramp up over one period, cruise, ramp down.

    The cycle rate rises linearly from zero to 1/period over the first
    period (covering half a cycle), holds, and falls back to zero over
    the last. Total phase is exactly n_cycles, so the run ends at phase
    0 again -- both tripods down -- and takes (n_cycles + 1) periods.
    """
    total = (n_cycles + 1) * period
    if t <= 0:
        return 0.0
    if t < period:
        return 0.5 * (t / period) ** 2
    if t < n_cycles * period:
        return 0.5 + (t - period) / period
    if t < total:
        tau = (total - t) / period
        return n_cycles - 0.5 * tau * tau
    return float(n_cycles)


# =============================================================
# PLANNING
# =============================================================
def plan(args, legs):
    """Work out the cycle count and stride, then fly the whole run on
    paper at the frame rate: every pose solvable, every coxa in limit,
    and the peak joint speed and acceleration each setting needs."""
    base = pose_stand(args.height, args.foot_out)
    stand = pose_stand(args.height, STAND_FOOT_OUT)
    r_foot = planted_radius(base)
    cap = max_half_stride(args.height, r_foot, args.coxa_limit,
                          args.heading, legs)
    if cap <= 0:
        raise Unreachable(f"no usable stride at {args.height:g} mm on "
                          f"heading {args.heading:g}")
    # The swing carries the foot past the stance extremes, so the stride
    # the legs can really hold is the stance cap shrunk by that overshoot.
    swing_cap = cap / swing_overshoot(args.duty)

    # The body moves 2*half while a foot is in stance, which is `duty` of
    # a cycle, so one cycle carries the body 2*half/duty.
    half_max = min(args.stride / 2.0, swing_cap)
    per_cycle_max = 2.0 * half_max / args.duty
    n_cycles = max(1, math.ceil(args.distance / per_cycle_max))
    per_cycle = args.distance / n_cycles
    half = per_cycle * args.duty / 2.0

    common = dict(half=half, duty=args.duty, lift=args.lift,
                  height=args.height, r_foot=r_foot, heading=args.heading)
    dt = 1.0 / args.rate
    total = (n_cycles + 1) * args.period
    n_frames = int(math.ceil(total / dt)) + 1

    prev = prev_v = None
    peak_v = {name: 0.0 for _, name in JOINTS}
    peak_a = {name: 0.0 for _, name in JOINTS}
    worst_coxa = 0.0
    for i in range(n_frames):
        ph = phase_at(min(i * dt, total), args.period, n_cycles)
        try:
            frame, _ = gait_frame(ph, legs, **common)
        except Unreachable as e:
            raise Unreachable(f"at phase {ph:.2f}: {e}")
        # Differenced in float degrees, not rounded counts: rounding to
        # whole counts turns a smooth curve into a staircase, and its
        # second difference is mostly noise.
        for leg, p in frame.items():
            worst_coxa = max(worst_coxa, abs(_normalise(p["coxa"])))
        if prev is not None:
            v = {}
            for leg in legs:
                for _dg, name in JOINTS:
                    vel = (frame[leg][name] - prev[leg][name]) / dt * STEPS_PER_DEG
                    v[(leg, name)] = vel
                    peak_v[name] = max(peak_v[name], abs(vel))
                    if prev_v is not None:
                        acc = abs(vel - prev_v[(leg, name)]) / dt
                        peak_a[name] = max(peak_a[name], acc)
            prev_v = v
        prev = frame

    if worst_coxa > args.coxa_limit:
        raise Unreachable(f"swing needs {worst_coxa:.0f} deg of coxa, "
                          f"limit {args.coxa_limit:g}")

    top_v = max(peak_v.values())
    top_a = max(peak_a.values())
    # Servo speed ceiling: comfortably above what the stream asks for, so
    # the servo is never the thing holding a joint back.
    servo_speed = int(min(3400, max(600, top_v * 1.5)))
    # Acceleration: enough to follow the stream with headroom. A low value
    # here makes the servo lag the trajectory; 0 is unlimited, which lets
    # every goal step land as a kick.
    #
    # But the register stops at ACCEL_MAX, and a short period needs more
    # than that. Pinning it there would cap the servo silently, and it
    # would fall behind the trajectory with nothing in the plan to say so.
    # So past the ceiling it goes unlimited instead, and the smoothness
    # has to come from the stream: the kicks are one frame's worth of
    # motion, which is why a faster --rate matters then.
    accel_wanted = max(1, math.ceil(top_a * 1.5 / 100.0))
    accel = 0 if accel_wanted > ACCEL_MAX else accel_wanted

    return {"base": base, "stand": stand, "r_foot": r_foot, "cap": cap,
            "swing_cap": swing_cap, "half": half,
            "n_cycles": n_cycles, "per_cycle": per_cycle, "total": total,
            "speed_mm_s": per_cycle / args.period, "peak_v": peak_v,
            "peak_a": peak_a, "top_v": top_v, "servo_speed": servo_speed,
            "accel": accel, "accel_wanted": accel_wanted,
            "worst_coxa": worst_coxa, "common": common}


# =============================================================
# BUS
# =============================================================
def sync_write_goals(ports_ids, targets):
    """One sync-write packet per URT carrying every goal on it.

    Built by hand rather than through the SDK's GroupSyncWrite, whose
    constructor signature differs between feetech-servo-sdk releases.
    The frame is the documented one (instruction 0x83, as in
    software.old/crabora_bus.py):

        FF FF FE <len> 83 <addr> <2>  <id> <lo> <hi>  ...  <checksum>

    No servo replies to a broadcast, so this costs one write per port
    and no waiting.
    """
    for port, sids in ports_ids.items():
        params = bytearray([ADDR_GOAL_POSITION, 2])
        for sid in sids:
            v = targets[sid]
            params += bytes([sid, v & 0xFF, (v >> 8) & 0xFF])
        body = bytes([BROADCAST_ID, len(params) + 2, INST_SYNC_WRITE]) + params
        port.writePort(b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF]))
        for sid in sids:
            note_goal(sid, targets[sid])     # passive; see gait_lib


class Streamer:
    """Holds the bus wiring for one run: which servo is on which port,
    each joint's firmware limits, and how many goals got clamped.

    Also keeps the last goal and commanded speed of every servo, so a
    position read mid-stream can be turned into a tracking error -- the
    one thing an open-loop stream otherwise never learns.
    """

    def __init__(self, packet, joints, legs, limits):
        self.packet = packet
        self.joints = joints
        self.legs = legs
        self.limits = limits
        self.by_port = {}
        self.where = {}
        for leg in legs:
            for dg, name in JOINTS:
                sid, port = joints[dg][leg]
                self.by_port.setdefault(port, []).append(sid)
                self.where[sid] = (port, name)
        self.sids = sorted(self.where)
        self.clamped = 0
        self.goal = {}           # {sid: counts} last sent
        self.speed = {}          # {sid: steps/s} between the last two frames
        self.sent_at = None
        self.lag = {name: [] for _, name in JOINTS}   # [(|err|, |speed|)]

    def counts(self, frame):
        out = {}
        for leg, pose in frame.items():
            c = pose_counts(pose)
            for dg, _ in JOINTS:
                sid, _port = self.joints[dg][leg]
                want = c[dg]
                got = clamp(want, self.limits[(leg, dg)])
                if got != want:
                    self.clamped += 1
                out[sid] = got
        return out

    def stream(self, frame):
        targets = self.counts(frame)
        sync_write_goals(self.by_port, targets)
        now = time.monotonic()
        if self.sent_at is not None and now > self.sent_at:
            dt = now - self.sent_at
            for sid, c in targets.items():
                self.speed[sid] = (c - self.goal[sid]) / dt
        self.goal = targets
        self.sent_at = now

    def sample_lag(self, i):
        """Read where the i-th servo (round-robin) actually is, against
        the goal it was just sent.

        The goal went out this same frame, so even a servo that tracks
        perfectly reads about one frame behind. Error / commanded speed
        turns that into a time, which is comparable across joints and
        settings; anything well past one frame is the servo falling
        behind.
        """
        sid = self.sids[i % len(self.sids)]
        if sid not in self.goal:
            return
        port, name = self.where[sid]
        # read_status, not a bare position read: the same round trip
        # brings back the servo's current for agentix's power view.
        pos = read_status(self.packet, port, sid, "position")["pos"]
        self.lag[name].append((abs(pos - self.goal[sid]),
                               abs(self.speed.get(sid, 0.0))))

    def lag_report(self):
        """{joint: {"n", "max_deg", "mean_deg", "lag_ms"}} for joints read.
        lag_ms is None when the joint was never caught moving."""
        out = {}
        for name, samples in self.lag.items():
            if not samples:
                continue
            errs = [e for e, _ in samples]
            moving = [e / v * 1000.0 for e, v in samples if v >= LAG_MIN_SPEED]
            out[name] = {
                "n": len(samples),
                "max_deg": round(max(errs) / STEPS_PER_DEG, 1),
                "mean_deg": round(sum(errs) / len(errs) / STEPS_PER_DEG, 1),
                "lag_ms": round(sum(moving) / len(moving)) if moving else None,
            }
        return out

    def move(self, frame):
        """walk.py-style: send and wait. Used outside the stream only."""
        group, targets = {}, {}
        for leg, pose in frame.items():
            c = pose_counts(pose)
            for dg, _ in JOINTS:
                group[(leg, dg)] = self.joints[dg][leg]
                targets[(leg, dg)] = clamp(c[dg], self.limits[(leg, dg)])
        move_group(self.packet, group, targets, LOADED_TOLERANCE)

    def set_motion(self, speed, accel=None):
        for leg in self.legs:
            for dg, _ in JOINTS:
                sid, port = self.joints[dg][leg]
                write2(self.packet, port, sid, ADDR_GOAL_SPEED, speed, "speed")
                if accel is not None:
                    write1(self.packet, port, sid, ADDR_ACCELERATION, accel,
                           "acceleration")

    def save_accel(self):
        """Remember each servo's acceleration, so the run leaves them as
        found. Nothing else in walk/ sets this register, so whatever it
        holds is what walk.py and stand.py have been tuned against."""
        self.saved_accel = {}
        for leg in self.legs:
            for dg, _ in JOINTS:
                sid, port = self.joints[dg][leg]
                self.saved_accel[sid] = (port, read1(
                    self.packet, port, sid, ADDR_ACCELERATION, "acceleration"))

    def restore_accel(self):
        for sid, (port, value) in getattr(self, "saved_accel", {}).items():
            write1(self.packet, port, sid, ADDR_ACCELERATION, value,
                   "acceleration")

    def volts(self, leg):
        sid, port = self.joints[FEMUR_JOINT_DIGIT][leg]
        return read1(self.packet, port, sid, ADDR_PRESENT_VOLTAGE,
                     "voltage") / 10.0


def reposition(st, current, target, lift):
    """Get from one six-leg pose to another without dragging feet:
    one tripod at a time lifts, moves, and plants while the other holds.
    """
    for tripod in (TRIPOD_A, TRIPOD_B):
        group = [l for l in st.legs if l in tripod]
        if all(current[l] == target[l] for l in group):
            continue
        frame = dict(current)
        for l in group:
            frame[l] = dict(current[l], femur=current[l]["femur"] + lift)
        st.move(frame)
        for l in group:
            frame[l] = dict(target[l], femur=target[l]["femur"] + lift)
        st.move(frame)
        for l in group:
            frame[l] = target[l]
        st.move(frame)
        current = frame
    return current


# =============================================================
# THE RUN
# =============================================================
def run_scurry(st, p, args, *, should_cancel=None, log=print):
    """Set up, stream the gait, wind down.

    Returns {"travelled", "elapsed", "min_volts", "late", "stopped", "lag"}
    where `stopped` is None for a clean finish or the reason it ended, and
    `lag` is Streamer.lag_report().
    """
    common = p["common"]
    legs = st.legs
    # From the stand pose, not the gait's own width: the feet are wherever
    # stand.py left them, and reposition() is what moves them in.
    base = {l: p["stand"] for l in legs}

    log("\n  -> into the gait, one tripod at a time")
    st.save_accel()
    st.set_motion(SETUP_SPEED)
    start, _ = gait_frame(0.0, legs, **common)
    reposition(st, base, start, args.lift)

    log(f"  -> streaming at {args.rate:g} Hz: servo speed "
        f"{p['servo_speed']}, acceleration {p['accel']}")
    st.set_motion(p["servo_speed"], p["accel"])

    dt = 1.0 / args.rate
    total = p["total"]
    min_volts = None
    late = 0
    stopped = None
    phase = 0.0
    volt_leg = 0
    next_volt = 0.0
    lag_i = 0
    next_lag = 0.0
    t0 = time.monotonic()
    next_tick = t0
    last_cycle = -1
    try:
        while True:
            now = time.monotonic()
            t = now - t0
            phase = phase_at(t, args.period, p["n_cycles"])
            frame, _ = gait_frame(phase, legs, **common)
            st.stream(frame)
            if t >= total:
                break

            cycle = int(phase)
            if cycle != last_cycle:
                last_cycle = cycle
                log(f"  cycle {cycle + 1}/{p['n_cycles']}  "
                    f"{phase * p['per_cycle']:.0f} mm"
                    + (f"  {min_volts:.1f} V low" if min_volts else ""))

            if t >= next_volt:
                next_volt = t + VOLT_EVERY
                v = st.volts(legs[volt_leg % len(legs)])
                volt_leg += 1
                min_volts = v if min_volts is None else min(min_volts, v)
                if v < args.min_volts:
                    stopped = (f"supply fell to {v:.1f} V (limit "
                               f"{args.min_volts:g})")
                    break
            elif t >= next_lag:
                # Never in the same frame as a voltage read: two round
                # trips in one tick is how a frame goes out late.
                next_lag = t + LAG_EVERY
                st.sample_lag(lag_i)
                lag_i += 1

            if should_cancel is not None and should_cancel():
                stopped = "cancelled"
                break

            next_tick += dt
            wait = next_tick - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            else:
                late += 1
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        stopped = "interrupted"
    except (BusError, SerialException, OSError) as e:
        stopped = f"bus error: {e}"

    travelled = phase * p["per_cycle"]
    elapsed = time.monotonic() - t0

    if stopped:
        # Feet down where they are, slowly, and hold. Not back to the
        # neutral stance: if the supply is the reason, the less asked of
        # it now the better.
        log(f"\n  STOPPING: {stopped}")
        log("  -> planting the swing feet and holding, torque ON")
        try:
            st.set_motion(SETUP_SPEED)
            st.restore_accel()
            down, _ = gait_frame(phase, legs, planted=True, **common)
            st.move(down)
        except (BusError, SerialException, OSError) as e:
            log(f"  could not plant: {e}")
    else:
        log("\n  -> back to neutral stance, one tripod at a time")
        st.set_motion(SETUP_SPEED)
        st.restore_accel()
        end, _ = gait_frame(float(p["n_cycles"]), legs, **common)
        reposition(st, end, base, args.lift)

    return {"travelled": travelled, "elapsed": elapsed,
            "min_volts": min_volts, "late": late, "stopped": stopped,
            "lag": st.lag_report()}


# =============================================================
# MAIN
# =============================================================
def print_plan(p, args):
    name = {0: "FORWARD", 180: "BACKWARD",
            90: "LEFT", 270: "RIGHT"}.get(args.heading % 360,
                                          f"on heading {args.heading:g}°")
    print(f"CRABORA scurry -- {args.distance:g} mm {name}")
    print(f"  {p['n_cycles']} cycle(s) of {args.period:g} s, "
          f"{p['per_cycle']:.0f} mm per cycle, stance stride "
          f"{2 * p['half']:.0f} mm (legs can hold +/-{p['swing_cap']:.0f} "
          f"once the swing overshoot is allowed for)")
    print(f"  cruise speed {p['speed_mm_s']:.0f} mm/s; whole run "
          f"{p['total']:.1f} s including one period of ramp at each end")
    print(f"  duty {args.duty:g} (each tripod on the floor "
          f"{args.duty * 100:.0f}% of the cycle), lift {args.lift:g}°, "
          f"{args.rate:g} frames/s")
    print(f"  {args.height:g} mm deck = {ground_clearance(args.height):.0f} mm "
          f"under the caps, ~{tripod_gap(args.height):.0f} mm on one tripod")
    print(f"  worst coxa {p['worst_coxa']:.1f}° of {args.coxa_limit:g}°")
    print(f"\n  {'joint':<6} {'peak speed':>18} {'peak accel':>20}")
    for _, name in JOINTS:
        v = p["peak_v"][name]
        print(f"  {name:<6} {v:>8.0f} steps/s {v * 360 / 4096:>4.0f}°/s"
              f" {p['peak_a'][name]:>12.0f} steps/s²")
    if p["accel"]:
        print(f"\n  servo speed limit {p['servo_speed']}, acceleration "
              f"{p['accel']} (x100 steps/s²)")
    else:
        print(f"\n  servo speed limit {p['servo_speed']}, acceleration "
              f"UNLIMITED: the stream needs {p['accel_wanted']}, over the "
              f"register's {ACCEL_MAX}")
        if args.rate < 100:
            print(f"  ⚠ with no acceleration limit each frame lands as a "
                  f"step; --rate 100 halves them")
    print(f"  stops if the supply reads below {args.min_volts:g} V")


def main():
    ap = argparse.ArgumentParser(
        description="Walk quickly: walk.py's gait, streamed on a clock.")
    ap.add_argument("distance", type=float, help="how far to travel, in mm")
    ap.add_argument("--heading", type=float, default=0.0,
                    help="0 forward, 180 back, 90 left, 270 right")
    ap.add_argument("--period", type=float, default=DEFAULT_PERIOD,
                    help=f"seconds per full cycle (default {DEFAULT_PERIOD:g}); "
                         f"shorter is faster")
    ap.add_argument("--stride", type=float, default=DEFAULT_STRIDE,
                    help=f"mm each stance foot covers (default "
                         f"{DEFAULT_STRIDE:g})")
    ap.add_argument("--duty", type=float, default=DEFAULT_DUTY,
                    help=f"fraction of the cycle a tripod is down "
                         f"(default {DEFAULT_DUTY:g})")
    ap.add_argument("--lift", type=float, default=DEFAULT_LIFT,
                    help=f"femur degrees of swing lift (default "
                         f"{DEFAULT_LIFT:g})")
    ap.add_argument("--height", type=float, default=DEFAULT_HEIGHT)
    ap.add_argument("--foot-out", type=float, default=DEFAULT_FOOT_OUT,
                    help=f"mm outboard of the femur axis while scurrying "
                         f"(default {DEFAULT_FOOT_OUT:g}; stand.py uses "
                         f"{STAND_FOOT_OUT:g})")
    ap.add_argument("--coxa-limit", type=float, default=DEFAULT_COXA_LIMIT)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE,
                    help=f"frames per second (default {DEFAULT_RATE:g})")
    ap.add_argument("--min-volts", type=float, default=DEFAULT_MIN_VOLTS,
                    help=f"stop below this supply voltage (default "
                         f"{DEFAULT_MIN_VOLTS:g})")
    ap.add_argument("--allow-fast", action="store_true",
                    help=f"run even if a joint needs more than "
                         f"{MAX_FOLLOW_SPEED} steps/s")
    ap.add_argument("--legs", type=int, nargs="+", default=list(ALL_LEGS))
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.add_argument("--force", action="store_true",
                    help="walk even if the measured stance is too low")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan, touch nothing")
    ap.add_argument("--ports", nargs="+")
    args = ap.parse_args()

    legs = sorted(set(args.legs))
    if any(l not in ALL_LEGS for l in legs):
        ap.error("legs must be 1-6")
    if args.distance <= 0:
        ap.error("distance must be positive")
    if not 0.5 <= args.period <= 20:
        ap.error("--period must be 0.5-20 s")
    if not 0 < args.stride <= 150:
        ap.error("--stride must be 0-150 mm")
    if not 0.5 <= args.duty < 0.9:
        ap.error("--duty must be 0.5-0.9 (below 0.5 there are moments "
                 "with fewer than three feet down)")
    if not 0 <= args.lift <= 60:
        ap.error("--lift must be 0-60 degrees")
    if not 10 <= args.rate <= 100:
        ap.error("--rate must be 10-100 Hz")

    try:
        p = plan(args, legs)
    except Unreachable as e:
        print(f"Cannot scurry like that: {e}")
        return 1
    print_plan(p, args)

    if p["top_v"] > MAX_FOLLOW_SPEED:
        print(f"\n  ⚠ a joint needs {p['top_v']:.0f} steps/s, more than the "
              f"{MAX_FOLLOW_SPEED} a loaded STS3215 can be trusted to follow.")
        print(f"    Lengthen --period, shorten --stride or --lift, or pass "
              f"--allow-fast.")
        if not args.allow_fast:
            return 1

    if args.dry_run:
        print("\n  Dry run -- nothing was sent to the bus.")
        return 0

    devices = args.ports or find_urt_devices()
    if not devices:
        print("No FE-URT-2 boards found.")
        return 1
    if require_bus_free(args.force):
        return 1
    ports = open_ports(devices)
    if not ports:
        print("No URT could be opened.")
        return 1

    packet = PacketHandler(0)  # STS servos use protocol_end=0
    reset_bus_stats()
    try:
        joints = {dg: find_joints(packet, ports, legs, dg, quiet=True)
                  for dg, _ in JOINTS}
        legs = [l for l in legs if all(l in joints[dg] for dg, _ in JOINTS)]
        in_a = [l for l in legs if l in TRIPOD_A]
        in_b = [l for l in legs if l in TRIPOD_B]
        print(f"\n  tripod A: {in_a}   tripod B: {in_b}")
        if not in_a or not in_b:
            print("  ⚠ one tripod is empty -- it cannot alternate. Stopping.")
            return 1

        limits = {}
        for leg in legs:
            for dg, _ in JOINTS:
                sid, port = joints[dg][leg]
                limits[(leg, dg)] = (
                    read2(packet, port, sid, ADDR_MIN_ANGLE, "min"),
                    read2(packet, port, sid, ADDR_MAX_ANGLE, "max"))
                write1(packet, port, sid, ADDR_TORQUE_ENABLE, 1, "torque on")

        st = Streamer(packet, joints, legs, limits)
        v0 = min(st.volts(l) for l in legs)
        print(f"  supply {v0:.1f} V at rest")
        if v0 < args.min_volts + 0.3:
            print(f"  ⚠ already within 0.3 V of the {args.min_volts:g} V "
                  f"stop. Charge the battery first.")
            return 1

        measured = measure_stance(packet, joints, legs)
        if measured and min(measured.values()) < args.height - 25.0:
            lo = min(measured.values())
            print(f"\n  ⚠ the body is at {lo:.0f} mm, the gait assumes "
                  f"{args.height:g}. Stand first.")
            if not args.force:
                return 1

        if args.pause and not confirm(
                f"Standing, clear of obstacles, and ready to scurry "
                f"{args.distance:g} mm at {p['speed_mm_s']:.0f} mm/s?"):
            print("\n  Stopped.")
            return 1

        out = run_scurry(st, p, args)

        retries, failures = bus_stats()
        print(f"\n  Commanded {out['travelled']:.0f} mm in "
              f"{out['elapsed']:.1f} s"
              + (f" ({out['travelled'] / out['elapsed']:.0f} mm/s)"
                 if out["elapsed"] > 0 else ""))
        if out["min_volts"] is not None:
            print(f"  lowest supply reading {out['min_volts']:.1f} V "
                  f"(started {v0:.1f})")
        if out["late"]:
            print(f"  {out['late']} frame(s) sent late -- the loop could "
                  f"not keep up with {args.rate:g} Hz")
        if out["lag"]:
            frame_ms = 1000.0 / args.rate
            print(f"\n  tracking (one frame is {frame_ms:.0f} ms; a servo "
                  f"that keeps up reads about that far behind):")
            print(f"  {'joint':<6} {'reads':>5} {'max err':>8} "
                  f"{'mean err':>9} {'lag':>8}")
            for _, name in JOINTS:
                r = out["lag"].get(name)
                if r:
                    lag = f"{r['lag_ms']} ms" if r["lag_ms"] is not None else "-"
                    print(f"  {name:<6} {r['n']:>5} {r['max_deg']:>7.1f}° "
                          f"{r['mean_deg']:>8.1f}° {lag:>8}")
        if st.clamped:
            print(f"  {st.clamped} goal(s) clamped to a firmware limit")
        if retries or failures:
            print(f"  bus: {retries} retries, {failures} failures")
        print("  Open loop: measure the real distance with a tape. A short "
              "fall is\n  slip or servo lag, and lag shows first as the "
              "period shrinks.")
        return 1 if out["stopped"] else 0

    except KeyboardInterrupt:
        print("\n\n  Interrupted -- stopped where it stands, torque ON.")
        return 130
    except BusError as e:
        print(f"\nBus error: {e}")
        return 1
    finally:
        for port in ports.values():
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
