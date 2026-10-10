# CRABORA walk

Main gait and locomotion code. Starting fresh — the previous generation
(`walk.py` … `walk6.py`, `tripod_walk.py`, `stand*.py`, `take_step.py`,
`teach_pose.py`) was deleted on 2026-10-10 and is recoverable from git
history if a specific idea is worth retrieving.

This document fixes the conventions everything in here depends on.
Get these wrong and a gait that is mathematically perfect will walk
backwards, or tear a leg off against its neighbour.

## Body geometry

The body is a hexagon with one leg on each of the six flats, so a
**hexagon vertex (a "point") falls between each adjacent pair of legs**.

The **front of the robot is the point between leg 1 and leg 6.**

Legs are numbered **1 … 6 clockwise, viewed from above.**

```
                   FRONT
                     ·  ← front point, between legs 6 and 1
                    ╱ ╲
          6 ·                 · 1
          (330°)             (30°)

      5 ·                         · 2
      (270°)                     (90°)

          4 ·                 · 3
          (210°)             (150°)
                    ╲ ╱
                     ·  ← rear point, between legs 3 and 4
```

Bearings above are measured **clockwise from the front point**, which is
the natural reading of "clockwise order": leg *n* sits at
`30° + (n-1) × 60°`.

| Leg | Bearing | Position     |
|-----|---------|--------------|
| 1   | 30°     | front-right  |
| 2   | 90°     | right        |
| 3   | 150°    | rear-right   |
| 4   | 210°    | rear-left    |
| 5   | 270°    | left         |
| 6   | 330°    | front-left   |

The rear point sits between legs 3 and 4, directly opposite the front.

Note there is **no leg on the centreline**, front or back. The robot
leads with a point, not with a leg, so forward motion is symmetric
about the 1/6 and 3/4 pairs.

## Body frame

Right-handed, origin at the body centre on the deck plane:

- **+X** forward, out through the front point
- **+Y** to the **left** (port)
- **+Z** up

So a leg's bearing θ (clockwise from front, as tabulated) maps to a
direction vector `(cos θ, −sin θ, 0)` — the sign on Y because bearings
run clockwise while the frame is right-handed.

This matches the usual robotics convention (ROS REP-103). It is a
choice, not a measurement: if anything else in the codebase already
assumes otherwise, change it **here** and fix the code, rather than
letting two conventions coexist.

## Servo IDs

Two-digit scheme, unchanged from the rest of the project:

```
    ID = leg × 10 + joint        joint: 1 = coxa, 2 = femur, 3 = tibia
```

| Leg | Coxa | Femur | Tibia |
|-----|------|-------|-------|
| 1   | 11   | 12    | 13    |
| 2   | 21   | 22    | 23    |
| 3   | 31   | 32    | 33    |
| 4   | 41   | 42    | 43    |
| 5   | 51   | 52    | 53    |
| 6   | 61   | 62    | 63    |

## Tripod groups

Alternating legs, which with this numbering gives two valid tripods:

- **Tripod A** — legs 1, 3, 5
- **Tripod B** — legs 2, 4, 6

Each group has two legs on one side and one on the other, straddling
the centre of mass. One tripod is always in stance while the other
swings.

## Mirror symmetry

Legs 1–3 are the right side, 4–6 the left. A leg and its mirror
(1↔6, 2↔5, 3↔4) are physically identical parts rotated into place, so
**a positive coxa command swings them in opposite directions in the body
frame.** Gait code must apply a per-side sign, not assume one direction
for all six.

This is the single most likely source of a leg driving into its
neighbour. It bit the previous generation — the tripod gait's coxa
direction was never verified on hardware.

## Still undefined

Deliberately not fixed here yet, because they want measuring on the
assembled robot rather than guessing:

- **Joint zero poses** — what physical pose each joint's 2048 centre
  corresponds to, per joint. Set with `software/utils/set_center.py`.
- **Joint angle signs** — which way positive goes for coxa, femur and
  tibia, and the per-side mirror sign above.
- **Link lengths** — coxa offset, femur and tibia, from the current
  `leg/*/linkage_leg` designs. The old figures (58 / 100 / 120 mm)
  predate the current leg revision and should be re-read from the
  design files, not copied forward.
- **Travel limits** — each joint's safe swing, written into servo
  firmware with `set_center.py` so a bad command is clamped by the
  servo itself.

## Bring-up order

Power is the binding constraint (see the project power notes — the
LM2596 buck tops out around 2–3 A, which is fine for exercising joints
but will not stand the robot). Build up a tier at a time:

1. All six coxas bolted → `software/tests/test_04_coxa_sweep.py`
2. Add femurs → `software/utils/swing_legs.py --mode by-joint`
3. Add tibias → same, then static stance
4. Standing, then walking — needs the power path sorted first
