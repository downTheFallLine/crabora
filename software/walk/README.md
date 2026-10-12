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

**This generation's table is verified.** On 2026-10-10 `tripod_coxa.py`
drove all six coxas together on the test stand, with the two tripods
opposing as intended and no `--invert` needed. `FORWARD_SIGN` in
`gait_lib.py` is correct as written. `--invert` stays as a diagnostic:
if a later run needs it, a servo has been remounted.

## The femur

**Centre (2048) is the femur horizontal**, and the travel envelope is
**±90°** — straight up to straight down. Decided 2026-10-10.

### Why centre is horizontal

The alternative was centre = the neutral stance pose, which makes gait
maths read naturally (swing is a positive offset from zero) at the cost
of lopsided limits. Horizontal was chosen instead: it is a pose you can
identify by eye when calibrating, it is geometrically easy to reason
about, and it makes the limits symmetric. The price is that every stance
command carries a constant offset from centre.

### Up and down are not the same job

Unlike the coxa — where forward and rear are the same action mirrored,
so symmetric travel is natural — the femur's two directions do different
work:

- **Up** — swing clearance (small, 15–25°), and folding into a tuck or
  park pose (large). Also how the robot gets up from a collapsed start.
- **Down** — raises the body above neutral ride height. Useful range
  runs out quickly: as the femur drops the leg straightens, the foot
  pulls inward, and mechanical advantage falls away.

So although the *envelope* is symmetric at ±90°, the *useful* range is
not, and gait code should not assume that +30° and −30° are equally
available or equally cheap. Down is also gravity-assisted while up works
against the leg's weight, so the two directions differ in torque and in
how a move settles.

### Femur sign — verified

The coxa mirror comes from **body geometry**: legs 1 and 6 point
different ways, so "forward" is opposite rotations. The femur has no
equivalent — *lift is the same physical motion on every leg.*

So the femur sign should be **uniform across all six legs, not mirrored
like the coxa.**

**Confirmed on hardware 2026-10-11.** All six femurs moved together, so
the reasoning holds and one sign serves every leg. The run also showed
the direction was backwards from the first guess: **a rising servo count
LOWERS the femur**, so `FEMUR_UP_SIGN` in `gait_lib.py` is −1 everywhere,
not +1.

If a later run shows the two sides splitting — legs 1–3 rising while 4–6
drop — a servo has been remounted and the table would need per-side
values after all.

### Travel limits

Pick them from the hardware, not from this document. With torque off,
swing each femur by hand through its full range and find where it
actually fouls — cap against coxa body, tibia against femur, leg against
neighbour — then set limits a few degrees inside that. `set_center.py`
takes independent left/right values, so asymmetric limits need no new
code:

```
python3 utils/set_center.py 12 50 25     # leg 1 femur: 50° up, 25° down
```

## The tibia

**Centre (2048) is the tibia COLLINEAR WITH THE FEMUR** — the leg dead
straight, zero knee bend. Decided 2026-10-11.

### Why collinear

The deciding factor is **repeatability across six legs**. Centre is set
six times by hand with `set_center.py`, and the legs have to match. A
straight leg can be pressed against a table edge or a straightedge — it
is physically repeatable, not judged by eye. The alternative (tibia
perpendicular to the femur) means eyeballing a right angle six times,
and if leg 3 ends up 5° off leg 1 the robot limps in a way that is
miserable to trace back.

Secondary: zero knee bend = zero is the standard way to parameterise a
knee, so the tibia angle is simply "flexion from straight" for any IK
written later. And it is the same *kind* of reference as
femur-horizontal — a geometric datum you can verify, not a working pose.

### Travel is one-sided

A knee only bends one way, so **straight is the edge of useful travel,
not the middle.** There is almost nothing on the extend side and all of
it on the flex side — roughly 0 to 120–135° of flex. Firmware limits
will be lopsided (something like 5° extend, 120° flex) and that is
correct, not a mistake: nothing requires centre to be centred.

The direct consequence is that **code sweeping this joint symmetrically
is the wrong shape.** `proto_walk_tibia.py` therefore takes separate
`--extend` and `--flex` angles rather than one `--angle`; a symmetric
sweep would clamp against the extend limit on every leg, every rep.

### Tibia sign

`TIBIA_EXTEND_SIGN` is **+1 everywhere, verified on hardware 2026-10-12**
(`tests/test_05_calibration.py --signs --legs 1 4`). Both predictions
from the other two joints held except one:

- **Uniform across all six, not mirrored** — as expected. Legs 1 and 4
  are one per side and both extended, so the sides did not split. Flexing
  a knee is the same physical motion on every leg, exactly as lifting is
  for the femur. The coxa's mirror comes from body geometry and has no
  analogue at the knee.
- **+1, not −1** — the femur needed inverting from its first guess and
  this one did not, so the tibia servos do *not* follow the femur's
  mounting convention.

This was the last unverified joint sign. All three tables in
`gait_lib.py` now match hardware.

## Link geometry

Read from the CAD and confirmed against the built leg with calipers on
2026-10-11. All distances are between **joint axes** — the centre of
each servo horn — which is the kinematic quantity, not part edges.
They live in `gait_lib.py`.

| Parameter | Value | Meaning |
|---|---|---|
| `BODY_TO_COXA_AXIS` | 163.22 mm | body centre → coxa axis, horizontal, along the leg's bearing |
| `COXA_RADIAL` | 58 mm | coxa axis → femur axis, outboard |
| `COXA_TANGENT` | 21 mm | coxa axis → femur axis, **sideways** |
| `COXA_VERTICAL` | 17.6 mm | coxa axis → femur axis, up |
| `FEMUR_LEN` | 100 mm | femur axis → tibia axis |
| `TIBIA_LEN` | 120 mm | tibia axis → foot tip |

### The leg is not a planar 2-link chain

Worth flagging because it is easy to get silently wrong: the femur axis
is **not** directly outboard of the coxa axis. It is displaced 21 mm
sideways and 17.6 mm up. So the femur/tibia plane is offset from the
coxa rotation axis rather than passing through it.

Any IK written against the textbook "yaw joint plus planar 2-link"
model will be subtly wrong — the tangent term has to be carried through.
The error would be small enough to look like calibration drift, which
is the worst kind of bug to chase.

### Measuring these again

If a leg revision changes them, there is an easy repeat procedure now
that the centre conventions are fixed. Command one leg to centre on all
three joints — coxa neutral, femur horizontal, tibia collinear — and
**the leg becomes a straight horizontal line from the femur axis out to
the foot.** Everything is then measurable with a steel rule in one pose,
with no angles to judge. Measure horn-centre to horn-centre; the central
horn screw marks the axis.

## Still undefined

Deliberately not fixed here yet, because they want measuring on the
assembled robot rather than guessing:

- ~~Joint zero poses~~ — all three are now settled: coxa neutral, femur
  horizontal, tibia collinear with the femur. Set them with
  `software/utils/set_center.py`.
- ~~Joint angle signs~~ — all three are settled and verified on
  hardware: coxa 2026-10-10, femur 2026-10-11, tibia 2026-10-12.
- ~~Travel limits~~ — set on hardware as of 2026-10-11. Every joint's
  safe swing is now written into servo firmware, so a bad command is
  clamped by the servo itself rather than by the calling code.

## Bring-up order

Power is the binding constraint (see the project power notes — the
LM2596 buck tops out around 2–3 A, which is fine for exercising joints
but will not stand the robot). Build up a tier at a time:

1. All six coxas bolted → `software/tests/test_04_coxa_sweep.py`
2. Add femurs → `software/utils/swing_legs.py --mode by-joint`
3. Add tibias → same, then static stance
4. Standing, then walking — needs the power path sorted first
