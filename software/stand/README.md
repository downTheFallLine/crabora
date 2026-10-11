# CRABORA stand

Getting from an arbitrary, possibly tangled pose to standing.

The hard version of the problem: all 18 servos are somewhere unknown,
a leg may be folded under the body, and a servo may be bound against
structure. This directory holds the recovery strategy and the code that
implements it.

## The strategy

### 1. You are never actually blind — survey first

"Unknown position" overstates it. The STS3215 has an absolute encoder.
Torque off, read register 56 on all 18 servos, and every joint angle is
known before anything moves. Add present load (60), temperature (63) and
the error register (65) and you also know which servos are latched in
overload and will ignore commands until cleared.

With the link geometry (`walk/gait_lib.py`) plus forward kinematics,
joint angles give you **where each foot actually is**.

### A leg cannot fold under the body

Worth knowing before designing around it: running the numbers, **it is
geometrically impossible for a foot to get under the deck.** The knee
(femur axis) sits at radius 222 mm from the body centre — 163.22 out to
the coxa axis plus the 58 mm coxa segment — which is outside even the
deck's 210 mm circumradius, let alone its 181.8 mm flats. Nothing distal
to the knee can reach underneath. The most folded pose reachable puts
the foot at about 203 mm radial, still ~21 mm clear of the deck edge.

So the tangle risk is **leg against neighbour**, not leg under body. At
the nominal straight-out pose adjacent feet are ~440 mm apart; the
survey flags any pair closer than 120 mm. That is also why the untangle
sequence works one leg at a time — the thing a leg can collide with is
the leg next to it.

The survey is free and risk-free. Nothing moves. `survey.py`.

### 2. Belly-down is the safe state

A collapsed robot is a safe robot. With the body resting on its belly,
the ground carries the weight and every leg is unloaded and free to be
repositioned. Every dangerous case involves the body partly supported by
legs, where moving one drops weight onto the others.

So do not recover *toward* standing. Deliberately go belly-down first,
confirm the body is resting, and untangle from there. A few seconds buys
a state where nothing you do can topple anything.

### 3. Untangle one leg at a time, compact before you swing

**Across legs: strictly sequential.** Six legs moving at once in an
unknown configuration is six chances for neighbours to fight each other.
One at a time avoids it entirely, and keeps current draw trivial.

**Within a leg: LIFT first, then fold.**

⚠ This ordering was wrong in the first version of this document, and it
broke a tibia on 2026-10-11. The original text said "flex the tibia,
then lift the femur". Do not restore it. With the body belly-down the
floor sits **62.6 mm below the femur axis**, and flexing a tibia while
the femur is horizontal swings the foot *downward* — 100° of flex puts
it 118 mm down, i.e. 56 mm into the floor. The servo pushes until
something gives.

1. **Lift the femur** — clear of the ground and the body. Raising is
   always safe; it can only move a foot away from the floor.
2. **Flex the tibia** — fold the foot in, making the leg short. Safe
   now, because the knee is already high.
3. **Rotate the coxa** to its target bearing — now a short stub is
   sweeping, not a long lever that clips its neighbours.
4. **Extend** femur and tibia to place the foot.

Swinging a fully extended leg around the hexagon is how things break.
Compact-then-rotate is the whole trick.

The target is a known **tuck pose**: foot on the ground, out to the
side, roughly under the hip, body still on its belly. Once all six are
tucked, standing is a planned motion rather than a recovery.

### 3a. Never plan a joint order by intuition

The tibia breakage came from ordering joints by what sounded sensible.
The geometry does not care what sounds sensible. `pose.py` therefore
provides the check and the planner:

- `foot_height(femur, flex)` — height above the floor, belly-down
- `floor_strikes(a, b)` — samples the path the servos actually take
  between two poses and returns every point that would hit. Note the
  servos move at equal *count* rate, so the joint with less travel
  arrives first and waits; that is not a straight line in joint space,
  and the difference is exactly what bit us.
- `safe_sequence(a, b)` — waypoints that avoid a strike, or `None` if
  it cannot find any, in which case the caller must stop rather than
  issue the move anyway.

A leg already reading as underground is not an error — it means the body
is not resting where the model assumes, or the leg is propping it up. So
the rule is **never go lower than you started**, not "always stay above
the floor".

`collapse` uses a fixed safe order for this reason: raise every femur to
at least 45° (reading each one first, so it only ever *raises*),
straighten the tibias while they are up there, and lower the femurs
last.

### 4. Binds, and the shake

The safety net is the **torque limit register**, capped low throughout
recovery. A jammed servo then simply fails to move, instead of stripping
gears, browning out the supply or levering the frame apart. Raise the
cap only for the final lift, which genuinely needs force.

**Detecting a bind:** command a small move; high present load with no
change in present position means bound. More reliable than a timeout,
which cannot tell "stuck" from "slow".

**The shake escalates, then gives up:**

- small oscillation, a few degrees, several cycles — often enough, since
  static friction exceeds kinetic and a reversal relieves a wedge
- then larger amplitude
- then the *opposite* direction from wherever it was heading, since a jam
  usually has a way back out
- then stop and report. Never escalate torque to force it.

Caveat: if a leg is genuinely wedged under the body, oscillating it
shakes the *robot*. Harmless on the test stand, but it could rock the
machine over on the ground.

### 5. Then stand

From tuck this is the easy part: coordinated symmetric push, slow, in
stages with a pause between, watching current. This is the one phase
where legs **must** move together, or the body tips.

## Failure policy

**Continue with five legs and report.** (Tom, 2026-10-11.) A leg that
cannot be freed is excluded from the remaining phases and named in the
summary, rather than aborting the whole routine. The run's exit code
reflects that something was left behind.

## Prerequisites and their status

| | Status |
|---|---|
| Travel limits in servo firmware | ✓ set 2026-10-11 |
| Belly-down ride height | ✓ measured: coxa horn 45 mm above floor |
| Link lengths | ✓ confirmed with calipers 2026-10-11 |
| Coxa sign | ✓ verified on hardware |
| Femur sign | ✓ verified on hardware |
| **Tibia sign** | ⚠ **TBD — ASSUMED** |

### ⚠ The tibia sign is assumed, not verified

`TIBIA_EXTEND_SIGN` in `walk/gait_lib.py` is `+1` as a first guess and
has never been run on hardware. Tom's call on 2026-10-11 was to proceed
on the assumption and come back to it.

**What that means for this code:** anything depending on tibia
*direction* is provisional. That specifically includes step 1 of the
untangle sequence — "flex the tibia" is the very first move made on a
tangled leg, and if the sign is backwards it will **extend into**
whatever the leg is caught on instead of pulling clear.

Until it is verified, run recovery with the tibia excluded, or with the
body on the test stand where a wrong move is harmless. Settle it with:

```
python3 ../walk/proto_walk_tibia.py --legs 1 4 --flex 20
```

## Layout

- `survey.py` — phase 0 standalone. Reads all 18 joints, computes foot
  positions, flags binds, latched errors and out-of-range joints.
  Moves nothing.
- `pose.py` — leg kinematics: two-link IK in the leg's vertical plane,
  its inverse, and the named poses (splay, compact, tuck, stand). Pure
  geometry, no bus; run it directly for a self-check.
- `stand.py` — the recovery itself, all four phases. `--phase` runs one
  at a time, `--dry-run` prints the plan without touching the bus.

### Travel limits must be generous

The recovery poses need far more travel than a gait does — `tuck` alone
is about 56° of femur lift and 129° of tibia flex. `stand.py` runs a
**preflight check** that compares every pose against the firmware limits
it reads, and names each joint that would clamp, before anything moves.

If that check reports clamping, widen the limits with `set_center.py`.
Otherwise the robot stops short at every pose and the stand quietly
never completes.

Geometry, signs and bus plumbing are imported from `walk/gait_lib.py`,
so "which way is forward" stays defined in exactly one place. That makes
`stand/` depend on `walk/`, which is backwards; if this grows further,
`gait_lib` should move up to `software/`.
