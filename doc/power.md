# Power architecture: PRIPOWER, LOCOPOWER, LOGIPOWER

Crabora's power is three separate domains. Thinking of them separately is
what keeps a servo brownout from looking like a software bug, and a Pi
under-volt from looking like a servo fault.

| Domain | What it is | Feeds |
| --- | --- | --- |
| **PRIPOWER** | The battery world: pack, fuse, alarm, charging | LOCOPOWER and LOGIPOWER |
| **LOCOPOWER** | Locomotion power: the 7.4 V buck and everything downstream to the servos | 18 STS3215 servos, via the 3 FE-URT-2s |
| **LOGIPOWER** | Logic power: the 5 V converter and everything that thinks or senses | Raspberry Pi, IMU, later a camera etc. |

```
                    +--------------------------------------------+
                    |                 PRIPOWER                   |
                    |  3S LiPo --> fuse --> main switch --> split |
                    |     |                                      |
                    |  balance lead --> cell alarm               |
                    +----------------------+---------------------+
                                           |
                     +---------------------+---------------------+
                     |                                           |
          +----------v-----------+                    +----------v-----------+
          |      LOCOPOWER       |                    |      LOGIPOWER       |
          |  buck --> 7.4 V      |                    |  BEC --> 5 V         |
          |   --> 3x FE-URT-2    |                    |   --> Raspberry Pi   |
          |   --> 6 legs x 3     |                    |   --> 3.3 V: IMU     |
          |       STS3215        |                    |   --> (camera, later)|
          +----------+-----------+                    +----------+-----------+
                     |                                           |
                     +-------------- common ground --------------+
```

The two converters are **not** a split load. LOCOPOWER's buck is the whole
ceiling for the legs, and the Pi's draw does not come out of it (see
`doc/uniBody.md`). The domains do share one ground: the URTs talk to the
Pi over USB, so the servo ground and the Pi ground must be common or the
bus misbehaves.

## PRIPOWER: batteries

**Today:** a 2S pack (8.4 V full). Capacity and C rating unconfirmed.

**Planned:** 3S, because LOCOPOWER's buck needs headroom (below).

| Part | Choice | Notes |
| --- | --- | --- |
| Pack | Zeee 3S 2200 mAh 50C, XT60, 2-pack (Amazon B0BYNSH6Q7) | 137 g, 75 x 34 x 26.5 mm. 12.6 V full, ~9.9 V empty. 50C = ~110 A available, far above need |
| Fuse | 20-25 A inline, right after the pack | The buck has no protective current limit of its own worth trusting |
| Wire | 14 AWG pack -> split | |
| Cell alarm | Apex RC #1655 (1-8S, 2-pack) on the balance lead | Set to **3.5 V/cell**. Black wire to the alarm's "-" end |
| Charger | 3S-capable balance charger | Always balance-charge |

Why 3S and not 2S: a buck converter needs roughly 1.5-2 V more in than
out. A 2S pack is 8.4 V full and ~7.4 V near empty, so no buck can hold
7.4 V from it. Today's LM2596 just passes the pack through minus a
volt, and the servos lose torque as the pack drains. A 3S pack stays
above ~9.9 V for its whole discharge.

Rules:

- **Unplug the cell alarm when the robot is put away.** It draws a little
  current constantly and will over-discharge a stored pack.
- Stop at the alarm. Don't run a LiPo below 3.0 V/cell.
- Store LiPos at ~3.8 V/cell (the charger's storage mode), not full.

**Future:** an INA226 on the Pi's I2C bus reading pack voltage, so the
software can see the pack (see the LOCOPOWER gotcha below). Its stock
shunt is only good for ~0.8 A, so it reads voltage only unless the shunt
is swapped for a heavy one.

## LOCOPOWER: servos

**Today:** SELOKY LM2596 buck, set to 7.4 V. Good for ~2 A continuous
(3 A absolute). Enough for coxa and femur bring-up and a short walk; not
enough for reliable walking or scurry.

**Ordered (2026-10-25):** XINGYHENG 20A 300W high-power buck converter
module (Amazon B081X5YX8V), to replace the LM2596. Set to 7.4 V.

| Spec | Value | Source |
| --- | --- | --- |
| Input | DC 6-40 V | listing title |
| Output | DC 1.2-36 V, adjustable | listing title |
| Current / power | 20 A, 300 W | listing title; continuous vs peak not stated |
| Modes | constant voltage + constant current (CV/CC) | listing title |
| Protection | short circuit | listing title |
| Continuous rating, cooling, efficiency, size, weight | **unconfirmed** | check the module and its paperwork when it arrives |

What to expect from this class of module (not confirmed for this one):
"20 A" is usually the peak; ~12-15 A continuous on the bare heatsink, more
only with a fan. Walking (~10-12 A) should be comfortable; scurry peaks
are at the edge, so give it airflow.

**It needs the 3S pack to regulate.** With 6 V minimum input and the usual
~1.5-2 V of headroom, 2S (8.4 V full) at 7.4 V out puts it in dropout
like the LM2596. Install it with the 3S packs, not before.

Setting it up, **before any servo is connected**:

1. Feed it from the 3S pack through the fuse, output open.
2. Turn the CV pot until a meter on the output reads **7.4 V**. Never above
   8.0 V: the STS3215s trip over-voltage at 8 V.
3. Use the CC pot as a current limit, the way the old 5 A bench supply was
   one. Start around **8 A** for bring-up (stand / lay down), raise toward
   15 A once walking is proven on it. (Setting CC means briefly shorting
   the output through a meter on its 10-20 A range; follow the module's
   instructions.)
4. Add a 1000-2200 uF / 16 V+ capacitor across the output to soak up the
   spikes when many servos start at once.
5. Connect the URTs, power up with the robot lying down, and check
   `software/utils/read_servo_voltage.py <id>` reads ~7.4 V.

Path: buck -> each FE-URT-2's power input -> the URT's two bus ports ->
one leg per port -> coxa -> femur -> tibia. 3 URTs x 2 legs = 6 legs.

Estimated load at 7.4 V (from the STS3215 datasheet, not measured):

| Situation | Current |
| --- | --- |
| All 18 idle, unloaded | ~0.4 A |
| Standing still | ~2-3 A |
| Stand up / lay down | ~5-8 A |
| Walking (tripod) | ~10-12 A, peaks ~15 A |
| Scurry | 15 A+ peaks |
| All 18 stalled (theoretical) | 45 A, never happens |

Servo limits (STS3215 datasheet): stall 2.5 A, rated 0.65 A,
over-current cut at >2 A for 2 s, over-voltage trip above 8 V, under-voltage
below 4 V.

Gotchas:

- **Brownout mimics wiring faults.** When the buck tops out, servos drop off
  the bus and IDs vanish intermittently. Suspect LOCOPOWER before the harness.
- **Once the buck regulates, the servos can't see a low pack.** They read a
  steady 7.4 V until the pack collapses, so `motion.py`'s <6.2 V preflight
  stops catching a draining battery. The cell alarm covers this today; the
  INA226 is the software fix.
- **The URTs are the next bottleneck.** Each FE-URT-2 carries two legs,
  ~5-6 A while walking, through connectors rated ~3 A per pin. If URTs run
  warm or errors appear after the buck upgrade, inject power into each leg
  directly and let the URTs carry data only.

Verify the table above with an inline watt meter between the buck and the
URTs, running stand, walk and scurry.

## LOGIPOWER: Pi and sensors

**Today:** a separate BEC set to 5 V -> Raspberry Pi 3. The BEC is an
**LM2596 module, rated up to 40 V input** (confirmed by Tom 2026-10-26),
so it takes the 3S pack (12.6 V full) directly -- no change needed for the
3S upgrade, and with 3S it actually regulates (7+ V of headroom). The IMU
(MPU-6050) runs from the Pi's 3.3 V pin, **not 5 V** (see `doc/imu.md`).
The URTs' USB data connections come from the Pi.

Budget: a Pi 3 wants 5.1 V at up to 2.5 A. An LM2596 is good for about
2 A continuous (3 A absolute), so the Pi alone is near its comfortable
limit under load, and a camera would push past it. Before adding a
camera, swap the BEC for a 5 V / 3 A+ converter (a 5 V UBEC rated 3-5 A,
or an XL4015-class module). Set it to **5.1 V**, not 5.0: the Pi flags
under-voltage below about 4.65 V, and the cable drops some.

Gotchas:

- Under-voltage on the Pi almost always means the cable or connector into
  the Pi, not the Pi. `software/tests/test_01_pi_stability.py` catches it.
- Nothing on the I2C bus or GPIO may see more than 3.3 V. Never connect a
  pack voltage or balance lead to a GPIO pin.

## Open items

- PRIPOWER: buy the 3S packs, charger, fuse, and alarm. The new buck
  needs the 3S packs to regulate.
- LOCOPOWER: install the XINGYHENG 20 A buck when it arrives (ordered
  2026-10-25); confirm its continuous rating and cooling; measure real
  currents with a watt meter.
- agentix: once the new buck regulates, update the agent's system prompt
  (`software/agentix/agent.py`), which still says the servo converter "is
  not regulating".
- LOGIPOWER: the 5 V BEC is an LM2596 (40 V in, ~2 A continuous). Fine
  for the 3S upgrade; replace with a 3 A+ converter before a camera.
- Add the INA226 pack-voltage check to the motion preflight.
