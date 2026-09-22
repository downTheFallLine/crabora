## Notes on Servo

Project is based on Feetech STS3215 servos

### Dimension of servo is:

45.2 mm x 24.7 mm x 35 mm (Length x Width x Height)

### Electrical

Electrical connection from servo is via 3 pin Molex 5264 (power, signal, ground)

Servos are daisy chained within a leg (tibia, femur, coxa)

Coxa is wired upstream to the URT.

### Six-leg interconnect (URT-2)

3x FE-URT-2 boards are used for the 6-leg hexapod. Each FE-URT-2 has two
servo-bus connector ports built in, so it natively joins 2 legs' coxa
cables with no splitter or extra part needed:

- URT-2 #1: legs 1 & 6
- URT-2 #2: legs 3 & 5
- URT-2 #3: legs 2 & 4

(paired by adjacency in `LEG_LAYOUT_CW` for shortest cable runs -- the
software doesn't require this specific pairing)

Power is routed into each FE-URT-2 (not a separate power bus); the
URT-2's bus connectors carry both power and signal downstream to its two
legs. `software/crabora_bus.py`'s `MultiBus` opens every URT the Mac can
see and auto-discovers which servo lives on which URT at runtime, so the
leg-to-URT pairing above doesn't need to be hardcoded anywhere.


