# Design concept of Unibody

Unibody is a Gen2 concept of making the bot have one really large deck, abandoning the earlier 3 deck approach which became unstable. 

Since we're working on a Bamboo A1 Mini printer, unibody is really a large hexagon comprising 13 parts:

	1 Internal Hub
	6 Mid-ring
	6 outer ring

## Current design: bolted flanges

`unibody/design_flanged_deck.py` generates the deck (run with
`~/.pyenv/shims/python3`; needs trimesh + manifold3d). It writes
`deck_hub.stl`, `deck_mid.stl` and `deck_outer.stl`, already in print
orientation, plus `deck_assembly.stl` for a sanity look.

	deck_hub    x1   167.5 x 167.5 x 25 mm
	deck_mid    x6    93.6 x  93.6 x 25 mm
	deck_outer  x6   171.7 x 171.7 x 25 mm

Every seam has a 20 mm flange hanging under the 5 mm plate. Mating flanges
bolt back to back, so the seams form a rib waffle (6 spokes, 2 hex rings)
under a flat top. The outer perimeter has no flange, to clear the coxa.
The mid/outer seam sits at 104 mm to clear the coxa-cap nuts.

Hardware: 66 x M3x16 bolts, nyloc nuts, 2 washers each.

Print with the plate face down and the flanges up, no supports. Use a brim
on the hub and outer pieces. Use plenty of walls in the flanges: the flange
root is loaded across the layers when the deck sags.

## Superseded: glued deck

`unibody/glued/` holds the first version (`generate_parts.py` and its
STLs): the same 13 pieces edge-glued with braces underneath. It failed at
the glue joints, since a 5 mm butt joint gives only 5 mm of glue width
right where the leg moment is highest. Kept for reference only.

## What bolts to the deck

a.) Battery
b.) BEC converter (Qty 2) -- two different outputs, NOT a split load:
       - 5 V   -> Raspberry Pi
       - 7.4 V -> servo bus, via the URTs
    So the servos are fed by ONE converter. Its current rating is the
    ceiling for everything the legs do, and the Pi's draw does not come
    out of that budget. See doc/power.md (LOCOPOWER / LOGIPOWER).
c.) Raspberry Pi
d.) URT (Qty 3)
e.) Coxa servo w. cap (Qty 6)
