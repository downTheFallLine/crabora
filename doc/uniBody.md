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

## Lid

`unibody/design_lid.py` generates a faceted shell over the deck (same
command). Wall 48 mm, flush with the deck edge in the 5 mm band outside
the coxa caps, then a 10.6° slope up to a flat top at 60 mm. Every panel
is an open honeycomb (1.2 mm skin, 1.6 mm wall, 1.8 mm ribs, about 75%
open), trimmed to run right up to the seams, with a small-hex
speaker/mic grille in the middle of the hub. 13 pieces, seams in the same
places as the deck's, bolted through lips inside (72 x M3x8 + nyloc).
About 240 g of PLA if solid.

It holds on with magnets: a printed puck (`lid_puck.stl`) bolts through
the `LID_PUCK` hole near one corner of each deck outer piece (M3x12) with
an 8x3 mm magnet on top, and seats in a socket in the lid with its own
magnet. On an already-printed deck, drill that hole, 3.4 mm, on each
of the six edges: 10.8 mm in from the deck edge and 29 mm from the
corner you reach going clockwise (seen from above) from that edge's leg.
Put all six deck magnets in with the same face up.

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
