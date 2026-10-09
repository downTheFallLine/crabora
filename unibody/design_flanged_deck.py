"""
CRABORA flanged body deck, v1 -- BOLTED, not glued
==================================================

Replaces the edge-glued hub + mid_ring + outer_ring deck (generate_parts.py),
which failed at the glue joints. Same idea -- a flat-top hex deck printed in
13 pieces (1 hub, 6 mid, 6 outer) -- but every seam now has a FLANGE: a wall
hanging down from the underside of each piece along that edge. Mating
flanges sit back to back and are bolted through with M3 bolts + nuts:

        piece A          seam          piece B
    ════════════════════╗ │ ╔═════════════════════   <- 5 mm deck plate
                      ◢ ║ │ ║ ◣                      <- 45° fillet under plate
                        ║ │ ║
            nut ▐▌══════╬═│═╬══════▐▌ bolt head      <- M3 x 16 through both
                        ║ │ ║
                        ╚═╧═╝                        <- 20 mm flange

Each seam becomes a 25 mm deep, 10 mm thick rib, so the joints are now the
stiffest part of the deck instead of the weakest. The radial seams line up
into 6 continuous spokes and the circumferential seams into 2 hex rings --
a waffle frame under a flat top.

The top surface stays flat (flanges hang DOWN) so electronics can go
anywhere on top. The outer perimeter has NO flange, so nothing hangs down
where the coxa pan swings under the deck edge.

Seam placement vs. the coxa cap
-------------------------------
The coxa cap's 4 M3 bolts come down through the outer piece and take nuts
on the underside, the inner pair at radius ~121 mm. With the original
mid/outer seam at A1 = 115 mm the flange + fillet would occupy exactly
where those nuts go, so the seam moves inward to MID_OUTER_APOTHEM = 104
(fillet ends at 115, washer edge at ~117.7). Leg hole positions are
unchanged -- they're tied to the deck's outer edge, not the seam.

Print orientation
-----------------
Each STL is written already in print orientation: deck plate face down on
the bed, flanges pointing up (vertical walls, no supports), and rotated in
XY to the angle that best fits the A1 mini's 180 x 180 mm bed. The bolt
holes through the flanges are horizontal 3.4 mm holes -- small enough to
bridge cleanly.

Hardware: M3 x 16 bolts + M3 nyloc nuts (+ washers) through the flanges;
the count is printed at the end.

Needs numpy + trimesh with the manifold3d boolean engine:
    ~/.pyenv/shims/python3 design_flanged_deck.py
"""

import math
import os

import numpy as np
import trimesh

# =============================================================
# PARAMETERS (mm)
# =============================================================
PLATE_THK = 5.0              # deck plate (coxa cap bolt lengths assume 5)

HUB_APOTHEM = 75.075         # hub flat-to-flat / 2 (max ~77 to fit the bed)
MID_OUTER_APOTHEM = 104.0    # mid/outer seam -- see "Seam placement" above
DECK_APOTHEM = 181.825       # outer edge; same as the printed deck's A2

FLANGE_DEPTH = 20.0          # hangs this far below the plate underside
FLANGE_THK = 5.0
FILLET = 6.0                 # 45° fillet where flange meets plate underside

SEAM_BOLT_DIA = 3.4          # M3 clearance
SEAM_BOLT_Z = -13.0          # bolt axis height (plate underside is z = 0)
SEAM_BOLT_PITCH = 35.0       # max spacing between bolts along a seam
SEAM_BOLT_END_MARGIN = 16.0  # clear of the corner where two flanges meet

# Leg (coxa cap) holes -- copied from generate_parts.py, which copies them
# from ~/crabora/leg/coxa/design_coxa_body_cap.py. Keep in sync.
CAP_LENGTH = 52.2
CAP_HORN_OFFSET = 12.5
CAP_BOLT_RADIAL_OFFSET = 15.0
CAP_BOLT_TANGENT_OFFSET = 15.35
CAP_EDGE_SETBACK = 5.0
HORN_HOLE_DIAMETER = 22.0
MOUNT_HOLE_DIAMETER = 3.4

BED_SIZE = 178.0             # usable A1 mini plate (see memory: keep < ~178)
CIRCLE_SECTIONS = 48

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# =============================================================
# 2D HELPERS (all deck pieces are convex polygons, CCW)
# =============================================================
TAN30 = math.tan(math.radians(30))


def rot2(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def hexagon(apothem):
    """Flat-top hex: corners at 0, 60, ... deg; edge k centered on 30 + 60k."""
    R = apothem / math.cos(math.radians(30))
    return np.array([[R * math.cos(math.radians(60 * k)),
                      R * math.sin(math.radians(60 * k))] for k in range(6)])


def ring_segment(inner_a, outer_a, deg):
    """Trapezoid between two apothems, centered on +Y then rotated by deg.
    Edge order (CCW): inner, +side, outer, -side."""
    wi, wo = inner_a * TAN30, outer_a * TAN30
    pts = np.array([[-wi, inner_a], [wi, inner_a], [wo, outer_a], [-wo, outer_a]])
    return pts @ rot2(deg).T


def inset(poly, dists):
    """Offset each edge i (poly[i] -> poly[i+1]) of a convex CCW polygon
    inward by dists[i] (negative = outward); return the new polygon."""
    n = len(poly)
    lines = []
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        d = (q - p) / np.linalg.norm(q - p)
        nrm = np.array([-d[1], d[0]])               # inward for CCW
        lines.append((nrm, nrm @ p + dists[i]))
    out = []
    for i in range(n):
        (n1, c1), (n2, c2) = lines[i - 1], lines[i]
        out.append(np.linalg.solve(np.array([n1, n2]), np.array([c1, c2])))
    return np.array(out)


# =============================================================
# 3D HELPERS
# =============================================================
def prism(poly, z0, z1):
    pts = [[x, y, z] for x, y in poly for z in (z0, z1)]
    return trimesh.convex.convex_hull(np.array(pts))


def loft(poly_a, z_a, poly_b, z_b):
    pts = [[x, y, z_a] for x, y in poly_a] + [[x, y, z_b] for x, y in poly_b]
    return trimesh.convex.convex_hull(np.array(pts))


def vertical_hole(x, y, dia):
    return trimesh.creation.cylinder(
        radius=dia / 2, height=PLATE_THK + 2, sections=CIRCLE_SECTIONS,
        transform=trimesh.transformations.translation_matrix([x, y, PLATE_THK / 2]))


def seam_hole(point, normal2d):
    """Horizontal bolt hole through a seam, axis along the in-plane normal."""
    axis = np.array([normal2d[0], normal2d[1], 0.0])
    T = trimesh.geometry.align_vectors([0, 0, 1], axis)
    T[:3, 3] = [point[0], point[1], SEAM_BOLT_Z]
    return trimesh.creation.cylinder(
        radius=SEAM_BOLT_DIA / 2, height=2 * FLANGE_THK + 2 * FILLET,
        sections=CIRCLE_SECTIONS, transform=T)


def seam_bolt_points(p, q):
    """Evenly spaced, symmetric along the edge -- so both pieces sharing a
    seam compute identical points whichever direction they walk it."""
    L = np.linalg.norm(q - p)
    usable = L - 2 * SEAM_BOLT_END_MARGIN
    # A short seam with room for only bolts crammed together gets one, centered.
    n = 1 if usable < SEAM_BOLT_PITCH / 2 else math.ceil(usable / SEAM_BOLT_PITCH) + 1
    ts = [0.5] if n == 1 else np.linspace(SEAM_BOLT_END_MARGIN, L - SEAM_BOLT_END_MARGIN, n) / L
    return [p + t * (q - p) for t in ts]


# =============================================================
# PIECE BUILDER
# =============================================================
def build_piece(poly, flanged, extra_holes=()):
    """poly: convex CCW outline. flanged[i]: edge i gets a flange.
    extra_holes: (x, y, dia) vertical holes through the plate.
    Returns (mesh in deck frame, number of seam bolt holes)."""
    plate = prism(poly, 0.0, PLATE_THK)

    # Flanges + fillets = slab under the whole outline, minus the empty
    # interior. The interior is a prism down from the flange inset, plus a
    # frustum from that inset (at z = -FILLET) up to a wider inset at z = 0
    # that carves the 45° fillets. Unflanged edges push the insets 1 mm
    # OUTWARD so no wall is left along them.
    d_wall = [FLANGE_THK if f else -1.0 for f in flanged]
    d_fillet = [FLANGE_THK + FILLET if f else -1.0 for f in flanged]
    inner_wall = inset(poly, d_wall)
    inner_fillet = inset(poly, d_fillet)
    slab = prism(poly, -FLANGE_DEPTH, 0.0)
    void = trimesh.boolean.union([
        prism(inner_wall, -FLANGE_DEPTH - 1, -FILLET),
        loft(inner_wall, -FILLET, inner_fillet, 0.0),
    ], engine='manifold')
    flanges = trimesh.boolean.difference([slab, void], engine='manifold')

    solid = trimesh.boolean.union([plate, flanges], engine='manifold')

    cutters = [vertical_hole(x, y, d) for x, y, d in extra_holes]
    n_bolts = 0
    for i, f in enumerate(flanged):
        if not f:
            continue
        p, q = poly[i], poly[(i + 1) % len(poly)]
        d = (q - p) / np.linalg.norm(q - p)
        for pt in seam_bolt_points(p, q):
            cutters.append(seam_hole(pt, np.array([-d[1], d[0]])))
            n_bolts += 1
    if cutters:
        solid = trimesh.boolean.difference([solid] + cutters, engine='manifold')
    return solid, n_bolts


def leg_holes(deg):
    """Coxa cap horn + 4 bolt holes for the leg centered on +Y, rotated by deg."""
    cap_c = (DECK_APOTHEM - CAP_EDGE_SETBACK) - CAP_LENGTH / 2.0
    holes = [(0.0, cap_c + CAP_HORN_OFFSET, HORN_HOLE_DIAMETER)]
    holes += [(sx * CAP_BOLT_TANGENT_OFFSET, cap_c + sy * CAP_BOLT_RADIAL_OFFSET,
               MOUNT_HOLE_DIAMETER) for sx in (-1, 1) for sy in (-1, 1)]
    R = rot2(deg)
    return [(*(R @ np.array([x, y])), d) for x, y, d in holes]


# =============================================================
# PRINT ORIENTATION
# =============================================================
def to_print_pose(mesh):
    """Flip so the plate's top face is on the bed and flanges point up, then
    rotate in XY to the smallest square footprint and center on the bed."""
    m = mesh.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]))
    xy = m.convex_hull.vertices[:, :2]
    best = None
    for deg in np.arange(0, 90, 0.5):
        p = xy @ rot2(deg).T
        side = (p.max(0) - p.min(0)).max()
        if best is None or side < best[0]:
            best = (side, deg)
    m.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(best[1]), [0, 0, 1]))
    lo, hi = m.bounds
    m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    return m


# =============================================================
# MAIN
# =============================================================
def build_all():
    pieces = {}
    # Hub: flange on all 6 edges.
    pieces['deck_hub'] = build_piece(hexagon(HUB_APOTHEM), [True] * 6)
    # Mid (leg 0, centered on +Y): flanged on every edge.
    pieces['deck_mid'] = build_piece(
        ring_segment(HUB_APOTHEM, MID_OUTER_APOTHEM, 0.0), [True] * 4)
    # Outer (leg 0): inner + both sides flanged, perimeter edge (index 2) not.
    pieces['deck_outer'] = build_piece(
        ring_segment(MID_OUTER_APOTHEM, DECK_APOTHEM, 0.0),
        [True, True, False, True], extra_holes=leg_holes(0.0))
    return pieces


def assembly(pieces):
    """All 13 pieces in deck frame, for a sanity look."""
    parts = [pieces['deck_hub'][0]]
    for k in range(6):
        R = trimesh.transformations.rotation_matrix(math.radians(60 * k), [0, 0, 1])
        for name in ('deck_mid', 'deck_outer'):
            parts.append(pieces[name][0].copy().apply_transform(R))
    return trimesh.util.concatenate(parts)


if __name__ == '__main__':
    pieces = build_all()
    total_bolt_holes = 0
    for name, (mesh, n_bolts) in pieces.items():
        qty = 1 if name == 'deck_hub' else 6
        total_bolt_holes += qty * n_bolts
        printed = to_print_pose(mesh)
        printed.export(os.path.join(OUT_DIR, f'{name}.stl'))
        ext = printed.extents
        fits = 'fits' if max(ext[:2]) <= BED_SIZE else 'DOES NOT FIT'
        print(f'{name:11s} x{qty}  {ext[0]:6.1f} x {ext[1]:6.1f} x {ext[2]:4.1f} mm  '
              f'({fits})  watertight={mesh.is_watertight}  '
              f'{mesh.volume / 1000:5.1f} cm3  seam holes={n_bolts}')
    assembly(pieces).export(os.path.join(OUT_DIR, 'deck_assembly.stl'))
    # Every seam bolt passes through two flanges, so bolts = holes / 2.
    print(f'seam bolts: {total_bolt_holes // 2} x M3x16 + nyloc nut + 2 washers')
