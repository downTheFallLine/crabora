"""
CRABORA lid, v1 -- faceted shell over the deck, held on by magnets
===================================================================

A hex shell that sits on the deck top and covers the electronics and the
six coxa caps. The coxa caps are the tallest things on the deck (41 mm)
and stop 5 mm short of the deck edge, so the lid's wall stands in that
5 mm band, flush with the deck edge:

                         flat top (hub + mid ring)
          ┌───────────────────────────────────┐ TOP_H
         ╱ slope facet                         ╲
        ╱  (hex vents)                          ╲
       │ WALL_H                                  │
       │ wall          ┌─────┐   coxa cap        │
       │ (hex vents)   │     │   (41 mm)         │
  ═════╧═══════════════╧═════╧═══════════════════╧═════  deck top, z = 0

Like the deck it is too big for the A1 mini, so it prints in 13 pieces
(1 hub, 6 mid, 6 outer) with seams in the same places as the deck's: a
hub, a ring at TOP_APOTHEM where the slope meets the flat top, and radial
seams at the hex corners -- clear of the caps, which sit mid-edge. No
glue: every seam has a lip hanging down inside each piece, and mating
lips bolt back to back with M3 bolts, the deck's flange idea at a lighter
scale (the lid carries no load).

Magnets
-------
Each outer piece has a boss inside the wall near its +X corner with a
socket in its underside and a magnet in the socket's ceiling. On the deck
a printed puck (lid_puck.stl) is bolted through the deck's LID_PUCK hole
with a matching magnet on top. The puck sits in the socket, so it also
locates the lid; the socket is 0.5 mm deeper than the puck, so the wall
lands on the deck before the magnets touch. Get the polarity right: put
all six deck magnets in with the same face up, then let each lid magnet
pick its side by sticking it to a deck magnet before pressing it in.

Print orientation
-----------------
Every STL is written in print orientation, outside face on the bed: the
hub and mid pieces top-face down, the outer pieces lying on the slope
facet with the wall leaning ~10° outward. The hex vents are then vertical
through-holes, the lips and bosses grow upward, and the seam bolt holes
are horizontal 3.4 mm holes that bridge. No supports.

Hardware (counts printed at the end): M3 x 8 + nyloc nut through the
lips, M3 x 12 + nut for each puck, 8 x 3 mm disc magnets.

Needs numpy + trimesh with the manifold3d boolean engine, and matplotlib
for the preview:
    ~/.pyenv/shims/python3 design_lid.py
"""

import math
import os

import numpy as np
import trimesh
from matplotlib import pyplot as plt
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from design_flanged_deck import (
    BED_SIZE, CAP_EDGE_SETBACK, CAP_LENGTH, CIRCLE_SECTIONS, DECK_APOTHEM,
    LID_PUCK_X, LID_PUCK_Y,
    hexagon, inset, prism, ring_segment, rot2, to_print_pose,
)

# =============================================================
# PARAMETERS (mm)
# =============================================================
# The coxa cap, from ~/crabora/leg/coxa/design_coxa_body_cap.py (CAP_Z,
# CAP_Y). Only used to check clearance.
CAP_HEIGHT = 41.0
CAP_WIDTH = 39.7

WALL_THK = 1.6               # 4 lines at 0.4; outer face flush with the deck edge
WALL_H = 48.0                # wall height at the outer face; slope starts here
TOP_H = 60.0                 # flat top, outer surface
TOP_APOTHEM = 118.0          # slope meets flat top; also the mid/outer seam
HUB_APOTHEM = 78.0           # hub/mid seam (hub must fit the bed: < ~79)
SKIN = 1.2                   # slope and top thickness: 6 layers at 0.2

LIP_DEPTH = 7.0              # seam lips hang this far below the inside
LIP_THK = 2.0
CORNER_LIP_W = 10.0          # lips run down the wall this far from a corner
SEAM_BOLT_DIA = 3.4          # M3 clearance
SEAM_BOLT_PITCH = 50.0
SEAM_BOLT_END_MARGIN = 9.0   # along the seam, from where two lips meet
CORNER_BOLT_ZS = (10.0,)       # bolts down the wall at each corner seam

MAGNET_DIA = 8.0             # 8 x 3 mm disc magnets
MAGNET_THK = 3.0
MAGNET_POCKET_DIA = MAGNET_DIA + 0.2     # press fit
MAGNET_POCKET_DEPTH = MAGNET_THK + 0.2

PUCK_DIA = 14.0
PUCK_FLOOR = 3.0
PUCK_HEAD_DIA = 6.0          # M3 socket head is 5.5; smaller than the magnet
PUCK_HEAD_DEPTH = 3.0        # so the magnet rests on the ledge above it
PUCK_H = PUCK_FLOOR + PUCK_HEAD_DEPTH + MAGNET_POCKET_DEPTH

SOCKET_DIA = PUCK_DIA + 0.6
SOCKET_DEPTH = PUCK_H + 0.5  # wall lands on the deck before magnets touch
BOSS_WALL = 1.6
BOSS_WEB = 0.5                # web width to the wall, as a fraction of boss radius
BOSS_CONE_DEG = 60.0
BOSS_TOP = SOCKET_DEPTH + MAGNET_POCKET_DEPTH + 2.0

VENT_R = 7.0                 # hex vent circumradius (walls, slopes, mid, hub)
VENT_RIB = 1.8                # ~75% open where the vents are
VENT_EDGE_MARGIN = 2.0       # vent edge to a lip, fold or corner
GRILLE_R = 2.2               # small hexes in the hub: speaker / mic grille
GRILLE_RIB = 1.6
GRILLE_RADIUS = 36.0         # the hub's big vents start outside it
GRILLE_RING = 3.0            # solid ring between grille and vents

PLA_DENSITY = 1.24           # g/cm3, for the weight estimate

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# =============================================================
# PROFILE
# =============================================================
SLOPE_TAN = (TOP_H - WALL_H) / (DECK_APOTHEM - TOP_APOTHEM)
COS_A = 1.0 / math.hypot(1.0, SLOPE_TAN)
WALL_IN = DECK_APOTHEM - WALL_THK
SIN30, COS30 = 0.5, math.sqrt(3) / 2


def z_outer(h):
    """Outer surface height at hex radius h."""
    return TOP_H if h <= TOP_APOTHEM else TOP_H - (h - TOP_APOTHEM) * SLOPE_TAN


def ceiling(h):
    """Inside surface height at hex radius h (h <= WALL_IN)."""
    return min(TOP_H - SKIN, z_outer(h) - SKIN / COS_A)


# =============================================================
# 2D / 3D HELPERS
# =============================================================
def hull(layers):
    """Convex hull of [(polygon, z), ...]."""
    return trimesh.convex.convex_hull(np.array(
        [[x, y, z] for poly, z in layers for x, y in poly]))


def box(lo, hi):
    return trimesh.creation.box(bounds=[lo, hi])


def union(ms):
    return trimesh.boolean.union(ms, engine='manifold')


def intersect(a, b):
    return trimesh.boolean.intersection([a, b], engine='manifold')


def subtract(a, cutters):
    return trimesh.boolean.difference([a] + list(cutters), engine='manifold') if cutters else a


def clip(poly, n, c):
    """The part of convex polygon `poly` where n . p <= c."""
    out = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        dp, dq = n @ p - c, n @ q - c
        if dp <= 0:
            out.append(p)
        if dp * dq < 0:
            out.append(p + (q - p) * dp / (dp - dq))
    return np.array(out)


def edge(poly, i):
    """Endpoints and inward normal of edge i of a CCW polygon."""
    p, q = poly[i], poly[(i + 1) % len(poly)]
    d = (q - p) / np.linalg.norm(q - p)
    return p, q, np.array([-d[1], d[0]])


def strip(poly, i, width):
    """The band of `poly` within `width` of its edge i."""
    p, _, nrm = edge(poly, i)
    return clip(poly, nrm, nrm @ p + width)


def inside(poly, pt, margin=0.0):
    """pt is inside convex CCW poly by at least margin."""
    return all(nrm @ (pt - p) >= margin
               for p, _, nrm in (edge(poly, i) for i in range(len(poly))))


def hex2d(R):
    """Hexagon with a vertex along +v, as (u, v) offsets."""
    return [(R * math.cos(math.radians(90 + 60 * k)),
             R * math.sin(math.radians(90 + 60 * k))) for k in range(6)]


def honeycomb(R, rib, keep, span=400):
    """Centres of a hex vent lattice, filtered by keep(u, v)."""
    d = math.sqrt(3) * R + rib
    row = d * math.sqrt(3) / 2
    pts = []
    for k in range(-int(span / row), int(span / row) + 1):
        for j in range(-int(span / d), int(span / d) + 1):
            u, v = j * d + (d / 2 if k % 2 else 0.0), k * row
            if keep(u, v):
                pts.append((u, v))
    return pts


def hex_cutter(center, eu, ev, en, R, n0, n1):
    """Hex prism: hex in the (eu, ev) plane at `center`, from n0 to n1 along en."""
    c, eu, ev, en = (np.asarray(a, float) for a in (center, eu, ev, en))
    return trimesh.convex.convex_hull(np.array(
        [c + u * eu + v * ev + n * en for u, v in hex2d(R) for n in (n0, n1)]))


def bolt_hole(point, normal2d, z):
    axis = np.array([normal2d[0], normal2d[1], 0.0])
    T = trimesh.geometry.align_vectors([0, 0, 1], axis)
    T[:3, 3] = [point[0], point[1], z]
    return trimesh.creation.cylinder(
        radius=SEAM_BOLT_DIA / 2, height=2 * LIP_THK + 2,
        sections=CIRCLE_SECTIONS, transform=T)


def spaced(a, b):
    """Evenly spaced stations between a and b, at most SEAM_BOLT_PITCH apart."""
    n = max(2, math.ceil((b - a) / SEAM_BOLT_PITCH) + 1)
    return np.linspace(a, b, n)


def vent_field(R, rib, keep_mesh, near, cutter, span):
    """Honeycomb vents trimmed to keep_mesh. Cells whose centre is
    near(u, v) are cut; the trim turns the edge cells into partial ones,
    so the lattice runs right up to the margins instead of stopping a
    whole cell short of them."""
    cells = [cutter(u, v) for u, v in honeycomb(R, rib, near, span=span)]
    return [intersect(union(cells), keep_mesh)] if cells else []


# =============================================================
# THE SHELL
# =============================================================
OUTER = hull([(hexagon(DECK_APOTHEM), 0.0), (hexagon(DECK_APOTHEM), WALL_H),
              (hexagon(TOP_APOTHEM), TOP_H)])
# The inside: wall face straight up, then the slope offset by SKIN, then
# the underside of the top. H_KNEE is where the inner slope meets it.
H_KNEE = TOP_APOTHEM - (SKIN / COS_A - SKIN) / SLOPE_TAN
INNER = hull([(hexagon(WALL_IN), -1.0), (hexagon(WALL_IN), ceiling(WALL_IN)),
              (hexagon(H_KNEE), TOP_H - SKIN)])
SHELL = subtract(OUTER, [INNER])
# Everything inside within LIP_DEPTH of the ceiling: lips are this band
# cut to a strip along each seam.
BAND = intersect(subtract(INNER, [INNER.copy().apply_translation([0, 0, -LIP_DEPTH])]),
                 box([-300, -300, 0.0], [300, 300, TOP_H]))


# =============================================================
# PIECES (leg 0 frame: piece centred on +Y, like the deck's)
# =============================================================
def seam_lips(region, seams):
    return [intersect(BAND, prism(strip(region, i, LIP_THK), -1.0, TOP_H + 1))
            for i in seams]


def ring_seam_holes(region, i, apothem):
    """Bolts along a circumferential seam: symmetric along the edge, so
    the pieces on both sides drill the same points."""
    p, q, nrm = edge(region, i)
    L = np.linalg.norm(q - p)
    z = ceiling(apothem + LIP_THK) - LIP_DEPTH / 2
    return [bolt_hole(p + t * (q - p), nrm, z)
            for t in spaced(SEAM_BOLT_END_MARGIN, L - SEAM_BOLT_END_MARGIN) / L]


def radial_seam_holes(region, i, stations):
    """Bolts along a corner seam at the given (hex radius, z) stations.
    Both neighbours use the same stations, so the holes line up."""
    p, q, nrm = edge(region, i)
    holes = []
    for y, z in stations:
        pt = p + (q - p) * (y - p[1]) / (q[1] - p[1])
        holes.append(bolt_hole(pt, nrm, z))
    return holes


def slope_frame():
    """Unit vectors on the leg-0 slope facet: along X, down the slope
    (outward), and the outward normal."""
    t = np.array([0.0, 1.0, -SLOPE_TAN]) * COS_A
    n = np.array([0.0, SLOPE_TAN, 1.0]) * COS_A
    return np.array([1.0, 0.0, 0.0]), t, n


def corner_margin(x, y):
    """Distance from (x, y) inside the leg-0 wedge to the nearer corner seam."""
    return y * SIN30 - abs(x) * COS30


def flat_vent(u, v):
    return hex_cutter([u, v, TOP_H - SKIN / 2], [1, 0, 0], [0, 1, 0], [0, 0, 1],
                      VENT_R, -SKIN, SKIN)


def flat_keep(region, seams):
    """Where a flat piece may be vented: clear of its lips by the margin."""
    d = [LIP_THK + VENT_EDGE_MARGIN if i in seams else VENT_EDGE_MARGIN
         for i in range(len(region))]
    return prism(inset(region, d), -1.0, TOP_H + 2)


def build_hub():
    region = hexagon(HUB_APOTHEM)
    solid = union([intersect(SHELL, prism(region, -2.0, TOP_H + 2))]
                  + seam_lips(region, range(6)))
    holes = [h for i in range(6) for h in ring_seam_holes(region, i, HUB_APOTHEM)]
    cutters = list(holes)
    # Speaker / mic grille.
    for u, v in honeycomb(GRILLE_R, GRILLE_RIB,
                          lambda u, v: math.hypot(u, v) + GRILLE_R <= GRILLE_RADIUS):
        cutters.append(hex_cutter([u, v, TOP_H - SKIN / 2], [1, 0, 0], [0, 1, 0],
                                  [0, 0, 1], GRILLE_R, -SKIN, SKIN))
    # Big vents around it, clear of the grille ring and the lips.
    keep = subtract(flat_keep(region, range(6)), [trimesh.creation.cylinder(
        radius=GRILLE_RADIUS + GRILLE_RING, height=TOP_H * 3, sections=CIRCLE_SECTIONS)])
    cutters += vent_field(
        VENT_R, VENT_RIB, keep,
        lambda u, v: (math.hypot(u, v) + VENT_R / 2 > GRILLE_RADIUS + GRILLE_RING
                      and inside(region, np.array([u, v]), LIP_THK)),
        flat_vent, HUB_APOTHEM)
    return subtract(solid, cutters), len(holes)


def build_mid():
    region = ring_segment(HUB_APOTHEM, TOP_APOTHEM, 0.0)
    solid = union([intersect(SHELL, prism(region, -2.0, TOP_H + 2))]
                  + seam_lips(region, range(4)))
    m = SEAM_BOLT_END_MARGIN * COS30
    stations = [(y, TOP_H - SKIN - LIP_DEPTH / 2)
                for y in spaced(HUB_APOTHEM + m, TOP_APOTHEM - m)]
    holes = (ring_seam_holes(region, 0, HUB_APOTHEM)
             + ring_seam_holes(region, 2, TOP_APOTHEM)
             + radial_seam_holes(region, 1, stations)
             + radial_seam_holes(region, 3, stations))
    vents = vent_field(VENT_R, VENT_RIB, flat_keep(region, range(4)),
                       lambda u, v: inside(region, np.array([u, v]), LIP_THK),
                       flat_vent, TOP_APOTHEM)
    return subtract(solid, holes + vents), len(holes)


def boss():
    """Magnet boss inside the wall: a column joined to the wall by a web,
    capped with a 60° cone so it prints without support (lying on the
    slope it tilts ~10°, still steeper than 45°)."""
    r = SOCKET_DIA / 2 + BOSS_WALL
    base = [LID_PUCK_X, LID_PUCK_Y]
    col = trimesh.creation.cylinder(radius=r, sections=CIRCLE_SECTIONS,
                                    segment=[base + [0.0], base + [BOSS_TOP]])
    web = box([LID_PUCK_X - r * BOSS_WEB, LID_PUCK_Y, 0.0],
              [LID_PUCK_X + r * BOSS_WEB, WALL_IN + 0.5, BOSS_TOP])
    cone_h = r * math.tan(math.radians(BOSS_CONE_DEG))
    cone = trimesh.creation.cone(
        radius=r, height=cone_h, sections=CIRCLE_SECTIONS,
        transform=trimesh.transformations.translation_matrix(base + [BOSS_TOP]))
    # The web's top ramps into the wall at the cone's angle too.
    ramp = box([LID_PUCK_X - r * BOSS_WEB, WALL_IN - 0.5, 0.0],
               [LID_PUCK_X + r * BOSS_WEB, WALL_IN + 0.5,
                BOSS_TOP + (WALL_IN - LID_PUCK_Y) * math.tan(math.radians(BOSS_CONE_DEG))])
    web = trimesh.convex.convex_hull(np.vstack([web.vertices, ramp.vertices]))
    return intersect(union([col, web, cone]), OUTER), BOSS_TOP + cone_h


def build_outer():
    region = ring_segment(TOP_APOTHEM, DECK_APOTHEM + 5, 0.0)
    parts = [intersect(SHELL, prism(region, -2.0, TOP_H + 2))]
    parts += seam_lips(region, (0, 1, 3))
    for i in (1, 3):
        corner = clip(strip(region, i, LIP_THK), np.array([0.0, -1.0]),
                      -(WALL_IN - CORNER_LIP_W))
        parts.append(intersect(INNER, prism(corner, 0.0, TOP_H)))
    b, boss_top = boss()
    parts.append(b)
    solid = union(parts)

    m = SEAM_BOLT_END_MARGIN * COS30
    stations = [(y, ceiling(y) - LIP_DEPTH / 2)
                for y in spaced(TOP_APOTHEM + m, WALL_IN - CORNER_LIP_W)]
    stations += [(WALL_IN - CORNER_LIP_W / 2, z) for z in CORNER_BOLT_ZS]
    holes = (ring_seam_holes(region, 0, TOP_APOTHEM)
             + radial_seam_holes(region, 1, stations)
             + radial_seam_holes(region, 3, stations))

    cutters = list(holes)
    # Socket for the puck, and the magnet pocket in its ceiling.
    cutters.append(trimesh.creation.cylinder(
        radius=SOCKET_DIA / 2, sections=CIRCLE_SECTIONS,
        segment=[[LID_PUCK_X, LID_PUCK_Y, -1.0], [LID_PUCK_X, LID_PUCK_Y, SOCKET_DEPTH]]))
    cutters.append(trimesh.creation.cylinder(
        radius=MAGNET_POCKET_DIA / 2, sections=CIRCLE_SECTIONS,
        segment=[[LID_PUCK_X, LID_PUCK_Y, SOCKET_DEPTH - 0.01],
                 [LID_PUCK_X, LID_PUCK_Y, SOCKET_DEPTH + MAGNET_POCKET_DEPTH]]))

    # Wall vents, clear of the corner lips and the boss.
    r_boss = SOCKET_DIA / 2 + BOSS_WALL
    x_lim = WALL_IN * SIN30 / COS30 - LIP_THK / COS30 - VENT_EDGE_MARGIN
    z_lo, z_hi = 4.0, ceiling(WALL_IN) - VENT_EDGE_MARGIN
    z_mid = (z_lo + z_hi) / 2
    keep = subtract(box([-x_lim, WALL_IN - 2, z_lo], [x_lim, DECK_APOTHEM + 2, z_hi]),
                    [box([LID_PUCK_X - r_boss - VENT_EDGE_MARGIN, WALL_IN - 3, -1],
                         [LID_PUCK_X + r_boss + VENT_EDGE_MARGIN, DECK_APOTHEM + 3,
                          boss_top + VENT_EDGE_MARGIN])])
    cutters += vent_field(
        VENT_R, VENT_RIB, keep,
        lambda u, v: abs(u) < x_lim + VENT_R / 2 and abs(v) < (z_hi - z_lo) / 2 + VENT_R / 2,
        lambda u, v: hex_cutter([u, (WALL_IN + DECK_APOTHEM) / 2, z_mid + v],
                                [1, 0, 0], [0, 0, 1], [0, 1, 0], VENT_R,
                                -WALL_THK, WALL_THK),
        DECK_APOTHEM)

    # Slope vents, laid out in the facet's own plane.
    ex, et, en = slope_frame()
    fold = np.array([0.0, TOP_APOTHEM, TOP_H])
    s_len = (DECK_APOTHEM - TOP_APOTHEM) / COS_A
    e = VENT_EDGE_MARGIN
    slope_area = inset(ring_segment(TOP_APOTHEM, WALL_IN, 0.0),
                       [LIP_THK + e, LIP_THK + e, e, LIP_THK + e])
    cutters += vent_field(
        VENT_R, VENT_RIB, prism(slope_area, -1.0, TOP_H + 2),
        lambda u, v: inside(slope_area, np.array([u, TOP_APOTHEM + (s_len / 2 + v) * COS_A]),
                            -VENT_R / 2),
        lambda u, v: hex_cutter(fold + u * ex + (s_len / 2 + v) * et - en * SKIN / 2,
                                ex, et, en, VENT_R, -SKIN, SKIN),
        DECK_APOTHEM)
    return subtract(solid, cutters), len(holes)


def build_puck():
    """Bolts to the deck: M3 up through the deck and the puck floor, its
    head in the counterbore, the magnet on top of it."""
    puck = trimesh.creation.cylinder(radius=PUCK_DIA / 2, sections=CIRCLE_SECTIONS,
                                     segment=[[0, 0, 0], [0, 0, PUCK_H]])
    cuts = [
        trimesh.creation.cylinder(radius=SEAM_BOLT_DIA / 2, sections=CIRCLE_SECTIONS,
                                  segment=[[0, 0, -1], [0, 0, PUCK_H + 1]]),
        trimesh.creation.cylinder(radius=PUCK_HEAD_DIA / 2, sections=CIRCLE_SECTIONS,
                                  segment=[[0, 0, PUCK_FLOOR], [0, 0, PUCK_H + 1]]),
        trimesh.creation.cylinder(radius=MAGNET_POCKET_DIA / 2, sections=CIRCLE_SECTIONS,
                                  segment=[[0, 0, PUCK_H - MAGNET_POCKET_DEPTH],
                                           [0, 0, PUCK_H + 1]]),
    ]
    return subtract(puck, cuts)


# =============================================================
# PRINT ORIENTATION
# =============================================================
def outer_print_pose(mesh):
    """Tilt the slope facet level, then the deck's flip-and-fit puts it
    face down on the bed."""
    m = mesh.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(
        math.atan(SLOPE_TAN), [1, 0, 0]))
    return to_print_pose(m)


# =============================================================
# ASSEMBLY, CHECKS, PREVIEW
# =============================================================
def ring_of(mesh):
    return [mesh.copy().apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(60 * k), [0, 0, 1]))
        for k in range(6)]


def cap_boxes():
    c = DECK_APOTHEM - CAP_EDGE_SETBACK - CAP_LENGTH / 2
    cap = box([-CAP_WIDTH / 2, c - CAP_LENGTH / 2, 0.0],
              [CAP_WIDTH / 2, c + CAP_LENGTH / 2, CAP_HEIGHT])
    return ring_of(cap)


def render(parts, path):
    bg, fg = "#0c0d11", "#9fb3c8"
    key = np.array([0.35, 0.45, 0.82]); key /= np.linalg.norm(key)
    fill = np.array([-0.55, -0.25, 0.30]); fill /= np.linalg.norm(fill)

    def shaded(mesh, base):
        n = mesh.face_normals
        lit = 0.24 + 0.62 * np.clip(n @ key, 0, 1) + 0.22 * np.clip(n @ fill, 0, 1)
        return np.clip(np.array(to_rgb(base))[None, :] * np.clip(lit, 0, 1.15)[:, None], 0, 1)

    panels = [("assembled -- 3/4 view", 28, -60, False),
              ("side -- caps hidden under the wall", 0, -90, False),
              ("lid from below -- lips, bosses, sockets", -40, -60, True)]
    fig = plt.figure(figsize=(18, 7), facecolor=bg)
    for k, (title, elev, azim, lid_only) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.set_facecolor(bg)
        for name, m, c in parts:
            if lid_only and name != "lid":
                continue
            ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=shaded(m, c),
                                                 edgecolors="none"))
        half = DECK_APOTHEM * 1.05
        ax.set_xlim(-half, half); ax.set_ylim(-half, half); ax.set_zlim(-10, TOP_H + 5)
        ax.set_box_aspect((2 * half, 2 * half, TOP_H + 15))
        ax.set_axis_off()
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color=fg, fontsize=10)
    fig.suptitle("Crabora lid v1 -- 13 bolted pieces, 6 magnet pucks",
                 color="#d8e2ec", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


if __name__ == '__main__':
    hub, hub_holes = build_hub()
    mid, mid_holes = build_mid()
    outer, outer_holes = build_outer()
    puck = build_puck()
    pieces = {'lid_hub': (hub, 1, hub_holes, to_print_pose),
              'lid_mid': (mid, 6, mid_holes, to_print_pose),
              'lid_outer': (outer, 6, outer_holes, outer_print_pose)}

    total_holes, grams = 0, 0.0
    for name, (mesh, qty, n_holes, pose) in pieces.items():
        printed = pose(mesh)
        printed.export(os.path.join(OUT_DIR, f'{name}.stl'))
        ext = printed.extents
        fits = 'fits' if max(ext[:2]) <= BED_SIZE else 'DOES NOT FIT'
        g = mesh.volume / 1000 * PLA_DENSITY
        grams += qty * g
        total_holes += qty * n_holes
        print(f'{name:9s} x{qty}  {ext[0]:6.1f} x {ext[1]:6.1f} x {ext[2]:4.1f} mm  '
              f'({fits})  watertight={mesh.is_watertight}  {g:5.1f} g  seam holes={n_holes}')
    puck.export(os.path.join(OUT_DIR, 'lid_puck.stl'))
    grams += 6 * puck.volume / 1000 * PLA_DENSITY

    lid = trimesh.util.concatenate([hub] + ring_of(mid) + ring_of(outer))
    lid.export(os.path.join(OUT_DIR, 'lid_assembly.stl'))

    pucks = ring_of(puck.copy().apply_translation([LID_PUCK_X, LID_PUCK_Y, 0.0]))
    caps = cap_boxes()
    clash = sum(intersect(lid_part, other).volume
                for lid_part in [hub] + ring_of(mid) + ring_of(outer)
                for other in caps + pucks)
    print(f'clearance: cap top to ceiling {ceiling(DECK_APOTHEM - CAP_EDGE_SETBACK) - CAP_HEIGHT:.1f} mm '
          f'at the cap\'s outer end, cap end to wall {WALL_IN - (DECK_APOTHEM - CAP_EDGE_SETBACK):.1f} mm, '
          f'lid/cap/puck overlap {clash:.2f} mm3')
    print(f'slope {math.degrees(math.atan(SLOPE_TAN)):.1f}°, weight ~{grams:.0f} g of PLA if solid')
    print(f'seam bolts: {total_holes // 2} x M3x8 + nyloc nut; '
          f'pucks: 6 x M3x12 + nut; magnets: 12 x {MAGNET_DIA:g}x{MAGNET_THK:g} mm')

    deck = prism(hexagon(DECK_APOTHEM), -5.0, 0.0)
    render([("deck", deck, "#3a4250")] + [("cap", c, "#c8a040") for c in caps]
           + [("puck", p, "#d06040") for p in pucks] + [("lid", lid, "#7fa8d0")],
           os.path.join(OUT_DIR, 'lid.png'))
    print('wrote lid.png')
