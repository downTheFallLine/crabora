"""
Parametric generator for the crabora unibody hex assembly:
hub.stl, mid_ring.stl, outer_ring.stl.

Geometry model
--------------
A flat-top regular hexagon (hub) surrounded by two rings of trapezoidal
segments. Each ring segment is a "radial slice" whose two straight side
edges, extended, pass through the hexagon's center -- so hub, mid_ring and
outer_ring all share one running apothem dimension and are *guaranteed* to
fit together by construction:

    hub apothem       A0 = HEX_APOTHEM
    mid_ring spans     A0 -> A1 = A0 + MID_RING_DEPTH
    outer_ring spans   A1 -> A2 = A1 + OUTER_RING_DEPTH

HEX_APOTHEM and MID_RING_DEPTH below were measured directly off the
already-printed hub.stl / mid_ring.stl (bolt-hole notches already removed),
so build_hub()/build_mid_ring() reproduce those parts to within the ~0.2-0.3mm
noise floor of the original (voxelized) meshes -- run compare_to_existing()
to see the actual deviation before trusting a reprint.

outer_ring is derived from the same running apothem, so its inner edge
matches mid_ring's outer edge exactly, regardless of that small noise floor.

No CAD kernel dependency (no CadQuery/OpenSCAD): this environment has a
broken numpy/scipy ABI (system numpy 2.2.5 vs a scipy built for numpy 1.x),
which is exactly the kind of thing that breaks CadQuery's OCP bindings too.
Everything here only needs numpy + numpy-stl, both confirmed working, and
these parts are simple flat extrusions -- no fillets/booleans required.
"""

import numpy as np
from stl import mesh as stlmesh

# ---------------------------------------------------------------------------
# Parameters (mm)
# ---------------------------------------------------------------------------

THICKNESS = 5.0

HEX_APOTHEM = 75.075        # hub flat-to-flat / 2 -- measured from printed hub.stl
MID_RING_DEPTH = 39.9       # measured radial depth of printed mid_ring.stl
OUTER_RING_DEPTH = 66.85    # measured radial depth of outer_ring.stl

A0 = HEX_APOTHEM
A1 = A0 + MID_RING_DEPTH
A2 = A1 + OUTER_RING_DEPTH

TAN30 = np.tan(np.radians(30))

# ---------------------------------------------------------------------------
# outer_ring hole layout -- driven by ~/crabora/leg/coxa/design_coxa_body_cap.py
# ---------------------------------------------------------------------------
# The coxa body cap sits on top of outer_ring and covers the STS3215 servo;
# its horn hole and 4 mounting bolts pass down through the ring beneath it.
# Values below are copied from that script's parameters -- keep in sync if
# the cap design changes.
#
#   CAP_X (cap length, radial direction)      = 52.2
#   HORN_AXIS_OFFSET (horn offset from centre) = 12.5, toward the cap's +X
#       ("narrow"/horn) end -- the end that faces outer_ring's outer edge
#   SCREW_OFFSET_X (bolt offset, radial)       = 15.0
#   SCREW_OFFSET_Y (bolt offset, tangential)   = 15.35
#   HORN_CLEAR_DIA (20mm horn + 2mm slack)     = 22.0
#   MOUNT_HOLE_DIA (M3 clearance)              = 3.4
CAP_LENGTH = 52.2
CAP_HORN_OFFSET = 12.5
CAP_BOLT_RADIAL_OFFSET = 15.0
CAP_BOLT_TANGENT_OFFSET = 15.35

HORN_HOLE_DIAMETER = 22.0
MOUNT_HOLE_DIAMETER = 3.4

# The cap's narrow (horn) end is set back this far from outer_ring's outer
# edge; its wide (cable) end faces inward, toward the hex center.
CAP_EDGE_SETBACK = 5.0

_cap_outer_wall_y = A2 - CAP_EDGE_SETBACK
_cap_center_y = _cap_outer_wall_y - CAP_LENGTH / 2.0

# Offsets below are measured from the segment's inner edge (A1) along its
# centerline, matching the convention build_outer_ring() expects.
HORN_HOLE_OFFSET = (_cap_center_y + CAP_HORN_OFFSET) - A1
MOUNT_HOLE_OFFSETS = [
    (-CAP_BOLT_TANGENT_OFFSET, (_cap_center_y - CAP_BOLT_RADIAL_OFFSET) - A1),
    (CAP_BOLT_TANGENT_OFFSET, (_cap_center_y - CAP_BOLT_RADIAL_OFFSET) - A1),
    (-CAP_BOLT_TANGENT_OFFSET, (_cap_center_y + CAP_BOLT_RADIAL_OFFSET) - A1),
    (CAP_BOLT_TANGENT_OFFSET, (_cap_center_y + CAP_BOLT_RADIAL_OFFSET) - A1),
]

CIRCLE_SEGMENTS = 64

# ---------------------------------------------------------------------------
# brace -- a flat bar glued to the underside of the unibody for reinforcement
# ---------------------------------------------------------------------------
# One brace sits under each of the 6 leg positions, running outward from the
# hex center. Its 15mm-wide face is the glue face (against the body's flat
# underside), so it's extruded exactly like hub/mid_ring/outer_ring -- the
# top/bottom faces of the extrusion *are* the 15x length glue faces, and the
# THICKNESS extrusion depth is the bar's 5mm thickness.
#
# The inner end is pointed rather than square: 6 braces, one per leg position
# 60 degrees apart, all run toward the hex center, and a full-width (15mm)
# bar can't reach the center itself without overlapping its neighbors well
# before getting there. Instead each brace tapers from full width down to a
# point exactly at the origin, with a 30 degree taper half-angle -- matching
# the hex's own 60 degree division -- so adjacent braces' tapered edges are
# parallel to each other and meet edge-to-edge at the center point with
# neither overlap nor gap.
#
# Outer (far) end is just a flat cut; BRACE_LENGTH is set by what fits on the
# Bambu A1 mini's ~180x180mm bed (keep under ~178mm), not by the unibody's
# actual outer radius -- this is a reinforcing bar, not a full spoke.
BRACE_WIDTH = 15.0
BRACE_LENGTH = 175.0
BRACE_TAPER_HALF_ANGLE_DEG = 30.0
BRACE_TAPER_LENGTH = (BRACE_WIDTH / 2.0) / np.tan(np.radians(BRACE_TAPER_HALF_ANGLE_DEG))

# ---------------------------------------------------------------------------
# 2D outline builders
# ---------------------------------------------------------------------------

def regular_hexagon(apothem):
    """Flat-top/bottom hexagon centered at the origin, CCW, pointy left/right."""
    R = apothem / np.cos(np.radians(30))
    angles = np.radians(np.arange(6) * 60.0)
    return np.stack([R * np.cos(angles), R * np.sin(angles)], axis=1)


def ring_segment_outline(inner_a, outer_a):
    """Radial trapezoid spanning apothem inner_a -> outer_a, centered on +Y,
    whose slanted sides extend back to the origin (hex center)."""
    w_in = inner_a * TAN30
    w_out = outer_a * TAN30
    pts = np.array([
        [-w_in, inner_a],
        [w_in, inner_a],
        [w_out, outer_a],
        [-w_out, outer_a],
    ])
    return ensure_ccw(pts)


def brace_outline(width=BRACE_WIDTH, length=BRACE_LENGTH, taper_len=BRACE_TAPER_LENGTH):
    """Flat bar centered on +Y: a point at the origin, tapering out to full
    width by y=taper_len, then a constant-width rectangle out to y=length."""
    hw = width / 2.0
    pts = np.array([
        [0.0, 0.0],
        [hw, taper_len],
        [hw, length],
        [-hw, length],
        [-hw, taper_len],
    ])
    return ensure_ccw(pts)


def circle(cx, cy, diameter, n=CIRCLE_SEGMENTS):
    r = diameter / 2.0
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.stack([cx + r * np.cos(angles), cy + r * np.sin(angles)], axis=1)
    return ensure_cw(pts)  # holes must be CW for the extrude()/bridging logic


def signed_area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ensure_ccw(pts):
    return pts if signed_area(pts) > 0 else pts[::-1].copy()


def ensure_cw(pts):
    return pts if signed_area(pts) < 0 else pts[::-1].copy()


# ---------------------------------------------------------------------------
# Ear-clipping triangulation (supports holes via bridging)
# ---------------------------------------------------------------------------

def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _point_in_tri(p, a, b, c):
    d1, d2, d3 = _cross(a, b, p), _cross(b, c, p), _cross(c, a, p)
    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (has_neg and has_pos)


def ear_clip(points):
    """Simple-polygon ear clipping. Returns a list of (a, b, c) point triples."""
    pts = points if signed_area(points) > 0 else points[::-1].copy()
    n = len(pts)
    nxt = list(range(1, n)) + [0]
    prv = [n - 1] + list(range(0, n - 1))

    def is_convex(i):
        return _cross(pts[prv[i]], pts[i], pts[nxt[i]]) > 1e-12

    reflex = set(i for i in range(n) if not is_convex(i))

    def is_ear(i):
        if not is_convex(i):
            return False
        a, b, c = pts[prv[i]], pts[i], pts[nxt[i]]
        for r in reflex:
            if r in (i, prv[i], nxt[i]):
                continue
            if _point_in_tri(pts[r], a, b, c):
                return False
        return True

    triangles = []
    active = list(range(n))
    remaining = n
    ptr = 0
    guard = 0
    while remaining > 3:
        guard += 1
        if guard > 20 * n:
            raise RuntimeError("ear clipping failed to converge")
        i = active[ptr % len(active)]
        if is_ear(i):
            a_i, c_i = prv[i], nxt[i]
            triangles.append((pts[a_i].copy(), pts[i].copy(), pts[c_i].copy()))
            nxt[a_i] = c_i
            prv[c_i] = a_i
            active.pop(ptr % len(active))
            remaining -= 1
            for v in (a_i, c_i):
                was_reflex = v in reflex
                now_convex = is_convex(v)
                if now_convex and was_reflex:
                    reflex.discard(v)
                elif not now_convex and not was_reflex:
                    reflex.add(v)
            if ptr >= len(active):
                ptr = 0
        else:
            ptr = (ptr + 1) % len(active)

    a_i, b_i, c_i = active
    triangles.append((pts[a_i].copy(), pts[b_i].copy(), pts[c_i].copy()))
    return triangles


def bridge_hole(outer_poly, hole):
    """Splice a CW hole loop into a CCW outer polygon via a zero-width bridge
    so the result is a single simple polygon that ear_clip() can handle."""
    d2 = ((outer_poly[:, None, :] - hole[None, :, :]) ** 2).sum(axis=2)
    oi, hj = np.unravel_index(np.argmin(d2), d2.shape)
    hole_rot = np.roll(hole, -hj, axis=0)
    part1 = outer_poly[:oi + 1]
    part2 = np.vstack([hole_rot, hole_rot[0:1]])
    part3 = outer_poly[oi:oi + 1]
    part4 = outer_poly[oi + 1:]
    return np.vstack([part1, part2, part3, part4])


# ---------------------------------------------------------------------------
# Extrusion -> 3D mesh
# ---------------------------------------------------------------------------

def add_walls(triangles, loop, z0, z1):
    n = len(loop)
    for i in range(n):
        p1, p2 = loop[i], loop[(i + 1) % n]
        b1, b2 = [p1[0], p1[1], z0], [p2[0], p2[1], z0]
        t1, t2 = [p1[0], p1[1], z1], [p2[0], p2[1], z1]
        triangles.append([b1, b2, t2])
        triangles.append([b1, t2, t1])


def extrude(outline, holes, thickness):
    """outline: CCW Nx2 array. holes: list of CW Mx2 arrays. Returns an Fx3x3
    triangle array for a watertight solid from z=0 to z=thickness."""
    outline = ensure_ccw(outline)
    poly = outline.copy()
    for h in holes:
        poly = bridge_hole(poly, ensure_cw(h))

    top_tris = ear_clip(poly)

    z0, z1 = 0.0, thickness
    triangles = []
    for a, b, c in top_tris:
        triangles.append([[a[0], a[1], z0], [c[0], c[1], z0], [b[0], b[1], z0]])
    for a, b, c in top_tris:
        triangles.append([[a[0], a[1], z1], [b[0], b[1], z1], [c[0], c[1], z1]])

    add_walls(triangles, outline, z0, z1)
    for h in holes:
        add_walls(triangles, h, z0, z1)

    return np.array(triangles)


def save_stl(triangles, path):
    data = np.zeros(len(triangles), dtype=stlmesh.Mesh.dtype)
    for i, tri in enumerate(triangles):
        data['vectors'][i] = tri
    m = stlmesh.Mesh(data)
    m.save(path)
    return m


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def build_hub(path='hub_v2.stl'):
    outline = regular_hexagon(A0)
    tris = extrude(outline, [], THICKNESS)
    return save_stl(tris, path)


def build_mid_ring(path='mid_ring_v2.stl'):
    outline = ring_segment_outline(A0, A1)
    tris = extrude(outline, [], THICKNESS)
    return save_stl(tris, path)


def build_outer_ring(path='outer_ring_v2.stl'):
    outline = ring_segment_outline(A1, A2)
    holes = [circle(0.0, A1 + HORN_HOLE_OFFSET, HORN_HOLE_DIAMETER)]
    for x, off in MOUNT_HOLE_OFFSETS:
        holes.append(circle(x, A1 + off, MOUNT_HOLE_DIAMETER))
    tris = extrude(outline, holes, THICKNESS)
    return save_stl(tris, path)


def build_brace(path='brace.stl'):
    outline = brace_outline()
    tris = extrude(outline, [], THICKNESS)
    return save_stl(tris, path)


# ---------------------------------------------------------------------------
# Sanity check against the parts already printed
# ---------------------------------------------------------------------------

def compare_to_existing():
    for gen_path, existing_path in [
        ('hub_v2.stl', 'hub.stl'),
        ('mid_ring_v2.stl', 'mid_ring.stl'),
    ]:
        gen = stlmesh.Mesh.from_file(gen_path)
        existing = stlmesh.Mesh.from_file(existing_path)
        gp = gen.vectors.reshape(-1, 3)
        ep = existing.vectors.reshape(-1, 3)
        print(f"{gen_path} vs {existing_path}:")
        print(f"  generated bounds: {gp.min(axis=0)} .. {gp.max(axis=0)}")
        print(f"  existing  bounds: {ep.min(axis=0)} .. {ep.max(axis=0)}")
        print(f"  max abs difference in bounds: "
              f"{np.abs(gp.max(axis=0) - ep.max(axis=0))}")


if __name__ == '__main__':
    build_hub()
    build_mid_ring()
    build_outer_ring()
    build_brace()
    compare_to_existing()
