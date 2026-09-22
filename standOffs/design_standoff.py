"""
CRABORA PCB standoff, v1
========================

A short cylindrical standoff that sits between a PC board and the base.
The standoff's bottom face is glued to the base; the PCB rests on its top
face. A 1.5 mm pilot hole runs all the way through the standoff so a 2 mm
wood screw can self-tap down through it (and through the PCB's own hole)
into the base underneath -- the pilot is undersized relative to the screw
on purpose, so the screw's threads cut into the standoff's plastic and
grip it, rather than passing through with clearance.

    PCB  ───────▭───────
              │▒▒▒│         <- screw head bears here (or on PCB pad)
              │▒▒▒│         <- 2mm wood screw self-taps through 1.5mm pilot
    stand-  ──┤▒▒▒├──  5mm tall
    off       │▒▒▒│
    base ─────┴▒▒▒┴─────    <- standoff glued here; screw bites into base too

No CAD kernel dependency (no CadQuery/OpenSCAD/trimesh booleans): this
environment has a broken numpy/scipy ABI and no boolean backend
installed, the same issue noted in ../unibody/generate_parts.py. This
script reuses that file's plain numpy + numpy-stl approach (manual
polygon extrusion via ear-clipping, holes spliced in by bridging) --
a standoff is just a circle with a concentric circular hole, extruded,
so no CAD kernel is needed at all.

Usage:
  python3 design_standoff.py
"""

import numpy as np
from stl import mesh as stlmesh

# =============================================================
# PARAMETERS (mm)
# =============================================================
HEIGHT = 5.0            # standoff height: PCB-to-base gap

SCREW_DIA = 2.0          # nominal wood screw shank diameter, for reference/printing only
BORE_DIA = 1.5           # pilot hole -- undersized so the screw self-taps into it

OUTER_DIA = 6.0          # standoff outer diameter
WALL_THK = (OUTER_DIA - BORE_DIA) / 2.0  # 1.8 -- resulting wall thickness

CIRCLE_SEGMENTS = 32

OUT_PATH = "standoff.stl"


# =============================================================
# Geometry helpers (plain numpy + numpy-stl, no CAD kernel needed)
# same approach as ../unibody/generate_parts.py -- see that file's
# header comment for why: this machine's numpy/scipy ABI is broken
# and no trimesh boolean backend is installed, so booleans/CadQuery
# are avoided entirely in favor of manual polygon extrusion.
# =============================================================

def circle(cx, cy, diameter, n=CIRCLE_SEGMENTS):
    r = diameter / 2.0
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.stack([cx + r * np.cos(angles), cy + r * np.sin(angles)], axis=1)
    return ensure_cw(pts)  # holes must be CW -- add_walls() uses this array
                            # as-is for hole walls, without re-deriving winding


def signed_area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ensure_ccw(pts):
    return pts if signed_area(pts) > 0 else pts[::-1].copy()


def ensure_cw(pts):
    return pts if signed_area(pts) < 0 else pts[::-1].copy()


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


# =============================================================
# Part builder
# =============================================================

def build_standoff(path=OUT_PATH):
    outline = circle(0.0, 0.0, OUTER_DIA)
    bore = circle(0.0, 0.0, BORE_DIA)
    tris = extrude(outline, [bore], HEIGHT)
    return save_stl(tris, path)


# =============================================================
# Main
# =============================================================

def main():
    m = build_standoff()
    print(f"standoff: OD {OUTER_DIA:.1f} mm, pilot hole {BORE_DIA:.1f} mm "
          f"(wall {WALL_THK:.2f} mm), height {HEIGHT:.1f} mm")
    print(f"  screw: {SCREW_DIA:.1f} mm wood screw self-taps into the pilot hole")
    print(f"  triangles: {len(m.vectors)}")
    print(f"✓ wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
