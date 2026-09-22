"""
CRABORA side shim, v1
======================

A plain rectangular shim for packing out a side-wall gap (e.g. the
coxa cap's side walls against the STS3215 servo body, POCKET_FIT_GAP
in ../leg/coxa/design_coxa_cap.py) -- thin enough to tune the fit
without needing to reprint the whole cap.

    ┌───────────────────────────────┐
    │                                 │  35 mm (Y)
    └───────────────────────────────┘
              45 mm (X)                 0.4 mm thick (Z)

Same approach as design_coxa_cap_shim.py in this directory: plain
numpy + numpy-stl, no CAD kernel / trimesh boolean, since this
environment's python3 has a broken numpy/scipy ABI (see that file's
header for details).

Usage:
  python3 design_side_shim.py
"""

import numpy as np
from stl import mesh as stlmesh

# =============================================================
# PARAMETERS (mm)
# =============================================================
SHIM_L = 45.0   # X
SHIM_W = 35.0   # Y
SHIM_T = 0.4    # Z (thickness)

OUT_PATH = "side_shim.stl"


# =============================================================
# Geometry
# =============================================================
def build_box(l, w, t):
    """Axis-aligned box, corner at origin, spanning (0,0,0) to (l,w,t)."""
    x0, x1 = 0.0, l
    y0, y1 = 0.0, w
    z0, z1 = 0.0, t

    v = {
        "000": [x0, y0, z0], "100": [x1, y0, z0],
        "110": [x1, y1, z0], "010": [x0, y1, z0],
        "001": [x0, y0, z1], "101": [x1, y0, z1],
        "111": [x1, y1, z1], "011": [x0, y1, z1],
    }

    # Each face as two CCW (outward-facing) triangles.
    quads = [
        ("010", "110", "100", "000"),  # bottom (normal -Z)
        ("101", "111", "011", "001"),  # top    (normal +Z)
        ("001", "011", "010", "000"),  # -X side
        ("110", "111", "101", "100"),  # +X side
        ("100", "101", "001", "000"),  # -Y side
        ("011", "111", "110", "010"),  # +Y side
    ]

    triangles = []
    for a, b, c, d in quads:
        triangles.append([v[a], v[b], v[c]])
        triangles.append([v[a], v[c], v[d]])

    return np.array(triangles, dtype=np.float64)


def save_stl(triangles, path):
    data = np.zeros(len(triangles), dtype=stlmesh.Mesh.dtype)
    for i, tri in enumerate(triangles):
        data['vectors'][i] = tri
    m = stlmesh.Mesh(data)
    m.save(path)
    return m


# =============================================================
# Main
# =============================================================
def main():
    tris = build_box(SHIM_L, SHIM_W, SHIM_T)
    m = save_stl(tris, OUT_PATH)
    print(f"side shim: {SHIM_L:.1f} x {SHIM_W:.1f} x {SHIM_T:.1f} mm")
    print(f"  triangles: {len(m.vectors)}")
    print(f"✓ wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
