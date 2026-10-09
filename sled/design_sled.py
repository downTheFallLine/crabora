"""
CRABORA board sled -- mounting slab for a PCB module
======================================================

A flat rectangular slab with standoffs that raise a PCB off the slab and
line up with the screw holes in the board.  The board screws down to the
standoffs; the slab then carries it as one unit, velcro-taped (and later
glued) to the chassis -- these are not weight-bearing parts.

One script, several boards: pick with --board.  The geometry is the same
for all of them, only the footprint, hole inset and screw size differ.

Geometry:
  - Slab: rectangle, SLAB_THK (3 mm) thick, sized to the board footprint
    plus MARGIN on each side
  - Standoffs: STANDOFF_H (5 mm) tall posts, Φ6, one per PCB screw hole,
    at the four corners inset from the board edges
  - Screw hole through each standoff and the slab: a self-tapping pilot
    sized ~0.8 x the screw's major diameter by default, or a clearance
    hole with --clearance (screw + nut under the slab)
  - Optional --vent cutout through the slab, for airflow

The 5 mm standoff height clears the solder tails on the board's
underside, so the PCB sits on the posts and nothing touches the slab.

Print orientation: slab flat on the bed, standoffs pointing up.  No
supports needed.  Both boards are well inside the A1 mini's ~180 mm
plate.  The bed face comes out flat and smooth, which is what the velcro
adhesive wants; scuff it with 120-220 grit before gluing later.

Usage:
  python design_sled.py --board buck      # SELOKY LM2596 buck converter
  python design_sled.py --board urt       # Feetech FE-URT-2
  python design_sled.py --board urt --clearance --vent
  python design_sled.py --board buck --pilot 1.5
"""

import argparse
import os

import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# BOARDS
# =============================================================
# length / width : PCB footprint, mm
# inset          : hole centre to the nearest PCB edge, both directions
# screw_major    : screw outside thread diameter, mm
#
# ⚠ Dimensions measured off Tom's actual boards -- note the buck is the
# LARGER LM2596 variant; plenty of listings sell a ~43 x 21 mm board
# under the same name, so don't "correct" it back to that.
BOARDS = {
    "buck": {
        "name": "SELOKY LM2596 buck converter",
        "length": 66.0,
        "width": 36.0,
        "inset": 2.0,
        "screw_major": 1.8,          # measured 2026-10-08
        "screw_verified": True,
    },
    "urt": {
        "name": "Feetech FE-URT-2",
        "length": 56.5,
        "width": 36.5,
        "inset": 3.0,                # measured 2026-10-08
        "screw_major": 1.8,          # ASSUMED same as the buck -- unmeasured
        "screw_verified": False,
    },
}


# =============================================================
# PARAMETERS  (mm)
# =============================================================
SLAB_THK              = 3.0       # slab thickness
MARGIN                = 3.0       # slab border beyond the PCB footprint

STANDOFF_H            = 5.0       # height above the slab top face
STANDOFF_DIA          = 6.0       # outer diameter of each post

# Self-tapping pilot is ~0.8 x the screw's major diameter, nudged up a
# touch because FDM prints small holes undersize.  Clearance adds 0.3.
PILOT_RATIO           = 0.89
CLEARANCE_EXTRA       = 0.3

# Optional ventilation cutout, as a fraction of the board footprint
VENT_FRAC             = 0.30

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


# =============================================================
# HELPERS
# =============================================================
def z_cylinder(radius, height, center, sections=48):
    cyl = creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_translation(center)
    return cyl


def hole_positions(dx, dy):
    """Standoff centres in the slab frame (origin at slab centre)."""
    hx, hy = dx / 2.0, dy / 2.0
    return [(+hx, +hy), (+hx, -hy), (-hx, -hy), (-hx, +hy)]


# =============================================================
# BUILD
# =============================================================
def build_sled(board, pilot=None, clearance=False, vent=False):
    """CAD frame: origin at the slab's centre on its BOTTOM face (Z=0).
    Slab occupies Z = 0 .. SLAB_THK; standoffs rise to SLAB_THK + STANDOFF_H."""

    dx = board["length"] - 2 * board["inset"]
    dy = board["width"] - 2 * board["inset"]
    centres = hole_positions(dx, dy)

    if pilot is None:
        pilot = round(board["screw_major"] * PILOT_RATIO, 1)
    screw_dia = (board["screw_major"] + CLEARANCE_EXTRA) if clearance else pilot

    # Slab must cover the PCB footprint AND every standoff, whichever is
    # wider -- a hole pattern wider than the board would otherwise leave a
    # post hanging off the edge.
    span_x = dx + STANDOFF_DIA
    span_y = dy + STANDOFF_DIA
    slab_l = max(board["length"] + 2 * MARGIN, span_x + 2.0)
    slab_w = max(board["width"] + 2 * MARGIN, span_y + 2.0)

    # ---- positive: slab + standoffs --------------------------------
    parts = []
    slab = creation.box(extents=[slab_l, slab_w, SLAB_THK])
    slab.apply_translation([0.0, 0.0, SLAB_THK / 2.0])
    parts.append(slab)

    for cx, cy in centres:
        parts.append(z_cylinder(
            STANDOFF_DIA / 2.0, STANDOFF_H,
            [cx, cy, SLAB_THK + STANDOFF_H / 2.0]))

    # ---- negative cutters ------------------------------------------
    cutters = []
    total_h = SLAB_THK + STANDOFF_H
    for cx, cy in centres:
        cutters.append(z_cylinder(
            screw_dia / 2.0, total_h + 2.0, [cx, cy, total_h / 2.0]))

    vent_dims = None
    if vent:
        vent_dims = (board["length"] * VENT_FRAC, board["width"] * VENT_FRAC)
        hole = creation.box(extents=[vent_dims[0], vent_dims[1], SLAB_THK + 2.0])
        hole.apply_translation([0.0, 0.0, SLAB_THK / 2.0])
        cutters.append(hole)

    sled = trimesh.boolean.union(parts)
    sled = trimesh.boolean.difference([sled] + cutters)
    sled.merge_vertices()
    sled.fix_normals()
    return sled, dict(slab_l=slab_l, slab_w=slab_w, screw_dia=screw_dia,
                      centres=centres, dx=dx, dy=dy, vent=vent_dims)


# =============================================================
# PREVIEW
# =============================================================
def render_png(sled, path, board, info):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.colors import to_rgb

    bg = "#0c0d11"; fg = "#9fb3c8"
    key  = np.array([0.35, 0.45, 0.82]); key  = key  / np.linalg.norm(key)
    fill = np.array([-0.55, -0.25, 0.30]); fill = fill / np.linalg.norm(fill)

    def shaded(mesh, base):
        n = mesh.face_normals
        lit = (0.24 + 0.62 * np.clip(n @ key, 0, 1) + 0.22 * np.clip(n @ fill, 0, 1))
        col = np.array(to_rgb(base))
        return np.clip(col[None, :] * np.clip(lit, 0, 1.15)[:, None], 0, 1)

    col = "#b9c2cf"
    b = sled.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.15

    panels = [
        ("3/4 view  (standoffs up)", 28, -55),
        ("top down  (hole pattern)", 88, -90),
        ("side view (slab + standoff)", 2, -90),
    ]
    fig = plt.figure(figsize=(12, 4.5), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            sled.triangles, facecolors=shaded(sled, col), edgecolors="none"))
        ax.set_xlim(ctr[0] - half, ctr[0] + half)
        ax.set_ylim(ctr[1] - half, ctr[1] + half)
        ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()
        ax.set_facecolor(bg)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color=fg, fontsize=11)

    fig.suptitle(
        f"CRABORA sled -- {board['name']}  -  slab "
        f"{info['slab_l']:.1f} x {info['slab_w']:.1f} x {SLAB_THK:.0f} mm, "
        f"{STANDOFF_H:.0f} mm standoffs",
        color="#d8e2ec", fontsize=13, y=1.04)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(
        description="Build a CRABORA board sled.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", choices=sorted(BOARDS), required=True,
                    help="which board the sled is for")
    ap.add_argument("--pilot", type=float,
                    help="override the self-tapping pilot diameter (mm)")
    ap.add_argument("--clearance", action="store_true",
                    help="clearance holes (screw + nut) instead of self-tapping pilots")
    ap.add_argument("--vent", action="store_true",
                    help="cut a ventilation window through the slab")
    args = ap.parse_args()

    board = BOARDS[args.board]
    print(f"Building sled for {board['name']}...")
    sled, info = build_sled(board, pilot=args.pilot,
                            clearance=args.clearance, vent=args.vent)

    e = sled.extents
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {sled.volume / 1000.0:.2f} cm^3   watertight {sled.is_watertight}")
    print(f"  board     : {board['length']:g} x {board['width']:g} mm PCB, "
          f"holes inset {board['inset']:g} mm from each edge")
    print(f"  slab      : {info['slab_l']:.1f} x {info['slab_w']:.1f} x "
          f"{SLAB_THK:.0f} mm  (+{MARGIN:.0f} mm margin)")
    print(f"  standoffs : 4x Φ{STANDOFF_DIA:.0f} x {STANDOFF_H:.0f} mm tall  at "
          f"{', '.join(f'({x:+.2f},{y:+.2f})' for x, y in info['centres'])}")
    print(f"  hole span : {info['dx']:g} x {info['dy']:g} mm centre-to-centre")
    print(f"  screws    : Φ{info['screw_dia']:.1f} "
          f"({'clearance, nut under slab' if args.clearance else 'self-tapping pilot'})"
          f"  for a {board['screw_major']:g} mm thread")
    if not board["screw_verified"]:
        print(f"    ⚠ the {board['screw_major']:g} mm screw size for this board is "
              f"ASSUMED, not measured --")
        print(f"      check it and pass --pilot if it differs.")
    if info["vent"]:
        print(f"  vent      : {info['vent'][0]:.0f} x {info['vent'][1]:.0f} mm "
              f"window through the slab")
    print(f"  print     : slab flat on bed, standoffs up -- no supports")

    stl_path = f"{OUT_DIR}/{args.board}_sled.stl"
    png_path = f"{OUT_DIR}/{args.board}_sled.png"
    sled.export(stl_path)
    print(f"✓ wrote {stl_path}")
    print("Rendering preview...")
    render_png(sled, png_path, board, info)
    print(f"✓ wrote {png_path}")


if __name__ == "__main__":
    main()
