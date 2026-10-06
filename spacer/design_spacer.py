"""
CRABORA horn spacer
====================

A short cylindrical spacer that sits between an STS3215's horn output
face and the mating link's hip disc.  Lets you set a precise vertical
gap between the servo body and the rotating link -- e.g. to keep a
link arm clear of the servo case while it sweeps, or to align the
link's plane with another reference.

Geometry:
  - Φ19.9 mm cylinder, thickness set on the command line (default 8.5)
  - 4 × M3 clearance holes on the standard 10 mm square horn pattern
    (±5 mm from centre, matching coxa_pan / femur_link / tibia_link)
  - Φ8 central clearance for the horn's central M3 screw head and
    centre boss (same convention as the other horn-bolted discs)

Print orientation: flat on the bed, either face down -- the part is
symmetric in Z apart from the through-holes.

Usage:
  python design_spacer.py              # default DISC_THK
  python design_spacer.py 12.5         # -> spacer/horn_spacer.12.5mm.stl
"""

import argparse
import os

import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  (mm)
# =============================================================
DISC_DIA              = 19.9       # outer diameter of the spacer
DISC_THK              = 8.5        # default thickness (override on command line)

# Horn bolt pattern (matches coxa_pan / femur_link / tibia_link discs)
HORN_BOLT_SQUARE      = 10.0       # 4 bolts at ±5,±5 = Φ14 bolt circle
HORN_BOLT_CLEAR       = 3.4        # M3 medium clearance

# Central clearance for the horn's central M3 screw head + centre boss
HORN_CENTRAL_CLEAR    = 8.0

OUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spacer")
PNG_PATH = OUT_DIR + "/horn_spacer.png"


def stl_path(thk):
    return f"{OUT_DIR}/horn_spacer.{thk:g}mm.stl"


# =============================================================
# HELPERS
# =============================================================
def z_cylinder(radius, height, center, sections=48):
    cyl = creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_translation(center)
    return cyl


# =============================================================
# BUILD
# =============================================================
def build_spacer(thk=DISC_THK):
    """CAD frame: origin at disc centre, on the disc's mid-plane (Z=0).
    Disc occupies Z = -thk/2 .. +thk/2."""

    # ---- positive: the disc ----------------------------------------
    body = creation.cylinder(
        radius=DISC_DIA / 2.0, height=thk, sections=96)

    # ---- negative cutters ------------------------------------------
    cutters = []

    # Central clearance hole (full thickness)
    cutters.append(z_cylinder(
        HORN_CENTRAL_CLEAR / 2.0, thk + 2.0, [0.0, 0.0, 0.0]))

    # 4 horn-bolt clearance holes at ±5, ±5
    h = HORN_BOLT_SQUARE / 2.0
    for dx, dy in [(+h, +h), (+h, -h), (-h, -h), (-h, +h)]:
        cutters.append(z_cylinder(
            HORN_BOLT_CLEAR / 2.0, thk + 2.0, [dx, dy, 0.0]))

    spacer = trimesh.boolean.difference([body] + cutters)
    spacer.merge_vertices()
    spacer.fix_normals()
    return spacer


# =============================================================
# PREVIEW
# =============================================================
def render_png(spacer, path):
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
    b = spacer.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.15

    panels = [
        ("3/4 view", 25, -55),
        ("top down  (face)", 88, -90),
        ("side view (thickness)", 0, -90),
    ]
    fig = plt.figure(figsize=(12, 4.5), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            spacer.triangles, facecolors=shaded(spacer, col),
            edgecolors="none"))
        ax.set_xlim(ctr[0] - half, ctr[0] + half)
        ax.set_ylim(ctr[1] - half, ctr[1] + half)
        ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()
        ax.set_facecolor(bg)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color=fg, fontsize=11)

    fig.suptitle(
        f"CRABORA horn spacer  -  Φ{DISC_DIA:.1f} × {spacer.extents[2]:.1f} mm",
        color="#d8e2ec", fontsize=13, y=1.04)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    ap = argparse.ArgumentParser(description="Build a CRABORA horn spacer.")
    ap.add_argument("thickness", type=float, nargs="?", default=DISC_THK,
                    help=f"spacer thickness in mm (default {DISC_THK})")
    thk = ap.parse_args().thickness

    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building horn spacer...")
    spacer = build_spacer(thk)
    e = spacer.extents
    vol = spacer.volume / 1000.0
    print(f"  bbox {e[0]:.2f} x {e[1]:.2f} x {e[2]:.2f} mm   "
          f"vol {vol:.2f} cm^3   watertight {spacer.is_watertight}")
    print(f"  disc      : Φ{DISC_DIA:.1f} x {thk:g} mm thick")
    print(f"  bolt holes: 4x M3 clearance Φ{HORN_BOLT_CLEAR:.1f} on "
          f"{HORN_BOLT_SQUARE:.0f} mm square  (+ Φ{HORN_CENTRAL_CLEAR:.0f} central relief)")

    path = stl_path(thk)
    spacer.export(path)
    print(f"✓ wrote {path}")
    print("Rendering preview...")
    render_png(spacer, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
