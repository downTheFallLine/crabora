"""
CRABORA coxa-pan plate v5 -- FLAT paddle for lying-on-side femur cap
=====================================================================

Reverts the L-bracket experiment (v4) back to a flat horizontal paddle
(like v3) -- much stronger, simpler to print, less cantilever stress at
the disc-to-wall junction.  Bolt holes repositioned to host the v4
femur cap (which lies on its side and bolts down through its ±X walls).

Shape: a flat "lollipop"
  - DISC at one end (horn-attach): Φ32 x 3 mm, with 4 M3 horn-bolt
    holes on the Φ14 bolt circle (10 mm square) + Φ8 central relief
    for the horn's central M3 screw head.
  - PADDLE extending outboard: a flat rectangle hosting 4 M3 clearance
    holes for the femur cap v4 bolts at (CAP_CENTER_X ± 26.6, ±14)
    in pan frame.

The femur cap v4 lies flat on the paddle (open face down) and is
clamped down by 4 long M3 bolts that drop in from the top of the cap,
through the cap walls, through the paddle, and secured with nuts on the
paddle's underside.

Cap install in pan frame: cap origin (cap centre on open face) sits at
pan (CAP_CENTER_X=45.5, 0, PADDLE_THK=5).  Femur horn axis lands at
pan (58, 21, 17.6) -- radial 58 (was 46 in earlier versions; bumped
+12 mm so the cap clears the coxa-horn bolt heads at X=±5 underneath
the disc), tangent 21 (close to v2's 20.5), vertical 17.6 above pan
disc bottom.  Bolt span = cap 28.2 + pan 5 + nut+washer ~3 = ~36 mm
-> M3 x 40 mm cap-mount bolts.

Print orientation: flat on the bed, disc side down (or either side --
plate is symmetric in Z apart from the through-holes).

Usage:
  python design_coxa_pan.py
"""

import os
import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  (mm)
# =============================================================
# ---- Disc (horn-attach end) ----
DISC_DIA              = 32.0
DISC_THK              = 5.0
HORN_BOLT_SQUARE      = 10.0   # 4 bolts at ±5,±5 = Φ14 bolt circle
HORN_BOLT_CLEAR       = 3.4    # M3 medium clearance
HORN_CENTRAL_CLEAR    = 8.0    # Φ8 clearance for horn's central M3 screw head

# ---- Paddle (flat, hosts femur cap v4) ----
# v4 femur cap bolts are at (±26.6, ±14) in cap frame; cap body extends
# ±30.1 in X.  Cap MUST clear the coxa-horn bolt heads on the disc, which
# sit at (±5, ±5) with M3 cap-head OD ~5.5 (outer edge at X ≈ ±7.75).
# Cap centred at pan X = 45.5 puts its inboard edge at X = 15.4 -- just
# past the disc edge at X = 16, and ~7.65 mm clear of the horn bolt heads.
# Femur horn axis lands at radial X = 58 (= 45.5 + cap HORN_AXIS_OFFSET 12.5).
CAP_CENTER_X          = 45.5   # pan-frame X position of cap centre
                               # (shifted +12 mm outboard from v5 first cut
                               # so the cap clears the coxa-horn bolts)

# Paddle extent: must cover both inboard bolt (X = CAP_CENTER_X - 26.6 = 18.9)
# and outboard bolt (X = CAP_CENTER_X + 26.6 = 72.1) with margin.
# Inboard end stops at X = 8 -- past the horn bolt heads (outer ~7.75)
# but overlapping the disc (radius 16) by 8 mm for a sturdy neck.
PADDLE_X_LO           = 8.0    # past horn bolt heads, 8 mm overlap into disc
PADDLE_X_HI           = 77.0   # 5 mm past outboard cap-bolt hole at X=72.1
PADDLE_Y_HALF         = 21.0   # ±Y half-width.  Bolt at ±14 leaves 7 mm margin.
PADDLE_THK            = 5.0    # same as disc -- one continuous plate

# ---- Mount holes for the femur cap v4 (axis along Z, thru paddle) ----
CAP_SCREW_OFFSET_X    = 26.6   # ±X from cap centre, matches cap SCREW_OFFSET_X
CAP_SCREW_OFFSET_Y    = 14.0   # ±Y from cap centre, matches cap SCREW_OFFSET_Y
PADDLE_MOUNT_CLEAR    = 3.4    # M3 clearance

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/coxa_pan.stl"
PNG_PATH = OUT_DIR + "/coxa_pan.png"


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
def build_pan():
    """CAD frame: origin at the HORN AXIS, on the BOTTOM face of the
    plate (the face that meets the coxa horn output).
      +X = radial OUTBOARD
      +Y = tangent
      +Z = up (toward the femur cap's open face)"""

    # ---------------------------------------------------------------
    # POSITIVE solids: disc + paddle
    # ---------------------------------------------------------------
    disc = creation.cylinder(radius=DISC_DIA / 2.0, height=DISC_THK,
                             sections=96)
    disc.apply_translation([0.0, 0.0, DISC_THK / 2.0])

    paddle_x_extent = PADDLE_X_HI - PADDLE_X_LO
    paddle = creation.box(extents=[
        paddle_x_extent,
        2 * PADDLE_Y_HALF,
        PADDLE_THK])
    paddle.apply_translation([
        (PADDLE_X_LO + PADDLE_X_HI) / 2.0,
        0.0,
        PADDLE_THK / 2.0])

    body = trimesh.boolean.union([disc, paddle])

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # Disc: central clearance hole for horn's central M3 screw head
    cutters.append(z_cylinder(
        HORN_CENTRAL_CLEAR / 2.0, PADDLE_THK + 2.0,
        [0.0, 0.0, PADDLE_THK / 2.0]))

    # Disc: 4 horn-bolt clearance holes on the 10 mm square
    h = HORN_BOLT_SQUARE / 2.0
    for dx, dy in [(+h, +h), (+h, -h), (-h, -h), (-h, +h)]:
        cutters.append(z_cylinder(
            HORN_BOLT_CLEAR / 2.0, PADDLE_THK + 2.0,
            [dx, dy, PADDLE_THK / 2.0]))

    # Paddle: 4 cap-mount clearance holes (axis along Z, full paddle thk)
    for dx in (-CAP_SCREW_OFFSET_X, +CAP_SCREW_OFFSET_X):
        for dy in (-CAP_SCREW_OFFSET_Y, +CAP_SCREW_OFFSET_Y):
            cutters.append(z_cylinder(
                PADDLE_MOUNT_CLEAR / 2.0, PADDLE_THK + 2.0,
                [CAP_CENTER_X + dx, dy, PADDLE_THK / 2.0]))

    pan = trimesh.boolean.difference([body] + cutters)
    pan.merge_vertices()
    pan.fix_normals()
    return pan


# =============================================================
# PREVIEW
# =============================================================
def render_png(pan, path):
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

    col = "#cf8b6e"
    b = pan.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.1

    panels = [
        ("3/4 view",                       25, -55),
        ("top down  (face that meets horn / cap mating face)", 88, -90),
        ("side view",                       0, -90),
    ]
    fig = plt.figure(figsize=(15, 5.5), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            pan.triangles, facecolors=shaded(pan, col),
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
        f"CRABORA coxa-pan plate  v5 (FLAT paddle for v4 lying-on-side cap)  "
        f"-  disc Φ{DISC_DIA:.0f}, paddle {PADDLE_X_HI-PADDLE_X_LO:.0f}x{2*PADDLE_Y_HALF:.0f} mm",
        color="#d8e2ec", fontsize=13, y=1.02)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building coxa-pan plate v5 (flat paddle for v4 lying-on-side femur cap)...")
    pan = build_pan()
    e = pan.extents
    vol = pan.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {pan.is_watertight}")
    print(f"  disc      : Φ{DISC_DIA:.0f} x {DISC_THK:.0f} mm thick  "
          f"(on horn, 4x M3 horn bolts + Φ{HORN_CENTRAL_CLEAR:.0f} central relief)")
    print(f"  paddle    : X=[{PADDLE_X_LO:.0f}, {PADDLE_X_HI:.0f}] x "
          f"Y=±{PADDLE_Y_HALF:.0f} x Z={PADDLE_THK:.0f}  "
          f"({PADDLE_X_HI-PADDLE_X_LO:.0f} long x {2*PADDLE_Y_HALF:.0f} wide)")
    print(f"  cap bolts : 4x M3 Φ{PADDLE_MOUNT_CLEAR:.1f} thru paddle at "
          f"X={CAP_CENTER_X:.1f}±{CAP_SCREW_OFFSET_X:.1f}, "
          f"Y=±{CAP_SCREW_OFFSET_Y:.1f}")
    print(f"  derived femur joint axis (pan frame): "
          f"X={CAP_CENTER_X + 12.5:.1f}, Y={36/2 + 3/2:.1f}, Z={PADDLE_THK + 25.2/2:.1f}")
    print(f"  hardware  : 4x M3 x 40 mm bolts + M3 nuts on paddle underside")

    pan.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(pan, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
