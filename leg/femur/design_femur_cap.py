"""
CRABORA femur cap v4 -- lying on side, FLAT pan mount, Φ32 open back
=====================================================================

Servo orientation (lying flat inside the cap):
  - L (45.2) along cap X = radial direction in operating
  - H (35)   along cap Y = tangent direction = horn axis
  - W (24.7) along cap Z = vertical (servo sits ON the pan)
  - Horn axis offset +12.5 mm in cap X (= STS3215 horn offset along L)

Cap is OPEN at the BOTTOM (-Z) -- it slides down over the servo from
above and seats on the pan's flat top.

Cap walls:
  - +Y BACK wall (3.45 mm): Φ22 horn U-slot, offset +12.5 in X --
    semicircle on the roof side, lower half open straight down to the
    open face (Z=0) so the cap slides over a servo with horn attached.
    Horn pokes through to drive the femur link beyond.
  - -Y FRONT wall (3.45 mm): SQUARED-OFF OPEN BACK -- a rectangular
    cutout 30 W x 25 H, centred horizontally on the wall, with its
    BOTTOM edge flush with the cap's open face (Z=0).  TOP edge at
    Z=25 stops below the cap roof (Z=[25.2, 28.2]), so the top wall
    is fully intact -- NO notch in the cap top.  The -Y wall ends up
    as a U-shape: two vertical pillars on either side of the opening,
    connected by a thin strip just under the roof.  Serves as cable
    exit + servo back-face access + weight relief.
  - ±X SIDE walls (7.6 mm): solid, host 4 vertical M3 bolt passages
    that clamp the cap down to the flat pan.  Bolts at (±26.6, ±14)
    in cap frame (fixed to match the pan).
  - +Z TOP wall (3 mm): closed roof above the servo.

Bolt span: cap (28.2) + pan (5) + nut+washer (~3) = ~36 mm.
M3 x 40 mm cap-head bolts (much shorter than tibia cap's M3 x 50,
and far shorter than v2 femur cap's M3 x 55).

Operating-orientation mapping (no rotation needed when installed):
  cap +X  ->  pan +X   (radial outboard)
  cap +Y  ->  pan +Y   (tangent, horn axis)
  cap +Z  ->  pan +Z   (vertical up)

Femur joint axis lands at pan (46, 19.5, 15.6) when cap origin is
placed at pan (33.5, 0, 3) -- radial offset 46 (matches all prior
versions); tangent offset 19.5 (was 20.5 in v2, 40.5 in v3 L-bracket);
vertical 15.6 above pan disc bottom (much lower than v2's 38).

Print orientation: closed top face DOWN on the bed, open face UP.
The Φ32 back opening becomes a horizontal-axis arch that spans the
full Z range of the wall -- bridging the open arc requires no support
because the cut goes all the way through the top of the wall.

Usage:
  python design_femur_cap.py
"""

import os
import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS (mm)
# =============================================================
# STS3215 body
SERVO_L                  = 45.2
SERVO_W                  = 24.7
SERVO_H                  = 35.0
HORN_AXIS_OFFSET         = 12.5    # from servo body centre, along L = cap X
HORN_DISC_THK            = 3.1

# Pocket fit (servo lying on side: L=cap X, H=cap Y=horn axis, W=cap Z)
POCKET_FIT_GAP           = 0.5
POCKET_LENGTH_TRIM       = 1.2     # tighten pocket length; absorbed by thicker ±X
                                   # walls so the outer length (CAP_X) and bolt
                                   # positions (SCREW_OFFSET_X) don't move
POCKET_X = SERVO_L + 2 * POCKET_FIT_GAP - POCKET_LENGTH_TRIM   # 45.0  (cap X = servo L  = radial)
POCKET_WIDTH_TRIM        = 0.9     # tighten pocket width; absorbed by thicker ±Y
                                   # walls so the outer footprint (CAP_Y) doesn't change
POCKET_Y = SERVO_H + 2 * POCKET_FIT_GAP - POCKET_WIDTH_TRIM   # 35.1  (cap Y = servo H  = horn axis / tangent)
POCKET_Z = SERVO_W + 0.5                  # 25.2  (cap Z = servo W  = vertical)

# Cap walls
WALL_THK_X               = 7.0 + POCKET_LENGTH_TRIM / 2.0   # 7.6 -- ±X SIDE walls, thick,
                                   # host vertical bolt passages, plus half the
                                   # POCKET_LENGTH_TRIM each so CAP_X stays 60.2.
WALL_THK_Y               = 3.0 + POCKET_WIDTH_TRIM / 2.0   # 3.45 -- ±Y walls,
                                   # +Y has Φ22 horn U-slot, -Y has rect open back
TOP_THK                  = 3.0     # +Z TOP wall (closed roof)

# Derived cap outer dimensions
CAP_X = POCKET_X + 2 * WALL_THK_X                       # 60.2
CAP_Y = POCKET_Y + 2 * WALL_THK_Y                       # 42.0
CAP_Z = POCKET_Z + TOP_THK                              # 28.2

# +Y BACK wall horn hole (same as tibia cap)
HORN_CLEAR_DIA           = 22.0

# -Y FRONT wall SQUARED-OFF OPENING (rectangular, no top notch)
BACK_OPEN_W              = 30.0    # X width  (centred horizontally on wall)
BACK_OPEN_H              = 25.0    # Z height (bottom flush with open face Z=0,
                                   # top at Z=25 -- stops 0.2 mm under pocket
                                   # top so the cap roof Z=[25.2, 28.2] stays
                                   # fully intact, NO notch in the top wall)

# Vertical bolt passages through ±X walls, open face -> top
MOUNT_HOLE_DIA           = 3.4     # M3 clearance
SCREW_OFFSET_X           = 26.6    # ±X from cap centre -- FIXED (matches the coxa
                                   # pan's CAP_SCREW_OFFSET_X); not derived from the
                                   # pocket, so pocket trims can't move the bolts.
                                   # Leaves 2.4 mm wall inboard / 1.8 mm outboard.
SCREW_OFFSET_Y           = 14.0    # ±Y from cap centre

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/femur_cap.stl"
PNG_PATH = OUT_DIR + "/femur_cap.png"


# =============================================================
# HELPERS
# =============================================================
def z_cylinder(radius, height, center, sections=48):
    cyl = creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_translation(center)
    return cyl


def y_cylinder(radius, height, center, sections=48):
    """Cylinder with axis along Y."""
    from trimesh import transformations as tf
    cyl = creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_transform(tf.rotation_matrix(np.pi / 2, [1, 0, 0]))
    cyl.apply_translation(center)
    return cyl


# =============================================================
# BUILD
# =============================================================
def build_cap():
    """CAD frame: origin at cap's centre on the OPEN FACE (Z=0).
    Cap +Z is the closed top wall direction.  Cap -Z (open face) mates
    flush with the pan's flat top surface."""

    # ---------------------------------------------------------------
    # POSITIVE solid: cap shoebox
    # ---------------------------------------------------------------
    body = creation.box(extents=[CAP_X, CAP_Y, CAP_Z])
    body.apply_translation([0.0, 0.0, CAP_Z / 2.0])

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # Servo pocket: open at Z=0 (bottom), extends up to Z = POCKET_Z.
    pocket_z_lo = -0.5
    pocket_z_hi = POCKET_Z + 0.5
    pocket = creation.box(extents=[
        POCKET_X, POCKET_Y, pocket_z_hi - pocket_z_lo])
    pocket.apply_translation([0.0, 0.0, (pocket_z_lo + pocket_z_hi) / 2.0])
    cutters.append(pocket)

    # Horn clearance hole through +Y BACK wall.
    # Axis along Y, centred at (X=+HORN_AXIS_OFFSET, Z=POCKET_Z/2).
    horn_hole = y_cylinder(
        HORN_CLEAR_DIA / 2.0,
        WALL_THK_Y + 4.0,
        [HORN_AXIS_OFFSET,
         POCKET_Y / 2.0 + WALL_THK_Y / 2.0,
         POCKET_Z / 2.0])
    cutters.append(horn_hole)

    # Open the lower half of the horn hole down to the open face (Z=0),
    # turning it into a U-slot: semicircle on the roof side, straight
    # sides running out the open end, so the cap slides down over a
    # servo with its horn already attached.
    horn_slot_z_lo = -1.0
    horn_slot_z_hi = POCKET_Z / 2.0
    horn_slot = creation.box(extents=[
        HORN_CLEAR_DIA,
        WALL_THK_Y + 4.0,
        horn_slot_z_hi - horn_slot_z_lo])
    horn_slot.apply_translation([
        HORN_AXIS_OFFSET,
        POCKET_Y / 2.0 + WALL_THK_Y / 2.0,
        (horn_slot_z_lo + horn_slot_z_hi) / 2.0])
    cutters.append(horn_slot)

    # SQUARED-OFF OPEN BACK through -Y FRONT wall.
    # Rectangular cut, centred horizontally on the wall (X=0), bottom
    # edge flush with the cap's open face (Z=0), top edge at Z=25 --
    # stays clear of the cap roof so there's NO notch in the top wall.
    back_open = creation.box(extents=[
        BACK_OPEN_W,
        WALL_THK_Y + 4.0,         # over-cut through wall on both sides
        BACK_OPEN_H])
    back_open.apply_translation([
        0.0,
        -(POCKET_Y / 2.0 + WALL_THK_Y / 2.0),
        BACK_OPEN_H / 2.0])
    cutters.append(back_open)

    # 4 vertical bolt passages through the ±X wall material, full cap
    # height (Z=0 to Z=CAP_Z).  Bolts enter from the closed TOP, exit at
    # the OPEN FACE, continue through the pan, secured with M3 nuts on
    # the pan's underside.
    for sx in (-SCREW_OFFSET_X, +SCREW_OFFSET_X):
        for sy in (-SCREW_OFFSET_Y, +SCREW_OFFSET_Y):
            cutters.append(z_cylinder(
                MOUNT_HOLE_DIA / 2.0, CAP_Z + 2.0,
                [sx, sy, CAP_Z / 2.0]))

    cap = trimesh.boolean.difference([body] + cutters)
    cap.merge_vertices()
    cap.fix_normals()
    return cap


# =============================================================
# PREVIEW
# =============================================================
def render_png(cap, path):
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

    col = "#8595a6"
    b = cap.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.05

    panels = [
        ("3/4 view  (open back on left, horn on right)", 22, -55),
        ("looking at +Y BACK wall  (Φ22 horn hole)",      0,  90),
        ("looking at -Y FRONT wall (rect OPEN BACK)",     0, -90),
        ("looking UP at open bottom (mating face on pan)", -85, -90),
    ]
    fig = plt.figure(figsize=(15, 11), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            cap.triangles, facecolors=shaded(cap, col),
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
        f"CRABORA femur cap  v4  (lying on side, flat-pan mount, rect open back)  "
        f"-  cap {CAP_X:.0f}x{CAP_Y:.0f}x{CAP_Z:.0f} mm",
        color="#d8e2ec", fontsize=13, y=0.97)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building femur cap v4 (lying on side, flat-pan mount, rect open back)...")
    cap = build_cap()
    e = cap.extents
    vol = cap.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {cap.is_watertight}")
    print(f"  pocket    : {POCKET_X:.1f} x {POCKET_Y:.1f} x {POCKET_Z:.1f} mm")
    print(f"  cap       : {CAP_X:.1f} x {CAP_Y:.1f} x {CAP_Z:.1f} mm  "
          f"(shoebox, no flange)")
    print(f"  walls     : ±X = {WALL_THK_X:.1f} mm (host bolt passages),  "
          f"±Y = {WALL_THK_Y:.1f} mm,  top = {TOP_THK:.1f} mm")
    print(f"  horn slot : Φ{HORN_CLEAR_DIA:.1f} U-slot thru +Y BACK wall at X=+{HORN_AXIS_OFFSET:.1f}, Z={POCKET_Z/2:.1f}  "
          f"(round on roof side, open down to open face)")
    print(f"  open back : {BACK_OPEN_W:.0f} W x {BACK_OPEN_H:.0f} H rectangle thru -Y FRONT wall  "
          f"(bottom flush with open face; top stops below cap roof -- no top notch)")
    print(f"  4x M3 bolt passages at +/-{SCREW_OFFSET_X:.1f} X, "
          f"+/-{SCREW_OFFSET_Y:.1f} Y  (vertical, full cap height)")
    print(f"  hardware  : 4x M3 x 40 mm bolts + M3 nuts on pan underside")
    print(f"  derived femur joint axis (cap frame): "
          f"X=+{HORN_AXIS_OFFSET:.1f}, Y=+{POCKET_Y/2 + WALL_THK_Y/2:.1f}, "
          f"Z=+{POCKET_Z/2:.1f}")

    cap.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(cap, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
