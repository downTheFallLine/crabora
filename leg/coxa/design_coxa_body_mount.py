"""
CRABORA coxa body mount -- 2x4 mockup, v3 (CRADLE + FLANGE, no body screws)
============================================================================

A 3D cradle that captures the STS3215 coxa servo by enclosing it,
instead of bolting to it.  No screws into the servo body anywhere.
The servo drops into a snug pocket from the top, settles on the
cradle floor, and the horn output pokes down through a hole in the
floor into open air below.

v3 changes from v2:
  - Walls reduced from ~16/12 mm to 3 mm (only as thick as needed
    to hold the servo)
  - Cradle separated from the wood-screw plate: the cradle is now
    a small box that rises out of a thin FLANGE.  Wood screws go
    through the flange, not the cradle wall.
  - Result is much lighter (less plastic) and the screw heads stay
    well clear of the servo body.

Architecture (all dims mm):
  - Cradle outer ≈ 52.2 x 31.7 x 41   (servo + 0.5 gap + 3 wall)
  - Pocket       46.2 x 25.7 x 35     (servo + 0.5 gap)
  - Floor 5 mm thick under the pocket
  - Flange     ≈ 78  x 55  x 5        (cradle sits on top of it)
  - 4 wood-screw clearance holes through the FLANGE corners only
  - Horn clearance hole (22 mm dia) through the floor, OFFSET from
    the cradle centre by HORN_AXIS_OFFSET (12.5 mm in +X)

Servo retention:
  Friction + gravity. Pocket has a 0.5 mm fit gap on each side.
  The servo settles to the bottom of the pocket; its front face
  (the face with the horn) rests on the cradle floor, with the
  horn poking through the floor hole.

  If the leg ever pulls the servo *up* out of the cradle: zip-tie
  over the top, or print a separate cap plate.

STS3215 dimensions confirmed from Feetech datasheet (45.2 x 24.7 x 35).
Horn axis offset taken from the supplied STEP file analysis.

Usage:
  python design_coxa_body_mount.py        (from /Users/tom/crabora/linkage_leg/)
"""

import os

import numpy as np
import trimesh
from trimesh import creation, transformations as tf


# =============================================================
# PARAMETERS  (millimetres)
# =============================================================
# STS3215 body
SERVO_L                  = 45.2
SERVO_W                  = 24.7
SERVO_H                  = 35.0
HORN_AXIS_OFFSET         = 12.5    # horn axis is offset from body centre along L

# Pocket fit
POCKET_FIT_GAP           = 0.5     # per side; pocket is SERVO + 2*GAP

# Cradle floor + walls
FLOOR_THK                = 5.0     # bottom of the cradle (= what the servo sits on)
WALL_THK                 = 3.0     # cradle wall thickness around the pocket

# Derived cradle outer footprint
CRADLE_W = SERVO_L + 2 * POCKET_FIT_GAP + 2 * WALL_THK   # X ≈ 52.2
CRADLE_H = SERVO_W + 2 * POCKET_FIT_GAP + 2 * WALL_THK   # Y ≈ 31.7
CRADLE_Z = FLOOR_THK + SERVO_H + 1.0                     # ≈ 41

# Flange (the wood-screw plate that fans out from under the cradle)
FLANGE_OVERHANG_X        = 13.0    # extra plate past cradle, each side X
FLANGE_OVERHANG_Y        = 12.0    # extra plate past cradle, each side Y
FLANGE_THK               = 5.0     # = FLOOR_THK; flange and floor are co-planar
FLANGE_W = CRADLE_W + 2 * FLANGE_OVERHANG_X              # ≈ 78
FLANGE_H = CRADLE_H + 2 * FLANGE_OVERHANG_Y              # ≈ 55

# Horn clearance hole (in the cradle floor)
HORN_CLEAR_DIA           = 22.0    # 20 mm horn + 2 mm slack

# Cable exit hole through one of the narrow end walls.  Closed
# rectangular hole at the BOTTOM of the wall (level with the pocket
# floor), so the servo's cable port -- which sits against the floor
# when the servo is face-down -- has a clear path out.  Horn axis is
# offset +X, so the servo's cable port faces -X --> hole in -X wall.
CABLE_NOTCH_W            = 9.0     # Y span (width)
CABLE_NOTCH_H            = 6.0     # Z span (height)
CABLE_NOTCH_SIDE         = -1      # -1 for -X wall, +1 for +X wall

# Wood-screw holes through the FLANGE corners into the 2x4 below.
# Centred in the flange overhang so heads stay clear of the cradle wall.
WOOD_SCREW_DIA           = 4.5     # ~#8 / 4 mm wood screw clearance
WOOD_SCREW_OFFSET_X = CRADLE_W / 2.0 + FLANGE_OVERHANG_X / 2.0   # ≈ 32.6
WOOD_SCREW_OFFSET_Y = CRADLE_H / 2.0 + FLANGE_OVERHANG_Y / 2.0   # ≈ 21.85

# Output
OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/coxa_body_mount.stl"
PNG_PATH = OUT_DIR + "/coxa_body_mount.png"


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
def build_mount():
    # ---------------------------------------------------------------
    # POSITIVE solids: flange (bottom slab) + cradle (rises from it)
    # ---------------------------------------------------------------
    # Flange: thin horizontal plate, wood screws go through this.
    # Bottom at Z=0, top at Z=FLANGE_THK.
    flange = creation.box(extents=[FLANGE_W, FLANGE_H, FLANGE_THK])
    flange.apply_translation([0.0, 0.0, FLANGE_THK / 2.0])

    # Cradle: small box that hosts the servo pocket. Shares its
    # bottom 5 mm with the flange (those merge into one solid).
    cradle = creation.box(extents=[CRADLE_W, CRADLE_H, CRADLE_Z])
    cradle.apply_translation([0.0, 0.0, CRADLE_Z / 2.0])

    body = trimesh.boolean.union([flange, cradle])

    # ---------------------------------------------------------------
    # NEGATIVE cutters: pocket, horn hole, wood-screw holes
    # ---------------------------------------------------------------
    cutters = []

    # Servo pocket -- sunk into the top of the cradle. Open at +Z (top),
    # closed at -Z (floor). Centred in X-Y at the cradle origin
    # (NOT the horn axis -- horn is offset from servo body centre).
    pocket_x = SERVO_L + 2 * POCKET_FIT_GAP   # 46.2
    pocket_y = SERVO_W + 2 * POCKET_FIT_GAP   # 25.7
    pocket_z = SERVO_H + 1.0                  # depth, +1 mm slack
    pocket = creation.box(extents=[pocket_x, pocket_y, pocket_z])
    pocket_center_z = CRADLE_Z - pocket_z / 2.0 + 0.5  # +0.5 over-cut
    pocket.apply_translation([0.0, 0.0, pocket_center_z])
    cutters.append(pocket)

    # Horn clearance hole in the FLOOR -- offset from cradle centre by
    # HORN_AXIS_OFFSET in +X (matching the servo's actual horn position).
    horn_hole = z_cylinder(
        HORN_CLEAR_DIA / 2.0, FLOOR_THK + 2.0,
        [HORN_AXIS_OFFSET, 0.0, FLOOR_THK / 2.0])
    cutters.append(horn_hole)

    # Cable exit hole -- closed rectangular port through one narrow
    # end wall, sitting at the BOTTOM of the wall flush with the
    # pocket floor.  Cuts all the way through (pocket <-> outside).
    notch_x_center = CABLE_NOTCH_SIDE * (CRADLE_W / 2.0 - WALL_THK / 2.0)
    notch = creation.box(extents=[
        WALL_THK + 3.0,           # X: spans the wall + over-cut both sides
        CABLE_NOTCH_W,            # Y: 9 mm wide
        CABLE_NOTCH_H])           # Z: 6 mm tall (closed top and bottom)
    notch.apply_translation([
        notch_x_center,
        0.0,
        FLOOR_THK + CABLE_NOTCH_H / 2.0])   # bottom edge at the pocket floor
    cutters.append(notch)

    # Wood-screw clearance holes through the FLANGE corners.
    # Centred in the overhang zone so heads stay clear of cradle walls.
    for wx in (-WOOD_SCREW_OFFSET_X, +WOOD_SCREW_OFFSET_X):
        for wy in (-WOOD_SCREW_OFFSET_Y, +WOOD_SCREW_OFFSET_Y):
            cutters.append(z_cylinder(
                WOOD_SCREW_DIA / 2.0, FLANGE_THK + 2.0,
                [wx, wy, FLANGE_THK / 2.0]))

    mount = trimesh.boolean.difference([body] + cutters)
    mount.merge_vertices()
    mount.fix_normals()
    return mount


# =============================================================
# PREVIEW
# =============================================================
def render_png(mount, path):
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

    col = "#7a8295"
    b = mount.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.05

    panels = [
        ("3/4 view  (top open, servo drops in)", 22, -55),
        ("looking INTO the pocket (down +Z)",    85, -90),
        ("looking at the FLOOR from below (-Z)",-85, -90),
    ]
    fig = plt.figure(figsize=(15, 6), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            mount.triangles, facecolors=shaded(mount, col),
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
        f"CRABORA coxa body mount  v3 (cradle + flange, no body screws)  "
        f"-  flange {FLANGE_W:.0f}x{FLANGE_H:.0f} mm, cradle "
        f"{CRADLE_W:.0f}x{CRADLE_H:.0f}x{CRADLE_Z:.0f} mm",
        color="#d8e2ec", fontsize=13, y=1.02)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building coxa body mount v3 (CRADLE + FLANGE, no body screws)...")
    mount = build_mount()
    e = mount.extents
    vol = mount.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {mount.is_watertight}")
    print(f"  cradle outer  : {CRADLE_W:.1f} x {CRADLE_H:.1f} x {CRADLE_Z:.0f} mm  "
          f"(wall {WALL_THK:.1f} mm)")
    print(f"  servo pocket  : {SERVO_L + 2*POCKET_FIT_GAP:.1f} x "
          f"{SERVO_W + 2*POCKET_FIT_GAP:.1f} x {SERVO_H:.0f} mm")
    print(f"  flange        : {FLANGE_W:.1f} x {FLANGE_H:.1f} x {FLANGE_THK:.0f} mm")
    print(f"  wood screws at: +/-{WOOD_SCREW_OFFSET_X:.1f} X, "
          f"+/-{WOOD_SCREW_OFFSET_Y:.1f} Y  (dia {WOOD_SCREW_DIA:.1f} mm)")
    print(f"  horn hole offset from cradle centre: +{HORN_AXIS_OFFSET:.1f} mm in X")
    print(f"  cable hole    : {CABLE_NOTCH_W:.0f} W x {CABLE_NOTCH_H:.0f} H mm "
          f"thru {'+X' if CABLE_NOTCH_SIDE > 0 else '-X'} wall, bottom at floor")

    mount.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(mount, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
