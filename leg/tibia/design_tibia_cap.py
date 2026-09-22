"""
CRABORA tibia cap -- holds the tibia STS3215 at the knee, bolts to femur link
==============================================================================

Different geometry from the femur cap: here the horn axis is
PERPENDICULAR to the mounting flange (not parallel to it).  That's
because:

  - The femur link is a flat plate that rotates in the leg's vertical
    plane (the radial-vertical plane, with surface normal along the
    body's tangent direction).
  - The tibia joint axis must also be along the tangent direction so
    the tibia rotates in the same vertical plane as the femur.
  - When the cap mounts flat on the femur link, the cap's mounting
    flange is in the link's plane.  The horn axis -- perpendicular to
    that flange -- naturally points along the tangent direction.  ✓

Geometry (CAD frame: Z = horn axis direction, open face at Z=0):
  - Pure shoebox shape, one open face -- no flange wings.  v1 had wide
    14 mm flanges on +/-Y at the open face hosting 4 short M3 bolts;
    v2 eliminates the flanges and replaces them with 4 LONG bolts that
    pass vertically through the cap (open face to back face), tying the
    cap to the femur link's knee plate above with M3 nuts.
  - Pocket holds tibia STS3215 with H axis = horn axis = cap Z
  - Walls: 3 mm on +/-X; THICKENED 7 mm on +/-Y to host the vertical
    bolt passages.  2.5 mm on +Z (back wall, with horn hole + bolt
    head clearance).
  - Open at -Z (the mating face -- this is what mates with the femur
    link surface)
  - Horn hole on +Z back wall, OFFSET +12.5 mm in X (so horn axis is
    OFFSET from cap centre -- matches STS3215 horn position)
  - Cable port: 13 W x 30 H mm on -X side wall, bottom at open face

Build sequence:
  1. Place tibia servo flat on the femur link's knee plate, with its
     horn face UP (away from the link) and horn axis offset toward +X.
  2. Lower the tibia cap DOWN OVER the servo, open face going down.
  3. Cap's open face comes to rest on the knee plate (no flange wings
     in v2; the cap footprint and the plate footprint are nearly the
     same size).
  4. Drop 4 long M3 bolts (~50 mm) through the cap's back wall, down
     through the +/-Y wall passages, through the plate, and secure with
     M3 nuts on the plate's underside.  Bolt heads on top (cap back
     face), nuts on bottom (plate bottom face).
  5. Horn pokes ~0.6 mm proud of the cap's +Z back wall, ready for
     the tibia link.

Servo orientation inside the cap:
  - Servo L (45.2) = cap X
  - Servo W (24.7) = cap Y
  - Servo H (35.0) = cap Z (= horn axis direction)

The horn axis is then perpendicular to the femur link surface, which
in operating orientation is along world Y (tangent to body).  Tibia
link rotates in the leg's vertical plane.  ✓

Usage:
  python design_tibia_cap.py
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

# Pocket fit
POCKET_FIT_GAP           = 0.5
POCKET_X = SERVO_L + 2 * POCKET_FIT_GAP   # 46.2  (cap X = servo L)
POCKET_Y = SERVO_W + 2 * POCKET_FIT_GAP   # 25.7  (cap Y = servo W)
POCKET_Z = SERVO_H + 0.5                  # 35.5  (cap Z = servo H, horn axis)

# Cap structure
WALL_THK_X               = 3.0     # walls on ±X (thin -- no bolts pass through)
WALL_THK_Y               = 7.0     # walls on ±Y (thickened to host vertical bolt
                                   # passages from open face to back face).
                                   # With a Φ3.4 bolt centred in the wall this
                                   # leaves ~1.8 mm of wall material on each side
                                   # of the bolt -- printable in PLA with 4-5
                                   # perimeters.
BACK_THK                 = 2.5     # +Z back wall (thinner so horn pokes proud)

# Derived cap outer dimensions
CAP_X = POCKET_X + 2 * WALL_THK_X                       # 52.2
CAP_Y_CORE = POCKET_Y + 2 * WALL_THK_Y                  # 39.7
CAP_Y_OUTER = CAP_Y_CORE                                # 39.7 (no flange)
CAP_Z = POCKET_Z + BACK_THK                             # 38.0

# Horn clearance hole in +Z back wall
HORN_CLEAR_DIA           = 22.0    # 20 mm horn + 2 mm slack

# Cable exit port through -X side wall, at the open face (Z=0)
CABLE_PORT_W             = 13.0    # Y span (wide enough for the connector body)
CABLE_PORT_H             = 30.0    # Z span (bottom flush with open face)

# Vertical bolt passages through the ±Y walls of the cap, full Z height
# (open face -> back face).  Long M3 bolts thread through these into
# nuts on the underside of the femur link's knee plate.
MOUNT_HOLE_DIA           = 3.4     # M3 clearance
SCREW_OFFSET_X           = 15.0    # ±X from cap centre (within the ±Y wall span)
SCREW_OFFSET_Y           = (POCKET_Y + WALL_THK_Y) / 2.0   # 16.35, centred in the
                                                            # ±Y wall material
                                                            # (between pocket edge
                                                            # and cap outer face)

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/tibia_cap.stl"
PNG_PATH = OUT_DIR + "/tibia_cap.png"


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
def build_cap():
    """CAD frame: origin at cap's centre on the open face (Z=0).
    Horn axis = cap Z direction.  +Z is the closed back wall direction."""

    # ---------------------------------------------------------------
    # POSITIVE solid: just the central core (no flange in v2).
    # ---------------------------------------------------------------
    body = creation.box(extents=[CAP_X, CAP_Y_CORE, CAP_Z])
    body.apply_translation([0.0, 0.0, CAP_Z / 2.0])

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # Servo pocket: open at Z=0 (bottom), closed at Z=POCKET_Z.
    # Over-cut both ends for clean edges.
    pocket_z_lo = -0.5
    pocket_z_hi = POCKET_Z + 0.5
    pocket = creation.box(extents=[
        POCKET_X, POCKET_Y, pocket_z_hi - pocket_z_lo])
    pocket.apply_translation([0.0, 0.0, (pocket_z_lo + pocket_z_hi) / 2.0])
    cutters.append(pocket)

    # Horn clearance hole through +Z back wall, offset +X by HORN_AXIS_OFFSET
    cutters.append(z_cylinder(
        HORN_CLEAR_DIA / 2.0,
        BACK_THK + 2.0,
        [HORN_AXIS_OFFSET, 0.0, POCKET_Z + BACK_THK / 2.0]))

    # Cable exit port through -X side wall, at the open face level
    port_x_center = -CAP_X / 2.0 + WALL_THK_X / 2.0
    port = creation.box(extents=[
        WALL_THK_X + 3.0,         # X: spans the wall + over-cut both sides
        CABLE_PORT_W,             # Y: connector clearance
        CABLE_PORT_H])            # Z: tall enough for the connector head
    port.apply_translation([
        port_x_center, 0.0, CABLE_PORT_H / 2.0])
    cutters.append(port)

    # 4 vertical bolt passages through the ±Y wall material, full cap
    # height (Z=0 to Z=CAP_Z).  Bolts enter from the back face (top),
    # exit at the open face (bottom), continue through the plate, and
    # are secured with M3 nuts on the plate's underside.
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

    col = "#9aa8b6"
    b = cap.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.05

    panels = [
        ("3/4 view  (horn pokes UP through +Z back wall)", 22, -55),
        ("looking DOWN at back wall (horn hole + flange holes)", 85, -90),
        ("looking UP at open mating face (servo enters here)", -85, -90),
        ("side view -- cable port LEFT (-X wall)",         5, -90),
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
        f"CRABORA tibia cap  v2  (flangeless, long-bolt through-cap)  "
        f"-  cap {CAP_X:.0f}x{CAP_Y_OUTER:.0f}x{CAP_Z:.0f} mm",
        color="#d8e2ec", fontsize=13, y=0.97)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building tibia cap v2 (flangeless, long-bolt through-cap)...")
    cap = build_cap()
    e = cap.extents
    vol = cap.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {cap.is_watertight}")
    print(f"  pocket    : {POCKET_X:.1f} x {POCKET_Y:.1f} x {POCKET_Z:.1f} mm")
    print(f"  cap       : {CAP_X:.1f} x {CAP_Y_CORE:.1f} x {CAP_Z:.1f} mm  "
          f"(shoebox, no flange)")
    print(f"  walls     : ±X = {WALL_THK_X:.1f} mm,  "
          f"±Y = {WALL_THK_Y:.1f} mm (host bolt passages)")
    print(f"  horn hole : Φ{HORN_CLEAR_DIA:.1f} thru +Z back wall at X=+{HORN_AXIS_OFFSET:.1f}")
    print(f"  cable port: {CABLE_PORT_W:.0f} W x {CABLE_PORT_H:.0f} H mm thru -X wall, "
          f"bottom at open face")
    print(f"  4x M3 bolt passages at +/-{SCREW_OFFSET_X:.1f} X, "
          f"+/-{SCREW_OFFSET_Y:.2f} Y  (vertical, full cap height)")
    print(f"  hardware  : 4x M3 x ~50 mm bolts + M3 nuts on plate underside")

    cap.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(cap, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
