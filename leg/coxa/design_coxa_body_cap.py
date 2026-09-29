"""
CRABORA coxa body CAP -- v2, flangeless, for printed body deck
================================================================

Holds the coxa STS3215 on the printed body deck.  Same role as the
v1 design_coxa_body_mount.py (which had a wide flange and wood
screws for the 2x4 bench mockup), but redesigned for a clean
attachment to the printed hex deck:

  - NO flange wings (the v1 flange was 78 x 55 mm; with 6 legs at
    60° spacing the flanges overlapped each other AND clashed with
    the deck's vent slots).
  - ±Y walls THICKENED from 3 to 5 mm to host vertical M3 bolt passages.
  - 4 long M3 bolts pass from the top of the cap walls down through
    the deck, secured with M3 nuts on the deck's underside.
  - Footprint shrinks to 52.2 x 35.7 mm so 6 caps fit at 60° spacing
    on a small body without colliding (cap-to-cap clearance ≈ 5 mm at
    LEG_RADIUS=76, which is what the v3 body deck uses).
  - Bolt size is M3 (matches tibia_cap and femur_cap convention; v1
    of this file used M4 which made the cap and the body deck unable
    to fit together on the A1 mini printer plate).

Architecture in BUILD/OPERATING orientation (body deck horizontal,
cap sits on top, servo dropped in from above with horn pointing DOWN):

    open top  ____________________
            ⌐                    ¬   <-- servo drops in here
            │  ╔══════════════╗  │
            │  ║              ║  │
            │  ║  [ servo ]   ║  │
   bolt ▾   │  ║              ║   ▾ bolt
            │  ╚══════════════╝  │
            ├────────────────────┤   <-- floor (5 mm) with Φ22 horn hole
            └────────────────────┘
            ░░░░░░░░░░░░░░░░░░░░░░  <-- BODY DECK (5 mm) with Φ22 horn hole
            ▾                    ▾   <-- bolt continues through deck
                                          M3 nut on underside

The 4 bolts (M3 x ~55 mm) thread through Φ4.5 vertical passages in
the ±Y walls of the cap, then through matching Φ4.5 holes in the
body deck.  Bolt heads sit on top of the cap walls (accessible);
nuts sit on the underside of the body deck (inside the body cavity).

CAD frame:
  - Origin at the cap's centre on the BOTTOM (mating) face, Z=0.
  - +X = servo L direction (cable port on -X wall).
  - +Y = servo W direction.
  - +Z = up (away from the deck).
  - Cap occupies Z = 0..CAP_Z (≈ 41 mm).
  - Floor (with horn hole) is at Z = 0..FLOOR_THK (≈ 5 mm).
  - Pocket (servo space) is at Z = FLOOR_THK..CAP_Z.

The horn axis passes vertically through (HORN_AXIS_OFFSET, 0, *) --
NOT through the cap centre, since the servo's horn is offset
HORN_AXIS_OFFSET mm from the servo body's centre along its L axis.

Usage:
  python design_coxa_body_cap.py
"""

import os

import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  (mm)
# =============================================================
# STS3215 body
SERVO_L                  = 45.2
SERVO_W                  = 24.7
SERVO_H                  = 35.0
HORN_AXIS_OFFSET         = 12.5    # horn axis is offset from body centre along L

# Pocket fit
POCKET_FIT_GAP           = 0.5
POCKET_X = SERVO_L + 2 * POCKET_FIT_GAP   # 46.2  (servo L direction)
POCKET_WIDTH_TRIM        = 0.8     # tighten pocket width; absorbed by thicker ±Y
                                   # walls so SCREW_OFFSET_Y (bolt position) doesn't move
POCKET_Y = SERVO_W + 2 * POCKET_FIT_GAP - POCKET_WIDTH_TRIM   # 24.9  (servo W direction)
POCKET_DEPTH_TRIM        = 1.9     # shallower pocket -- open top, so servo just sits
                                   # proud of the wall rim by this much more
POCKET_Z = SERVO_H + 1.0 - POCKET_DEPTH_TRIM   # 34.1  (servo H + 1mm slack on top)

# Cap structure
FLOOR_THK                = 5.0     # bottom of the cap (mates with deck top)
WALL_THK_X               = 3.0     # ±X walls (short walls, no bolts)
WALL_THK_Y               = 5.0 + POCKET_WIDTH_TRIM   # 5.8 -- ±Y walls THICKENED to host
                                   # vertical M3 bolt passages, plus the extra
                                   # POCKET_WIDTH_TRIM absorbed here so SCREW_OFFSET_Y
                                   # (bolt hole position) stays put while the outer
                                   # footprint (CAP_Y) grows by POCKET_WIDTH_TRIM.
                                   # With Φ3.4 centred in a 5.8 mm wall this leaves
                                   # ~1.2 mm of wall material on each side of the
                                   # bolt -- printable in PLA with 4-5 perimeters.
                                   # v2 (this version) uses M3 for consistency with
                                   # the tibia + femur caps (was M3 in v1; M3 also
                                   # shrinks the cap so 6 legs fit at 60° spacing
                                   # without overlap -- NOTE: the 0.8mm CAP_Y growth
                                   # here tightens that clearance from ~5mm to ~4.2mm).

# Derived cap outer footprint
CAP_X = POCKET_X + 2 * WALL_THK_X                       # 52.2
CAP_Y = POCKET_Y + 2 * WALL_THK_Y                       # 39.7
CAP_Z = FLOOR_THK + POCKET_Z                            # 41.0

# Horn clearance hole through the FLOOR (Z = 0..FLOOR_THK), offset
# +HORN_AXIS_OFFSET in X from the cap centre to match the servo's horn.
HORN_CLEAR_DIA           = 22.0    # 20 mm horn + 2 mm slack

# CABLE PASS-THROUGH: a slot through the floor near the -X (back) end, over
# the servo's rear cable connectors (opposite the +X horn).  The cap is a
# cover that sits ON TOP of the servo; the horn passes through the DECK, not the
# cap, so the cap has NO horn hole.  (HORN_* params above are kept only because
# the body deck imports them for ITS round horn hole + cap placement.)
CABLE_HOLE_X_SIZE        = 26.0    # cable hole span along X (16.0 + 10mm longer)
CABLE_HOLE_Y_SIZE        = POCKET_Y    # cable hole spans the full pocket width (Y)
CABLE_HOLE_X             = -(POCKET_X / 2.0 - CABLE_HOLE_X_SIZE / 2.0 - 1.5)  # near -X end

# Vertical bolt passages through the ±Y walls of the cap, full Z height.
# Long M3 bolts thread from the cap top down through these passages
# and through the body deck below.
MOUNT_HOLE_DIA           = 3.4     # M3 clearance
SCREW_OFFSET_X           = 15.0    # ±X from cap centre (within the ±Y wall span)
SCREW_OFFSET_Y           = (POCKET_Y + WALL_THK_Y) / 2.0   # 15.35, centred in the
                                                            # ±Y wall material
                                                            # (between pocket edge
                                                            # and cap outer face)

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/coxa_body_cap.stl"
PNG_PATH = OUT_DIR + "/coxa_body_cap.png"


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
    """CAD frame: origin at cap centre on BOTTOM (mating) face, Z=0.
    Cap occupies Z = 0..CAP_Z."""

    # ---------------------------------------------------------------
    # POSITIVE solid: just the central core (no flange in v2).
    # ---------------------------------------------------------------
    body = creation.box(extents=[CAP_X, CAP_Y, CAP_Z])
    body.apply_translation([0.0, 0.0, CAP_Z / 2.0])

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # Servo pocket: open at +Z (top), closed at -Z (floor at Z=FLOOR_THK).
    # Centred in X and Y at the cap origin.
    pocket = creation.box(extents=[POCKET_X, POCKET_Y, POCKET_Z + 1.0])
    pocket_center_z = FLOOR_THK + POCKET_Z / 2.0
    pocket.apply_translation([0.0, 0.0, pocket_center_z + 0.5])  # over-cut top
    cutters.append(pocket)

    # CABLE pass-through slot through the FLOOR, near the -X (back) end, over
    # the servo's rear connectors, spanning the full pocket width (Y).  No horn
    # hole -- the horn passes through the DECK, not the cap.  -X end wall is
    # solid (the old port is removed).
    cable = creation.box(extents=[CABLE_HOLE_X_SIZE, CABLE_HOLE_Y_SIZE, FLOOR_THK + 2.0])
    cable.apply_translation([CABLE_HOLE_X, 0.0, FLOOR_THK / 2.0])
    cutters.append(cable)

    # 4 vertical bolt passages through the ±Y wall material, full cap
    # height (Z=0 to Z=CAP_Z).  Bolts drop in from the top of the cap
    # walls, exit at the bottom (mating with the deck), continue through
    # the deck, and are secured with M3 nuts on the deck's underside.
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
        ("3/4 view  (servo drops in top)",          22, -55),
        ("looking DOWN at the open top",            85, -90),
        ("looking UP at the bottom (mating face)", -85, -90),
        ("side view -- cable port LEFT (-X wall)",   5, -90),
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
        f"CRABORA coxa body cap  v2  (flangeless, body-mount)  "
        f"-  cap {CAP_X:.0f}x{CAP_Y:.0f}x{CAP_Z:.0f} mm",
        color="#d8e2ec", fontsize=13, y=0.97)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building coxa body cap v2 (flangeless, for printed body deck)...")
    cap = build_cap()
    e = cap.extents
    vol = cap.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {cap.is_watertight}")
    print(f"  pocket    : {POCKET_X:.1f} x {POCKET_Y:.1f} x {POCKET_Z:.1f} mm")
    print(f"  cap       : {CAP_X:.1f} x {CAP_Y:.1f} x {CAP_Z:.1f} mm  "
          f"(shoebox, no flange)")
    print(f"  walls     : ±X = {WALL_THK_X:.1f} mm,  "
          f"±Y = {WALL_THK_Y:.1f} mm (host bolt passages)")
    print(f"  floor     : {FLOOR_THK:.1f} mm thick (no horn hole -- horn through deck)")
    print(f"  cable hole: {CABLE_HOLE_X_SIZE:.1f} x {CABLE_HOLE_Y_SIZE:.1f} mm thru floor "
          f"at X={CABLE_HOLE_X:.1f} (back/-X end), spans full pocket width, -X wall solid")
    print(f"  4x M3 bolt passages at +/-{SCREW_OFFSET_X:.1f} X, "
          f"+/-{SCREW_OFFSET_Y:.2f} Y  (vertical, full cap height)")
    print(f"  hardware  : 4x M3 x ~55 mm bolts + M3 nuts on deck underside")

    cap.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(cap, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
