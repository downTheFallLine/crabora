"""
CRABORA coxa servo CAP -- 2x4 mockup, v1 (sandwich / "the body IS the cradle")
==============================================================================

A U-shaped clamp ("cap") that fits OVER an STS3215 coxa servo lying
flat on a body plate.  The body plate becomes the bottom of the
sandwich; the cap clamps the servo from above.  No screws into the
servo body.

Inspired by typical hexapod body plates that sandwich servos between
top + bottom plates.  Here the "bottom plate" is the 2x4 wood (for the
mockup), and this printed cap is the "top plate" for ONE servo.

Architecture in BUILD orientation (2x4 flat on bench, horn UP):

    ┌───────────────────────┐    <-- TOP DECK (with horn hole at +12.5 X)
    │     ░░░░░░░O░░░░░░    │             horn pokes UP through hole
    ├──┐                  ┌──┤
    │  │                  │  │    <-- SIDE WALLS grip the servo
    │  │    [ servo ]     │  │        (long sides)
   ─┤  │                  │  ├─   <-- FLANGE WINGS at the bottom
    └──┴──────────────────┴──┘       4 wood screws through wings into 2x4
    ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒    <-- 2x4 wood surface

End walls on +X and -X close the pocket so the servo can't slide out.
The -X end wall has a cable port (9 W x 6 H mm, bottom edge flush
with the 2x4 surface) so the cable can exit through the wall.

Build sequence:
  1. Place servo on 2x4, horn-face UP (cable exit toward -X)
  2. Drop the cap down OVER the servo (open bottom slides over servo)
  3. Cap's flange wings come to rest on 2x4 surface
  4. Wood-screw the 4 corner screws through wings into 2x4
  5. Horn pokes UP through the top-deck hole, ~0.6 mm proud of the deck
  6. Bolt the coxa-pan plate down onto the horn (4 x M3 from above)
  7. Continue stacking the rest of the leg upward

Operating orientation (whole assembly flipped 180 deg):
  - 2x4 (body proxy) is on top
  - Cap hangs from underside of body, wood screws now point UP into 2x4
  - Servo hangs in cap, horn points DOWN, leg hangs below
  - Gravity presses the servo's back-face up against the body, and the
    cap holds it in place from below

Print orientation: cap as-modelled, flange-side DOWN on bed.  The top
deck spans a 25.7 mm gap between the side walls -- doable as a bridge
with standard slicer settings, no supports needed.

STS3215 dimensions: 45.2 x 24.7 x 35 (Feetech datasheet)
Horn axis offset:   +12.5 mm from servo body centre (along the L axis)
Horn disc:          Φ19.95 x 3.1 mm thick
Top-deck thickness: kept just under the horn-disc thickness so the
                    horn pokes ~0.6 mm proud -- the pan plate then sits
                    directly on the horn face, not on the cap deck.

Usage:
  python design_coxa_cap.py
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
HORN_AXIS_OFFSET         = 12.5    # +X from servo body centre
HORN_DISC_THK            = 3.1     # horn disc thickness (Feetech spec)

# Pocket fit
POCKET_FIT_GAP           = 0.5     # per side
POCKET_L = SERVO_L + 2 * POCKET_FIT_GAP   # 46.2  (X)
POCKET_W = SERVO_W + 2 * POCKET_FIT_GAP   # 25.7  (Y)
POCKET_H = SERVO_H + 0.5                  # 35.5  (Z)  -- 0.5 mm slack on top

# Cap structure
WALL_THK                 = 3.0     # side & end wall thickness
TOP_THK                  = 2.5     # top deck (< HORN_DISC_THK so horn pokes proud)
FLANGE_THK               = 5.0     # flange-wing thickness (sits on 2x4)
FLANGE_WIDTH             = 14.0    # how far each wing sticks out past side wall (Y)

# Derived cap outer footprint
CAP_L = POCKET_L + 2 * WALL_THK                                 # 52.2  (X)
CAP_W_CORE  = POCKET_W + 2 * WALL_THK                           # 31.7  (Y core)
CAP_W_OUTER = CAP_W_CORE + 2 * FLANGE_WIDTH                     # 59.7  (Y w/ wings)
CAP_H_TOTAL = POCKET_H + TOP_THK                                # 38.0  (Z)

# Horn clearance hole (in top deck)
HORN_CLEAR_DIA           = 22.0    # 20 mm horn disc + 2 mm slack

# Cable exit port -- through -X end wall, bottom edge at 2x4 level
CABLE_PORT_W             = 9.0     # Y span
CABLE_PORT_H             = 6.0     # Z span (bottom flush with 2x4)

# Wood-screw clearance holes through flange wings into 2x4 below
WOOD_SCREW_DIA           = 4.5     # ~#8 / 4 mm wood screw clearance
SCREW_OFFSET_X = CAP_L / 2.0 - 5.0                               # ≈ 21
SCREW_OFFSET_Y = (CAP_W_CORE + FLANGE_WIDTH) / 2.0               # ≈ 22.85

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/coxa_cap.stl"
PNG_PATH = OUT_DIR + "/coxa_cap.png"


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
    """Cap centred on the SERVO BODY in X-Y, sitting on Z=0 (the 2x4 surface)."""
    # ---------------------------------------------------------------
    # POSITIVE solids: central U (walls + top) + bottom flange slab
    # ---------------------------------------------------------------
    # Central core: covers Y = CAP_W_CORE (no wings), full Z height.
    core = creation.box(extents=[CAP_L, CAP_W_CORE, CAP_H_TOTAL])
    core.apply_translation([0.0, 0.0, CAP_H_TOTAL / 2.0])

    # Bottom slab: full outer Y (includes wings), only FLANGE_THK tall.
    slab = creation.box(extents=[CAP_L, CAP_W_OUTER, FLANGE_THK])
    slab.apply_translation([0.0, 0.0, FLANGE_THK / 2.0])

    body = trimesh.boolean.union([core, slab])

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # Servo pocket: open at Z=0 (bottom). Cuts up to POCKET_H, leaving
    # TOP_THK of material above as the top deck.  Over-cut +/-0.5 mm
    # for clean edges.
    pocket_z_lo = -0.5
    pocket_z_hi = POCKET_H + 0.5
    pocket = creation.box(extents=[
        POCKET_L, POCKET_W, pocket_z_hi - pocket_z_lo])
    pocket.apply_translation([0.0, 0.0, (pocket_z_lo + pocket_z_hi) / 2.0])
    cutters.append(pocket)

    # Horn clearance hole through top deck at horn axis offset +X
    horn_hole = z_cylinder(
        HORN_CLEAR_DIA / 2.0,
        TOP_THK + 2.0,
        [HORN_AXIS_OFFSET, 0.0, POCKET_H + TOP_THK / 2.0])
    cutters.append(horn_hole)

    # Cable exit port through -X end wall (bottom edge at 2x4 level)
    port_x_center = -CAP_L / 2.0 + WALL_THK / 2.0
    port = creation.box(extents=[
        WALL_THK + 3.0,           # X: spans the wall + over-cut both sides
        CABLE_PORT_W,             # Y: 9 mm wide
        CABLE_PORT_H])            # Z: 6 mm tall
    port.apply_translation([
        port_x_center, 0.0, CABLE_PORT_H / 2.0])
    cutters.append(port)

    # Wood-screw clearance holes (4) through flange wings
    for sx in (-SCREW_OFFSET_X, +SCREW_OFFSET_X):
        for sy in (-SCREW_OFFSET_Y, +SCREW_OFFSET_Y):
            cutters.append(z_cylinder(
                WOOD_SCREW_DIA / 2.0, FLANGE_THK + 2.0,
                [sx, sy, FLANGE_THK / 2.0]))

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

    col = "#7a8295"
    b = cap.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.05

    panels = [
        ("3/4 view  (BUILD orientation, horn pokes UP)", 22, -55),
        ("looking DOWN onto the top deck",               85, -90),
        ("looking UP at the open bottom (servo enters here)", -85, -90),
        ("side view -- cable end on the LEFT (-X)",       0,   0),
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
        f"CRABORA coxa servo CAP  v1  (sandwich design, no body screws)  "
        f"-  cap {CAP_L:.0f}x{CAP_W_OUTER:.0f}x{CAP_H_TOTAL:.0f} mm",
        color="#d8e2ec", fontsize=13, y=0.97)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building coxa cap v1 (sandwich, no body screws)...")
    cap = build_cap()
    e = cap.extents
    vol = cap.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {cap.is_watertight}")
    print(f"  pocket    : {POCKET_L:.1f} x {POCKET_W:.1f} x {POCKET_H:.1f} mm")
    print(f"  cap core  : {CAP_L:.1f} x {CAP_W_CORE:.1f} x {CAP_H_TOTAL:.1f} mm")
    print(f"  flange    : {CAP_L:.1f} x {CAP_W_OUTER:.1f} x {FLANGE_THK:.1f} mm")
    print(f"  top deck  : {TOP_THK:.1f} mm thick "
          f"(horn disc {HORN_DISC_THK:.1f} mm; horn pokes {HORN_DISC_THK-TOP_THK:+.1f} mm proud)")
    print(f"  horn hole : Φ{HORN_CLEAR_DIA:.1f} at X=+{HORN_AXIS_OFFSET:.1f}")
    print(f"  cable port: {CABLE_PORT_W:.0f} W x {CABLE_PORT_H:.0f} H mm thru -X wall, bottom at floor")
    print(f"  4x wood screws at +/-{SCREW_OFFSET_X:.1f} X, +/-{SCREW_OFFSET_Y:.1f} Y  "
          f"(dia {WOOD_SCREW_DIA:.1f} mm)")

    cap.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(cap, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
