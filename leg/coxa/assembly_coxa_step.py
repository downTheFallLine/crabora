"""
CRABORA leg subsystem preview -- through the femur cap
======================================================

Stack (BUILD orientation, 2x4 flat on bench, building UP):
  1. 2x4 wood beam (body proxy)
  2. STS3215 #1 (coxa) lying flat on 2x4, horn-face UP
  3. Printed coxa CAP over the coxa servo, wood-screwed to 2x4
  4. Coxa pan plate (lollipop) bolted to coxa horn from above
  5. STS3215 #2 (femur) STANDING on the pan plate's paddle, horn
     facing OUTWARD (away from the coxa axis, tangentially)
  6. Printed FEMUR CAP over the femur servo, M3-bolted down through
     the paddle

Operating orientation: flip the whole thing 180 deg.  Body up, leg
hangs down.  Coxa rotates leg fore-aft about the vertical axis.
Femur servo's horn-axis is horizontal-tangent and will swing the
femur link up/down in the leg's vertical plane (femur link to be
designed next).

Usage:
  python assembly_coxa_step.py
"""

import os
import numpy as np
import trimesh
from trimesh import creation, transformations as tf
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import to_rgb


# -------------------------------------------------------------------
# Nominal dims (mm)
# -------------------------------------------------------------------
SERVO_L  = 45.2
SERVO_W  = 24.7
SERVO_H  = 35.0
HORN_AXIS_OFFSET = 12.5
HORN_DIA = 19.95
HORN_THK = 3.1

# Coxa cap details (must match design_coxa_cap.py)
COXA_CAP_FLANGE_TOP_Z = 5.0
COXA_CAP_TOP_DECK_Z   = 38.0    # top of cap, where pan plate sits
COXA_CAP_SCREW_OFFSET_X = 21.1
COXA_CAP_SCREW_OFFSET_Y = 22.85

# Pan plate (must match design_coxa_pan.py)
PAN_DISC_THK = 3.0
PAN_CAP_CENTER_X = 46.0          # where femur cap centres on paddle
PAN_CAP_SCREW_OFFSET_X = 22.85
PAN_CAP_SCREW_OFFSET_Y = 14.0

# Femur cap details (must match design_femur_cap.py)
FEM_CAP_FLANGE_THK = 5.0
FEM_CAP_HORN_AXIS_Z = 35.1       # above the paddle top

# Wood-screw visual
WOOD_SCREW_HEAD_DIA = 7.0
WOOD_SCREW_HEAD_THK = 2.0
WOOD_SCREW_SHAFT_DIA = 4.0
WOOD_SCREW_SHAFT_LEN = 28.0

# M3 bolt visual (cap-to-paddle)
M3_HEAD_DIA = 5.5
M3_HEAD_THK = 2.5
M3_SHAFT_DIA = 3.0
M3_SHAFT_LEN = 12.0

# Horn bolt visual (4 x M3, pan plate to coxa horn)
HORN_BOLT_LEN = 8.0

# 2x4 nominal
WOOD_LENGTH = 240.0
WOOD_WIDTH  =  89.0
WOOD_THICK  =  38.0

OUT_DIR  = "linkage_leg"
PNG_PATH = OUT_DIR + "/assembly_coxa_step.png"


# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------
def make_box(extents, center):
    b = creation.box(extents=extents)
    b.apply_translation(center)
    return b


def z_cylinder(radius, height, center, sections=32):
    c = creation.cylinder(radius=radius, height=height, sections=sections)
    c.apply_translation(center)
    return c


def y_cylinder(radius, height, center, sections=32):
    c = creation.cylinder(radius=radius, height=height, sections=sections)
    c.apply_transform(tf.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation(center)
    return c


def build():
    parts = []

    # ---- 2x4 wood beam ----
    wood = make_box(
        [WOOD_WIDTH, WOOD_LENGTH, WOOD_THICK],
        [0.0, 0.0, -WOOD_THICK / 2.0])
    parts.append(("2x4 body proxy", wood, "#a07a4e"))

    # ---- Coxa STS3215, lying flat on the 2x4, horn UP ----
    # Servo body centred at world origin in X-Y, footprint on Z=0.
    coxa_servo = make_box(
        [SERVO_L, SERVO_W, SERVO_H],
        [0.0, 0.0, SERVO_H / 2.0])
    parts.append(("coxa STS3215", coxa_servo, "#3a3f48"))

    # Coxa horn (disc above the servo's top face), offset +X
    coxa_horn = z_cylinder(
        HORN_DIA / 2.0, HORN_THK,
        [HORN_AXIS_OFFSET, 0.0, SERVO_H + HORN_THK / 2.0])
    parts.append(("coxa horn", coxa_horn, "#c9b27a"))

    # ---- Coxa cap (STL) ----
    coxa_cap = trimesh.load(os.path.join(OUT_DIR, "coxa_cap.stl"))
    parts.append(("coxa cap", coxa_cap, "#7a8295"))

    # ---- 4 wood screws ----
    for sx in (-COXA_CAP_SCREW_OFFSET_X, +COXA_CAP_SCREW_OFFSET_X):
        for sy in (-COXA_CAP_SCREW_OFFSET_Y, +COXA_CAP_SCREW_OFFSET_Y):
            head = z_cylinder(
                WOOD_SCREW_HEAD_DIA / 2.0, WOOD_SCREW_HEAD_THK,
                [sx, sy, COXA_CAP_FLANGE_TOP_Z + WOOD_SCREW_HEAD_THK / 2.0])
            shaft = z_cylinder(
                WOOD_SCREW_SHAFT_DIA / 2.0, WOOD_SCREW_SHAFT_LEN,
                [sx, sy, COXA_CAP_FLANGE_TOP_Z - WOOD_SCREW_SHAFT_LEN / 2.0])
            parts.append(("wood screw", head,  "#4a4f58"))
            parts.append(("wood screw", shaft, "#4a4f58"))

    # ---- Coxa-pan plate (STL) ----
    # Plate's CAD origin = horn axis on its bottom face.  Translate so
    # its origin lands at the coxa horn axis on top of the cap's deck.
    pan = trimesh.load(os.path.join(OUT_DIR, "coxa_pan.stl"))
    pan.apply_translation([HORN_AXIS_OFFSET, 0.0, COXA_CAP_TOP_DECK_Z])
    parts.append(("coxa pan plate", pan, "#cf8b6e"))

    # ---- 4 M3 horn bolts (pan plate to coxa horn, heads on top) ----
    pan_top_z = COXA_CAP_TOP_DECK_Z + PAN_DISC_THK
    for dx, dy in [(+5, +5), (+5, -5), (-5, +5), (-5, -5)]:
        head = z_cylinder(
            M3_HEAD_DIA / 2.0, M3_HEAD_THK,
            [HORN_AXIS_OFFSET + dx, dy, pan_top_z + M3_HEAD_THK / 2.0])
        shaft = z_cylinder(
            M3_SHAFT_DIA / 2.0, HORN_BOLT_LEN,
            [HORN_AXIS_OFFSET + dx, dy,
             pan_top_z - HORN_BOLT_LEN / 2.0])
        parts.append(("M3 horn bolt", head,  "#d8d0c0"))
        parts.append(("M3 horn bolt", shaft, "#d8d0c0"))

    # ---- Femur STS3215, STANDING on the paddle ----
    # Paddle top at pan_top_z.  Femur servo standing with L vertical,
    # centred at pan plate X = PAN_CAP_CENTER_X (in pan plate frame),
    # which is world X = HORN_AXIS_OFFSET + PAN_CAP_CENTER_X.
    # Footprint: W (X) x H (Y) = 24.7 x 35.0.
    femur_x_center = HORN_AXIS_OFFSET + PAN_CAP_CENTER_X
    femur_y_center = 0.0
    femur_z_bottom = pan_top_z
    femur_servo = make_box(
        [SERVO_W, SERVO_H, SERVO_L],
        [femur_x_center, femur_y_center, femur_z_bottom + SERVO_L / 2.0])
    parts.append(("femur STS3215", femur_servo, "#3a3f48"))

    # ---- Femur horn (on +Y face of the standing femur servo) ----
    # Horn axis horizontal +Y, at height FEM_CAP_HORN_AXIS_Z above paddle.
    horn_face_y = femur_y_center + SERVO_H / 2.0   # +Y face of servo
    femur_horn = y_cylinder(
        HORN_DIA / 2.0, HORN_THK,
        [femur_x_center, horn_face_y + HORN_THK / 2.0,
         femur_z_bottom + FEM_CAP_HORN_AXIS_Z])
    parts.append(("femur horn", femur_horn, "#c9b27a"))

    # ---- Femur cap (STL) ----
    fcap = trimesh.load(os.path.join(OUT_DIR, "femur_cap.stl"))
    fcap.apply_translation([femur_x_center, femur_y_center, femur_z_bottom])
    parts.append(("femur cap", fcap, "#8595a6"))

    # ---- 4 M3 mount bolts (femur cap to paddle, heads on top of cap flange) ----
    fem_flange_top_z = femur_z_bottom + FEM_CAP_FLANGE_THK
    for dx in (-PAN_CAP_SCREW_OFFSET_X, +PAN_CAP_SCREW_OFFSET_X):
        for dy in (-PAN_CAP_SCREW_OFFSET_Y, +PAN_CAP_SCREW_OFFSET_Y):
            head = z_cylinder(
                M3_HEAD_DIA / 2.0, M3_HEAD_THK,
                [femur_x_center + dx, femur_y_center + dy,
                 fem_flange_top_z + M3_HEAD_THK / 2.0])
            # Shaft goes DOWN through the cap flange AND through the paddle.
            shaft_len = FEM_CAP_FLANGE_THK + PAN_DISC_THK + 2.0
            shaft = z_cylinder(
                M3_SHAFT_DIA / 2.0, shaft_len,
                [femur_x_center + dx, femur_y_center + dy,
                 fem_flange_top_z - shaft_len / 2.0])
            parts.append(("M3 cap bolt", head,  "#d8d0c0"))
            parts.append(("M3 cap bolt", shaft, "#d8d0c0"))

    return parts


def render(parts, path):
    bg = "#0c0d11"; fg = "#9fb3c8"
    key  = np.array([0.35, 0.45, 0.82]); key  = key  / np.linalg.norm(key)
    fill = np.array([-0.55, -0.25, 0.30]); fill = fill / np.linalg.norm(fill)

    def shaded(mesh, base):
        n = mesh.face_normals
        lit = (0.24 + 0.62 * np.clip(n @ key,  0.0, 1.0)
                    + 0.22 * np.clip(n @ fill, 0.0, 1.0))
        col = np.array(to_rgb(base))
        return np.clip(col[None, :] * np.clip(lit, 0.0, 1.15)[:, None], 0.0, 1.0)

    # Frame around the leg sub-assembly (exclude most of the 2x4)
    focus_parts = [m for name, m, _ in parts if name != "2x4 body proxy"]
    corners = np.vstack([m.bounds for m in focus_parts])
    ctr  = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    ctr[2] -= 5.0  # show some wood below
    half = (corners.max(axis=0) - corners.min(axis=0)).max() / 2.0 * 1.2

    panels = [
        ("3/4 view  -- BUILD orientation, leg builds UP", 18, -60),
        ("side view  -- looking along Y (cable side LEFT)", 5, -90),
        ("front view -- looking from +Y (femur horn out toward camera)", 5, 0),
    ]

    fig = plt.figure(figsize=(17, 9), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.set_facecolor(bg)
        for _, m, c in parts:
            ax.add_collection3d(Poly3DCollection(
                m.triangles, facecolors=shaded(m, c), edgecolors="none"))
        ax.set_xlim(ctr[0] - half, ctr[0] + half)
        ax.set_ylim(ctr[1] - half, ctr[1] + half)
        ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color=fg, fontsize=10)

    from matplotlib.patches import Patch
    seen = set(); handles = []
    for label, _, color in parts:
        if label in seen: continue
        seen.add(label)
        handles.append(Patch(facecolor=color, edgecolor="white", label=label))
    fig.legend(handles=handles, loc="lower center", ncol=6,
               facecolor=bg, edgecolor=fg, labelcolor=fg, fontsize=9,
               bbox_to_anchor=(0.5, 0.02))

    fig.suptitle("Leg subassembly  -- coxa + pan + femur cap stack",
                 color="#d8e2ec", fontsize=13, y=0.97)
    fig.tight_layout(rect=[0, 0.07, 1, 0.95])
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    parts = build()
    print(f"Rendered {len(parts)} part instances...")
    render(parts, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
