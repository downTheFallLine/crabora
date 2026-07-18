"""
CRABORA femur generator
=======================

Generates the middle leg segment (femur) for CRABORA and writes:
  - femur.stl       : the main part -- body, plate, cradle body
  - femur_cover.stl : the bolt-on side cover for the cradle
  - femur.png       : a shaded multi-view preview render

The femur is the structural link between two leg joints:
  - PLATE END  (joint A): a round plate that bolts onto the femur
                          servo's output horn.
  - CRADLE END (joint B): a side-loading cradle for the tibia servo
                          (the third and last servo). The servo is set
                          into the cradle body through its open +Y
                          side; a cover panel then screws over that
                          side. A counterbore in the cover lets the
                          short output post reach out to the tibia.

Style: "smooth organic-mech". The body is a lofted chain of cross-
sections -- a soft waisted ellipse through the body that swells into a
gently bellied cradle and tapers to a rounded nose at the tip. The
body bows upward; the cradle stays straight to track the servo, and
smooth scooped pockets lighten the flanks.

Everything is parametric -- edit the PARAMETERS block and re-run.

  IMPORTANT: the STS3215 numbers below are datasheet-nominal. Before
  printing the real part, measure a servo (body + horn bolt pattern)
  with calipers, update the parameters, and re-run. The cradle
  interface in particular is a first pass and will want tuning.

Usage:
  python design_femur.py
"""

import numpy as np
import trimesh
from trimesh import creation, transformations as tf


# =============================================================
# PARAMETERS  (millimetres)
# =============================================================

# ---- STS3215 servo body (measured) ----
SERVO_L = 45.5          # body length
SERVO_W = 24.6          # body width
SERVO_H = 35.5          # body height
FIT_GAP = 0.6           # clearance around the servo in the cradle pocket

# ---- Femur layout along its length (X) ----
BODY_START  = 10.0      # where the lofted body begins (inside the plate)
BODY_LEN    = 49.0      # length of the slim body section
CRADLE_BACK = 8.0       # cradle material between the body and the pocket
CRADLE_NOSE = 11.0      # tapered cradle nose past the pocket (organic tip)

# ---- Plate end (joint A -- bolts to the femur servo horn) ----
PLATE_DIA = 33.0        # outer diameter of the round mounting plate
PLATE_THK = 7.0         # plate thickness, along the hinge axis (Y)
HORN_BOLT_SQUARE = 10.0 # horn screws form a 10 mm square (side to side)
HORN_BOLT_DIA    = 2.8  # MEASURE: clearance for the horn screws
HUB_BORE_DIA     = 9.0  # central clearance for the servo spline boss

# ---- Cradle end (joint B -- cradles the tibia servo) ----
# The tibia servo lies with its 45.5 mm length along X and its rotation
# axis (the 35.5 mm horn-to-idler dimension) along Y -- horizontal and
# parallel to the plate hinge -- so the tibia swings in the leg's
# vertical plane. The horn exits one side wall; the 24.6 mm servo width
# is the vertical (Z) dimension.
WALL = 3.4              # cradle wall / floor thickness
SHAFT_BORE_DIA = 13.0   # clearance bore for the tibia servo shaft/horn
HORN_OFFSET    = 14.7   # measured: horn-axis offset from the servo length
                        # centre, toward the end away from the connector

# ---- Side cover (closes the cradle's open +Y face) ----
# The cradle is split by a vertical cut at the +Y wall of the servo
# pocket. The servo is set into the main body through the open +Y side,
# then this cover panel screws on. A counterbore in the cover lets the
# short (~6 mm) output post reach out for the tibia.
COUNTERBORE_DIA   = 17.0  # MEASURE: recess in the cover at the bore
COUNTERBORE_DEPTH = 4.5   # so the short output post is reachable
SCREW_CLEAR_DIA   = 3.4   # cover screw clearance hole (M3)
SCREW_PILOT_DIA   = 2.6   # main-body pilot hole for an M3 self-tapper
SCREW_DEPTH       = 8.0   # how far the pilot holes run into the body
SCREW_Z_SPREAD    = 5.0   # screw offset above/below the horn axis

# ---- Cross-section half-sizes (Y = width, Z = height) ----
HY_PLATE  = PLATE_THK / 2 + 1.6
HY_WAIST  = 8.5
HY_CRADLE = SERVO_H / 2 + FIT_GAP + WALL + 4.0   # Y spans the servo height
HZ_PLATE  = 12.5
HZ_WAIST  = 10.5
HZ_CRADLE = SERVO_W / 2 + FIT_GAP + WALL + 4.0   # Z spans the servo width

# ---- Organic shaping ----
SPINE_RISE   = 9.0      # gentle upward (Z) bow of the body
WAIST_AT     = 0.42     # where the slim waist sits along the body (0..1)
SCOOP_DEPTH  = 3.6      # depth of the smooth lightening scoops
CRADLE_BELLY = 0.06     # subtle organic swell at the middle of the cradle
NOSE_FRAC    = 0.40     # how far the cradle nose tapers down (fraction)
N_STATIONS   = 46       # cross-sections lofted along body + cradle
N_NOSE       = 14       # extra cross-sections through the tapered nose
N_RING       = 64       # points per cross-section

# ---- Output ----
STL_PATH = "femur.stl"
COVER_STL_PATH = "femur_cover.stl"
PNG_PATH = "femur.png"


# =============================================================
# DERIVED LAYOUT
# =============================================================
BODY_END = BODY_START + BODY_LEN
POCKET_LEN = SERVO_L + FIT_GAP
POCKET_X0 = BODY_END + CRADLE_BACK
POCKET_X1 = POCKET_X0 + POCKET_LEN
CRADLE_CX = 0.5 * (POCKET_X0 + POCKET_X1)
X_TOTAL = POCKET_X1 + CRADLE_NOSE
HORN_X = CRADLE_CX + HORN_OFFSET          # tibia joint axis, along Y
Y_SPLIT = 0.5 * (SERVO_H + FIT_GAP)       # +Y pocket wall = cover split


def cross_section(x):
    """Return (z_center, half_width, half_height, superellipse_exponent).

    Three regions blend smoothly: the body loft (plate -> waist), the
    cradle (full size with a soft belly around the servo pocket), and
    the nose (tapering past the pocket into a rounded organic tip). The
    exponent stays low (~2-3) throughout, so the cradle reads as a
    softened, bellied form rather than a hard box.
    """
    if x <= BODY_END:
        # body: plate -> waist -> cradle
        t = np.clip((x - BODY_START) / BODY_LEN, 0.0, 1.0)
        hy = np.interp(t, [0.0, WAIST_AT, 1.0], [HY_PLATE, HY_WAIST, HY_CRADLE])
        hz = np.interp(t, [0.0, WAIST_AT, 1.0], [HZ_PLATE, HZ_WAIST, HZ_CRADLE])
    elif x <= POCKET_X1:
        # cradle: full size around the pocket, with a subtle organic belly
        u = np.clip((x - BODY_END) / (POCKET_X1 - BODY_END), 0.0, 1.0)
        belly = 1.0 + CRADLE_BELLY * np.sin(np.pi * u)
        hy, hz = HY_CRADLE * belly, HZ_CRADLE * belly
    else:
        # nose: taper past the pocket into a soft rounded tip
        t = np.clip((x - POCKET_X1) / CRADLE_NOSE, 0.0, 1.0)
        k = NOSE_FRAC + (1.0 - NOSE_FRAC) * (1.0 - t * t)
        hy, hz = HY_CRADLE * k, HZ_CRADLE * k
    expo = np.interp(x, [BODY_START, BODY_END, CRADLE_CX, X_TOTAL],
                     [2.0, 2.6, 2.9, 2.1])
    # The body bows upward; the cradle stays straight so it tracks the
    # straight servo -- a bowed cradle drifts off the box-shaped cavity
    # and lets a cavity corner break through the outer surface.
    if x <= BODY_END:
        z_center = SPINE_RISE * np.sin(0.5 * np.pi * x / BODY_END)
    else:
        z_center = SPINE_RISE
    return z_center, hy, hz, expo


# =============================================================
# GEOMETRY HELPERS
# =============================================================
def superellipse_ring(x, n):
    """A closed ring of `n` points forming the cross-section at `x`."""
    zc, hy, hz, expo = cross_section(x)
    th = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    p = 2.0 / expo
    yy = hy * np.sign(c) * np.abs(c) ** p
    zz = hz * np.sign(s) * np.abs(s) ** p
    return np.column_stack([np.full(n, x), yy, zc + zz])


def loft_body(xs, n):
    """Loft a watertight tube through the cross-section rings at `xs`."""
    rings = [superellipse_ring(x, n) for x in xs]
    verts = list(np.vstack(rings))
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n):
            j2 = (j + 1) % n
            faces.append([a + j, a + j2, b + j2])
            faces.append([a + j, b + j2, b + j])
    # cap both ends with a triangle fan to the ring centroid
    first = len(verts)
    verts.append(rings[0].mean(axis=0))
    for j in range(n):
        faces.append([first, (j + 1) % n, j])
    last = len(verts)
    verts.append(rings[-1].mean(axis=0))
    base = (len(rings) - 1) * n
    for j in range(n):
        faces.append([last, base + j, base + (j + 1) % n])
    mesh = trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces))
    mesh.merge_vertices()
    mesh.fix_normals()
    return mesh


def y_cylinder(radius, length, center, sections=48):
    """A cylinder whose axis runs along Y, centred at `center`."""
    cyl = creation.cylinder(radius=radius, height=length, sections=sections)
    cyl.apply_transform(tf.rotation_matrix(np.pi / 2, [1, 0, 0]))
    cyl.apply_transform(tf.translation_matrix(center))
    return cyl


def ellipsoid(radii, center, subdivisions=3):
    """A smooth ellipsoid -- used to carve organic lightening scoops."""
    ball = creation.icosphere(subdivisions=subdivisions, radius=1.0)
    scale = np.eye(4)
    scale[0, 0], scale[1, 1], scale[2, 2] = radii
    ball.apply_transform(tf.translation_matrix(center) @ scale)
    return ball


# =============================================================
# BUILD
# =============================================================
def build_femur():
    # --- body loft + plate, blended into one blank -----------------
    xs = np.concatenate([
        np.linspace(BODY_START, POCKET_X1, N_STATIONS),
        np.linspace(POCKET_X1, X_TOTAL, N_NOSE)[1:],
    ])
    body = loft_body(xs, N_RING)
    plate = y_cylinder(PLATE_DIA / 2.0, PLATE_THK, [0.0, 0.0, 0.0])
    blank = trimesh.boolean.union([body, plate])

    zc = cross_section(CRADLE_CX)[0]    # cradle / horn-axis height
    hy_horn = cross_section(HORN_X)[1]  # cradle half-width at the bore

    # --- features cut through the whole femur ----------------------
    cutters = []

    # servo pocket: an internal cavity whose +Y face sits exactly on
    # the cover split plane, so the cover comes off as a solid panel.
    cutters.append(creation.box(
        extents=[SERVO_L + FIT_GAP, SERVO_H + FIT_GAP, SERVO_W + FIT_GAP],
        transform=tf.translation_matrix([CRADLE_CX, 0.0, zc]),
    ))

    # tibia joint: horn/idler through-bore on the Y axis.
    cutters.append(y_cylinder(SHAFT_BORE_DIA / 2.0, 2 * HY_CRADLE + 20.0,
                              [HORN_X, 0.0, zc]))

    # plate: central spline-boss bore + square of horn bolt holes
    cutters.append(y_cylinder(HUB_BORE_DIA / 2.0, PLATE_THK + 30.0,
                              [0.0, 0.0, 0.0]))
    h = HORN_BOLT_SQUARE / 2.0
    for bx, bz in [(h, h), (h, -h), (-h, -h), (-h, h)]:
        cutters.append(y_cylinder(HORN_BOLT_DIA / 2.0, PLATE_THK + 30.0,
                                  [bx, 0.0, bz]))

    # smooth scoops in the body flanks -- lightening + organic look
    scoop_x = BODY_START + WAIST_AT * BODY_LEN
    zc_waist, hy_waist, _, _ = cross_section(scoop_x)
    for side in (+1, -1):
        cy = side * (hy_waist + 8.0 - SCOOP_DEPTH)
        cutters.append(ellipsoid([27.0, 8.0, 14.0], [scoop_x, cy, zc_waist]))

    femur = trimesh.boolean.difference([blank] + cutters)

    # --- side split: cradle body (main) + bolt-on cover -----------
    # A vertical cut at the +Y wall of the servo pocket. The servo is
    # set into the main body through the open +Y side; the cover panel
    # then screws over it.
    cover_box = creation.box(
        extents=[(X_TOTAL + 20.0) - BODY_END, 400.0, 600.0],
        transform=tf.translation_matrix(
            [0.5 * (BODY_END + X_TOTAL + 20.0), Y_SPLIT + 200.0, zc]),
    )
    main = trimesh.boolean.difference([femur, cover_box])
    cover = trimesh.boolean.intersection([femur, cover_box])

    # counterbore in the cover: a recess at the bore so the short
    # output post pokes into it and the tibia hub can reach the post.
    cb_floor = hy_horn - COUNTERBORE_DEPTH
    cover = trimesh.boolean.difference([cover, y_cylinder(
        COUNTERBORE_DIA / 2.0, 200.0, [HORN_X, cb_floor + 100.0, zc])])

    # --- fasteners: screws clamp the cover to the body -----------
    # Clearance holes through the cover, blind pilot holes into the
    # body. The nose is too slim to contain an upper screw, so it gets
    # a single one on the horn-axis line while the back end gets two.
    back_x = BODY_END + CRADLE_BACK / 2.0
    nose_x = POCKET_X1 + CRADLE_NOSE * 0.4
    screw_pts = [
        (back_x, zc + SCREW_Z_SPREAD),
        (back_x, zc - SCREW_Z_SPREAD),
        (nose_x, zc - SCREW_Z_SPREAD),
    ]
    cover_cuts, main_cuts = [], []
    for sx, sz in screw_pts:
        cover_cuts.append(y_cylinder(SCREW_CLEAR_DIA / 2.0, 120.0,
                                     [sx, 0.0, sz]))
        main_cuts.append(y_cylinder(
            SCREW_PILOT_DIA / 2.0, SCREW_DEPTH + 4.0,
            [sx, Y_SPLIT - SCREW_DEPTH / 2.0 + 2.0, sz]))
    cover = trimesh.boolean.difference([cover] + cover_cuts)
    main = trimesh.boolean.difference([main] + main_cuts)

    for part in (main, cover):
        part.merge_vertices()
        part.fix_normals()
    return main, cover


# =============================================================
# PREVIEW RENDER
# =============================================================
def render_png(main, cover, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    bg = "#0c0d11"
    key = np.array([0.35, 0.45, 0.82])
    key = key / np.linalg.norm(key)
    fill = np.array([-0.55, -0.25, 0.30])
    fill = fill / np.linalg.norm(fill)

    def shaded(mesh, base):
        n = mesh.face_normals
        lit = (0.24
               + 0.62 * np.clip(n @ key, 0.0, 1.0)
               + 0.22 * np.clip(n @ fill, 0.0, 1.0))
        col = np.array(base)
        return np.clip(col[None, :] * np.clip(lit, 0.0, 1.15)[:, None], 0.0, 1.0)

    main_col = (0.44, 0.60, 0.74)
    cover_col = (0.66, 0.71, 0.47)   # tinted so the seam reads clearly

    corners = np.vstack([main.bounds, cover.bounds])
    ctr = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    half = (corners.max(axis=0) - corners.min(axis=0)).max() / 2.0 * 1.06

    panels = [
        ("assembled - 3/4", 26, -56, [(main, main_col), (cover, cover_col)]),
        ("assembled - side", 2, -90, [(main, main_col), (cover, cover_col)]),
        ("cover - outer face", 14, 64, [(cover, cover_col)]),
        ("cradle - servo bay", 22, 120, [(main, main_col)]),
    ]
    fig = plt.figure(figsize=(11, 9), facecolor=bg)
    for k, (title, elev, azim, items) in enumerate(panels):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        for mesh, base in items:
            ax.add_collection3d(Poly3DCollection(
                mesh.triangles, facecolors=shaded(mesh, base),
                edgecolors="none"))
        ax.set_xlim(ctr[0] - half, ctr[0] + half)
        ax.set_ylim(ctr[1] - half, ctr[1] + half)
        ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_axis_off()
        ax.set_facecolor(bg)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color="#9fb3c8", fontsize=11)
    fig.suptitle("CRABORA femur  -  side-loading cradle", color="#d8e2ec",
                 fontsize=14, y=0.97)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    print("Building femur (side-loading cradle)...")
    main_part, cover = build_femur()

    for label, part in [("main  (body + plate + cradle)", main_part),
                        ("cover (cradle side panel)", cover)]:
        e = part.extents
        vol = part.volume / 1000.0
        print(f"  {label}")
        print(f"    bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
              f"vol {vol:.1f} cm^3   watertight {part.is_watertight}")
    print(f"  femur length : {HORN_X:.1f} mm  (plate hinge to tibia axis)")

    main_part.export(STL_PATH)
    cover.export(COVER_STL_PATH)
    print(f"✓ wrote {STL_PATH} + {COVER_STL_PATH}")

    print("Rendering preview...")
    render_png(main_part, cover, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
