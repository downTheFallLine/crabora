"""
CRABORA tibia link -- knee to foot
====================================

The last rigid leg part.  Bolts to the tibia servo's horn at the
knee, extends 120 mm to the foot tip.

Outline shape (v2): a TEARDROP.  The two side edges are tangent lines
from each foot corner to the knee-disc circle, so the boundary
transitions SMOOTHLY from the disc into the tapered body -- no "notch"
or step where the disc meets the beam (which v1 had, because v1 was a
union of a disc + rectangular beam, leaving a 90-deg step at the disc
edge).  The whole link tapers continuously from KNEE_DISC_DIA wide at
the knee to FOOT_WIDTH wide at the foot, ending in a SQUARED flat edge
(v1 ended in a semicircular tip).

Knee end:
  - Disc Φ32, 5 mm thick
  - 4 M3 horn-bolt holes on a Φ14 bolt circle (10 mm square pattern)
  - Φ8 central relief for the horn's central M3 screw head

Body:
  - Continuous taper from the disc to the foot, no rectangular beam
    section.  5 mm thick (bumped from v1's 3 mm to match the femur
    link strength upgrade).
  - Warren-truss lightening: N_TRUSS triangles in the body region,
    alternating apex up/down.  Each triangle auto-scales to the local
    half-width of the tapered outline, so they get smaller toward the
    foot.  The tibia link sees mostly axial (compression) load when
    the foot is planted, not the bending peaks the femur sees, so the
    apex-corner stress risers that snapped the femur are less of a
    concern here.

Foot end:
  - Flat squared edge, FOOT_WIDTH mm wide (default 10 mm).  Foot tip =
    the +X edge of the link = the ground-contact line in the leg's
    vertical plane.
  - For better grip on smooth surfaces, glue a rubber pad / silicone
    bump / hot-glue blob on the link's edge at the foot tip post-print.

Per leg: 1 tibia link.  Hexapod: 6.

Print orientation: flat on the bed, either face down.  The whole part
is a flat plate with through-features only -- no overhangs.

Usage:
  python design_tibia_link.py
"""

import os
import numpy as np
import shapely.geometry
import shapely.geometry.polygon
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  (mm)
# =============================================================
# Overall length: from KNEE horn axis to FOOT tip
LINK_LEN              = 120.0

# Knee disc (where it bolts to tibia horn)
KNEE_DISC_DIA         = 32.0
LINK_THK              = 5.0        # was 3.0; bumped to match the femur-link v2
                                   # strength upgrade (bending strength ~thk^2).

# Foot end: flat squared edge of this width at X=LINK_LEN.
# v1 had a semicircular tip of radius BEAM_WIDTH/2 = 12 mm (so the foot
# was effectively 24 mm wide and rounded).  v2 tapers to a narrow flat
# edge.  10 mm is a moderate taper from the 32 mm disc.
FOOT_WIDTH            = 10.0

# Knee-end fasteners (horn attach)
HORN_BOLT_SQUARE      = 10.0   # 4 bolts at ±5,±5 = Φ14 bolt circle
HORN_BOLT_CLEAR       = 3.4    # M3 medium clearance
HORN_CENTRAL_CLEAR    = 8.0    # Φ8 relief for the horn's central M3 + head

# Warren-truss lightening: alternating triangles in the body region.
# Each triangle's vertices auto-track the link's local half-width along
# the taper, so they shrink toward the foot.  TRUSS_X_LO/HI bound the
# region: LO sits past the horn-bolt circle, HI stays clear of the
# narrow foot end.  N_TRUSS=0 disables the truss (solid body).
N_TRUSS               = 5
TRUSS_MARGIN          = 2.0    # min wall between adjacent triangle cells
TRUSS_CHORD_MARGIN    = 2.5    # min wall between triangle vertex and link edge
TRUSS_X_LO            = 12.0   # X where the truss region starts
TRUSS_X_HI            = 108.0  # X where the truss region ends

# Outline tessellation: number of points along the back-of-disc arc.
N_ARC                 = 96

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/tibia_link.stl"
PNG_PATH = OUT_DIR + "/tibia_link.png"


# =============================================================
# HELPERS
# =============================================================
def z_cylinder(radius, height, center, sections=48):
    cyl = creation.cylinder(radius=radius, height=height, sections=sections)
    cyl.apply_translation(center)
    return cyl


def z_triangle_prism(p1, p2, p3, thickness):
    """Triangular prism extruded along Z (thickness = Z extent)."""
    h2 = thickness / 2.0
    verts = np.array([
        [p1[0], p1[1], -h2], [p1[0], p1[1], +h2],
        [p2[0], p2[1], -h2], [p2[0], p2[1], +h2],
        [p3[0], p3[1], -h2], [p3[0], p3[1], +h2],
    ])
    faces = np.array([
        [0, 4, 2], [1, 3, 5],
        [0, 2, 3], [0, 3, 1],
        [2, 4, 5], [2, 5, 3],
        [4, 0, 1], [4, 1, 5],
    ])
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    mesh.merge_vertices()
    mesh.fix_normals()
    return mesh


def tangent_point_geometry(disc_radius, link_len, foot_y):
    """Return (theta_t, t_x, t_y): the angle and (x,y) of the upper
    tangent point on the disc circle for the upper foot corner.  See
    teardrop_outline for the derivation."""
    R = disc_radius
    op_dist = np.sqrt(link_len * link_len + foot_y * foot_y)
    tp_len  = np.sqrt(op_dist * op_dist - R * R)
    alpha = np.arctan2(foot_y, link_len)
    beta  = np.arctan2(tp_len, R)
    theta_t = alpha + beta
    return theta_t, R * np.cos(theta_t), R * np.sin(theta_t)


def teardrop_outline(disc_radius, link_len, foot_width, n_arc):
    """2D outline of the tibia link: knee disc smoothly tapering to a
    flat foot edge of width foot_width at X=link_len.

    Construction: each foot corner has two tangent lines to the disc
    (circle of radius disc_radius at the origin); we use the tangent
    that touches the FAR side of the disc, so the link body extends to
    +X.  Then the outline is:

        back-of-disc arc  (CCW from upper tangent point through θ=π
                           to lower tangent point)
        lower foot corner (link_len, -foot_width/2)
        upper foot corner (link_len, +foot_width/2)
        [shapely closes back to the upper tangent point]

    Returns a CCW shapely Polygon.
    """
    R = disc_radius
    foot_y = foot_width / 2.0

    # Angle (rad) on the disc of the tangent point touching the upper
    # foot corner.  From external point P=(LX, foot_y), the two tangent
    # points are at angles α±β where α = atan2(P.y, P.x) is the angle of
    # OP and β = atan2(|TP|, R) is the angle between OT and OP (since
    # the triangle OTP has a right angle at T).  We pick α+β so the
    # tangent point lies on the +Y side of the disc.
    op_dist = np.sqrt(link_len * link_len + foot_y * foot_y)
    tp_len  = np.sqrt(op_dist * op_dist - R * R)
    alpha = np.arctan2(foot_y, link_len)
    beta  = np.arctan2(tp_len, R)
    theta_t = alpha + beta

    # Back-of-disc arc: CCW from upper tangent (θ=theta_t, +Y side)
    # through θ=π (-X side) to lower tangent (θ=2π-theta_t, -Y side).
    arc_thetas = np.linspace(theta_t, 2 * np.pi - theta_t, n_arc)
    arc_pts = [(R * np.cos(t), R * np.sin(t)) for t in arc_thetas]

    # Walk arc upper->back->lower, then to the lower foot corner, then
    # to the upper foot corner.  shapely closes back to the start
    # (upper tangent point), giving the upper tapered edge automatically.
    pts = arc_pts + [(link_len, -foot_y), (link_len, +foot_y)]
    poly = shapely.geometry.Polygon(pts)
    return shapely.geometry.polygon.orient(poly, sign=1.0)


# =============================================================
# BUILD
# =============================================================
def build_link():
    """CAD frame: origin at KNEE horn axis, link's centre plane at Z=0.
    +X points toward foot.  Foot tip = +X edge at X=LINK_LEN."""

    # ---------------------------------------------------------------
    # POSITIVE solid: extruded teardrop outline
    # ---------------------------------------------------------------
    outline = teardrop_outline(
        disc_radius=KNEE_DISC_DIA / 2.0,
        link_len=LINK_LEN,
        foot_width=FOOT_WIDTH,
        n_arc=N_ARC,
    )
    body = creation.extrude_polygon(outline, LINK_THK)
    # extrude_polygon extrudes along +Z from 0 to LINK_THK; centre on Z=0
    body.apply_translation([0.0, 0.0, -LINK_THK / 2.0])

    # ---------------------------------------------------------------
    # NEGATIVE cutters: knee-end fasteners only.  No truss in v2 --
    # the solid teardrop is the strength story (mirroring femur v2).
    # ---------------------------------------------------------------
    cutters = []

    # Central relief for the horn screw head
    cutters.append(z_cylinder(
        HORN_CENTRAL_CLEAR / 2.0, LINK_THK + 2.0, [0.0, 0.0, 0.0]))
    # 4 horn-bolt clearance holes on a 10 mm square
    h = HORN_BOLT_SQUARE / 2.0
    for dx, dy in [(+h, +h), (+h, -h), (-h, -h), (-h, +h)]:
        cutters.append(z_cylinder(
            HORN_BOLT_CLEAR / 2.0, LINK_THK + 2.0, [dx, dy, 0.0]))

    # ---- Warren-truss lightening (auto-scaled to the taper) ---------
    if N_TRUSS > 0:
        R = KNEE_DISC_DIA / 2.0
        foot_y = FOOT_WIDTH / 2.0
        _, tangent_x, tangent_y = tangent_point_geometry(R, LINK_LEN, foot_y)

        def half_width_at(x):
            """+Y of the upper edge of the outline at this x.  Uses the
            disc circle for x <= tangent_x, the tangent line otherwise."""
            if x <= tangent_x:
                return np.sqrt(max(0.0, R * R - x * x))
            frac = (x - tangent_x) / (LINK_LEN - tangent_x)
            return tangent_y + (foot_y - tangent_y) * frac

        cell_w = (TRUSS_X_HI - TRUSS_X_LO) / N_TRUSS
        half_margin = TRUSS_MARGIN / 2.0
        truss_cutter_thk = LINK_THK + 2.0
        for i in range(N_TRUSS):
            cell_x0 = TRUSS_X_LO + i * cell_w
            cell_x1 = cell_x0 + cell_w
            base_x0 = cell_x0 + half_margin
            base_x1 = cell_x1 - half_margin
            apex_x  = 0.5 * (base_x0 + base_x1)
            # Inset from the local outline by TRUSS_CHORD_MARGIN so the
            # triangle stays clear of the edge even as the link tapers.
            hw_b0 = half_width_at(base_x0) - TRUSS_CHORD_MARGIN
            hw_b1 = half_width_at(base_x1) - TRUSS_CHORD_MARGIN
            hw_ap = half_width_at(apex_x)  - TRUSS_CHORD_MARGIN
            if i % 2 == 0:
                # apex on +Y, base along -Y edge
                p_a = (base_x0, -hw_b0)
                p_b = (base_x1, -hw_b1)
                p_c = (apex_x,  +hw_ap)
            else:
                # apex on -Y, base along +Y edge
                p_a = (base_x0, +hw_b0)
                p_b = (base_x1, +hw_b1)
                p_c = (apex_x,  -hw_ap)
            cutters.append(z_triangle_prism(p_a, p_b, p_c, truss_cutter_thk))

    link = trimesh.boolean.difference([body] + cutters)
    link.merge_vertices()
    link.fix_normals()
    return link


# =============================================================
# PREVIEW
# =============================================================
def render_png(link, path):
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
    b = link.bounds
    ctr  = 0.5 * (b.min(axis=0) + b.max(axis=0))
    half = (b.max(axis=0) - b.min(axis=0)).max() / 2.0 * 1.05

    panels = [
        ("3/4 view",              25, -55),
        ("top down  (face view)", 88, -90),
        ("side view",              0, -90),
    ]
    fig = plt.figure(figsize=(15, 5.5), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.set_proj_type("ortho")
        ax.add_collection3d(Poly3DCollection(
            link.triangles, facecolors=shaded(link, col),
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
        f"CRABORA tibia link  v2  (teardrop, squared foot)  -  "
        f"{LINK_LEN:.0f} mm knee-to-foot,  "
        f"Φ{KNEE_DISC_DIA:.0f} disc -> {FOOT_WIDTH:.0f} mm foot,  "
        f"{LINK_THK:.0f} mm thick",
        color="#d8e2ec", fontsize=13, y=1.02)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building tibia link v2 (teardrop, squared foot)...")
    link = build_link()
    e = link.extents
    vol = link.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {link.is_watertight}")
    print(f"  length    : {LINK_LEN:.0f} mm (tibia horn axis -> foot tip)")
    truss_note = (f"{N_TRUSS} truss triangles (auto-scaled X={TRUSS_X_LO:.0f}..{TRUSS_X_HI:.0f})"
                  if N_TRUSS > 0 else "solid (no truss)")
    print(f"  taper     : Φ{KNEE_DISC_DIA:.0f} disc -> {FOOT_WIDTH:.0f} mm "
          f"flat foot,  {LINK_THK:.0f} mm thick,  {truss_note}")
    print(f"  knee      : Φ{KNEE_DISC_DIA:.0f} disc, 4 M3 horn bolts on "
          f"{HORN_BOLT_SQUARE:.0f} mm square + Φ{HORN_CENTRAL_CLEAR:.0f} central relief,  "
          f"tangent-blended into body (no notch)")
    print(f"  foot      : flat squared edge, {FOOT_WIDTH:.0f} mm wide  "
          f"(glue rubber pad post-print for grip)")

    link.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(link, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
