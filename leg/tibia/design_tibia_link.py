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
the knee to FOOT_WIDTH wide at the foot, ending in a ROUNDED tip
(v1 ended in a semicircular tip).

Knee end:
  - Disc Φ32, 5 mm thick
  - 4 M3 horn-bolt holes on a Φ14 bolt circle (10 mm square pattern)
  - Φ8 central relief for the horn's central M3 screw head

Body:
  - Continuous taper from the disc to the foot, no rectangular beam
    section.  5 mm thick (bumped from v1's 3 mm to match the femur
    link strength upgrade).
  - Warren-truss lightening: DISABLED in v3 (N_TRUSS=0).  Was N_TRUSS triangles in the body region,
    alternating apex up/down.  Each triangle auto-scales to the local
    half-width of the tapered outline, so they get smaller toward the
    foot.  The tibia link sees mostly axial (compression) load when
    the foot is planted, not the bending peaks the femur sees, so the
    apex-corner stress risers that snapped the femur are less of a
    concern here.

Foot end:
  - Rounded tip, FOOT_WIDTH mm across (default 10 mm).  Foot tip =
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
import shapely.ops
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

# Foot end: ROUNDED tip of this diameter, tip at X=LINK_LEN.
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
# --- v4 spine (2026-10-11) -------------------------------------------
# Tibias were snapping NEAR THE KNEE, on carpet, during turns. That is
# the cantilever root -- maximum bending moment -- and the load is
# LATERAL (perpendicular to the plate face), which is the one direction
# a flat 5 mm plate is hopeless in.
#
# A rib down the centreline turns the section into a T-beam. At a
# representative 20 mm-wide station:
#       5 mm flat plate            Z =  83 mm^3   (1.0x)
#       8 mm flat plate            Z = 213 mm^3   (2.6x) for +60% material
#       5 mm plate + 5x12 mm rib   Z = 322 mm^3   (3.9x) for +60% material
# Same mass, half again the strength -- so a rib, not a thicker plate.
# PLA only, so this has to be won geometrically.
#
# The rib sits on ONE face (away from the horn), starts just outboard of
# the knee disc so it clears the horn bolts, and tapers away toward the
# foot where the moment is small. Prints flat with the rib upward: no
# overhangs, no supports.
SPINE_ENABLED   = True
SPINE_WIDTH     = 5.0     # Y thickness of the rib
SPINE_HEIGHT    = 12.0    # how far it stands proud of the plate face
SPINE_X_START   = 17.0    # just past the Phi32 disc (R=16)
SPINE_X_FULL    = 30.0    # full height by here -- ramped, not a step, so
                          # the rib does not create its own stress riser
SPINE_X_TAPER   = 95.0    # starts dying away
SPINE_X_END     = 112.0   # gone before the rounded foot
SPINE_SIDE      = +1.0    # +Z face; the horn mounts on the other side

# --- v4 markings -----------------------------------------------------
# Name and block, raised on the flat -Z face (the side without the rib).
# DACTYL is the terminal segment of an arthropod leg -- coxa, basis,
# ischium, merus, carpus, propodus, dactylus -- so it is the anatomically
# correct name for the part that actually touches the ground.
MARK_ENABLED    = True
MARK_NAME       = "DACTYL"
MARK_BLOCK      = "BLOCK 4"
MARK_NAME_SIZE  = 5.5     # font size, mm (cap height is ~0.72 of this)
MARK_BLOCK_SIZE = 4.0
MARK_DEPTH      = 0.6     # ENGRAVED this deep into the face
MARK_OVERLAP    = 0.2     # cutter stands this far proud before cutting,
                          # so the boolean has no coincident faces to
                          # resolve -- those leave the mesh non-watertight
MARK_X          = 22.0    # start of the lettering, just past the disc
MARK_SIDE       = +1.0    # +Z face -- the SAME side as the spine
MARK_Y_GAP      = 0.7     # clear gap between the rib and the first row;
                          # the text sits in the band beside the rib,
                          # which runs ~11.6 mm at the knee down to
                          # ~6.8 mm by X=70, so it has to stay compact

# v3 (2026-10-11): truss disabled. The open triangles removed material
# from the middle of the link, which is exactly where bending stress
# peaks, and a tibia broke. Solid is heavier but the leg is not mass
# critical -- the servo is.
N_TRUSS               = 0
TRUSS_MARGIN          = 2.0    # min wall between adjacent triangle cells
TRUSS_CHORD_MARGIN    = 2.5    # min wall between triangle vertex and link edge
TRUSS_X_LO            = 12.0   # X where the truss region starts
TRUSS_X_HI            = 108.0  # X where the truss region ends

# Outline tessellation: number of points along the back-of-disc arc.
N_ARC                 = 96

# Resolved from this script's own location, so output lands beside the
# design file no matter which directory it is run from.
OUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "linkage_leg")
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
    ROUNDED foot tip of radius foot_width/2, with the tip at X=link_len.

    v3 (2026-10-11): the foot was a flat squared edge and caught on the
    floor hard enough to snap a tibia. A round tip presents a tangent to
    the ground through the whole swing instead of a corner, so the leg
    skids rather than digging in.

    Construction is simply the convex hull of two circles -- the knee
    disc at the origin and the foot circle at (link_len - r, 0). That
    yields the two tangent flanks and both arcs exactly, with no tangent
    trigonometry to get wrong. The tip stays at X=link_len, so
    TIBIA_LEN=120 in walk/gait_lib.py remains the knee-axis-to-foot-tip
    distance and the kinematics are unaffected.
    """
    r = foot_width / 2.0
    quad = max(8, n_arc // 4)
    disc = shapely.geometry.Point(0.0, 0.0).buffer(disc_radius, quad_segs=quad)
    foot = shapely.geometry.Point(link_len - r, 0.0).buffer(r, quad_segs=quad)
    poly = disc.union(foot).convex_hull
    return shapely.geometry.polygon.orient(poly, sign=1.0)


def _unused_squared_outline(disc_radius, link_len, foot_width, n_arc):
    """Superseded v2 outline, kept only as a record of the flat foot.

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


def _text_polygons(text, size):
    """Glyph outlines for `text` as shapely polygons, holes included.

    matplotlib gives every ring as a separate closed polygon with no
    indication of which are holes, so they are combined by symmetric
    difference -- the even-odd rule, which is how fonts are filled. Do
    it by union and the counters in A, B, C, O, D fill in solid.
    """
    from matplotlib.font_manager import FontProperties
    from matplotlib.textpath import TextPath
    import functools

    path = TextPath((0, 0), text, size=size,
                    prop=FontProperties(family="DejaVu Sans", weight="bold"))
    rings = [shapely.geometry.Polygon(r) for r in path.to_polygons()
             if len(r) >= 3]
    rings = [r.buffer(0) for r in rings if r.is_valid or True]
    if not rings:
        return None
    glyphs = functools.reduce(lambda a, b: a.symmetric_difference(b), rings)
    # Adjacent letters can meet at exactly one POINT at their kerning.
    # unary_union leaves those as separate polygons -- they overlap
    # nowhere -- and extruding a point contact produces a vertical edge
    # with four faces on it, i.e. a non-manifold mesh. Fattening by a
    # hair first makes them genuinely overlap so the union merges them.
    # 0.02 mm is far below anything a 0.4 mm nozzle will render.
    return shapely.ops.unary_union(glyphs.buffer(0.02))


def _lettering(outline):
    """ENGRAVED DACTYL / BLOCK n beside the rib. Returns cutter solids.

    The text goes on the same face as the spine, in the flat band to one
    side of it. That band is narrow and tapers -- about 11.6 mm wide at
    the knee, 6.8 mm by X=70 -- so the rows are stacked outward from the
    rib and kept small.

    Each cutter stands MARK_OVERLAP proud of the face before cutting
    down MARK_DEPTH. Starting exactly level would share a face with the
    body, which booleans resolve badly.
    """
    parts = []
    y = SPINE_WIDTH / 2.0 + MARK_Y_GAP      # outward edge of the rib
    # Rows stack OUTWARD from the rib, so the block goes down first and
    # the name ends up on top, which is the way round you want to read it.
    rows = ((MARK_BLOCK, MARK_BLOCK_SIZE), (MARK_NAME, MARK_NAME_SIZE))
    for text, size in rows:
        glyphs = _text_polygons(text, size)
        if glyphs is None or glyphs.is_empty:
            continue
        pieces = (list(glyphs.geoms)
                  if glyphs.geom_type == "MultiPolygon" else [glyphs])
        solids = [creation.extrude_polygon(g, MARK_DEPTH + MARK_OVERLAP)
                  for g in pieces]
        solid = (solids[0] if len(solids) == 1
                 else trimesh.boolean.union(solids))

        b = solid.bounds
        solid.apply_translation([
            MARK_X - b[0][0],
            y - b[0][1],
            MARK_SIDE * LINK_THK / 2.0 - MARK_DEPTH,
        ])
        y += (b[1][1] - b[0][1]) + MARK_Y_GAP

        # Engraving that runs off the edge is a hole, not a marking.
        bb = solid.bounds
        box = shapely.geometry.box(bb[0][0], bb[0][1], bb[1][0], bb[1][1])
        if not outline.contains(box):
            raise ValueError(
                f"lettering {text!r} at size {size} does not fit beside the "
                f"rib (needs X {bb[0][0]:.1f}..{bb[1][0]:.1f}, "
                f"Y {bb[0][1]:.1f}..{bb[1][1]:.1f}) -- reduce MARK_*_SIZE "
                f"or move MARK_X inboard where the part is wider")
        parts.append(solid)
    return parts


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
    # ---------------------------------------------------------------
    # POSITIVE: the spine. A ribbed T-section, because the break is at
    # the knee and the load is lateral.  See the v4 note above.
    # ---------------------------------------------------------------
    if SPINE_ENABLED:
        # Swept as a single profile rather than stacked boxes: the ramps
        # come out smooth. A stepped rib would put a row of little stress
        # risers along the exact region that keeps breaking.
        from trimesh import transformations as tf
        profile = shapely.geometry.Polygon([
            (SPINE_X_START, 0.0),
            (SPINE_X_FULL, SPINE_HEIGHT),
            (SPINE_X_TAPER, SPINE_HEIGHT),
            (SPINE_X_END, 0.0),
        ])
        # Build in XY (y = rib height), extrude along Z by the rib width,
        # then stand it up so height is Z and width is Y.
        spine = creation.extrude_polygon(profile, SPINE_WIDTH)
        spine.apply_transform(tf.rotation_matrix(np.pi / 2, [1, 0, 0]))
        spine.apply_translation([0.0, SPINE_WIDTH / 2.0,
                                 SPINE_SIDE * LINK_THK / 2.0])
        body = trimesh.boolean.union([body, spine])

    cutters = []

    # Engraved lettering, cut into the face beside the spine.
    if MARK_ENABLED:
        cutters.extend(_lettering(outline))

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
    # Glyph outlines carry the odd degenerate sliver, which leaves a
    # handful of open edges after the boolean. Harmless in itself, but a
    # non-watertight STL is one a slicer may quietly mangle, so clean it.
    link.update_faces(link.nondegenerate_faces())
    link.update_faces(link.unique_faces())
    link.remove_unreferenced_vertices()
    if not link.is_watertight:
        link.fill_holes()
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
        ("3/4 view  (spine up)",     25, -55),
        ("top down  (spine face)",   88, -90),
        ("side view",                 0, -90),

        ("spine face  (lettering)",  88, -90),
    ]
    fig = plt.figure(figsize=(15, 10), facecolor=bg)
    for k, (title, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
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
        f"CRABORA tibia link  v4  (rounded foot, ribbed spine)  -  "
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
    print("Building tibia link v4 (rounded foot, solid, ribbed spine)...")
    link = build_link()
    e = link.extents
    vol = link.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {link.is_watertight}")
    print(f"  length    : {LINK_LEN:.0f} mm (tibia horn axis -> foot tip)")
    truss_note = (f"{N_TRUSS} truss triangles (auto-scaled X={TRUSS_X_LO:.0f}..{TRUSS_X_HI:.0f})"
                  if N_TRUSS > 0 else "solid (no truss)")
    print(f"  taper     : Φ{KNEE_DISC_DIA:.0f} disc -> Φ{FOOT_WIDTH:.0f} mm rounded tip, "
          f"{LINK_THK:.0f} mm thick,  {truss_note}")
    print(f"  knee      : Φ{KNEE_DISC_DIA:.0f} disc, 4 M3 horn bolts on "
          f"{HORN_BOLT_SQUARE:.0f} mm square + Φ{HORN_CENTRAL_CLEAR:.0f} central relief,  "
          f"tangent-blended into body (no notch)")
    if SPINE_ENABLED:
        print(f"  spine     : {SPINE_WIDTH:.0f} x {SPINE_HEIGHT:.0f} mm rib on "
              f"the {'+' if SPINE_SIDE > 0 else '-'}Z face, X="
              f"{SPINE_X_START:.0f}..{SPINE_X_END:.0f} "
              f"(full {SPINE_X_FULL:.0f}..{SPINE_X_TAPER:.0f})  "
              f"-> ~3.9x lateral strength at the knee")
    if MARK_ENABLED:
        print(f"  marking   : \"{MARK_NAME}\" / \"{MARK_BLOCK}\" ENGRAVED "
              f"{MARK_DEPTH:.1f} mm into the "
              f"{'+' if MARK_SIDE > 0 else '-'}Z face from X={MARK_X:.0f}")
    print(f"  foot      : ROUNDED tip, Φ{FOOT_WIDTH:.0f} mm  "
          f"(glue rubber pad post-print for grip)")

    link.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(link, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
