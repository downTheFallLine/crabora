"""
CRABORA femur link -- rigid arm from femur horn (hip) to knee
==============================================================

Connects the femur servo's horn (at the hip) to the tibia subsystem
(at the knee).  Length parameterized; default 100 mm centre-to-centre.

Architecture:
  - Hip end: Φ32 disc bolted to the femur horn (4 M3 on Φ14 bolt
    circle = 10 mm square, plus Φ8 central relief)
  - Knee end: Φ32 disc with hole(s) for whatever the tibia subsystem
    needs.  Two modes:
        * "passive" (default): M4 bushing hole at centre -- the tibia
          pivots passively about this axis, driven by a push-rod from
          a crank servo at the hip (4-bar architecture).
        * "active": 4 M3 mount holes for a tibia cap that drives the
          tibia directly (direct-drive 2-DoF architecture).
  - Beam between: 24 mm wide x 5 mm thick, SOLID, with a small 45-deg
    chamfer on all 4 long edges (CHAMFER) for a "machined" aesthetic
    and quarter-arc fillets (FILLET_R) at the two inboard corners where
    the beam meets the knee plate -- those corners are stress risers
    exactly where bending load peaks. (v1 had a Warren-truss lightening
    pattern; it failed in the field at a triangle apex -- see the
    N_TRUSS comment in PARAMETERS.)

Print orientation: flat on the bed, either side down -- all features
are through-holes so the part is symmetric in thickness.

Per leg: 1 femur link.  For a hexapod: 6.

Usage:
  python design_femur_link.py
"""

import os
import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  (mm)
# =============================================================
# Overall length: from HIP horn axis to TIBIA horn axis
# (the "useful" femur length, axis-to-axis)
LINK_LEN              = 100.0

# Hip-end disc
HIP_DISC_DIA          = 32.0
LINK_THK              = 5.0        # was 3.0; bumped after a mid-beam fracture --
                                   # bending strength goes as thickness^2, so
                                   # 3->5 mm gives ~2.8x the strength.

# Beam between hip disc and knee plate
BEAM_WIDTH            = 24.0
BEAM_CHAMFER          = 1.2        # 45-deg chamfer on all 4 long edges of the
                                   # beam -- the "machined-aluminium" aesthetic.
                                   # 0 = sharp rectangle (cosmetic only, no
                                   # strength impact either way).

# Hip-end fasteners (horn attach)
HORN_BOLT_SQUARE      = 10.0   # 4 bolts at ±5,±5 = Φ14 bolt circle
HORN_BOLT_CLEAR       = 3.4    # M3 medium clearance
HORN_CENTRAL_CLEAR    = 8.0    # Φ8 relief for the horn's central M3 + head

# Knee-end mode: "passive" (M4 bushing) or "active" (4x M3 tibia-cap mount)
KNEE_MODE             = "active"

# Passive knee (4-bar architecture): single M4 bushing hole at the knee axis
BUSHING_BORE          = 6.1    # 6 mm OD bushing + 0.1 slip fit
KNEE_DISC_DIA         = 32.0   # disc size for passive mode

# Active knee (direct-drive 2-DoF): rectangular plate sized to match
# the tibia cap's v2 (flangeless) footprint, with small "wings" that
# host the bolt holes.  The cap is 52.2 x 39.7 mm; the plate is 52.2 x
# 42.0 mm -- the extra ~1 mm of Y on each side leaves room for the
# bolt heads (or nuts) without overhanging the cap.  The 4 mount holes
# align with the cap's vertical bolt passages, which thread long M3
# bolts that tie the plate to the cap (heads on cap back, nuts on
# plate underside).  The cap's horn axis sits +KNEE_TIBIA_OFFSET past
# the plate centre, so the plate centre is inboard of the tibia axis.
KNEE_TIBIA_OFFSET     = 12.5    # = tibia cap's HORN_AXIS_OFFSET
KNEE_PLATE_X          = 52.2    # X extent of knee plate (= tibia cap's CAP_X)
KNEE_PLATE_Y          = 42.0    # Y extent (= cap CAP_Y_CORE 39.7 + 2.3 wing margin)
KNEE_MOUNT_X_HALF     = 15.0    # ±X from knee plate centre  (= cap SCREW_OFFSET_X)
KNEE_MOUNT_Y_HALF     = 16.35   # ±Y from knee plate centre  (= cap SCREW_OFFSET_Y;
                                # centred in the cap's thickened ±Y wall)
KNEE_BOLT_CLEAR       = 3.8     # M3 generous clearance (was 3.4; loosened for tolerance)
KNEE_FILLET_R         = 4.0     # quarter-arc fillet at each of the two inboard
                                # corners where the beam meets the plate.  These
                                # 90-deg corners were stress risers exactly where
                                # bending load is highest -- a fillet smooths the
                                # load path AND adds material to the inboard edge
                                # (partially compensating for the cable notch
                                # that cuts the middle of that edge).  0 = sharp
                                # corner.  Reduced from 6 to 4 mm in v2 since
                                # KNEE_MOUNT_Y_HALF shrank from 22.85 to 16.35,
                                # so the inboard mount holes are now closer to
                                # the beam edge and a 6 mm fillet would crowd
                                # them.  4 mm clears the holes comfortably.

# Cable slot through the link at the tibia cap's cable-port location.
# The tibia cap's -X wall has a 13W x 30H cable port whose bottom edge
# is flush with the link's surface; this slot lets the cable bend down
# through the link to route inboard along the underside.  The slot
# starts at the inboard edge of the knee plate and extends halfway
# across the plate (X span = KNEE_PLATE_X / 2, sitting under the cap's
# -X half).
CABLE_SLOT_H          = 14.0    # Y span of slot (cable connector width)

# Warren-truss lightening -- DISABLED.
# The v1 truss snapped a femur mid-beam through one of the triangle
# cutouts: the triangle apex corners are stress concentrators in the
# exact spot the beam wants to crack under impact loads. A solid beam is
# stronger per gram for a leg link that absorbs landing shocks. The
# code is kept (just gated on N_TRUSS>0) in case a future, well-radiused
# truss is worth reintroducing.
N_TRUSS               = 0      # alternating triangles in the beam (0 = solid)
TRUSS_MARGIN          = 2.0    # min wall between cell edge and next cell
TRUSS_CHORD_MARGIN    = 2.5    # min wall between cell apex/base and beam edge

OUT_DIR  = "linkage_leg"
STL_PATH = OUT_DIR + "/femur_link.stl"
PNG_PATH = OUT_DIR + "/femur_link.png"


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


def chamfered_beam(length, width, thk, chamfer):
    """Octagonal prism: long axis along X, width Y, thickness Z, with
    `chamfer` mm clipped off each of the 4 long edges.  If chamfer == 0,
    returns a plain rectangular box of the same dimensions.

    Built by hand (cross-section verts in the YZ plane, extruded along
    X) so we don't pull in shapely just for this one shape.
    """
    if chamfer <= 0:
        return creation.box(extents=[length, width, thk])

    hw, ht, c = width / 2.0, thk / 2.0, chamfer
    # 8-vertex octagonal cross-section in the YZ plane (CCW looking down +X).
    cs = np.array([
        [ hw,  ht - c],   # right edge, near top
        [ hw - c,  ht],   # top edge, near right
        [-hw + c,  ht],
        [-hw,  ht - c],
        [-hw, -ht + c],
        [-hw + c, -ht],
        [ hw - c, -ht],
        [ hw, -ht + c],
    ])
    n = len(cs)
    hl = length / 2.0
    verts = np.zeros((2 * n, 3))
    for i in range(n):
        verts[i]     = [-hl, cs[i, 0], cs[i, 1]]
        verts[n + i] = [+hl, cs[i, 0], cs[i, 1]]
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])
    for i in range(1, n - 1):
        faces.append([0, i, i + 1])               # cap A (X=-hl)
        faces.append([n, n + i + 1, n + i])       # cap B (X=+hl)
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces))
    mesh.merge_vertices()
    mesh.fix_normals()
    return mesh


def corner_fillet(corner_x, corner_y, radius, thk, sign_y):
    """A solid that fills one inside corner of a 90-deg beam/plate
    junction with a smooth quarter-arc.

    The corner is at (corner_x, corner_y) where the beam edge runs along
    Y = corner_y (for X < corner_x) and the plate inboard edge runs
    along X = corner_x (for Y on the +sign_y side of corner_y).
    sign_y = +1 for the +Y corner, -1 for the -Y corner.

    Built as a `radius x radius` box at the corner with a quarter
    cylinder subtracted at the diagonally opposite point -- what's left
    is a quarter-pipe wedge that fills the inside corner with an arc
    tangent to both the beam edge and the plate inboard edge.
    """
    s = sign_y
    box = creation.box(extents=[radius, radius, thk])
    box.apply_translation(
        [corner_x - radius / 2.0, corner_y + s * radius / 2.0, 0.0])
    cyl = z_cylinder(
        radius, thk + 2.0,
        [corner_x - radius, corner_y + s * radius, 0.0])
    return trimesh.boolean.difference([box, cyl])


# =============================================================
# BUILD
# =============================================================
def build_link():
    """CAD frame: origin at HIP horn axis, on the link's centre plane
    (Z=0 is the mid-plane of the link's thickness).  +X points toward
    tibia.  TIBIA axis at X=LINK_LEN.  In active mode the knee plate
    centre is at X = LINK_LEN - KNEE_TIBIA_OFFSET (inboard of the
    tibia axis, since the tibia cap's horn is offset from its centre)."""

    # ---------------------------------------------------------------
    # POSITIVE solids: hip disc + beam + knee end
    # ---------------------------------------------------------------
    hip_disc = creation.cylinder(
        radius=HIP_DISC_DIA / 2.0, height=LINK_THK, sections=96)

    if KNEE_MODE == "active":
        # Rectangular knee plate centred inboard of tibia axis
        knee_center_x = LINK_LEN - KNEE_TIBIA_OFFSET
        knee_end = creation.box(extents=[KNEE_PLATE_X, KNEE_PLATE_Y, LINK_THK])
        knee_end.apply_translation([knee_center_x, 0.0, 0.0])
        knee_inner_edge = knee_center_x - KNEE_PLATE_X / 2.0
    else:
        # Round knee disc centred at the knee axis
        knee_center_x = LINK_LEN
        knee_end = creation.cylinder(
            radius=KNEE_DISC_DIA / 2.0, height=LINK_THK, sections=96)
        knee_end.apply_translation([knee_center_x, 0.0, 0.0])
        knee_inner_edge = knee_center_x - KNEE_DISC_DIA / 2.0

    # Beam: from hip disc edge to knee-end inner edge, with 3 mm overlap
    # into each so the boolean union is clean.  Chamfered if BEAM_CHAMFER>0.
    beam_x_lo = HIP_DISC_DIA / 2.0 - 3.0
    beam_x_hi = knee_inner_edge + 3.0
    beam = chamfered_beam(
        length=beam_x_hi - beam_x_lo,
        width=BEAM_WIDTH,
        thk=LINK_THK,
        chamfer=BEAM_CHAMFER,
    )
    beam.apply_translation([(beam_x_lo + beam_x_hi) / 2.0, 0.0, 0.0])

    positives = [hip_disc, beam, knee_end]

    # Fillets at the two beam->plate inside corners (active knee only;
    # passive knee is a round disc and has no sharp corner to fillet).
    if KNEE_MODE == "active" and KNEE_FILLET_R > 0:
        for sign_y in (+1, -1):
            positives.append(corner_fillet(
                corner_x=knee_inner_edge,
                corner_y=sign_y * BEAM_WIDTH / 2.0,
                radius=KNEE_FILLET_R,
                thk=LINK_THK,
                sign_y=sign_y,
            ))

    body = trimesh.boolean.union(positives)

    # ---------------------------------------------------------------
    # NEGATIVE cutters
    # ---------------------------------------------------------------
    cutters = []

    # ---- HIP end fasteners ----
    # Central relief for horn screw head
    cutters.append(z_cylinder(
        HORN_CENTRAL_CLEAR / 2.0, LINK_THK + 2.0, [0.0, 0.0, 0.0]))
    # 4 horn bolt holes
    h = HORN_BOLT_SQUARE / 2.0
    for dx, dy in [(+h, +h), (+h, -h), (-h, -h), (-h, +h)]:
        cutters.append(z_cylinder(
            HORN_BOLT_CLEAR / 2.0, LINK_THK + 2.0, [dx, dy, 0.0]))

    # ---- KNEE end fasteners (per mode) ----
    if KNEE_MODE == "passive":
        # Single M4 bushing bore at knee axis (= tibia axis)
        cutters.append(z_cylinder(
            BUSHING_BORE / 2.0, LINK_THK + 2.0, [LINK_LEN, 0.0, 0.0]))
    elif KNEE_MODE == "active":
        # 4 M3 mount holes around knee plate centre.  Pattern matches
        # the tibia cap's flange holes.  Centre is inboard of tibia axis.
        kx_center = LINK_LEN - KNEE_TIBIA_OFFSET
        for dx in (-KNEE_MOUNT_X_HALF, +KNEE_MOUNT_X_HALF):
            for dy in (-KNEE_MOUNT_Y_HALF, +KNEE_MOUNT_Y_HALF):
                cutters.append(z_cylinder(
                    KNEE_BOLT_CLEAR / 2.0, LINK_THK + 2.0,
                    [kx_center + dx, dy, 0.0]))
        # Cable slot through link for the tibia cap's cable exit.
        # Spans from the inboard edge of the knee plate halfway across
        # the plate (i.e. under the cap's -X half, where the cable
        # exits).
        slot_w = KNEE_PLATE_X / 2.0
        slot_inboard = kx_center - KNEE_PLATE_X / 2.0
        slot_x = slot_inboard + slot_w / 2.0
        slot = creation.box(extents=[slot_w, CABLE_SLOT_H, LINK_THK + 2.0])
        slot.apply_translation([slot_x, 0.0, 0.0])
        cutters.append(slot)
    else:
        raise ValueError(f"Unknown KNEE_MODE: {KNEE_MODE!r}")

    # ---- Warren-truss lightening in the beam ----
    # Triangular cells alternating apex up/down, fitting inside the
    # beam between (beam_x_lo + truss_inset) and (beam_x_hi - truss_inset).
    # Guarded so N_TRUSS=0 yields a solid beam (see PARAMETERS note).
    truss_inset = 4.0    # don't cut too close to the discs
    truss_x_lo = beam_x_lo + truss_inset
    truss_x_hi = beam_x_hi - truss_inset
    cell_w = (truss_x_hi - truss_x_lo) / N_TRUSS if N_TRUSS > 0 else 0.0
    chord_y_lo = -BEAM_WIDTH / 2.0 + TRUSS_CHORD_MARGIN
    chord_y_hi = +BEAM_WIDTH / 2.0 - TRUSS_CHORD_MARGIN
    half_margin = TRUSS_MARGIN / 2.0
    truss_cutter_thk = LINK_THK + 2.0
    for i in range(N_TRUSS):
        cell_x0 = truss_x_lo + i * cell_w
        cell_x1 = cell_x0 + cell_w
        base_x0 = cell_x0 + half_margin
        base_x1 = cell_x1 - half_margin
        apex_x  = 0.5 * (base_x0 + base_x1)
        if i % 2 == 0:
            p_a = (base_x0, chord_y_lo)
            p_b = (base_x1, chord_y_lo)
            p_c = (apex_x,  chord_y_hi)
        else:
            p_a = (base_x0, chord_y_hi)
            p_b = (base_x1, chord_y_hi)
            p_c = (apex_x,  chord_y_lo)
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
        f"CRABORA femur link  v1  ({KNEE_MODE} knee)  -  "
        f"{LINK_LEN:.0f} mm axis-to-axis, beam {BEAM_WIDTH:.0f} mm wide, "
        f"{LINK_THK:.0f} mm thick",
        color="#d8e2ec", fontsize=13, y=1.02)
    fig.savefig(path, dpi=120, facecolor=bg, bbox_inches="tight")
    plt.close(fig)


# =============================================================
# MAIN
# =============================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Building femur link v1 ({KNEE_MODE} knee)...")
    link = build_link()
    e = link.extents
    vol = link.volume / 1000.0
    print(f"  bbox {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm   "
          f"vol {vol:.1f} cm^3   watertight {link.is_watertight}")
    print(f"  axis-to-axis: {LINK_LEN:.1f} mm (hip horn -> knee)")
    chamfer_note = (f"chamfer {BEAM_CHAMFER:.1f} mm" if BEAM_CHAMFER > 0
                    else "no chamfer")
    print(f"  beam        : {BEAM_WIDTH:.0f} wide x {LINK_THK:.0f} thick mm,  "
          f"{N_TRUSS} truss cells,  {chamfer_note}")
    print(f"  hip         : Φ{HIP_DISC_DIA:.0f} disc, 4 M3 horn bolts on "
          f"{HORN_BOLT_SQUARE:.0f} mm square + Φ{HORN_CENTRAL_CLEAR:.0f} central relief")
    if KNEE_MODE == "passive":
        print(f"  knee (PASSIVE): Φ{KNEE_DISC_DIA:.0f} disc, Φ{BUSHING_BORE:.1f} bushing bore at axis")
    else:
        print(f"  knee (ACTIVE) : {KNEE_PLATE_X:.0f}x{KNEE_PLATE_Y:.0f} plate centred "
              f"{KNEE_TIBIA_OFFSET:.1f} mm inboard of tibia axis")
        print(f"                  4 M3 mount holes at +/-{KNEE_MOUNT_X_HALF:.1f}, "
              f"+/-{KNEE_MOUNT_Y_HALF:.2f} from plate centre  "
              f"(Φ{KNEE_BOLT_CLEAR:.1f} clear)")
        slot_w = KNEE_PLATE_X / 2.0
        slot_inboard = (LINK_LEN - KNEE_TIBIA_OFFSET) - KNEE_PLATE_X / 2.0
        print(f"                  cable slot {slot_w:.1f}x{CABLE_SLOT_H:.0f} mm  "
              f"(X={slot_inboard:.1f}..{slot_inboard + slot_w:.1f}, halfway across plate)")
        fillet_note = (f"R={KNEE_FILLET_R:.1f} mm" if KNEE_FILLET_R > 0 else "off")
        print(f"                  beam->plate inside-corner fillets: {fillet_note}")

    link.export(STL_PATH)
    print(f"✓ wrote {STL_PATH}")
    print("Rendering preview...")
    render_png(link, PNG_PATH)
    print(f"✓ wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
