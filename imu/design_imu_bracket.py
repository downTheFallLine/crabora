"""
CRABORA IMU bracket -- C-section sled for the MPU-6050
=======================================================

A C in profile, lying on its back:

      ___________________          <- TOP plate: the IMU bolts here
     |                   |            2x M3 clearance, nut underneath
     |___________        |
                 |       |
      ___________|       |         <- WEB: sets the wiring clearance
     |___________________|         <- BASE: glued to the deck

The open side faces forward, so a nut can be started on a bolt dropped
through the top plate and the IMU's wiring can leave underneath without
being pinched. The web is at the back for the same reason: nothing to
reach past when fitting it.

WHY THE ARROW MATTERS MORE THAN THE BRACKET

Two bolt holes cannot key rotation. The board will fit perfectly
mounted 180 degrees out, +X pointing aft instead of forward, and
NOTHING WILL LOOK WRONG. The sensor answers, the test passes if you run
it after mounting it backwards, and every heading the robot computes is
inverted.

So the top plate carries an engraved arrow, forward of the board
footprint where the PCB cannot hide it. Point it at the hexagon vertex
between legs 1 and 6 and the body frame is right:

    +X forward   +Y left   +Z up

MOUNTING

Rigid, deliberately. Foam or tape would isolate the sensor from exactly
the body motion it exists to measure -- the 13 mm of sag that the joint
encoders could not see (2026-10-13) is the whole reason it is here.
Glue the base flat to the deck, as close to the body centre as the
cabling allows, so a turn shows as rotation rather than as linear
acceleration on a lever arm.

Print flat on the bed, base down, no supports. The web and top plate
are a simple step up; the overhang under the top plate is bridged over
the open side, which prints cleanly at this span.

Usage:
  python design_imu_bracket.py
  python design_imu_bracket.py --height 12 --hole-spacing 15.4
  python design_imu_bracket.py --nut-trap
  python design_imu_bracket.py --summary        # dimensions only, no STL
"""

import argparse
import math
import os

import numpy as np
import trimesh
from trimesh import creation


# =============================================================
# PARAMETERS  --  ⚠ CONFIRM THE FIRST FOUR WITH CALIPERS
# =============================================================
# These are typical GY-521 figures. Boards vary, and a wrong hole
# spacing is a reprint, so measure before committing filament.

BOARD_L = 20.0        # PCB length, mm  (along +X / forward)   MEASURED
BOARD_W = 16.0        # PCB width, mm   (along +Y / left)      MEASURED
# Measured OUTSIDE-to-outside, which is how you can actually get a
# caliper on it. Centres are that minus one hole diameter.
HOLE_OUTSIDE = 17.5   # outer edge to outer edge, mm            MEASURED
PCB_HOLE_DIA = 3.0    # the holes in the PCB itself
HOLE_SPACING = HOLE_OUTSIDE - PCB_HOLE_DIA   # -> 14.5 mm centres

# The holes run along the board's LENGTH, not its width: 17.5 mm of span
# does not fit across a 16 mm board. So the two bolts sit fore and aft.
C_HEIGHT = 35.0       # clear gap under the board, for the wiring
                      # (given by Tom 2026-10-13)

# Below here are the design's own numbers, not measurements.
BASE_THK = 3.0        # glued face: thick enough not to curl
TOP_THK = 3.0         # carries the bolt load
WEB_THK = 6.0         # the only thing resisting the cantilever.
                      # At 35 mm tall this sets the resonance:
                      # 4 mm -> 309 Hz, 6 mm -> 568 Hz, 8 mm -> 874 Hz.
                      # A foot plant rings the structure at roughly
                      # 50-200 Hz, so 6 mm keeps the mount well clear
                      # of it. A bracket that resonates reports its own
                      # wobble as body motion, which is the one thing
                      # this sensor must not do.
MARGIN = 2.5          # plate material beyond the board footprint
M3_CLEAR = 3.4        # clearance for an M3 bolt
M3_NUT_AF = 6.2       # M3 nut across flats, plus a little
M3_NUT_H = 2.6        # M3 nut thickness, plus a little
MIN_OVER_NUT = 2.0    # material that must remain above a trapped nut
FILLET = 1.2          # web-to-plate fillet leg, mm
ARROW_DEPTH = 0.6     # engraving depth
ARROW_LEN = 7.0


def rounded_fillet(size_x, size_y, leg):
    """A triangular prism, used as a fillet along the web joints."""
    tri = np.array([[0, 0], [leg, 0], [0, leg]], dtype=float)
    return creation.extrude_triangulation(
        vertices=tri, faces=np.array([[0, 1, 2]]), height=size_y)


def arrow_prism(length, width, depth):
    """A flat arrow pointing +X, as a solid to subtract."""
    head = length * 0.45
    shaft_w = width * 0.34
    pts = np.array([
        [length, 0.0],
        [length - head,  width / 2],
        [length - head,  shaft_w / 2],
        [0.0,            shaft_w / 2],
        [0.0,           -shaft_w / 2],
        [length - head, -shaft_w / 2],
        [length - head, -width / 2],
    ])
    poly = trimesh.path.polygons.Polygon(pts)
    return creation.extrude_polygon(poly, height=depth)


def build(board_l, board_w, hole_spacing, c_height, nut_trap):
    # A nut pocket 2.6 mm deep in a 3 mm plate leaves 0.4 mm of PLA
    # carrying the entire clamp load, which crushes on first tightening.
    # So the plate grows to suit when a trap is asked for.
    top_thk = TOP_THK if not nut_trap else M3_NUT_H + MIN_OVER_NUT
    # web + margin + board + gap + arrow + margin. The web thickness
    # has to be in here: the board starts AFTER the web, not after the
    # margin, and leaving it out ran the arrow off the front edge.
    ARROW_GAP = 2.0
    top_l = (WEB_THK + MARGIN + board_l + ARROW_GAP
             + ARROW_LEN + MARGIN)
    # Wide enough for the board OR the bolt span, whichever needs more.
    top_w = max(board_w + 2 * MARGIN,
                hole_spacing + M3_CLEAR + 2 * MARGIN)
    # A tall bracket levers on its glue joint, so the base is kept
    # long: the taller the web, the more the moment tries to peel it.
    base_l = max(top_l * 0.75, c_height * 0.9)
    base_w = top_w

    parts = []

    # --- BASE: glued to the deck, at the back ---------------------
    base = creation.box(extents=(base_l, base_w, BASE_THK))
    base.apply_translation((base_l / 2, 0, BASE_THK / 2))
    parts.append(base)

    # --- WEB: the riser, at the back edge -------------------------
    web_h = c_height + BASE_THK
    web = creation.box(extents=(WEB_THK, base_w, web_h))
    web.apply_translation((WEB_THK / 2, 0, web_h / 2))
    parts.append(web)

    # --- TOP: cantilevered forward over the open side -------------
    top = creation.box(extents=(top_l, top_w, top_thk))
    top.apply_translation((top_l / 2, 0, web_h + top_thk / 2))
    parts.append(top)

    # --- fillets at both web joints -------------------------------
    for z, flip in ((BASE_THK, False), (web_h, True)):
        f = rounded_fillet(FILLET, base_w, FILLET)
        f.apply_transform(trimesh.transformations.rotation_matrix(
            math.pi / 2, [1, 0, 0]))
        f.apply_translation((WEB_THK, base_w / 2, z))
        if flip:
            f.apply_transform(trimesh.transformations.reflection_matrix(
                [0, 0, web_h], [0, 0, 1]))
        parts.append(f)

    solid = trimesh.boolean.union(parts)

    # --- bolt holes through the top plate -------------------------
    board_cx = WEB_THK + MARGIN + board_l / 2
    cuts = []
    for dy in (-hole_spacing / 2, hole_spacing / 2):
        h = creation.cylinder(radius=M3_CLEAR / 2, height=top_thk * 3)
        h.apply_translation((board_cx, dy, web_h + top_thk / 2))
        cuts.append(h)
        if nut_trap:
            # Hex pocket in the UNDERSIDE, so the nut is captive and you
            # only need one spanner. Open side still lets you drop it in.
            nut = creation.cylinder(radius=M3_NUT_AF / math.sqrt(3),
                                    height=M3_NUT_H, sections=6)
            nut.apply_translation((board_cx, dy, web_h + M3_NUT_H / 2))
            cuts.append(nut)

    # --- the arrow, forward of the board footprint ----------------
    arrow = arrow_prism(ARROW_LEN, top_w * 0.5, ARROW_DEPTH * 3)
    arrow.apply_translation(
        (WEB_THK + MARGIN + board_l + ARROW_GAP, 0,
         web_h + top_thk - ARROW_DEPTH))
    cuts.append(arrow)

    solid = trimesh.boolean.difference([solid] + cuts)
    return solid, dict(top_l=top_l, top_w=top_w, base_l=base_l,
                       base_w=base_w, web_h=web_h, board_cx=board_cx,
                       overall_h=web_h + top_thk, top_thk=top_thk)


def preview(solid, path, title):
    """Multi-panel PNG beside the STL, as the other design scripts do.

    Worth the few lines: a bracket that is subtly inside out or has the
    arrow on the wrong face is obvious in a picture and invisible in a
    dimensions table.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    views = (("iso", 24, -58), ("front (open side)", 2, 0),
             ("side", 2, -90), ("top", 89, -90))
    fig = plt.figure(figsize=(13, 3.4))
    tris = solid.vertices[solid.faces]
    for i, (name, elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(1, len(views), i, projection="3d")
        ax.add_collection3d(Poly3DCollection(
            tris, facecolor="#9ec5ff", edgecolor="#2b4a7a", linewidth=0.12))
        lo, hi = solid.bounds
        span = (hi - lo).max() / 2
        mid = (hi + lo) / 2
        for setter, m in ((ax.set_xlim, mid[0]), (ax.set_ylim, mid[1]),
                          (ax.set_zlim, mid[2])):
            setter(m - span, m + span)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(name, fontsize=9)
        ax.set_axis_off()
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="C-section IMU bracket")
    ap.add_argument("--board-l", type=float, default=BOARD_L)
    ap.add_argument("--board-w", type=float, default=BOARD_W)
    ap.add_argument("--hole-spacing", type=float, default=HOLE_SPACING,
                    help="M3 hole centres, centre-to-centre (mm)")
    ap.add_argument("--height", type=float, default=C_HEIGHT,
                    help="clear gap under the board for wiring (mm)")
    ap.add_argument("--nut-trap", action="store_true",
                    help="hex pocket under the top plate to capture the nut")
    ap.add_argument("--summary", action="store_true",
                    help="print dimensions without writing an STL")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    if args.hole_spacing >= args.board_l:
        ap.error(f"hole spacing {args.hole_spacing:g} mm does not fit along a "
                 f"{args.board_l:g} mm board -- check which axis you measured")

    solid, d = build(args.board_l, args.board_w, args.hole_spacing,
                     args.height, args.nut_trap)

    print("CRABORA IMU bracket")
    print(f"  board            {args.board_l:g} x {args.board_w:g} mm")
    print(f"  M3 holes         {args.hole_spacing:g} mm apart, "
          f"{M3_CLEAR:g} mm clearance"
          + (", with nut trap" if args.nut_trap else ""))
    print(f"  wiring gap       {args.height:g} mm")
    print()
    print(f"  top plate        {d['top_l']:.1f} x {d['top_w']:.1f} x "
          f"{d['top_thk']:g} mm"
          + (f"  (thickened for the nut trap)" if args.nut_trap else ""))
    print(f"  base plate       {d['base_l']:.1f} x {d['base_w']:.1f} x {BASE_THK:g} mm")
    print(f"  web              {WEB_THK:g} mm thick, {d['web_h']:.1f} mm tall")
    print(f"  overall height   {d['overall_h']:.1f} mm")
    print(f"  glue area        {d['base_l'] * d['base_w'] / 100:.1f} cm²")
    print()
    I = d["base_w"] * WEB_THK ** 3 / 12
    k = 3 * 3500.0 * I / d["web_h"] ** 3
    f_n = (1 / (2 * math.pi)) * math.sqrt((k * 1000) / 0.009)
    print(f"  resonance        ~{f_n:.0f} Hz "
          f"(a foot plant rings at 50-200 Hz)")
    print(f"  watertight       {solid.is_watertight}")
    print(f"  volume           {solid.volume / 1000:.1f} cm³")

    if args.summary:
        return 0
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "imu_bracket.stl")
    solid.export(out)
    png = os.path.splitext(out)[0] + ".png"
    preview(solid, png, f"IMU bracket -- {args.height:g} mm gap, "
                        f"M3 at {args.hole_spacing:g} mm")
    print(f"\n  wrote {out}")
    print(f"  wrote {png}")
    print("  Print base-down, no supports. Point the arrow FORWARD --")
    print("  at the hexagon vertex between legs 1 and 6.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
