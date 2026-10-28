"""
CRABORA lid -- dimensioned OmniGraffle drawing
==============================================

Draws lid_drawing.graffle (and lid_drawing.pdf) from the numbers in
design_lid.py, so the drawing can't drift from the STLs. One canvas:

  PLAN        top view: outline, seams, hidden caps and pucks
  SECTION A-A half-section through a leg: profile, lips, cap clearance
  DETAIL B    magnet puck in its socket
  table       parts, print sizes, masses, hardware, notes

It writes an Omni Automation script and runs it in OmniGraffle 7 through
osascript, in a new document, then saves and exports. macOS only; the
first run asks to let the terminal control OmniGraffle.

    ~/.pyenv/shims/python3 draw_lid_omnigraffle.py
"""

import datetime
import json
import math
import os
import subprocess
import tempfile

import design_lid as L
from design_flanged_deck import (CAP_EDGE_SETBACK, CAP_LENGTH, DECK_APOTHEM,
                                 LID_PUCK_X, LID_PUCK_Y, to_print_pose)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
GRAFFLE = os.path.join(OUT_DIR, 'lid_drawing.graffle')
PDF = os.path.join(OUT_DIR, 'lid_drawing.pdf')

PAGE = (1760, 1120)          # points
S_PLAN, S_SEC, S_DET = 1.25, 2.6, 8.0   # points per mm in each view

TAN30 = math.tan(math.radians(30))
COS30 = math.cos(math.radians(30))

PRELUDE = r"""
var c = canvases[0];
c.canvasSizingMode = CanvasSizingMode.Fixed;
c.size = new Size(%d, %d);
var INK = Color.RGB(0.10, 0.12, 0.15), DIM = Color.RGB(0.13, 0.36, 0.66),
    HID = Color.RGB(0.50, 0.50, 0.52), LID = Color.RGB(0.74, 0.83, 0.93),
    DECK = Color.RGB(0.86, 0.86, 0.86), PUCK = Color.RGB(0.95, 0.74, 0.55),
    MAG = Color.RGB(0.55, 0.20, 0.20), STEEL = Color.RGB(0.70, 0.70, 0.72);
function P(a) { return new Point(a[0], a[1]); }
function pl(pts, w, col, dash, closed) {
  var l = c.newLine(); var q = pts.map(P); if (closed) q.push(q[0]);
  l.points = q; l.strokeThickness = w; l.strokeColor = col; l.shadowColor = null;
  if (dash) l.strokePattern = StrokeDash[dash];
  return l;
}
function poly(pts, fill, w) {
  var xs = pts.map(function (p) { return p[0]; }), ys = pts.map(function (p) { return p[1]; });
  var x0 = Math.min.apply(null, xs), y0 = Math.min.apply(null, ys);
  var s = c.addShape("Rectangle", new Rect(x0, y0, Math.max.apply(null, xs) - x0,
                                            Math.max.apply(null, ys) - y0));
  s.shapeVertices = pts.map(P); s.fillColor = fill; s.strokeColor = INK;
  s.strokeThickness = w; s.shadowColor = null; return s;
}
function circ(cx, cy, r, w, col, dash, fill) {
  var s = c.addShape("Circle", new Rect(cx - r, cy - r, 2 * r, 2 * r));
  s.strokeThickness = w; s.strokeColor = col; s.fillColor = fill || null;
  s.shadowColor = null; if (dash) s.strokePattern = StrokeDash[dash]; return s;
}
function txt(str, x, y, size, col, rot, opts) {
  opts = opts || {};
  var t = c.addText(str, new Point(x, y));
  t.textSize = size; t.textColor = col;
  if (opts.bold) t.fontName = "HelveticaNeue-Bold";
  if (opts.mono) t.fontName = "Menlo-Regular";
  var g = t.geometry, ax = opts.align || "c";
  var x0 = ax == "l" ? x : ax == "r" ? x - g.width : x - g.width / 2;
  t.geometry = new Rect(x0, y - g.height / 2, g.width, g.height);
  if (ax == "l") t.textHorizontalAlignment = HorizontalTextAlignment.Left;
  if (rot) t.rotation = rot;
  return t;
}
function dim(x1, y1, x2, y2, off, label) {
  var dx = x2 - x1, dy = y2 - y1, L = Math.sqrt(dx * dx + dy * dy);
  var ux = dx / L, uy = dy / L, nx = -uy, ny = ux, s = off >= 0 ? 1 : -1;
  var a1 = [x1 + nx * off, y1 + ny * off], a2 = [x2 + nx * off, y2 + ny * off];
  if (Math.abs(off) > 1) {
    pl([[x1 + nx * s * 3, y1 + ny * s * 3], [x1 + nx * (off + s * 4), y1 + ny * (off + s * 4)]], 0.4, DIM);
    pl([[x2 + nx * s * 3, y2 + ny * s * 3], [x2 + nx * (off + s * 4), y2 + ny * (off + s * 4)]], 0.4, DIM);
  }
  var l = pl([a1, a2], 0.6, DIM); l.headType = "FilledArrow"; l.tailType = "FilledArrow";
  var ang = Math.atan2(dy, dx) * 180 / Math.PI;
  if (ang > 90) ang -= 180; if (ang <= -90) ang += 180;
  var side = s, lx = (a1[0] + a2[0]) / 2 + nx * side * 9, ly = (a1[1] + a2[1]) / 2 + ny * side * 9;
  txt(label, lx, ly, 10, DIM, ang);
}
function leader(tx, ty, px, py, label, align) {
  var l = pl([[tx, ty], [px, py]], 0.5, DIM); l.headType = "FilledArrow";
  txt(label, align == "r" ? tx - 3 : tx + 3, ty, 9.5, DIM, 0, {align: align || "l"});
}
"""


class Sheet:
    """Collects JS calls in page coordinates."""

    def __init__(self):
        self.js = [PRELUDE % PAGE]

    def pl(self, pts, w=0.8, col='INK', dash=None, closed=False):
        self.js.append('pl(%s, %.2f, %s, %s, %s);' % (
            json.dumps([[round(x, 2), round(y, 2)] for x, y in pts]), w, col,
            json.dumps(dash), 'true' if closed else 'false'))

    def poly(self, pts, fill, w=0.8):
        self.js.append('poly(%s, %s, %.2f);' % (
            json.dumps([[round(x, 2), round(y, 2)] for x, y in pts]), fill, w))

    def circ(self, c, r, w=0.8, col='INK', dash=None, fill=None):
        self.js.append('circ(%.2f, %.2f, %.2f, %.2f, %s, %s, %s);' % (
            c[0], c[1], r, w, col, json.dumps(dash), fill or 'null'))

    def txt(self, s, p, size=10, col='INK', rot=0, **opts):
        self.js.append('txt(%s, %.2f, %.2f, %.1f, %s, %.1f, %s);' % (
            json.dumps(s), p[0], p[1], size, col, rot, json.dumps(opts)))

    def dim(self, p1, p2, off, label):
        self.js.append('dim(%.2f, %.2f, %.2f, %.2f, %.2f, %s);' % (*p1, *p2, off, json.dumps(label)))

    def leader(self, t, p, label, align='l'):
        self.js.append('leader(%.2f, %.2f, %.2f, %.2f, %s, %s);' % (*t, *p, json.dumps(label),
                                                                   json.dumps(align)))


def hexagon_mm(a):
    R = a / COS30
    return [(R * math.cos(math.radians(60 * k)), R * math.sin(math.radians(60 * k)))
            for k in range(6)]


def fmt(v):
    return ('%.1f' % v).rstrip('0').rstrip('.')


# =============================================================
# VIEWS
# =============================================================
def plan(sh, ox, oy):
    S = S_PLAN

    def P(x, y):
        return (ox + x * S, oy - y * S)

    sh.txt('PLAN  —  top view', (ox, oy - 285), 14, bold=True)
    R_out = DECK_APOTHEM / COS30
    sh.pl([P(-R_out - 12, 0), P(R_out + 12, 0)], 0.4, 'HID', 'Dash8on4off1on4off')
    sh.pl([P(0, -DECK_APOTHEM - 12), P(0, DECK_APOTHEM + 12)], 0.4, 'HID', 'Dash8on4off1on4off')

    # Hidden: coxa caps and magnet pucks.
    for k in range(6):
        th = math.radians(30 + 60 * k)
        rad, tan = (math.cos(th), math.sin(th)), (-math.sin(th), math.cos(th))
        r0 = DECK_APOTHEM - CAP_EDGE_SETBACK - CAP_LENGTH
        corners = [(r * rad[0] + t * tan[0], r * rad[1] + t * tan[1])
                   for r, t in ((r0, -L.CAP_WIDTH / 2), (r0, L.CAP_WIDTH / 2),
                                (r0 + CAP_LENGTH, L.CAP_WIDTH / 2), (r0 + CAP_LENGTH, -L.CAP_WIDTH / 2))]
        sh.pl([P(*p) for p in corners], 0.6, 'HID', 'Dash4on4off', closed=True)
        a = th - math.radians(90)
        px = LID_PUCK_X * math.cos(a) - LID_PUCK_Y * math.sin(a)
        py = LID_PUCK_X * math.sin(a) + LID_PUCK_Y * math.cos(a)
        sh.circ(P(px, py), L.PUCK_DIA / 2 * S, 0.6, 'HID', 'Dash4on4off')

    # Visible: outline, fold/seam rings, radial seams, grille.
    sh.pl([P(*p) for p in hexagon_mm(DECK_APOTHEM)], 1.8, closed=True)
    sh.pl([P(*p) for p in hexagon_mm(L.TOP_APOTHEM)], 0.9, closed=True)
    sh.pl([P(*p) for p in hexagon_mm(L.HUB_APOTHEM)], 0.9, closed=True)
    for (x0, y0), (x1, y1) in zip(hexagon_mm(L.HUB_APOTHEM), hexagon_mm(DECK_APOTHEM)):
        sh.pl([P(x0, y0), P(x1, y1)], 0.9)
    sh.circ(P(0, 0), L.GRILLE_RADIUS * S, 0.6)

    # Cutting plane A-A: centre out through leg 0 (+Y).
    sh.pl([P(0, -8), P(0, DECK_APOTHEM + 22)], 1.6, 'INK', 'Dash8on4off1on4off')
    for y in (-8, DECK_APOTHEM + 22):
        sh.js.append('var l = pl(%s, 1.4, INK); l.headType = "FilledArrow";'
                     % json.dumps([list(P(0, y)), list(P(-14, y))]))
        sh.txt('A', P(-20, y), 13, bold=True)

    # Dimensions.
    Rh = L.TOP_APOTHEM / COS30
    sh.dim(P(-DECK_APOTHEM * TAN30, -DECK_APOTHEM), P(-DECK_APOTHEM * TAN30, DECK_APOTHEM),
           -(R_out - DECK_APOTHEM * TAN30 + 18) * S, '%s across flats' % fmt(2 * DECK_APOTHEM))
    sh.dim(P(-R_out, 0), P(R_out, 0), (DECK_APOTHEM + 22) * S,
           '%s across corners' % fmt(2 * R_out))
    sh.dim(P(L.TOP_APOTHEM * TAN30, -L.TOP_APOTHEM), P(L.TOP_APOTHEM * TAN30, L.TOP_APOTHEM),
           (R_out + 14 - L.TOP_APOTHEM * TAN30) * S, '%s flat top' % fmt(2 * L.TOP_APOTHEM))
    sh.dim(P(L.HUB_APOTHEM * TAN30, -L.HUB_APOTHEM), P(L.HUB_APOTHEM * TAN30, L.HUB_APOTHEM),
           (R_out + 36 - L.HUB_APOTHEM * TAN30) * S, '%s hub' % fmt(2 * L.HUB_APOTHEM))
    # Puck location on the +Y edge.
    xc = DECK_APOTHEM * TAN30
    sh.pl([P(LID_PUCK_X, LID_PUCK_Y - 9), P(LID_PUCK_X, DECK_APOTHEM + 3)], 0.4, 'HID',
          'Dash8on4off1on4off')
    sh.dim(P(LID_PUCK_X, DECK_APOTHEM), P(xc, DECK_APOTHEM), -14 * S,
           fmt(xc - LID_PUCK_X))
    sh.dim(P(LID_PUCK_X, LID_PUCK_Y), P(LID_PUCK_X, DECK_APOTHEM), -12 * S,
           fmt(DECK_APOTHEM - LID_PUCK_Y))

    # Callouts.
    sh.leader(P(-232, 150), P(-60, DECK_APOTHEM - 2), 'outer piece ×6:\nwall + slope', 'r')
    sh.leader(P(-232, 105), P(-L.HUB_APOTHEM * TAN30 - 12, 95), 'mid piece ×6', 'r')
    sh.leader(P(-232, -150), P(-150, -86.6 + 0), 'radial seam ×6\n(at the corners)', 'r')
    sh.leader(P(-232, -105), P(-L.TOP_APOTHEM * TAN30 - 3, -L.TOP_APOTHEM + 3),
              'fold: slope → flat top\n= seam, r %s' % fmt(L.TOP_APOTHEM), 'r')
    sh.leader(P(-232, 60), P(-L.HUB_APOTHEM / COS30 + 4, 0), 'hub seam, r %s' % fmt(L.HUB_APOTHEM), 'r')
    sh.leader(P(-232, -40), P(-L.GRILLE_RADIUS * 0.7, -L.GRILLE_RADIUS * 0.7),
              'speaker / mic grille\nØ%s' % fmt(2 * L.GRILLE_RADIUS), 'r')
    sh.leader(P(140, 230), P(LID_PUCK_X + 4, LID_PUCK_Y + 4),
              'magnet puck ×6 (hidden)\nØ%s, see DETAIL B' % fmt(L.PUCK_DIA))
    sh.leader(P(-140, 230), P(-12, DECK_APOTHEM - 20),
              'coxa cap ×6 (hidden)\n%s × %s' % (fmt(L.CAP_WIDTH), fmt(CAP_LENGTH)), 'r')


def section(sh, ox, oy):
    S = S_SEC

    def Q(r, z):
        return (ox + r * S, oy - z * S)

    sh.txt('SECTION  A–A  —  through a leg (vents not shown)', (ox + 240, oy - 230), 14, bold=True)
    sh.pl([Q(0, -14), Q(0, L.TOP_H + 14)], 0.4, 'HID', 'Dash8on4off1on4off')
    sh.txt('℄', Q(0, L.TOP_H + 18), 11, 'HID')

    # Deck plate and coxa cap, for reference.
    sh.poly([Q(0, 0), Q(DECK_APOTHEM, 0), Q(DECK_APOTHEM, -5), Q(0, -5)], 'DECK', 0.6)
    r0 = DECK_APOTHEM - CAP_EDGE_SETBACK - CAP_LENGTH
    sh.pl([Q(r0, 0), Q(r0, L.CAP_HEIGHT), Q(r0 + CAP_LENGTH, L.CAP_HEIGHT), Q(r0 + CAP_LENGTH, 0)],
          0.8, 'HID', 'Dash4on4off')
    sh.txt('coxa cap (ref)', Q(r0 + CAP_LENGTH / 2, L.CAP_HEIGHT / 2), 10, 'HID')
    sh.txt('deck plate (ref)', Q(60, -2.5), 9, 'HID')

    # Lid profile, cut.
    wi = L.WALL_IN
    sh.poly([Q(0, L.TOP_H), Q(L.TOP_APOTHEM, L.TOP_H), Q(DECK_APOTHEM, L.WALL_H),
             Q(DECK_APOTHEM, 0), Q(wi, 0), Q(wi, L.ceiling(wi)),
             Q(L.H_KNEE, L.TOP_H - L.SKIN), Q(0, L.TOP_H - L.SKIN)], 'LID', 0.8)

    # Seam lips and their bolts.
    def lips(r):
        for a, b in ((r - L.LIP_THK, r), (r, r + L.LIP_THK)):
            top_a, top_b = L.ceiling(a), L.ceiling(b)
            sh.poly([Q(a, top_a), Q(b, top_b), Q(b, top_b - L.LIP_DEPTH),
                     Q(a, top_a - L.LIP_DEPTH)], 'LID', 0.6)
        zb = L.ceiling(r + L.LIP_THK) - L.LIP_DEPTH / 2
        sh.poly([Q(r - L.LIP_THK - 2.5, zb + 2.75), Q(r - L.LIP_THK, zb + 2.75),
                 Q(r - L.LIP_THK, zb - 2.75), Q(r - L.LIP_THK - 2.5, zb - 2.75)], 'STEEL', 0.5)
        sh.poly([Q(r - L.LIP_THK, zb + 1.5), Q(r + L.LIP_THK + 2.4, zb + 1.5),
                 Q(r + L.LIP_THK + 2.4, zb - 1.5), Q(r - L.LIP_THK, zb - 1.5)], 'STEEL', 0.5)
        sh.poly([Q(r + L.LIP_THK, zb + 2.75), Q(r + L.LIP_THK + 2.4, zb + 2.75),
                 Q(r + L.LIP_THK + 2.4, zb - 2.75), Q(r + L.LIP_THK, zb - 2.75)], 'STEEL', 0.5)
        return zb
    zb = lips(L.TOP_APOTHEM)
    lips(L.HUB_APOTHEM)

    # Dimensions.
    sh.dim(Q(0, 0), Q(0, L.TOP_H), -22, fmt(L.TOP_H))
    sh.dim(Q(DECK_APOTHEM, 0), Q(DECK_APOTHEM, L.WALL_H), 22, fmt(L.WALL_H))
    sh.dim(Q(0, L.TOP_H), Q(L.TOP_APOTHEM, L.TOP_H), -18, 'r %s' % fmt(L.TOP_APOTHEM))
    sh.dim(Q(L.TOP_APOTHEM, L.TOP_H), Q(DECK_APOTHEM, L.TOP_H), -18,
           fmt(DECK_APOTHEM - L.TOP_APOTHEM))
    sh.dim(Q(0, -5), Q(DECK_APOTHEM, -5), 22, 'r %s  (deck edge)' % fmt(DECK_APOTHEM))
    sh.dim(Q(0, -5), Q(L.HUB_APOTHEM, -5), 44 + 0, 'r %s' % fmt(L.HUB_APOTHEM))
    sh.dim(Q(r0, 0), Q(r0, L.CAP_HEIGHT), -16, fmt(L.CAP_HEIGHT))
    rc = DECK_APOTHEM - CAP_EDGE_SETBACK
    sh.dim(Q(rc, L.CAP_HEIGHT), Q(rc, L.ceiling(rc)), -14, fmt(L.ceiling(rc) - L.CAP_HEIGHT))
    sh.dim(Q(rc, 18), Q(wi, 18), 0, '')
    sh.leader(Q(rc - 32, 26), Q(rc + 1.5, 18), '%s gap' % fmt(wi - rc), 'r')

    # Slope angle.
    ang = math.atan(L.SLOPE_TAN)
    cx, cy = Q(DECK_APOTHEM, L.WALL_H)
    sh.pl([(cx, cy), (cx - 120, cy)], 0.4, 'DIM', 'Dash4on4off')
    arc = [(cx - 100 * math.cos(t), cy - 100 * math.sin(t))
           for t in [ang * i / 12 for i in range(13)]]
    sh.pl(arc, 0.6, 'DIM')
    sh.txt('%.1f°' % math.degrees(ang), (cx - 112, cy - 10), 10, 'DIM')

    # Callouts.
    # Callouts from the empty interior, so nothing crowds the dimensions.
    sh.leader(Q(14, 40), Q(L.TOP_APOTHEM - 1, zb - 3),
              'seam lips %s × %s, M3×8 + nyloc' % (fmt(L.LIP_THK), fmt(L.LIP_DEPTH)))
    sh.leader(Q(14, 32), Q(100, L.TOP_H - L.SKIN / 2), 'slope / top skin %s' % fmt(L.SKIN))
    sh.leader(Q(14, 24), Q(wi + L.WALL_THK / 2, 10), 'wall %s thick' % fmt(L.WALL_THK))


def detail(sh, ox, oy):
    S = S_DET

    def D(x, z):
        return (ox + x * S, oy - z * S)

    sh.txt('DETAIL B  —  magnet puck in its socket', (ox + 40, oy - 300), 14, bold=True)
    sh.pl([D(0, -12), D(0, 33)], 0.4, 'HID', 'Dash8on4off1on4off')

    hb = L.SEAM_BOLT_DIA / 2
    # Deck plate with the M3 hole.
    sh.poly([D(-16, 0), D(-hb, 0), D(-hb, -5), D(-16, -5)], 'DECK', 0.6)
    w_wall = L.WALL_IN - LID_PUCK_Y
    sh.poly([D(hb, 0), D(w_wall + L.WALL_THK + 2, 0), D(w_wall + L.WALL_THK + 2, -5), D(hb, -5)],
            'DECK', 0.6)
    sh.txt('deck', D(-12, -2.5), 9, 'HID')

    # Puck, two halves either side of the bolt hole.
    ph, fl, hd = L.PUCK_H, L.PUCK_FLOOR, L.PUCK_HEAD_DIA / 2
    mp = L.MAGNET_POCKET_DIA / 2
    for s in (-1, 1):
        sh.poly([D(s * hb, 0), D(s * L.PUCK_DIA / 2, 0), D(s * L.PUCK_DIA / 2, ph),
                 D(s * mp, ph), D(s * mp, fl + L.PUCK_HEAD_DEPTH), D(s * hd, fl + L.PUCK_HEAD_DEPTH),
                 D(s * hd, fl), D(s * hb, fl)], 'PUCK', 0.6)

    # Lid boss, web and wall, cut through the puck centre.
    r = L.SOCKET_DIA / 2 + L.BOSS_WALL
    rs, sd = L.SOCKET_DIA / 2, L.SOCKET_DEPTH
    t60 = math.tan(math.radians(L.BOSS_CONE_DEG))
    apex = L.BOSS_TOP + r * t60
    xi = (apex - L.BOSS_TOP) / (2 * t60)           # cone meets the web ramp
    wall_top = 36.0                                  # break line
    sh.poly([D(-r, 0), D(-r, L.BOSS_TOP), D(0, apex), D(xi, L.BOSS_TOP + xi * t60),
             D(w_wall, L.BOSS_TOP + w_wall * t60), D(w_wall, wall_top),
             D(w_wall + L.WALL_THK, wall_top), D(w_wall + L.WALL_THK, 0),
             D(rs, 0), D(rs, sd), D(mp, sd), D(mp, sd + L.MAGNET_POCKET_DEPTH),
             D(-mp, sd + L.MAGNET_POCKET_DEPTH), D(-mp, sd), D(-rs, sd), D(-rs, 0)], 'LID', 0.8)
    sh.pl([D(w_wall - 1.5, wall_top), D(w_wall + L.WALL_THK + 1.5, wall_top)], 0.6, 'INK',
          'Dash4on4off')

    # Magnets, bolt, washer, nut.
    m, mt = L.MAGNET_DIA / 2, L.MAGNET_THK
    z_mag = ph - L.MAGNET_POCKET_DEPTH
    sh.poly([D(-m, z_mag), D(m, z_mag), D(m, z_mag + mt), D(-m, z_mag + mt)], 'MAG', 0.5)
    sh.poly([D(-m, sd), D(m, sd), D(m, sd + mt), D(-m, sd + mt)], 'MAG', 0.5)
    sh.poly([D(-2.75, fl), D(2.75, fl), D(2.75, fl + 3), D(-2.75, fl + 3)], 'STEEL', 0.5)
    sh.poly([D(-1.5, fl), D(1.5, fl), D(1.5, fl - 12), D(-1.5, fl - 12)], 'STEEL', 0.5)
    sh.poly([D(-3.5, -5), D(3.5, -5), D(3.5, -5.5), D(-3.5, -5.5)], 'STEEL', 0.5)
    sh.poly([D(-2.75, -5.5), D(2.75, -5.5), D(2.75, -7.9), D(-2.75, -7.9)], 'STEEL', 0.5)

    # Dimensions.
    sh.dim(D(-L.PUCK_DIA / 2, 0), D(-L.PUCK_DIA / 2, ph), -(r - L.PUCK_DIA / 2) * S - 22,
           fmt(ph))
    sh.dim(D(-rs, 0), D(-rs, sd), -(r - rs) * S - 52, fmt(sd))
    sh.dim(D(-L.PUCK_DIA / 2, 0), D(L.PUCK_DIA / 2, 0), 12 * S, 'Ø%s puck' % fmt(L.PUCK_DIA))
    sh.dim(D(-r, L.BOSS_TOP), D(r, L.BOSS_TOP), -(apex - L.BOSS_TOP) * S - 18,
           'Ø%s boss' % fmt(2 * r))

    # Callouts, to the right.
    tx = D(w_wall + L.WALL_THK, 0)[0] + 60
    sh.leader((tx, D(0, 29)[1]), D(2.2, apex - 2.2 * t60 - 0.3),
              '%g° cone: prints\nwithout support' % L.BOSS_CONE_DEG)
    sh.leader((tx, D(0, 20)[1]), D(w_wall + L.WALL_THK, 20), 'lid wall')
    sh.leader((tx, D(0, 14)[1]), D(m - 0.5, sd + mt - 0.5),
              'lid magnet %g×%g, pocket\nØ%s × %s, face at socket top' % (
                  L.MAGNET_DIA, L.MAGNET_THK, fmt(L.MAGNET_POCKET_DIA), fmt(L.MAGNET_POCKET_DEPTH)))
    sh.leader((tx, D(0, 9.45)[1]), D(rs + 0.2, 9.45),
              'socket Ø%s × %s deep;\n%s landing gap above puck' % (
                  fmt(L.SOCKET_DIA), fmt(sd), fmt(sd - ph)))
    sh.leader((tx, D(0, 5.5)[1]), D(m - 0.5, z_mag + 1.5),
              'deck magnet %g×%g' % (L.MAGNET_DIA, L.MAGNET_THK))
    sh.leader((tx, D(0, 1.5)[1]), D(2.75, fl + 1.5), 'M3×12 socket head')
    sh.leader((tx, D(0, -6.7)[1]), D(2.75, -6.7), 'washer + nut under deck')


def table(sh, x, y, pieces):
    sh.txt('PARTS', (x, y), 14, bold=True, align='l')
    rows = ['%-10s %3s   %-22s %7s   %s' % ('part', 'qty', 'print size (mm)', 'mass', 'print')]
    total = 0.0
    for name, qty, ext, g, how in pieces:
        total += qty * g
        rows.append('%-10s %3d   %-22s %5.0f g   %s' % (
            name, qty, ' × '.join(fmt(e) for e in ext), g, how))
    rows.append('%-10s %3s   %-22s %5.0f g' % ('total', '', '', total))
    sh.txt('\n'.join(rows), (x, y + 62), 10, mono=True, align='l')

    sh.txt('HARDWARE', (x, y + 132), 14, bold=True, align='l')
    sh.txt('\n'.join([
        '%d × M3×8 + nyloc nut   seam lips' % HARDWARE['seam_bolts'],
        '6 × M3×12 + washer + nut   pucks through the deck',
        '12 × %g×%g mm disc magnet (N52)' % (L.MAGNET_DIA, L.MAGNET_THK),
    ]), (x, y + 168), 10, mono=True, align='l')

    sh.txt('NOTES', (x + 470, y), 14, bold=True, align='l')
    sh.txt('\n'.join([
        '1. Dimensions in mm. Do not scale the drawing.',
        '2. Lid is 6-fold symmetric: any corner can face front.',
        '3. Seams line up with the deck\'s: hub r %s, ring r %s,' % (
            fmt(L.HUB_APOTHEM), fmt(L.TOP_APOTHEM)),
        '   radial at the corners.',
        '4. Deck: Ø3.4 puck hole in each outer piece, %s in from' % fmt(DECK_APOTHEM - LID_PUCK_Y),
        '   the edge, %s from the corner clockwise of its leg.' % fmt(
            DECK_APOTHEM * TAN30 - LID_PUCK_X),
        '5. Magnets: deck magnets all the same face up; seat each',
        '   lid magnet by sticking it to a deck magnet first.',
        '6. Vents: hex, %s across corners, %s ribs, trimmed to' % (
            fmt(2 * L.VENT_R), fmt(L.VENT_RIB)),
        '   the panel edges. Not shown in the plan or section.',
    ]), (x + 470, y + 100), 10, align='l')


def title_block(sh, x, y, w, h):
    sh.pl([(x, y), (x + w, y), (x + w, y + h), (x, y + h)], 1.2, closed=True)
    sh.pl([(x, y + 44), (x + w, y + 44)], 0.6)
    sh.txt('CRABORA  —  LID v1', (x + 14, y + 22), 18, bold=True, align='l')
    sh.txt('\n'.join([
        'Faceted magnetic lid over the flanged unibody deck',
        'Source: unibody/design_lid.py   Drawing: draw_lid_omnigraffle.py',
        'Tom Danner   %s   units: mm' % datetime.date.today().isoformat(),
    ]), (x + 14, y + 74), 10, align='l')


HARDWARE = {}


def main():
    hub, hub_holes = L.build_hub()
    mid, mid_holes = L.build_mid()
    outer, outer_holes = L.build_outer()
    HARDWARE['seam_bolts'] = (hub_holes + 6 * mid_holes + 6 * outer_holes) // 2
    g = lambda m: m.volume / 1000 * L.PLA_DENSITY
    ext = lambda m: [round(e, 1) for e in m.extents]
    pieces = [
        ('lid_hub', 1, ext(to_print_pose(hub)), g(hub), 'top face down'),
        ('lid_mid', 6, ext(to_print_pose(mid)), g(mid), 'top face down'),
        ('lid_outer', 6, ext(L.outer_print_pose(outer)), g(outer), 'on the slope facet'),
        ('lid_puck', 6, [L.PUCK_DIA, L.PUCK_DIA, L.PUCK_H], g(L.build_puck()), 'pocket up'),
    ]

    sh = Sheet()
    plan(sh, 420, 405)
    section(sh, 820, 330)
    detail(sh, 1440, 470)
    table(sh, 60, 760, pieces)
    title_block(sh, 1300, 960, 420, 120)

    with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False) as f:
        f.write('\n'.join(sh.js) + '\n"ok";\n')
        js_path = f.name
    for p in (GRAFFLE, PDF):
        if os.path.exists(p):
            subprocess.run(['rm', '-rf', p], check=True)
    script = f'''
tell application "OmniGraffle"
  make new document
  delay 1
  evaluate javascript (read POSIX file "{js_path}" as «class utf8»)
  save front document in POSIX file "{GRAFFLE}"
  export front document as "PDF" scope entire document to POSIX file "{PDF}"
end tell
'''
    subprocess.run(['osascript', '-e', script], check=True)
    os.unlink(js_path)
    print(f'wrote {GRAFFLE}\nwrote {PDF}')


if __name__ == '__main__':
    main()
