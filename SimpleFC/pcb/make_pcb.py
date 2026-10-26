"""
SimpleFC shield for the ESP32-C3 SuperMini - single-sided PCB generator + design-rule check.

Run:  python make_pcb.py
Writes (all 1:1, print at 100 % / "actual size"):
  copper_toner_print.svg   black = copper. Print this for toner transfer / photo resist (NOT mirrored: it is
                           mirrored automatically when the paper is ironed face-down onto the copper).
  copper_side_view.svg     what the finished copper side looks like (mirrored). Use it to draw with a marker,
                           to cut by hand (scrape away the dark gaps) or as the CNC isolation reference.
  assembly_top.svg         component side: where every part goes, labels, the jumper wire.
  drill_list.txt           hole sizes and counts.
Grid: 2.54 mm. Coordinates below are grid units, x to the right, y down, seen from the COMPONENT side
(SuperMini USB-C at the top). GND is a copper flood: every pad on net "GND" simply sits in it.
"""
import math
import numpy as np

U = 2.54                  # grid (mm)
PAD = 1.9                 # pad diameter (mm)
CLR = 0.5                 # isolation gap (mm)
EDGE = 0.4                # flood kept this far from the board edge (mm)
W_SIG, W_PWR = 0.8, 1.1   # trace widths (mm)
BX0, BX1, BY0, BY1 = -11.8, 11.8, -4.0, 10.0   # board outline (grid units)

pads = []     # (name, gx, gy, net, drill)
traces = []   # (net, width, [(gx, gy), ...])
def pad(name, x, y, net, drill=1.0): pads.append((name, x, y, net, drill))
def tr(net, w, *pts): traces.append((net, w, list(pts)))

# ---- ESP32-C3 SuperMini (female headers), top view, USB-C up ----
L = ["G5", "G6", "G7", "G8", "G9", "G10", "G20", "G21"]
R = ["5V", "GND", "3V3", "G4", "G3", "G2", "G1", "G0"]
LNET = ["M4A", "SDA", "SCL", "NC_G8", "NC_G9", "O4TX", "BUZ", "O4RX"]
RNET = ["5V", "GND", "3V3", "VBAT", "M3", "NC_G2", "M2", "M1"]
for i in range(8):
    pad("U1." + L[i], 0, i, LNET[i])
    pad("U1." + R[i], 6, i, RNET[i])

# ---- motors: ESC header 2x4 (signal x=10, GND x=11), M4 comes over jumper wire W1 ----
pad("W1a", 0, -1, "M4A"); tr("M4A", W_SIG, (0, 0), (0, -1))
pad("W1b", 8, 5, "M4B")
for y, n in [(4, "M3"), (5, "M4B"), (6, "M2"), (7, "M1")]:
    pad("ESC." + n[:2], 10, y, n)
    pad("ESC.GND%d" % y, 11, y, "GND")
tr("M3", W_SIG, (6, 4), (10, 4))
tr("M4B", W_SIG, (8, 5), (10, 5))
tr("M2", W_SIG, (6, 6), (10, 6))
tr("M1", W_SIG, (6, 7), (10, 7))

# ---- power: 5V in from BEC, C1 470uF, buzzer + ----
pad("J5V.+", 8, 0, "5V"); pad("J5V.-", 8, 1, "GND")
pad("C1.+", 10, 0, "5V"); pad("C1.-", 10, 1, "GND")
pad("BZ.+", 7, -1, "5V"); pad("BZ.-", 8, -1, "BZN")
tr("5V", W_PWR, (6, 0), (10, 0))
tr("5V", W_PWR, (7, 0), (7, -1))

# ---- battery sense: BAT+ -R1 100k- G4 -R2 10k- GND, C2 100nF G4-GND ----
pad("R2.a", 7, 3, "VBAT"); pad("R2.b", 7, 2, "GND")
pad("C2.a", 8, 3, "VBAT"); pad("C2.b", 8, 2, "GND")
pad("R1.a", 9, 3, "VBAT"); pad("R1.b", 9, 2, "BATP")
pad("JBAT", 10, 3, "BATP")
tr("VBAT", W_SIG, (6, 3), (9, 3))
tr("BATP", W_SIG, (9, 2), (10, 2), (10, 3))

# ---- buzzer driver: G20 -R3 1k- Q1 base, Q1 E = GND, Q1 C = buzzer - ----
pad("R3.a", 7, -2, "BUZ"); pad("R3.b", 8, -2, "QB")
pad("Q1.C", 9, -1, "BZN"); pad("Q1.B", 9, -2, "QB"); pad("Q1.E", 9, -3, "GND")
tr("BUZ", W_SIG, (0, 6), (1, 6), (1, -2), (7, -2))
tr("QB", W_SIG, (8, -2), (9, -2))
tr("BZN", W_SIG, (8, -1), (9, -1))

# ---- I2C: SDA y=1, SCL y=2, GND y=3 (flood), 3V3 y=4 ----
# OLED header x=-2 (cable), BMP280 x=-3 (stands up), MPU6050 GY-521 x=-5 (lies flat, body to the left)
for x, nm in [(-2, "OLED"), (-3, "BMP"), (-5, "MPU")]:
    pad(nm + ".SDA", x, 1, "SDA"); pad(nm + ".SCL", x, 2, "SCL")
    pad(nm + ".GND", x, 3, "GND"); pad(nm + ".VCC", x, 4, "3V3")
pad("BMP.CSB", -3, 0, "NC_BCSB"); pad("BMP.SDO", -3, -1, "NC_BSDO")
for y, nm in [(0, "XDA"), (-1, "XCL"), (-2, "AD0"), (-3, "INT")]:
    pad("MPU." + nm, -5, y, "NC_M" + nm)
tr("SDA", W_SIG, (0, 1), (-5, 1))
tr("SCL", W_SIG, (0, 2), (-5, 2))
tr("3V3", W_PWR, (6, 2), (5, 2), (5, 9), (-6, 9), (-6, 4), (-2, 4))

# ---- DJI O4 air unit: 1x3 at x=-1 rows 4..6 (GND, O4 TX -> FC GPIO10, O4 RX <- FC GPIO21) ----
# GND sits at the top so it joins the GND strip (row 3); the rows below are boxed in by 3V3.
pad("O4.GND", -1, 4, "GND"); pad("O4.TX", -1, 5, "O4TX"); pad("O4.RX", -1, 6, "O4RX")
tr("O4TX", W_SIG, (0, 5), (-1, 5))
tr("O4RX", W_SIG, (0, 7), (-1, 7), (-1, 6))

# ===================== geometry helpers (mm) =====================
def mm(gx, gy): return ((gx - BX0) * U, (gy - BY0) * U)
BW, BH = (BX1 - BX0) * U, (BY1 - BY0) * U
nets = sorted({p[3] for p in pads} | {t[0] for t in traces})
signal_nets = [n for n in nets if n != "GND"]

# ===================== DRC on a raster =====================
RES = 0.05
NX, NY = int(BW / RES) + 1, int(BH / RES) + 1
XX, YY = np.meshgrid(np.arange(NX) * RES, np.arange(NY) * RES)

def draw(mask, net, grow):
    for (_, gx, gy, n, _) in pads:
        if n != net: continue
        cx, cy = mm(gx, gy)
        r = PAD / 2 + grow
        x0, x1 = int((cx - r) / RES) - 1, int((cx + r) / RES) + 2
        y0, y1 = int((cy - r) / RES) - 1, int((cy + r) / RES) + 2
        sl = (slice(max(y0, 0), y1), slice(max(x0, 0), x1))
        mask[sl] |= (XX[sl] - cx) ** 2 + (YY[sl] - cy) ** 2 <= r * r
    for (n, w, pts) in traces:
        if n != net: continue
        for a, b in zip(pts, pts[1:]):
            ax, ay = mm(*a); bx, by = mm(*b)
            r = w / 2 + grow
            x0, x1 = int((min(ax, bx) - r) / RES) - 1, int((max(ax, bx) + r) / RES) + 2
            y0, y1 = int((min(ay, by) - r) / RES) - 1, int((max(ay, by) + r) / RES) + 2
            sl = (slice(max(y0, 0), y1), slice(max(x0, 0), x1))
            px, py = XX[sl], YY[sl]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0, 1) if L2 else 0
            mask[sl] |= (px - ax - t * dx) ** 2 + (py - ay - t * dy) ** 2 <= r * r
    return mask

def connected_from(allowed, seeds):
    reach = np.zeros_like(allowed)
    for (sx, sy) in seeds:
        reach[int(sy / RES), int(sx / RES)] = True
    reach &= allowed
    while True:
        g = reach.copy()
        g[1:, :] |= reach[:-1, :]; g[:-1, :] |= reach[1:, :]
        g[:, 1:] |= reach[:, :-1]; g[:, :-1] |= reach[:, 1:]
        g &= allowed
        if (g == reach).all(): return reach
        reach = g

errors = []
# 1) clearance between different nets (each grown by CLR/2 must not overlap)
count = np.zeros((NY, NX), np.int16)
grown = {}
for n in nets:
    m = draw(np.zeros((NY, NX), bool), n, CLR / 2 - 0.01)
    grown[n] = m
    count += m
bad = count >= 2
if bad.any():
    ys, xs = np.nonzero(bad)
    for n1 in nets:
        for n2 in nets:
            if n1 < n2 and (grown[n1] & grown[n2]).any():
                yy, xx = np.nonzero(grown[n1] & grown[n2])
                errors.append("CLEARANCE %s / %s near (%.1f, %.1f) mm" % (n1, n2, xx[0] * RES, yy[0] * RES))
# 2) every signal net is one piece of copper
for n in signal_nets:
    cu = draw(np.zeros((NY, NX), bool), n, 0)
    seeds = [mm(gx, gy) for (_, gx, gy, nn, _) in pads if nn == n]
    reach = connected_from(cu, seeds[:1])
    for (nm, gx, gy, nn, _) in pads:
        if nn == n:
            cx, cy = mm(gx, gy)
            if not reach[int(cy / RES), int(cx / RES)]:
                errors.append("OPEN net %s: pad %s not connected" % (n, nm))
# 3) GND flood: board minus edge minus (other nets + CLR) minus antenna keep-out, slivers removed,
#    thermal reliefs on GND pads. Every GND pad must end up in ONE connected flood.
ANT = (0.6, 6.7, 5.4, 8.6)              # no copper under the SuperMini's chip antenna (grid units x0, y0, x1, y1)
SLIVER = 0.25                            # flood narrower than 2 x this (mm) is removed
RELIEF_GAP, SPOKE = 0.4, 0.7             # thermal relief ring width / spoke width (mm)

def disc_offsets(r):
    n = int(r / RES)
    return [(dy, dx) for dy in range(-n, n + 1) for dx in range(-n, n + 1) if dx * dx + dy * dy <= n * n]
def dilate(m, r):
    out = m.copy()
    for dy, dx in disc_offsets(r):
        out |= np.roll(np.roll(m, dy, 0), dx, 1)
    return out
def erode(m, r): return ~dilate(~m, r)

inside = (XX >= EDGE) & (XX <= BW - EDGE) & (YY >= EDGE) & (YY <= BH - EDGE)
keepout = np.zeros((NY, NX), bool)
for n in signal_nets: draw(keepout, n, CLR)
ax0, ay0 = mm(ANT[0], ANT[1]); ax1, ay1 = mm(ANT[2], ANT[3])
antenna = (XX >= ax0) & (XX <= ax1) & (YY >= ay0) & (YY <= ay1)
flood = inside & ~keepout & ~antenna
flood = dilate(erode(flood, SLIVER), SLIVER) & inside & ~keepout & ~antenna     # opening: removes thin slivers
gnd_pads = [(nm, mm(gx, gy)) for (nm, gx, gy, nn, _) in pads if nn == "GND"]
for nm, (cx, cy) in gnd_pads:                                                    # thermal relief
    d2 = (XX - cx) ** 2 + (YY - cy) ** 2
    ring = (d2 > (PAD / 2) ** 2) & (d2 <= (PAD / 2 + RELIEF_GAP) ** 2)
    spokes = (np.abs(XX - cx) <= SPOKE / 2) | (np.abs(YY - cy) <= SPOKE / 2)
    flood &= ~(ring & ~spokes)
flood |= draw(np.zeros((NY, NX), bool), "GND", 0)
reach = connected_from(flood, [gnd_pads[0][1]])
for nm, (cx, cy) in gnd_pads:
    if not reach[int(cy / RES), int(cx / RES)]:
        errors.append("GND pad %s is not connected to the main flood" % nm)
floating = flood & ~reach
# floating islands are kept: harmless, and less copper to remove when cutting by hand
sig_cu = np.zeros((NY, NX), bool)
for n in signal_nets: draw(sig_cu, n, 0)
# final check: flood never closer than CLR - 1 px to any signal copper
if (dilate(sig_cu, CLR - 2 * RES) & flood).any(): errors.append("FLOOD too close to a signal")
copper = flood | sig_cu
print("floating copper islands (kept, not connected): %.1f mm^2" % (floating.sum() * RES * RES))
print("DRC:", "OK - no errors" if not errors else "%d ERROR(S)" % len(errors))
for e in errors: print("  ", e)

# ===================== SVG output =====================
def raster_rects(m):
    """run-length rectangles (merged vertically) of a boolean raster, in mm"""
    out, open_runs = [], {}
    for y in range(m.shape[0]):
        row = m[y]
        d = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
        runs = set(zip(starts.tolist(), ends.tolist()))
        nxt = {}
        for r in runs:
            nxt[r] = open_runs.pop(r, y)
        for (x0, x1), y0 in open_runs.items(): out.append((x0, y0, x1, y))
        open_runs = nxt
    for (x0, x1), y0 in open_runs.items(): out.append((x0, y0, x1, m.shape[0]))
    return ['<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f"/>' % ((x0 - 0.5) * RES, (y0 - 0.5) * RES, (x1 - x0) * RES, (y1 - y0) * RES)
            for (x0, y0, x1, y1) in out]
RECTS = raster_rects(copper)

def circle(cx, cy, r, fill): return '<circle cx="%.3f" cy="%.3f" r="%.3f" fill="%s"/>' % (cx, cy, r, fill)
def copper_layer(cu, hole):
    s = ['<g fill="%s" shape-rendering="crispEdges">' % cu] + RECTS + ["</g>"]
    for (_, gx, gy, n, d) in pads: s.append(circle(*mm(gx, gy), 0.35, hole))   # drill guide (centre mark)
    return s

def svg(name, body, mirror=False, bg="white", title="", extra_h=6, footer=()):
    g = '<g transform="translate(%.3f 0) scale(-1 1)">' % BW if mirror else "<g>"
    H = BH + extra_h
    with open(name, "w", encoding="utf-8") as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" width="%.3fmm" height="%.3fmm" viewBox="0 0 %.3f %.3f">\n' % (BW, H, BW, H))
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="white"/>\n' % (BW, H))
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="%s"/>\n' % (BW, BH, bg))
        f.write(g + "\n" + "\n".join(body) + "\n</g>\n")
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="none" stroke="#888" stroke-width="0.1"/>\n' % (BW, BH))
        f.write('<text x="0.5" y="%.3f" font-family="Arial" font-size="1.5">%s (%.1f x %.1f mm, print at 100%%)</text>\n'
                % (BH + 2.6, title, BW, BH))
        for i, line in enumerate(footer):
            f.write('<text x="0.5" y="%.3f" font-family="Arial" font-size="1.12">%s</text>\n' % (BH + 4.6 + i * 1.6, line))
        f.write("</svg>\n")

svg("copper_toner_print.svg", copper_layer("black", "white"), title="SimpleFC shield - COPPER, toner print, NOT mirrored")
svg("copper_side_view.svg", copper_layer("#c87533", "#fff"), mirror=True, bg="#2a2a2a",
    title="SimpleFC shield - COPPER SIDE VIEW (mirrored; dark = remove copper)")

# ---- assembly (component side) ----
A = ['<g opacity="0.33">'] + copper_layer("#c87533", "#fff") + ["</g>"]
def rect(gx0, gy0, gx1, gy1, color, w=0.25, fill="none"):
    x0, y0 = mm(gx0, gy0); x1, y1 = mm(gx1, gy1)
    return '<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f" fill="%s" stroke="%s" stroke-width="%.2f"/>' % (
        x0, y0, x1 - x0, y1 - y0, fill, color, w)
def text(gx, gy, s, size=1.2, color="black", anchor="middle", bold=False):
    x, y = mm(gx, gy)
    return '<text x="%.3f" y="%.3f" font-family="Arial" font-size="%.2f" fill="%s" text-anchor="%s"%s>%s</text>' % (
        x, y, size, color, anchor, ' font-weight="bold"' if bold else "", s)
def part(gx0, gy0, gx1, gy1, ref, color="#000"):
    return rect(gx0, gy0, gx1, gy1, color, 0.22)
for (_, gx, gy, n, _) in pads: A.append(circle(*mm(gx, gy), 0.55, "#444"))
# SuperMini
A += [rect(-0.543, -0.93, 6.543, 7.93, "#06c", 0.35), text(3, 3.2, "ESP32-C3", 1.6, "#06c", bold=True),
      text(3, 3.9, "SuperMini", 1.6, "#06c", bold=True), text(3, -0.35, "USB-C", 1.1, "#06c"), text(3, 7.65, "antenna", 0.9, "#06c")]
for i in range(8):
    A.append(text(0.45, i + 0.13, L[i], 0.85, "#06c", "start"))
    A.append(text(5.55, i + 0.13, R[i], 0.85, "#06c", "end"))
# I2C side
A += [rect(-11.3, -3.67, -4.5, 4.67, "#c00", 0.35), text(-7.9, 0.0, "MPU6050", 1.6, "#c00", bold=True),
      text(-7.9, 0.8, "GY-521, lies flat", 1.1, "#c00"), text(-7.9, 1.5, "X arrow = quad FRONT", 1.0, "#c00")]
A += [rect(-3.42, -1.45, -2.58, 4.45, "#080", 0.3), text(-3.0, -1.75, "BMP280", 1.0, "#080", bold=True),
      text(-3.0, 5.05, "upright", 0.9, "#080")]
A += [rect(-2.42, 0.55, -1.58, 4.45, "#808", 0.3), text(-3.3, 0.2, "OLED", 0.8, "#808", "end")]
A += [rect(-1.42, 3.55, -0.58, 6.45, "#d06000", 0.3), text(-1.6, 7.35, "DJI O4", 0.9, "#d06000", bold=True)]
for y, s in [(4, "GND"), (5, "TX"), (6, "RX")]: A.append(text(-1.55, y + 0.62, s, 0.65, "#d06000", "end"))
for y, s in [(1, "SDA"), (2, "SCL"), (3, "GND"), (4, "VCC")]: A.append(text(-1.45, y + 0.12, s, 0.75, "#808", "start"))
# ESC header
A += [rect(9.55, 3.55, 11.45, 7.45, "#000", 0.3), text(10.5, 8.05, "ESC", 1.0, bold=True), text(10.5, 8.6, "sig  gnd", 0.75)]
for y, s in [(4, "M3"), (5, "M4"), (6, "M2"), (7, "M1")]: A.append(text(9.45, y + 0.12, s, 0.8, "#000", "end"))
# power / battery / buzzer
A += [text(8, -0.3 + 0.0, "", 0.1), text(8.0, 0.75 - 0.05, "5V", 0.7, "#a00"), text(8.0, 1.62, "GND", 0.6, "#333"),
      text(7.55, 0.45, "IN", 0.6, "#a00", "end")]
A += ['<circle cx="%.3f" cy="%.3f" r="%.3f" fill="none" stroke="#a00" stroke-width="0.25"/>' % (*mm(10, 0.5), 3.15),
      text(10, 0.32 + 0.0, "+", 0.9, "#a00"), text(11.2, -0.55, "C1", 0.9, "#a00")]
A += [part(6.6, 1.6, 7.4, 3.4, "R2"), part(7.6, 1.6, 8.4, 3.4, "C2"), part(8.6, 1.6, 9.4, 3.4, "R1"),
      text(7, 1.45, "R2", 0.7), text(8, 1.45, "C2", 0.7), text(9, 1.45, "R1", 0.7),
      text(10.5, 2.75, "BAT+", 0.7, "#a00")]
A += [part(6.6, -2.4, 8.4, -1.6, "R3"), text(7.5, -2.6, "R3", 0.7),
      rect(8.6, -3.45, 9.4, -0.55, "#000", 0.25), text(9.6, -1.9, "Q1", 0.8, "#000", "start"),
      text(9.55, -2.85, "E", 0.6, "#000", "start"), text(9.55, -0.85, "C", 0.6, "#000", "start"),
      text(7, -0.45, "+", 0.8, "#a00"), text(8, -0.45, "-", 0.9), text(7.5, 0.25 - 0.95, "", 0.1),
      text(5.0, -2.6, "BUZZER", 0.7, "#000"), text(6.5, -0.7, "BZ", 0.7, "#000", "end")]
# jumper W1
wx, wy = mm(0, -1); bx, by = mm(8, 5)
yy1 = mm(0, -1.5)[1]; xx1 = mm(7.6, 0)[0]; yy2 = mm(0, 4.45)[1]
A.append('<path d="M %.3f %.3f L %.3f %.3f L %.3f %.3f L %.3f %.3f L %.3f %.3f" fill="none" stroke="#e0a000" stroke-width="0.6" '
         'stroke-dasharray="1.2 0.6"/>' % (wx, wy, wx, yy1, xx1, yy1, xx1, yy2, bx, by))
A.append(text(3.6, -1.68, "W1: insulated wire link (motor 4)", 0.8, "#b07800"))
FOOT = ["U1 ESP32-C3 SuperMini on two 1x8 female headers.  MPU6050 GY-521 on 1x8 female (lies flat), BMP280 on 1x6 female (upright),",
        "OLED on a 1x4 male header + cable.  ESC: 2x4 male header (signal | GND).  R1 100k, R2 10k, R3 1k (upright), C2 100nF,",
        "C1 470uF 10V radial 2.5 mm.  Q1 NPN TO-92 (2N2222A or BC547 - check that E, B, C match the pads!).  Active 5 V buzzer on BZ + / -.",
        "Wire pads: 5V IN + GND from the BEC, BAT+ from the pack +.  W1: insulated wire link G5 -> M4 (route it around the socket).",
        "DJI O4: 1x3 header GND / TX / RX = the air unit's TX, GND and RX wires (GND / FC GPIO10 / FC GPIO21). Power the O4 from the pack."]
svg("assembly_top.svg", A, title="SimpleFC shield - COMPONENT SIDE (top view)", extra_h=12, footer=FOOT)

drills = {}
for (_, _, _, _, d) in pads: drills[d] = drills.get(d, 0) + 1
with open("drill_list.txt", "w") as f:
    f.write("SimpleFC shield drill list\n")
    for d, c in sorted(drills.items()): f.write("  %.1f mm : %d holes\n" % (d, c))
    f.write("Board %.1f x %.1f mm, single-sided, copper on the bottom.\n" % (BW, BH))
print("pads:", len(pads), "board %.1f x %.1f mm" % (BW, BH))

# ---- preview.html: the three drawings inlined and enlarged (for checking on screen, NOT for printing) ----
import re
parts = []
for fn, h in [("assembly_top.svg", "Component side (assembly)"), ("copper_toner_print.svg", "Copper - toner print (black = copper)"),
              ("copper_side_view.svg", "Copper side view (mirrored, dark = remove copper)")]:
    s = open(fn, encoding="utf-8").read()
    s = re.sub(r'width="[\d.]+mm" height="[\d.]+mm"', 'width="100%"', s, count=1)
    parts.append("<h3>%s</h3>%s" % (h, s))
with open("preview.html", "w", encoding="utf-8") as f:
    f.write('<!doctype html><html><head><meta charset="utf-8"><title>SimpleFC shield preview</title>'
            '<style>body{font-family:Arial;margin:8px;background:#ddd} svg{background:#fff;border:1px solid #999}</style>'
            '</head><body>' + "\n".join(parts) + "</body></html>")
