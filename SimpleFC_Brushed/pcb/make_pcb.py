"""
SimpleFC BRUSHED shield for the ESP32-C3 SuperMini - single-sided PCB generator + design-rule check.

Run:  python make_pcb.py       (print the SVGs at 100 % / actual size)
  copper_toner_print.svg   black = copper, for toner transfer (NOT mirrored: ironing it on mirrors it).
  copper_side_view.svg     the finished copper side (mirrored), dark = remove. Use it to cut by hand.
  smd_copper_side.svg      copper side with the SMD parts: MOSFETs, 0805 resistors, diodes are soldered HERE.
  assembly_top.svg         component side: sockets, capacitors, wire pads.
  drill_list.txt           hole sizes and counts.
Grid: 2.54 mm. Coordinates are grid units seen from the COMPONENT side (x-ray), SuperMini USB-C at the top.
GND is a copper flood. Driver per motor: GPIO -100R- gate, gate -10k- GND, source = GND, drain = motor (-),
motor (+) = battery (+), Schottky across the motor (cathode to battery +).
"""
import re
import numpy as np

U = 2.54                  # grid (mm)
PAD = 1.9                 # through-hole pad diameter (mm)
PAD_W = 2.4               # motor / battery wire pads
CLR = 0.5                 # isolation gap (mm)
EDGE = 0.4                # flood kept this far from the board edge (mm)
W_SIG, W_PWR, W_MOT = 0.8, 1.1, 1.3   # trace widths (mm); W_MOT carries motor current
BX0, BX1, BY0, BY1 = -10.8, 13.8, -4.3, 10.9   # board outline (grid units)

pads = []     # (name, gx, gy, net, drill, w, h)   w/h in mm, drill 0 = SMD rectangle, else round pad of diameter w
traces = []   # (net, width, [(gx, gy), ...])
def pad(name, x, y, net, drill=1.0, dia=PAD): pads.append((name, x, y, net, drill, dia, dia))
def smd(name, x, y, net, w, h): pads.append((name, x, y, net, 0, w, h))
def tr(net, w, *pts): traces.append((net, w, list(pts)))

# ---- ESP32-C3 SuperMini (female headers), top view, USB-C up ----
L = ["G5", "G6", "G7", "G8", "G9", "G10", "G20", "G21"]
R = ["5V", "GND", "3V3", "G4", "G3", "G2", "G1", "G0"]
LNET = ["S4", "SDA", "SCL", "NC_G8", "NC_G9", "NC_G10", "NC_G20", "NC_G21"]
RNET = ["5V", "GND", "3V3", "VBAT", "S3", "NC_G2", "S2", "S1"]
for i in range(8):
    pad("U1." + L[i], 0, i, LNET[i])
    pad("U1." + R[i], 6, i, RNET[i])

# ---- motor driver block ----
def driver(n, X0, Y, rail_y):
    """Signal net S<n> must arrive at (X0, Y). All SMD on the copper side."""
    s, g, d = "S%d" % n, "Q%d" % n, "D%d" % n
    a = X0 + 4.0                                  # diode anode column, cathode 1.57 further (SMA, 4 mm)
    smd("R1%d.a" % n, X0, Y, s, 1.2, 1.4); smd("R1%d.b" % n, X0 + 0.8, Y, g, 1.2, 1.4)   # gate 100R
    smd("R2%d.a" % n, X0 + 1.6, Y, g, 1.4, 1.2); smd("R2%d.b" % n, X0 + 1.6, Y + 0.8, "GND", 1.4, 1.2)   # gate 10k to GND
    tr(g, W_SIG, (X0 + 0.8, Y), (X0 + 2.6, Y))
    # SOT-23 (x-ray view): drain to the right, source up, gate down. On the copper side: D left, S up, G down.
    smd("Q%d.G" % n, X0 + 2.6, Y, g, 1.0, 0.9)
    smd("Q%d.S" % n, X0 + 2.6, Y - 0.75, "GND", 1.0, 0.9)
    smd("Q%d.D" % n, X0 + 3.47, Y - 0.37, d, 1.0, 1.0)
    smd("D%d.A" % n, a, Y - 0.37, d, 1.5, 1.8)
    smd("D%d.K" % n, a + 1.57, Y - 0.37, "BAT", 1.5, 1.8)
    tr(d, W_MOT, (X0 + 3.47, Y - 0.37), (a, Y - 0.37))
    pad("M%d-" % n, a - 0.3, Y + 0.6, d, 1.3, PAD_W)   # motor (-) wire
    tr(d, W_MOT, (a, Y - 0.37), (a - 0.3, Y + 0.6))
    tr("BAT", W_MOT, (a + 1.57, Y - 0.37), (a + 1.8, rail_y))

RX = 13.0                                         # battery + rail (right side)
TOP = -3.5                                        # battery + rail (top)
# M3, M2, M1 on the right. Rows spaced 2.4 so the GND flood reaches every source between the blocks.
driver(3, 7.2, 4.4, 4.03); tr("S3", W_SIG, (6, 4), (7.2, 4.4))
driver(2, 7.2, 6.8, 6.43); tr("S2", W_SIG, (6, 6), (7.2, 6.8))
driver(1, 7.2, 9.2, 8.83); tr("S1", W_SIG, (6, 7), (6, 9.2), (7.2, 9.2))
for n, y in [(3, 4.03), (2, 6.43), (1, 8.83)]: pad("M%d+" % n, RX, y, "BAT", 1.3, PAD_W)
tr("BAT", 1.5, (RX, 8.83), (RX, TOP), (5.97, TOP), (5.97, -2.37))
# M4 at the top, above the socket (G5 is on the far side from the rest)
driver_x = 0.4
n = 4
s, g, d = "S4", "Q4", "D4"
driver(4, 0.4, -2.0, -2.37)
tr("S4", W_SIG, (0, 0), (0, -2.0), (0.4, -2.0))
traces[:] = [t for t in traces if not (t[0] == "BAT" and t[2] == [(0.4 + 4.0 + 1.57, -2.37), (0.4 + 4.0 + 1.8, -2.37)])]
pad("M4+", 5.97, TOP, "BAT", 1.3, PAD_W)

# ---- power ----
pad("BAT+", 11, -1, "BAT", 1.3, PAD_W); pad("BAT-", 11, 0.3, "GND", 1.3, PAD_W)
tr("BAT", W_MOT, (11, -1), (11, TOP))
pad("C1.+", 12.0, TOP, "BAT"); pad("C1.-", 12.0, TOP + 1, "GND")   # 470-1000 uF, 2.5 mm pitch
pad("BST.OUT", 7, -1, "5V"); pad("C3.+", 8, -1, "5V"); pad("C3.-", 8, 0, "GND")   # 100 uF on the 5 V pin
pad("BST.IN", 9, -1, "BAT"); pad("BST.GND", 9, 0, "GND")           # 2S -> 5 V buck module on three wires
tr("5V", W_PWR, (6, 0), (6, -1), (8, -1))
tr("BAT", W_PWR, (9, -1), (9, TOP))

# ---- GND bus wires (thick wire on the component side, soldered through these holes): the copper necks between the diode
# pads are too thin for ~10 A. W1 runs down the channel from BAT- past all three right-side sources, W2 feeds Q4's source.
for k, y in enumerate((1.0, 5.23, 7.63, 10.03)): pad("W1.%d" % k, 12, y, "GND", 1.3, PAD)
pad("W2.a", 4.4, -3.3, "GND", 1.3, PAD); pad("W2.b", 10, -2.2, "GND", 1.3, PAD)

# ---- battery sense: BAT -R1 100k- G4 -R2 33k- GND, C2 100 nF G4-GND. R1 is through-hole, it bridges the GND channel ----
tr("VBAT", W_SIG, (6, 3), (10, 3), (11, 2.2))
pad("R1.a", 11, 2.2, "VBAT"); pad("R1.b", RX, 2.2, "BAT")
smd("R2.a", 7.5, 3, "VBAT", 1.4, 1.2); smd("R2.b", 7.5, 2.2, "GND", 1.4, 1.2)
smd("C2.a", 8.5, 3, "VBAT", 1.4, 1.2); smd("C2.b", 8.5, 2.2, "GND", 1.4, 1.2)

# ---- I2C: SDA y=1, SCL y=2, GND y=3 (flood), 3V3 y=4 ----
# BMP280 header x=-2 (upright or cable), MPU6050 GY-521 x=-4 (lies flat, body to the left). No OLED: configured over USB.
for x, nm in [(-2, "BMP"), (-4, "MPU")]:
    pad(nm + ".SDA", x, 1, "SDA"); pad(nm + ".SCL", x, 2, "SCL")
    pad(nm + ".GND", x, 3, "GND"); pad(nm + ".VCC", x, 4, "3V3")
pad("BMP.CSB", -2, 0, "NC_BCSB"); pad("BMP.SDO", -2, -1, "NC_BSDO")
for y, nm in [(0, "XDA"), (-1, "XCL"), (-2, "AD0"), (-3, "INT")]:
    pad("MPU." + nm, -4, y, "NC_M" + nm)
tr("SDA", W_SIG, (0, 1), (-4, 1))
tr("SCL", W_SIG, (0, 2), (-4, 2))
tr("3V3", W_PWR, (6, 2), (5, 2), (5, 10), (-5, 10), (-5, 4), (-2, 4))

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
    for (_, gx, gy, n, drill, w, h) in pads:
        if n != net: continue
        cx, cy = mm(gx, gy)
        ext = max(w, h) / 2 + grow
        x0, x1 = int((cx - ext) / RES) - 1, int((cx + ext) / RES) + 2
        y0, y1 = int((cy - ext) / RES) - 1, int((cy + ext) / RES) + 2
        sl = (slice(max(y0, 0), y1), slice(max(x0, 0), x1))
        if drill:
            mask[sl] |= (XX[sl] - cx) ** 2 + (YY[sl] - cy) ** 2 <= (w / 2 + grow) ** 2
        else:
            dx = np.maximum(np.abs(XX[sl] - cx) - w / 2, 0); dy = np.maximum(np.abs(YY[sl] - cy) - h / 2, 0)
            mask[sl] |= dx ** 2 + dy ** 2 <= grow ** 2 + 1e-9
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
# 1) clearance between different nets
grown = {n: draw(np.zeros((NY, NX), bool), n, CLR / 2 - 0.01) for n in nets}
for i, n1 in enumerate(nets):
    for n2 in nets[i + 1:]:
        both = grown[n1] & grown[n2]
        if both.any():
            yy, xx = np.nonzero(both)
            errors.append("CLEARANCE %s / %s near grid (%.1f, %.1f)" % (n1, n2, xx[0] * RES / U + BX0, yy[0] * RES / U + BY0))
# 2) every signal net is one piece of copper
for n in signal_nets:
    cu = draw(np.zeros((NY, NX), bool), n, 0)
    seeds = [mm(p[1], p[2]) for p in pads if p[3] == n]
    reach = connected_from(cu, seeds[:1])
    for p in pads:
        if p[3] == n:
            cx, cy = mm(p[1], p[2])
            if not reach[int(cy / RES), int(cx / RES)]:
                errors.append("OPEN net %s: pad %s not connected" % (n, p[0]))
# 3) GND flood
ANT = (0.6, 6.7, 5.4, 8.6)   # no copper under the SuperMini's chip antenna
SLIVER = 0.25                # flood narrower than 2 x this (mm) is removed
RELIEF_GAP, SPOKE = 0.4, 0.8

def disc_offsets(r):
    k = int(r / RES)
    return [(dy, dx) for dy in range(-k, k + 1) for dx in range(-k, k + 1) if dx * dx + dy * dy <= k * k]
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
flood = dilate(erode(flood, SLIVER), SLIVER) & inside & ~keepout & ~antenna
gnd_pads = [(p[0], mm(p[1], p[2]), p[4], p[5]) for p in pads if p[3] == "GND"]
for nm, (cx, cy), drill, w in gnd_pads:             # thermal relief on through-hole GND pads only
    if not drill or nm == "BAT-" or nm.startswith("W"): continue   # battery return and bus wires stay solid
    d2 = (XX - cx) ** 2 + (YY - cy) ** 2
    ring = (d2 > (w / 2) ** 2) & (d2 <= (w / 2 + RELIEF_GAP) ** 2)
    spokes = (np.abs(XX - cx) <= SPOKE / 2) | (np.abs(YY - cy) <= SPOKE / 2)
    flood &= ~(ring & ~spokes)
flood |= draw(np.zeros((NY, NX), bool), "GND", 0)
reach = connected_from(flood, [gnd_pads[0][1]])
for nm, (cx, cy), _, _ in gnd_pads:
    if not reach[int(cy / RES), int(cx / RES)]:
        errors.append("GND pad %s is not connected to the main flood" % nm)
floating = flood & ~reach
sig_cu = np.zeros((NY, NX), bool)
for n in signal_nets: draw(sig_cu, n, 0)
if (dilate(sig_cu, CLR - 2 * RES) & flood).any(): errors.append("FLOOD too close to a signal")
copper = flood | sig_cu
# narrowest GND path feeding each MOSFET source: flood must survive a 1 mm wide probe from the battery pad (current check)
def path_ok(r):
    w = connected_from(erode(flood, r), [mm(12.0, 1.0)])
    return dilate(w, r) & flood
wide = path_ok(0.5)
for r in (0.25, 0.35, 0.5, 0.75):
    w = path_ok(r)
    print("  GND probe %.1f mm wide reaches sources:" % (2 * r), [p[0] for p in pads if p[0] in ("Q1.S", "Q2.S", "Q3.S", "Q4.S") and
          w[max(int((mm(p[1], p[2])[1] - 0.7) / RES), 0):int((mm(p[1], p[2])[1] + 0.7) / RES), max(int((mm(p[1], p[2])[0] - 0.7) / RES), 0):int((mm(p[1], p[2])[0] + 0.7) / RES)].any()])
print("floating copper islands (kept, not connected): %.1f mm^2" % (floating.sum() * RES * RES))
print("DRC:", "OK - no errors" if not errors else "%d ERROR(S)" % len(errors))
for e in errors: print("  ", e)

# ===================== SVG output =====================
def raster_rects(m):
    out, open_runs = [], {}
    for y in range(m.shape[0]):
        d = np.diff(np.concatenate(([0], m[y].astype(np.int8), [0])))
        runs = set(zip(np.nonzero(d == 1)[0].tolist(), np.nonzero(d == -1)[0].tolist()))
        nxt = {}
        for r in runs: nxt[r] = open_runs.pop(r, y)
        for (x0, x1), y0 in open_runs.items(): out.append((x0, y0, x1, y))
        open_runs = nxt
    for (x0, x1), y0 in open_runs.items(): out.append((x0, y0, x1, m.shape[0]))
    return ['<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f"/>' % ((x0 - 0.5) * RES, (y0 - 0.5) * RES, (x1 - x0) * RES, (y1 - y0) * RES)
            for (x0, y0, x1, y1) in out]
RECTS = raster_rects(copper)

def circle(cx, cy, r, fill): return '<circle cx="%.3f" cy="%.3f" r="%.3f" fill="%s"/>' % (cx, cy, r, fill)
def copper_layer(cu, hole):
    s = ['<g fill="%s" shape-rendering="crispEdges">' % cu] + RECTS + ["</g>"]
    for p in pads:
        if p[4]: s.append(circle(*mm(p[1], p[2]), 0.35, hole))   # drill guide (centre mark)
    return s

def svg(name, body, mirror=False, bg="white", title="", extra_h=6, footer=(), after=()):
    g = '<g transform="translate(%.3f 0) scale(-1 1)">' % BW if mirror else "<g>"
    H = BH + extra_h
    with open(name, "w", encoding="utf-8") as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" width="%.3fmm" height="%.3fmm" viewBox="0 0 %.3f %.3f">\n' % (BW, H, BW, H))
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="white"/>\n' % (BW, H))
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="%s"/>\n' % (BW, BH, bg))
        f.write(g + "\n" + "\n".join(body) + "\n</g>\n")
        f.write("\n".join(after) + "\n")
        f.write('<rect x="0" y="0" width="%.3f" height="%.3f" fill="none" stroke="#888" stroke-width="0.1"/>\n' % (BW, BH))
        f.write('<text x="0.5" y="%.3f" font-family="Arial" font-size="1.5">%s (%.1f x %.1f mm, print at 100%%)</text>\n'
                % (BH + 2.6, title, BW, BH))
        for i, line in enumerate(footer):
            f.write('<text x="0.5" y="%.3f" font-family="Arial" font-size="1.12">%s</text>\n' % (BH + 4.6 + i * 1.6, line))
        f.write("</svg>\n")

svg("copper_toner_print.svg", copper_layer("black", "white"), title="SimpleFC Brushed shield - COPPER, toner print, NOT mirrored")
svg("copper_side_view.svg", copper_layer("#c87533", "#fff"), mirror=True, bg="#2a2a2a",
    title="SimpleFC Brushed shield - COPPER SIDE VIEW (mirrored; dark = remove copper)")

# ---- SMD placement, seen from the copper side (mirrored; the labels stay readable) ----
def mx(gx): return BW - mm(gx, 0)[0]
def ctext(gx, gy, s, size=0.8, color="#000", anchor="middle", bold=False):
    return '<text x="%.3f" y="%.3f" font-family="Arial" font-size="%.2f" fill="%s" text-anchor="%s"%s>%s</text>' % (
        mx(gx), mm(0, gy)[1], size, color, anchor, ' font-weight="bold"' if bold else "", s)
def crect(gx, gy, w, h, stroke, fill="none", sw=0.2):
    return '<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f" fill="%s" stroke="%s" stroke-width="%.2f"/>' % (
        mx(gx) - w / 2, mm(0, gy)[1] - h / 2, w, h, fill, stroke, sw)
S = []
for n, X0, Y in [(3, 7.2, 4.4), (2, 7.2, 6.8), (1, 7.2, 9.2), (4, 0.4, -2.0)]:
    a = X0 + 4.0
    S += [crect(X0 + 0.4, Y, 2.0, 1.25, "#00c"), ctext(X0 + 0.4, Y + 0.14, "100", 0.8, "#00c", bold=True)]
    S += [crect(X0 + 1.6, Y + 0.4, 1.25, 2.0, "#080"), ctext(X0 + 1.6, Y + 0.54, "10k", 0.7, "#060", bold=True)]
    S += [crect(X0 + 3.03, Y - 0.37, 2.9, 3.0, "#c00"), ctext(X0 + 3.03, Y - 0.23, "Q%d" % n, 1.0, "#c00", bold=True)]
    S += [ctext(X0 + 2.6, Y + 0.65, "G", 0.8, "#c00"), ctext(X0 + 2.6, Y - 1.1, "S", 0.8, "#c00"), ctext(X0 + 3.6, Y - 0.85, "D", 0.8, "#c00")]
    S += [crect(a + 0.785, Y - 0.37, 4.3, 2.6, "#a0a"), ctext(a + 0.785, Y - 0.23, "SS14", 0.8, "#808", bold=True)]
    S += ['<rect x="%.3f" y="%.3f" width="0.7" height="2.6" fill="#a0a"/>' % (mx(a + 1.57) - 0.35 - 0.8, mm(0, Y - 0.37)[1] - 1.3)]
    S += [ctext(a - 0.3, Y + 0.74, "M%d-" % n, 0.8, "#000", bold=True)]
for y, n in [(4.03, 3), (6.43, 2), (8.83, 1)]: S.append(ctext(RX + 0.45, y + 0.14, "M%d+" % n, 0.8, "#000", "end", True))
S.append(ctext(5.97, TOP + 0.8, "M4+", 0.8, "#000", bold=True))
S += [crect(7.5, 2.6, 1.25, 2.0, "#00c"), ctext(7.5, 2.0, "R2", 0.7, "#00c", bold=True), crect(8.5, 2.6, 1.25, 2.0, "#080"), ctext(8.5, 2.0, "C2", 0.7, "#060", bold=True)]
SM = copper_layer("#d9a066", "#fff")
svg("smd_copper_side.svg", SM, mirror=True, bg="#555", title="SimpleFC Brushed shield - SMD parts, COPPER SIDE", extra_h=10, after=S,
    footer=["Q1-Q4 AO3400 / SI2302 (SOT-23): marking up, drain on the side the single pin points to, as drawn. 100 = 100R 0805, 10k = 10k 0805.",
            "SS14 (SMA): band (cathode) on the M+ side. R2 = 33k 0805 (2S), C2 = 100 nF 0805.",
            "Check each Q with a meter after soldering: gate to source must NOT be shorted."])

# ---- component side ----
A = ['<g opacity="0.33">'] + copper_layer("#c87533", "#fff") + ["</g>"]
def rect(gx0, gy0, gx1, gy1, color, w=0.25):
    x0, y0 = mm(gx0, gy0); x1, y1 = mm(gx1, gy1)
    return '<rect x="%.3f" y="%.3f" width="%.3f" height="%.3f" fill="none" stroke="%s" stroke-width="%.2f"/>' % (x0, y0, x1 - x0, y1 - y0, color, w)
def text(gx, gy, s, size=1.2, color="black", anchor="middle", bold=False):
    x, y = mm(gx, gy)
    return '<text x="%.3f" y="%.3f" font-family="Arial" font-size="%.2f" fill="%s" text-anchor="%s"%s>%s</text>' % (
        x, y, size, color, anchor, ' font-weight="bold"' if bold else "", s)
for p in pads:
    if p[4]: A.append(circle(*mm(p[1], p[2]), 0.55, "#444"))
A += [rect(-0.543, -0.93, 6.543, 7.93, "#06c", 0.35), text(3, 3.2, "ESP32-C3", 1.6, "#06c", bold=True),
      text(3, 3.9, "SuperMini", 1.6, "#06c", bold=True), text(3, -0.35, "USB-C", 1.1, "#06c"), text(3, 7.65, "antenna", 0.9, "#06c")]
for i in range(8):
    A.append(text(0.45, i + 0.13, L[i], 0.85, "#06c", "start"))
    A.append(text(5.55, i + 0.13, R[i], 0.85, "#06c", "end"))
A += [rect(-10.3, -3.67, -3.5, 4.67, "#c00", 0.35), text(-6.9, 0.0, "MPU6050", 1.6, "#c00", bold=True),
      text(-6.9, 0.8, "GY-521, lies flat", 1.1, "#c00"), text(-6.9, 1.5, "X arrow = quad FRONT", 1.0, "#c00")]
A += [rect(-2.42, -1.45, -1.58, 4.45, "#080", 0.3), text(-2.0, -1.75, "BMP280", 1.0, "#080", bold=True)]
for y, s in [(1, "SDA"), (2, "SCL"), (3, "GND"), (4, "VCC")]: A.append(text(-1.45, y + 0.12, s, 0.75, "#080", "start"))
A += ['<circle cx="%.3f" cy="%.3f" r="3.15" fill="none" stroke="#a00" stroke-width="0.25"/>' % mm(12.0, TOP + 0.5), text(12.0, TOP + 0.65, "C1 +", 0.8, "#a00")]
A += [text(11.6, -0.85, "BAT+", 0.8, "#a00", "start", True), text(11.6, 0.45, "BAT-", 0.8, "#000", "start", True)]
A += [text(7, -1.6, "BST", 0.7, "#a00"), text(7, -1.25 + 0.75, "OUT", 0.6, "#a00"), text(9, -1.6, "BST", 0.7, "#a00"), text(9, -0.5, "IN", 0.6, "#a00"), text(9, 0.75, "BST-", 0.6, "#000"),
      text(8, 0.75, "C3", 0.7, "#a00")]
A += [text(12, 1.8, "R1 100k", 0.75), '<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" stroke="#000" stroke-width="0.5"/>' % (*mm(11, 2.2), *mm(RX, 2.2))]
for n, y in [(3, 4.4), (2, 6.8), (1, 9.2)]:
    A.append(text(10.4, y + 0.74, "M%d-" % n, 0.75, "#000", "end", True))
    A.append(text(RX - 0.5, y - 0.05, "M%d+" % n, 0.75, "#000", "end", True))
A += [text(4.1, -0.85, "M4-", 0.75, "#000", "middle", True), text(5.97, TOP + 0.85, "M4+", 0.75, "#000", "middle", True)]
for a, b, nm in [((12, 1.0), (12, 10.03), "W1"), ((4.4, -3.3), (10, -2.2), "W2")]:
    A.append('<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" stroke="#e0a000" stroke-width="0.8"/>' % (*mm(*a), *mm(*b)))
A += [text(12.0, 3.5, "W1", 0.9, "#b07800", "middle", True), text(7.5, -3.0, "W2", 0.9, "#b07800", "middle", True)]
FOOT = ["W1, W2: thick GND wires (1 mm2 / 18 AWG) on this side, soldered through every hole they pass. They carry the motor current: do not skip them.",
        "U1 ESP32-C3 SuperMini on two 1x8 female headers. MPU6050 GY-521 on 1x8 female (lies flat). BMP280 1x6 female or cable. No OLED: configure over USB.",
        "C1 470-1000 uF low-ESR 16 V or more, C3 100 uF 10 V or more, both radial 2.5 mm. R1 100k through-hole: stand it up and take the long lead across (it bridges a GND channel).",
        "Buck module 2S to 5V on three wires (set it to 5.0 V first): BST IN, BST- and BST OUT. Motors on wire pads M1..M4 + and -; swap a motor's wires if it spins the wrong way.",
        "Everything else is SMD on the copper side: see smd_copper_side.svg."]
svg("assembly_top.svg", A, title="SimpleFC Brushed shield - COMPONENT SIDE (top view)", extra_h=13, footer=FOOT)

drills = {}
for p in pads:
    if p[4]: drills[p[4]] = drills.get(p[4], 0) + 1
with open("drill_list.txt", "w") as f:
    f.write("SimpleFC Brushed shield drill list\n")
    for d, c in sorted(drills.items()): f.write("  %.1f mm : %d holes\n" % (d, c))
    f.write("Board %.1f x %.1f mm, single-sided, SMD parts on the copper side.\n" % (BW, BH))
print("pads:", len(pads), "board %.1f x %.1f mm" % (BW, BH))

# ---- copper_preview.png: on-screen check only (x-ray view from the component side), NOT for printing ----
import zlib, struct
def write_png(name, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\0" + rgb[y].tobytes() for y in range(h))
    def chunk(tag, data): return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
    with open(name, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
img = np.full((NY, NX, 3), 40, np.uint8)
img[flood] = (200, 117, 51)
img[sig_cu] = (235, 190, 90)
for p in pads:
    if p[4]:
        cx, cy = mm(p[1], p[2])
        img[(XX - cx) ** 2 + (YY - cy) ** 2 <= 0.4 ** 2] = (255, 255, 255)
write_png("copper_preview.png", img[::2, ::2])
