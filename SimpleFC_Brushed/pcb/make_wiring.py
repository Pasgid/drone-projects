"""Writes wiring_diagram.svg: every connection of the SimpleFC Brushed quad.  Run: python make_wiring.py"""
W, H = 1120, 860
COL = {"BAT": "#d00000", "5V": "#e07000", "3V3": "#b000b0", "GND": "#111111", "SDA": "#0060d0", "SCL": "#008a3c",
       "SIG": "#6a3fb5", "VB": "#008c95"}
o = []
def line(x1, y1, x2, y2, c="#111", w=2.2): o.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{c}" stroke-width="{w}" stroke-linecap="round"/>')
def poly(pts, c="#111", w=2.2): o.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="%s" stroke-linejoin="round" stroke-linecap="round"/>' % (" ".join(f"{x},{y}" for x, y in pts), c, w))
def box(x, y, w, h, fill="#f4f6f8", c="#333", sw=1.8, r=6): o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{c}" stroke-width="{sw}"/>')
def text(x, y, s, size=14, c="#111", a="start", b=False): o.append(f'<text x="{x}" y="{y}" font-family="Arial" font-size="{size}" fill="{c}" text-anchor="{a}"{" font-weight=\"bold\"" if b else ""}>{s}</text>')
def dot(x, y, c="#111", r=4): o.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{c}"/>')
def label(x, y, net, s, side="r", small=False):
    """coloured net tag at the end of a wire; side r: tag starts at x, l: tag ends at x"""
    c = COL[net]; w = (7 + 6.4 * len(s)) if small else (9 + 7.6 * len(s)); hh = 7 if small else 11
    x0 = x if side == "r" else x - w
    o.append(f'<rect x="{x0}" y="{y-hh}" width="{w}" height="{2*hh}" rx="3" fill="{c}"/>')
    text(x0 + w / 2, y + (4 if small else 5), s, 11 if small else 13, "#fff", "middle", True)
def gnd(x, y, c="#111"):
    line(x, y, x, y + 10, c); line(x - 11, y + 10, x + 11, y + 10, c, 2.4); line(x - 7, y + 15, x + 7, y + 15, c, 2.4); line(x - 3, y + 20, x + 3, y + 20, c, 2.4)
def res(x, y, s, vertical=False, c="#111"):
    """resistor box centred at x,y, wire ends +-26"""
    if vertical:
        o.append(f'<rect x="{x-9}" y="{y-20}" width="18" height="40" fill="#fff" stroke="{c}" stroke-width="2"/>'); text(x + 15, y + 5, s, 13, "#111")
    else:
        o.append(f'<rect x="{x-20}" y="{y-9}" width="40" height="18" fill="#fff" stroke="{c}" stroke-width="2"/>'); text(x, y - 14, s, 13, "#111", "middle")
def cap(x, y, s, pol=False):
    """vertical capacitor centred at x,y"""
    line(x, y - 18, x, y - 5); line(x, y + 5, x, y + 18); line(x - 13, y - 5, x + 13, y - 5, "#111", 3); line(x - 13, y + 5, x + 13, y + 5, "#111", 3)
    if pol: text(x - 24, y - 8, "+", 15, "#d00000", "middle", True)
    text(x + 18, y + 5, s, 13)

text(20, 32, "SimpleFC Brushed: wiring", 24, "#111", "start", True)
text(20, 54, "Same-coloured tags are joined together. On the shield PCB these are already routed; this is what each pad does.", 14, "#444")

# ---------- power ----------
box(20, 72, 330, 300, "#fff7f5"); text(32, 96, "POWER", 15, "#a00", "start", True)
box(30, 125, 74, 92, "#ffe3dc"); text(67, 165, "2S", 18, "#111", "middle", True); text(67, 185, "LiPo", 14, "#111", "middle")
poly([(77, 125), (77, 118), (285, 118)], COL["BAT"]); label(285, 118, "BAT", "BAT+")
poly([(77, 217), (77, 330)], COL["GND"]); text(84, 262, "BAT-", 13, "#111", "start", True); gnd(77, 330)
line(165, 118, 165, 150, COL["BAT"]); dot(165, 118, COL["BAT"]); cap(165, 168, ""); text(148, 158, "C1", 13, "#111", "end"); text(148, 174, "470-", 12, "#111", "end"); text(148, 188, "1000 uF", 12, "#111", "end"); line(165, 186, 165, 330); gnd(165, 330)
box(205, 215, 130, 92, "#fff"); text(270, 240, "Buck module", 14, "#111", "middle", True); text(270, 258, "2S to 5 V", 13, "#333", "middle")
text(216, 281, "IN+", 13, COL["BAT"], "start", True); text(324, 281, "OUT+", 13, COL["5V"], "end", True); text(270, 298, "GND", 13, "#111", "middle", True)
poly([(225, 118), (225, 215)], COL["BAT"]); dot(225, 118, COL["BAT"])
line(270, 307, 270, 330); gnd(270, 330)
poly([(335, 262), (372, 262)], COL["5V"])

# ---------- SuperMini ----------
box(470, 72, 200, 300, "#eaf3ff", "#06c", 2.4); text(570, 100, "ESP32-C3", 17, "#06c", "middle", True); text(570, 120, "SuperMini", 17, "#06c", "middle", True)
text(570, 142, "(USB-C at the top)", 12, "#06c", "middle")
Lp = [("GPIO5", "SIG", "M4 gate", 165), ("GPIO6", "SDA", "SDA", 190), ("GPIO7", "SCL", "SCL", 215), ("GPIO8", None, "not used", 240),
      ("GPIO9", None, "not used", 265), ("GPIO10", None, "free", 290), ("GPIO20", None, "free", 315), ("GPIO21", None, "free", 340)]
Rp = [("5V", "5V", "5V", 165), ("GND", "GND", "GND", 190), ("3V3", "3V3", "3V3", 215), ("GPIO4", "VB", "VBAT", 240),
      ("GPIO3", "SIG", "M3 gate", 265), ("GPIO2", None, "not used", 290), ("GPIO1", "SIG", "M2 gate", 315), ("GPIO0", "SIG", "M1 gate", 340)]
for nm, net, lab, y in Lp:
    text(480, y + 5, nm, 13, "#06c")
    line(470, y, 450, y, COL[net] if net else "#bbb")
    if net: label(450, y, net, lab, "l")
    else: text(446, y + 5, lab, 12, "#999", "end")
for nm, net, lab, y in Rp:
    text(660, y + 5, nm, 13, "#06c", "end")
    line(670, y, 690, y, COL[net] if net else "#bbb")
    if net: label(690, y, net, lab, "r")
    else: text(694, y + 5, lab, 12, "#999")
label(372, 262, "5V", "5V", "r")
cap(395, 306, "") ; line(395, 262 + 12, 395, 288, COL["5V"]) if False else None
poly([(385, 273), (395, 273), (395, 288)], COL["5V"]); line(395, 324, 395, 336); gnd(395, 336); text(381, 303, "C3", 13, "#111", "end"); text(381, 319, "100 uF", 12, "#111", "end")

# ---------- sensors ----------
box(800, 72, 300, 300, "#f6fff6"); text(812, 96, "SENSORS (no OLED: USB serial)", 15, "#070", "start", True)
y = 112
for title, sub in [("MPU6050 (GY-521)", "flat, X arrow = front"), ("BMP280 or GY-63", "GY-63: PS=VCC, CSB=GND")]:
    box(815, y, 130, 72, "#fff"); text(880, y + 30, title.split(" (")[0], 13, "#111", "middle", True); text(880, y + 50, sub, 10, "#555", "middle")
    for i, (net, s) in enumerate([("3V3", "VCC"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA")]):
        py = y + 12 + i * 16
        line(945, py, 985, py, COL[net]); text(950, py - 3, s, 10, COL[net], "start", True); label(985, py, net, s if s not in ("VCC",) else "3V3", "r", True)
    y += 84

# ---------- battery sense ----------
box(20, 385, 330, 150, "#f2fbfc"); text(32, 408, "BATTERY SENSE (GPIO4)", 15, "#007a82", "start", True)
label(32, 462, "BAT", "BAT+"); line(82, 462, 104, 462, COL["BAT"]); res(130, 462, "R1 100k"); line(156, 462, 200, 462, COL["VB"])
dot(200, 462, COL["VB"]); line(200, 462, 275, 462, COL["VB"]); dot(270, 462, COL["VB"]); label(275, 462, "VB", "VBAT")
line(200, 462, 200, 478, COL["VB"]); res(200, 498, "", True); text(186, 495, "R2", 13, "#111", "end"); text(186, 511, "33k", 13, "#111", "end"); gnd(200, 518)
line(270, 462, 270, 480, COL["VB"]); cap(270, 498, "C2 100 nF"); gnd(270, 516)

# ---------- motor driver ----------
box(370, 385, 730, 300, "#fbf8ff"); text(382, 408, "MOTOR DRIVER: build four, one per motor (table below)", 15, "#5a2da0", "start", True)
gy = 560  # gate row
label(392, gy, "SIG", "GPIO"); line(445, gy, 470, gy, COL["SIG"]); res(496, gy, "100 ohm"); line(522, gy, 640, gy, COL["SIG"]); dot(575, gy, COL["SIG"])
line(575, gy, 575, gy + 22, COL["SIG"]); res(575, gy + 42, "10k", True); gnd(575, gy + 62)
# MOSFET symbol
mx = 660
o.append(f'<circle cx="{mx+8}" cy="{gy}" r="32" fill="#fff" stroke="#111" stroke-width="2"/>')
line(640, gy, mx - 4, gy, COL["SIG"]); line(mx - 4, gy - 16, mx - 4, gy + 16, "#111", 3); line(mx + 2, gy - 18, mx + 2, gy + 18, "#111", 3)
line(mx + 2, gy - 12, mx + 22, gy - 12); line(mx + 2, gy + 12, mx + 22, gy + 12); line(mx + 2, gy, mx + 16, gy); poly([(mx + 8, gy - 5), (mx + 2, gy), (mx + 8, gy + 5)])
line(mx + 22, gy, mx + 22, gy + 12); line(mx + 16, gy, mx + 22, gy)
text(mx + 34, gy + 50, "N-MOSFET", 12, "#111", "start", True); text(mx + 34, gy + 65, "SOT-23", 11, "#555", "start")
text(mx - 28, gy - 8, "G", 13, COL["SIG"], "end", True); text(mx + 28, gy - 22, "D", 13, "#111", "start", True); text(mx + 28, gy + 26, "S", 13, "#111", "start", True)
poly([(mx + 22, gy + 12), (mx + 22, gy + 62)]); gnd(mx + 22, gy + 62)
# drain up to motor
tx = 860
poly([(mx + 22, gy - 12), (mx + 22, 480), (tx, 480)], "#111", 3); text(770, 473, "motor (-)", 13, "#111", "middle", True)
# motor + diode between battery + (top) and drain
o.append(f'<circle cx="{tx+70}" cy="{480+45-45}" r="0" fill="none"/>')
my = 480
line(tx, my, tx, my, "#111")
poly([(tx, my), (tx + 40, my)], "#111", 3)
o.append(f'<circle cx="{tx+70}" cy="{my}" r="30" fill="#fff" stroke="#111" stroke-width="2.4"/>'); text(tx + 70, my + 5, "M", 22, "#111", "middle", True)
text(tx + 70, my + 50, "820 coreless", 12, "#333", "middle")
poly([(tx + 100, my), (tx + 170, my)], COL["BAT"], 3); label(tx + 170 - 40, my - 25, "BAT", "BAT+"); line(tx + 170, my - 14, tx + 170, my, COL["BAT"], 3)
dot(tx + 170, my, COL["BAT"]); dot(tx + 20, my, "#111")
# diode: anode at drain side, cathode to battery +
dy = my + 80
poly([(tx + 20, my), (tx + 20, dy), (tx + 55, dy)], "#111", 2.4)
o.append(f'<polygon points="{tx+55},{dy-13} {tx+55},{dy+13} {tx+80},{dy}" fill="#fff" stroke="#111" stroke-width="2.2"/>')
line(tx + 80, dy - 14, tx + 80, dy + 14, "#111", 3.4)
poly([(tx + 80, dy), (tx + 170, dy), (tx + 170, my)], COL["BAT"], 2.4)
text(tx + 68, dy + 32, "SS14 Schottky", 13, "#111", "middle", True); text(tx + 68, dy + 48, "band (cathode) to BAT+", 12, "#333", "middle")

# ---------- table ----------
box(20, 700, 1080, 142, "#fff", "#999"); cols = [40, 150, 300, 500, 720]
for x, s in zip(cols, ["Motor", "Gate from", "Position (top view)", "Spin (props in)", "If it spins the wrong way"]): text(x, 724, s, 14, "#111", "start", True)
line(30, 733, 1090, 733, "#999", 1)
for i, (m, g, p, sp) in enumerate([("M1", "GPIO0", "rear right", "CW"), ("M2", "GPIO1", "front right", "CCW"), ("M3", "GPIO3", "rear left", "CCW"), ("M4", "GPIO5", "front left", "CW")]):
    y = 754 + i * 24
    text(cols[0], y, m, 14, "#111", "start", True); text(cols[1], y, g, 14, COL["SIG"], "start", True); text(cols[2], y, p, 14); text(cols[3], y, sp, 14)
    if i == 0: text(cols[4], y, "swap that motor's two wires", 14, "#333")
    if i == 1: text(cols[4], y, "(the diode stays as drawn)", 14, "#333")
# battery-sense spare space note
box(20, 545, 330, 140, "#fff", "#ccc"); text(32, 568, "NOTES", 14, "#111", "start", True)
for i, s in enumerate(["Motor current: thick wire from the battery.", "Keep motor wires away from the IMU + I2C.", "10k keeps motors OFF while the C3 boots.", "Test one MOSFET with a motor first", "(your GGR38 rating is unconfirmed)."]):
    text(32, 590 + i * 19, s, 13, "#333")

svg = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}"><rect width="{W}" height="{H}" fill="#fff"/>' + "".join(o) + "</svg>"
open("wiring_diagram.svg", "w", encoding="utf-8").write(svg)
print("ok")
