"""Draws the wiring / pinout pictures in docs/img/ (SVG).  Run:  python docs/make_pinouts.py
Pin numbers are taken from the sketches (#define PIN_..., MOTOR_PIN[], AXIS_PIN[], OUT_PIN[]). Re-run after a pin change."""
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "img")
os.makedirs(OUT, exist_ok=True)

COL = {"i2c": "#1f6fd1", "motor": "#e0620d", "pwr": "#d11f1f", "gnd": "#222222", "adc": "#16a34a",
       "buzz": "#8b3fd1", "btn": "#6b7280", "uart": "#0e9aa7", "free": "#b8b8b8", "bad": "#b8b8b8"}
LEGEND = [("i2c", "I2C"), ("motor", "motor / output"), ("adc", "analog in"), ("uart", "serial (UART)"),
          ("buzz", "buzzer"), ("btn", "button / switch"), ("pwr", "power"), ("gnd", "ground")]


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def txt(x, y, s, size=13, color="#222", anchor="start", bold=False):
    w = ' font-weight="bold"' if bold else ""
    return '<text x="%.1f" y="%.1f" font-size="%d" fill="%s" text-anchor="%s"%s>%s</text>' % (x, y, size, color, anchor, w, esc(s))


def pinout(fname, title, board, left, right, pins, notes, board_note):
    """left/right: physical pin order (top to bottom, USB at the top). pins: name -> (kind, label)."""
    rows = max(len(left), len(right))
    pitch, top, bw = 34, 160, 190
    W, cx = 1180, 590
    H = top + rows * pitch + 70 + 20 * len(notes) + 60
    bx0, bx1 = cx - bw / 2, cx + bw / 2
    s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" font-family="Segoe UI, Arial, sans-serif">' % (W, H, W, H),
         '<rect width="100%" height="100%" fill="#ffffff"/>',
         txt(24, 36, title, 22, "#111", bold=True), txt(24, 60, board_note, 13, "#555")]
    # legend
    lx = 24
    for k, name in LEGEND:
        s.append('<rect x="%d" y="74" width="14" height="14" rx="3" fill="%s"/>' % (lx, COL[k]))
        s.append(txt(lx + 20, 86, name, 12, "#333"))
        lx += 34 + 7 * len(name)
    # board
    s.append('<rect x="%.1f" y="%d" width="%d" height="%d" rx="12" fill="#1d2b3a" stroke="#0b141d" stroke-width="2"/>'
             % (bx0, top - 50, bw, rows * pitch + 60))
    s.append('<rect x="%.1f" y="%d" width="44" height="22" rx="5" fill="#c9ced6" stroke="#888"/>' % (cx - 22, top - 64))
    s.append(txt(cx, top - 48, "USB", 11, "#333", "middle", True))
    s.append(txt(cx, top - 22, board, 14, "#fff", "middle", True))
    s.append(txt(cx, top + rows * pitch - 4, "top view", 11, "#9fb3c8", "middle"))
    for side, plist in ((-1, left), (1, right)):
        for i, p in enumerate(plist):
            y = top + i * pitch
            px = bx0 + 16 if side < 0 else bx1 - 16
            kind, label = pins.get(p, ("free", "not used"))
            c = COL[kind]
            used = kind not in ("free", "bad")
            s.append('<circle cx="%.1f" cy="%d" r="7" fill="%s" stroke="#d9b24a" stroke-width="2.5"/>' % (px, y, c if used else "#3a4a5c"))
            s.append(txt(px - side * 14, y + 4, p, 12, "#fff", "start" if side < 0 else "end", True))
            # wire + label
            x_end = 60 if side < 0 else W - 60
            wx = bx0 - 6 if side < 0 else bx1 + 6
            dash = '' if used else ' stroke-dasharray="4 4"'
            s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" stroke-width="%d"%s/>' % (px, y, wx + side * 40, y, c, 4 if used else 2, dash))
            lab_x = wx + side * 48
            s.append('<rect x="%.1f" y="%d" width="%d" height="24" rx="5" fill="%s" opacity="%s"/>'
                     % (lab_x if side > 0 else lab_x - 330, y - 12, 330, c, "1" if used else "0.25"))
            s.append(txt(lab_x + side * 8, y + 5, label, 13, "#fff" if used else "#555", "start" if side > 0 else "end", used))
    y = top + rows * pitch + 50
    s.append(txt(24, y, "Notes", 15, "#111", bold=True))
    for n in notes:
        y += 20
        s.append(txt(24, y, "- " + n, 13, "#333"))
    s.append("</svg>")
    open(os.path.join(OUT, fname), "w", encoding="utf-8").write("\n".join(s))


def pinlist(fname, title, board, pins, notes, board_note):
    """Board whose physical order we don't draw: a single list of the used pins (follow the labels printed on the board)."""
    names = list(pins)
    half = (len(names) + 1) // 2
    pinout(fname, title, board, names[:half], names[half:], pins, notes, board_note)


SUPER_L = ["G5", "G6", "G7", "G8", "G9", "G10", "G20", "G21"]
SUPER_R = ["5V", "GND", "3V3", "G4", "G3", "G2", "G1", "G0"]
SUPER_NOTE = "ESP32-C3 SuperMini, USB-C at the top, pin order as on the shield PCBs. Check the labels printed on your board."

# ---------------- SimpleFC (quad, ESCs) ----------------
pinout("simplefc_pinout.svg", "SimpleFC - brushless quad (ESCs)", "SuperMini", SUPER_L, SUPER_R, {
    "G0": ("motor", "M1 rear-right ESC signal (CW)"),
    "G1": ("motor", "M2 front-right ESC signal (CCW)"),
    "G3": ("motor", "M3 rear-left ESC signal (CCW)"),
    "G5": ("motor", "M4 front-left ESC signal (CW)"),
    "G6": ("i2c", "SDA: MPU6050, BMP280, OLED"),
    "G7": ("i2c", "SCL: MPU6050, BMP280, OLED"),
    "G4": ("adc", "battery: pack+ 100k / 10k GND (ratio 11)"),
    "G20": ("buzz", "active buzzer: 1k to NPN base"),
    "G21": ("uart", "DJI O4 RX (OSD, 115200)"),
    "G10": ("uart", "DJI O4 TX (OSD)"),
    "G9": ("btn", "BOOT button (on the board)"),
    "G8": ("free", "not used (LED, strapping pin)"),
    "G2": ("bad", "not used"),
    "5V": ("pwr", "5 V from the ESC BEC / buck"),
    "GND": ("gnd", "GND: ESCs, sensors, battery -"),
    "3V3": ("pwr", "3.3 V to sensors / OLED"),
}, ["ESC signal GND must be joined to the FC GND. Motor order is Betaflight (see motor_layout.svg).",
    "Battery divider: 100 nF from G4 to GND. Pin must stay below 2.5 V: 100k/10k is good up to 6S.",
    "OLED is an SH1106 1.3 in at 0x3C; MPU6050 0x68; BMP280 0x76/0x77. All on one I2C bus at 400 kHz.",
    "Settings: OLED menu with the TX sticks (serial is the backup)."], SUPER_NOTE)

# ---------------- SimpleFC_Brushed ----------------
pinout("brushed_pinout.svg", "SimpleFC_Brushed - 2S brushed micro (MOSFETs)", "SuperMini", SUPER_L, SUPER_R, {
    "G0": ("motor", "M1 rear-right: 100R to MOSFET gate (CW)"),
    "G1": ("motor", "M2 front-right: 100R to MOSFET gate (CCW)"),
    "G3": ("motor", "M3 rear-left: 100R to MOSFET gate (CCW)"),
    "G5": ("motor", "M4 front-left: 100R to MOSFET gate (CW)"),
    "G6": ("i2c", "SDA: MPU6050, BMP280 or GY-63"),
    "G7": ("i2c", "SCL: MPU6050, BMP280 or GY-63"),
    "G4": ("adc", "battery: pack+ 100k / 33k GND (ratio 4.03)"),
    "G20": ("buzz", "passive buzzer: 100R, buzzer, GND"),
    "G21": ("uart", "DJI O4 RX (optional)"),
    "G10": ("uart", "DJI O4 TX (optional)"),
    "G9": ("btn", "BOOT button (on the board)"),
    "G8": ("free", "not used (LED, strapping pin)"),
    "G2": ("bad", "not used"),
    "5V": ("pwr", "5 V from a 2S to 5 V BUCK (never 2S here)"),
    "GND": ("gnd", "GND: MOSFET sources, battery -"),
    "3V3": ("pwr", "3.3 V to the barometer"),
}, ["Each motor: gate 10k to GND (motor off while booting), source to GND, drain to motor -, motor + to battery +.",
    "Schottky (SS14) across each motor: cathode to battery +. 470-1000 uF 16 V on the battery near the MOSFETs.",
    "GY-63 (MS5611): PS to VCC, CSB to GND. Battery divider: 100 nF from G4 to GND.",
    "No OLED: settings over USB serial (help, ?, name=value, save). Full circuit: SimpleFC_Brushed/pcb/wiring_diagram.svg."], SUPER_NOTE)

# ---------------- SimpleTX ----------------
pinout("simpletx_pinout.svg", "SimpleTX - two-button ESP-NOW transmitter", "SuperMini", SUPER_L, SUPER_R, {
    "G1": ("adc", "THROTTLE pot wiper (left stick up/down)"),
    "G0": ("adc", "YAW pot wiper (left stick left/right)"),
    "G4": ("adc", "PITCH pot wiper (right stick up/down)"),
    "G3": ("adc", "ROLL pot wiper (right stick left/right)"),
    "G5": ("i2c", "OLED SDA (SH1106 1.3 in, 0x3C)"),
    "G6": ("i2c", "OLED SCL"),
    "G7": ("btn", "ARM button to GND"),
    "G20": ("btn", "MODE button to GND (SELF / ALT)"),
    "G21": ("buzz", "passive buzzer: NPN or 100R to GND"),
    "G2": ("adc", "TX battery: batt+ 100k / 100k GND (ratio 2)"),
    "G8": ("free", "not used"),
    "G9": ("btn", "BOOT button (on the board)"),
    "G10": ("free", "not used"),
    "5V": ("pwr", "5 V in (USB or 1S boost)"),
    "GND": ("gnd", "GND: pot ends, buttons, OLED"),
    "3V3": ("pwr", "3.3 V: pot ends, OLED VCC"),
}, ["Each pot: one end to 3V3, other end to GND, wiper to its pin. Pots must be on G0-G4 (ADC1 works with the radio on).",
    "Buttons need no resistor (internal pull-up). Battery divider: 100 nF from G2 to GND.",
    "Sim mode (menu): restarts once as a BLE gamepad for PC simulators."], SUPER_NOTE)

ZERO_NOTE = "ESP32-C3 Zero: only the used pins are drawn, not their physical order. Follow the labels printed on the board."

# ---------------- Transmitter2 ----------------
pinlist("transmitter2_pinout.svg", "Transmitter2 - ESP-NOW transmitter with 3-position switch", "C3 Zero", {
    "G1": ("adc", "THROTTLE pot wiper"),
    "G0": ("adc", "YAW pot wiper"),
    "G4": ("adc", "PITCH pot wiper"),
    "G3": ("adc", "ROLL pot wiper"),
    "G2": ("adc", "TX battery divider (ratio 2.03 for 1S)"),
    "G5": ("i2c", "OLED SDA (SH1106, 0x3C)"),
    "G6": ("i2c", "OLED SCL"),
    "G7": ("btn", "ARM button to GND"),
    "G20": ("btn", "3-pos switch POS1 leg (DISARM)"),
    "G8": ("btn", "3-pos switch POS3 leg (ARM + extra)"),
    "G21": ("buzz", "passive buzzer: 1k to NPN base"),
    "GND": ("gnd", "GND: switch middle leg, pots, OLED"),
    "3V3": ("pwr", "3.3 V: pot ends, OLED VCC"),
    "5V": ("pwr", "5 V in"),
}, ["3-position ON-OFF-ON switch: middle leg to GND, outer legs to G20 and G8, no resistors.",
    "Not G10: on the C3 Zero it is the RGB LED data line.",
    "Battery divider: batt+ R1 G2 R2 GND, 100 nF from G2 to GND."], ZERO_NOTE)

# ---------------- FlightController (old, C3 Zero) ----------------
pinlist("flightcontroller_pinout.svg", "FlightController - old C3 Zero quad (GPS, flow, RTH)", "C3 Zero", {
    "G0": ("motor", "M1 rear-right ESC signal (CW)"),
    "G1": ("motor", "M2 front-right ESC signal (CCW)"),
    "G3": ("motor", "M3 rear-left ESC signal (CCW)"),
    "G8": ("motor", "M4 front-left ESC signal (CW)"),
    "G6": ("i2c", "SDA: MPU6050, BMP280, OLED, VL53L1X, compass"),
    "G7": ("i2c", "SCL: same devices"),
    "G4": ("adc", "battery: pack+ 100k / 10k GND"),
    "G20": ("buzz", "active buzzer: 1k to NPN base"),
    "G21": ("uart", "GPS TX through 1k (FC RX)"),
    "G10": ("uart", "GPS RX (FC TX)"),
    "G5": ("uart", "MTF-01 optical flow TX (FC RX)"),
    "G9": ("btn", "BOOT button (OLED page)"),
    "G2": ("bad", "do NOT use: shorted to the crystal can"),
    "5V": ("pwr", "5 V from a BEC / buck (not a 7805)"),
    "GND": ("gnd", "GND: ESCs, GPS, sensors"),
    "3V3": ("pwr", "3.3 V to sensors"),
}, ["GPS u-blox (UBX) 1k resistor in the TX wire: the boot ROM drives G21 for a moment at power-up.",
    "Compass QMC5883L (0x0D) or HMC5883L (0x1E) on the I2C bus. VL53L1X faces down (alt hold).",
    "Motor 4 is on G8 on this board (G4 is the battery divider)."], ZERO_NOTE)

# ---------------- NodeMCU FC ----------------
pinlist("nodemcu_pinout.svg", "ESP32_NodeMCU_FC - ESP32 DevKit / NodeMCU-32S", "ESP32 DevKit", {
    "IO18": ("motor", "Output 1: M1 rear-right (plane: aileron)"),
    "IO19": ("motor", "Output 2: M2 front-right (elevator)"),
    "IO23": ("motor", "Output 3: M3 rear-left (throttle ESC)"),
    "IO25": ("motor", "Output 4: M4 front-left (rudder)"),
    "IO21": ("i2c", "SDA: MPU6050 + OLED"),
    "IO22": ("i2c", "SCL: MPU6050 + OLED"),
    "IO32": ("uart", "PPM receiver signal (3.3 V only)"),
    "IO34": ("adc", "battery: pack+ 100k / 10k GND"),
    "IO26": ("buzz", "active buzzer: 1k to NPN base"),
    "IO13": ("btn", "page button to GND"),
    "VIN": ("pwr", "5 V from ESC BEC (not with USB)"),
    "GND": ("gnd", "GND common"),
    "3V3": ("pwr", "3.3 V to sensors"),
}, ["PPM from a 5 V receiver needs a divider: 10k series, 20k to GND. Or use the ESP-NOW link (no receiver).",
    "Do not use IO6-IO11 (flash), IO12 / IO15 (strapping), IO2 (strapping + LED).",
    "Battery divider: 100 nF from IO34 to GND."], "ESP32 DevKit: only the used pins are drawn, not their physical order. The labels are the IO numbers.")

# ---------------- motor layout ----------------
s = ['<svg xmlns="http://www.w3.org/2000/svg" width="560" height="520" viewBox="0 0 560 520" font-family="Segoe UI, Arial, sans-serif">',
     '<rect width="100%" height="100%" fill="#fff"/>', txt(24, 36, "Motor order (Betaflight), top view, props in", 20, "#111", bold=True),
     '<line x1="160" y1="140" x2="400" y2="380" stroke="#555" stroke-width="10"/>',
     '<line x1="400" y1="140" x2="160" y2="380" stroke="#555" stroke-width="10"/>',
     '<rect x="235" y="215" width="90" height="90" rx="10" fill="#1d2b3a"/>', txt(280, 265, "FC", 18, "#fff", "middle", True),
     '<polygon points="280,70 262,100 298,100" fill="#16a34a"/>', txt(280, 64, "FRONT", 14, "#16a34a", "middle", True)]
mot = [("M4", 160, 140, "CW", "front-left"), ("M2", 400, 140, "CCW", "front-right"),
       ("M3", 160, 380, "CCW", "rear-left"), ("M1", 400, 380, "CW", "rear-right")]
for name, x, y, d, where in mot:
    c = "#e0620d" if d == "CW" else "#1f6fd1"
    s.append('<circle cx="%d" cy="%d" r="62" fill="none" stroke="%s" stroke-width="3" stroke-dasharray="10 6"/>' % (x, y, c))
    s.append('<circle cx="%d" cy="%d" r="26" fill="%s"/>' % (x, y, c))
    s.append(txt(x, y + 6, name, 18, "#fff", "middle", True))
    # rotation arrow head on the circle
    if d == "CW":
        s.append('<polygon points="%d,%d %d,%d %d,%d" fill="%s"/>' % (x + 62, y + 10, x + 54, y - 6, x + 70, y - 6, c))
    else:
        s.append('<polygon points="%d,%d %d,%d %d,%d" fill="%s"/>' % (x - 62, y + 10, x - 70, y - 6, x - 54, y - 6, c))
    s.append(txt(x, y + 90, "%s  %s" % (d, where), 14, c, "middle", True))
s.append(txt(24, 500, "SimpleFC / Brushed: M1 G0, M2 G1, M3 G3, M4 G5.  Old FC: M4 on G8.  NodeMCU: IO18 / 19 / 23 / 25.", 12, "#333"))
s.append("</svg>")
open(os.path.join(OUT, "motor_layout.svg"), "w", encoding="utf-8").write("\n".join(s))
print("written to", OUT, sorted(os.listdir(OUT)))
