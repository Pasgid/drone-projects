"""Builds docs/img/setup_guide_brushed.gif: a step-by-step setup guide for SimpleFC_Brushed.
Each step is an HTML card (picture + a few lines), screenshotted with headless Edge/Chrome, then joined with Pillow.
Run:  python docs/make_setup_gif.py      (needs:  pip install pillow)"""
import os, subprocess, tempfile, html
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG = os.path.join(ROOT, "docs", "img")
OUT = os.path.join(IMG, "setup_guide_brushed.gif")
W, H = 1000, 720
BROWSERS = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe"]

def url(p): return "file:///" + p.replace("\\", "/").replace(" ", "%20")

# (title, picture or None, crop [x, y, w, h] of the picture in its own pixels or None, lines, seconds)
STEPS = [
    ("SimpleFC_Brushed setup", "docs/img/motor_layout.svg", (40, 50, 480, 410), [
        "<b>You need:</b> ESP32-C3 SuperMini, MPU6050, BMP280 or GY-63 barometer",
        "4 logic-level N-MOSFETs + 4 SS14 diodes + 100R / 10k resistors",
        "2S to 5 V buck, passive buzzer, 100k / 33k divider, 470-1000 uF 16 V capacitor",
        "4 x 820 coreless motors, 2S battery, SimpleTX transmitter",
        "<i>Full pin list: WIRING.md</i>"], 4.0),
    ("1. Power", "SimpleFC_Brushed/pcb/wiring_diagram.svg", (10, 62, 680, 320), [
        "Set the buck to <b>5.0 V before</b> connecting it, then buck OUT to the <b>5V</b> pin.",
        "<b style='color:#d11f1f'>Never put 2S on the 5V pin.</b> Big capacitor on the battery, all GNDs joined."], 4.5),
    ("2. Motors (one MOSFET each)", "SimpleFC_Brushed/pcb/wiring_diagram.svg", (360, 375, 750, 315), [
        "G0 = M1, G1 = M2, G3 = M3, G5 = M4: pin, 100R, MOSFET gate. 10k from gate to GND.",
        "Source to GND, drain to motor -, motor + to battery +. Diode across the motor (stripe to +)."], 4.5),
    ("3. Sensors (I2C)", "SimpleFC_Brushed/pcb/wiring_diagram.svg", (460, 62, 650, 320), [
        "<b>G6 = SDA, G7 = SCL</b> to the MPU6050 and the barometer (both on the same two wires).",
        "GY-63: PS to VCC, CSB to GND, or it blocks the bus."], 4.0),
    ("4. Battery sense + buzzer", "docs/img/brushed_pinout.svg", (40, 120, 1100, 290), [
        "Battery: pack+ 100k, <b>G4</b>, 33k to GND, plus 100 nF from G4 to GND.",
        "Passive buzzer: <b>G20</b>, 100R, buzzer, GND."], 4.0),
    ("5. Motor order and spin", "docs/img/motor_layout.svg", None, [
        "Front is up. M1 and M4 spin clockwise, M2 and M3 counter-clockwise (props in).",
        "Swap a brushed motor's two wires to reverse it."], 4.0),
    ("6. First power-up: PROPS OFF", ["> st", "SAFE SELF R+0.2 P-0.1 | 8.10V 2S | -60dBm 100% ... | baro BMP280 +0.00m ...",
        "> m1", "spinning m1 for 2 s, PROPS OFF! (stop = stop now)", "> stop", "motors off"], None, [
        "USB to the PC, Serial Monitor at <b>115200</b>. Type <code>help</code> for the list.",
        "<code>st</code>  = sensors found?   <code>?</code> = all settings",
        "<code>m1</code> <code>m2</code> <code>m3</code> <code>m4</code> = spin one motor 2 s: check its place and direction",
        "<code>stop</code> = stop at once"], 5.0),
    ("7. Settings", ["> cells=2", "cells = 2 (type 'save' to keep)", "> vRatio=4.03", "vRatio = 4.03 (type 'save' to keep)",
        "> cal", "LEVEL CAL: rollTrim 0.12 pitchTrim -0.30", "> save", "saved"], None, [
        "<code>cells=2</code>   <code>vRatio=4.03</code>  (fine-tune until the voltage matches a meter)",
        "Put the quad on a flat surface: <code>cal</code>  (level calibration)",
        "<code>save</code>  - nothing is kept until you save"], 4.5),
    ("8. Fly", "docs/img/simpletx_pinout.svg", (40, 120, 1100, 290), [
        "Props on. TX on, battery in, wait for the ready beep.",
        "<b>ARM</b> button (TX G7), throttle up gently. <b>MODE</b> button (G20) = SELF-LEVEL / ALT HOLD.",
        "After landing: USB, type <code>D</code> to dump the flight log (or ask Claude to \"read flight\")."], 5.0),
]

CSS = """body{margin:0;width:%dpx;height:%dpx;background:#f4f6f9;font-family:'Segoe UI',Arial,sans-serif;color:#1b1f24;overflow:hidden}
.card{position:absolute;inset:18px;background:#fff;border-radius:16px;box-shadow:0 2px 10px #0002;display:flex;flex-direction:column;padding:22px 30px}
h1{margin:0 0 4px;font-size:32px} .bar{height:6px;border-radius:3px;background:#e3e7ed;margin:6px 0 14px}
.bar i{display:block;height:100%%;border-radius:3px;background:#e0620d}
.pic{flex:1;min-height:0;display:flex;align-items:center;justify-content:center}
.pic img{max-width:100%%;max-height:100%%;object-fit:contain}
.txt{font-size:21px;line-height:1.5} .big .txt{font-size:25px;line-height:1.9;margin-top:30px}
code{background:#1d2b3a;color:#fff;border-radius:5px;padding:1px 8px;font-size:0.95em}
.term{flex:1;margin:4px 0 16px;background:#0f1720;color:#cfe3d0;border-radius:10px;padding:16px 22px;font:17px/1.55 Consolas,monospace;white-space:pre;overflow:hidden}
.term div:nth-child(odd){color:#7ee787}
.step{position:absolute;right:34px;top:30px;color:#8a94a3;font-size:18px}""" % (W, H)

def card(i, title, pic, crop, lines):
    body = "".join("<div>%s</div>" % l for l in lines)
    picture = ""
    if isinstance(pic, list):          # a serial-monitor mock-up: typed command, reply, ...
        picture = '<div class="term">%s</div>' % "".join("<div>%s</div>" % html.escape(l) for l in pic)
    elif pic:
        p = os.path.join(ROOT, pic)
        import re
        head = open(p, encoding="utf-8").read(400)
        fw, fh = (float(v) for v in re.search(r'width="([\d.]+)"[^>]*height="([\d.]+)"', head).groups())
        x, y, w, h = crop or (0, 0, fw, fh)
        picture = ('<div class="pic"><svg viewBox="%g %g %g %g" style="width:100%%;height:100%%" preserveAspectRatio="xMidYMid meet">'
                   '<defs><clipPath id="c"><rect x="%g" y="%g" width="%g" height="%g"/></clipPath></defs>'
                   '<image clip-path="url(#c)" href="%s" width="%g" height="%g"/></svg></div>') % (x, y, w, h, x, y, w, h, url(p), fw, fh)
    return ("<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body><div class='card %s'>"
            "<span class='step'>%d / %d</span><h1>%s</h1><div class='bar'><i style='width:%d%%'></i></div>%s<div class='txt'>%s</div>"
            "</div></body></html>") % (CSS, "" if pic else "big", i + 1, len(STEPS), html.escape(title),
                                      100 * (i + 1) // len(STEPS), picture, body)

browser = next(b for b in BROWSERS if os.path.exists(b))
tmp = tempfile.mkdtemp(prefix="setupgif_")
frames, durations = [], []
for i, (title, pic, _crop, lines, secs) in enumerate(STEPS):
    h = os.path.join(tmp, "f%02d.html" % i); png = os.path.join(tmp, "f%02d.png" % i)
    open(h, "w", encoding="utf-8").write(card(i, title, pic, _crop, lines))
    subprocess.run([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--allow-file-access-from-files",
                    "--force-device-scale-factor=1", "--window-size=%d,%d" % (W, H), "--screenshot=" + png, url(h)],
                   check=True, capture_output=True, timeout=60)
    frames.append(Image.open(png).convert("RGB").crop((0, 0, W, H)))
    durations.append(int(secs * 1000))
pal = [f.quantize(colors=255, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
pal[0].save(OUT, save_all=True, append_images=pal[1:], duration=durations, loop=0, optimize=True)
print("wrote", OUT, "%.0f kB" % (os.path.getsize(OUT) / 1024), "frames", len(frames), "pngs in", tmp)
