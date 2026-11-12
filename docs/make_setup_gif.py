"""Builds the animated setup guides in docs/img/setup_guide_*.gif.
Each step is an HTML card (picture + a few lines), screenshotted with headless Edge/Chrome, then joined with Pillow.
Run:  python docs/make_setup_gif.py            (all guides)
      python docs/make_setup_gif.py brushed    (one guide: brushed, simple, tx2, oldfc)
Also writes a vertical 1080x1920 MP4 of each guide (docs/video/, ready for TikTok / Reels / Shorts) with LINK at the bottom.
Needs:  pip install pillow imageio-ffmpeg"""
import os, re, sys, subprocess, tempfile, html
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG = os.path.join(ROOT, "docs", "img")
W, H = 1000, 720
VID = os.path.join(ROOT, "docs", "video")
LINK = "github.com/Pasgid/drone-projects"            # shown at the bottom of every video frame
VW, VH, FPS = 1080, 1920, 30
BROWSERS = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe"]

def url(p): return "file:///" + p.replace("\\", "/").replace(" ", "%20")

WD = "SimpleFC_Brushed/pcb/wiring_diagram.svg"
PIN_ROWS = (40, 96, 1100, 320)          # the pin rows of a docs/img/*_pinout.svg (no title / notes)
def oled(*lines): return {"oled": list(lines)}

# A step: (title, picture, crop, lines, seconds)
#   picture: a file path (SVG), a list of strings (serial-monitor mock-up: typed command, reply, ...),
#            oled(...) (a TX / FC OLED mock-up), or None (text only)
#   crop:    (x, y, w, h) of the SVG in its own viewBox units, or None for all of it
GUIDES = {}

GUIDES["brushed"] = ("setup_guide_brushed.gif", [
    ("SimpleFC_Brushed setup", "docs/img/motor_layout.svg", (40, 50, 480, 410), [
        "<b>You need:</b> ESP32-C3 SuperMini, MPU6050, BMP280 or GY-63 barometer",
        "4 logic-level N-MOSFETs + 4 SS14 diodes + 100R / 10k resistors",
        "2S to 5 V buck, passive buzzer, 100k / 33k divider, 470-1000 uF 16 V capacitor",
        "4 x 820 coreless motors, 2S battery, SimpleTX transmitter",
        "<i>Full pin list: WIRING.md</i>"], 4.0),
    ("1. Power", WD, (10, 62, 680, 320), [
        "Set the buck to <b>5.0 V before</b> connecting it, then buck OUT to the <b>5V</b> pin.",
        "<b style='color:#d11f1f'>Never put 2S on the 5V pin.</b> Big capacitor on the battery, all GNDs joined."], 4.5),
    ("2. Motors (one MOSFET each)", WD, (360, 375, 750, 315), [
        "G0 = M1, G1 = M2, G3 = M3, G5 = M4: pin, 100R, MOSFET gate. 10k from gate to GND.",
        "Source to GND, drain to motor -, motor + to battery +. Diode across the motor (stripe to +)."], 4.5),
    ("3. Sensors (I2C)", WD, (460, 62, 650, 320), [
        "<b>G6 = SDA, G7 = SCL</b> to the MPU6050 and the barometer (both on the same two wires).",
        "GY-63: PS to VCC, CSB to GND, or it blocks the bus."], 4.0),
    ("4. Battery sense + buzzer", "docs/img/brushed_pinout.svg", PIN_ROWS, [
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
    ("8. Fly", "docs/img/simpletx_pinout.svg", PIN_ROWS, [
        "Props on. TX on, battery in, wait for the ready beep.",
        "<b>ARM</b> button (TX G7), throttle up gently. <b>MODE</b> button (G20) = SELF-LEVEL / ALT HOLD.",
        "After landing: USB, type <code>D</code> to dump the flight log (or ask Claude to \"read flight\")."], 5.0),
])

GUIDES["simple"] = ("setup_guide_simpletx_simplefc.gif", [
    ("SimpleTX + SimpleFC setup", "docs/img/motor_layout.svg", (40, 50, 480, 410), [
        "<b>TX:</b> ESP32-C3 SuperMini, 2 gimbals (4 pots), SH1106 1.3\" OLED, 2 push buttons, passive buzzer",
        "<b>FC (the receiver):</b> SuperMini, MPU6050, BMP280, SH1106 OLED, active buzzer, 4 ESCs, 5 V BEC / buck",
        "The radio is ESP-NOW between the two boards: no separate receiver.",
        "<i>Full pin list: WIRING.md</i>"], 4.5),
    ("1. Transmitter wiring", "docs/img/simpletx_pinout.svg", PIN_ROWS, [
        "Pots: <b>G1</b> throttle, <b>G0</b> yaw, <b>G4</b> pitch, <b>G3</b> roll. Each pot: ends on 3V3 and GND.",
        "OLED G5 / G6. <b>ARM</b> button G7, <b>MODE</b> button G20 (to GND). Buzzer G21. TX battery G2 (100k / 100k)."], 5.0),
    ("2. Flight controller wiring", "docs/img/simplefc_pinout.svg", PIN_ROWS, [
        "ESC signals: <b>G0</b> M1, <b>G1</b> M2, <b>G3</b> M3, <b>G5</b> M4. ESC signal GND to FC GND.",
        "I2C G6 / G7: MPU6050 + BMP280 + OLED. Battery G4 (100k / 10k). Buzzer G20 through an NPN."], 5.0),
    ("3. Power and shield", "SimpleFC/pcb/assembly_top.svg", None, [
        "5 V from a buck / BEC into the <b>5V</b> pin: <b>not a 7805</b> (it overheated and browned out in flight).",
        "470-1000 uF on the battery leads, 100-470 uF at the FC 5 V input. Shield PCB: SimpleFC/pcb."], 4.5),
    ("4. Motor order and spin", "docs/img/motor_layout.svg", None, [
        "Front is up. M1 and M4 spin clockwise, M2 and M3 counter-clockwise (props in).",
        "Swap any two of a brushless motor's three wires to reverse it."], 4.0),
    ("5. Calibrate the TX sticks", oled("STICK CALIBRATION", "Move BOTH sticks to", "all ends, then press", "T1830 95%  R2010 92%",
                                        "P2240 97%  Y1950 90%"), None, [
        "TX on, <b>long press ARM</b> = MENU. Right stick up/down = move, left/right = change.",
        "<b>Calibrate sticks</b>: stir both sticks to all ends, press; centre them, press. Long press = save &amp; exit."], 5.0),
    ("6. ESC calibration + motor test: PROPS OFF", oled("SETUP", "> TRIM / MOTORS", "  BATTERY", "  LEVEL CALIBRATE",
                                                         "  MOTOR TEST", "  ESC CALIBRATE"), None, [
        "FC SETUP: <b>throttle LOW + yaw LEFT + pitch FORWARD</b> 1 s. Pitch = cursor, yaw right = run, yaw left = back.",
        "<b>ESC CALIBRATE</b> (choose it twice), unplug battery + USB, power from the battery: MAX 5 s, then MIN 5 s.",
        "<b>MOTOR TEST</b>: pitch picks M1-M4, hold roll right = spin. Check each motor's place and direction."], 6.0),
    ("7. Level calibration and trim", "docs/img/motor_layout.svg", (40, 50, 480, 410), [
        "Quad on a <b>level</b> surface: <b>throttle LOW + yaw full LEFT + pitch full BACK</b> for 2 s.",
        "Drifts in flight? Disarmed: <b>throttle HIGH</b>, push the right stick the way it should go instead.",
        "0.1 deg per beep, saved when you let go."], 5.0),
    ("8. Fly", oled("ESP-NOW ch 13", "FC 15.8V  LQ 100%", "SELF", "ARMED"), None, [
        "Props on, battery in. <b>ARM</b> button with the throttle low. <b>MODE</b> button: SELF-LEVEL / ALT HOLD.",
        "Radio lost > 1 s: lands itself in ALT HOLD (with a good altitude), otherwise disarms.",
        "After landing: USB, type <code>D</code> to dump the flight log (or ask Claude to \"read flight\")."], 5.5),
])

GUIDES["tx2"] = ("setup_guide_transmitter2.gif", [
    ("Transmitter2 setup", "docs/img/transmitter2_pinout.svg", PIN_ROWS, [
        "<b>You need:</b> ESP32-C3 Zero, 2 gimbals (4 pots), SH1106 1.3\" OLED, 1 push button,",
        "ON-OFF-ON 3-position switch, passive buzzer + NPN, 1S cell + divider. Flies the old C3 Zero FC (and SimpleFC).",
        "<i>Full pin list: WIRING.md</i>"], 4.5),
    ("1. Sticks and OLED", "docs/img/transmitter2_pinout.svg", PIN_ROWS, [
        "Pots: <b>G1</b> throttle, <b>G3</b> roll, <b>G4</b> pitch, <b>G0</b> yaw (ADC1 pins only). Ends on 3V3 and GND.",
        "OLED SDA <b>G5</b>, SCL <b>G6</b>. ARM button <b>G7</b> to GND."], 4.5),
    ("2. Switch, buzzer, battery", "docs/img/transmitter2_pinout.svg", PIN_ROWS, [
        "3-position switch: <b>middle leg GND</b>, outer legs <b>G20</b> (POS1) and <b>G8</b> (POS3). No resistors.",
        "Positions backwards? Swap the outer wires. Not G10: it drives the C3 Zero's RGB LED.",
        "Buzzer G21 through 1k to an NPN. TX battery divider on G2 (ratio about 2 for 1S)."], 5.0),
    ("3. Calibrate the sticks", oled("STICK CALIBRATION", "Move BOTH sticks to", "all ends, then press", "T 95%  R 92%", "P 97%  Y 90%"), None, [
        "<b>Long press</b> the button (disarmed) = MENU. Right stick up/down = move, left/right = change.",
        "<b>Calibrate sticks</b>: move both sticks to every end, press, centre them, press.",
        "Also here: expo, rates, deadbands, stick filter, reverse, TX battery ratio. Long press = save &amp; exit."], 5.5),
    ("4. Flight modes and arming", oled("POS1  SELF STAB", "POS2  ALT HOLD", "POS3  POS HOLD", "button = ARM"), None, [
        "The switch picks the FC's flight mode: <b>POS1 SELF</b>, <b>POS2 ALT HOLD</b>, <b>POS3 POS HOLD</b>.",
        "<b>Short press</b> = ARM / DISARM (throttle low). Armed, <b>hold 2 s</b> = RETURN TO HOME on / off (GPS FC).",
        "The header shows the FC battery from telemetry; the buzzer beeps when it is low."], 5.5),
    ("5. Sim mode (PC simulators)", oled("SIM MODE (BT)", "ESP-NOW TX Sim", "paired", "radio OFF"), None, [
        "MENU &gt; <b>Sim mode (BT)</b>: the TX restarts as a Bluetooth gamepad \"ESP-NOW TX Sim\".",
        "Pair it in Windows, then pick it in the simulator. One boot only: power-cycle to fly again."], 4.5),
])

GUIDES["oldfc"] = ("setup_guide_c3zero_fc.gif", [
    ("C3 Zero FC (FlightController) setup", "docs/img/flightcontroller_pinout.svg", PIN_ROWS, [
        "<b>You need:</b> ESP32-C3 Zero, MPU6050, BMP280, SH1106 OLED, 4 ESCs, 5 V buck / BEC, active buzzer",
        "Options: u-blox GPS + QMC5883L / HMC5883L compass, MTF-01 optical flow, VL53L1X rangefinder.",
        "Flown with Transmitter2. <i>Full pin list: WIRING.md</i>"], 4.5),
    ("1. Motors, sensors, power", "docs/img/flightcontroller_pinout.svg", PIN_ROWS, [
        "ESC signals: <b>G0</b> M1, <b>G1</b> M2, <b>G3</b> M3, <b>G8</b> M4 (G4 is the battery divider, 100k / 10k).",
        "I2C <b>G6 / G7</b>: MPU6050, BMP280, OLED, VL53L1X, compass. Buzzer G20 through an NPN.",
        "<b style='color:#d11f1f'>Never use G2</b>: it is shorted to the crystal can on this board."], 5.0),
    ("2. GPS, compass, optical flow", "docs/img/flightcontroller_pinout.svg", PIN_ROWS, [
        "GPS TX through a <b>1k resistor</b> to <b>G21</b>, GPS RX to <b>G10</b>. Compass on the I2C wires.",
        "MTF-01 optical flow TX to <b>G5</b>. VL53L1X faces down (keep the battery lead out of its view)."], 4.5),
    ("3. Motor order and spin", "docs/img/motor_layout.svg", None, [
        "Front is up. M1 and M4 spin clockwise, M2 and M3 counter-clockwise (props in).",
        "Swap any two of a brushless motor's three wires to reverse it."], 4.0),
    ("4. The SETUP menu", oled("SETUP", "> VEHICLE", "  SELF-LEVEL", "  LEVEL TRIM", "  GPS/COMPASS", "  TX CHECK"), None, [
        "Open (disarmed): BOOT button to the SETUP page, or <b>throttle LOW + yaw LEFT + pitch FORWARD</b> 1 s.",
        "Pitch = cursor, roll = value -/+ (throttle high = x10), yaw right = open / run, yaw left = back.",
        "<b>*</b> in the title = not saved: choose <b>SAVE</b>."], 5.5),
    ("5. TX CHECK, ESCs, OUTPUT TEST: PROPS OFF", oled("TX CHECK", "CH1 1000-2000 OK", "CH2 1000-2000 OK", "CH3 1000-2000 OK",
                                                        "CH4 1000-2000 OK", "CH6 1000-2000 OK"), None, [
        "<b>TX CHECK</b>: move every stick and the switch to both ends until each channel says OK.",
        "<b>ESC CALIBRATE</b>, then <b>OUTPUT TEST</b>: pitch picks the output, hold roll right = spin slowly.",
        "Check each motor's place and direction against the motor-order picture."], 5.5),
    ("6. Level and compass calibration", oled("LEVEL TRIM", "> CALIBRATE", "  ROLL TRIM  +0.0", "  PITCH TRIM +0.0"), None, [
        "LEVEL TRIM &gt; <b>CALIBRATE</b> on a level surface, hold still. Drift right / forward: lower the trim.",
        "GPS/COMPASS &gt; <b>COMPASS CAL</b>: turn the quad through every direction for 45 s.",
        "Then <b>SAVE</b>."], 5.0),
    ("7. Fly", oled("SELF  15.6V", "GPS 9 sats HOME", "ARMED"), None, [
        "Switch: <b>POS1 SELF</b>, <b>POS2 ALT HOLD</b>, <b>POS3 POS HOLD</b>. ARM = TX button, throttle low.",
        "With a GPS fix the home point is set when you arm. Hold the TX button 2 s = <b>RETURN TO HOME</b>.",
        "Radio lost > 1 s: returns home or lands (ALT / POS HOLD), otherwise disarms."], 5.5),
])

CSS = """body{margin:0;width:%dpx;height:%dpx;background:#f4f6f9;font-family:'Segoe UI',Arial,sans-serif;color:#1b1f24;overflow:hidden}
.card{position:absolute;inset:18px;background:#fff;border-radius:16px;box-shadow:0 2px 10px #0002;display:flex;flex-direction:column;padding:22px 30px}
h1{margin:0 0 4px;font-size:32px} .bar{height:6px;border-radius:3px;background:#e3e7ed;margin:6px 0 14px}
.bar i{display:block;height:100%%;border-radius:3px;background:#e0620d}
.pic{flex:1;min-height:0;display:flex;align-items:center;justify-content:center}
.txt{font-size:21px;line-height:1.5} .big .txt{font-size:25px;line-height:1.9;margin-top:30px}
code{background:#1d2b3a;color:#fff;border-radius:5px;padding:1px 8px;font-size:0.95em}
.term{flex:1;margin:4px 0 16px;background:#0f1720;color:#cfe3d0;border-radius:10px;padding:16px 22px;font:17px/1.55 Consolas,monospace;white-space:pre;overflow:hidden}
.term div:nth-child(odd){color:#7ee787}
.oledwrap{flex:1;display:flex;align-items:center;justify-content:center;margin-bottom:14px}
.oledpcb{background:#1d4f8a;padding:18px 22px 26px;border-radius:12px;box-shadow:0 3px 8px #0003}
.oled{width:512px;height:256px;background:#05070a;border:3px solid #333;padding:14px 18px;box-sizing:border-box;
 font:600 25px/1.3 Consolas,monospace;color:#a9d8ff;white-space:pre;overflow:hidden}
.oled div:first-child{background:#a9d8ff;color:#05070a;padding:0 6px;margin:0 -6px 4px}
.step{position:absolute;right:34px;top:30px;color:#8a94a3;font-size:18px}""" % (W, H)

def svg_size(p):
    head = open(p, encoding="utf-8").read(600)
    m = re.search(r'viewBox="([\d.\s-]+)"', head)
    if m:
        v = [float(x) for x in m.group(1).split()]
        return v[2], v[3]
    return tuple(float(v) for v in re.search(r'width="([\d.]+)"[^>]*height="([\d.]+)"', head).groups())

def card(i, n, title, pic, crop, lines):
    body = "".join("<div>%s</div>" % l for l in lines)
    picture = ""
    if isinstance(pic, list):          # serial-monitor mock-up: typed command, reply, ...
        picture = '<div class="term">%s</div>' % "".join("<div>%s</div>" % html.escape(l) for l in pic)
    elif isinstance(pic, dict):        # OLED mock-up
        picture = ('<div class="oledwrap"><div class="oledpcb"><div class="oled">%s</div></div></div>'
                   % "".join("<div>%s</div>" % html.escape(l) for l in pic["oled"]))
    elif pic:
        p = os.path.join(ROOT, pic)
        fw, fh = svg_size(p)
        x, y, w, h = crop or (0, 0, fw, fh)
        picture = ('<div class="pic"><svg viewBox="%g %g %g %g" style="width:100%%;height:100%%" preserveAspectRatio="xMidYMid meet">'
                   '<defs><clipPath id="c"><rect x="%g" y="%g" width="%g" height="%g"/></clipPath></defs>'
                   '<image clip-path="url(#c)" href="%s" width="%g" height="%g"/></svg></div>') % (x, y, w, h, x, y, w, h, url(p), fw, fh)
    return ("<!doctype html><html><head><meta charset='utf-8'><style>%s</style></head><body><div class='card %s'>"
            "<span class='step'>%d / %d</span><h1>%s</h1><div class='bar'><i style='width:%d%%'></i></div>%s<div class='txt'>%s</div>"
            "</div></body></html>") % (CSS, "" if pic else "big", i + 1, n, html.escape(title), 100 * (i + 1) // n, picture, body)

def build(key):
    out_name, steps = GUIDES[key]
    browser = next(b for b in BROWSERS if os.path.exists(b))
    tmp = tempfile.mkdtemp(prefix="setupgif_%s_" % key)
    frames, durations = [], []
    for i, (title, pic, crop, lines, secs) in enumerate(steps):
        h = os.path.join(tmp, "f%02d.html" % i); png = os.path.join(tmp, "f%02d.png" % i)
        open(h, "w", encoding="utf-8").write(card(i, len(steps), title, pic, crop, lines))
        subprocess.run([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--allow-file-access-from-files",
                        "--force-device-scale-factor=1", "--window-size=%d,%d" % (W, H), "--screenshot=" + png, url(h)],
                       check=True, capture_output=True, timeout=60)
        frames.append(Image.open(png).convert("RGB").crop((0, 0, W, H)))
        durations.append(int(secs * 1000))
    pal = [f.quantize(colors=255, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
    out = os.path.join(IMG, out_name)
    pal[0].save(out, save_all=True, append_images=pal[1:], duration=durations, loop=0, optimize=True)
    print("wrote %s  %.0f kB  %d frames  (pngs in %s)" % (out, os.path.getsize(out) / 1024, len(frames), tmp))
    video(key, steps, frames, durations, tmp)

def font(size, bold=True):
    for n in (("segoeuib.ttf" if bold else "segoeui.ttf"), "arialbd.ttf", "arial.ttf"):
        try: return ImageFont.truetype(os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", n), size)
        except OSError: pass
    return ImageFont.load_default()

def centre(d, y, s, f, fill):
    w = d.textlength(s, font=f); d.text(((VW - w) / 2, y), s, font=f, fill=fill)

def video(key, steps, frames, durations, tmp):
    """Vertical MP4: the guide's title on top, the step card in the middle, the link at the bottom."""
    import imageio_ffmpeg
    os.makedirs(VID, exist_ok=True)
    title = steps[0][0].replace(" setup", "")
    lst = os.path.join(tmp, "list.txt")
    with open(lst, "w", encoding="utf-8") as L:
        for i, (fr, ms) in enumerate(zip(frames, durations)):
            v = Image.new("RGB", (VW, VH), "#0f1720"); d = ImageDraw.Draw(v)
            centre(d, 170, "ESP32 DIY DRONE", font(44), "#e0620d")
            centre(d, 235, title, font(58), "#ffffff")
            centre(d, 320, "setup guide  -  step %d of %d" % (i + 1, len(frames)), font(36, False), "#9fb3c8")
            card = fr.resize((VW, round(H * VW / W)), Image.LANCZOS)
            v.paste(card, (0, (VH - card.height) // 2 + 20))
            d.rounded_rectangle((90, 1560, VW - 90, 1680), 30, fill="#1d2b3a")
            centre(d, 1576, "Wiring + code:", font(32, False), "#9fb3c8")
            centre(d, 1614, LINK, font(42), "#7ee787")
            p = os.path.join(tmp, "v%02d.png" % i); v.save(p)
            L.write("file '%s'\nduration %.3f\n" % (p.replace("\\", "/"), ms / 1000))
        L.write("file '%s'\n" % p.replace("\\", "/"))       # concat needs the last frame twice
    out = os.path.join(VID, os.path.splitext(GUIDES[key][0])[0].replace("setup_guide_", "setup_") + ".mp4")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst,
                    "-f", "lavfi", "-i", "anullsrc=r=44100:cl=stereo", "-shortest",
                    "-vf", "fps=%d,format=yuv420p" % FPS, "-c:v", "libx264", "-preset", "slow", "-crf", "20",
                    "-c:a", "aac", "-b:a", "64k", "-movflags", "+faststart", out], check=True)
    print("wrote %s  %.1f MB  %.0f s" % (out, os.path.getsize(out) / 1e6, sum(durations) / 1000))

for k in (sys.argv[1:] or GUIDES):
    build(k)
