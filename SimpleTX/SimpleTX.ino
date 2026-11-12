/*
 * SimpleTX - ESP32-C3 Zero ESP-NOW transmitter for SimpleFC
 *   2 gimbals (Mode 2: left = throttle/yaw, right = pitch/roll), ARM push button, MODE push button,
 *   SH1106 1.3" OLED, passive buzzer, TX battery on GPIO2.
 *
 * ---------------- WIRING ----------------
 *   Pots (ADC1): THR GPIO1, ROLL GPIO3, PITCH GPIO4, YAW GPIO0  (3.3 V / GND on the pot ends)
 *   OLED SDA GPIO5, SCL GPIO6 (0x3C).   Arm button GPIO7 to GND.
 *   MODE button GPIO20 to GND.
 *   Passive buzzer GPIO21 (via NPN, or ~100 ohm straight to GND).
 *   TX battery: batt+ --R1-- GPIO2 --R2-- GND, 100 nF GPIO2-GND. 1S: 100k/100k (ratio 2.00). Set USE_TX_VBAT 0 if not fitted.
 *
 * ---------------- USE ----------------
 *   Button short press: ARM / DISARM (arming needs the throttle low). Long press (disarmed): MENU.
 *   MODE button short press: SELF-LEVEL <-> ALT HOLD (beep: 1 short = SELF, 1 long = ALT). Starts in SELF-LEVEL.
 *   (CH6 = 1000 SELF / 1500 ALT; the FC engages alt hold on the change.)
 *   MENU: right stick up/down = move, left/right = change, short press = select, long press = save & exit.
 *   Telemetry loss only WARNS ("NO TELEMETRY"); the TX never disarms by itself (the FC has its own failsafe).
 *
 * Channels: THR ROLL PITCH YAW ARM CH6 (1000..2000), 50 packets/s.
 *   SIM MODE (menu > "Sim mode (BT)"): restarts as Bluetooth gamepad "SimpleTX Sim" for FPV simulators (ESP-NOW off).
 *   Axes: X roll, Y pitch, Z throttle, RZ yaw, RX arm, RY mode. Buttons: 1 = ARM on, 2 = MODE ALT.
 *   One boot only: the next power-up (or menu > "Sim mode (BT)" = EXIT) is the normal transmitter again.
 * Libraries: U8g2, ESP32-BLE-Gamepad (+ NimBLE-Arduino). Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <BleGamepad.h>

// ======================= CONFIG =======================
#define FW_VERSION "SimpleTX 1.0"
enum { CH_THR, CH_ROLL, CH_PITCH, CH_YAW, CH_ARM, CH_CH6, NUM_CH };
static const int AXIS_PIN[4] = {1, 3, 4, 0};   // THR, ROLL, PITCH, YAW
#define PIN_SDA   5
#define PIN_SCL   6
#define PIN_BTN   7
#define PIN_MODE  20                         // MODE push button to GND
#define PIN_BUZZ  21
#define BUZZ_HZ   2700
#define USE_TX_VBAT 1
#define PIN_VBAT  2
//#define OLED_IS_SSD1306

#define PKT_MAGIC 0xC3F1
#define BIND_ID   42
#define TLM_MAGIC 0xC3F2                     // SimpleFC telemetry (0xC3F1 = old FlightController layout)
#define WIFI_CH   13
#define SEND_MS   20                          // 50 Hz

// ======================= TYPES (before the first function) =======================
struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  seq;
  uint16_t ch[NUM_CH];
  uint8_t  crc;
};
enum ArmBlock : uint8_t { AB_NONE, AB_THROTTLE, AB_SWITCH, AB_IMU, AB_TILT, AB_NOLINK, AB_CAL, AB_SETUP };
struct __attribute__((packed)) TelemetryPacket {   // identical to SimpleFC
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  armed;
  uint8_t  mode;                   // 0 SELF, 1 ALT HOLD
  uint8_t  armBlock;
  int16_t  roll10, pitch10;
  uint16_t vbat100;
  uint8_t  cells;
  uint8_t  flags;                  // bit0 batt present, bit1 batt LOW, bit2 alt valid, bit3 landed, bit4 failsafe land, bit5 no baro
  int16_t  alt100;
  int8_t   rssi;
  uint8_t  lq;
  uint8_t  crc;
};
struct Settings {
  uint8_t  version;
  uint16_t cmin[4], ccen[4], cmax[4];  // raw ADC per axis
  int8_t   trim[4];                    // us (roll/pitch/yaw)
  uint8_t  revMask;                    // bit per axis
  uint8_t  expo;                       // %
  uint8_t  buzzerOff;
  uint16_t vRatio100;                  // TX battery divider x100
  uint8_t  cells;                      // 0 = auto
  uint16_t warnCell100;
  uint8_t  dbRP, dbYaw, dbThr;          // deadbands, % of stick travel (roll+pitch / yaw at centre, throttle at the bottom)
  uint8_t  filtHz;                     // stick low-pass, Hz, 0 = off
  // end points (added in version 3, appended so a version-2 blob still loads)
  uint16_t thrMid;                     // throttle output at MID travel of the stick (us); bottom = 1000
  uint16_t thrMax;                     // throttle output at full stick (us)
  uint16_t rpyRange;                   // roll/pitch: full stick = 1500 +- this (us) (yaw too before version 6)
  uint8_t  thrExpo;                    // % (version 5): flattens the throttle around mid stick (hover), ends unchanged
  uint16_t yawRange;                   // yaw: full stick = 1500 +- this (us) (version 6)
};
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };
enum UiState { UI_MAIN, UI_MENU, UI_CALIB };
enum MenuItem { MI_CALIB, MI_EXPO, MI_THR_MID, MI_THR_MAX, MI_THR_EXPO, MI_RANGE, MI_YAW_RANGE, MI_TRIM_R, MI_TRIM_P, MI_TRIM_Y, MI_REV_T, MI_REV_R, MI_REV_P, MI_REV_Y,
                MI_DB_RP, MI_DB_Y, MI_DB_T, MI_FILT, MI_BUZZ, MI_VRATIO, MI_CELLS, MI_WARN, MI_SIM, MI_SAVE, MI_EXIT, MI_COUNT };
struct StickKey {                      // stick as a key with auto-repeat
  int last = 0; uint32_t next = 0;
  int poll(float v) {
    int d = v > 0.6f ? 1 : (v < -0.6f ? -1 : 0), ev = 0;
    uint32_t now = millis();
    if (d && d != last) { ev = d; next = now + 400; }
    else if (d && now >= next) { ev = d; next = now + 150; }
    last = d;
    return ev;
  }
};

// ======================= SETTINGS =======================
#define SETTINGS_VERSION 6
static const uint8_t FILTER_HZ[] = {0, 25, 20, 15, 10, 7, 5, 3};   // stick filter choices (same as Transmitter2)
static Settings cfg;
static portMUX_TYPE cfgMux = portMUX_INITIALIZER_UNLOCKED;
static Preferences prefs;

static void defaultSettings(Settings &s) {
  memset(&s, 0, sizeof s);
  s.version = SETTINGS_VERSION;
  // defaults = the current TX's figures (read from its flash on 2026-10-21)
  static const uint16_t MN[4] = {381, 659, 953, 813}, CE[4] = {2238, 2195, 2666, 2360}, MX[4] = {4095, 4046, 4095, 4095};
  for (int i = 0; i < 4; i++) { s.cmin[i] = MN[i]; s.ccen[i] = CE[i]; s.cmax[i] = MX[i]; }
  s.expo = 0;
  s.vRatio100 = 203;
  s.cells = 1;
  s.warnCell100 = 400;
  s.dbRP = 10; s.dbYaw = 15; s.dbThr = 10;
  s.filtHz = 0;
  s.thrMid = 1350; s.thrMax = 1700; s.rpyRange = 300;   // roll/pitch/yaw 1200..1800
  s.thrExpo = 30;
  s.rpyRange = 400;                                // roll/pitch 1100..1900
  s.yawRange = 300;                                // yaw 1200..1800
}

// Stick calibration from the old Transmitter2 sketch (same board, namespace "tx"): first 25 bytes match
static bool importOldTx(Settings &s) {
  uint8_t buf[64];
  if (!prefs.begin("tx", true)) return false;
  size_t n = prefs.getBytesLength("cfg");
  bool ok = n >= 32 && n <= sizeof buf && prefs.getBytes("cfg", buf, n) == n;
  prefs.end();
  if (!ok) return false;
  // old layout: version u8, pad u8, cmin[4] u16 @2, ccen[4] @10, cmax[4] @18, trim[4] @26, rev @30, expo @31
  memcpy(s.cmin, buf + 2, 8); memcpy(s.ccen, buf + 10, 8); memcpy(s.cmax, buf + 18, 8);
  memcpy(s.trim, buf + 26, 4);
  s.revMask = buf[30]; s.expo = buf[31];
  if (n >= 44 && buf[0] >= 3) {      // version 3/4: battery @34.., deadbands @40, filter index @43
    s.buzzerOff = buf[33];
    memcpy(&s.vRatio100, buf + 34, 2); s.cells = buf[36]; memcpy(&s.warnCell100, buf + 38, 2);
    s.dbRP = buf[40]; s.dbYaw = buf[41]; s.dbThr = buf[42];
    s.filtHz = buf[43] < sizeof FILTER_HZ ? FILTER_HZ[buf[43]] : 0;
  }
  for (int i = 0; i < 4; i++) if (s.cmax[i] <= s.cmin[i] + 800 || s.cmax[i] > 4095) return false;
  return true;
}

static void sanitize(Settings &s) {
  if (s.expo > 100) s.expo = 20;
  if (s.buzzerOff > 1) s.buzzerOff = 0;
  if (s.vRatio100 < 100 || s.vRatio100 > 2000) s.vRatio100 = 200;
  if (s.cells > 6) s.cells = 0;
  if (s.warnCell100 < 300 || s.warnCell100 > 400) s.warnCell100 = 340;
  for (int i = 1; i < 4; i++) s.trim[i] = constrain(s.trim[i], -100, 100);
  if (s.dbRP > 25 || s.dbYaw > 25 || s.dbThr > 25) { s.dbRP = 10; s.dbYaw = 15; s.dbThr = 10; }
  if (s.filtHz > 25) s.filtHz = 0;
  if (s.thrMid < 1100 || s.thrMid > 1600) s.thrMid = 1350;
  if (s.thrMax < s.thrMid + 100 || s.thrMax > 2000) s.thrMax = max(1700, s.thrMid + 100);
  if (s.rpyRange < 200 || s.rpyRange > 500) s.rpyRange = 400;
  if (s.yawRange < 200 || s.yawRange > 500) s.yawRange = 300;
  if (s.thrExpo > 100) s.thrExpo = 30;
  s.ccen[0] = (s.cmin[0] + s.cmax[0]) / 2;      // throttle "centre" = middle of its travel
}

static void saveSettings() {
  Settings s;
  portENTER_CRITICAL(&cfgMux); s = cfg; portEXIT_CRITICAL(&cfgMux);
  if (prefs.begin("stx", false)) { prefs.putBytes("cfg", &s, sizeof s); prefs.end(); }
}

static void loadSettings() {
  Settings s;
  defaultSettings(s);
  bool have = false, migrated = false;
  if (prefs.begin("stx", true)) {
    // Newer versions only APPEND fields, so an older blob is read as far as it goes and the rest keeps its default.
    size_t len = prefs.getBytesLength("cfg");
    if (len >= offsetof(Settings, thrMid) && len <= sizeof s) {
      prefs.getBytes("cfg", &s, len);
      if (s.version >= 2 && s.version <= SETTINGS_VERSION) {
        have = true;
        if (s.version <= 3) s.rpyRange = 300;            // v2/v3 -> roll/pitch/yaw 1200..1800
        if (s.version <= 4) s.thrExpo = 30;              // v5 added throttle expo
        if (s.version <= 5) { s.rpyRange = 400; s.yawRange = 300; }   // v6: roll/pitch 1100..1900, yaw 1200..1800
        migrated = s.version != SETTINGS_VERSION;
        s.version = SETTINGS_VERSION;
      }
    }
    prefs.end();
  }
  if (!have) {
    defaultSettings(s);
    if (importOldTx(s)) Serial.println("Stick calibration imported from Transmitter2.");
  }
  sanitize(s);
  portENTER_CRITICAL(&cfgMux); cfg = s; portEXIT_CRITICAL(&cfgMux);
  if (!have || migrated) saveSettings();
}

// ======================= SHARED STATE (radio task <-> UI) =======================
static portMUX_TYPE stMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t rawAxis[4];
static volatile float stickPos[4];             // thr -1 (bottom) .. 0 (centre) .. +1, others -1..1 (calibrated, reversed)
#define THR_IS_LOW(p) ((p) < -0.95f)              // throttle stick at the bottom
static volatile uint16_t chOut[NUM_CH] = {1000, 1500, 1500, 1500, 1000, 1000};
static volatile bool armRequest = false, forceSafe = true;
static volatile bool modeAlt = false;          // MODE button: false = SELF-LEVEL, true = ALT HOLD
static volatile float txVbat = 0;

static TelemetryPacket telem;
static volatile uint32_t lastTelemMs = 0;
static uint16_t telemCount = 0;
static volatile int8_t telemRssi = 0;
static uint8_t txLq = 0;                       // % of the FC's 20 telemetry packets/s received
static float txRssiF = 0;

static uint8_t fcMac[6];
static volatile bool fcMacKnown = false;
static bool fcPeerAdded = false;
static bool simMode = false;                   // Bluetooth gamepad for simulators instead of ESP-NOW
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) { c ^= *d++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
  return c;
}

// ======================= RADIO =======================
// Telemetry from SimpleFC (magic 0xC3F2, layout TelemetryPacket) or from the old FlightController (magic 0xC3F1, same
// 19-byte size but another layout: ... roll10, pitch10, loopUs, vbat100, cells, battFlags, rssi, lq, crc). The old one is
// converted, so the FC battery, mode and arm-block show with either FC.
struct __attribute__((packed)) OldTelemetry {
  uint16_t magic; uint8_t bindId, armed, mode, armBlock;
  int16_t roll10, pitch10; uint16_t loopUs, vbat100; uint8_t cells, battFlags; int8_t rssi; uint8_t lq, crc;
};
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(TelemetryPacket) || crc8(data, len - 1) != data[len - 1]) return;
  TelemetryPacket t;
  memcpy(&t, data, sizeof t);
  if (t.bindId != BIND_ID) return;
  if (t.magic == PKT_MAGIC) {                           // old FlightController
    OldTelemetry o;
    memcpy(&o, data, sizeof o);
    t.mode = (o.mode == 1) ? 2 : (o.mode >= 2 ? 1 : 0); // old 0 self / 1 acro / 2 alt / 3 pos -> 0 SELF, 1 ALT, 2 ACRO
    t.armBlock = (o.armBlock == 6) ? 7 : o.armBlock;    // old AB_SETUP = 6, here 7
    t.vbat100 = o.vbat100;
    t.cells = o.cells;
    t.flags = o.battFlags & 3;                          // bit0 battery present, bit1 LOW (same meaning)
    t.alt100 = 0;
    t.rssi = o.rssi;
    t.lq = o.lq;
  } else if (t.magic != TLM_MAGIC) return;
  portENTER_CRITICAL(&stMux);
  telem = t;
  lastTelemMs = millis();
  telemCount++;
  telemRssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  if (!fcMacKnown) { memcpy(fcMac, info->src_addr, 6); fcMacKnown = true; }
  portEXIT_CRITICAL(&stMux);
}

static void radioBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_country_t ctry = {"EU", 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL};
  esp_wifi_set_country(&ctry);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CH; peer.ifidx = WIFI_IF_STA; peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_wifi_set_max_tx_power(80);
  Serial.printf("TX MAC %s, channel %d\n", WiFi.macAddress().c_str(), WIFI_CH);
}

static float centred(float v, float mn, float ce, float mx, float db) {   // -1..1, deadband db (0..1) at the centre
  float x = (v >= ce) ? (mx > ce ? (v - ce) / (mx - ce) : 0) : (ce > mn ? -(ce - v) / (ce - mn) : 0);
  x = constrain(x, -1.0f, 1.0f);
  return fabsf(x) <= db ? 0 : (x - copysignf(db, x)) / (1 - db);
}

// Sticks -> packet, 50 Hz, its own task so the OLED never delays it.
// Broadcast until the FC's telemetry arrives, then unicast to its MAC (acknowledged + retried).
static void radioTask(void *) {
  ControlPacket pkt;
  pkt.magic = PKT_MAGIC; pkt.bindId = BIND_ID; pkt.seq = 0;
  TickType_t wake = xTaskGetTickCount();
  float filt[4] = {-1, -1, -1, -1};
  for (;;) {
    Settings c;
    portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);
    const float alpha = c.filtHz ? 1 - expf(-2 * PI * c.filtHz * SEND_MS * 1e-3f) : 1.0f;   // 1 = no filter
    uint16_t raw[4]; float pos[4];
    for (int i = 0; i < 4; i++) {
      uint32_t s = 0;
      for (int k = 0; k < 4; k++) s += analogRead(AXIS_PIN[i]);
      float r = s / 4.0f;
      filt[i] = filt[i] < 0 ? r : filt[i] + alpha * (r - filt[i]);
      raw[i] = (uint16_t)(filt[i] + 0.5f);
    }
    // Throttle: bottom half of the travel 1000..THR MID, top half THR MID..THR MAX (two linear halves around the
    // middle of the calibrated travel). The joystick's usable range is poor, so this spreads hover over more stick. pos[CH_THR] = -1 (bottom) .. 0 (centre) .. +1 (top).
    {
      float lo = c.ccen[0] > c.cmin[0] ? (raw[0] - (float)c.cmin[0]) / (c.ccen[0] - c.cmin[0]) : 0;   // 0..1 below centre
      float hi = c.cmax[0] > c.ccen[0] ? (raw[0] - (float)c.ccen[0]) / (c.cmax[0] - c.ccen[0]) : 0;   // 0..1 above centre
      float t = raw[0] < c.ccen[0] ? constrain(lo, 0.0f, 1.0f) - 1.0f : constrain(hi, 0.0f, 1.0f);
      if (c.revMask & 1) t = -t;
      { float te = c.thrExpo / 100.0f; t = t * (1 - te) + t * t * t * te; }   // THR EXPO around mid stick
      float dbT = c.dbThr / 100.0f;                                    // deadband at the very bottom: rests at 1000
      if (t < 0) t = (t + 1 <= dbT) ? -1.0f : -1.0f + (t + 1 - dbT) / (1 - dbT);
      pos[CH_THR] = t;
    }
    for (int i = 1; i < 4; i++) {
      pos[i] = centred(raw[i], c.cmin[i], c.ccen[i], c.cmax[i], (i == CH_YAW ? c.dbYaw : c.dbRP) / 100.0f);
      if (c.revMask & (1 << i)) pos[i] = -pos[i];
    }
    uint16_t ch[NUM_CH];
    float e = c.expo / 100.0f;
    ch[CH_THR] = (uint16_t)lroundf(pos[CH_THR] < 0 ? c.thrMid + pos[CH_THR] * (c.thrMid - 1000) : c.thrMid + pos[CH_THR] * (c.thrMax - c.thrMid));
    for (int i = 1; i < 4; i++) {
      float x = pos[i];
      x = x * (1 - e) + x * x * x * e;
      ch[i] = constrain((int)lroundf(1500 + x * (i == CH_YAW ? c.yawRange : c.rpyRange) + c.trim[i]), 1000, 2000);   // end points
    }
    bool safe = forceSafe;
    if (safe) { ch[CH_THR] = 1000; ch[CH_ROLL] = ch[CH_PITCH] = ch[CH_YAW] = 1500; }
    ch[CH_ARM] = (armRequest && !safe) ? 2000 : 1000;
    ch[CH_CH6] = modeAlt ? 1500 : 1000;

    pkt.seq++;
    memcpy(pkt.ch, ch, sizeof ch);
    pkt.crc = crc8((const uint8_t *)&pkt, sizeof pkt - 1);
    if (fcMacKnown && !fcPeerAdded) {
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, fcMac, 6);
      p.channel = WIFI_CH; p.ifidx = WIFI_IF_STA; p.encrypt = false;
      fcPeerAdded = esp_now_add_peer(&p) == ESP_OK || esp_now_is_peer_exist(fcMac);
    }
    if (!simMode) esp_now_send(fcPeerAdded ? fcMac : BCAST, (const uint8_t *)&pkt, sizeof pkt);

#if USE_TX_VBAT
    float v = analogReadMilliVolts(PIN_VBAT) * 0.001f * c.vRatio100 / 100.0f;
    float nv = txVbat == 0 ? v : txVbat + 0.02f * (v - txVbat);
#else
    float nv = 0;
#endif
    portENTER_CRITICAL(&stMux);
    for (int i = 0; i < 4; i++) { rawAxis[i] = raw[i]; stickPos[i] = pos[i]; }
    for (int i = 0; i < NUM_CH; i++) chOut[i] = ch[i];
    txVbat = nv;
    portEXIT_CRITICAL(&stMux);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SEND_MS));
  }
}

// ======================= BUZZER (passive) =======================
static uint32_t evBits = 0, evStart = 0; static uint8_t evLen = 0;
static void beep(uint32_t bits, uint8_t len) { evBits = bits; evLen = len; evStart = millis(); }
static bool patternOn(uint32_t bits, uint8_t len, uint32_t t) { return (bits >> ((t / 100) % len)) & 1; }
static void buzzerSet(bool on) {
  static bool st = false;
  if (on == st) return;
  st = on;
  ledcWrite(PIN_BUZZ, on ? 128 : 0);
}

// ======================= TX BATTERY =======================
static bool txBattPresent = false, txBattLow = false;
static uint8_t txCells = 0;
static void txBatteryTick(uint32_t now) {
#if USE_TX_VBAT
  static uint32_t lowSince = 0;
  float v; Settings c;
  portENTER_CRITICAL(&stMux); v = txVbat; portEXIT_CRITICAL(&stMux);
  portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);
  txBattPresent = v > 2.5f;
  if (!txBattPresent) { txBattLow = false; lowSince = 0; return; }
  if (c.cells) txCells = c.cells;
  else if (!txCells && now > 1500) txCells = constrain((int)(v / 4.35f) + 1, 1, 6);
  if (!txCells) return;
  float cell = v / txCells, warn = c.warnCell100 / 100.0f;
  if (cell < warn) { if (!lowSince) lowSince = now ? now : 1; if (now - lowSince > 3000) txBattLow = true; }
  else if (cell > warn + 0.05f) { lowSince = 0; txBattLow = false; }
#endif
}

// ======================= BUTTON =======================
static BtnEvent pollButton() {
  static bool stable = HIGH, lastRead = HIGH, longFired = false;
  static uint32_t changeMs = 0, pressMs = 0;
  uint32_t now = millis();
  bool r = digitalRead(PIN_BTN);
  if (r != lastRead) { lastRead = r; changeMs = now; }
  if (now - changeMs > 25 && r != stable) {
    stable = r;
    if (stable == LOW) { pressMs = now; longFired = false; }
    else if (!longFired) return BTN_SHORT;
  }
  if (stable == LOW && !longFired && now - pressMs > 800) { longFired = true; return BTN_LONG; }
  return BTN_NONE;
}

static bool pollModeButton() {         // true once per press (debounced)
  static bool stable = HIGH, lastRead = HIGH;
  static uint32_t changeMs = 0;
  bool r = digitalRead(PIN_MODE);
  uint32_t now = millis();
  if (r != lastRead) { lastRead = r; changeMs = now; }
  if (now - changeMs > 25 && r != stable) { stable = r; return stable == LOW; }
  return false;
}

// ======================= DISPLAY / UI =======================
#if defined(OLED_IS_SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SH1106_128X64_NONAME_F_HW_I2C  oled(U8G2_R0, U8X8_PIN_NONE);
#endif

static UiState ui = UI_MAIN;
static char msgText[22] = ""; static uint32_t msgUntil = 0;
static void showMsg(const char *m) { strncpy(msgText, m, sizeof msgText - 1); msgText[sizeof msgText - 1] = 0; msgUntil = millis() + 1500; }

static const char *MENU_LABEL[MI_COUNT] = {
  "Calibrate sticks", "Expo", "Thr centre", "Thr max", "Thr expo", "R/P range", "Yaw range", "Trim roll", "Trim pitch", "Trim yaw",
  "Rev throttle", "Rev roll", "Rev pitch", "Rev yaw",
  "Deadband R/P", "Deadband yaw", "Deadband thr", "Stick filter", "Buzzer",
  "TX batt ratio", "TX cells", "TX warn V/cell", "Sim mode (BT)", "Save & exit", "Exit (no save)"
};
static int menuSel = 0, menuTop = 0;
static StickKey keyV, keyH;
static int calStep = 0;
static uint16_t calMin[4], calMax[4];

static void menuValue(int item, char *b, size_t n) {
  Settings c;
  portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);
  b[0] = 0;
  switch (item) {
    case MI_EXPO:   snprintf(b, n, "%u%%", c.expo); break;
    case MI_THR_MID: snprintf(b, n, "%u", c.thrMid); break;
    case MI_THR_MAX: snprintf(b, n, "%u", c.thrMax); break;
    case MI_THR_EXPO: snprintf(b, n, "%u%%", c.thrExpo); break;
    case MI_RANGE:  snprintf(b, n, "+-%u", c.rpyRange); break;
    case MI_YAW_RANGE: snprintf(b, n, "+-%u", c.yawRange); break;
    case MI_TRIM_R: snprintf(b, n, "%+d", c.trim[CH_ROLL]); break;
    case MI_TRIM_P: snprintf(b, n, "%+d", c.trim[CH_PITCH]); break;
    case MI_TRIM_Y: snprintf(b, n, "%+d", c.trim[CH_YAW]); break;
    case MI_REV_T: case MI_REV_R: case MI_REV_P: case MI_REV_Y:
      snprintf(b, n, "%s", (c.revMask & (1 << (item - MI_REV_T))) ? "REV" : "NOR"); break;
    case MI_DB_RP:  snprintf(b, n, "%u%%", c.dbRP); break;
    case MI_DB_Y:   snprintf(b, n, "%u%%", c.dbYaw); break;
    case MI_DB_T:   snprintf(b, n, "%u%%", c.dbThr); break;
    case MI_FILT:   if (c.filtHz) snprintf(b, n, "%uHz", c.filtHz); else snprintf(b, n, "OFF"); break;
    case MI_BUZZ:   snprintf(b, n, "%s", c.buzzerOff ? "OFF" : "ON"); break;
    case MI_SIM:    snprintf(b, n, "%s", simMode ? "EXIT" : "START"); break;
    case MI_VRATIO: snprintf(b, n, "%.2f", c.vRatio100 / 100.0f); break;
    case MI_CELLS:  if (c.cells) snprintf(b, n, "%uS", c.cells); else snprintf(b, n, "AUTO"); break;
    case MI_WARN:   snprintf(b, n, "%.2f", c.warnCell100 / 100.0f); break;
    default:        snprintf(b, n, ">"); break;
  }
}

static void menuAdjust(int item, int dir) {
  bool buzzOn;
  portENTER_CRITICAL(&cfgMux);
  switch (item) {
    case MI_EXPO:   cfg.expo = constrain((int)cfg.expo + dir * 5, 0, 100); break;
    case MI_THR_MID: cfg.thrMid = constrain((int)cfg.thrMid + dir * 10, 1100, 1600); if (cfg.thrMax < cfg.thrMid + 100) cfg.thrMax = cfg.thrMid + 100; break;
    case MI_THR_MAX: cfg.thrMax = constrain((int)cfg.thrMax + dir * 10, cfg.thrMid + 100, 2000); break;
    case MI_THR_EXPO: cfg.thrExpo = constrain((int)cfg.thrExpo + dir * 5, 0, 100); break;
    case MI_RANGE:  cfg.rpyRange = constrain((int)cfg.rpyRange + dir * 10, 200, 500); break;
    case MI_YAW_RANGE: cfg.yawRange = constrain((int)cfg.yawRange + dir * 10, 200, 500); break;
    case MI_TRIM_R: cfg.trim[CH_ROLL]  = constrain(cfg.trim[CH_ROLL]  + dir * 2, -100, 100); break;
    case MI_TRIM_P: cfg.trim[CH_PITCH] = constrain(cfg.trim[CH_PITCH] + dir * 2, -100, 100); break;
    case MI_TRIM_Y: cfg.trim[CH_YAW]   = constrain(cfg.trim[CH_YAW]   + dir * 2, -100, 100); break;
    case MI_REV_T: case MI_REV_R: case MI_REV_P: case MI_REV_Y: cfg.revMask ^= 1 << (item - MI_REV_T); break;
    case MI_DB_RP:  cfg.dbRP  = constrain((int)cfg.dbRP  + dir, 0, 25); break;
    case MI_DB_Y:   cfg.dbYaw = constrain((int)cfg.dbYaw + dir, 0, 25); break;
    case MI_DB_T:   cfg.dbThr = constrain((int)cfg.dbThr + dir, 0, 25); break;
    case MI_FILT: {                                  // right = smoother: OFF, 25, 20, 15, 10, 7, 5, 3 Hz
      int k = 0;
      for (int j = 0; j < (int)sizeof FILTER_HZ; j++) if (FILTER_HZ[j] == cfg.filtHz) k = j;
      cfg.filtHz = FILTER_HZ[constrain(k + dir, 0, (int)sizeof FILTER_HZ - 1)];
      break;
    }
    case MI_BUZZ:   cfg.buzzerOff = !cfg.buzzerOff; break;
    case MI_VRATIO: cfg.vRatio100 = constrain((int)cfg.vRatio100 + dir, 100, 2000); break;
    case MI_CELLS:  cfg.cells = (cfg.cells + dir + 7) % 7; break;
    case MI_WARN:   cfg.warnCell100 = constrain((int)cfg.warnCell100 + dir * 5, 300, 400); break;
  }
  buzzOn = !cfg.buzzerOff;
  portEXIT_CRITICAL(&cfgMux);
  if (item == MI_BUZZ && buzzOn) beep(0x3, 3);
}

static void enterMenu() { forceSafe = true; ui = UI_MENU; menuSel = menuTop = 0; keyV = StickKey(); keyH = StickKey(); }
static void exitMenu()  { ui = UI_MAIN; armRequest = false; forceSafe = false; }

static void drawBar(int x, int y, int w, int h, uint16_t us, bool centredBar) {
  oled.drawFrame(x, y, w, h);
  int inner = w - 2, p = (int)(us - 1000) * inner / 1000;
  if (centredBar) {
    int mid = x + 1 + inner / 2, q = x + 1 + p;
    if (q >= mid) oled.drawBox(mid, y + 1, q - mid + 1, h - 2); else oled.drawBox(q, y + 1, mid - q, h - 2);
    oled.drawVLine(mid, y - 1, h + 2);
  } else if (p > 0) oled.drawBox(x + 1, y + 1, p, h - 2);
}

static const char *armBlockText(uint8_t b) {
  switch (b) {
    case AB_THROTTLE: return "THR HIGH"; case AB_SWITCH: return "RE-ARM"; case AB_IMU: return "IMU ERR";
    case AB_TILT: return "TILTED"; case AB_NOLINK: return "NO LINK"; case AB_CAL: return "CAL"; case AB_SETUP: return "IN SETUP"; default: return "";
  }
}

static void inverted(int x, int y, int w, bool on) { if (on) { oled.setDrawColor(1); oled.drawBox(x, y, w, 9); oled.setDrawColor(0); } }

// ======================= SIM MODE (BLE gamepad) =======================
// One boot only: choosing SIM MODE stores a flag and restarts; that boot clears the flag again, so the next power-up is
// always the normal transmitter. In SIM MODE ESP-NOW is never started (the radio is used by Bluetooth).
static BleGamepad *gamepad = nullptr;
static void restartSim(bool on) {
  if (prefs.begin("stx", false)) { prefs.putUChar("sim", on ? 1 : 0); prefs.end(); }
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(10, 34, on ? "Starting SIM MODE" : "Leaving SIM MODE");
  oled.sendBuffer();
  delay(300);
  ESP.restart();
}
static bool takeSimFlag() {
  bool on = false;
  if (prefs.begin("stx", false)) { on = prefs.getUChar("sim", 0); if (on) prefs.putUChar("sim", 0); prefs.end(); }
  return on;
}
static void simBegin() {
  gamepad = new BleGamepad("SimpleTX Sim", "Espressif", 100);
  BleGamepadConfiguration gc;
  gc.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  gc.setAutoReport(false);
  gc.setButtonCount(2);
  gc.setHatSwitchCount(0);
  gc.setWhichSpecialButtons(false, false, false, false, false, false, false, false);
  gc.setWhichAxes(true, true, true, true, true, true, false, false);
  gc.setWhichSimulationControls(false, false, false, false, false);
  gc.setAxesMin(0);
  gc.setAxesMax(32767);
  gamepad->begin(&gc);
}
static int16_t usToAxis(uint16_t us) { return (int16_t)((constrain((int)us, 1000, 2000) - 1000) * 32767L / 1000); }
static void simTick(uint32_t now) {            // 50 Hz, only when something changed (or every 0.5 s)
  static uint32_t ms = 0, lastSend = 0;
  static uint16_t last[NUM_CH];
  if (!gamepad || now - ms < SEND_MS) return;
  ms = now;
  if (!gamepad->isConnected()) return;
  uint16_t c[NUM_CH];
  portENTER_CRITICAL(&stMux); for (int i = 0; i < NUM_CH; i++) c[i] = chOut[i]; portEXIT_CRITICAL(&stMux);
  if (!memcmp(c, last, sizeof c) && now - lastSend < 500) return;
  memcpy(last, c, sizeof c);
  lastSend = now;
  gamepad->setAxes(usToAxis(c[CH_ROLL]), usToAxis(c[CH_PITCH]), usToAxis(c[CH_THR]),
                   usToAxis(c[CH_ARM]), usToAxis(c[CH_CH6]), usToAxis(c[CH_YAW]));
  if (c[CH_ARM] > 1500) gamepad->press(BUTTON_1); else gamepad->release(BUTTON_1);
  if (c[CH_CH6] > 1250) gamepad->press(BUTTON_2); else gamepad->release(BUTTON_2);
  gamepad->sendReport();
}
static void drawSimHeader(uint32_t now, float tv) {
  bool con = gamepad && gamepad->isConnected(), blink = (now / 300) & 1;
  char b[24];
  inverted(0, 0, 44, true); oled.drawStr(2, 8, "SIM MODE"); oled.setDrawColor(1);
  oled.drawStr(50, 8, con ? "BT CONNECTED" : (blink ? "BT PAIRING.." : ""));
  oled.drawStr(2, 17, armRequest ? "ARM ON" : "arm off");
  oled.drawStr(44, 17, modeAlt ? "ALT" : "SELF");
#if USE_TX_VBAT
  if (txBattPresent) snprintf(b, sizeof b, "TX%5.2fV", tv); else snprintf(b, sizeof b, "TX USB");
  oled.drawStr(86, 17, b);
#endif
}

static void drawMain(uint32_t now) {
  uint16_t ch[NUM_CH]; TelemetryPacket t; uint32_t lt; float tv;
  portENTER_CRITICAL(&stMux);
  for (int i = 0; i < NUM_CH; i++) ch[i] = chOut[i];
  t = telem; lt = lastTelemMs; tv = txVbat;
  portEXIT_CRITICAL(&stMux);
  bool link = lt && now - lt < 500, blink = (now / 300) & 1;
  char b[24];
  oled.setFont(u8g2_font_5x8_tf);

  if (simMode) drawSimHeader(now, tv);
  else {
  // row 1: armed/safe | mode | RSSI + LQ of the weaker direction
  if (link && t.armed) { inverted(0, 0, 30, true); oled.drawStr(2, 8, "ARMED"); oled.setDrawColor(1); }
  else if (armRequest) { if (blink) oled.drawStr(2, 8, "ARM?"); }
  else oled.drawStr(2, 8, "SAFE");
  const char *md = link ? (t.mode == 1 ? ((t.flags & 8) ? "ALT-GND" : "ALT") : (t.mode == 2 ? "ACRO" : "SELF")) : (modeAlt ? "(ALT)" : "(SELF)");
  if (link && (t.flags & 16)) md = blink ? "AUTOLAND" : "";
  oled.drawStr(34, 8, md);
  if (link) {
    int rFc = t.rssi, rTx = (int)lroundf(txRssiF);
    int r = (rFc && rTx) ? min(rFc, rTx) : (rFc ? rFc : rTx);
    uint8_t lq = min<uint8_t>(t.lq, txLq);
    bool weak = (r && r < -85) || lq < 60;
    inverted(76, 0, 52, weak && blink);
    snprintf(b, sizeof b, "%4d %3u%%", r, lq);
    oled.drawStr(78, 8, b);
    oled.setDrawColor(1);
  } else oled.drawStr(82, 8, "NO LINK");

  // row 2: FC battery | altitude | TX battery
  bool fcB = link && (t.flags & 1);
  inverted(0, 9, 42, fcB && (t.flags & 2) && blink);
  if (fcB) snprintf(b, sizeof b, "FC%5.2fV", t.vbat100 / 100.0f); else snprintf(b, sizeof b, "FC --");
  oled.drawStr(2, 17, b);
  oled.setDrawColor(1);
  if (link && (t.flags & 4)) { snprintf(b, sizeof b, "%+.1fm", t.alt100 / 100.0f); oled.drawStr(46, 17, b); }
  else if (link && (t.flags & 32)) oled.drawStr(46, 17, "NOBARO");
#if USE_TX_VBAT
  inverted(84, 9, 44, txBattPresent && txBattLow && blink);
  if (txBattPresent) snprintf(b, sizeof b, "TX%5.2fV", tv); else snprintf(b, sizeof b, "TX USB");
  oled.drawStr(86, 17, b);
  oled.setDrawColor(1);
#endif
  }
  oled.drawHLine(0, 19, 128);

  static const char *NAMES[NUM_CH] = {"THR", "ROL", "PIT", "YAW", "ARM", "MOD"};
  for (int i = 0; i < NUM_CH; i++) {
    int y = 21 + i * 7;
    oled.drawStr(0, y + 6, NAMES[i]);
    snprintf(b, sizeof b, "%4u", ch[i]);
    oled.drawStr(18, y + 6, b);
    drawBar(42, y, 86, 6, ch[i], i >= CH_ROLL && i <= CH_YAW);
  }
  if (armRequest && link && !t.armed && t.armBlock != AB_NONE) {
    snprintf(b, sizeof b, "FC: %s", armBlockText(t.armBlock));
    oled.setDrawColor(0); oled.drawBox(18, 28, 92, 14); oled.setDrawColor(1);
    oled.drawFrame(18, 28, 92, 14);
    oled.drawStr(24, 38, b);
  }
}

static void drawMenu() {
  char b[24];
  oled.setFont(u8g2_font_6x10_tf);
#if USE_TX_VBAT
  float tv; portENTER_CRITICAL(&stMux); tv = txVbat; portEXIT_CRITICAL(&stMux);
  snprintf(b, sizeof b, "MENU   TX %5.2fV", tv);
#else
  snprintf(b, sizeof b, "MENU");
#endif
  oled.drawStr(0, 9, b);
  oled.drawHLine(0, 11, 128);
  const int ROWS = 5;
  if (menuSel < menuTop) menuTop = menuSel;
  if (menuSel >= menuTop + ROWS) menuTop = menuSel - ROWS + 1;
  for (int r = 0; r < ROWS && menuTop + r < MI_COUNT; r++) {
    int item = menuTop + r, y = 14 + r * 10;
    bool sel = item == menuSel;
    if (sel) { oled.drawBox(0, y, 124, 10); oled.setDrawColor(0); }
    oled.drawStr(2, y + 8, MENU_LABEL[item]);
    menuValue(item, b, sizeof b);
    oled.drawStr(122 - oled.getStrWidth(b), y + 8, b);
    oled.setDrawColor(1);
  }
  int barH = 50 * ROWS / MI_COUNT, barY = 13 + (50 - barH) * menuTop / (MI_COUNT - ROWS);
  oled.drawBox(126, barY, 2, barH);
}

static void drawCalib() {
  uint16_t raw[4];
  portENTER_CRITICAL(&stMux); for (int i = 0; i < 4; i++) raw[i] = rawAxis[i]; portEXIT_CRITICAL(&stMux);
  char b[24];
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 9, "STICK CALIBRATION");
  oled.drawHLine(0, 11, 128);
  if (calStep == 0) { oled.drawStr(0, 22, "Move BOTH sticks to"); oled.drawStr(0, 32, "all ends, then press"); }
  else              { oled.drawStr(0, 22, "Centre roll/pitch/yaw"); oled.drawStr(0, 32, "then press button"); }
  static const char *N[4] = {"T", "R", "P", "Y"};
  for (int i = 0; i < 4; i++) {
    if (calStep == 0) snprintf(b, sizeof b, "%s%4u %3u%%", N[i], raw[i], calMax[i] > calMin[i] ? (calMax[i] - calMin[i]) * 100 / 4095 : 0);
    else snprintf(b, sizeof b, "%s %4u", N[i], raw[i]);
    oled.drawStr((i % 2) * 64, 46 + (i / 2) * 11, b);
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);
  ledcAttach(PIN_BUZZ, BUZZ_HZ, 8);
  ledcWrite(PIN_BUZZ, 0);
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) analogSetPinAttenuation(AXIS_PIN[i], ADC_11db);
#if USE_TX_VBAT
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
#endif
  Wire.begin(PIN_SDA, PIN_SCL);
  oled.setBusClock(400000);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(10, 24, FW_VERSION);
  simMode = takeSimFlag();
  oled.drawStr(10, 40, simMode ? "SIM MODE (BT)" : "ESP-NOW ch 13");
  oled.sendBuffer();

  loadSettings();
  if (simMode) simBegin(); else radioBegin();
  xTaskCreate(radioTask, "radio", 4096, nullptr, 5, nullptr);
  delay(500);

  for (;;) {                                   // refuse to start with the throttle up
    float thr;
    portENTER_CRITICAL(&stMux); thr = stickPos[CH_THR]; portEXIT_CRITICAL(&stMux);
    if (THR_IS_LOW(thr)) break;
    if (pollButton() == BTN_LONG) { enterMenu(); return; }   // escape hatch: recalibrate
    oled.clearBuffer();
    oled.drawStr(0, 14, "THROTTLE NOT LOW!");
    oled.drawStr(0, 34, "Lower it, or hold");
    oled.drawStr(0, 46, "button for the menu");
    oled.sendBuffer();
    delay(50);
  }
  forceSafe = false;
  beep(0x1, 2);
}

void loop() {
  uint32_t now = millis();
  if (simMode) simTick(now);
  BtnEvent btn = pollButton();
  txBatteryTick(now);

  static uint32_t lqMs = 0, rsMs = 0;
  if (now - lqMs >= 1000) {                    // telemetry is 20 Hz
    lqMs = now;
    uint16_t n;
    portENTER_CRITICAL(&stMux); n = telemCount; telemCount = 0; portEXIT_CRITICAL(&stMux);
    txLq = min<uint16_t>(100, n * 5);
  }
  if (now - rsMs >= 20) {
    rsMs = now;
    int8_t r = telemRssi;
    if (r) txRssiF = txRssiF == 0 ? r : txRssiF + 0.05f * (r - txRssiF);
  }

  TelemetryPacket t; uint32_t lt;
  portENTER_CRITICAL(&stMux); t = telem; lt = lastTelemMs; portEXIT_CRITICAL(&stMux);
  bool link = lt && now - lt < 1000;

  // telemetry loss: warn only, never disarm
  static uint32_t lastWarn = 0;
  if (armRequest && lt && now - lt > 1000 && now - lastWarn > 3000) { lastWarn = now; showMsg("NO TELEMETRY"); }

  // FC disarmed itself (crash cut-off, landed after failsafe...): drop the arm request so the next press re-arms
  static bool fcWasArmed = false;
  if (link && now - lt < 200) {
    if (fcWasArmed && !t.armed && armRequest) { armRequest = false; showMsg("FC DISARMED"); beep(0x15, 6); }
    fcWasArmed = t.armed;
  }

  // MODE button: short press toggles SELF-LEVEL <-> ALT HOLD (main screen only; in the menu it does nothing)
  if (pollModeButton() && ui == UI_MAIN) {
    modeAlt = !modeAlt;
    showMsg(modeAlt ? "MODE: ALT HOLD" : "MODE: SELF-LEVEL");
    beep(modeAlt ? 0x7 : 0x1, 4);
  }

  float pitchPos, rollPos, thrPos;
  portENTER_CRITICAL(&stMux); pitchPos = stickPos[CH_PITCH]; rollPos = stickPos[CH_ROLL]; thrPos = stickPos[CH_THR]; portEXIT_CRITICAL(&stMux);

  switch (ui) {
    case UI_MAIN:
      if (btn == BTN_SHORT) {
        if (armRequest)          { armRequest = false; showMsg("DISARMED"); beep(0x5, 4); }
        else if (!THR_IS_LOW(thrPos)) { showMsg("LOWER THROTTLE!"); beep(0x15, 6); }
        else                     { armRequest = true; beep(0x3, 3); }
      } else if (btn == BTN_LONG) {
        if (armRequest) showMsg("DISARM FIRST"); else enterMenu();
      }
      break;
    case UI_MENU: {
      int v = keyV.poll(pitchPos), h = keyH.poll(rollPos);
      if (v > 0) menuSel = (menuSel + MI_COUNT - 1) % MI_COUNT;
      if (v < 0) menuSel = (menuSel + 1) % MI_COUNT;
      if (h) menuAdjust(menuSel, h);
      if (btn == BTN_SHORT) {
        if (menuSel == MI_CALIB) { ui = UI_CALIB; calStep = 0; for (int i = 0; i < 4; i++) { calMin[i] = 4095; calMax[i] = 0; } }
        else if (menuSel == MI_SIM) { saveSettings(); restartSim(!simMode); }
        else if (menuSel == MI_SAVE) { saveSettings(); showMsg("SAVED"); exitMenu(); }
        else if (menuSel == MI_EXIT) { loadSettings(); exitMenu(); }
        else menuAdjust(menuSel, 1);
      } else if (btn == BTN_LONG) { saveSettings(); showMsg("SAVED"); exitMenu(); }
      break;
    }
    case UI_CALIB: {
      uint16_t raw[4];
      portENTER_CRITICAL(&stMux); for (int i = 0; i < 4; i++) raw[i] = rawAxis[i]; portEXIT_CRITICAL(&stMux);
      if (calStep == 0) {
        for (int i = 0; i < 4; i++) { calMin[i] = min(calMin[i], raw[i]); calMax[i] = max(calMax[i], raw[i]); }
        if (btn == BTN_SHORT) calStep = 1;
      } else if (btn == BTN_SHORT) {
        bool ok = true;
        for (int i = 0; i < 4; i++) if (calMax[i] - calMin[i] < 800) ok = false;
        if (ok) {
          portENTER_CRITICAL(&cfgMux);
          for (int i = 0; i < 4; i++) {
            cfg.cmin[i] = calMin[i]; cfg.cmax[i] = calMax[i];
            cfg.ccen[i] = i == CH_THR ? (calMin[i] + calMax[i]) / 2 : constrain(raw[i], calMin[i] + 1, calMax[i] - 1);
          }
          portEXIT_CRITICAL(&cfgMux);
          saveSettings();
          showMsg("CALIBRATED");
        } else showMsg("CAL FAILED-RETRY");
        ui = UI_MENU; keyV = StickKey(); keyH = StickKey();
      }
      break;
    }
  }

  // buzzer: one-shot events, then FC low battery (2 beeps / 2 s), no telemetry while armed, TX low (long / 3 s)
  {
    bool off; portENTER_CRITICAL(&cfgMux); off = cfg.buzzerOff; portEXIT_CRITICAL(&cfgMux);
    bool on = false;
    if (evLen && now - evStart < evLen * 100UL) on = patternOn(evBits, evLen, now - evStart);
    else if (!off && link && (t.flags & 3) == 3) on = patternOn(0x5, 20, now);
    else if (!off && armRequest && lt && now - lt > 1000) on = patternOn(0x1, 10, now);
    else if (!off && txBattLow) on = patternOn(0xF, 30, now);
    buzzerSet(on);
  }

  static uint32_t drawMs = 0;
  if (now - drawMs >= 50) {
    drawMs = now;
    oled.clearBuffer();
    if (ui == UI_MAIN) drawMain(now); else if (ui == UI_MENU) drawMenu(); else drawCalib();
    if ((int32_t)(msgUntil - now) > 0) {
      oled.setFont(u8g2_font_6x10_tf);
      int w = oled.getStrWidth(msgText) + 8, x = (128 - w) / 2;
      oled.setDrawColor(0); oled.drawBox(x, 44, w, 14); oled.setDrawColor(1);
      oled.drawFrame(x, 44, w, 14);
      oled.drawStr(x + 4, 54, msgText);
    }
    oled.sendBuffer();
  }

  static uint32_t printMs = 0;
  if (Serial && now - printMs >= 100) {                  // 10 lines/s (USB only; the radio task is not affected)
    printMs = now;
    uint16_t c[NUM_CH]; float tv;
    portENTER_CRITICAL(&stMux); for (int i = 0; i < NUM_CH; i++) c[i] = chOut[i]; tv = txVbat; portEXIT_CRITICAL(&stMux);
    Serial.printf("CH %u %u %u %u %u %u | link %s FC %+ddBm %u%% TX %+.0fdBm %u%% | TX %.2fV\n", c[0], c[1], c[2], c[3], c[4], c[5],
                  link ? "ok" : "--", t.rssi, t.lq, txRssiF, txLq, tv);
  }
  delay(2);
}
