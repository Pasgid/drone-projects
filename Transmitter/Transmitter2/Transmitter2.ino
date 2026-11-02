/*
  ESP32-C3 Zero - 6 Channel ESP-NOW Transmitter
  ---------------------------------------------
  - 2 analog gimbals (4 pots), Mode 2: left = throttle/yaw, right = pitch/roll
  - 1 push button: short press = ARM / DISARM (throttle low), long press = MENU (disarmed)
    Armed: hold the button 2 s = RETURN TO HOME on / off (FC with GPS; CH5 1850 instead of 2000). Any shorter
    press disarms, as before.
  - 1.3" 128x64 SH1106 I2C OLED (SSD1306 also supported, see OLED_IS_SSD1306): live bars for all 6 channels + link quality + FC status
  - On-screen menu driven by the RIGHT stick:
        up/down    = move through the menu
        left/right = change the value
        short press = select (calibrate / save / exit)
        long press  = save & exit
  - 3-position ON-OFF-ON switch on CH6 (POS1 leg GPIO20, POS3 leg GPIO8, no resistors) = FLIGHT MODE on the FC:
    POS1 = SELF STAB, POS2 = ALT HOLD, POS3 = POS HOLD. It does NOT arm: arming is the button (short press).
  - FC battery voltage shown in the header (from telemetry); passive buzzer on GPIO21 beeps when it is LOW.
  - Settings (calibration, trims, reverse, expo, rates) saved to flash
  - Flight mode is set on the FC with stick commands (disarmed, throttle low):
        YAW full LEFT + ROLL full LEFT  for 1 s -> SELF (self-level)
        YAW full LEFT + ROLL full RIGHT for 1 s -> ACRO
    The OLED header shows the mode reported back by the FC.

  Channels sent:  CH1 THR | CH2 ROLL | CH3 PITCH | CH4 YAW | CH5 AUX1 = ARM (button or switch) | CH6 = 3-pos switch

  SIM MODE (menu > "Sim mode (BT)"): the TX restarts as a Bluetooth gamepad "ESP-NOW TX Sim" for FPV simulators
  (radio off). Axes X roll, Y pitch, Z throttle, RZ yaw, RX arm, RY CH6 switch; buttons 1 arm, 2 switch POS2, 3 switch POS3.
  Power-cycle (or menu > "Sim mode (BT)" = EXIT) to go back to flying.

  Board: "ESP32C3 Dev Module", USB CDC On Boot: Enabled
  Libraries: Adafruit SH110X (or Adafruit SSD1306 for a 0.96" SSD1306), Adafruit GFX, ESP32-BLE-Gamepad (+ NimBLE-Arduino)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <BleGamepad.h>
// 1.3" OLEDs are almost always SH1106; 0.96" ones are SSD1306. Uncomment for an SSD1306:
//#define OLED_IS_SSD1306
#if defined(OLED_IS_SSD1306)
  #include <Adafruit_SSD1306.h>
  #define OLED_WHITE SSD1306_WHITE
  #define OLED_BLACK SSD1306_BLACK
#else
  #include <Adafruit_SH110X.h>
  #define OLED_WHITE SH110X_WHITE
  #define OLED_BLACK SH110X_BLACK
#endif

// ====================== USER CONFIG ======================
// ---- Pins (ESP32-C3 Zero). Gimbal pots MUST be on ADC1 pins GPIO0-4 (ADC2 doesn't work with WiFi on) ----
enum { CH_THR, CH_ROLL, CH_PITCH, CH_YAW, CH_AUX1, CH_AUX2, NUM_CH };
static const int AXIS_PIN[4] = {
  1,  // THR   - left stick, vertical
  3,  // ROLL  - right stick, horizontal
  4,  // PITCH - right stick, vertical
  0   // YAW   - left stick, horizontal
};
static const int PIN_SDA = 5;
static const int PIN_SCL = 6;
static const int PIN_BTN = 7;  // button to GND (internal pull-up)

// CH6: 3-position ON-OFF-ON toggle switch, no resistors. Middle leg -> GND, outer legs -> these pins
// (internal pull-ups).
//   POS 1 (SW_PIN_A low)  = DISARM          -> CH6 1000
//   POS 2 (both high)     = ARM             -> CH6 1500
//   POS 3 (SW_PIN_B low)  = ARM + extra     -> CH6 2000  (extra is chosen on the FC: OFF / AIRMODE / BEEPER)
// If the positions come out backwards, swap the two outer wires.
#define USE_CH6_SWITCH 1       // 0 = no switch fitted: CH6 stays at 1000, arm with the button only
// Not GPIO10: on the ESP32-C3 Zero that is the on-board RGB LED's data line (the switch made it light up).
// GPIO8 is a boot strapping pin, so it gets the POS3 leg: the TX always starts in POS1, so GPIO8 is high at boot.
static const int SW_PIN_A = 20;   // POS1 (DISARM) leg
static const int SW_PIN_B = 8;    // POS3 (ARM + extra) leg

// TX battery voltage on GPIO2 through a resistor divider (the pin must stay below ~2.5 V):
//   battery+ --R1-- GPIO2 --R2-- GND, plus 100 nF from GPIO2 to GND.
//   1S (4.2 V) : R1 100k, R2 100k -> ratio 2.00 (default)
//   2S (8.4 V) : R1 100k, R2 33k  -> ratio 4.03
//   3S (12.6 V): R1 100k, R2 22k  -> ratio 5.55
// Set the ratio in the menu ("TX batt ratio") and nudge it until the menu title matches a multimeter.
// GPIO2 is a strapping pin; with this divider it only matters for manual download mode (hold BOOT + reset).
#define USE_TX_VBAT 1
static const int PIN_VBAT = 2;

// Passive buzzer (driven with a PWM tone). Beeps when the FC battery is low (from telemetry).
// Wiring: GPIO21 -> 1k -> NPN base, emitter -> GND, buzzer between collector and 3V3/5V
// (a small passive buzzer can also go straight from GPIO21 to GND through ~100 ohm).
// GPIO21 is the boot-log UART TX, so the buzzer may click briefly at power-up - that is normal.
static const int PIN_BUZZ  = 21;
static const int BUZZ_FREQ = 2700;     // Hz, most passive buzzers are loudest around 2-4 kHz

static const uint8_t OLED_ADDR = 0x3C;

// ---- Radio (MUST match the flight controller) ----
static const uint16_t PKT_MAGIC = 0xC3F1;
static const uint8_t  BIND_ID   = 42;
static const uint8_t  WIFI_CH   = 13;  // away from phone hotspots / routers (1, 6, 11). MUST match the FC
static const uint32_t SEND_PERIOD_MS = 20;  // 50 Hz

// ====================== PACKETS (keep identical to flight controller) ======================
struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  seq;
  uint16_t ch[NUM_CH];
  uint8_t  crc;
};

enum ArmBlock : uint8_t { AB_NONE, AB_THROTTLE, AB_SWITCH, AB_IMU, AB_TILT, AB_NOLINK, AB_SETUP };

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  armed;
  uint8_t  mode;
  uint8_t  armBlock;
  int16_t  roll10;
  int16_t  pitch10;
  uint16_t loopUs;
  uint16_t vbat100;   // FC pack voltage * 100 (0 = no battery)
  uint8_t  cells;     // FC cell count (0 = unknown)
  uint8_t  battFlags; // bit0 = battery present, bit1 = LOW battery, bit2 = obstacle ahead, bit3 = avoidance ON
  int8_t   rssi;      // dBm the FC receives this TX with (smoothed), 0 = unknown
  uint8_t  lq;        // % of this TX's packets the FC received in the last second
  uint8_t  crc;
};

// GPS state from the FC (5 Hz, its own size)
static const uint16_t GPS_TLM_MAGIC = 0xC3F5;
struct __attribute__((packed)) GpsTelemetry {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  sats;
  uint8_t  flags;     // bit0 GPS good, 1 home set, 2 compass OK, 3 RTH, 4 GPS hold, 5 compass found, 6 GPS data
  uint16_t distHome;  // m
  int16_t  alt10;     // dm
  uint8_t  crc;
};

// Types used in function signatures must be declared before the first function
// (the Arduino IDE inserts auto-generated prototypes there).
struct Settings {
  uint8_t  version;
  uint16_t cmin[4], ccen[4], cmax[4];  // raw ADC calibration per axis
  int8_t   trim[4];                    // us, -100..100 (roll/pitch/yaw)
  uint8_t  reverseMask;                // bit per axis
  uint8_t  expo;                       // 0..100 %
  uint8_t  rate;                       // 0 LOW, 1 MID, 2 HIGH
  uint8_t  buzzerOff;                  // 0 = buzzer on, 1 = off
  // ---- added in version 2: TX battery ----
  uint16_t vRatio100;                  // divider ratio (R1+R2)/R2 x100, e.g. 200 = 2.00
  uint8_t  cells;                      // TX battery cells, 0 = detect at power-up
  uint16_t warnCell100;                // low warning, volts per cell x100, e.g. 340 = 3.40 V
  // ---- added in version 3: deadbands, % of stick travel ----
  uint8_t  dbRP;                       // roll + pitch, around centre
  uint8_t  dbYaw;                      // yaw, around centre
  uint8_t  dbThr;                      // throttle, at the bottom (so it rests at exactly 1000)
  // ---- added in version 4 (uses the former padding byte, so the size is unchanged) ----
  uint8_t  filtIdx;                    // stick low-pass filter, index into FILTER_HZ (0 = off)
};
static const size_t SETTINGS_V1_SIZE = offsetof(Settings, vRatio100);   // saves from older versions
static const size_t SETTINGS_V2_SIZE = offsetof(Settings, dbRP);
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG, BTN_LONG_UP, BTN_HOLD2 };   // LONG at 0.8 s, HOLD2 at 2 s, LONG_UP = released in between

static uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ====================== SETTINGS ======================
static const uint8_t SETTINGS_VERSION = 4;
static const float   RATE_SCALE[3] = {0.6f, 0.8f, 1.0f};
// Stick filter cut-off frequencies (Hz). Lower = smoother but slightly slower to react. 0 = off.
static const uint8_t FILTER_HZ[] = {0, 25, 20, 15, 10, 7, 5, 3};
static const uint8_t FILTER_COUNT = sizeof(FILTER_HZ);
static const uint8_t FILTER_DEFAULT = 3;   // 15 Hz
static const char   *RATE_NAME[3]  = {"LOW", "MID", "HIGH"};

static Settings cfg;
static portMUX_TYPE cfgMux = portMUX_INITIALIZER_UNLOCKED;
static Preferences prefs;

static void defaultBattery(Settings &s) {
  s.vRatio100 = 200;       // 100k + 100k divider (1S Li-ion / LiPo)
  s.cells = 0;             // auto
  s.warnCell100 = 340;     // 3.40 V per cell
}

static void defaultDeadband(Settings &s) {
  s.dbRP = 5;              // %
  s.dbYaw = 10;
  s.dbThr = 3;
}

static void defaultSettings(Settings &s) {
  memset(&s, 0, sizeof(s));
  s.version = SETTINGS_VERSION;
  for (int i = 0; i < 4; i++) { s.cmin[i] = 150; s.ccen[i] = 2048; s.cmax[i] = 3950; }
  s.expo = 20;
  s.rate = 1;
  defaultBattery(s);
  defaultDeadband(s);
  s.filtIdx = FILTER_DEFAULT;
}

static void loadSettings() {
  Settings s;
  defaultSettings(s);
  prefs.begin("tx", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(s)) {
    prefs.getBytes("cfg", &s, sizeof(s));
    if (s.version == 3) {                    // version 3 has the same size: just add the filter default
      s.version = SETTINGS_VERSION;
      s.filtIdx = FILTER_DEFAULT;
    } else if (s.version != SETTINGS_VERSION) defaultSettings(s);
  } else if (n == SETTINGS_V1_SIZE || n == SETTINGS_V2_SIZE) {   // older version: keep what it had, default the rest
    uint8_t buf[sizeof(Settings)];
    prefs.getBytes("cfg", buf, n);
    uint8_t ver = buf[0];
    if ((ver == 1 && n == SETTINGS_V1_SIZE) || (ver == 2 && n == SETTINGS_V2_SIZE)) {
      memcpy(&s, buf, n);
      s.version = SETTINGS_VERSION;
      if (ver < 2) defaultBattery(s);
      defaultDeadband(s);
      s.filtIdx = FILTER_DEFAULT;
    }
  }
  prefs.end();
  if (s.buzzerOff > 1) s.buzzerOff = 0;
  if (s.vRatio100 < 100 || s.vRatio100 > 2000) s.vRatio100 = 200;
  if (s.cells > 6) s.cells = 0;
  if (s.warnCell100 < 300 || s.warnCell100 > 400) s.warnCell100 = 340;
  if (s.dbRP > 25 || s.dbYaw > 25 || s.dbThr > 25) defaultDeadband(s);
  if (s.filtIdx >= FILTER_COUNT) s.filtIdx = FILTER_DEFAULT;
  portENTER_CRITICAL(&cfgMux); cfg = s; portEXIT_CRITICAL(&cfgMux);
}

static void saveSettings() {
  Settings s;
  portENTER_CRITICAL(&cfgMux); s = cfg; portEXIT_CRITICAL(&cfgMux);
  prefs.begin("tx", false);
  prefs.putBytes("cfg", &s, sizeof(s));
  prefs.end();
}

// ====================== SHARED STATE (radio task <-> UI) ======================
static portMUX_TYPE stMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t rawAxis[4];
static volatile float    stickPos[4];       // calibrated + reversed, no expo: -1..1 (thr 0..1)
static volatile uint16_t chOut[NUM_CH] = {1000, 1500, 1500, 1500, 1000, 1000};
static volatile bool     armRequest = false;
static volatile bool     rthRequest = false;  // armed + return to home (hold the button 2 s)
static volatile bool     forceSafe  = true;  // true while in menu / calibration / boot
static bool              simMode    = false; // Bluetooth gamepad for simulators instead of ESP-NOW
static volatile uint8_t  ch6Pos     = 0;     // 0 = DISARM, 1 = ARM, 2 = ARM + extra (debounced switch)
static volatile float    txVbat     = 0;     // TX battery voltage, filtered (V), measured in the radio task
static volatile float    txPinMv    = 0;     // raw pin voltage (mV)

// TX battery state (UI side)
static bool    txBattPresent = false, txBattLow = false;
static uint8_t txCellsUsed = 0;

static void txBatteryTick(uint32_t now) {
#if USE_TX_VBAT
  static uint32_t lowSince = 0;
  float v;
  Settings c;
  portENTER_CRITICAL(&stMux); v = txVbat; portEXIT_CRITICAL(&stMux);
  portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);
  txBattPresent = v > 2.5f;                  // ~0 V when only USB powers the TX
  if (!txBattPresent) { txBattLow = false; lowSince = 0; return; }
  if (c.cells) txCellsUsed = c.cells;
  else if (txCellsUsed == 0 && now > 1500) txCellsUsed = constrain((int)(v / 4.35f) + 1, 1, 6);   // detect once
  if (!txCellsUsed) return;
  float cell = v / txCellsUsed, warn = c.warnCell100 / 100.0f;
  if (cell < warn) {                         // must stay low for 3 s
    if (lowSince == 0) lowSince = now ? now : 1;
    if (now - lowSince > 3000) txBattLow = true;
  } else if (cell > warn + 0.05f) {
    lowSince = 0;
    txBattLow = false;
  }
#endif
}

// Reads the 3-position switch
static uint8_t readSwitchPos() {
#if USE_CH6_SWITCH
  if (digitalRead(SW_PIN_A) == LOW) return 0;
  if (digitalRead(SW_PIN_B) == LOW) return 2;
  return 1;
#else
  return 0;
#endif
}

// Debounced: a position must be stable for 30 ms
static void pollSwitch() {
  static uint8_t last = 0;
  static uint32_t since = 0;
  uint8_t r = readSwitchPos();
  if (r != last) { last = r; since = millis(); return; }
  if (millis() - since >= 30) ch6Pos = r;
}

// ====================== PASSIVE BUZZER ======================
static void buzzerBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZ, BUZZ_FREQ, 8);
  ledcWrite(PIN_BUZZ, 0);
#else
  ledcSetup(0, BUZZ_FREQ, 8);
  ledcAttachPin(PIN_BUZZ, 0);
  ledcWrite(0, 0);
#endif
}

static void buzzerSet(bool on) {
  static bool state = false;
  if (on == state) return;
  state = on;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_BUZZ, on ? 128 : 0);   // 50 % duty square wave = tone
#else
  ledcWrite(0, on ? 128 : 0);
#endif
}

// Pattern bits, 100 ms per slot
static bool patternOn(uint32_t bits, uint8_t len, uint32_t t) { return (bits >> ((t / 100) % len)) & 1; }
static uint32_t buzzTestUntil = 0;

static TelemetryPacket   telem;
static volatile uint32_t lastTelemMs = 0;
static uint16_t          telemCount = 0;  // guarded by stMux
static uint8_t           linkQuality = 0;
static volatile int8_t   txRxRssi = 0;     // dBm this TX receives the FC's telemetry with (last packet)
static float             txRssiF = 0;      // smoothed

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ====================== RADIO ======================
// Sticks go out as BROADCAST until the FC is known; broadcasts are sent once, with no acknowledgement or retry.
// As soon as the first telemetry packet arrives, the FC's address is known and the sticks are sent to it
// directly (unicast): the radio then acknowledges every packet and retries lost ones automatically, which
// makes the link far more robust against short interference.
static uint8_t       fcMac[6];
static volatile bool fcMacKnown = false;
static bool          fcPeerAdded = false;

static GpsTelemetry      gtel;
static volatile uint32_t lastGpsMs = 0;

static void handleTelem(const uint8_t *mac, const uint8_t *data, int len, int8_t rssi) {
  if (len == sizeof(GpsTelemetry)) {
    GpsTelemetry g;
    memcpy(&g, data, sizeof(g));
    if (g.magic != GPS_TLM_MAGIC || g.bindId != BIND_ID || crc8((const uint8_t *)&g, sizeof(g) - 1) != g.crc) return;
    portENTER_CRITICAL(&stMux); gtel = g; lastGpsMs = millis(); portEXIT_CRITICAL(&stMux);
    return;
  }
  if (len != sizeof(TelemetryPacket)) return;
  TelemetryPacket t;
  memcpy(&t, data, sizeof(t));
  if (t.magic != PKT_MAGIC || t.bindId != BIND_ID) return;
  if (crc8((const uint8_t *)&t, sizeof(t) - 1) != t.crc) return;
  portENTER_CRITICAL(&stMux);
  telem = t;
  lastTelemMs = millis();
  telemCount++;
  txRxRssi = rssi;
  if (!fcMacKnown && mac) { memcpy(fcMac, mac, 6); fcMacKnown = true; }
  portEXIT_CRITICAL(&stMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handleTelem(info->src_addr, data, len, info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0);
}
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleTelem(mac, data, len, 0); }
#endif

static uint16_t readAxisRaw(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return sum / 4;
}

// Deadband: inside it the output is exactly 0; outside, the rest of the travel is rescaled so
// full stick is still full output (no jump at the edge of the deadband).
static float applyDeadband(float x, float db) {
  if (fabsf(x) <= db) return 0;
  return (x - copysignf(db, x)) / (1 - db);
}

static float normCentered(float v, float mn, float ce, float mx, float db) {
  float x;
  if (v >= ce) x = (mx > ce) ? (v - ce) / (mx - ce) : 0;
  else         x = (ce > mn) ? -(ce - v) / (ce - mn) : 0;
  x = constrain(x, -1.0f, 1.0f);
  return applyDeadband(x, db);
}

static void radioTask(void *) {
  ControlPacket pkt;
  pkt.magic = PKT_MAGIC;
  pkt.bindId = BIND_ID;
  pkt.seq = 0;
  TickType_t lastWake = xTaskGetTickCount();
  float filt[4] = {-1, -1, -1, -1};          // stick low-pass state (-1 = not started)

  for (;;) {
    Settings c;
    portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);

    // Stick filter: first-order low-pass on the raw readings, run at the 50 Hz send rate.
    // alpha = 1 - e^(-2*pi*fc*dt); 0 Hz = off (alpha 1 = no filtering).
    float fc = FILTER_HZ[c.filtIdx < FILTER_COUNT ? c.filtIdx : 0];
    float alpha = (fc > 0) ? 1.0f - expf(-2.0f * PI * fc * (SEND_PERIOD_MS / 1000.0f)) : 1.0f;

    uint16_t raw[4];
    float pos[4];
    for (int i = 0; i < 4; i++) {
      float r = readAxisRaw(AXIS_PIN[i]);
      filt[i] = (filt[i] < 0) ? r : filt[i] + alpha * (r - filt[i]);
      raw[i] = (uint16_t)(filt[i] + 0.5f);
    }

    // Throttle 0..1, others -1..1
    pos[CH_THR] = (c.cmax[0] > c.cmin[0]) ? constrain(((float)raw[0] - c.cmin[0]) / (c.cmax[0] - c.cmin[0]), 0.0f, 1.0f) : 0;
    if (c.reverseMask & 1) pos[CH_THR] = 1.0f - pos[CH_THR];
    {                                          // throttle deadband at the bottom only
      float dbT = c.dbThr / 100.0f;
      pos[CH_THR] = (pos[CH_THR] <= dbT) ? 0.0f : (pos[CH_THR] - dbT) / (1.0f - dbT);
    }
    for (int i = 1; i < 4; i++) {
      float db = ((i == CH_YAW) ? c.dbYaw : c.dbRP) / 100.0f;
      pos[i] = normCentered(raw[i], c.cmin[i], c.ccen[i], c.cmax[i], db);
      if (c.reverseMask & (1 << i)) pos[i] = -pos[i];
    }

    uint16_t ch[NUM_CH];
    float e = c.expo / 100.0f;
    float rs = RATE_SCALE[min<uint8_t>(c.rate, 2)];
    ch[CH_THR] = 1000 + (uint16_t)(pos[CH_THR] * 1000 + 0.5f);
    for (int i = 1; i < 4; i++) {
      float x = pos[i];
      x = x * (1 - e) + x * x * x * e;  // expo
      ch[i] = constrain((int)(1500 + x * 500 * rs + c.trim[i]), 1000, 2000);
    }

    bool safe = forceSafe;
    bool arm = armRequest && !safe;
    if (safe) {
      ch[CH_THR] = 1000;
      ch[CH_ROLL] = ch[CH_PITCH] = ch[CH_YAW] = 1500;
    }
    ch[CH_AUX1] = arm ? (rthRequest ? 1850 : 2000) : 1000;   // 1850 = armed + RTH
    ch[CH_AUX2] = 1000 + 500 * ch6Pos;   // 3-position switch: 1000 / 1500 / 2000

    pkt.seq++;
    memcpy(pkt.ch, ch, sizeof(ch));
    pkt.crc = crc8((const uint8_t *)&pkt, sizeof(pkt) - 1);
    if (fcMacKnown && !fcPeerAdded) {         // first telemetry seen: add the FC as a direct (acknowledged) peer
      esp_now_peer_info_t fp = {};
      memcpy(fp.peer_addr, fcMac, 6);
      fp.channel = WIFI_CH;
      fp.ifidx = WIFI_IF_STA;
      fp.encrypt = false;
      fcPeerAdded = (esp_now_add_peer(&fp) == ESP_OK) || esp_now_is_peer_exist(fcMac);
    }
    if (!simMode) esp_now_send(fcPeerAdded ? fcMac : BCAST, (const uint8_t *)&pkt, sizeof(pkt));

    portENTER_CRITICAL(&stMux);
    for (int i = 0; i < 4; i++) { rawAxis[i] = raw[i]; stickPos[i] = pos[i]; }
    for (int i = 0; i < NUM_CH; i++) chOut[i] = ch[i];
    portEXIT_CRITICAL(&stMux);

#if USE_TX_VBAT
    {                                         // TX battery: calibrated mV, low-passed (~1 s)
      uint32_t mv = 0;
      for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(PIN_VBAT);
      float pinMv = mv / 4.0f;
      float v = pinMv * 0.001f * (c.vRatio100 / 100.0f);
      float old = txVbat;
      float nv = (old == 0) ? v : old + 0.02f * (v - old);
      portENTER_CRITICAL(&stMux); txVbat = nv; txPinMv = pinMv; portEXIT_CRITICAL(&stMux);
    }
#endif

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SEND_PERIOD_MS));
  }
}

static void radioBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_country_t ctry = {"EU", 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL};   // allow channels 1-13
  esp_wifi_set_country(&ctry);
  esp_err_t chErr = esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  Serial.printf("ESP-NOW channel %d: %s\n", WIFI_CH, chErr == ESP_OK ? "ok" : "FAILED");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CH;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_wifi_set_max_tx_power(80);            // full power (20 dBm)
  Serial.print("TX MAC: ");
  Serial.println(WiFi.macAddress());
}

// ====================== BUTTON ======================
static BtnEvent pollButton() {
  static bool stable = HIGH, lastRead = HIGH, longFired = false, holdFired = false;
  static uint32_t changeMs = 0, pressMs = 0;
  uint32_t now = millis();
  bool r = digitalRead(PIN_BTN);
  if (r != lastRead) { lastRead = r; changeMs = now; }
  if (now - changeMs > 25 && r != stable) {
    stable = r;
    if (stable == LOW) { pressMs = now; longFired = holdFired = false; }
    else if (!longFired) return BTN_SHORT;
    else if (!holdFired) return BTN_LONG_UP;
  }
  if (stable == LOW && !longFired && now - pressMs > 800) { longFired = true; return BTN_LONG; }
  if (stable == LOW && longFired && !holdFired && now - pressMs > 2000) { holdFired = true; return BTN_HOLD2; }
  return BTN_NONE;
}

// ====================== DISPLAY / UI ======================
#if defined(OLED_IS_SSD1306)
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#else
Adafruit_SH1106G display(128, 64, &Wire, -1);
#endif

enum UiState { UI_MAIN, UI_MENU, UI_CALIB };
static UiState ui = UI_MAIN;

static char     msgText[22] = "";
static uint32_t msgUntil = 0;
static void showMsg(const char *m, uint32_t ms = 1200) {
  strncpy(msgText, m, sizeof(msgText) - 1);
  msgText[sizeof(msgText) - 1] = 0;
  msgUntil = millis() + ms;
}

enum MenuItem {
  MI_RATE, MI_EXPO, MI_TRIM_R, MI_TRIM_P, MI_TRIM_Y,
  MI_REV_T, MI_REV_R, MI_REV_P, MI_REV_Y, MI_DB_RP, MI_DB_Y, MI_DB_T, MI_FILT, MI_CALIB, MI_BUZZ,
  MI_VRATIO, MI_CELLS, MI_WARN, MI_SIM, MI_SAVE, MI_EXIT, MI_COUNT
};
static const char *MENU_LABEL[MI_COUNT] = {
  "Rates", "Expo", "Trim roll", "Trim pitch", "Trim yaw",
  "Rev throttle", "Rev roll", "Rev pitch", "Rev yaw",
  "Deadband R/P", "Deadband yaw", "Deadband thr", "Stick filter", "Calibrate sticks", "Buzzer",
  "TX batt ratio", "TX cells", "TX warn V/cell", "Sim mode (BT)",
  "Save & exit", "Exit (no save)"
};
static int menuSel = 0, menuTop = 0;
static const int MENU_ROWS = 5;

static void menuValue(int item, char *buf, size_t n) {
  Settings c;
  portENTER_CRITICAL(&cfgMux); c = cfg; portEXIT_CRITICAL(&cfgMux);
  buf[0] = 0;
  switch (item) {
    case MI_RATE:   snprintf(buf, n, "%s", RATE_NAME[min<uint8_t>(c.rate, 2)]); break;
    case MI_EXPO:   snprintf(buf, n, "%u%%", c.expo); break;
    case MI_TRIM_R: snprintf(buf, n, "%+d", c.trim[CH_ROLL]); break;
    case MI_TRIM_P: snprintf(buf, n, "%+d", c.trim[CH_PITCH]); break;
    case MI_TRIM_Y: snprintf(buf, n, "%+d", c.trim[CH_YAW]); break;
    case MI_REV_T: case MI_REV_R: case MI_REV_P: case MI_REV_Y:
      snprintf(buf, n, "%s", (c.reverseMask & (1 << (item - MI_REV_T))) ? "REV" : "NOR"); break;
    case MI_BUZZ:   snprintf(buf, n, "%s", c.buzzerOff ? "OFF" : "ON"); break;
    case MI_DB_RP:  snprintf(buf, n, "%u%%", c.dbRP); break;
    case MI_DB_Y:   snprintf(buf, n, "%u%%", c.dbYaw); break;
    case MI_DB_T:   snprintf(buf, n, "%u%%", c.dbThr); break;
    case MI_FILT:   if (FILTER_HZ[c.filtIdx]) snprintf(buf, n, "%uHz", FILTER_HZ[c.filtIdx]); else snprintf(buf, n, "OFF"); break;
    case MI_VRATIO: snprintf(buf, n, "%.2f", c.vRatio100 / 100.0f); break;
    case MI_CELLS:  if (c.cells) snprintf(buf, n, "%uS", c.cells); else snprintf(buf, n, "AUTO"); break;
    case MI_WARN:   snprintf(buf, n, "%.2f", c.warnCell100 / 100.0f); break;
    case MI_SIM:    snprintf(buf, n, "%s", simMode ? "EXIT" : "START"); break;
    case MI_CALIB: case MI_SAVE: case MI_EXIT: snprintf(buf, n, ">"); break;
  }
}

static void menuAdjust(int item, int dir) {
  portENTER_CRITICAL(&cfgMux);
  switch (item) {
    case MI_RATE:   cfg.rate = constrain((int)cfg.rate + dir, 0, 2); break;
    case MI_EXPO:   cfg.expo = constrain((int)cfg.expo + dir * 5, 0, 100); break;
    case MI_TRIM_R: cfg.trim[CH_ROLL]  = constrain(cfg.trim[CH_ROLL]  + dir * 2, -100, 100); break;
    case MI_TRIM_P: cfg.trim[CH_PITCH] = constrain(cfg.trim[CH_PITCH] + dir * 2, -100, 100); break;
    case MI_TRIM_Y: cfg.trim[CH_YAW]   = constrain(cfg.trim[CH_YAW]   + dir * 2, -100, 100); break;
    case MI_REV_T: case MI_REV_R: case MI_REV_P: case MI_REV_Y:
      cfg.reverseMask ^= (1 << (item - MI_REV_T)); break;
    case MI_BUZZ:   cfg.buzzerOff = !cfg.buzzerOff; break;
    case MI_DB_RP:  cfg.dbRP  = constrain((int)cfg.dbRP  + dir, 0, 25); break;
    case MI_DB_Y:   cfg.dbYaw = constrain((int)cfg.dbYaw + dir, 0, 25); break;
    case MI_DB_T:   cfg.dbThr = constrain((int)cfg.dbThr + dir, 0, 25); break;
    case MI_FILT:   cfg.filtIdx = constrain((int)cfg.filtIdx + dir, 0, FILTER_COUNT - 1); break;   // right = smoother
    case MI_VRATIO: cfg.vRatio100 = constrain((int)cfg.vRatio100 + dir, 100, 2000); break;   // 0.01 steps
    case MI_CELLS:  cfg.cells = (uint8_t)((cfg.cells + dir + 7) % 7); break;                 // AUTO,1..6
    case MI_WARN:   cfg.warnCell100 = constrain((int)cfg.warnCell100 + dir * 5, 300, 400); break;
  }
  bool buzzOn = !cfg.buzzerOff;
  portEXIT_CRITICAL(&cfgMux);
  if (item == MI_BUZZ && buzzOn) buzzTestUntil = millis() + 500;   // test beep when switched on
}

// Stick "key" with auto-repeat: returns -1/0/+1 events
struct StickKey {
  int last = 0;
  uint32_t nextMs = 0;
  int poll(float v) {
    int d = v > 0.6f ? 1 : (v < -0.6f ? -1 : 0);
    uint32_t now = millis();
    int ev = 0;
    if (d != 0 && d != last)      { ev = d; nextMs = now + 400; }
    else if (d != 0 && now >= nextMs) { ev = d; nextMs = now + 150; }
    last = d;
    return ev;
  }
};
static StickKey keyV, keyH;

// Calibration state
static int      calStep = 0;
static uint16_t calMin[4], calMax[4];

// ====================== SIM MODE (BLE gamepad) ======================
// The flag only lives for one boot: choosing SIM MODE saves it and restarts, and that boot clears it again,
// so the next power-up is always the normal transmitter.
static BleGamepad *gamepad = nullptr;
static void restartSim(bool on) {
  prefs.begin("tx", false); prefs.putUChar("sim", on ? 1 : 0); prefs.end();
  display.clearDisplay();
  display.setCursor(10, 28);
  display.print(on ? "Starting SIM MODE" : "Leaving SIM MODE");
  display.display();
  delay(300);
  ESP.restart();
}
static bool takeSimFlag() {
  prefs.begin("tx", false);
  bool on = prefs.getUChar("sim", 0);
  if (on) prefs.putUChar("sim", 0);
  prefs.end();
  return on;
}
static void simBegin() {
  gamepad = new BleGamepad("ESP-NOW TX Sim", "Espressif", 100);
  BleGamepadConfiguration gc;
  gc.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  gc.setAutoReport(false);
  gc.setButtonCount(3);
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
  if (!gamepad || now - ms < SEND_PERIOD_MS) return;
  ms = now;
  if (!gamepad->isConnected()) return;
  uint16_t c[NUM_CH];
  portENTER_CRITICAL(&stMux); for (int i = 0; i < NUM_CH; i++) c[i] = chOut[i]; portEXIT_CRITICAL(&stMux);
  if (!memcmp(c, last, sizeof c) && now - lastSend < 500) return;
  memcpy(last, c, sizeof c);
  lastSend = now;
  gamepad->setAxes(usToAxis(c[CH_ROLL]), usToAxis(c[CH_PITCH]), usToAxis(c[CH_THR]),
                   usToAxis(c[CH_AUX1]), usToAxis(c[CH_AUX2]), usToAxis(c[CH_YAW]));
  if (c[CH_AUX1] > 1500) gamepad->press(BUTTON_1); else gamepad->release(BUTTON_1);
  if (c[CH_AUX2] > 1250 && c[CH_AUX2] < 1750) gamepad->press(BUTTON_2); else gamepad->release(BUTTON_2);
  if (c[CH_AUX2] >= 1750) gamepad->press(BUTTON_3); else gamepad->release(BUTTON_3);
  gamepad->sendReport();
}
static void drawSimHeader(bool blink) {
  bool con = gamepad && gamepad->isConnected();
  display.fillRect(0, 0, 52, 8, OLED_WHITE);
  display.setTextColor(OLED_BLACK);
  display.setCursor(2, 0);
  display.print("SIM MODE");
  display.setTextColor(OLED_WHITE);
  display.setCursor(56, 0);
  display.print(con ? "BT CONNECTED" : (blink ? "BT PAIRING.." : ""));
  display.setCursor(2, 8);
  display.print(armRequest ? "ARM ON" : "arm off");
#if USE_TX_VBAT
  float tv;
  portENTER_CRITICAL(&stMux); tv = txVbat; portEXIT_CRITICAL(&stMux);
  display.setCursor(68, 8);
  if (txBattPresent) display.printf("TX %5.2fV", tv);
  else               display.print("TX  USB");
#endif
}

static void enterMenu() {
  forceSafe = true;
  ui = UI_MENU;
  menuSel = 0; menuTop = 0;
  keyV = StickKey(); keyH = StickKey();
}

static void exitMenu() {
  ui = UI_MAIN;
  armRequest = false;
  rthRequest = false;
  forceSafe = false;
}

static void startCalib() {
  ui = UI_CALIB;
  calStep = 0;
  for (int i = 0; i < 4; i++) { calMin[i] = 4095; calMax[i] = 0; }
}

static void drawBar(int x, int y, int w, int h, uint16_t us, bool centred) {
  display.drawRect(x, y, w, h, OLED_WHITE);
  int inner = w - 2;
  int pos = (int)(us - 1000) * inner / 1000;
  if (centred) {
    int mid = x + 1 + inner / 2;
    int p = x + 1 + pos;
    if (p >= mid) display.fillRect(mid, y + 1, p - mid + 1, h - 2, OLED_WHITE);
    else          display.fillRect(p, y + 1, mid - p, h - 2, OLED_WHITE);
    display.drawFastVLine(mid, y - 1, h + 2, OLED_WHITE);
  } else {
    display.fillRect(x + 1, y + 1, pos, h - 2, OLED_WHITE);
  }
}

static const char *armBlockText(uint8_t b) {
  switch (b) {
    case AB_THROTTLE: return "THR HIGH";
    case AB_SWITCH:   return "RE-ARM";
    case AB_IMU:      return "IMU ERR";
    case AB_TILT:     return "TILTED";
    case AB_NOLINK:   return "NO LINK";
    case AB_SETUP:    return "IN SETUP";
    default:          return "";
  }
}

static void drawMain() {
  uint16_t ch[NUM_CH];
  TelemetryPacket t;
  portENTER_CRITICAL(&stMux);
  for (int i = 0; i < NUM_CH; i++) ch[i] = chOut[i];
  t = telem;
  uint32_t lt = lastTelemMs;
  portEXIT_CRITICAL(&stMux);
  bool linkOk = lt != 0 && millis() - lt < 500;
  bool blink = (millis() / 300) & 1;

  display.setTextSize(1);
  if (simMode) drawSimHeader(blink);
  else {
  // Header line 1: arm state | mode | link quality
  if (linkOk && t.armed) {
    display.fillRect(0, 0, 34, 8, OLED_WHITE);
    display.setTextColor(OLED_BLACK);
    display.setCursor(2, 0);
    display.print("ARMED");
    display.setTextColor(OLED_WHITE);
  } else if (armRequest) {
    display.setCursor(2, 0);
    if (blink) display.print("ARM?");
  } else {
    display.setCursor(2, 0);
    display.print("SAFE");
  }
  display.setCursor(44, 0);
  if (rthRequest && !(linkOk && t.mode == 4)) { if (blink) display.print("RTH?"); }   // asked, FC not (yet) flying it
  else if (linkOk) display.print(t.mode == 4 ? "RTH" : t.mode == 3 ? "POS" : (t.mode == 2 ? "ALT" : (t.mode ? "ACRO" : "SELF")));  // mode from the FC
  if (linkOk) {
    // signal of the WEAKER direction (FC hears TX / TX hears FC) and the lower link quality of the two
    int rFc = t.rssi, rTx = (int)lroundf(txRssiF);
    int r = (rFc && rTx) ? min(rFc, rTx) : (rFc ? rFc : rTx);
    uint8_t lq = min<uint8_t>(linkQuality, t.lq ? t.lq : linkQuality);
    bool weak = (r != 0 && r < -85) || lq < 60;      // getting close to losing the link
    if (weak && blink) { display.fillRect(84, 0, 44, 8, OLED_WHITE); display.setTextColor(OLED_BLACK); }
    display.setCursor(86, 0);
    display.printf("%3d %2u%%", r, min<uint8_t>(lq, 99));
    display.setTextColor(OLED_WHITE);
  } else {
    display.setCursor(86, 0);
    display.print("NO LINK");
  }

  // Header line 2: FC battery | TX battery (each blinks inverted when LOW); with GPS data also sats + home distance
  GpsTelemetry g;
  uint32_t lg;
  portENTER_CRITICAL(&stMux); g = gtel; lg = lastGpsMs; portEXIT_CRITICAL(&stMux);
  if (lg && millis() - lg < 1000) {
    bool fcPresent = linkOk && (t.battFlags & 1), fcLow = t.battFlags & 2;
    if (fcPresent && fcLow && blink) { display.fillRect(0, 8, 40, 8, OLED_WHITE); display.setTextColor(OLED_BLACK); }
    display.setCursor(2, 8);
    if (fcPresent) display.printf("FC%4.1f", t.vbat100 / 100.0f);
    else           display.print("FC --");
    display.setTextColor(OLED_WHITE);
    display.setCursor(44, 8);
    if (!(g.flags & 64))     display.print("NO GPS");
    else if (g.flags & 1)    display.printf("%2us%4um", g.sats, (unsigned)min<uint16_t>(g.distHome, 9999));
    else                     display.printf("%2us WAIT", g.sats);
#if USE_TX_VBAT
    float tv;
    portENTER_CRITICAL(&stMux); tv = txVbat; portEXIT_CRITICAL(&stMux);
    if (txBattPresent && txBattLow && blink) { display.fillRect(96, 8, 32, 8, OLED_WHITE); display.setTextColor(OLED_BLACK); }
    display.setCursor(98, 8);
    if (txBattPresent) display.printf("T%3.1f", tv);
    else               display.print("TUSB");
    display.setTextColor(OLED_WHITE);
#endif
  } else
  {
    bool fcPresent = linkOk && (t.battFlags & 1), fcLow = t.battFlags & 2;
    if (fcPresent && fcLow && blink) { display.fillRect(0, 8, 60, 8, OLED_WHITE); display.setTextColor(OLED_BLACK); }
    display.setCursor(2, 8);
    if (fcPresent) display.printf("FC %5.2fV", t.vbat100 / 100.0f);
    else           display.print("FC  --  ");
    display.setTextColor(OLED_WHITE);

#if USE_TX_VBAT
    float tv;
    portENTER_CRITICAL(&stMux); tv = txVbat; portEXIT_CRITICAL(&stMux);
    if (txBattPresent && txBattLow && blink) { display.fillRect(64, 8, 64, 8, OLED_WHITE); display.setTextColor(OLED_BLACK); }
    display.setCursor(68, 8);
    if (txBattPresent) display.printf("TX %5.2fV", tv);
    else               display.print("TX  USB");
    display.setTextColor(OLED_WHITE);
#endif
  }
  }
  display.drawFastHLine(0, 17, 128, OLED_WHITE);

  // Channel rows, 7 px apart
  static const char *NAMES[NUM_CH] = {"THR", "ROL", "PIT", "YAW", "ARM", "CH6"};
  for (int i = 0; i < NUM_CH; i++) {
    int y = 19 + i * 7;
    display.setCursor(0, y);
    display.print(NAMES[i]);
    display.setCursor(21, y);
    display.printf("%4u", ch[i]);
    drawBar(48, y, 80, 6, ch[i], i >= CH_ROLL && i <= CH_YAW);
  }

  // Obstacle ahead (forward VL53L1X on the FC)
  if (linkOk && (t.battFlags & 4)) {
    display.fillRect(28, 44, 72, 12, OLED_WHITE);
    display.setTextColor(OLED_BLACK);
    display.setCursor(40, 46);
    display.print("OBSTACLE");
    display.setTextColor(OLED_WHITE);
  }

  // Why won't it arm?
  if (armRequest && linkOk && !t.armed && t.armBlock != AB_NONE) {
    display.fillRect(20, 26, 88, 14, OLED_BLACK);
    display.drawRect(20, 26, 88, 14, OLED_WHITE);
    display.setCursor(26, 30);
    display.printf("FC: %s", armBlockText(t.armBlock));
  }
}

static void drawMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
#if USE_TX_VBAT
  {                                          // live TX voltage: match it to a multimeter with "TX batt ratio"
    float tv;
    portENTER_CRITICAL(&stMux); tv = txVbat; portEXIT_CRITICAL(&stMux);
    display.printf("MENU  TX %5.2fV", tv);
  }
#else
  display.print("MENU   hold btn=save");
#endif
  display.drawFastHLine(0, 9, 128, OLED_WHITE);

  if (menuSel < menuTop) menuTop = menuSel;
  if (menuSel >= menuTop + MENU_ROWS) menuTop = menuSel - MENU_ROWS + 1;

  for (int r = 0; r < MENU_ROWS; r++) {
    int item = menuTop + r;
    if (item >= MI_COUNT) break;
    int y = 12 + r * 10;
    bool sel = item == menuSel;
    if (sel) display.fillRect(0, y - 1, 124, 10, OLED_WHITE);
    display.setTextColor(sel ? OLED_BLACK : OLED_WHITE);
    display.setCursor(2, y);
    display.print(MENU_LABEL[item]);
    char v[10];
    menuValue(item, v, sizeof(v));
    display.setCursor(122 - (int)strlen(v) * 6, y);
    display.print(v);
    display.setTextColor(OLED_WHITE);
  }
  // scrollbar
  int barH = 50 * MENU_ROWS / MI_COUNT;
  int barY = 11 + (50 - barH) * menuTop / (MI_COUNT - MENU_ROWS);
  display.fillRect(126, barY, 2, barH, OLED_WHITE);
}

static void drawCalib() {
  uint16_t raw[4];
  portENTER_CRITICAL(&stMux);
  for (int i = 0; i < 4; i++) raw[i] = rawAxis[i];
  portEXIT_CRITICAL(&stMux);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("STICK CALIBRATION");
  display.drawFastHLine(0, 9, 128, OLED_WHITE);
  display.setCursor(0, 12);
  if (calStep == 0) {
    display.println("Move BOTH sticks to");
    display.println("all corners, then");
    display.println("press button.");
  } else {
    display.println("Centre roll/pitch/yaw");
    display.println("(throttle anywhere)");
    display.println("then press button.");
  }
  static const char *N[4] = {"T", "R", "P", "Y"};
  for (int i = 0; i < 4; i++) {
    display.setCursor((i % 2) * 64, 44 + (i / 2) * 10);
    if (calStep == 0) display.printf("%s%4u %3u%%", N[i], raw[i], calMax[i] > calMin[i] ? (calMax[i] - calMin[i]) * 100 / 4095 : 0);
    else              display.printf("%s %4u", N[i], raw[i]);
  }
}

// ====================== SETUP / LOOP ======================
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // never wait on USB serial when no serial monitor is open
  pinMode(PIN_BTN, INPUT_PULLUP);
#if USE_CH6_SWITCH
  pinMode(SW_PIN_A, INPUT_PULLUP);
  pinMode(SW_PIN_B, INPUT_PULLUP);
#endif
  buzzerBegin();
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) analogSetPinAttenuation(AXIS_PIN[i], ADC_11db);
#if USE_TX_VBAT
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  {                                          // battery pin self-test (serial only): what is connected to the pin?
    int rNormal = analogRead(PIN_VBAT);
    gpio_pullup_en((gpio_num_t)PIN_VBAT);    // weak internal pull-up (~45k) to 3.3 V
    delay(20);
    int rPull = analogRead(PIN_VBAT);
    gpio_pullup_dis((gpio_num_t)PIN_VBAT);
    delay(20);
    Serial.printf("TX VBAT pin test (GPIO%d): normal raw %d (%u mV), with pull-up raw %d  ", PIN_VBAT, rNormal,
                  (unsigned)analogReadMilliVolts(PIN_VBAT), rPull);
    if (rPull > 3500)                        Serial.println("-> NOTHING connected to the battery pin");
    else if (rPull < 100)                    Serial.println("-> battery pin SHORTED to GND");
    else if (rNormal < 100 && rPull < 1600)  Serial.println("-> only the resistor to GND: the battery side is open");
    else                                     Serial.println("-> divider connected");
  }
#endif

  Wire.begin(PIN_SDA, PIN_SCL);
#if defined(OLED_IS_SSD1306)
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) Serial.println("OLED not found");
#else
  if (!display.begin(OLED_ADDR, true)) Serial.println("OLED not found");
#endif
  Wire.setClock(400000);
  display.clearDisplay();
  display.setTextColor(OLED_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.print("ESP-NOW");
  display.setTextSize(1);
  display.setCursor(10, 34);
  simMode = takeSimFlag();
  display.print(simMode ? "SIM MODE (BT)" : "6CH Transmitter");
  display.display();

  loadSettings();
  if (simMode) simBegin(); else radioBegin();
  xTaskCreate(radioTask, "radio", 4096, nullptr, 5, nullptr);
  delay(800);

  // Safety: refuse to leave the boot screen with the throttle up
  for (;;) {
    float thr;
    portENTER_CRITICAL(&stMux); thr = stickPos[CH_THR]; portEXIT_CRITICAL(&stMux);
    pollSwitch();
    if (thr < 0.05f) break;
    if (pollButton() == BTN_LONG) { enterMenu(); return; }  // escape hatch to recalibrate
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("THROTTLE NOT LOW!");
    display.println();
    display.println("Lower throttle, or");
    display.println("hold button for menu");
    display.println("to calibrate sticks.");
    display.display();
    delay(50);
  }
  forceSafe = false;
}

void loop() {
  uint32_t now = millis();
  BtnEvent btn = pollButton();
  txBatteryTick(now);
  pollSwitch();
  if (simMode) simTick(now);

  // Link quality: telemetry is sent at 20 Hz
  static uint32_t lqMs = 0;
  if (now - lqMs >= 1000) {
    lqMs = now;
    uint16_t n;
    portENTER_CRITICAL(&stMux); n = telemCount; telemCount = 0; portEXIT_CRITICAL(&stMux);
    linkQuality = min<uint16_t>(100, n * 5);
  }
  static uint32_t rsMs = 0;                  // smoothed telemetry RSSI (~0.4 s)
  if (now - rsMs >= 20) {
    rsMs = now;
    int8_t r = txRxRssi;
    if (r != 0) txRssiF = (txRssiF == 0) ? r : txRssiF + 0.05f * (r - txRssiF);
  }

  // No telemetry from the FC: WARN only. (This used to drop the arm request, which disarmed the quad in flight
  // whenever the FC->TX direction lost packets for 1 s even though the FC still received the sticks fine.
  // The FC disarms itself if it really loses the radio, and it cannot re-arm by surprise after that.)
  uint32_t lt;
  portENTER_CRITICAL(&stMux); lt = lastTelemMs; portEXIT_CRITICAL(&stMux);
  static uint32_t lastWarn = 0;
  if (armRequest && lt != 0 && now - lt > 1000 && now - lastWarn > 3000) {
    lastWarn = now;
    showMsg("NO TELEMETRY", 2000);
  }

  float pitchPos, rollPos, thrPos;
  portENTER_CRITICAL(&stMux);
  pitchPos = stickPos[CH_PITCH]; rollPos = stickPos[CH_ROLL]; thrPos = stickPos[CH_THR];
  portEXIT_CRITICAL(&stMux);

  // (CH6 no longer arms: it only selects the flight mode on the FC. Arming = short press of the button.)

  switch (ui) {
    case UI_MAIN:
      if (btn == BTN_SHORT || (btn == BTN_LONG_UP && armRequest)) {   // armed: any press under 2 s disarms
        if (armRequest)          { armRequest = false; rthRequest = false; showMsg("DISARMED"); }
        else if (thrPos > 0.05f) showMsg("LOWER THROTTLE!");
        else                     armRequest = true;
      } else if (btn == BTN_LONG) {
        if (armRequest) showMsg("KEEP HOLDING: RTH");
        else            enterMenu();
      } else if (btn == BTN_HOLD2 && armRequest) {                    // 2 s: return to home on / off
        rthRequest = !rthRequest;
        showMsg(rthRequest ? "RETURN TO HOME" : "RTH CANCELLED", 2000);
        buzzTestUntil = millis() + (rthRequest ? 600 : 200);
      }
      break;

    case UI_MENU: {
      int v = keyV.poll(pitchPos);
      int h = keyH.poll(rollPos);
      if (v > 0) menuSel = (menuSel + MI_COUNT - 1) % MI_COUNT;  // stick up
      if (v < 0) menuSel = (menuSel + 1) % MI_COUNT;            // stick down
      if (h != 0) menuAdjust(menuSel, h);
      if (btn == BTN_SHORT) {
        switch (menuSel) {
          case MI_CALIB: startCalib(); break;
          case MI_SIM:   saveSettings(); restartSim(!simMode); break;
          case MI_SAVE:  saveSettings(); showMsg("SAVED"); exitMenu(); break;
          case MI_EXIT:  loadSettings(); exitMenu(); break;
          default:       menuAdjust(menuSel, 1); break;
        }
      } else if (btn == BTN_LONG) {
        saveSettings(); showMsg("SAVED"); exitMenu();
      }
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
            cfg.cmin[i] = calMin[i];
            cfg.cmax[i] = calMax[i];
            cfg.ccen[i] = (i == CH_THR) ? (calMin[i] + calMax[i]) / 2 : constrain(raw[i], calMin[i] + 1, calMax[i] - 1);
          }
          portEXIT_CRITICAL(&cfgMux);
          saveSettings();
          showMsg("CALIBRATED");
        } else {
          showMsg("CAL FAILED-RETRY");
        }
        ui = UI_MENU;
        keyV = StickKey(); keyH = StickKey();
      }
      break;
    }
  }

  // Passive buzzer: FC battery LOW -> two short beeps every 2 s (and a test beep from the menu)
  {
    TelemetryPacket t;
    portENTER_CRITICAL(&stMux); t = telem; portEXIT_CRITICAL(&stMux);
    bool linkOk = lt != 0 && now - lt < 1000;
    bool fcLow = linkOk && (t.battFlags & 3) == 3;
    bool buzzOff;
    portENTER_CRITICAL(&cfgMux); buzzOff = cfg.buzzerOff; portEXIT_CRITICAL(&cfgMux);
    bool on = false;
    if ((int32_t)(buzzTestUntil - now) > 0) on = true;
    else if (linkOk && (t.battFlags & 12) == 12 && !buzzOff) on = patternOn(0x1, 3, now);   // obstacle ahead, avoidance ON: fast beeps
    else if (fcLow && !buzzOff)             on = patternOn(0x5, 20, now);   // FC low: beep-beep ... every 2 s
    else if (txBattLow && !buzzOff)         on = patternOn(0xF, 30, now);   // TX low: one long beep every 3 s
    buzzerSet(on);
  }

  // Redraw at ~20 fps
  static uint32_t drawMs = 0;
  if (now - drawMs >= 50) {
    drawMs = now;
    display.clearDisplay();
    if (ui == UI_MAIN)      drawMain();
    else if (ui == UI_MENU) drawMenu();
    else                    drawCalib();
    if ((int32_t)(msgUntil - now) > 0) {
      int w = strlen(msgText) * 6 + 8;
      int x = (128 - w) / 2;
      display.fillRect(x, 44, w, 14, OLED_BLACK);
      display.drawRect(x, 44, w, 14, OLED_WHITE);
      display.setCursor(x + 4, 47);
      display.print(msgText);
    }
    display.display();
  }
  delay(2);
}
