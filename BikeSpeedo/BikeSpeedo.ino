/*
 * BikeSpeedo: wheel-sensor speedometer for a motorbike or e-bike.
 *
 * ESP32-C3 SuperMini + SH1106 1.3" OLED + A3144 hall sensor + one button.
 * Wiring and setup: see README.md.
 *
 * Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled, Partition: Default 4MB (with spiffs).
 * Libraries: U8g2. Everything else ships with the ESP32 core.
 *
 * Button: short press = next page.
 *         hold 1.5 s  = reset trip (TRIP page) or Wi-Fi on/off (WI-FI page). Ignored while moving.
 *
 * Serial (115200): "?" lists settings, name=value changes one, "sim=80" fakes 80 km/h for bench
 * tests (sim=0 stops), "wifi" toggles Wi-Fi, "reset" resets the trip.
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_sleep.h>

// ---------------------------------------------------------------- pins (deep-sleep wake needs GPIO0-5)
const int PIN_BTN  = 0;   // to GND, 10k pull-up to 3V3
const int PIN_HALL = 1;   // A3144 output through 1k, 10k pull-up to 3V3, 10 nF to GND
const int PIN_VIN  = 4;   // supply voltage divider
const int PIN_SDA  = 6;
const int PIN_SCL  = 7;

//#define OLED_IS_SSD1306   // uncomment if your 128x64 OLED is an SSD1306
#if defined(OLED_IS_SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SH1106_128X64_NONAME_F_HW_I2C  oled(U8G2_R0, U8X8_PIN_NONE);
#endif

const char *AP_SSID = "BikeSpeedo";
const char *AP_PASS = "bikespeedo";       // WPA2, at least 8 characters. Change it if you like.

const float    MIN_KMH        = 3.0f;     // slower than this reads as stopped
const float    LOG_START_KMH  = 5.0f;     // a ride log starts at this speed
const float    SAVE_EVERY_M   = 2000.0f;  // odometer save interval while moving
const uint32_t LOG_FLUSH_MS   = 30000;
const uint32_t LONG_PRESS_MS  = 1500;
const uint32_t WIFI_IDLE_MS   = 600000;   // Wi-Fi turns itself off after 10 min unused
const size_t   FS_RESERVE     = 128 * 1024;  // oldest rides are deleted to keep this much free

// ---------------------------------------------------------------- settings (saved in NVS)
struct Settings {
  int   wheelMm  = 1900;    // rolling circumference with the rider on
  int   magnets  = 1;
  int   maxKmh   = 300;     // faster pulses than this are rejected as noise
  bool  mph      = false;
  float vRatio   = 7.667f;  // divider ratio, 100k over 15k
  float lowV     = 11.5f;   // low-supply warning, 0 = off
  int   sleepMin = 10;      // deep sleep after this long with no activity, 0 = never
} cfg;

Preferences prefs;
WebServer   server(80);

// ---------------------------------------------------------------- pulse capture
const uint8_t PERIODS = 5;
portMUX_TYPE  pulseMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCount = 0, isrLastUs = 0, isrMinUs = 0;
volatile uint32_t isrPeriod[PERIODS];
volatile uint8_t  isrIdx = 0, isrValid = 0;
volatile bool     isrHave = false;

void IRAM_ATTR recordPulse(uint32_t now) {
  if (isrHave) {
    uint32_t dt = now - isrLastUs;
    if (dt < isrMinUs) return;       // faster than maxKmh: noise
    isrPeriod[isrIdx] = dt;
    isrIdx = (isrIdx + 1) % PERIODS;
    if (isrValid < PERIODS) isrValid++;
  }
  isrLastUs = now;
  isrHave = true;
  isrCount++;
}

void IRAM_ATTR hallISR() {
  portENTER_CRITICAL_ISR(&pulseMux);
  recordPulse(micros());
  portEXIT_CRITICAL_ISR(&pulseMux);
}

// ---------------------------------------------------------------- state
float    pulseM = 1.9f;       // metres per pulse
float    speedKmh = 0;
double   odoM = 0;
float    tripM = 0, tripMaxKmh = 0;
uint32_t tripSec = 0;
float    movingMsAcc = 0;
float    vin = 0;
uint32_t lastCount = 0;
uint32_t lastActivityMs = 0, stoppedAtMs = 0;
float    unsavedM = 0;
bool     statsDirty = false;

uint8_t  page = 0;
const uint8_t PAGES = 4;
bool     wifiOn = false;
uint32_t wifiUsedMs = 0;
float    simKmh = 0;
uint32_t simNextUs = 0;

// ride log
uint32_t rideNo = 0;
bool     logActive = false, logWasMoving = false;
char     logPath[24];
char     logBuf[2048];
size_t   logLen = 0;
uint32_t logStartMs = 0, logFlushMs = 0;

// ---------------------------------------------------------------- settings helpers
void applyDerived() {
  pulseM = (cfg.wheelMm / 1000.0f) / cfg.magnets;
  float minSec = pulseM / (cfg.maxKmh / 3.6f);
  portENTER_CRITICAL(&pulseMux);
  isrMinUs = (uint32_t)(minSec * 1e6f);
  isrHave = false;
  isrValid = 0;
  isrIdx = 0;
  portEXIT_CRITICAL(&pulseMux);
}

void loadAll() {
  prefs.begin("bspd", false);
  cfg.wheelMm  = prefs.getInt("wheel", cfg.wheelMm);
  cfg.magnets  = prefs.getInt("mags", cfg.magnets);
  cfg.maxKmh   = prefs.getInt("maxk", cfg.maxKmh);
  cfg.mph      = prefs.getBool("mph", cfg.mph);
  cfg.vRatio   = prefs.getFloat("vr", cfg.vRatio);
  cfg.lowV     = prefs.getFloat("lowv", cfg.lowV);
  cfg.sleepMin = prefs.getInt("slp", cfg.sleepMin);
  odoM         = prefs.getDouble("odo", 0);
  tripM        = prefs.getFloat("trip", 0);
  tripSec      = prefs.getUInt("tsec", 0);
  tripMaxKmh   = prefs.getFloat("tmax", 0);
  rideNo       = prefs.getUInt("ride", 0);
}

void saveSettings() {
  prefs.putInt("wheel", cfg.wheelMm);
  prefs.putInt("mags", cfg.magnets);
  prefs.putInt("maxk", cfg.maxKmh);
  prefs.putBool("mph", cfg.mph);
  prefs.putFloat("vr", cfg.vRatio);
  prefs.putFloat("lowv", cfg.lowV);
  prefs.putInt("slp", cfg.sleepMin);
}

void saveStats() {
  prefs.putDouble("odo", odoM);
  prefs.putFloat("trip", tripM);
  prefs.putUInt("tsec", tripSec);
  prefs.putFloat("tmax", tripMaxKmh);
  unsavedM = 0;
  statsDirty = false;
}

void resetTrip() {
  tripM = 0;
  tripSec = 0;
  tripMaxKmh = 0;
  movingMsAcc = 0;
  saveStats();
}

// Shared by the serial console and the web form. Out-of-range values are refused.
bool applySetting(const String &name, const String &value) {
  float v = value.toFloat();
  if (name == "wheel" && v >= 500 && v <= 3500)      cfg.wheelMm = (int)v;
  else if (name == "magnets" && v >= 1 && v <= 60)   cfg.magnets = (int)v;
  else if (name == "maxkmh" && v >= 30 && v <= 400)  cfg.maxKmh = (int)v;
  else if (name == "mph" && (v == 0 || v == 1))      cfg.mph = v == 1;
  else if (name == "vratio" && v >= 1 && v <= 40)    cfg.vRatio = v;
  else if (name == "lowv" && v >= 0 && v <= 100)     cfg.lowV = v;
  else if (name == "sleep" && v >= 0 && v <= 240)    cfg.sleepMin = (int)v;
  else if (name == "odo" && v >= 0 && v <= 1000000) { odoM = v * 1000.0; saveStats(); return true; }
  else return false;
  saveSettings();
  applyDerived();
  return true;
}

// ---------------------------------------------------------------- speed
void updateSpeed() {
  static uint32_t lastMs = 0;
  uint32_t ms = millis();
  uint32_t dtMs = ms - lastMs;
  lastMs = ms;

  uint32_t p[PERIODS], cnt, last, now;
  uint8_t n;
  bool have;
  portENTER_CRITICAL(&pulseMux);
  cnt = isrCount;
  last = isrLastUs;
  n = isrValid;
  have = isrHave;
  for (uint8_t i = 0; i < PERIODS; i++) p[i] = isrPeriod[i];
  now = micros();
  portEXIT_CRITICAL(&pulseMux);

  uint32_t fresh = cnt - lastCount;
  lastCount = cnt;
  if (fresh) {
    float d = fresh * pulseM;
    odoM += d;
    tripM += d;
    unsavedM += d;
    statsDirty = true;
    lastActivityMs = ms;
  }

  // The ring holds the newest n periods, so its first n slots are valid while it fills and all
  // slots once full. The upper median throws out the two short periods a noise spike creates.
  float kmh = 0;
  float per = 0;
  if (n >= 2) {
    for (uint8_t i = 1; i < n; i++) {
      uint32_t x = p[i];
      int8_t j = i - 1;
      while (j >= 0 && p[j] > x) { p[j + 1] = p[j]; j--; }
      p[j + 1] = x;
    }
    per = p[n / 2];
    kmh = (pulseM / (per * 1e-6f)) * 3.6f;
  }

  if (have) {
    float elapsed = (float)(now - last);
    float stopUs = (pulseM / (MIN_KMH / 3.6f)) * 1e6f;
    if (elapsed > stopUs) {
      portENTER_CRITICAL(&pulseMux);
      isrHave = false;
      isrValid = 0;
      isrIdx = 0;
      portEXIT_CRITICAL(&pulseMux);
      kmh = 0;
    } else if (n >= 2 && elapsed > per) {
      // No pulse when one was due: the wheel is slowing, and can't be faster than this.
      kmh = min(kmh, (pulseM / (elapsed * 1e-6f)) * 3.6f);
    }
  }
  if (kmh < MIN_KMH) kmh = 0;

  if (speedKmh > 0 && kmh == 0) stoppedAtMs = ms;
  speedKmh = kmh;

  if (speedKmh > 0) {
    movingMsAcc += dtMs;
    while (movingMsAcc >= 1000) { movingMsAcc -= 1000; tripSec++; }
    if (n == PERIODS && speedKmh > tripMaxKmh) tripMaxKmh = speedKmh;
  }
}

// ---------------------------------------------------------------- supply voltage
void updateVin() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VIN);
  float v = (mv / 8.0f) * 0.001f * cfg.vRatio;
  vin = (vin == 0) ? v : vin + 0.2f * (v - vin);
}

bool vinLow() { return cfg.lowV > 0 && vin > 3.0f && vin < cfg.lowV; }

// ---------------------------------------------------------------- ride log
bool isRideName(const String &n) {
  if (n.length() != 13 || !n.startsWith("ride_") || !n.endsWith(".csv")) return false;
  for (int i = 5; i < 9; i++) if (!isDigit(n[i])) return false;
  return true;
}

void makeSpace() {
  while (LittleFS.totalBytes() - LittleFS.usedBytes() < FS_RESERVE) {
    String oldest;
    File dir = LittleFS.open("/");
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      String n = f.name();
      if (isRideName(n) && ("/" + n) != logPath && (oldest.isEmpty() || n < oldest)) oldest = n;
    }
    dir.close();
    if (oldest.isEmpty()) return;
    LittleFS.remove("/" + oldest);
  }
}

// Open-write-close on every flush, so a key-off loses at most the last 30 s.
void flushLog() {
  logFlushMs = millis();
  if (!logLen) return;
  makeSpace();
  File f = LittleFS.open(logPath, FILE_APPEND);
  if (f) {
    f.write((const uint8_t *)logBuf, logLen);
    f.close();
  }
  logLen = 0;
}

void logTick() {
  static uint32_t lastMs = 0;
  uint32_t ms = millis();
  if (ms - lastMs >= 1000) {
    lastMs = ms;
    if (!logActive && speedKmh >= LOG_START_KMH) {
      rideNo = (rideNo + 1) % 10000;
      prefs.putUInt("ride", rideNo);
      snprintf(logPath, sizeof(logPath), "/ride_%04u.csv", (unsigned)rideNo);
      LittleFS.remove(logPath);   // the number wrapped round to an old ride
      logActive = true;
      logStartMs = logFlushMs = ms;
      logLen = snprintf(logBuf, sizeof(logBuf), "t_s,kmh,trip_km,vin\n");
    }
    if (logActive) {
      bool moving = speedKmh > 0;
      if (moving || logWasMoving) {   // stops keep one 0 row, then nothing until moving again
        logLen += snprintf(logBuf + logLen, sizeof(logBuf) - logLen, "%lu,%.1f,%.3f,%.1f\n",
                           (unsigned long)((ms - logStartMs) / 1000), speedKmh, tripM * 0.001f, vin);
      }
      logWasMoving = moving;
    }
  }
  if (logActive && (ms - logFlushMs >= LOG_FLUSH_MS || logLen > sizeof(logBuf) - 64)) flushLog();
}

// ---------------------------------------------------------------- Wi-Fi
void wifiStart() {
  if (wifiOn) return;
  flushLog();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // many SuperMini boards drop Wi-Fi at full power
  server.begin();
  wifiOn = true;
  wifiUsedMs = millis();
}

void wifiStop() {
  if (!wifiOn) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiOn = false;
}

String numField(const char *label, const char *name, String value, const char *hint) {
  return "<label>" + String(label) + " <input name=" + name + " value='" + value +
         "' inputmode=decimal></label> <small>" + hint + "</small><br>";
}

void handleRoot() {
  wifiUsedMs = millis();
  flushLog();
  String h = "<!DOCTYPE html><meta name=viewport content='width=device-width,initial-scale=1'>"
             "<title>BikeSpeedo</title><style>body{font:16px sans-serif;margin:12px;max-width:560px}"
             "input{width:90px;font-size:16px}td{padding:4px 10px 4px 0}label{display:inline-block;"
             "min-width:220px;margin:4px 0}small{color:#666}</style><h2>BikeSpeedo</h2>";
  h += "<p>Odometer " + String(odoM * 0.001, 1) + " km<br>Trip " + String(tripM * 0.001f, 2) +
       " km, max " + String(tripMaxKmh, 0) + " km/h<br>Supply " + String(vin, 1) + " V</p>";
  h += "<h3>Rides</h3><table>";
  File dir = LittleFS.open("/");
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    String n = f.name();
    if (!isRideName(n)) continue;
    h += "<tr><td><a href='/file?name=" + n + "'>" + n + "</a></td><td>" + String(f.size() / 1024.0f, 1) +
         " kB</td><td><form method=post action=/del style=margin:0><input type=hidden name=name value='" +
         n + "'><button>Delete</button></form></td></tr>";
  }
  dir.close();
  h += "</table><p><small>" + String(LittleFS.usedBytes() / 1024) + " of " +
       String(LittleFS.totalBytes() / 1024) + " kB used. The oldest rides go first when full.</small></p>";
  h += "<h3>Settings</h3><form method=post action=/set>";
  h += numField("Wheel circumference, mm", "wheel", String(cfg.wheelMm), "500-3500");
  h += numField("Magnets on wheel", "magnets", String(cfg.magnets), "1-60");
  h += numField("Top speed limit, km/h", "maxkmh", String(cfg.maxKmh), "faster pulses count as noise");
  h += numField("Show mph", "mph", cfg.mph ? "1" : "0", "1 = mph, 0 = km/h");
  h += numField("Voltage divider ratio", "vratio", String(cfg.vRatio, 3), "(R1+R2)/R2");
  h += numField("Low supply warning, V", "lowv", String(cfg.lowV, 1), "0 = off");
  h += numField("Sleep after, minutes", "sleep", String(cfg.sleepMin), "0 = never");
  h += numField("Odometer, km", "odo", String(odoM * 0.001, 1), "to match the bike");
  h += "<button>Save</button></form><form method=post action=/resettrip style=margin-top:16px>"
       "<button>Reset trip</button></form>";
  server.send(200, "text/html", h);
}

void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleFile() {
  wifiUsedMs = millis();
  String n = server.arg("name");
  if (!isRideName(n) || !LittleFS.exists("/" + n)) { server.send(404, "text/plain", "No such ride"); return; }
  flushLog();
  File f = LittleFS.open("/" + n, "r");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + n + "\"");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleDelete() {
  wifiUsedMs = millis();
  String n = server.arg("name");
  if (isRideName(n)) {
    if (("/" + n) == logPath) { logActive = false; logLen = 0; logPath[0] = 0; }
    LittleFS.remove("/" + n);
  }
  redirectHome();
}

void handleSet() {
  wifiUsedMs = millis();
  const char *names[] = {"wheel", "magnets", "maxkmh", "mph", "vratio", "lowv", "sleep"};
  for (const char *n : names) if (server.hasArg(n)) applySetting(n, server.arg(n));
  // Only touch the odometer if it was edited: the form's rounded value would otherwise chip at it.
  if (server.hasArg("odo") && server.arg("odo") != String(odoM * 0.001, 1)) applySetting("odo", server.arg("odo"));
  redirectHome();
}

void handleResetTrip() {
  wifiUsedMs = millis();
  resetTrip();
  redirectHome();
}

// ---------------------------------------------------------------- display
float toUnit(float km) { return cfg.mph ? km * 0.621371f : km; }
const char *distUnit() { return cfg.mph ? "mi" : "km"; }

void drawRow(int y, const char *label, const char *value) {
  oled.setFont(u8g2_font_7x13_tr);
  oled.drawStr(0, y, label);
  oled.drawStr(127 - oled.getStrWidth(value), y, value);
}

void drawTitle(const char *title) {
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 8, title);
  oled.drawHLine(0, 11, 128);
}

void drawSpeedPage() {
  char s[24];
  oled.setFont(u8g2_font_6x10_tr);
  snprintf(s, sizeof(s), "%.1f %s", toUnit(tripM * 0.001f), distUnit());
  oled.drawStr(0, 8, s);
  bool blinkOff = (millis() / 500) % 2;
  if (vin > 3.0f && !(vinLow() && blinkOff)) {
    snprintf(s, sizeof(s), "%s%.1fV", vinLow() ? "LOW " : "", vin);
    oled.drawStr(127 - oled.getStrWidth(s), 8, s);
  }
  snprintf(s, sizeof(s), "%d", (int)(toUnit(speedKmh) + 0.5f));
  oled.setFont(u8g2_font_logisoso42_tn);
  oled.drawStr(98 - oled.getStrWidth(s), 60, s);
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(101, 60, cfg.mph ? "mph" : "km/h");
}

void drawTripPage() {
  char s[24];
  drawTitle("TRIP  (hold = reset)");
  snprintf(s, sizeof(s), "%.2f %s", toUnit(tripM * 0.001f), distUnit());
  drawRow(25, "Distance", s);
  snprintf(s, sizeof(s), "%lu:%02lu", (unsigned long)(tripSec / 3600), (unsigned long)((tripSec % 3600) / 60));
  drawRow(38, "Riding time", s);
  float avg = tripSec ? (tripM / tripSec) * 3.6f : 0;
  snprintf(s, sizeof(s), "%.0f", toUnit(avg));
  drawRow(51, "Average", s);
  snprintf(s, sizeof(s), "%.0f", toUnit(tripMaxKmh));
  drawRow(64, "Max", s);
}

void drawTotalPage() {
  char s[24];
  drawTitle("TOTAL");
  snprintf(s, sizeof(s), "%.1f %s", toUnit(odoM * 0.001), distUnit());
  drawRow(25, "Odometer", s);
  snprintf(s, sizeof(s), "%.1f V", vin);
  drawRow(38, "Supply", s);
  snprintf(s, sizeof(s), "%d mm", cfg.wheelMm);
  drawRow(51, "Wheel", s);
  snprintf(s, sizeof(s), "%u", (unsigned)rideNo);
  drawRow(64, "Rides", s);
}

void drawWifiPage() {
  drawTitle("WI-FI  (hold = on/off)");
  if (wifiOn) {
    drawRow(25, "Net", AP_SSID);
    drawRow(38, "Pass", AP_PASS);
    drawRow(51, "Open", WiFi.softAPIP().toString().c_str());
    char s[12];
    snprintf(s, sizeof(s), "%u", (unsigned)WiFi.softAPgetStationNum());
    drawRow(64, "Clients", s);
  } else {
    drawRow(25, "Off", "");
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 42, "Stop the bike, then");
    oled.drawStr(0, 54, "hold to get ride logs");
  }
}

void drawScreen() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  oled.clearBuffer();
  switch (page) {
    case 0: drawSpeedPage(); break;
    case 1: drawTripPage(); break;
    case 2: drawTotalPage(); break;
    default: drawWifiPage(); break;
  }
  oled.sendBuffer();
}

// ---------------------------------------------------------------- button
void buttonTick() {
  static bool down = false, longDone = false, armed = false;
  static uint32_t downMs = 0, pollMs = 0;
  uint32_t ms = millis();
  if (ms - pollMs < 10) return;
  pollMs = ms;
  bool pressed = digitalRead(PIN_BTN) == LOW;
  if (!armed) {               // the press that woke the board is not a command
    if (!pressed) armed = true;
    return;
  }
  if (pressed && !down) {
    down = true;
    longDone = false;
    downMs = ms;
    lastActivityMs = ms;
  } else if (pressed && !longDone && ms - downMs >= LONG_PRESS_MS) {
    longDone = true;
    lastActivityMs = ms;
    if (speedKmh == 0) {
      if (page == 1) resetTrip();
      else if (page == 3) { if (wifiOn) wifiStop(); else wifiStart(); }
    }
  } else if (!pressed && down) {
    down = false;
    if (!longDone && ms - downMs >= 30) page = (page + 1) % PAGES;
  }
}

// ---------------------------------------------------------------- serial console
void printSettings() {
  Serial.printf("wheel=%d (mm)\nmagnets=%d\nmaxkmh=%d\nmph=%d\nvratio=%.3f\nlowv=%.1f\nsleep=%d (min)\nodo=%.1f (km)\n",
                cfg.wheelMm, cfg.magnets, cfg.maxKmh, cfg.mph, cfg.vRatio, cfg.lowV, cfg.sleepMin, odoM * 0.001);
  Serial.printf("speed=%.1f km/h trip=%.2f km vin=%.1f V sim=%.0f\n", speedKmh, tripM * 0.001f, vin, simKmh);
}

void serialTick() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (line.length() < 40) line += c; continue; }
    line.trim();
    if (line.length()) {
      int eq = line.indexOf('=');
      if (line == "?" || line == "help") printSettings();
      else if (line == "wifi") { if (wifiOn) wifiStop(); else wifiStart(); Serial.println(wifiOn ? "wifi on" : "wifi off"); }
      else if (line == "reset") { resetTrip(); Serial.println("trip reset"); }
      else if (line.startsWith("sim=")) {
        simKmh = line.substring(4).toFloat();
        simNextUs = micros();
        Serial.printf("sim=%.0f km/h\n", simKmh);
      } else if (eq > 0 && applySetting(line.substring(0, eq), line.substring(eq + 1))) printSettings();
      else Serial.println("? to list settings");
    }
    line = "";
  }
}

// Fake pulses for bench tests. The timestamps are computed, so loop jitter doesn't reach the speed.
void simTick() {
  if (simKmh <= 0) return;
  uint32_t periodUs = (uint32_t)((pulseM / (simKmh / 3.6f)) * 1e6f);
  while ((int32_t)(micros() - simNextUs) >= 0) {
    portENTER_CRITICAL(&pulseMux);
    recordPulse(simNextUs);
    portEXIT_CRITICAL(&pulseMux);
    simNextUs += periodUs;
  }
}

// ---------------------------------------------------------------- sleep
void goToSleep() {
  flushLog();
  saveStats();
  wifiStop();
  oled.setPowerSave(1);
  // A wheel can stop with the magnet in front of the sensor, so wake on the level it isn't at now.
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_BTN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_HALL,
                                    digitalRead(PIN_HALL) ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_HALL, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_VIN, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  oled.begin();

  loadAll();
  if (!LittleFS.begin(true)) Serial.println("LittleFS failed: ride logs off");
  applyDerived();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/file", HTTP_GET, handleFile);
  server.on("/del", HTTP_POST, handleDelete);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/resettrip", HTTP_POST, handleResetTrip);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);
  lastActivityMs = millis();
  WiFi.mode(WIFI_OFF);
}

void loop() {
  uint32_t ms = millis();
  simTick();
  updateSpeed();
  updateVin();
  logTick();
  buttonTick();
  serialTick();
  drawScreen();

  if (speedKmh == 0) {
    if (ms - stoppedAtMs > 3000) {
      if (statsDirty) saveStats();
      if (logLen) flushLog();
    }
  } else {
    if (unsavedM >= SAVE_EVERY_M) saveStats();
    if (wifiOn) wifiStop();     // Wi-Fi is for when the bike is parked
  }

  if (wifiOn) {
    server.handleClient();
    if (WiFi.softAPgetStationNum() > 0) wifiUsedMs = ms;   // stays on while a phone is joined
    else if (ms - wifiUsedMs > WIFI_IDLE_MS) wifiStop();
  }

  if (cfg.sleepMin > 0 && !wifiOn && simKmh <= 0 && speedKmh == 0 &&
      ms - lastActivityMs > (uint32_t)cfg.sleepMin * 60000UL) {
    goToSleep();
  }
}
