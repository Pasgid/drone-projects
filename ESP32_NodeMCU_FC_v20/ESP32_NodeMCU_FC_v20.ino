/*
 * ESP32 (NodeMCU-32S / ESP32 DevKit, ESP-WROOM-32) flight controller (PPM) with a TRANSMITTER-DRIVEN SETUP MENU
 * MPU6050 (IMU) + PPM receiver + 1.3" OLED (SH1106 / SSD1306, 128x64, I2C)
 *
 * Vehicle type (QUAD / PLANE) and every tuning value are RUNTIME settings. Change them
 * with the transmitter sticks while DISARMED, then SAVE to flash.
 *
 * ---------------- HOW TO USE THE SETUP MENU (disarmed only) ----------------
 *   Open it : press the PAGE button until the SETUP page shows, OR on the sticks hold
 *             throttle LOW + yaw LEFT + pitch FORWARD for 1 second.
 *   Pitch stick  forward / back : move the cursor up / down
 *   Roll stick   right / left   : increase / decrease the value (hold to repeat, faster the longer)
 *   Throttle stick HIGH (>1700) : changes become x10 (coarse)
 *   Yaw stick    right          : enter a submenu / run an action
 *   Yaw stick    left           : back (from the main menu = leave setup)
 *   Changes act immediately; "*" in the title = not saved. Choose SAVE to keep them.
 *   (Leave the sticks centred for a moment after opening the menu; input starts then.)
 *
 *   Menu: VEHICLE, RC RATES, SELF-LEVEL, LEVEL TRIM, GYRO PID, DIRECTIONS, TX CHECK, OUTPUT TEST,
 *         SAVE, DEFAULTS (press twice), EXIT
 *   TX CHECK    : live value + min + max of CH1..CH6. Move every stick and switch to both
 *                 ends; each channel shows OK once it reached <=1100 and >=1900.
 *   OUTPUT TEST : PROPS OFF! Pitch picks the output, hold roll right to drive it slowly
 *                 (plane servos also move with roll left). Only works with the ARM switch low.
 *
 *   ACC TAU FLT : (in the LEVEL TRIM menu) how much the attitude estimate trusts the accelerometer
 *                 WHILE FLYING. In flight the accelerometer mostly measures thrust, not gravity,
 *                 so a short time constant makes the estimate forget a real lean and the craft
 *                 drifts. Default 3 s. Raise it if it still drifts sideways; lower it if the
 *                 horizon slowly wanders away from level over a long flight.
 *   LEVEL TRIM  : CALIBRATE (quad on a LEVEL surface, hold still 1 s) measures the sensor's idea of
 *                 level. ROLL TRIM / PITCH TRIM then fine-tune it in 0.1 deg steps. This cures
 *                 drift in SELF-STABILISE: drifts right / forward -> LOWER the trim,
 *                 drifts left / back -> RAISE it. Remember to SAVE.
 *
 * MOTOR IDLE + ACRO AIRMODE (DIRECTIONS menu):
 *   MOTOR IDLE  : the pulse the motors get as soon as you arm (stick at minimum). Raise it until all four props turn
 *                 slowly and reliably. The stick range above that is rescaled, so full throttle is still 2000.
 *   ACRO AIRMODE: in ACRO the quad keeps roll/pitch/yaw control at minimum throttle (for flips and rolls from idle).
 *                 The mixer lifts all four motors together instead of clipping the low one. The I terms stay at
 *                 zero below 10% throttle. CAUTION: while armed in ACRO, a roll/pitch stick move at idle will
 *                 tip the craft over, so keep the sticks centred on the ground. SELF-STABILISE is unchanged
 *                 (no stabilisation below 10% throttle).
 *
 * BATTERY VOLTAGE (BATTERY menu): wire  pack+ --100k-- IO34 --10k-- GND  with a 100 nF capacitor from IO34 to GND.
 *   That is ratio 11 (the default) and is good for up to about 25 V (6S is too much: use 200k/10k and set ratio 21).
 *   CHECK THE DIVIDER WITH A MULTIMETER BEFORE CONNECTING IT TO THE BOARD: the middle point must read pack/11
 *   (1.5 V for a 4S pack) and never more than 3.3 V. Never connect a pack straight to an IO pin.
 *   VOLT RATIO  : (R1+R2)/R2. The BATTERY menu title shows the live voltage: nudge the ratio until it matches a multimeter.
 *   CELLS       : 0 = detect from the pack voltage at power-up (connect a charged pack before powering the FC).
 *   WARN V/CELL : "LOW BATTERY" flashes on the STATUS page when the cell voltage stays below this for 2 s.
 *   The ADC is not very accurate (a few %), so calibrate against a multimeter. No battery = shows NO BATTERY.
 *
 * BUZZER (BATTERY menu: BUZZER on/off, TEST BUZZER): active buzzer on IO26 through an NPN transistor
 *   (IO26 -> 1k -> base, emitter -> GND, buzzer between collector and +5 V). Beeps: ready, armed, disarmed,
 *   arm refused, low battery, lost signal while armed (find-me beeps), IMU fault. Patterns are in the ALERTS block.
 *
 * OLED pages (PAGE button): STATUS, RC bars, PPM RAW (raw timing), RATE CURVE, FLIGHT STATS, TREND (estimate minus accelerometer,
 *   per second), SENSORS (live accelerometer-only angles, for the bench vibration test), SETUP.
 *
 * Safety: arming is refused while the SETUP page is open, the outputs stay safe in the menu,
 * and the menu is locked while armed.
 *
 * ---------------- CONTROL STRUCTURE ----------------
 *   ACRO       stick --[RC RATES]---------------------------> rate setpoint (deg/s) --+
 *   SELF STAB  stick --> target angle --[SELF-LEVEL PID]----> rate setpoint (deg/s) --+
 *   (yaw is always  stick --[RC RATES]--> rate setpoint)                              |
 *   MPU6050 gyro rate (deg/s) -------------------> [GYRO PID] <---- error = setpoint - gyro
 *                                                      |
 *                                    motor / servo mixer --> pulses (us)
 *
 * ---------------- WIRING (ESP32 NodeMCU-32S / DevKit; labels are the IO numbers) ----------------
 *   MPU6050 SDA + OLED SDA -> IO21,  SCL -> IO22   (3.3 V, shared bus)
 *   PPM signal             -> IO32   (3.3 V logic only! 5 V receivers need a divider: 10k series, 20k to GND)
 *   Output 1..4            -> IO18, IO19, IO23, IO25
 *       QUAD, BETAFLIGHT motor order (top view, front up), props in:
 *                                              M4 FL (CW)  M2 FR (CCW)
 *                                              M3 RL (CCW) M1 RR (CW)
 *       PLANE: 1 = aileron, 2 = elevator, 3 = throttle (ESC), 4 = rudder
 *   Page button            -> IO13 and GND (push button, normally open; the internal pull-up is used, no resistor needed)
 *   Buzzer (active)        -> IO26 -> 1k -> NPN base; emitter GND; buzzer between collector and +5 V
 *   Battery voltage        -> pack+ --100k-- IO34 --10k-- GND, 100 nF from IO34 to GND  (IO34 = ADC1, input only)
 *   Power                  -> USB, OR 5 V from an ESC/BEC into VIN with a common GND. Never both at once.
 *   Not used on purpose: IO6..IO11 (flash), IO12 and IO15 (boot strapping), IO2 (strapping + on-board LED).
 *
 * ---------------- RC CHANNELS ----------------
 *   CH1 roll  CH2 pitch  CH3 throttle  CH4 yaw  CH5 ARM (> 1700)  CH6 MODE
 *
 * Libraries: "U8g2" by oliver. (Preferences ships with the ESP32 core.)
 * Arduino IDE: Board "ESP32 Dev Module" (Tools -> Board -> esp32). Serial monitor 115200 over the USB-serial chip.
 *
 * !!! TEST WITH PROPS OFF. Verify directions and tune gains before flying.
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <math.h>
#include <stddef.h>

// ======================= FIXED CONFIG (compile time) =======================
#define PIN_SDA  21
#define PIN_SCL  22
#define PIN_PPM  32
#define PIN_BTN  13        // external page button to GND (IO0 = the on-board BOOT button is no longer used)
#define PIN_BUZZ 26        // active buzzer through an NPN transistor
#define USE_BUZZER  1      // set to 0 if no buzzer is fitted
#define PIN_VBAT 34        // battery voltage divider: ADC1, input only, not a strapping pin
const uint8_t OUT_PIN[4] = {18, 19, 23, 25};

//#define OLED_IS_SSD1306          // uncomment if your 1.3" OLED is an SSD1306 (most are SH1106)

#define FW_VERSION "v20 nodemcu ppmguard"   // shown on the start-up screen and on serial: confirms which build is running

// ---- PPM isolation switches. 0 = normal flight build. To find what disturbs the PPM input: set ONE of these to 1,
// ---- upload, open the PPM page and read "edges/s" (a healthy receiver gives about 450). Put it back to 0 afterwards.
#define DIAG_NO_PWM    0   // 1 = no motor PWM output at all (motors cannot run)
#define DIAG_NO_ADC    0   // 1 = no battery voltage reading (analogRead)
#define DIAG_NO_SERIAL 0   // 1 = no periodic status line on the serial port
#define PPM_ON_CORE0   1   // 1 = the PPM interrupt runs on core 0 (the flight loop runs on core 1): a busy loop cannot delay it

#define LOOP_HZ   500
#define LOOP_US   (1000000UL / LOOP_HZ)

#define GYRO_NOTCH_HZ    0.0f      // gyro notch centre in Hz, 0 = off (must be < 250)
#define GYRO_NOTCH_Q     4.0f
#define ACC_LPF_HZ       15.0f     // accelerometer low-pass (attitude correction only)
#define VIB_HPF_HZ       40.0f     // high-pass for the vibration meter
#define BIAS_TRACKING    1         // slow gyro drift removal while disarmed and still
#define ATT_TAU          0.25f     // complementary filter time constant (s)
#define LEVEL_D_LPF_HZ   20.0f     // low-pass on the self-level PID's D term
#define GYRO_I_LIMIT     150.0f    // cap of the gyro PID I term (us)
#define GYRO_OUT_LIMIT   400.0f    // cap of the gyro PID output (us)
#define OUT_TEST_SPIN_US 150       // output test: motor pulse = 1000 + this

// ======================= RUNTIME SETTINGS =======================
enum { VEH_QUAD = 0, VEH_PLANE = 1 };

struct Cfg {                       // shared by both vehicles
  uint8_t vehicle;                 // VEH_QUAD / VEH_PLANE
  uint8_t propsIn;                 // quad: 1 = props in (M1 RR CW, M2 FR CCW, M3 RL CCW, M4 FL CW), 0 = props out
  uint8_t selfOnHigh;              // 1: CH6 high = self-stabilise, low = acro; 0 reversed
  uint8_t rollStickRev, pitchStickRev;   // 1 = reverse the stick
  uint8_t rollSensRev, pitchSensRev;     // 1 = reverse the SENSED roll/pitch
  uint8_t ailRev, eleRev, rudRev;        // plane servo directions
  uint8_t acroAirmode;             // quad, ACRO only: keep roll/pitch authority at minimum throttle (uses a former padding byte)
  uint8_t alerts;                  // buzzer: 0 = off, 1 = on
  float   motorIdle;               // quad: lowest motor pulse while armed (us)
  float   rollTrim, pitchTrim;     // level trim (deg), subtracted from the accelerometer angles
  float   attTauFlight;            // accelerometer time constant while flying (s)
  float   vRatio;                  // battery divider ratio (R1+R2)/R2
  float   cells;                   // battery cell count, 0 = detect at power-up
  float   cellWarn;                // low-battery warning, volts per cell
  float   extrasVer;               // 1 once the three battery fields above hold real values (old saves read 0)
};
static_assert(sizeof(Cfg) == 44 && offsetof(Cfg, motorIdle) == 12, "Cfg layout must not change: stored settings depend on it");

struct Tune {                      // one set per vehicle
  // 1) RC rates
  float rcRollC, rcRollMax, rcRollExpo;
  float rcPitchC, rcPitchMax, rcPitchExpo;
  float rcYawC, rcYawMax, rcYawExpo;
  // 2) self-level (angle) PID
  float lvAngle, lvExpo;
  float lvRollP, lvRollI, lvRollD;
  float lvPitchP, lvPitchI, lvPitchD;
  float lvILim, lvMaxRate;
  // 3) gyro (rate) PID + its filters
  float gRollP, gRollI, gRollD;
  float gPitchP, gPitchI, gPitchD;
  float gYawP, gYawI, gYawD;
  float gyroLpf, dtermLpf;
};

// ---- setup-menu types (defined up here on purpose: the Arduino IDE inserts auto-generated
//      function prototypes above the first function, so every type used in a function
//      signature must already be declared before that point) ----
enum UiScreen : uint8_t { SCR_MENU, SCR_TXTEST, SCR_OUTTEST };
enum ItemType : uint8_t { IT_FLOAT, IT_ENUM, IT_SUB, IT_ACTION };
enum { SUB_MAIN, SUB_RC, SUB_LEVEL, SUB_GYRO, SUB_DIR, SUB_TRIM, SUB_BATT, SUB_COUNT };
enum { ACT_SAVE, ACT_DEFAULTS, ACT_TXTEST, ACT_OUTTEST, ACT_EXIT, ACT_LEVELCAL, ACT_TRIMCLEAR, ACT_ALERTTEST };

struct MenuItem {
  const char *label;
  ItemType type;
  void *ptr;                       // float* (IT_FLOAT) or uint8_t* (IT_ENUM)
  float minV, maxV, step;
  uint8_t dec;                     // decimals shown
  const char *const *names;        // IT_ENUM value names
  uint8_t arg;                     // submenu id / action id
};
struct Menu { const char *title; const MenuItem *items; uint8_t n; };

Cfg  cfg;
Tune T;                            // working copy for the CURRENT vehicle (menu edits this)
Tune tuneStore[2];                 // stored copies, index = vehicle
bool dirty = false;                // unsaved changes

void defaultCfg(Cfg &c) {
  c.vehicle = VEH_QUAD;
  c.propsIn = 1;
  c.selfOnHigh = 1;
  c.rollStickRev = c.pitchStickRev = 0;
  c.rollSensRev = c.pitchSensRev = 0;
  c.ailRev = c.eleRev = c.rudRev = 0;
  c.acroAirmode = 1;
  c.alerts = 1;
  c.motorIdle = 1100;
  c.rollTrim = c.pitchTrim = 0;
  c.attTauFlight = 3.0f;
  c.vRatio = 11.0f;                // 100k + 10k divider
  c.cells = 0;
  c.cellWarn = 3.5f;
  c.extrasVer = 2;
}

void defaultTune(Tune &t, uint8_t veh) {
  if (veh == VEH_QUAD) {
    t.rcRollC = 200;  t.rcRollMax = 400;  t.rcRollExpo = 0.10f;
    t.rcPitchC = 200; t.rcPitchMax = 400; t.rcPitchExpo = 0.10f;
    t.rcYawC = 120;   t.rcYawMax = 400;   t.rcYawExpo = 0.10f;
    t.lvAngle = 30; t.lvExpo = 0.10f;
    t.lvRollP = 1.0f;  t.lvRollI = 0;  t.lvRollD = 0;
    t.lvPitchP = 1.0f; t.lvPitchI = 0; t.lvPitchD = 0;
    t.lvILim = 30; t.lvMaxRate = 600;
    t.gRollP = 0.4f;  t.gRollI = 0.2f;  t.gRollD = 0.010f;
    t.gPitchP = 0.4f; t.gPitchI = 0.2f; t.gPitchD = 0.010f;
    t.gYawP = 0.8f;   t.gYawI = 0.2f;   t.gYawD = 0.0f;
  } else {
    t.rcRollC = 100;  t.rcRollMax = 300;  t.rcRollExpo = 0.20f;
    t.rcPitchC = 70;  t.rcPitchMax = 300; t.rcPitchExpo = 0.20f;
    t.rcYawC = 40;    t.rcYawMax = 90;    t.rcYawExpo = 0.20f;
    t.lvAngle = 30; t.lvExpo = 0.10f;
    t.lvRollP = 0.6f;  t.lvRollI = 0;  t.lvRollD = 0;
    t.lvPitchP = 0.6f; t.lvPitchI = 0; t.lvPitchD = 0;
    t.lvILim = 20; t.lvMaxRate = 120;
    t.gRollP = 0.5f;  t.gRollI = 0.3f;  t.gRollD = 0.01f;
    t.gPitchP = 0.5f; t.gPitchI = 0.3f; t.gPitchD = 0.01f;
    t.gYawP = 0.6f;   t.gYawI = 0.2f;   t.gYawD = 0.0f;
  }
  t.gyroLpf = 70;
  t.dtermLpf = 40;
}

// ---- derived runtime values (set by applyParams / vehicle change) ----
float rollStickSign = 1, pitchStickSign = 1, rollSensSign = 1, pitchSensSign = 1;
uint16_t outHz = 250;
uint16_t vehicleHz() { return (cfg.vehicle == VEH_QUAD) ? 250 : 50; }

// ---- flash storage ----
#define STORE_MAGIC    0xFC0C3A04UL
#define STORE_MAGIC_V2 0xFC0C3A03UL     // previous layout (level trim, no flight tau), still readable
#define STORE_MAGIC_V1 0xFC0C3A02UL     // oldest layout (no level trim), still readable
struct Store {
  uint32_t magic;
  Cfg  cfg;
  Tune tune[2];
  uint32_t crc;
};
struct CfgV2 {                     // layout of the previous version (kept only for migration)
  uint8_t vehicle, propsIn, selfOnHigh;
  uint8_t rollStickRev, pitchStickRev, rollSensRev, pitchSensRev;
  uint8_t ailRev, eleRev, rudRev;
  float   motorIdle;
  float   rollTrim, pitchTrim;
};
struct StoreV2 {
  uint32_t magic;
  CfgV2 cfg;
  Tune tune[2];
  uint32_t crc;
};
struct CfgV1 {                     // oldest layout (kept only for migration)
  uint8_t vehicle, propsIn, selfOnHigh;
  uint8_t rollStickRev, pitchStickRev, rollSensRev, pitchSensRev;
  uint8_t ailRev, eleRev, rudRev;
  float   motorIdle;
};
struct StoreV1 {
  uint32_t magic;
  CfgV1 cfg;
  Tune tune[2];
  uint32_t crc;
};
// the loader tells the formats apart by their size, so the sizes must all differ
static_assert(sizeof(Store) != sizeof(StoreV2) && sizeof(StoreV2) != sizeof(StoreV1) && sizeof(Store) != sizeof(StoreV1),
              "stored layouts must have different sizes");
Preferences prefs;

uint32_t fnv32(const uint8_t *d, size_t n) {
  uint32_t h = 2166136261UL;
  while (n--) { h ^= *d++; h *= 16777619UL; }
  return h;
}

void loadParams() {
  Store s;
  memset(&s, 0, sizeof s);
  bool ok = false, migrated = false;
  if (prefs.begin("fc", true)) {
    size_t n = prefs.getBytesLength("store");
    if (n == sizeof(Store)) {
      prefs.getBytes("store", &s, sizeof(Store));
      ok = (s.magic == STORE_MAGIC) && (s.crc == fnv32((const uint8_t *)&s, offsetof(Store, crc)));
    } else if (n == sizeof(StoreV2)) {                      // settings saved by the previous version
      StoreV2 o;
      memset(&o, 0, sizeof o);
      prefs.getBytes("store", &o, sizeof(StoreV2));
      if (o.magic == STORE_MAGIC_V2 && o.crc == fnv32((const uint8_t *)&o, offsetof(StoreV2, crc))) {
        defaultCfg(s.cfg);                                  // new fields start at their defaults
        s.cfg.vehicle = o.cfg.vehicle;           s.cfg.propsIn = o.cfg.propsIn;
        s.cfg.selfOnHigh = o.cfg.selfOnHigh;     s.cfg.rollStickRev = o.cfg.rollStickRev;
        s.cfg.pitchStickRev = o.cfg.pitchStickRev; s.cfg.rollSensRev = o.cfg.rollSensRev;
        s.cfg.pitchSensRev = o.cfg.pitchSensRev; s.cfg.ailRev = o.cfg.ailRev;
        s.cfg.eleRev = o.cfg.eleRev;             s.cfg.rudRev = o.cfg.rudRev;
        s.cfg.motorIdle = o.cfg.motorIdle;
        s.cfg.rollTrim = o.cfg.rollTrim;         s.cfg.pitchTrim = o.cfg.pitchTrim;
        s.tune[0] = o.tune[0];
        s.tune[1] = o.tune[1];
        ok = true;
        migrated = true;
      }
    } else if (n == sizeof(StoreV1)) {                      // settings saved by the oldest version
      StoreV1 o;
      memset(&o, 0, sizeof o);
      prefs.getBytes("store", &o, sizeof(StoreV1));
      if (o.magic == STORE_MAGIC_V1 && o.crc == fnv32((const uint8_t *)&o, offsetof(StoreV1, crc))) {
        defaultCfg(s.cfg);                                  // new fields (trims, flight tau) start at defaults
        s.cfg.vehicle = o.cfg.vehicle;           s.cfg.propsIn = o.cfg.propsIn;
        s.cfg.selfOnHigh = o.cfg.selfOnHigh;     s.cfg.rollStickRev = o.cfg.rollStickRev;
        s.cfg.pitchStickRev = o.cfg.pitchStickRev; s.cfg.rollSensRev = o.cfg.rollSensRev;
        s.cfg.pitchSensRev = o.cfg.pitchSensRev; s.cfg.ailRev = o.cfg.ailRev;
        s.cfg.eleRev = o.cfg.eleRev;             s.cfg.rudRev = o.cfg.rudRev;
        s.cfg.motorIdle = o.cfg.motorIdle;
        s.tune[0] = o.tune[0];
        s.tune[1] = o.tune[1];
        ok = true;
        migrated = true;
      }
    }
    prefs.end();
  }
  if (ok) {
    cfg = s.cfg;
    tuneStore[0] = s.tune[0];
    tuneStore[1] = s.tune[1];
    if (cfg.vehicle > 1) cfg.vehicle = VEH_QUAD;
    if (!(cfg.extrasVer >= 1.0f)) {                         // saved before battery sensing / airmode existed (also rejects NaN)
      Cfg d;
      defaultCfg(d);
      cfg.acroAirmode = d.acroAirmode;
      cfg.vRatio = d.vRatio;  cfg.cells = d.cells;  cfg.cellWarn = d.cellWarn;
    }
    if (!(cfg.extrasVer >= 2.0f)) {                         // saved before the LED / buzzer alerts existed
      Cfg d;
      defaultCfg(d);
      cfg.alerts = d.alerts;
    }
    cfg.extrasVer = 2;
    if (cfg.acroAirmode > 1) cfg.acroAirmode = 1;
    cfg.alerts = cfg.alerts ? 1 : 0;                         // older builds stored a LED/buzzer bit mask
    if (!(cfg.vRatio >= 1.0f && cfg.vRatio <= 40.0f))   cfg.vRatio = 11.0f;      // also rejects NaN
    if (!(cfg.cells >= 0.0f && cfg.cells <= 8.0f))      cfg.cells = 0;
    if (!(cfg.cellWarn >= 3.0f && cfg.cellWarn <= 4.0f)) cfg.cellWarn = 3.5f;
    if (!(cfg.attTauFlight >= 0.3f && cfg.attTauFlight <= 20.0f)) cfg.attTauFlight = 3.0f;   // also rejects NaN
    Serial.println(migrated ? "Settings loaded from flash (older format, upgraded)." : "Settings loaded from flash.");
  } else {
    defaultCfg(cfg);
    defaultTune(tuneStore[0], VEH_QUAD);
    defaultTune(tuneStore[1], VEH_PLANE);
    Serial.println("No valid settings in flash - using defaults.");
  }
  T = tuneStore[cfg.vehicle];
  outHz = vehicleHz();
  dirty = false;
}

bool saveParams() {
  Store s;
  memset(&s, 0, sizeof s);
  tuneStore[cfg.vehicle] = T;
  s.magic = STORE_MAGIC;
  s.cfg = cfg;
  s.tune[0] = tuneStore[0];
  s.tune[1] = tuneStore[1];
  s.crc = fnv32((const uint8_t *)&s, offsetof(Store, crc));
  bool ok = false;
  if (prefs.begin("fc", false)) {
    ok = (prefs.putBytes("store", &s, sizeof(Store)) == sizeof(Store));
    prefs.end();
  }
  if (ok) dirty = false;
  return ok;
}

// ======================= OLED =======================
#if defined(OLED_IS_SSD1306)
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
  U8G2_SH1106_128X64_NONAME_F_HW_I2C  u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

// ======================= LEDC / OUTPUTS =======================
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#define PWM_BITS 14

uint32_t lastDuty[4] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};   // last duty written per output: unchanged values are not rewritten

void pwmBegin(uint8_t i) {
#if DIAG_NO_PWM
  (void)i; return;
#endif
  lastDuty[i] = 0xFFFFFFFFu;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(OUT_PIN[i], outHz, PWM_BITS);
#else
  ledcSetup(i, outHz, PWM_BITS);
  ledcAttachPin(OUT_PIN[i], i);
#endif
}

void pwmReinit() {
#if DIAG_NO_PWM
  return;
#endif
  for (uint8_t i = 0; i < 4; i++) lastDuty[i] = 0xFFFFFFFFu;                          // after the output frequency changed (ESC 250 Hz <-> servo 50 Hz)
  for (uint8_t i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(OUT_PIN[i]);
    ledcAttach(OUT_PIN[i], outHz, PWM_BITS);
#else
    ledcDetachPin(OUT_PIN[i]);
    ledcSetup(i, outHz, PWM_BITS);
    ledcAttachPin(OUT_PIN[i], i);
#endif
  }
}

void writeUs(uint8_t i, int us) {
#if DIAG_NO_PWM
  (void)i; (void)us; return;
#endif
  us = constrain(us, 800, 2200);
  uint32_t duty = (uint32_t)((float)us * outHz * (1 << PWM_BITS) / 1e6f);
  if (duty == lastDuty[i]) return;          // the LEDC hardware keeps repeating the last pulse by itself
  lastDuty[i] = duty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(OUT_PIN[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

void outputsSafe() {
  if (cfg.vehicle == VEH_QUAD) {
    for (int i = 0; i < 4; i++) writeUs(i, 1000);
  } else {
    writeUs(0, 1500); writeUs(1, 1500); writeUs(2, 1000); writeUs(3, 1500);
  }
}

// ======================= BIQUAD FILTERS =======================
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float z1 = 0, z2 = 0;

  float apply(float x) {                    // direct form II transposed
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void clear() { z1 = z2 = 0; }

  void set(float nb0, float nb1, float nb2, float na0, float na1, float na2) {
    b0 = nb0 / na0; b1 = nb1 / na0; b2 = nb2 / na0;
    a1 = na1 / na0; a2 = na2 / na0;
  }
  void lpf(float fs, float fc, float q = 0.70710678f) {
    float w = 2.0f * PI * fc / fs, c = cosf(w), al = sinf(w) / (2.0f * q);
    set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
  }
  void hpf(float fs, float fc, float q = 0.70710678f) {
    float w = 2.0f * PI * fc / fs, c = cosf(w), al = sinf(w) / (2.0f * q);
    set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
  }
  void notch(float fs, float f0, float q) {
    float w = 2.0f * PI * f0 / fs, c = cosf(w), al = sinf(w) / (2.0f * q);
    set(1, -2 * c, 1, 1 + al, -2 * c, 1 - al);
  }
};

Biquad gLpf[3], gNotch[3], accLpf[3], vibHpf[3];
float vibSq[3] = {0, 0, 0};
const bool notchOn = (GYRO_NOTCH_HZ > 1.0f && GYRO_NOTCH_HZ < LOOP_HZ / 2.0f);

void initFilters() {                        // fixed filters (the gyro low-pass is set in applyParams)
  for (int k = 0; k < 3; k++) {
    if (notchOn) gNotch[k].notch(LOOP_HZ, GYRO_NOTCH_HZ, GYRO_NOTCH_Q);
    accLpf[k].lpf(LOOP_HZ, ACC_LPF_HZ);
    vibHpf[k].hpf(LOOP_HZ, VIB_HPF_HZ);
  }
}

float vibrationRms() {                      // deg/s of gyro content above VIB_HPF_HZ
  return sqrtf((vibSq[0] + vibSq[1] + vibSq[2]) / 3.0f);
}

// ======================= PPM =======================
// This block is the PPM code of esp32_nodemcu_ppm_oled_buzzer_test.ino, copied unchanged (apart from NP = 1).
#define PPM_MAX_CH  8
#define NP 1                       // the test sketch listened on 4 pins; the flight code uses only PIN_PPM
const uint8_t PPM_PINS[NP] = {PIN_PPM};

#define SYNC_US 2500   // a gap longer than this ends a frame
#define MIN_US   700   // shorter intervals are noise
#define MAX_US  2300   // longer (but below SYNC_US) marks a bad frame

volatile uint32_t pEdges[NP];       // every rising edge
volatile uint32_t pFrames[NP];      // valid frames
volatile uint32_t pLast[NP];        // time of last accepted edge
volatile uint32_t pLastFrame[NP];
volatile uint8_t  pCount[NP];
volatile bool     pDirty[NP];
volatile uint8_t  pChans[NP];
volatile uint16_t pTmp[NP][8];
volatile uint16_t pRaw[NP][8];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEdge(void *arg) {
  uint32_t i = (uint32_t)arg;
  uint32_t now = micros();
  pEdges[i]++;
  uint32_t dt = now - pLast[i];
  if (dt < MIN_US) return;
  pLast[i] = now;
  portENTER_CRITICAL_ISR(&mux);
  if (dt > SYNC_US) {
    uint8_t n = pCount[i];
    if (!pDirty[i] && n >= 4 && n <= 16) {
      for (uint8_t k = 0; k < 8; k++) pRaw[i][k] = (k < n) ? pTmp[i][k] : 0;
      pChans[i] = n;
      pFrames[i]++;
      pLastFrame[i] = now;
    }
    pCount[i] = 0;
    pDirty[i] = false;
  } else {
    if (dt > MAX_US) pDirty[i] = true;
    if (pCount[i] < 8) pTmp[i][pCount[i]] = (uint16_t)dt;
    if (pCount[i] < 32) pCount[i]++;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

uint16_t ch[PPM_MAX_CH] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500};
bool rcValid = false;
uint32_t ppmGoodSeen = 0, ppmBadSeen = 0;  // copies for the display / serial line
uint16_t ppmHz = 0;                        // accepted frames per second
uint8_t  ppmStable = 0;                    // channel count of the last frame the interrupt saw (display)
uint32_t ppmEdgeRate = 0;                  // every rising edge per second (display / diagnosis)

// Noise protection, applied here and not in the interrupt (the interrupt stays exactly the tested one):
//  - a frame is only used when its channel count equals the count of the frames before. A noise spike that
//    splits or swallows a pulse changes the count, so that frame is dropped instead of shifting every channel,
//  - a new count (receiver changed) is adopted after 3 identical frames in a row,
//  - until a count is confirmed (the first 3 frames, about 60 ms) nothing is published,
//  - ppmBad counts the dropped frames; the interrupt already refuses frames with a pulse longer than 2300 us.
uint32_t rcSeenFrames = 0, rcAcceptUs = 0;
uint8_t  rcStable = 0, rcOddN = 0, rcOdd = 0;
bool     rcEver = false;
uint32_t ppmBad = 0;
uint16_t rcGood[PPM_MAX_CH] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500};

void ppmFilterReset() {
  rcSeenFrames = 0; rcAcceptUs = 0; rcStable = 0; rcOddN = 0; rcOdd = 0; rcEver = false; ppmBad = 0;
}

void readRC() {
  static uint32_t lastRate = 0, lastGood = 0, lastEdges = 0;
  uint16_t raw[PPM_MAX_CH];
  uint8_t chans;
  uint32_t good, edges;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < PPM_MAX_CH; i++) raw[i] = pRaw[0][i];
  chans = pChans[0];
  good = pFrames[0];
  edges = pEdges[0];
  portEXIT_CRITICAL(&mux);
  ppmStable = chans;

  if (good != rcSeenFrames) {               // a new frame arrived since the last call
    rcSeenFrames = good;
    bool ok = false;
    if (chans == rcStable) { ok = true; rcOdd = 0; }
    else if (chans == rcOddN) { if (++rcOdd >= 3) { rcStable = chans; rcOdd = 0; ok = true; } }
    else { rcOddN = chans; rcOdd = 1; }
    if (ok) {
      for (int i = 0; i < PPM_MAX_CH; i++) rcGood[i] = (raw[i] == 0) ? 1500 : raw[i];   // channels the frame did not carry
      rcAcceptUs = micros();
      rcEver = true;
    } else if (rcStable != 0) {
      ppmBad++;
    }
  }
  for (int i = 0; i < PPM_MAX_CH; i++) ch[i] = rcGood[i];
  rcValid = rcEver && ((micros() - rcAcceptUs) < 100000);
  ppmGoodSeen = good;  ppmBadSeen = ppmBad;
  uint32_t ms = millis();
  if (ms - lastRate >= 1000) {
    ppmHz = (uint16_t)((good - lastGood) * 1000UL / (ms - lastRate));
    ppmEdgeRate = (uint32_t)((edges - lastEdges) * 1000UL / (ms - lastRate));
    lastGood = good;
    lastEdges = edges;
    lastRate = ms;
  }
}

float stick(uint16_t v) {                   // -1..+1, deadband rescaled
  const float DB = 0.03f;
  float x = constrain(((float)v - 1500.0f) / 500.0f, -1.0f, 1.0f);
  if (fabsf(x) < DB) return 0;
  return (x > 0 ? (x - DB) : (x + DB)) / (1.0f - DB);
}

float expoCurve(float x, float expo) {      // 0 = linear, 1 = pure cubic
  return (1.0f - expo) * x + expo * x * x * x;
}

float stickToRate(float x, float center, float maxRate, float expo) {
  float xe = expoCurve(x, expo);
  return center * xe + (maxRate - center) * xe * fabsf(xe);
}

// ======================= STATE =======================
enum Mode { MODE_ACRO, MODE_SELF };
Mode mode = MODE_SELF;
bool armed = false;
bool armSwitchSeenLow = false;
bool inFlight = false;                      // armed and the throttle has been raised (motors really working)

enum { PAGE_STATUS = 0, PAGE_RC, PAGE_PPM, PAGE_CURVE, PAGE_STATS, PAGE_TREND, PAGE_SENSORS, PAGE_SETUP, PAGE_COUNT };
uint8_t page = PAGE_STATUS;

char msgText[24] = "";
uint32_t msgUntil = 0;
void setMsg(const char *s) {
  strncpy(msgText, s, sizeof(msgText) - 1);
  msgText[sizeof(msgText) - 1] = 0;
  msgUntil = millis() + 3000;
}

// ======================= MPU6050 =======================
#define MPU_ADDR 0x68
const float GYRO_SCALE  = 32.8f;            // +-1000 dps
const float ACCEL_SCALE = 2048.0f;          // +-16 g: a big range clips less under vibration (clipping creates a false tilt)

float gyroBias[3] = {0, 0, 0};
float gyro[3] = {0, 0, 0};                  // filtered, bias removed (deg/s)
float gyroCorr[3] = {0, 0, 0};              // bias removed, unfiltered
float acc[3] = {0, 0, 1};                   // filtered (g)
float roll = 0, pitch = 0;                  // deg
float accRollDeg = 0, accPitchDeg = 0;      // angles from the accelerometer alone, trim applied (deg)
float accMag = 1.0f;                        // length of the filtered accelerometer vector (g)
bool imuOK = true;
int imuFail = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t *r) {               // r[0..2] acc, r[3..5] gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  r[0] = (Wire.read() << 8) | Wire.read();
  r[1] = (Wire.read() << 8) | Wire.read();
  r[2] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // temperature
  r[3] = (Wire.read() << 8) | Wire.read();
  r[4] = (Wire.read() << 8) | Wire.read();
  r[5] = (Wire.read() << 8) | Wire.read();
  return true;
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  uint8_t id = Wire.available() ? Wire.read() : 0;
  if (id != 0x68 && id != 0x70 && id != 0x72) {
    Serial.printf("MPU6050 not found (WHO_AM_I=0x%02X)\n", id);
    return false;
  }
  mpuWrite(0x6B, 0x01);   // wake, PLL
  delay(50);
  mpuWrite(0x19, 0x01);   // sample rate = 1 kHz / 2 = 500 Hz
  mpuWrite(0x1A, 0x02);   // hardware DLPF ~98 Hz gyro / 94 Hz accel (anti-aliasing)
  mpuWrite(0x1B, 0x10);   // +-1000 dps
  mpuWrite(0x1C, 0x18);   // +-16 g
  return true;
}

bool readIMU(float dt) {
  int16_t r[6];
  if (!mpuReadRaw(r)) {
    if (++imuFail > 50) imuOK = false;
    return false;
  }
  imuFail = 0; imuOK = true;

  float a[3] = {r[0] / ACCEL_SCALE, r[1] / ACCEL_SCALE, r[2] / ACCEL_SCALE};
  for (int k = 0; k < 3; k++) acc[k] = accLpf[k].apply(a[k]);

  for (int k = 0; k < 3; k++) {
    float g = r[3 + k] / GYRO_SCALE - gyroBias[k];
    gyroCorr[k] = g;

    float h = vibHpf[k].apply(g);                       // vibration meter (high-pass)
    vibSq[k] += 0.002f * (h * h - vibSq[k]);

    float f = notchOn ? gNotch[k].apply(g) : g;         // notch -> low-pass
    gyro[k] = gLpf[k].apply(f);
  }

  gyro[0] *= rollSensSign;                  // sensor direction fix (after the bias logic, which uses gyroCorr)
  gyro[1] *= pitchSensSign;

#if BIAS_TRACKING
  if (!armed) {                                         // slow drift removal (tau ~ 4 s)
    float am = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    bool still = fabsf(gyroCorr[0]) < 2 && fabsf(gyroCorr[1]) < 2 &&
                 fabsf(gyroCorr[2]) < 2 && fabsf(am - 1.0f) < 0.15f;
    if (still) for (int k = 0; k < 3; k++) gyroBias[k] += gyroCorr[k] * 0.0005f;
  }
#endif

  // ---- attitude: gyro integration + gated accel correction ----
  roll  += gyro[0] * dt;
  pitch += gyro[1] * dt;
  if (roll > 180) roll -= 360; else if (roll < -180) roll += 360;

  float mag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  accMag = mag;
  accRollDeg  = atan2f(rollSensSign * acc[1], acc[2]) * RAD_TO_DEG - cfg.rollTrim;
  accPitchDeg = atan2f(-pitchSensSign * acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG - cfg.pitchTrim;
  if (fabsf(mag - 1.0f) < 0.25f) {                      // ignore accel during hard maneuvers
    // In flight the accelerometer measures THRUST (along the body Z axis), not gravity, whenever the craft
    // accelerates sideways - it then reads "level" even though the craft leans. A fast correction would
    // make the estimate forget a real lean, so in flight the accelerometer only trims slow gyro drift.
    float tau = inFlight ? cfg.attTauFlight : ATT_TAU;
    float k = dt / (tau + dt);
    float dr = accRollDeg - roll;
    if (dr > 180) dr -= 360; else if (dr < -180) dr += 360;
    roll  += k * dr;
    pitch += k * (accPitchDeg - pitch);
  }
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - keep still...");
  float s[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
  int good = 0;
  int16_t r[6];
  for (int i = 0; i < 1000; i++) {
    if (mpuReadRaw(r)) {
      for (int k = 0; k < 3; k++) { s[k] += r[3 + k]; sa[k] += r[k]; }
      good++;
    }
    delay(2);
  }
  if (good) {
    for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / good / GYRO_SCALE;
    float ax = sa[0] / good / ACCEL_SCALE, ay = sa[1] / good / ACCEL_SCALE, az = sa[2] / good / ACCEL_SCALE;
    roll  = atan2f(rollSensSign * ay, az) * RAD_TO_DEG - cfg.rollTrim;
    pitch = atan2f(-pitchSensSign * ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - cfg.pitchTrim;
  }
  for (int i = 0; i < 300; i++) { readIMU(0.002f); delay(2); }   // settle the filters
  vibSq[0] = vibSq[1] = vibSq[2] = 0;
}

// ======================= BATTERY VOLTAGE =======================
// Divider on PIN_VBAT: pack+ --R1-- pin --R2-- GND, ratio = (R1+R2)/R2 (menu: BATTERY > VOLT RATIO).
// The ADC is sampled at 100 Hz and low-passed (tau 0.5 s) so motor/ESC noise and short sags do not flicker the reading.
float vBat = 0;                            // filtered pack voltage (V); ~0 when only USB powers the board
float vPinMv = 0;                          // raw pin voltage (mV), shown on the SENSORS page for calibration
bool  vInit = false;
bool  battPresent = false;
uint8_t cellsUsed = 0;                     // cell count in use (menu value, or detected)
bool  battLow = false;                     // low-battery warning (debounced)
uint32_t battLowSince = 0;

float cellVolts() { return cellsUsed ? vBat / cellsUsed : 0.0f; }

void batteryTick(uint32_t ms) {
  static uint32_t last = 0;
  if (ms - last < 10) return;
  float dt = (last == 0) ? 0.01f : (ms - last) * 1e-3f;
  last = ms;
#if DIAG_NO_ADC
  float mv = 0.0f;
#else
  float mv = (float)analogReadMilliVolts(PIN_VBAT);
#endif
  float v = mv * 0.001f * cfg.vRatio;
  if (!vInit) { vBat = v; vInit = true; }
  else vBat += (dt / (0.5f + dt)) * (v - vBat);
  vPinMv = mv;
  battPresent = (vBat > 3.0f);
  if (!battPresent) { battLow = false; battLowSince = 0; return; }

  if (cfg.cells >= 1.0f) cellsUsed = (uint8_t)cfg.cells;
  else if (cellsUsed == 0 && !armed && ms > 1500) cellsUsed = (uint8_t)constrain((int)(vBat / 4.35f) + 1, 1, 8);   // detect once, from the pack at power-up

  if (cellsUsed == 0) return;
  float c = cellVolts();
  if (c < cfg.cellWarn) {                  // must stay low for 2 s (throttle punches sag the pack briefly)
    if (battLowSince == 0) battLowSince = ms ? ms : 1;
    if (ms - battLowSince > 2000) battLow = true;
  } else if (c > cfg.cellWarn + 0.05f) {
    battLowSince = 0;
    battLow = false;
  }
}

// ---- flight state + statistics ----
// inFlight = armed and the throttle was above idle within the last 1.5 s. It selects the slow accelerometer
// time constant, and it delimits one "flight" for the statistics shown on the STATS / TREND pages.
// "d" = estimate minus accelerometer-only angle. Every sample is clamped to +-STAT_CLAMP_DEG so that a
// single spike (impact, clipping, handling the craft after landing) cannot dominate the numbers, and samples
// where the accelerometer failed the gate (its length is not ~1 g) are counted as "rejected" instead.
#define STAT_CLAMP_DEG 20.0f
#define TREND_LEN      128
struct FlightStats {
  float time, vibSum, vibPeak;
  float sumDR, sumDP;               // sums of clamped d, deg
  float sumAbsDR, sumAbsDP;         // sums of clamped |d|, deg
  uint32_t n;                       // ticks in flight
  uint32_t nValid;                  // ticks where the accelerometer passed the gate
  float vMin;                       // lowest filtered pack voltage during the flight (V), 0 = no battery
};
FlightStats fst;
bool fstValid = false, fstPrevFlying = false;
int8_t  trendR[TREND_LEN], trendP[TREND_LEN];     // per-second mean d, in 0.25 deg units
uint8_t trendLen = 0;
float   bkR = 0, bkP = 0;                         // accumulators of the second in progress
uint16_t bkN = 0;
int     bkIdx = 0;

void updateFlightState() {
  static uint32_t lastHigh = 0;
  uint32_t ms = millis();
  if (armed && ch[2] > 1100) lastHigh = ms;
  inFlight = armed && lastHigh != 0 && (uint32_t)(ms - lastHigh) < 1500;
}

void trendFlush() {                         // store the finished second
  if (bkN == 0) return;
  int i = (bkIdx < TREND_LEN) ? bkIdx : TREND_LEN - 1;
  trendR[i] = (int8_t)constrain((int)lroundf(bkR / bkN * 4.0f), -127, 127);
  trendP[i] = (int8_t)constrain((int)lroundf(bkP / bkN * 4.0f), -127, 127);
  if (i + 1 > trendLen) trendLen = (uint8_t)(i + 1);
  bkR = bkP = 0;
  bkN = 0;
}

void statsTick(float dt) {
  if (inFlight) {
    if (!fstPrevFlying) {                   // a new flight starts
      memset(&fst, 0, sizeof fst);
      memset(trendR, 0, sizeof trendR);
      memset(trendP, 0, sizeof trendP);
      trendLen = 0; bkR = bkP = 0; bkN = 0; bkIdx = 0;
      fst.vMin = battPresent ? vBat : 0.0f;
      fstValid = true;
    }
    if (battPresent && (fst.vMin == 0.0f || vBat < fst.vMin)) fst.vMin = vBat;
    float v = vibrationRms();
    fst.time += dt;
    fst.n++;
    fst.vibSum += v;
    if (v > fst.vibPeak) fst.vibPeak = v;
    if (fabsf(accMag - 1.0f) < 0.25f) {     // accelerometer usable this tick
      float dR = constrain(roll  - accRollDeg,  -STAT_CLAMP_DEG, STAT_CLAMP_DEG);
      float dP = constrain(pitch - accPitchDeg, -STAT_CLAMP_DEG, STAT_CLAMP_DEG);
      fst.nValid++;
      fst.sumDR += dR;  fst.sumDP += dP;
      fst.sumAbsDR += fabsf(dR);  fst.sumAbsDP += fabsf(dP);
      int idx = (int)fst.time;
      if (idx != bkIdx) { trendFlush(); bkIdx = idx; }
      bkR += dR;  bkP += dP;  bkN++;
    }
  } else if (fstPrevFlying && fst.n > 0) {
    trendFlush();
    float n = (float)fst.n, nv = fst.nValid ? (float)fst.nValid : 1.0f;
    Serial.printf("FLIGHT END: %.1f s | VIB avg %.1f peak %.1f | accel rejected %.0f%% | d roll mean %+.1f abs %.1f, pitch mean %+.1f abs %.1f deg | min batt %.2f V\n",
                  fst.time, fst.vibSum / n, fst.vibPeak, 100.0f * (fst.n - fst.nValid) / n,
                  fst.sumDR / nv, fst.sumAbsDR / nv, fst.sumDP / nv, fst.sumAbsDP / nv, fst.vMin);
  }
  fstPrevFlying = inFlight;
}

// ---- level calibration: quad on a LEVEL surface, measures what the sensor reads as "level" ----
// Attitude = accelerometer angle - trim, so after calibration "level" reads 0/0.
// "Still" is judged by how STEADY the readings are during the 1 s window (spread of the
// accelerometer angle and of the gyro noise), NOT by absolute values: an MPU6050's gravity
// magnitude and gyro offset are never exactly 1 g / 0 deg/s, and an absolute test would
// reject a perfectly still craft. The same window also re-measures the gyro offset.
#define LEVEL_CAL_SAMPLES       500        // 1 s at 500 Hz
#define LEVEL_CAL_ANGLE_SPREAD  1.5f       // max wander of the accelerometer angle inside the window (deg)
#define LEVEL_CAL_GYRO_SPREAD   3.0f       // max peak-to-peak gyro reading inside the window (deg/s)
#define LEVEL_CAL_TIMEOUT_MS    10000
#define TRIM_LIMIT              15.0f      // deg

int      levelCalLeft = 0;
float    lcSumR, lcSumP, lcSumG[3];
float    lcRmin, lcRmax, lcPmin, lcPmax, lcGmin[3], lcGmax[3];
uint32_t levelCalStartMs = 0, lcLastDiag = 0;
const char *lcWhy = "";

void levelCalRestart() {
  levelCalLeft = LEVEL_CAL_SAMPLES;
  lcSumR = lcSumP = 0;
  lcRmin = lcPmin = 1e9f;
  lcRmax = lcPmax = -1e9f;
  for (int k = 0; k < 3; k++) { lcSumG[k] = 0; lcGmin[k] = 1e9f; lcGmax[k] = -1e9f; }
}

void startLevelCal() {
  levelCalRestart();
  levelCalStartMs = millis();
  lcLastDiag = 0;
  lcWhy = "";
  setMsg("HOLD STILL 1S");
}

void levelCalTick() {                       // call once per loop tick, after readIMU
  if (levelCalLeft <= 0) return;
  if (armed) { levelCalLeft = 0; return; }

  float rawRoll  = atan2f(rollSensSign * acc[1], acc[2]) * RAD_TO_DEG;
  float rawPitch = atan2f(-pitchSensSign * acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG;

  lcSumR += rawRoll;  lcSumP += rawPitch;
  if (rawRoll  < lcRmin) lcRmin = rawRoll;
  if (rawRoll  > lcRmax) lcRmax = rawRoll;
  if (rawPitch < lcPmin) lcPmin = rawPitch;
  if (rawPitch > lcPmax) lcPmax = rawPitch;
  for (int k = 0; k < 3; k++) {
    lcSumG[k] += gyroCorr[k];
    if (gyroCorr[k] < lcGmin[k]) lcGmin[k] = gyroCorr[k];
    if (gyroCorr[k] > lcGmax[k]) lcGmax[k] = gyroCorr[k];
  }
  levelCalLeft--;

  bool angleOk = (lcRmax - lcRmin) <= LEVEL_CAL_ANGLE_SPREAD && (lcPmax - lcPmin) <= LEVEL_CAL_ANGLE_SPREAD;
  bool gyroOk = true;
  for (int k = 0; k < 3; k++) if ((lcGmax[k] - lcGmin[k]) > LEVEL_CAL_GYRO_SPREAD) gyroOk = false;

  if (!angleOk || !gyroOk) {                // the readings wandered: start the 1 s window again
    lcWhy = !angleOk ? "ANGLE UNSTABLE-RETRY" : "GYRO MOVING - RETRY";
    if (millis() - lcLastDiag > 1000) {     // tell the serial monitor WHY (helps tuning the limits)
      lcLastDiag = millis();
      Serial.printf("LEVELCAL restart: angle spread R %.2f P %.2f deg (max %.1f), gyro spread %.1f/%.1f/%.1f dps (max %.1f)\n",
                    lcRmax - lcRmin, lcPmax - lcPmin, LEVEL_CAL_ANGLE_SPREAD,
                    lcGmax[0] - lcGmin[0], lcGmax[1] - lcGmin[1], lcGmax[2] - lcGmin[2], LEVEL_CAL_GYRO_SPREAD);
    }
    levelCalRestart();
  } else if (levelCalLeft == 0) {           // a full steady second: apply
    cfg.rollTrim  = constrain(lcSumR / LEVEL_CAL_SAMPLES, -TRIM_LIMIT, TRIM_LIMIT);
    cfg.pitchTrim = constrain(lcSumP / LEVEL_CAL_SAMPLES, -TRIM_LIMIT, TRIM_LIMIT);
    for (int k = 0; k < 3; k++) gyroBias[k] += lcSumG[k] / LEVEL_CAL_SAMPLES;   // craft is still: refresh gyro offset too
    dirty = true;
    setMsg("LEVEL SET - SAVE IT");
    Serial.printf("LEVELCAL done: roll trim %.2f, pitch trim %.2f deg\n", cfg.rollTrim, cfg.pitchTrim);
    return;
  }

  if (millis() - levelCalStartMs > LEVEL_CAL_TIMEOUT_MS) {   // never got a steady second
    Serial.printf("LEVELCAL timeout. |a| = %.3f g, last spread: angle R %.2f P %.2f, gyro %.1f/%.1f/%.1f\n",
                  sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]),
                  lcRmax - lcRmin, lcPmax - lcPmin,
                  lcGmax[0] - lcGmin[0], lcGmax[1] - lcGmin[1], lcGmax[2] - lcGmin[2]);
    levelCalLeft = 0;
    setMsg(lcWhy[0] ? lcWhy : "TIMEOUT - RETRY");
  }
}

// ======================= PID =======================
struct PID {
  float kp, ki, kd, iLim, oLim;
  float integ = 0, prev = 0;
  bool first = true;
  Biquad dLpf;
  PID(float p, float i, float d, float il, float ol) : kp(p), ki(i), kd(d), iLim(il), oLim(ol) {}
  void init(float fs, float dFc) { dLpf.lpf(fs, dFc); }
  void reset() { integ = 0; first = true; dLpf.clear(); }
  float update(float err, float meas, float dt, bool useI = true) {
    if (useI) integ += err * ki * dt;
    else integ = 0;                                   // I term held at zero (craft on the ground)
    integ = constrain(integ, -iLim, iLim);
    float d = 0;
    if (!first) d = dLpf.apply(-(meas - prev) / dt);   // D on (filtered) measurement, then low-passed
    first = false; prev = meas;
    return constrain(kp * err + integ + kd * d, -oLim, oLim);
  }
};

// gains are filled in from the runtime settings by applyParams()
PID pidGyroRoll (0, 0, 0, GYRO_I_LIMIT, GYRO_OUT_LIMIT);     // rate error (deg/s) -> correction (us)
PID pidGyroPitch(0, 0, 0, GYRO_I_LIMIT, GYRO_OUT_LIMIT);
PID pidGyroYaw  (0, 0, 0, GYRO_I_LIMIT, GYRO_OUT_LIMIT);
PID pidLevelRoll (0, 0, 0, 30, 600);                          // angle error (deg) -> rate setpoint (deg/s)
PID pidLevelPitch(0, 0, 0, 30, 600);

void resetPIDs() {
  pidGyroRoll.reset(); pidGyroPitch.reset(); pidGyroYaw.reset();
  pidLevelRoll.reset(); pidLevelPitch.reset();
}

void sanitizeTune() {                       // keep the rate curves monotonic: centre <= max
  cfg.attTauFlight = constrain(cfg.attTauFlight, 0.3f, 20.0f);
  cfg.cells = roundf(constrain(cfg.cells, 0.0f, 8.0f));
  if (T.rcRollC  > T.rcRollMax)  T.rcRollC  = T.rcRollMax;
  if (T.rcPitchC > T.rcPitchMax) T.rcPitchC = T.rcPitchMax;
  if (T.rcYawC   > T.rcYawMax)   T.rcYawC   = T.rcYawMax;
}

void applyParams() {                        // push the runtime settings into the control code
  rollStickSign  = cfg.rollStickRev  ? -1.0f : 1.0f;
  pitchStickSign = cfg.pitchStickRev ? -1.0f : 1.0f;
  rollSensSign   = cfg.rollSensRev   ? -1.0f : 1.0f;
  pitchSensSign  = cfg.pitchSensRev  ? -1.0f : 1.0f;

  pidGyroRoll.kp = T.gRollP;   pidGyroRoll.ki = T.gRollI;   pidGyroRoll.kd = T.gRollD;
  pidGyroPitch.kp = T.gPitchP; pidGyroPitch.ki = T.gPitchI; pidGyroPitch.kd = T.gPitchD;
  pidGyroYaw.kp = T.gYawP;     pidGyroYaw.ki = T.gYawI;     pidGyroYaw.kd = T.gYawD;

  pidLevelRoll.kp = T.lvRollP;   pidLevelRoll.ki = T.lvRollI;   pidLevelRoll.kd = T.lvRollD;
  pidLevelPitch.kp = T.lvPitchP; pidLevelPitch.ki = T.lvPitchI; pidLevelPitch.kd = T.lvPitchD;
  pidLevelRoll.iLim = pidLevelPitch.iLim = T.lvILim;
  pidLevelRoll.oLim = pidLevelPitch.oLim = T.lvMaxRate;

  float gl = constrain(T.gyroLpf, 10.0f, 0.45f * LOOP_HZ);
  for (int k = 0; k < 3; k++) gLpf[k].lpf(LOOP_HZ, gl);
  float dl = constrain(T.dtermLpf, 5.0f, 0.45f * LOOP_HZ);
  pidGyroRoll.init(LOOP_HZ, dl); pidGyroPitch.init(LOOP_HZ, dl); pidGyroYaw.init(LOOP_HZ, dl);
  pidLevelRoll.init(LOOP_HZ, LEVEL_D_LPF_HZ); pidLevelPitch.init(LOOP_HZ, LEVEL_D_LPF_HZ);
}

void applyVehicleChange(uint8_t oldVeh) {   // QUAD <-> PLANE: swap tuning set, output frequency, outputs
  tuneStore[oldVeh] = T;
  T = tuneStore[cfg.vehicle];
  outHz = vehicleHz();
  pwmReinit();
  outputsSafe();
  dirty = true;
  applyParams();
  resetPIDs();
}

// ======================= SETUP MENU (stick driven, disarmed only) =======================
#define F_ITEM(label, var, mn, mx, st, dc) {label, IT_FLOAT, (void *)&(var), mn, mx, st, dc, nullptr, 0}
#define E_ITEM(label, var, nm, cnt)        {label, IT_ENUM,  (void *)&(var), 0, (float)((cnt) - 1), 1, 0, nm, 0}
#define S_ITEM(label, sub)                 {label, IT_SUB, nullptr, 0, 0, 0, 0, nullptr, sub}
#define A_ITEM(label, act)                 {label, IT_ACTION, nullptr, 0, 0, 0, 0, nullptr, act}
#define COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

const char *const NM_VEH[]  = {"QUAD", "PLANE"};
const char *const NM_NOYES[] = {"NO", "YES"};
const char *const NM_OUTIN[] = {"OUT", "IN"};
const char *const NM_CH6[]   = {"ACRO", "SELF"};
const char *const NM_OFFON[] = {"OFF", "ON"};

const MenuItem menuMain[] = {
  E_ITEM("VEHICLE", cfg.vehicle, NM_VEH, 2),
  S_ITEM("RC RATES", SUB_RC),
  S_ITEM("SELF-LEVEL", SUB_LEVEL),
  S_ITEM("LEVEL TRIM", SUB_TRIM),
  S_ITEM("GYRO PID", SUB_GYRO),
  S_ITEM("DIRECTIONS", SUB_DIR),
  S_ITEM("BATTERY", SUB_BATT),
  A_ITEM("TX CHECK", ACT_TXTEST),
  A_ITEM("OUTPUT TEST", ACT_OUTTEST),
  A_ITEM("SAVE", ACT_SAVE),
  A_ITEM("DEFAULTS", ACT_DEFAULTS),
  A_ITEM("EXIT", ACT_EXIT),
};

const MenuItem menuRc[] = {
  F_ITEM("ROLL CENTER",  T.rcRollC,    20, 800, 10, 0),
  F_ITEM("ROLL MAX",     T.rcRollMax,  50, 900, 10, 0),
  F_ITEM("ROLL EXPO",    T.rcRollExpo,  0, 1, 0.05f, 2),
  F_ITEM("PITCH CENTER", T.rcPitchC,   20, 800, 10, 0),
  F_ITEM("PITCH MAX",    T.rcPitchMax, 50, 900, 10, 0),
  F_ITEM("PITCH EXPO",   T.rcPitchExpo, 0, 1, 0.05f, 2),
  F_ITEM("YAW CENTER",   T.rcYawC,     20, 800, 10, 0),
  F_ITEM("YAW MAX",      T.rcYawMax,   50, 900, 10, 0),
  F_ITEM("YAW EXPO",     T.rcYawExpo,   0, 1, 0.05f, 2),
};

const MenuItem menuLevel[] = {
  F_ITEM("MAX ANGLE",   T.lvAngle,    5, 60, 1, 0),
  F_ITEM("STICK EXPO",  T.lvExpo,     0, 1, 0.05f, 2),
  F_ITEM("ROLL P",      T.lvRollP,    0, 10, 0.1f, 1),
  F_ITEM("ROLL I",      T.lvRollI,    0, 2, 0.01f, 2),
  F_ITEM("ROLL D",      T.lvRollD,    0, 1, 0.005f, 3),
  F_ITEM("PITCH P",     T.lvPitchP,   0, 10, 0.1f, 1),
  F_ITEM("PITCH I",     T.lvPitchI,   0, 2, 0.01f, 2),
  F_ITEM("PITCH D",     T.lvPitchD,   0, 1, 0.005f, 3),
  F_ITEM("I LIMIT",     T.lvILim,     0, 100, 5, 0),
  F_ITEM("MAX RATE",    T.lvMaxRate, 20, 900, 10, 0),
};

const MenuItem menuGyro[] = {
  F_ITEM("ROLL P",    T.gRollP,   0, 3, 0.01f, 2),
  F_ITEM("ROLL I",    T.gRollI,   0, 3, 0.01f, 2),
  F_ITEM("ROLL D",    T.gRollD,   0, 0.2f, 0.001f, 3),
  F_ITEM("PITCH P",   T.gPitchP,  0, 3, 0.01f, 2),
  F_ITEM("PITCH I",   T.gPitchI,  0, 3, 0.01f, 2),
  F_ITEM("PITCH D",   T.gPitchD,  0, 0.2f, 0.001f, 3),
  F_ITEM("YAW P",     T.gYawP,    0, 4, 0.01f, 2),
  F_ITEM("YAW I",     T.gYawI,    0, 3, 0.01f, 2),
  F_ITEM("YAW D",     T.gYawD,    0, 0.2f, 0.001f, 3),
  F_ITEM("GYRO LPF",  T.gyroLpf,  20, 200, 5, 0),
  F_ITEM("DTERM LPF", T.dtermLpf, 10, 100, 5, 0),
};

const MenuItem menuDir[] = {
  E_ITEM("PROPS",         cfg.propsIn,       NM_OUTIN, 2),
  E_ITEM("CH6 HIGH =",    cfg.selfOnHigh,    NM_CH6, 2),
  E_ITEM("ROLL STK REV",  cfg.rollStickRev,  NM_NOYES, 2),
  E_ITEM("PITCH STK REV", cfg.pitchStickRev, NM_NOYES, 2),
  E_ITEM("ROLL SNS REV",  cfg.rollSensRev,   NM_NOYES, 2),
  E_ITEM("PITCH SNS REV", cfg.pitchSensRev,  NM_NOYES, 2),
  E_ITEM("AIL REV",       cfg.ailRev,        NM_NOYES, 2),
  E_ITEM("ELE REV",       cfg.eleRev,        NM_NOYES, 2),
  E_ITEM("RUD REV",       cfg.rudRev,        NM_NOYES, 2),
  F_ITEM("MOTOR IDLE",    cfg.motorIdle,  1000, 1300, 10, 0),
  E_ITEM("ACRO AIRMODE",  cfg.acroAirmode, NM_OFFON, 2),
};

const MenuItem menuTrim[] = {
  A_ITEM("CALIBRATE", ACT_LEVELCAL),
  F_ITEM("ROLL TRIM",  cfg.rollTrim,  -15, 15, 0.1f, 1),
  F_ITEM("PITCH TRIM", cfg.pitchTrim, -15, 15, 0.1f, 1),
  A_ITEM("CLEAR TRIM", ACT_TRIMCLEAR),
  F_ITEM("ACC TAU FLT", cfg.attTauFlight, 0.3f, 20, 0.1f, 1),
};

const MenuItem menuBatt[] = {
  F_ITEM("VOLT RATIO", cfg.vRatio,   1, 40, 0.05f, 2),
  F_ITEM("CELLS 0=AUTO", cfg.cells,  0, 8, 1, 0),
  F_ITEM("WARN V/CELL", cfg.cellWarn, 3.0f, 4.0f, 0.05f, 2),
  E_ITEM("BUZZER",      cfg.alerts,  NM_OFFON, 2),
  A_ITEM("TEST BUZZER", ACT_ALERTTEST),
};

const Menu menus[SUB_COUNT] = {
  {"SETUP",      menuMain,  COUNT_OF(menuMain)},
  {"RC RATES",   menuRc,    COUNT_OF(menuRc)},
  {"SELF-LEVEL", menuLevel, COUNT_OF(menuLevel)},
  {"GYRO PID",   menuGyro,  COUNT_OF(menuGyro)},
  {"DIRECTIONS", menuDir,   COUNT_OF(menuDir)},
  {"LEVEL TRIM", menuTrim,  COUNT_OF(menuTrim)},
  {"BATTERY",    menuBatt,  COUNT_OF(menuBatt)},
};

// ---- menu state ----
UiScreen screen = SCR_MENU;
uint8_t curMenu = SUB_MAIN;
int8_t  curPos[SUB_COUNT] = {0};
bool    needCentre = true;                 // wait for centred sticks after opening the menu
uint32_t confirmDefaultsUntil = 0;

// TX check
uint16_t chMin[6], chMax[6];
// output test
uint8_t testOutIdx = 0;
int8_t  testDir = 0;
int     testUsNow = 0;

struct Repeater {                          // press detection with hysteresis + optional auto-repeat
  bool repeat;
  bool down = false;
  uint32_t tStart = 0, tNext = 0;
  explicit Repeater(bool r) : repeat(r) {}
  bool update(float x, uint32_t now) {     // x > 0 means "pushed in this direction"
    bool pressed = down ? (x > 0.3f) : (x > 0.6f);
    if (!pressed) { down = false; return false; }
    if (!down) { down = true; tStart = now; tNext = now + 400; return true; }
    if (repeat && now >= tNext) { tNext = now + ((now - tStart > 1500) ? 40 : 130); return true; }
    return false;
  }
};
Repeater rpUp(true), rpDown(true), rpLeft(true), rpRight(true), rpBack(false), rpOk(false);

void resetTxCheck() { for (int i = 0; i < 6; i++) chMin[i] = chMax[i] = ch[i]; }

void leaveSetup() {
  page = PAGE_STATUS;
  screen = SCR_MENU;
  curMenu = SUB_MAIN;
  testDir = 0;
  if (dirty) setMsg("NOT SAVED");
}

void onPageChange() {                      // called when the PAGE button changes the page
  testDir = 0;
  screen = SCR_MENU;
  curMenu = SUB_MAIN;
  needCentre = true;
}

void afterEdit() {
  dirty = true;
  sanitizeTune();
  applyParams();
}

void editFloat(const MenuItem &it, int dir, float mult) {
  float *f = (float *)it.ptr;
  float v = *f + dir * it.step * mult;
  v = constrain(v, it.minV, it.maxV);
  v = roundf(v / it.step) * it.step;       // avoid float drift
  v = constrain(v, it.minV, it.maxV);
  if (fabsf(v) < 1e-6f) v = 0;
  *f = v;
  afterEdit();
}

void editEnum(const MenuItem &it, int dir) {
  uint8_t *p = (uint8_t *)it.ptr;
  int n = (int)it.maxV + 1;
  uint8_t old = *p;
  *p = (uint8_t)((*p + dir + n) % n);
  if (it.ptr == (void *)&cfg.vehicle) applyVehicleChange(old);
  else afterEdit();
}

void doAction(uint8_t a) {
  switch (a) {
    case ACT_SAVE:
      setMsg(saveParams() ? "SAVED" : "SAVE FAILED");
      break;
    case ACT_DEFAULTS:
      if (millis() < confirmDefaultsUntil) {
        uint8_t v = cfg.vehicle;
        float keepRatio = cfg.vRatio, keepCells = cfg.cells;      // the voltage calibration belongs to the wiring, not the tune
        defaultCfg(cfg);
        cfg.vehicle = v;
        cfg.vRatio = keepRatio;  cfg.cells = keepCells;
        defaultTune(T, v);
        confirmDefaultsUntil = 0;
        afterEdit();
        setMsg("DEFAULTS LOADED");
      } else {
        confirmDefaultsUntil = millis() + 3000;
        setMsg("OK AGAIN TO RESET");
      }
      break;
    case ACT_TXTEST:
      screen = SCR_TXTEST;
      resetTxCheck();
      break;
    case ACT_OUTTEST:
      screen = SCR_OUTTEST;
      testOutIdx = 0;
      testDir = 0;
      break;
    case ACT_LEVELCAL:
      startLevelCal();
      break;
    case ACT_TRIMCLEAR:
      cfg.rollTrim = cfg.pitchTrim = 0;
      afterEdit();
      setMsg("TRIM CLEARED");
      break;
    case ACT_ALERTTEST:
      alertTest();
      setMsg("BUZZER TEST");
      break;
    case ACT_EXIT:
      leaveSetup();
      break;
  }
}

void menuTick() {                          // called every loop tick
  uint32_t now = millis();

  // open the setup page with a stick gesture: throttle low + yaw left + pitch forward for 1 s
  static uint32_t gestureStart = 0;
  bool gesture = !armed && rcValid && page != PAGE_SETUP && ch[2] < 1100 && ch[3] < 1200 && ch[1] > 1800;
  if (!gesture) gestureStart = 0;
  else if (!gestureStart) gestureStart = now;
  else if (now - gestureStart > 1000) {
    page = PAGE_SETUP;
    onPageChange();
    gestureStart = 0;
  }

  if (page != PAGE_SETUP || armed || !rcValid) { testDir = 0; return; }

  float sx = stick(ch[0]), sy = stick(ch[1]), sw = stick(ch[3]);

  if (screen == SCR_TXTEST) {              // track the extremes of every channel
    for (int i = 0; i < 6; i++) {
      if (ch[i] < chMin[i]) chMin[i] = ch[i];
      if (ch[i] > chMax[i]) chMax[i] = ch[i];
    }
  }

  if (needCentre) {                        // ignore stick input until everything is centred once
    if (fabsf(sx) < 0.3f && fabsf(sy) < 0.3f && fabsf(sw) < 0.3f) needCentre = false;
    testDir = 0;
    return;
  }

  bool up = rpUp.update(sy, now),    down = rpDown.update(-sy, now);
  bool right = rpRight.update(sx, now), left = rpLeft.update(-sx, now);
  bool back = rpBack.update(-sw, now),  ok = rpOk.update(sw, now);

  if (screen == SCR_TXTEST) {
    if (back) screen = SCR_MENU;
    return;
  }

  if (screen == SCR_OUTTEST) {
    if (up)   testOutIdx = (testOutIdx + 3) % 4;
    if (down) testOutIdx = (testOutIdx + 1) % 4;
    if (sx > 0.6f) testDir = 1;
    else if (sx < -0.6f && cfg.vehicle == VEH_PLANE) testDir = -1;
    else testDir = 0;
    if (back) { screen = SCR_MENU; testDir = 0; }
    return;
  }

  // ---- normal menu screen ----
  const Menu &m = menus[curMenu];
  int8_t &pos = curPos[curMenu];
  if (up)   pos = (pos + m.n - 1) % m.n;
  if (down) pos = (pos + 1) % m.n;
  const MenuItem &it = m.items[pos];
  float mult = (ch[2] > 1700) ? 10.0f : 1.0f;

  if (it.type == IT_FLOAT) {
    if (right) editFloat(it, +1, mult);
    if (left)  editFloat(it, -1, mult);
  } else if (it.type == IT_ENUM) {
    if (right) editEnum(it, +1);
    if (left)  editEnum(it, -1);
  } else if (it.type == IT_SUB) {
    if (ok) { curMenu = it.arg; }
  } else if (it.type == IT_ACTION) {
    if (ok) doAction(it.arg);
  }

  if (back) {
    if (curMenu == SUB_MAIN) leaveSetup();
    else curMenu = SUB_MAIN;
  }
}

// drive the selected output during OUTPUT TEST (disarmed only, ARM switch low)
bool outputTestActive() {
  return page == PAGE_SETUP && screen == SCR_OUTTEST && !armed && testDir != 0 && ch[4] < 1500;
}

void driveTestOutput() {
  outputsSafe();
  int us;
  if (cfg.vehicle == VEH_QUAD) us = (testDir > 0) ? 1000 + OUT_TEST_SPIN_US : 1000;
  else if (testOutIdx == 2)    us = (testDir > 0) ? 1000 + OUT_TEST_SPIN_US : 1000;   // plane throttle
  else                         us = 1500 + testDir * 300;                              // plane servo
  testUsNow = us;
  writeUs(testOutIdx, us);
}

// ======================= ARMING / MODE =======================
// ======================= ALERTS: BUZZER =======================
// Active buzzer, plain on/off with digitalWrite (about a microsecond per call, nothing ever waits), driven from a
// pattern table in the main loop. In order of priority:
//   fast chirping = IMU fault         long beeps = lost signal after being armed (stops when the signal returns)
//   one-shot beeps: ready (1), armed (1 long), disarmed (2), arm refused / failsafe (3)
//   2 short beeps every 2 s = LOW BATTERY
// MENU: BATTERY > BUZZER switches it off, TEST BUZZER checks the wiring.
// (An RGB LED indicator was tried and removed: the on-board WS2812 needs a blocking timed write.)
#define ALERT_SLOT_MS 100
uint32_t evBits = 0, evStart = 0;           // one-shot pattern (bit i = slot i, 100 ms each)
uint8_t  evLen = 0;
uint32_t testUntil = 0;
bool lostModel = false;                     // signal lost while armed
uint32_t lostSince = 0;

void alertEvent(uint32_t bits, uint8_t len) { evBits = bits; evLen = len; evStart = millis(); }
void alertTest() { testUntil = millis() + 1200; alertEvent(0x15, 5); }   // 3 beeps, even when the buzzer is off

bool patternOn(uint32_t bits, uint8_t len, uint32_t t) { return (bits >> ((t / ALERT_SLOT_MS) % len)) & 1; }

void alertTick(uint32_t ms) {
  static bool wasArmed = false;
  if (wasArmed && !armed && !rcValid && imuOK) { lostModel = true; lostSince = ms; }   // failsafe disarm
  if (rcValid || (lostModel && ms - lostSince > 300000UL)) lostModel = false;           // signal back (or give up after 5 min)
  wasArmed = armed;

  bool buzz = false;
  if (!imuOK)                   buzz = patternOn(0x1, 2, ms);                 // 100 ms on / 100 ms off
  else if (evLen && ms - evStart < (uint32_t)evLen * ALERT_SLOT_MS) buzz = patternOn(evBits, evLen, ms - evStart);
  else if (lostModel)           buzz = patternOn(0x7, 6, ms);                 // 300 ms on / 300 ms off
  else if (battLow)             buzz = patternOn(0x5, 20, ms);                // two short beeps every 2 s
#if USE_BUZZER
  digitalWrite(PIN_BUZZ, (buzz && (cfg.alerts || ms < testUntil)) ? HIGH : LOW);
#else
  (void)buzz;
#endif
}

void updateArming() {
  bool armSw = ch[4] > 1700;

  if (!rcValid || !imuOK) {
    if (armed) { Serial.println("DISARM: failsafe"); setMsg("FAILSAFE"); alertEvent(0x15, 5); }
    armed = false;
    armSwitchSeenLow = false;               // arm switch must be cycled after a signal loss
    return;
  }
  if (!armSw) {
    if (armed) { Serial.println("DISARMED"); setMsg("DISARMED"); alertEvent(0x5, 3); }
    armed = false;
    armSwitchSeenLow = true;
    return;
  }
  if (armed) return;

  static uint32_t lastDeny = 0;
  const char *why = nullptr;
  if (!armSwitchSeenLow)                           why = "SW HIGH AT BOOT";
  else if (page == PAGE_SETUP)                     why = "EXIT SETUP FIRST";
  else if (ch[2] > 1100)                           why = "THROTTLE NOT LOW";
  else if (fabsf(roll) > 25 || fabsf(pitch) > 25)  why = "TILT > 25 DEG";

  if (why) {
    if (millis() - lastDeny > 1000) { setMsg(why); Serial.printf("ARM DENIED: %s\n", why); lastDeny = millis(); alertEvent(0x15, 5); }
    return;
  }
  armed = true;
  resetPIDs();
  setMsg("ARMED");
  alertEvent(0x3, 2);
  Serial.println("ARMED");
}

void updateMode() {
  bool high = (ch[5] >= 1500);
  mode = (high == (cfg.selfOnHigh != 0)) ? MODE_SELF : MODE_ACRO;
}

// ======================= CONTROL =======================
// Signal flow:
//   ACRO      : stick --RC rates-------------------------> rate setpoint --+
//   SELF STAB : stick --> target angle --SELF-LEVEL PID--> rate setpoint --+--> GYRO PID --> mixer
//   YAW       : always stick --RC rates--> rate setpoint
float outRoll = 0, outPitch = 0, outYaw = 0;   // us
int   outThrottle = 1000;
int   outUs[4] = {1000, 1000, 1000, 1000};

void runControl(float dt) {
  float thrIn = ch[2];
  bool quad = (cfg.vehicle == VEH_QUAD);

  // ---- stick positions, -1..+1 ----
  float sRoll  = stick(ch[0]) * rollStickSign;       // +right down
  float sPitch = stick(ch[1]) * pitchStickSign;      // +nose down
  float sYaw   = -stick(ch[3]);                      // stick right = CW = negative gz

  // ---- rate setpoints (deg/s): what the GYRO PID must achieve ----
  float yawRateSet = stickToRate(sYaw, T.rcYawC, T.rcYawMax, T.rcYawExpo);     // RC rates
  float rollRateSet, pitchRateSet;

  static Mode lastMode = MODE_SELF;
  if (mode != lastMode) {                            // start the self-level PIDs fresh on mode entry
    pidLevelRoll.reset(); pidLevelPitch.reset();
    lastMode = mode;
  }

  if (mode == MODE_ACRO) {
    rollRateSet  = stickToRate(sRoll,  T.rcRollC,  T.rcRollMax,  T.rcRollExpo);
    pitchRateSet = stickToRate(sPitch, T.rcPitchC, T.rcPitchMax, T.rcPitchExpo);
  } else {
    float rollTarget  = expoCurve(sRoll,  T.lvExpo) * T.lvAngle;
    float pitchTarget = expoCurve(sPitch, T.lvExpo) * T.lvAngle;
    rollRateSet  = pidLevelRoll.update(rollTarget   - roll,  roll,  dt);
    pitchRateSet = pidLevelPitch.update(pitchTarget - pitch, pitch, dt);
  }

  outThrottle = (int)thrIn;

  // AIRMODE (quad, ACRO only): keep full roll/pitch/yaw authority even at minimum throttle, so flips and rolls can
  // start from idle. The mixer lifts all four motors together when one would drop below idle (see below).
  // The I terms stay at zero while the throttle is low, so a craft sitting on the ground cannot wind them up.
  bool airmode = quad && mode == MODE_ACRO && cfg.acroAirmode;
  bool lowThr  = thrIn < 1100;

  if (quad && lowThr && !airmode) {        // quad on the ground: idle, no stabilisation
    resetPIDs();
    outRoll = outPitch = outYaw = 0;
  } else {
    bool useI = !(airmode && lowThr);
    // GYRO PID: rate error (setpoint - measured gyro rate) -> correction in us
    outRoll  = pidGyroRoll.update(rollRateSet   - gyro[0], gyro[0], dt, useI);
    outPitch = pidGyroPitch.update(pitchRateSet - gyro[1], gyro[1], dt, useI);
    outYaw   = pidGyroYaw.update(yawRateSet     - gyro[2], gyro[2], dt, useI);
  }

  if (quad) {
    // roll+ = right side down, pitch+ = nose down, yaw+ = CCW (from above)
    // Betaflight motor order, props in:   M4 FL (CW)  M2 FR (CCW)
    //                                     M3 RL (CCW) M1 RR (CW)
    // Props out reverses all four spin directions. CW props get +yaw.
    const float YM = cfg.propsIn ? 1.0f : -1.0f;
    // Armed idle: stick minimum = MOTOR IDLE (the props turn slowly), stick maximum = 2000, linear in between.
    const float idle = constrain(cfg.motorIdle, 1000.0f, 1300.0f);
    float base = idle + (constrain(thrIn, 1000.0f, 2000.0f) - 1000.0f) * (2000.0f - idle) * 0.001f;
    outThrottle = (int)base;
    float m[4] = {                         // output 1..4 = Betaflight motors M1..M4
      base - outRoll + outPitch + YM * outYaw,   // M1 rear right
      base - outRoll - outPitch - YM * outYaw,   // M2 front right
      base + outRoll + outPitch - YM * outYaw,   // M3 rear left
      base + outRoll - outPitch + YM * outYaw,   // M4 front left
    };
    if (airmode) {                         // shift all motors together instead of clipping the low one
      float mn = m[0], mx = m[0];
      for (int i = 1; i < 4; i++) { if (m[i] < mn) mn = m[i]; if (m[i] > mx) mx = m[i]; }
      if (mn < idle)  { float sh = idle - mn;  for (int i = 0; i < 4; i++) m[i] += sh; }
      else if (mx > 2000.0f) { float sh = mx - 2000.0f; for (int i = 0; i < 4; i++) m[i] -= sh; }
    }
    for (int i = 0; i < 4; i++) outUs[i] = constrain((int)m[i], (int)idle, 2000);
  } else {
    outUs[0] = 1500 + (cfg.ailRev ? -1 : 1) * (int)constrain(outRoll,  -500.0f, 500.0f);
    outUs[1] = 1500 + (cfg.eleRev ? -1 : 1) * (int)constrain(outPitch, -500.0f, 500.0f);
    outUs[2] = constrain(outThrottle, 1000, 2000);
    outUs[3] = 1500 + (cfg.rudRev ? -1 : 1) * (int)constrain(outYaw,   -500.0f, 500.0f);
  }
}

// ======================= OLED UI =======================
void drawStatus() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);

  if (armed) {
    u8g2.drawBox(0, 0, 54, 12);
    u8g2.setDrawColor(0);
    u8g2.drawStr(5, 10, "ARMED");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawFrame(0, 0, 54, 12);
    u8g2.drawStr(5, 10, "DISARM");
  }
  u8g2.drawStr(62, 10, mode == MODE_ACRO ? "ACRO" : "SELF STAB");

  snprintf(b, sizeof b, "ROLL  %+6.1f", roll);   u8g2.drawStr(0, 24, b);
  snprintf(b, sizeof b, "PITCH %+6.1f", pitch);  u8g2.drawStr(0, 35, b);
  if (!battPresent) {
    u8g2.drawStr(0, 46, "NO BATTERY");
  } else {
    bool blink = battLow && ((millis() / 400) & 1);
    if (blink) { u8g2.drawBox(0, 37, 84, 11); u8g2.setDrawColor(0); }
    if (cellsUsed) snprintf(b, sizeof b, "%5.2fV %4.2f/c", vBat, cellVolts());
    else           snprintf(b, sizeof b, "%5.2fV", vBat);
    u8g2.drawStr(0, 46, b);
    u8g2.setDrawColor(1);
  }

  // mini artificial horizon
  const int cx = 104, cy = 31, r = 15;
  u8g2.drawCircle(cx, cy, r);
  float rr = roll * DEG_TO_RAD;
  int off = (int)(constrain(pitch / 45.0f, -1.0f, 1.0f) * r * 0.5f);   // nose down -> horizon rises
  int dx = (int)(cosf(rr) * (r - 4)), dy = (int)(sinf(rr) * (r - 4));
  int hy = cy - off;
  u8g2.drawLine(cx - dx, hy + dy, cx + dx, hy - dy);
  u8g2.drawHLine(cx - 6, cy, 4); u8g2.drawHLine(cx + 3, cy, 4); u8g2.drawPixel(cx, cy);

  // throttle bar
  float t = constrain((ch[2] - 1000) / 1000.0f, 0.0f, 1.0f);
  u8g2.drawFrame(0, 49, 128, 5);
  u8g2.drawBox(1, 50, (int)(126 * t), 3);

  // bottom line: warnings / messages, otherwise the vehicle type
  if (!rcValid)                u8g2.drawStr(0, 63, "NO RC SIGNAL");
  else if (!imuOK)             u8g2.drawStr(0, 63, "IMU FAULT");
  else if (battLow)            u8g2.drawStr(0, 63, "LOW BATTERY");
  else if (millis() < msgUntil) u8g2.drawStr(0, 63, msgText);
  else u8g2.drawStr(0, 63, cfg.vehicle == VEH_QUAD ? "QUAD" : "PLANE");
}

void drawRC() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  for (int i = 0; i < 6; i++) {
    int y = i * 9 + 1;
    snprintf(b, sizeof b, "%d", i + 1);
    u8g2.drawStr(0, y + 8, b);
    u8g2.drawFrame(10, y, 92, 8);
    int w = (int)(constrain((ch[i] - 1000) / 1000.0f, 0.0f, 1.0f) * 90);
    u8g2.drawBox(11, y + 1, w, 6);
    u8g2.drawVLine(56, y - 1, 10);
    snprintf(b, sizeof b, "%4u", ch[i]);
    u8g2.drawStr(104, y + 8, b);
  }
  u8g2.setFont(u8g2_font_5x7_tf);               // PPM health: channels, frames per second, dropped frames
  snprintf(b, sizeof b, "PPM %uch %uHz bad %lu", (unsigned)ppmStable, (unsigned)ppmHz, (unsigned long)ppmBadSeen);
  u8g2.drawStr(0, 63, b);
}

void drawPpm() {                            // PPM diagnosis: the same numbers the test sketch shows
  char b[40];
  u8g2.setFont(u8g2_font_5x7_tf);
  snprintf(b, sizeof b, "PPM IO%u edges/s %lu", (unsigned)PIN_PPM, (unsigned long)ppmEdgeRate);
  u8g2.drawStr(0, 7, b);
  snprintf(b, sizeof b, "%s %uch %uHz", rcValid ? "LOCK" : "NO LOCK", (unsigned)ppmStable, (unsigned)ppmHz);
  u8g2.drawStr(0, 17, b);
  snprintf(b, sizeof b, "1:%-4u 2:%-4u 3:%-4u", ch[0], ch[1], ch[2]);
  u8g2.drawStr(0, 29, b);
  snprintf(b, sizeof b, "4:%-4u 5:%-4u 6:%-4u", ch[3], ch[4], ch[5]);
  u8g2.drawStr(0, 39, b);
  snprintf(b, sizeof b, "7:%-4u 8:%-4u", ch[6], ch[7]);
  u8g2.drawStr(0, 49, b);
  u8g2.drawStr(0, 63, "good PPM: ~400 edges/s, 50Hz");
}

void drawCurve() {
  char b[20];
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "RATE CURVE (ROLL)");
  const int cx = 32, cy = 38, h = 24;
  u8g2.drawFrame(cx - h, cy - h, 2 * h + 1, 2 * h + 1);
  u8g2.drawHLine(cx - h, cy, 2 * h + 1);
  u8g2.drawVLine(cx, cy - h, 2 * h + 1);

  int px = 0, py = 0;
  for (int i = -h; i <= h; i++) {
    float x = i / (float)h;
    float rt = stickToRate(x, T.rcRollC, T.rcRollMax, T.rcRollExpo) / T.rcRollMax;
    int y = cy - (int)(rt * h);
    if (i > -h) u8g2.drawLine(px, py, cx + i, y);
    px = cx + i; py = y;
  }
  float sx = stick(ch[0]) * rollStickSign;
  float live = stickToRate(sx, T.rcRollC, T.rcRollMax, T.rcRollExpo);
  u8g2.drawDisc(cx + (int)(sx * h), cy - (int)(live / T.rcRollMax * h), 2);

  snprintf(b, sizeof b, "CTR %3.0f", T.rcRollC);   u8g2.drawStr(70, 26, b);
  snprintf(b, sizeof b, "MAX %3.0f", T.rcRollMax); u8g2.drawStr(70, 38, b);
  snprintf(b, sizeof b, "EXP %.2f",  T.rcRollExpo); u8g2.drawStr(70, 50, b);
  snprintf(b, sizeof b, "NOW %+4.0f", live);       u8g2.drawStr(70, 62, b);
}

void drawTxTest() {
  static const char NAMES[6] = {'R', 'P', 'T', 'Y', 'A', 'M'};
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  for (int i = 0; i < 6; i++) {
    int y = 9 + i * 9;
    bool ok = (chMin[i] <= 1100 && chMax[i] >= 1900);
    snprintf(b, sizeof b, "%c %4u %4u %4u %s", NAMES[i], ch[i], chMin[i], chMax[i], ok ? "OK" : "--");
    u8g2.drawStr(0, y, b);
  }
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 63, rcValid ? "MOVE ALL TO BOTH ENDS YL=BACK" : "NO RC SIGNAL     YL=BACK");
}

void drawOutTest() {
  static const char *const QN[4] = {"M1 RR", "M2 FR", "M3 RL", "M4 FL"};
  static const char *const PN[4] = {"AILERON", "ELEVATOR", "THROTTLE", "RUDDER"};
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "OUTPUT TEST  PROPS OFF!");
  snprintf(b, sizeof b, "OUT %d: %s", testOutIdx + 1, cfg.vehicle == VEH_QUAD ? QN[testOutIdx] : PN[testOutIdx]);
  u8g2.drawStr(0, 28, b);
  if (ch[4] >= 1500)           u8g2.drawStr(0, 44, "LOWER THE ARM SWITCH");
  else if (outputTestActive()) { snprintf(b, sizeof b, "PULSE %d us", testUsNow); u8g2.drawStr(0, 44, b); }
  else                         u8g2.drawStr(0, 44, "idle (hold ROLL right)");
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 63, "PITCH=select ROLL=drive YL=back");
}

void drawMenu() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);

  if (armed) {
    u8g2.drawStr(0, 10, "SETUP LOCKED");
    u8g2.drawStr(0, 30, "DISARM TO EDIT");
    return;
  }
  if (screen == SCR_TXTEST)  { drawTxTest();  return; }
  if (screen == SCR_OUTTEST) { drawOutTest(); return; }

  const Menu &m = menus[curMenu];
  int pos = curPos[curMenu];

  snprintf(b, sizeof b, "%s%s", m.title, dirty ? " *" : "");
  u8g2.drawStr(0, 10, b);
  if (curMenu == SUB_BATT) {                 // live reading, to set VOLT RATIO against a multimeter
    snprintf(b, sizeof b, "%5.2fV", vBat);
    u8g2.drawStr(86, 10, b);
  } else if (cfg.vehicle == VEH_QUAD) u8g2.drawStr(92, 10, "[QUAD]");
  else                                u8g2.drawStr(86, 10, "[PLANE]");
  u8g2.drawHLine(0, 12, 128);

  int top = pos - 1;
  if (top > (int)m.n - 4) top = (int)m.n - 4;
  if (top < 0) top = 0;

  for (int r = 0; r < 4 && top + r < m.n; r++) {
    const MenuItem &it = m.items[top + r];
    int y = 24 + r * 10;
    bool sel = (top + r == pos);
    if (sel) { u8g2.drawBox(0, y - 9, 128, 10); u8g2.setDrawColor(0); }
    u8g2.drawStr(2, y, it.label);

    char v[12];
    v[0] = 0;
    if (it.type == IT_FLOAT)      snprintf(v, sizeof v, "%.*f", (int)it.dec, *(float *)it.ptr);
    else if (it.type == IT_ENUM)  snprintf(v, sizeof v, "%s", it.names[*(uint8_t *)it.ptr]);
    else if (it.type == IT_SUB)   snprintf(v, sizeof v, ">");
    u8g2.drawStr(126 - 6 * (int)strlen(v), y, v);
    u8g2.setDrawColor(1);
  }

  u8g2.setFont(u8g2_font_5x7_tf);
  if (levelCalLeft > 0)    u8g2.drawStr(0, 63, "CALIBRATING - HOLD STILL");
  else if (millis() < msgUntil) u8g2.drawStr(0, 63, msgText);
  else if (!rcValid)       u8g2.drawStr(0, 63, "NO RC SIGNAL");
  else if (needCentre)     u8g2.drawStr(0, 63, "CENTRE THE STICKS");
  else                     u8g2.drawStr(0, 63, "UD=move LR=edit YR=ok YL=back");
}

void drawStats() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  if (!fstValid) {
    u8g2.drawStr(0, 10, "FLIGHT STATS");
    u8g2.drawStr(0, 30, "none yet: arm and");
    u8g2.drawStr(0, 41, "raise the throttle");
    return;
  }
  float n = fst.n ? (float)fst.n : 1.0f, nv = fst.nValid ? (float)fst.nValid : 1.0f;
  u8g2.drawStr(0, 9, inFlight ? "THIS FLIGHT (d=est-acc)" : "LAST FLIGHT (d=est-acc)");
  snprintf(b, sizeof b, "TIME %5.1fs TAU %.1f", fst.time, cfg.attTauFlight);
  u8g2.drawStr(0, 20, b);
  snprintf(b, sizeof b, "VIB avg%4.1f pk%5.1f", fst.vibSum / n, fst.vibPeak);
  u8g2.drawStr(0, 30, b);
  snprintf(b, sizeof b, "ACC REJECTED %3.0f%%", 100.0f * (fst.n - fst.nValid) / n);
  u8g2.drawStr(0, 40, b);
  snprintf(b, sizeof b, "ROLL  d%+5.1f |d|%4.1f", fst.sumDR / nv, fst.sumAbsDR / nv);
  u8g2.drawStr(0, 51, b);
  snprintf(b, sizeof b, "PITCH d%+5.1f |d|%4.1f", fst.sumDP / nv, fst.sumAbsDP / nv);
  u8g2.drawStr(0, 61, b);
}

// per-second mean of d = estimate - accelerometer angle over the flight: roll solid, pitch dotted
void drawTrend() {
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 7, "R=solid P=dotted +-10deg");
  const int top = 10, bot = 62, mid = 36;
  u8g2.drawFrame(0, top, 128, bot - top + 1);
  u8g2.drawHLine(1, mid, 126);
  if (!fstValid || trendLen < 2) { u8g2.drawStr(30, mid - 5, "NO DATA YET"); return; }
  int n = (trendLen < 126) ? trendLen : 126;
  int px = 0, rpy = 0, ppy = 0;
  for (int i = 0; i < n; i++) {
    int x = 1 + i;
    int ry = constrain(mid - (int)lroundf(trendR[i] * 0.25f * 2.5f), top + 1, bot - 1);
    int py = constrain(mid - (int)lroundf(trendP[i] * 0.25f * 2.5f), top + 1, bot - 1);
    if (i > 0) {
      u8g2.drawLine(px, rpy, x, ry);
      if ((i & 1) == 0) u8g2.drawLine(px, ppy, x, py);
    }
    px = x; rpy = ry; ppy = py;
  }
}

// live raw sensor view for the bench vibration test (props OFF)
void drawSensors() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 9, "SENSORS (live)");
  snprintf(b, sizeof b, "ACC R%+6.1f P%+6.1f", accRollDeg, accPitchDeg);
  u8g2.drawStr(0, 21, b);
  snprintf(b, sizeof b, "EST R%+6.1f P%+6.1f", roll, pitch);
  u8g2.drawStr(0, 32, b);
  snprintf(b, sizeof b, "|a|%5.2fg  VIB %5.1f", accMag, vibrationRms());
  u8g2.drawStr(0, 43, b);
  snprintf(b, sizeof b, "BIAS %+.1f %+.1f %+.1f", gyroBias[0], gyroBias[1], gyroBias[2]);
  u8g2.drawStr(0, 54, b);
  u8g2.setFont(u8g2_font_5x7_tf);
  snprintf(b, sizeof b, "PIN %4.0fmV BATT %5.2fV", vPinMv, vBat);
  u8g2.drawStr(0, 63, b);
}

void renderFrame() {
  u8g2.clearBuffer();
  if (page == PAGE_STATUS) drawStatus();
  else if (page == PAGE_RC) drawRC();
  else if (page == PAGE_PPM) drawPpm();
  else if (page == PAGE_CURVE) drawCurve();
  else if (page == PAGE_STATS) drawStats();
  else if (page == PAGE_TREND) drawTrend();
  else if (page == PAGE_SENSORS) drawSensors();
  else drawMenu();
}

// Send the frame buffer in tiny slices (2 tiles = 16 px wide, 8 px tall) so the
// I2C bus is never blocked long enough to disturb the 500 Hz control loop.
uint8_t sliceIdx = 0;
void oledPushSlice() {
  uint8_t row = sliceIdx >> 3;
  uint8_t col = (sliceIdx & 7) * 2;
  u8g2.updateDisplayArea(col, row, 2, 1);
  sliceIdx = (sliceIdx + 1) & 63;
}

void handleButton() {
  static bool last = true;
  static uint32_t t = 0;
  bool b = digitalRead(PIN_BTN);
  if (!b && last && millis() - t > 250) {
    page = (page + 1) % PAGE_COUNT;
    onPageChange();
    t = millis();
  }
  last = b;
}

void splash(const char *l1, const char *l2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, l1);
  u8g2.drawStr(0, 36, l2);
  u8g2.sendBuffer();
}

volatile bool ppmAttached = false;
volatile int ppmAttachCore = -1;
void ppmAttachTask(void *) {                // as in the test sketch, just run from the chosen core
  for (uint8_t i = 0; i < NP; i++) {
    pinMode(PPM_PINS[i], INPUT);
    attachInterruptArg(digitalPinToInterrupt(PPM_PINS[i]), onEdge, (void *)(uint32_t)i, RISING);
  }
  ppmAttachCore = xPortGetCoreID();
  ppmAttached = true;
#if PPM_ON_CORE0 && !CONFIG_FREERTOS_UNICORE
  vTaskDelete(NULL);
#endif
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("ESP32 FC " FW_VERSION);

#if USE_BUZZER
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);
  for (uint8_t k = 0; k < 2; k++) {         // power-on buzzer self-test, always, whatever the saved settings say
    digitalWrite(PIN_BUZZ, HIGH); delay(80);
    digitalWrite(PIN_BUZZ, LOW);  delay(80);
  }
#endif
  loadParams();                             // vehicle type + tuning from flash (or defaults)
  for (uint8_t i = 0; i < 4; i++) pwmBegin(i);
  outputsSafe();                            // ESCs see 1000 us immediately

  Wire.begin(PIN_SDA, PIN_SCL);             // before u8g2.begin() so our pins are kept
  Wire.setClock(400000);
  u8g2.begin();
  u8g2.setBusClock(400000);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_VBAT, INPUT);

  splash("ESP32 FC " FW_VERSION, "IMU init...");
  if (!mpuInit()) {
    splash("MPU6050 NOT FOUND", "check SDA21 SCL22");
    Serial.println("MPU6050 NOT FOUND: nothing else runs (no PPM, no buzzer patterns) until it answers on I2C 0x68/0x69.");
    while (true) {
      outputsSafe();
#if USE_BUZZER
      digitalWrite(PIN_BUZZ, (millis() / 100) & 1);   // fast chirping = IMU fault
#endif
      delay(20);
    }
  }

  applyParams();
  initFilters();

  splash("Calibrating gyro", "keep still...");
  calibrateGyro();

#if PPM_ON_CORE0 && !CONFIG_FREERTOS_UNICORE
  // An interrupt runs on the core that attached it. Attach from a short task pinned to core 0.
  xTaskCreatePinnedToCore(ppmAttachTask, "ppmAttach", 4096, NULL, 1, NULL, 0);
  for (uint8_t w = 0; w < 100 && !ppmAttached; w++) delay(2);
#else
  ppmAttachTask(NULL);
#endif
  Serial.printf("PPM interrupt on IO%u, attached from core %d (flight loop on core %d)\n",
                (unsigned)PIN_PPM, (int)ppmAttachCore, (int)xPortGetCoreID());
  resetPIDs();
  alertEvent(0x1, 3);                       // one beep: ready
  Serial.printf("Vehicle: %s. Open SETUP: PAGE button (IO13), or throttle low + yaw left + pitch forward for 1 s.\n",
                cfg.vehicle == VEH_QUAD ? "QUAD" : "PLANE");
}

void loop() {
  static uint32_t nextTick = micros() + LOOP_US;
  static uint32_t lastTick = micros();
  static uint32_t lastDraw = 0, lastPrint = 0;

  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += LOOP_US;
  if ((int32_t)(now - nextTick) > (int32_t)(4 * LOOP_US)) nextTick = now + LOOP_US;   // catch-up guard

  float dt = (now - lastTick) * 1e-6f;
  lastTick = now;
  dt = constrain(dt, 0.0005f, 0.01f);

  // ----- flight loop -----
  readRC();
  batteryTick(millis());
  updateFlightState();
  readIMU(dt);
  levelCalTick();
  statsTick(dt);
  updateArming();
  updateMode();
  alertTick(millis());
  menuTick();                               // stick-driven setup menu (does nothing while armed)

  if (armed) {
    runControl(dt);
    for (int i = 0; i < 4; i++) writeUs(i, outUs[i]);
  } else {
    resetPIDs();
    if (outputTestActive()) driveTestOutput();
    else outputsSafe();
  }

  // ----- UI (runs after outputs are written) -----
  handleButton();
  oledPushSlice();
  uint32_t ms = millis();
  if (ms - lastDraw >= 100) { lastDraw = ms; renderFrame(); }

#if !DIAG_NO_SERIAL
  if (ms - lastPrint >= 200) {
    lastPrint = ms;
    Serial.printf("%s %s %s R:%6.1f P:%6.1f VIB:%4.1f V:%5.2f | ",
                  cfg.vehicle == VEH_QUAD ? "QUAD " : "PLANE", armed ? "ARMED " : "DISARM",
                  mode == MODE_ACRO ? "ACRO" : "SELF", roll, pitch, vibrationRms(), vBat);
    Serial.printf("PPM %uch %uHz bad %lu | ", (unsigned)ppmStable, (unsigned)ppmHz, (unsigned long)ppmBadSeen);
    if (rcValid) for (int i = 0; i < 6; i++) Serial.printf("%4u ", ch[i]);
    else Serial.print("NO RC");
    Serial.println();
  }
#else
  (void)lastPrint;
#endif
}
