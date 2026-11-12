/*
 * SimpleFC BRUSHED - copy of SimpleFC for a 2S brushed micro quad (820 coreless, 7.4 V motors) on an ESP32-C3 SuperMini:
 *   SELF-LEVEL + barometer ALT HOLD. MPU6050 + barometer (BMP280 or GY-63 MS5611, auto-detected), no OLED, ESP-NOW link to SimpleTX.
 *   Differences from SimpleFC: motors are driven DIRECTLY with 20 kHz PWM duty (0-100 %) through one logic-level
 *   N-MOSFET per motor - no ESCs, no ESC calibration. 2S battery defaults. Settings in NVS "sfcb" (own namespace,
 *   never imports the brushless quad's tuning).
 *
 * ---------------- MOTOR DRIVER (one per motor) ----------------
 *   GPIO --100R-- gate of AO3400 / SI2302 (logic-level N-MOSFET, Vgs(th) < 1.5 V)
 *   gate --10k-- GND        (keeps the motor OFF while the C3 boots - important!)
 *   source -> GND,  drain -> motor (-),  motor (+) -> battery (+)
 *   Schottky diode (SS14 / 1N5819) across the motor: cathode to battery (+), anode to the drain (flyback)
 *   Optional 100 nF ceramic across each motor's terminals (brush noise).
 *   Power: 470-1000 uF low-ESR, 16 V or more, on the battery, close to the MOSFETs. Feed the SuperMini's 5 V pin from a
 *   2S->5 V BUCK (set to 5.0 V before connecting, 100 uF on its output). NEVER put 2S on the 5 V pin (about 6 V maximum).
 *
 * ---------------- WIRING (ESP32-C3 Zero) ----------------
 *   I2C SDA GPIO6, SCL GPIO7 (400 kHz): MPU6050 0x68, BMP280 or GY-63 (MS5611) 0x76/0x77
 *   Motors (Betaflight order, props in, top view, front up):   M4 FL (CW)   M2 FR (CCW)
 *                                                              M3 RL (CCW)  M1 RR (CW)
 *     M1 GPIO0, M2 GPIO1, M3 GPIO3, M4 GPIO5   (20 kHz PWM to the MOSFET gates; 1000 us = off, 2000 us = 100 %)
 *   Battery divider pack+ --100k-- GPIO4 --33k-- GND, 100 nF GPIO4-GND (ratio 4.03 for 2S, set vRatio)
 *   Passive buzzer (2.7 kHz tone): GPIO20 -> 100R -> buzzer -> GND. Louder: GPIO20 -> 1k -> NPN base, buzzer collector to +3V3/5V
 *   DJI O4 air unit (OSD, MSP DisplayPort 115200): FC GPIO21 -> air unit RX, FC GPIO10 <- air unit TX, GND.
 *     Power the air unit from the battery. OSD on/off and canvas size: SETUP > DJI OSD (default 53 x 20).
 *   GPIO2 is not used (shorted on the original board). GPIO10 = on-board RGB LED (not used).
 *   Power: 5 V buck/BEC (NOT a 7805 from 3S), 470-1000 uF on the battery leads, 100-470 uF at the FC 5 V input.
 *
 * ---------------- CHANNELS (from SimpleTX) ----------------
 *   THR, ROLL, PITCH, YAW, ARM (ARM button, 2000 = arm), CH6 (MODE button toggle: 1000 SELF-LEVEL, >= 1300 ALT HOLD)
 *
 * ---------------- FLYING ----------------
 *   Arm: TX button, throttle low, level (< 25 deg). After a failsafe or crash cut-off the ARM channel must go low first.
 *   ALT HOLD engages only on a CHANGE to ALT (a MODE press; arming with ALT already selected counts as a change):
 *     in the air  -> holds the current height
 *     on the ground -> motors idle until the throttle goes above the hover point, then it climbs
 *   Throttle in ALT HOLD: from HOVER POINT to mid stick (setting Stick mid), +-deadband = hold, above = climb (max 1 m/s),
 *   below = descend (max 0.7 m/s). Stick fully down on the ground for 0.8 s = LANDED (motors idle).
 *   Hover throttle is learned while hovering (SELF and ALT HOLD) and saved at disarm.
 *
 * ---------------- STICK COMMANDS (disarmed) ----------------
 *   Level calibration : quad on a LEVEL surface, throttle LOW + yaw full LEFT + pitch full BACK for 2 s.
 *   Trim drift        : throttle HIGH, yaw centred, push the right stick the way the quad should go INSTEAD of
 *                       where it drifts. 0.1 deg per 0.3 s held (short beep each), saved on release.
 *
 * ---------------- FAILSAFE ----------------
 *   No packet > 250 ms : roll/pitch/yaw centred, throttle and mode kept (ALT HOLD keeps holding).
 *   No packet > 1 s    : flying with a valid altitude -> AUTO-LAND (alt hold, max descent, disarm when landed or
 *                        after 15 s); otherwise disarm. Radio back -> control returned.
 *
 * ---------------- OLED (BOOT button = next page, disarmed): STATUS, SENSORS, STATS, TREND, SETUP ----------------
 *   STATS: last flight (time, max alt, min V, max vibration, worst RSSI/LQ, radio gaps, loop time, stop reason).
 *   TREND: last flight graph - altitude (line), alt hold target (dots), battery voltage (bottom).
 *   SETUP is opened with the BOOT button, or on the sticks: throttle LOW + yaw full LEFT + pitch full FORWARD for 1 s.
 *     pitch fwd/back = cursor up/down       roll right/left = value +/- (hold = repeat, throttle HIGH = x10 steps)
 *     yaw right = open group / run action   yaw left = back (from the main list = leave SETUP)
 *     Groups: SELF-LEVEL, RATE PID, FILTERS, ALT HOLD, TRIM / MOTORS, BATTERY (title shows live voltage);
 *     actions: LEVEL CALIBRATE, MOTOR TEST, ESC CALIBRATE (twice), SAVE, DEFAULTS (twice), EXIT.
 *     "*" = changed, not saved. Arming refused in SETUP.
 *   MOTOR TEST (PROPS OFF, TX ARM off): pitch picks M1..M4 / ALL, hold roll RIGHT = spin at 1150 us, yaw left = back.
 *   ESC CALIBRATE (PROPS OFF): choose it twice, unplug battery + USB, power up from the battery: MAX 5 s, MIN 5 s.
 *   CEILING: ALT HOLD > "Ceiling m" (0 = off), height above the arming point, in SELF-LEVEL and ALT HOLD.
 *   Defaults are the figures of the current quad (read from its flash on 2026-10-21).
 *   The OLED freezes while armed (drawing costs loop time).
 *
 * ---------------- USB SERIAL (115200, optional, only when a PC is connected) ----------------
 *   Status line 5x per second.  Commands (end with Enter):
 *     ?            list all settings          name=value   change one (e.g. gRollP=0.45)
 *     save         store settings in flash    defaults     factory settings (then 'save')
 *     D            dump the last flight (CSV) S            status line on/off
 *   Settings can only be changed while disarmed.
 *
 * ---------------- CONFIGURATION: USB SERIAL 115200 (no OLED on this build) ----------------
 *   help    command list                  ?         all settings, by group       name=value  change one, then: save
 *   cal     level calibration (flat!)     m1..m4, mall  spin a motor 2 s (PROPS OFF), stop = stop at once
 *   st      one status line               S         status line on/off           stats       last flight, stop reason
 *   D       dump the last flight (CSV)    defaults  factory settings
 * Libraries: none besides the core (U8g2 only with USE_OLED 1). Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled. Core esp32 3.x.
 * !!! FIRST TESTS WITH PROPS OFF: check the sensor directions (OLED / serial) and the motor reaction.
 */

#include <Wire.h>
#define USE_OLED 0                // no display on the micro (weight): configured over USB serial. 1 = 0.91 in SSD1306 + stick menu
#if USE_OLED
#include <U8g2lib.h>
#endif
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <math.h>

// ======================= FIXED CONFIG =======================
#define FW_VERSION "SimpleFC Brushed 1.0"
#define PIN_SDA   6
#define PIN_SCL   7
#define PIN_VBAT  4
#define PIN_BUZZ  20
#define PIN_OSD_TX 21             // DJI O4 air unit RX  (UART1 TX; also prints the ROM boot log - harmless)
#define PIN_OSD_RX 10             // DJI O4 air unit TX  (UART1 RX)
#define PIN_BTN   9               // BOOT button (not used for flying; OLED page)
const uint8_t MOTOR_PIN[4] = {0, 1, 3, 5};   // M1 RR, M2 FR, M3 RL, M4 FL (GPIO5: not a strapping pin, no LED)
// OLED: 0.91 inch SSD1306, 128 x 32 (four rows of 5x8 text; all pages below are laid out for this size)

#define LOOP_HZ   400
#define LOOP_US   (1000000UL / LOOP_HZ)
#define I2C_HZ    400000UL
#define PWM_HZ    20000           // brushed: 20 kHz (inaudible), duty 0..100 % to the MOSFET gates
#define PWM_BITS  10              // 1024 steps (LEDC: 80 MHz / 20 kHz allows up to 11 bits)

// radio (must match SimpleTX)
#define PKT_MAGIC   0xC3F1
#define TLM_MAGIC   0xC3F2         // SimpleFC telemetry (the old FlightController sends 0xC3F1 with another layout)
#define BIND_ID     42
#define WIFI_CH     13
#define RC_STALE_MS 250
#define FAILSAFE_MS 1000
#define FS_LAND_MAX_MS 15000

#define BARO_PERIOD_MS 40
#define BARO_LPF_HZ    1.5f
#define BARO_TAU       2.5f       // altitude estimate: how slowly the baro corrects the accelerometer (s)
#define BATCOMP_VREF_CELL 3.85f
#define EXIT_BLEND_MS  600
#define CRASH_ANGLE_DEG 70.0f
#define CRASH_ANGLE_MS  100
#define CRASH_VIB_ABS   40.0f
#define CRASH_VIB_RATIO 4.0f
#define CRASH_VIB_MS    250
#define ATT_TAU_GROUND  0.25f     // accelerometer time constant on the ground (s); in flight: accTauFlt
#define GYRO_I_LIMIT    150.0f
#define GYRO_OUT_LIMIT  400.0f
#define TRIM_LIMIT      10.0f

// ======================= TYPES (before the first function: Arduino auto-prototypes) =======================
enum { RX_THR, RX_ROLL, RX_PITCH, RX_YAW, RX_ARM, RX_CH6, RX_NUM_CH };

struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  seq;
  uint16_t ch[RX_NUM_CH];          // 1000..2000 us, order above
  uint8_t  crc;
};

enum ArmBlock : uint8_t { AB_NONE, AB_THROTTLE, AB_SWITCH, AB_IMU, AB_TILT, AB_NOLINK, AB_CAL, AB_SETUP };
enum Mode : uint8_t { MODE_SELF = 0, MODE_ALT = 1 };

// flags: bit0 battery present, bit1 battery LOW, bit2 altitude valid, bit3 landed (alt hold),
//        bit4 failsafe auto-land, bit5 baro missing
struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  armed;
  uint8_t  mode;                   // Mode
  uint8_t  armBlock;               // ArmBlock while disarmed
  int16_t  roll10, pitch10;        // deg x10
  uint16_t vbat100;                // pack V x100, 0 = none
  uint8_t  cells;
  uint8_t  flags;
  int16_t  alt100;                 // altitude estimate, cm
  int8_t   rssi;                   // dBm the FC hears the TX with
  uint8_t  lq;                     // % of the TX's 50 packets/s received
  uint8_t  crc;
};

// All runtime settings are floats, listed in PARAMS below (name, default, range). Stored per key in flash.
struct Params {
  float rollSensRev, pitchSensRev, propsIn, motorIdle;
  float rollTrim, pitchTrim, accTauFlt;
  float vRatio, cells, cellWarn;
  float angleMax, angleExpo, lvRollP, lvRollI, lvRollD, lvPitchP, lvPitchI, lvPitchD, lvILim, lvMaxRate;
  float yawRateC, yawRateMax, yawExpo;
  float gRollP, gRollI, gRollD, gPitchP, gPitchI, gPitchD, gYawP, gYawI;
  float gyroLpf, dtermLpf, notchHz, notchQ, accLpfHz, lvDLpfHz;
  float rcSmoothHz, itermRelax, antiGrav;
  float altP, velP, velI, maxClimb, maxDesc, deadband, minThr, hover, ceilingM, thrMid, estTau;
  float osdOn, osdCols, osdRows;
};
struct ParamDef { const char *name; const char *label; float Params::*m; float def, mn, mx, step; uint8_t dec, grp; };

struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  float apply(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
  void clear() { z1 = z2 = 0; }
  void set(float nb0, float nb1, float nb2, float na0, float na1, float na2) {
    b0 = nb0 / na0; b1 = nb1 / na0; b2 = nb2 / na0; a1 = na1 / na0; a2 = na2 / na0;
  }
  void lpf(float fs, float fc, float q = 0.70710678f) {
    float w = 2 * PI * fc / fs, c = cosf(w), al = sinf(w) / (2 * q);
    set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
  }
  void hpf(float fs, float fc, float q = 0.70710678f) {
    float w = 2 * PI * fc / fs, c = cosf(w), al = sinf(w) / (2 * q);
    set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
  }
  void notch(float fs, float f0, float q) {
    float w = 2 * PI * f0 / fs, c = cosf(w), al = sinf(w) / (2 * q);
    set(1, -2 * c, 1, 1 + al, -2 * c, 1 - al);
  }
};

struct PID {
  float kp = 0, ki = 0, kd = 0, iLim, oLim;
  float integ = 0, prev = 0;
  bool first = true;
  Biquad dLpf;
  PID(float il, float ol) : iLim(il), oLim(ol) {}
  void reset() { integ = 0; first = true; dLpf.clear(); }
  // D on measurement; useI = false holds I at 0; iScale = I-term relax x anti-gravity
  float update(float err, float meas, float dt, bool useI = true, float iScale = 1.0f) {
    if (useI) integ = constrain(integ + err * ki * dt * iScale, -iLim, iLim);
    else integ = 0;
    float d = 0;
    if (!first) d = dLpf.apply(-(meas - prev) / dt);
    first = false; prev = meas;
    return constrain(kp * err + integ + kd * d, -oLim, oLim);
  }
};

struct __attribute__((packed)) RecSample {   // flight recorder, 50 Hz
  uint16_t t10;                    // 10 ms units since arming
  int16_t  roll10, pitch10, rollT10, pitchT10;
  int16_t  gR, gP, gY;             // deg/s
  uint16_t thr, rcR, rcP, rcY;     // us
  uint16_t m[4];                   // us
  uint16_t vbat100, ctrlUs;
  int16_t  alt100, altT100;        // estimate / hold target, cm
  int16_t  accUp100, accCorr100, altI10;   // vertical acceleration and its offset (cm/s^2), VEL I correction (us x10)
  int16_t  vel100;                 // vertical speed, cm/s
  uint8_t  flags;                  // bit0 mode ALT, bit1 alt valid, bit2 landed, bit3 failsafe landing
  uint8_t  vib10;
  int8_t   rssi;
  uint8_t  lq;
};

// ======================= SETTINGS =======================
Params P;
// Menu groups (FC OLED SETUP page). Each setting: NVS/serial name, OLED label, default, range, menu step, decimals, group.
enum { G_LEVEL, G_PID, G_FILT, G_ALT, G_TRIM, G_BATT, G_OSD, G_COUNT };
const char *const GROUP_NAME[G_COUNT] = {"SELF-LEVEL", "GYRO PID", "FILTERS", "ALT HOLD", "TRIM / MOTORS", "BATTERY", "DJI OSD"};
const ParamDef PARAMS[] = {
  {"angleMax",     "Max angle",     &Params::angleMax,     35,    10, 60,     1,     0, G_LEVEL},  // deg at full stick
  {"angleExpo",    "Angle expo",    &Params::angleExpo,    0.40f, 0, 1,       0.05f, 2, G_LEVEL},
  {"lvRollP",      "Level P roll",  &Params::lvRollP,      2.9f,  0, 20,      0.1f,  1, G_LEVEL},  // deg/s per deg
  {"lvRollI",      "Level I roll",  &Params::lvRollI,      0.05f, 0, 10,      0.01f, 2, G_LEVEL},
  {"lvRollD",      "Level D roll",  &Params::lvRollD,      0.025f, 0, 1,     0.005f, 3, G_LEVEL},
  {"lvPitchP",     "Level P pitch", &Params::lvPitchP,     3.1f,  0, 20,      0.1f,  1, G_LEVEL},
  {"lvPitchI",     "Level I pitch", &Params::lvPitchI,     0.05f, 0, 10,      0.01f, 2, G_LEVEL},
  {"lvPitchD",     "Level D pitch", &Params::lvPitchD,     0.025f, 0, 1,     0.005f, 3, G_LEVEL},
  {"lvILim",       "Level I limit", &Params::lvILim,       35,    0, 200,     5,     0, G_LEVEL},  // deg/s
  {"lvMaxRate",    "Level max d/s", &Params::lvMaxRate,    820,   50, 1000,   10,    0, G_LEVEL},  // deg/s
  {"accTauFlt",    "Acc tau fly s", &Params::accTauFlt,    5.2f,  0.3f, 20,   0.1f,  1, G_LEVEL},  // accel trust in flight
  {"gRollP",       "Roll P",        &Params::gRollP,       0.51f,  0, 5,       0.01f, 2, G_PID},    // us per deg/s
  {"gRollI",       "Roll I",        &Params::gRollI,       0.23f,  0, 5,       0.01f, 2, G_PID},
  {"gRollD",       "Roll D",        &Params::gRollD,       0.005f, 0, 0.2f,   0.001f, 3, G_PID},
  {"gPitchP",      "Pitch P",       &Params::gPitchP,      0.50f,  0, 5,       0.01f, 2, G_PID},
  {"gPitchI",      "Pitch I",       &Params::gPitchI,      0.26f,  0, 5,       0.01f, 2, G_PID},
  {"gPitchD",      "Pitch D",       &Params::gPitchD,      0.005f, 0, 0.2f,   0.001f, 3, G_PID},
  {"gYawP",        "Yaw P",         &Params::gYawP,        1.59f,  0, 5,       0.01f, 2, G_PID},
  {"gYawI",        "Yaw I",         &Params::gYawI,        1.25f,  0, 5,       0.01f, 2, G_PID},
  {"yawRateC",     "Yaw rate ctr",  &Params::yawRateC,     110,   10, 500,    5,     0, G_PID},    // deg/s centre
  {"yawRateMax",   "Yaw rate max",  &Params::yawRateMax,   780,   50, 1000,   10,    0, G_PID},    // deg/s full stick
  {"yawExpo",      "Yaw expo",      &Params::yawExpo,      0.15f, 0, 1,       0.05f, 2, G_PID},
  {"gyroLpf",      "Gyro LPF Hz",   &Params::gyroLpf,      80,    10, 180,    5,     0, G_FILT},   // too low = slow wobble
  {"dtermLpf",     "D-term LPF Hz", &Params::dtermLpf,     30,    5, 180,     5,     0, G_FILT},
  {"notchHz",      "Notch Hz",      &Params::notchHz,      25,    0, 190,     5,     0, G_FILT},   // 0 = off
  {"notchQ",       "Notch Q",       &Params::notchQ,       4,     0.5f, 10,   0.5f,  1, G_FILT},
  {"accLpfHz",     "Acc LPF Hz",    &Params::accLpfHz,     40,    1, 100,     1,     0, G_FILT},   // self-level correction
  {"lvDLpfHz",     "Level D LPF",   &Params::lvDLpfHz,     16,    2, 100,     1,     0, G_FILT},
  {"rcSmoothHz",   "RC smooth Hz",  &Params::rcSmoothHz,   20,    0, 50,      1,     0, G_FILT},   // 0 = off
  {"itermRelax",   "I relax d/s",   &Params::itermRelax,   40,    0, 200,     5,     0, G_FILT},   // 0 = off
  {"antiGrav",     "Anti gravity",  &Params::antiGrav,     3,     0, 10,      0.5f,  1, G_FILT},   // 0 = off
  {"altP",         "Alt P",         &Params::altP,         2.0f,  0.1f, 5,    0.1f,  1, G_ALT},    // m/s per m
  {"velP",         "Vel P",         &Params::velP,         125,   0, 500,     5,     0, G_ALT},    // us per m/s
  {"velI",         "Vel I",         &Params::velI,         50,    0, 300,     5,     0, G_ALT},    // us per m/s*s
  {"maxClimb",     "Max climb m/s", &Params::maxClimb,     0.5f,  0.2f, 5,    0.1f,  1, G_ALT},
  {"maxDesc",      "Max desc m/s",  &Params::maxDesc,      0.5f,  0.2f, 3,    0.1f,  1, G_ALT},    // also auto-land speed
  {"deadband",     "Stick dband",   &Params::deadband,     100,    20, 250,    5,     0, G_ALT},    // us around hover = hold
  {"minThr",       "Min throttle",  &Params::minThr,       1100,  1050, 1500, 10,    0, G_ALT},    // lowest in the air
  {"hover",        "Hover thr",     &Params::hover,        1500,  1100, 1800, 5,     0, G_ALT},     // brushed micros hover near 50 %    // learned (ref 3.85 V/cell)
  {"ceilingM",     "Ceiling m",     &Params::ceilingM,     0,     0, 100,     0.5f,  1, G_ALT},    // above the arming point, 0 = off
  {"thrMid",       "Stick mid 0=hvr", &Params::thrMid,     1350,  0, 1800,    10,    0, G_ALT},    // what the TX sends at mid stick (SimpleTX Thr centre), also holds
  {"estTau",       "Est blend s",   &Params::estTau,       BARO_TAU, 0.5f, 5, 0.1f,  1, G_ALT},    // shorter trusts the baro more for vertical speed
  {"rollTrim",     "Roll trim",     &Params::rollTrim,     0.4f,   -TRIM_LIMIT, TRIM_LIMIT, 0.1f, 1, G_TRIM},  // deg
  {"pitchTrim",    "Pitch trim",    &Params::pitchTrim,    -3.56f,   -TRIM_LIMIT, TRIM_LIMIT, 0.1f, 1, G_TRIM},
  {"rollSensRev",  "Roll sens rev", &Params::rollSensRev,  0,     0, 1,       1,     0, G_TRIM},   // 1 = reversed
  {"pitchSensRev", "Pitch sns rev", &Params::pitchSensRev, 0,     0, 1,       1,     0, G_TRIM},
  {"propsIn",      "Props in",      &Params::propsIn,      1,     0, 1,       1,     0, G_TRIM},   // 1 = props in (M1 CW)
  {"motorIdle",    "Motor idle us", &Params::motorIdle,    1050,  1000, 1300, 5,     0, G_TRIM},     // brushed: ~5 % duty spins slowly
  {"vRatio",       "Volt ratio",    &Params::vRatio,       4.03f, 1, 40,      0.01f, 2, G_BATT},   // (R1+R2)/R2, 100k/33k: 8.4 V gives 2.08 V at the pin
  {"cells",        "Cells (0=auto)", &Params::cells,       2,     0, 8,       1,     0, G_BATT},
  {"cellWarn",     "Warn V/cell",   &Params::cellWarn,     3.5f,  3.0f, 4.0f, 0.05f, 2, G_BATT},
  {"osdOn",        "OSD on",        &Params::osdOn,        0,     0, 1,       1,     0, G_OSD},    // MSP DisplayPort to DJI O4
  {"osdCols",      "Canvas cols",   &Params::osdCols,      53,    30, 60,     1,     0, G_OSD},    // set by the air unit if it says
  {"osdRows",      "Canvas rows",   &Params::osdRows,      20,    16, 22,     1,     0, G_OSD},
};const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);
#define N_PARAMS_MAX 64
Preferences prefs;

void paramsDefault() { for (int i = 0; i < N_PARAMS; i++) P.*(PARAMS[i].m) = PARAMS[i].def; }

void paramsSanitize() {             // also rejects NaN
  for (int i = 0; i < N_PARAMS; i++) {
    float &v = P.*(PARAMS[i].m);
    if (!(v >= PARAMS[i].mn && v <= PARAMS[i].mx)) v = PARAMS[i].def;
  }
}

// One-time import from the old FlightController sketch (same board, NVS namespace "fc"): sensor directions,
// trims, motor idle, battery ratio, tuned gains and the learned hover throttle. Layout copied from that sketch.
struct OldCfg { uint8_t b[12]; float motorIdle, rollTrim, pitchTrim, attTauFlight, vRatio, cells, cellWarn, extrasVer; };
struct OldTune {
  float rcRollC, rcRollMax, rcRollExpo, rcPitchC, rcPitchMax, rcPitchExpo, rcYawC, rcYawMax, rcYawExpo;
  float lvAngle, lvExpo, lvRollP, lvRollI, lvRollD, lvPitchP, lvPitchI, lvPitchD, lvILim, lvMaxRate;
  float gRollP, gRollI, gRollD, gPitchP, gPitchI, gPitchD, gYawP, gYawI, gYawD, gyroLpf, dtermLpf;
};
struct OldStore { uint32_t magic; OldCfg cfg; OldTune tune[2]; uint32_t crc; };

uint32_t fnv32(const uint8_t *d, size_t n) {
  uint32_t h = 2166136261UL;
  while (n--) { h ^= *d++; h *= 16777619UL; }
  return h;
}

bool importOldFc() {
  return false;                      // brushed: the old brushless FlightController tuning does not fit this quad
  OldStore s;
  bool ok = false;
  float hov = 0;
  if (!prefs.begin("fc", true)) return false;
  if (prefs.getBytesLength("store") == sizeof(OldStore)) {
    prefs.getBytes("store", &s, sizeof s);
    ok = s.magic == 0xFC0C3A04UL && s.crc == fnv32((const uint8_t *)&s, offsetof(OldStore, crc));
  }
  hov = prefs.getFloat("hover", 0);
  float filt[6], alt[9];             // old "filt" (notch Hz, Q, acc LPF, level D LPF, ..) and "alth" (altP, velP, velI, climb, desc, deadband, minThr, ..)
  bool haveFilt = prefs.getBytesLength("filt") == sizeof filt && prefs.getBytes("filt", filt, sizeof filt) == sizeof filt;
  bool haveAlt = prefs.getBytesLength("alth") == sizeof alt && prefs.getBytes("alth", alt, sizeof alt) == sizeof alt;
  float rcsm = prefs.getFloat("rcsm", -1), irel = prefs.getFloat("irel", -1), antig = prefs.getFloat("antig", -1);
  prefs.end();
  if (!ok) return false;
  const OldCfg &c = s.cfg;
  const OldTune &t = s.tune[0];                 // quad set
  P.propsIn = c.b[1] ? 1 : 0;
  P.rollSensRev = c.b[5] ? 1 : 0;  P.pitchSensRev = c.b[6] ? 1 : 0;
  P.motorIdle = c.motorIdle;  P.rollTrim = c.rollTrim;  P.pitchTrim = c.pitchTrim;  P.accTauFlt = c.attTauFlight;
  P.vRatio = c.vRatio;  P.cells = c.cells;  P.cellWarn = c.cellWarn;
  P.angleMax = t.lvAngle;  P.angleExpo = t.lvExpo;
  P.lvRollP = t.lvRollP;  P.lvRollI = t.lvRollI;  P.lvRollD = t.lvRollD;
  P.lvPitchP = t.lvPitchP;  P.lvPitchI = t.lvPitchI;  P.lvPitchD = t.lvPitchD;
  P.lvILim = t.lvILim;  P.lvMaxRate = t.lvMaxRate;
  P.yawRateC = t.rcYawC;  P.yawRateMax = t.rcYawMax;  P.yawExpo = t.rcYawExpo;
  P.gRollP = t.gRollP;  P.gRollI = t.gRollI;  P.gRollD = t.gRollD;
  P.gPitchP = t.gPitchP;  P.gPitchI = t.gPitchI;  P.gPitchD = t.gPitchD;
  P.gYawP = t.gYawP;  P.gYawI = t.gYawI;
  P.gyroLpf = t.gyroLpf;  P.dtermLpf = t.dtermLpf;
  if (hov > 0) P.hover = hov;
  if (haveFilt) { P.notchHz = filt[0]; P.notchQ = filt[1]; P.accLpfHz = filt[2]; P.lvDLpfHz = filt[3]; }
  if (haveAlt) { P.altP = alt[0]; P.velP = alt[1]; P.velI = alt[2]; P.maxClimb = alt[3]; P.maxDesc = alt[4];
                 P.deadband = alt[5]; P.minThr = alt[6]; P.ceilingM = alt[7]; }
  if (rcsm >= 0) P.rcSmoothHz = rcsm;
  if (irel >= 0) P.itermRelax = irel;
  if (antig >= 0) P.antiGrav = antig;
  return true;
}

bool paramsSave() {
  if (!prefs.begin("sfcb", false)) return false;
  bool ok = true;
  for (int i = 0; i < N_PARAMS; i++) ok &= prefs.putFloat(PARAMS[i].name, P.*(PARAMS[i].m)) == sizeof(float);
  prefs.end();
  return ok;
}

void paramsLoad() {
  paramsDefault();
  bool have = false;
  if (prefs.begin("sfcb", true)) {
    have = prefs.isKey("gRollP");
    for (int i = 0; i < N_PARAMS; i++) P.*(PARAMS[i].m) = prefs.getFloat(PARAMS[i].name, PARAMS[i].def);
    prefs.end();
  }
  if (!have) {
    if (importOldFc()) { paramsSanitize(); paramsSave(); Serial.println("Settings imported from the old FlightController sketch."); }
    else Serial.println("No settings in flash: defaults.");
  }
  paramsSanitize();
}

void saveHover() { if (prefs.begin("sfcb", false)) { prefs.putFloat("hover", P.hover); prefs.end(); } }
void saveTrims() {
  if (prefs.begin("sfcb", false)) { prefs.putFloat("rollTrim", P.rollTrim); prefs.putFloat("pitchTrim", P.pitchTrim); prefs.end(); }
}

// ======================= STATE =======================
#if USE_OLED
U8G2 *oled;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oledDev(U8G2_R0, U8X8_PIN_NONE);
#endif

bool armed = false, armSeenLow = false, inFlight = false, liftedOff = false;
Mode mode = MODE_SELF;
ArmBlock armBlock = AB_NOLINK;
uint16_t ch[RX_NUM_CH] = {1000, 1500, 1500, 1500, 1000, 1000};   // TX order: THR ROLL PITCH YAW ARM CH6
bool rcValid = false, rcFresh = false, fsLanding = false;
uint32_t fsLandStart = 0;
uint16_t rcGapCount = 0; uint32_t rcGapMaxMs = 0;
float rcRssiF = 0; uint8_t rcLq = 0;

float gyroBias[3] = {0, 0, 0}, gyro[3] = {0, 0, 0}, acc[3] = {0, 0, 1};
float roll = 0, pitch = 0, accRollRaw = 0, accPitchRaw = 0, accMag = 1;
float accRef = 1;                  // slow average of |a| (this sensor's idea of 1 g)
bool imuStill() { return fabsf(gyro[0]) < 4 && fabsf(gyro[1]) < 4 && fabsf(gyro[2]) < 4 && fabsf(accMag - accRef) < 0.03f; }
bool imuOK = true; int imuFail = 0;
Biquad gLpf[3], gNotch[3], accLpf[3], vibHpf[3];
bool notchOn = false;
float vibSq[3] = {0, 0, 0}, vibFastSq[3] = {0, 0, 0};
float rollSensSign = 1, pitchSensSign = 1;

PID pidRoll(GYRO_I_LIMIT, GYRO_OUT_LIMIT), pidPitch(GYRO_I_LIMIT, GYRO_OUT_LIMIT), pidYaw(GYRO_I_LIMIT, GYRO_OUT_LIMIT);
PID pidLvRoll(30, 600), pidLvPitch(30, 600);
float rollTgt = 0, pitchTgt = 0;
int outUs[4] = {1000, 1000, 1000, 1000};

float vBat = 0; bool battPresent = false, battLow = false; uint8_t cellsUsed = 0;

bool baroOK = false; uint8_t baroAddr = 0;
uint16_t bT1; int16_t bT2, bT3; uint16_t bP1; int16_t bP2, bP3, bP4, bP5, bP6, bP7, bP8, bP9;
float baroPa = 0, baroGroundPa = 0, baroAlt = 0; bool baroNew = false;

float altEst = 0, velEst = 0, accCorr = 0, accUp = 0; bool altValid = false;
bool altHold = false, altLanded = false;
uint32_t altLandSince = 0, altStuckSince = 0;
float altZoneLo = 1350, altZoneHi = 1550;   // throttle stick inside this = hold height
float altTarget = 0, altInteg = 0, altRefThr = 1450, altHoverBase = 1450, altThrRef = 1450, altThrOut = 1000;
uint32_t exitBlendStart = 0; float exitBlendFrom = 1000;

char msgText[24] = ""; uint32_t msgUntil = 0;
uint16_t loopUs = 0, loopMaxUs = 0, ctrlUs = 0, ctrlMaxUs = 0;
bool statusPrint = false;           // S toggles it: quiet by default, this port is the configuration console
bool levelCalRunning = false, levelCalRequest = false;
enum { PG_STATUS, PG_SENSORS, PG_STATS, PG_TREND, PG_SETUP, PG_COUNT };   // OLED pages (BOOT button; SETUP last)
uint8_t page = PG_STATUS;

// ======================= HELPERS =======================
void setMsgFor(const char *s, uint32_t ms) {
  strncpy(msgText, s, sizeof msgText - 1); msgText[sizeof msgText - 1] = 0; msgUntil = millis() + ms;
#if !USE_OLED
  Serial.println(msgText);          // no display: messages go to the console
#endif
}
void setMsg(const char *s) { setMsgFor(s, 3000); }
uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) { c ^= *d++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
  return c;
}
float stick(uint16_t v) {                   // -1..+1, 3 % deadband, rescaled
  const float DB = 0.03f;
  float x = constrain((v - 1500.0f) / 500.0f, -1.0f, 1.0f);
  if (fabsf(x) < DB) return 0;
  return (x > 0 ? x - DB : x + DB) / (1 - DB);
}
float expoCurve(float x, float e) { return (1 - e) * x + e * x * x * x; }
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v); return Wire.endTransmission() == 0;
}

// ======================= BUZZER (passive: driven with a tone) =======================
// one-shot patterns: bit i = 100 ms slot i
uint32_t evBits = 0, evStart = 0; uint8_t evLen = 0;
bool lostModel = false; uint32_t lostSince = 0;
#define BUZZ_HZ 2700                // most small passive buzzers are loudest around 2-4 kHz
void buzz(bool on) {
  static int8_t st = -1;            // only touch the LEDC when it changes
  if (st == (on ? 1 : 0)) return;
  st = on ? 1 : 0;
  ledcWrite(PIN_BUZZ, on ? 128 : 0);   // 50 % square wave = tone
}
void beep(uint32_t bits, uint8_t len) { evBits = bits; evLen = len; evStart = millis(); }
#define BEEP_SHORT  beep(0x1, 2)
#define BEEP_LONG   beep(0x3, 3)
#define BEEP_TWO    beep(0x5, 4)
#define BEEP_THREE  beep(0x15, 6)
bool patternOn(uint32_t bits, uint8_t len, uint32_t t) { return (bits >> ((t / 100) % len)) & 1; }
void buzzerTick(uint32_t ms) {
  static bool wasArmed = false;
  if (wasArmed && !armed && !rcValid) { lostModel = true; lostSince = ms; }   // failsafe disarm: find-me beeps
  if (rcValid || (lostModel && ms - lostSince > 300000UL)) lostModel = false;
  wasArmed = armed;
  bool on = false;
  if (!imuOK)                                          on = patternOn(0x1, 2, ms);
  else if (evLen && ms - evStart < evLen * 100UL)      on = patternOn(evBits, evLen, ms - evStart);
  else if (lostModel)                                  on = patternOn(0x7, 6, ms);
  else if (battLow)                                    on = patternOn(0x5, 20, ms);
  buzz(on);
}

// ======================= MOTORS =======================
void motorsBegin() { for (int i = 0; i < 4; i++) ledcAttach(MOTOR_PIN[i], PWM_HZ, PWM_BITS); }
void writeUs(int i, int us) {                 // brushed: 1000 us = 0 % duty (off), 2000 us = 100 % duty
  us = constrain(us, 1000, 2000);
  ledcWrite(MOTOR_PIN[i], (uint32_t)((us - 1000) * ((1 << PWM_BITS) - 1) / 1000));
}
void motorsOff() { for (int i = 0; i < 4; i++) { outUs[i] = 1000; writeUs(i, 1000); } }

// ======================= MPU6050 =======================
#define MPU_ADDR 0x68
const float GYRO_SCALE = 32.8f;    // +-1000 dps
const float ACC_SCALE = 2048.0f;   // +-16 g (less clipping under vibration)

bool mpuRaw(int16_t *r) {           // r[0..2] acc, r[3..5] gyro
  uint8_t b[14];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) { r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]); r[3 + i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]); }
  return true;
}

bool mpuInit() {
  uint8_t id = 0;
  if (!i2cRead(MPU_ADDR, 0x75, &id, 1) || (id != 0x68 && id != 0x70 && id != 0x72)) {
    Serial.printf("MPU6050 not found (WHO_AM_I 0x%02X)\n", id);
    return false;
  }
  i2cWrite8(MPU_ADDR, 0x6B, 0x01); delay(50);   // wake, PLL
  i2cWrite8(MPU_ADDR, 0x19, 0x01);              // 500 Hz sample rate
  i2cWrite8(MPU_ADDR, 0x1A, 0x02);              // DLPF ~98 Hz (anti-aliasing)
  i2cWrite8(MPU_ADDR, 0x1B, 0x10);              // +-1000 dps
  i2cWrite8(MPU_ADDR, 0x1C, 0x18);              // +-16 g
  return true;
}

void filtersSetup() {
  rollSensSign = P.rollSensRev > 0.5f ? -1 : 1;
  pitchSensSign = P.pitchSensRev > 0.5f ? -1 : 1;
  notchOn = P.notchHz > 1 && P.notchHz < LOOP_HZ / 2.0f;
  for (int k = 0; k < 3; k++) {
    gLpf[k].lpf(LOOP_HZ, constrain(P.gyroLpf, 10.0f, 0.45f * LOOP_HZ));
    if (notchOn) gNotch[k].notch(LOOP_HZ, P.notchHz, P.notchQ);
    accLpf[k].lpf(LOOP_HZ, P.accLpfHz);
    vibHpf[k].hpf(LOOP_HZ, 40.0f);
  }
  float dl = constrain(P.dtermLpf, 5.0f, 0.45f * LOOP_HZ);
  pidRoll.dLpf.lpf(LOOP_HZ, dl); pidPitch.dLpf.lpf(LOOP_HZ, dl); pidYaw.dLpf.lpf(LOOP_HZ, dl);
  pidLvRoll.dLpf.lpf(LOOP_HZ, P.lvDLpfHz); pidLvPitch.dLpf.lpf(LOOP_HZ, P.lvDLpfHz);
  pidRoll.kp = P.gRollP;   pidRoll.ki = P.gRollI;   pidRoll.kd = P.gRollD;
  pidPitch.kp = P.gPitchP; pidPitch.ki = P.gPitchI; pidPitch.kd = P.gPitchD;
  pidYaw.kp = P.gYawP;     pidYaw.ki = P.gYawI;     pidYaw.kd = 0;
  pidLvRoll.kp = P.lvRollP;   pidLvRoll.ki = P.lvRollI;   pidLvRoll.kd = P.lvRollD;
  pidLvPitch.kp = P.lvPitchP; pidLvPitch.ki = P.lvPitchI; pidLvPitch.kd = P.lvPitchD;
  pidLvRoll.iLim = pidLvPitch.iLim = P.lvILim;
  pidLvRoll.oLim = pidLvPitch.oLim = P.lvMaxRate;
}

void resetPIDs() { pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidLvRoll.reset(); pidLvPitch.reset(); }

float vibration()     { return sqrtf((vibSq[0] + vibSq[1] + vibSq[2]) / 3); }
float vibrationFast() { return sqrtf((vibFastSq[0] + vibFastSq[1] + vibFastSq[2]) / 3); }

void readIMU(float dt) {
  int16_t r[6];
  if (!mpuRaw(r)) { if (++imuFail > 50) imuOK = false; return; }
  imuFail = 0; imuOK = true;
  float a[3], gc[3];
  for (int k = 0; k < 3; k++) {
    a[k] = r[k] / ACC_SCALE;
    acc[k] = accLpf[k].apply(a[k]);
    gc[k] = r[3 + k] / GYRO_SCALE - gyroBias[k];
    float h = vibHpf[k].apply(gc[k]);
    vibSq[k] += 0.002f * (h * h - vibSq[k]);
    vibFastSq[k] += 0.05f * (h * h - vibFastSq[k]);
    gyro[k] = gLpf[k].apply(notchOn ? gNotch[k].apply(gc[k]) : gc[k]);
  }
  gyro[0] *= rollSensSign;
  gyro[1] *= pitchSensSign;

  if (!armed) {                     // gyro bias tracking while disarmed and still (tau ~5 s)
    float am = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (fabsf(gc[0]) < 2 && fabsf(gc[1]) < 2 && fabsf(gc[2]) < 2 && fabsf(am - 1) < 0.15f)
      for (int k = 0; k < 3; k++) gyroBias[k] += gc[k] * 0.0005f;
  }

  roll += gyro[0] * dt;
  pitch += gyro[1] * dt;
  if (roll > 180) roll -= 360; else if (roll < -180) roll += 360;
  accMag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  accRef += 0.002f * (accMag - accRef);   // tau ~1.2 s
  accRollRaw  = atan2f(rollSensSign * acc[1], acc[2]) * RAD_TO_DEG;
  accPitchRaw = atan2f(-pitchSensSign * acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG;
  if (fabsf(accMag - 1) < 0.25f) {  // ignore the accelerometer when |a| is far from 1 g
    float tau = inFlight ? P.accTauFlt : ATT_TAU_GROUND;
    float k = dt / (tau + dt);
    float dr = (accRollRaw - P.rollTrim) - roll;
    if (dr > 180) dr -= 360; else if (dr < -180) dr += 360;
    roll += k * dr;
    pitch += k * ((accPitchRaw - P.pitchTrim) - pitch);
  }
}

bool calibrateGyro() {              // keep still; retries while it moves
  for (int attempt = 0; attempt < 5; attempt++) {
    float s[3] = {0, 0, 0}, sa[3] = {0, 0, 0}, mn[3] = {1e9, 1e9, 1e9}, mx[3] = {-1e9, -1e9, -1e9};
    int good = 0;
    int16_t r[6];
    for (int i = 0; i < 800; i++) {
      if (mpuRaw(r)) {
        for (int k = 0; k < 3; k++) {
          float g = r[3 + k] / GYRO_SCALE;
          s[k] += g; sa[k] += r[k] / ACC_SCALE; mn[k] = min(mn[k], g); mx[k] = max(mx[k], g);
        }
        good++;
      }
      delay(2);
    }
    if (good < 400) continue;
    bool still = true;
    for (int k = 0; k < 3; k++) if (mx[k] - mn[k] > 8) still = false;
    if (!still) { Serial.println("Gyro cal: moving, again..."); continue; }
    for (int k = 0; k < 3; k++) gyroBias[k] = s[k] / good;
    float ax = sa[0] / good, ay = sa[1] / good, az = sa[2] / good;
    roll  = atan2f(rollSensSign * ay, az) * RAD_TO_DEG - P.rollTrim;
    pitch = atan2f(-pitchSensSign * ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - P.pitchTrim;
    for (int i = 0; i < 300; i++) { readIMU(0.0025f); delay(2); }   // settle the filters
    vibSq[0] = vibSq[1] = vibSq[2] = 0;
    return true;
  }
  return false;
}

// ======================= BATTERY =======================
void batteryTick(uint32_t ms) {
  static uint32_t last = 0, lowSince = 0;
  if (ms - last < 10) return;
  last = ms;
  float v = analogReadMilliVolts(PIN_VBAT) * 0.001f * P.vRatio;
  vBat = (vBat == 0) ? v : vBat + 0.02f * (v - vBat);     // tau ~0.5 s
  battPresent = vBat > 3.0f;
  if (!battPresent) { battLow = false; lowSince = 0; return; }
  if (P.cells >= 1) cellsUsed = (uint8_t)P.cells;
  else if (cellsUsed == 0 && !armed && ms > 1500) cellsUsed = constrain((int)(vBat / 4.35f) + 1, 1, 8);
  if (!cellsUsed) return;
  float c = vBat / cellsUsed;
  if (c < P.cellWarn) { if (!lowSince) lowSince = ms; if (ms - lowSince > 2000) battLow = true; }
  else if (c > P.cellWarn + 0.05f) { lowSince = 0; battLow = false; }
}

// battery throttle compensation (iNav thr_comp): alt hold works in "reference throttle" at 3.85 V/cell
float batComp() {
  static float vF = 0;
  if (!battPresent || !cellsUsed || vBat < cellsUsed * 3.0f) { vF = 0; return 1; }
  vF = (vF == 0) ? vBat : vF + 0.002f * (vBat - vF);
  return constrain(cellsUsed * BATCOMP_VREF_CELL / vF, 0.85f, 1.25f);
}
float thrToRef(float raw)   { return 1000 + (raw - 1000) / batComp(); }
float thrFromRef(float ref) { return 1000 + (ref - 1000) * batComp(); }
float hoverPoint() { return constrain(thrFromRef(P.hover), 1200.0f, 1750.0f); }   // stick that hovers NOW

// ======================= BAROMETER: BMP280 or MS5611 (GY-63), auto-detected. Own drivers, never block =======================
// MS5611 (GY-63). I2C needs PS high; CSB low = address 0x77, CSB high = 0x76. It has no ID register: it is recognised by
// answering at the address and by its PROM. Each reading takes two conversions (temperature, pressure, up to 9.04 ms each
// at OSR 4096), so baroTick steps it every 20 ms: one pressure value per 40 ms, the same 25 Hz as the BMP280.
bool baroIsMs = false;
uint16_t msC[8];

bool msCmd(uint8_t c) { Wire.beginTransmission(baroAddr); Wire.write(c); return Wire.endTransmission() == 0; }

bool msRead(uint8_t reg, uint8_t *buf, uint8_t n) {   // this chip wants a STOP between the command and the read
  if (!msCmd(reg)) return false;
  if (Wire.requestFrom(baroAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool msCrcOk() {                    // Measurement Specialties AN520: CRC-4 over the 8 PROM words
  uint16_t rem = 0;
  for (int i = 0; i < 16; i++) {
    uint16_t w = msC[i >> 1];
    if (i >= 14) w &= 0xFF00;       // the CRC itself (low byte of word 7) counts as 0
    rem ^= (i & 1) ? (w & 0xFF) : (w >> 8);
    for (int b = 0; b < 8; b++) rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
  }
  return ((rem >> 12) & 0xF) == (msC[7] & 0xF);
}

bool msInit() {
  for (uint8_t a : {(uint8_t)0x77, (uint8_t)0x76}) {
    baroAddr = a;
    if (!msCmd(0x1E)) continue;     // reset; no ACK = nothing at this address
    delay(4);
    bool ok = true;
    for (int i = 0; i < 8 && ok; i++) {
      uint8_t b[2];
      ok = msRead(0xA0 + 2 * i, b, 2);
      if (ok) msC[i] = (b[0] << 8) | b[1];
    }
    for (int i = 1; i <= 6 && ok; i++) if (msC[i] == 0 || msC[i] == 0xFFFF) ok = false;
    if (!ok) continue;
    if (!msCrcOk()) Serial.println("MS5611: PROM CRC mismatch (clone?), using it anyway");
    baroIsMs = true;
    Serial.printf("MS5611 (GY-63) at 0x%02X\n", a);
    return true;
  }
  baroAddr = 0;
  Serial.println("Barometer not found (BMP280 / MS5611)");
  return false;
}

// one step per call; true = a new pressure in baroPa. A failed read restarts the cycle.
bool msStep() {
  static uint8_t st = 0;
  static uint32_t d2 = 0;
  uint8_t b[3];
  if (st == 0) { if (msCmd(0x58)) st = 1; return false; }                       // start temperature, OSR 4096
  if (!msRead(0x00, b, 3)) { st = 0; return false; }
  uint32_t adc = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  if (adc == 0) { st = 0; return false; }                                       // read too early or chip busy
  if (st == 1) { d2 = adc; st = msCmd(0x48) ? 2 : 0; return false; }            // start pressure, OSR 4096
  uint32_t d1 = adc;
  st = msCmd(0x58) ? 1 : 0;                                                     // next temperature
  int32_t dT = (int32_t)d2 - ((int32_t)msC[5] << 8);
  int32_t temp = 2000 + (int32_t)(((int64_t)dT * msC[6]) >> 23);
  int64_t off = ((int64_t)msC[2] << 16) + (((int64_t)msC[4] * dT) >> 7);
  int64_t sens = ((int64_t)msC[1] << 15) + (((int64_t)msC[3] * dT) >> 8);
  if (temp < 2000) {                                                            // second-order compensation (datasheet)
    int64_t tt = (int64_t)(temp - 2000) * (temp - 2000);
    off -= 5 * tt / 2; sens -= 5 * tt / 4;
    if (temp < -1500) { int64_t t2 = (int64_t)(temp + 1500) * (temp + 1500); off -= 7 * t2; sens -= 11 * t2 / 2; }
  }
  int64_t p = ((((int64_t)d1 * sens) >> 21) - off) >> 15;                       // 0.01 mbar = Pa
  baroPa = (float)p;
  return baroPa > 30000 && baroPa < 120000;
}

bool baroInit() {
  for (uint8_t a : {(uint8_t)0x76, (uint8_t)0x77}) {
    uint8_t id = 0;
    if (i2cRead(a, 0xD0, &id, 1) && (id == 0x58 || id == 0x60)) { baroAddr = a; break; }
  }
  if (!baroAddr) return msInit();
  uint8_t c[24];
  if (!i2cRead(baroAddr, 0x88, c, 24)) return false;
  bT1 = c[0] | (c[1] << 8);   bT2 = c[2] | (c[3] << 8);   bT3 = c[4] | (c[5] << 8);
  bP1 = c[6] | (c[7] << 8);   bP2 = c[8] | (c[9] << 8);   bP3 = c[10] | (c[11] << 8);
  bP4 = c[12] | (c[13] << 8); bP5 = c[14] | (c[15] << 8); bP6 = c[16] | (c[17] << 8);
  bP7 = c[18] | (c[19] << 8); bP8 = c[20] | (c[21] << 8); bP9 = c[22] | (c[23] << 8);
  i2cWrite8(baroAddr, 0xF5, 0x10);  // standby 0.5 ms, IIR x16
  i2cWrite8(baroAddr, 0xF4, 0x57);  // T x2, P x16, normal mode (~23 Hz)
  Serial.printf("BMP280 at 0x%02X\n", baroAddr);
  return true;
}

bool baroRead() {                   // Bosch float compensation
  uint8_t d[6];
  if (!i2cRead(baroAddr, 0xF7, d, 6)) return false;
  int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  if (adcP == 0x80000 || adcT == 0x80000) return false;
  float v1 = (adcT / 16384.0f - bT1 / 1024.0f) * bT2;
  float v2 = adcT / 131072.0f - bT1 / 8192.0f;
  float tFine = v1 + v2 * v2 * bT3;
  float p1 = tFine / 2 - 64000, p2 = p1 * p1 * bP6 / 32768.0f;
  p2 = (p2 + p1 * bP5 * 2) / 4 + bP4 * 65536.0f;
  p1 = (bP3 * p1 * p1 / 524288.0f + bP2 * p1) / 524288.0f;
  p1 = (1 + p1 / 32768.0f) * bP1;
  if (p1 == 0) return false;
  float p = (1048576.0f - adcP - p2 / 4096.0f) * 6250.0f / p1;
  baroPa = p + (bP9 * p * p / 2147483648.0f + p * bP8 / 32768.0f + bP7) / 16;
  return baroPa > 30000 && baroPa < 120000;
}

void baroTick(uint32_t ms) {
  static uint32_t last = 0;
  if (!baroOK || ms - last < (baroIsMs ? BARO_PERIOD_MS / 2u : (uint32_t)BARO_PERIOD_MS)) return;
  last = ms;
  if (!(baroIsMs ? msStep() : baroRead())) return;
  if (baroGroundPa <= 0) { baroGroundPa = baroPa; baroAlt = 0; }
  float a = 44330.0f * (1 - powf(baroPa / baroGroundPa, 0.190295f));
  baroAlt += (1 - expf(-2 * PI * BARO_LPF_HZ * BARO_PERIOD_MS * 1e-3f)) * (a - baroAlt);
  baroNew = true;
}

void baroZero() { if (baroOK && baroPa > 0) { baroGroundPa = baroPa; baroAlt = 0; altValid = false; } }

// ======================= ALTITUDE ESTIMATE (3rd-order complementary: accel + baro) =======================
void altEstTick(float dt) {
  float r = roll * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
  float ux = -pitchSensSign * sinf(p), uy = rollSensSign * sinf(r) * cosf(p), uz = cosf(r) * cosf(p);
  accUp = ((acc[0] * ux + acc[1] * uy + acc[2] * uz) - 1.0f) * 9.80665f;   // m/s^2, gravity removed
  if (!baroOK || baroGroundPa <= 0) { altValid = false; return; }
  if (!altValid) { altEst = baroAlt; velEst = 0; altValid = true; }
  // disarmed and still: the vertical speed is 0 -> learn the accelerometer bias, pin the estimate to the baro
  bool still = !armed && imuStill();
  if (still) { accCorr += (-accUp - accCorr) * dt; velEst = 0; altEst = baroAlt; return; }
  const float tau = P.estTau, k1 = 3 / tau, k2 = 3 / (tau * tau), k3 = 1 / (tau * tau * tau);
  float err = baroAlt - altEst;
  accCorr = constrain(accCorr + err * k3 * dt, -3.0f, 3.0f);
  velEst = constrain(velEst + (accUp + accCorr) * dt + err * k2 * dt, -10.0f, 10.0f);
  altEst += velEst * dt + err * k1 * dt;
}

// ======================= ALT HOLD =======================
float tiltComp(float thr) {
  float c = max(cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD), 0.7f);
  return 1000 + (thr - 1000) / c;
}

void altHoldStart(bool ground) {
  altHold = true;
  altLanded = ground;
  altLandSince = altStuckSince = 0;
  altTarget = altEst;
  altRefThr = hoverPoint();                          // hold zone = around the hover throttle
  altHoverBase = constrain(P.hover, 1150.0f, 1750.0f);   // feed-forward (reference)
  // Switched on in the air while (nearly) steady: the throttle the pilot is holding right now IS the hover throttle.
  // Use it for the hold zone and the feed-forward, so no stick change is needed and a stale learned hover value
  // cannot make it sink first and then overshoot (seen: rising past the target, then dropping almost to the ground).
  if (!ground && fabsf(velEst) < 0.5f && ch[RX_THR] > P.minThr) {
    altRefThr = ch[RX_THR];
    altHoverBase = constrain(thrToRef(ch[RX_THR]), 1150.0f, 1750.0f);
  }
  // hold zone = from the lower to the higher of the hover point and mid stick, +- deadband
  float mid = P.thrMid >= 1100 ? P.thrMid : altRefThr;
  altZoneLo = min(altRefThr, mid) - P.deadband;
  altZoneHi = max(altRefThr, mid) + P.deadband;
  altInteg = 0;
  altThrOut = ground ? 1000 : ch[RX_THR];
  setMsg(ground ? "ALT: STICK UP=TAKEOFF" : "ALT HOLD ON");
  BEEP_LONG;
  Serial.printf("ALT HOLD %s at %.2f m, hover point %.0f\n", ground ? "ground start" : "ON", altEst, altRefThr);
}

void altHoldStop(const char *why) {
  if (!altHold) return;
  altHold = false;
  exitBlendStart = millis(); exitBlendFrom = altThrOut;   // hand the throttle back smoothly
  setMsg(why);
  BEEP_TWO;
  Serial.printf("ALT HOLD OFF: %s\n", why);
}

float altHoldThrottle(float dt) {
  uint32_t ms = millis();
  float thr = ch[RX_THR];
  if (altLanded) {
    if (thr > altZoneHi) {                          // take off
      altLanded = false; altTarget = altEst; altInteg = 0; altLandSince = altStuckSince = 0;
      setMsg("TAKEOFF");
    } else { altThrOut = 1000; return 1000; }       // idle, no stabilisation
  } else {
    // stick fully down but not descending = sitting on the ground: bleed the integrator so the motors do not
    // stay at hover thrust, then call it landed after 0.8 s
    // Stick really down (< 1050) commands about -0.4 m/s, so "not descending" (< 0.1 m/s) there means the ground
    // holds it. (With stick < 1150 and < 0.2 m/s a slow descent in the air passed as "landed" and the motors were cut
    // at 0.6 m - seen on the old FC with the same logic.) Landed also needs the throttle clearly below hover, 1.5 s.
    bool down = thr < 1050;
    bool stuck = down && fabsf(velEst) < 0.1f;
    if (stuck) { if (!altStuckSince) altStuckSince = ms ? ms : 1; } else altStuckSince = 0;
    if (altStuckSince && ms - altStuckSince > 300) altInteg = max(-250.0f, altInteg - 300 * dt);
    bool still = stuck && altThrRef < altHoverBase - 50.0f;
    if (still) { if (!altLandSince) altLandSince = ms ? ms : 1; } else altLandSince = 0;
    if (altLandSince && ms - altLandSince > 1500) {
      altLanded = true; altLandSince = 0;
      setMsg("LANDED - DISARM");
      BEEP_TWO;
      Serial.println("ALT HOLD: landed");
      altThrOut = 1000;
      return 1000;
    }
  }
  float climb = 0;
  if (thr > altZoneHi)      climb =  constrain((thr - altZoneHi) / 300.0f, 0.0f, 1.0f) * P.maxClimb;
  else if (thr < altZoneLo) climb = -constrain((altZoneLo - thr) / 300.0f, 0.0f, 1.0f) * P.maxDesc;
  // the stick moves the TARGET at the commanded rate (kept within 0.5 m of the estimate)
  if (climb != 0) altTarget = constrain(altTarget + climb * dt, altEst - 0.5f, altEst + 0.5f);
  float velTgt = constrain(climb + P.altP * (altTarget - altEst), -P.maxDesc, P.maxClimb);
  if (ceilingOn()) {                                // never climb through the ceiling, push back down when above
    velTgt = ceilingVelLimit(velTgt);
    if (altTarget > P.ceilingM) altTarget = P.ceilingM;
  }
  float e = velTgt - velEst;
  altInteg = constrain(altInteg + P.velI * e * dt, -250.0f, 250.0f);
  altThrRef = tiltComp(altHoverBase + P.velP * e + altInteg);
  altThrOut = constrain(thrFromRef(altThrRef), P.minThr, 1850.0f);
  return altThrOut;
}

// CEILING (ALT HOLD setting "Ceiling m", 0 = off): height above the arming point, checked against the fused estimate
// (never the raw baro: its drift once pushed the old quad down). Works in SELF-LEVEL and ALT HOLD.
bool ceilingOn() { return P.ceilingM > 0.4f && altValid; }
float ceilingVelLimit(float vt) {                   // climb-rate limit near / above the ceiling
  float over = altEst - P.ceilingM;
  if (over > 0)     return min(vt, -min(P.altP * over, P.maxDesc));
  if (over > -0.3f) return min(vt, 0.0f);
  return vt;
}
float ceilingLimitThrottle(float thr) {             // SELF-LEVEL: only ever LOWERS the pilot's throttle
  if (!ceilingOn() || !inFlight || altEst < P.ceilingM - 0.3f) return thr;
  float lim = tiltComp(hoverPoint() + P.velP * (ceilingVelLimit(0) - velEst));
  return min(thr, max(lim, P.minThr));
}

// learn the hover throttle (reference units); saved at disarm
void hoverLearn(float dt) {
  if (!armed || !altValid || fabsf(roll) > 10 || fabsf(pitch) > 10) return;
  if (!altHold && inFlight && liftedOff && fabsf(velEst) < 0.3f && ch[RX_THR] > 1150)
    P.hover += (thrToRef(ch[RX_THR]) - P.hover) * dt / 4;
  // in ALT HOLD: whenever it is steady at its target (whatever the stick does), the throttle it needs is the hover
  // throttle. The feed-forward follows it and the integrator gives up the same amount (no bump), so the next hold
  // starts with the right value instead of winding the integrator up again.
  if (altHold && !altLanded && fabsf(velEst) < 0.15f && fabsf(altTarget - altEst) < 0.25f && altThrOut > P.minThr + 20) {
    float d = (altThrRef - P.hover) * dt / 6;
    P.hover += d;
    float nb = constrain(P.hover, 1150.0f, 1750.0f);
    altInteg = constrain(altInteg - (nb - altHoverBase), -250.0f, 250.0f);
    altHoverBase = nb;
  }
  P.hover = constrain(P.hover, 1100.0f, 1800.0f);
}

// ======================= RADIO =======================
volatile uint16_t rxCh[RX_NUM_CH] = {1000, 1500, 1500, 1500, 1000, 1000};
volatile uint32_t rxLastMs = 0;
volatile int8_t rxRssi = 0;
volatile uint16_t rxGood = 0;
volatile bool txMacKnown = false;
uint8_t txMac[6];
bool txPeerAdded = false;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ControlPacket)) return;
  ControlPacket p;
  memcpy(&p, data, sizeof p);
  if (p.magic != PKT_MAGIC || p.bindId != BIND_ID || crc8((const uint8_t *)&p, sizeof p - 1) != p.crc) return;
  if (txMacKnown && memcmp(info->src_addr, txMac, 6) != 0) return;   // locked to the first TX heard
  portENTER_CRITICAL(&rxMux);
  if (!txMacKnown) { memcpy(txMac, info->src_addr, 6); txMacKnown = true; }
  for (int i = 0; i < RX_NUM_CH; i++) rxCh[i] = constrain(p.ch[i], 1000, 2000);
  rxLastMs = millis();
  rxRssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  rxGood++;
  portEXIT_CRITICAL(&rxMux);
}

void radioBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_country_t ctry = {"EU", 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL};
  esp_wifi_set_country(&ctry);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(onRecv);
  esp_wifi_set_max_tx_power(80);
  Serial.printf("FC MAC %s, channel %d\n", WiFi.macAddress().c_str(), WIFI_CH);
}

// two-stage failsafe (see top)
void readRC() {
  uint32_t last;
  portENTER_CRITICAL(&rxMux);
  for (int i = 0; i < RX_NUM_CH; i++) ch[i] = rxCh[i];
  last = rxLastMs;
  portEXIT_CRITICAL(&rxMux);
  uint32_t now = millis(), age = last ? now - last : 0xFFFFFFFF;
  static bool wasFresh = true;
  rcFresh = last && age < RC_STALE_MS;
  rcValid = last && age < FAILSAFE_MS;
  if (!rcFresh) { ch[RX_ROLL] = ch[RX_PITCH] = ch[RX_YAW] = 1500; }
  bool canLand = altValid && !altLanded && inFlight;
  if (!rcValid && armed && (fsLanding || canLand)) {
    if (!fsLanding) {
      fsLanding = true; fsLandStart = now;
      if (!altHold) altHoldStart(false);
      Serial.println("FAILSAFE: radio lost - auto-land");
    }
    ch[RX_THR] = 1000;                              // alt hold: stick down = descend at maxDesc
    if (!altLanded && altValid && now - fsLandStart < FS_LAND_MAX_MS) rcValid = true;   // keep flying
  } else if (rcFresh && fsLanding) {
    fsLanding = false;
    Serial.println("FAILSAFE: radio back");
  }
  if (armed) {
    if (wasFresh && !rcFresh) rcGapCount++;
    if (!rcFresh && age != 0xFFFFFFFF && age > rcGapMaxMs) rcGapMaxMs = age;
  }
  wasFresh = rcFresh;
}

void telemetryTick(uint32_t ms) {
  static uint32_t last = 0, lqLast = 0, rsLast = 0;
  if (ms - lqLast >= 1000) {
    lqLast = ms;
    uint16_t n;
    portENTER_CRITICAL(&rxMux); n = rxGood; rxGood = 0; portEXIT_CRITICAL(&rxMux);
    rcLq = min<uint16_t>(100, n * 2);               // TX sends 50/s
  }
  if (ms - rsLast >= 20) {
    rsLast = ms;
    int8_t r = rxRssi;
    if (r) rcRssiF = (rcRssiF == 0) ? r : rcRssiF + 0.05f * (r - rcRssiF);
  }
  if (txMacKnown && !txPeerAdded) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMac, 6);
    peer.channel = WIFI_CH; peer.ifidx = WIFI_IF_STA; peer.encrypt = false;
    txPeerAdded = esp_now_add_peer(&peer) == ESP_OK || esp_now_is_peer_exist(txMac);
  }
  if (!txPeerAdded || ms - last < 50) return;      // 20 Hz
  last = ms;
  TelemetryPacket t;
  t.magic = TLM_MAGIC; t.bindId = BIND_ID;
  t.armed = armed; t.mode = mode; t.armBlock = armed ? AB_NONE : armBlock;
  t.roll10 = (int16_t)constrain(roll * 10, -1800.0f, 1800.0f);
  t.pitch10 = (int16_t)constrain(pitch * 10, -1800.0f, 1800.0f);
  t.vbat100 = battPresent ? (uint16_t)(vBat * 100) : 0;
  t.cells = cellsUsed;
  t.flags = (battPresent ? 1 : 0) | (battLow ? 2 : 0) | (altValid ? 4 : 0) | (altHold && altLanded ? 8 : 0) |
            (fsLanding ? 16 : 0) | (baroOK ? 0 : 32);
  t.alt100 = (int16_t)constrain(altEst * 100, -32000.0f, 32000.0f);
  t.rssi = (int8_t)lroundf(rcRssiF);
  t.lq = rcLq;
  t.crc = crc8((const uint8_t *)&t, sizeof t - 1);
  esp_now_send(txMac, (const uint8_t *)&t, sizeof t);
}

// ======================= FLIGHT STATS + STOP REASON (NVS) =======================
float flightTime = 0;
struct FlightStats { float maxAlt = 0, minVbat = 0, maxVib = 0, minRssi = 0; uint8_t minLq = 100; uint16_t maxCtrlUs = 0; };
FlightStats fst;                    // last / current flight (reset at arming), shown on the STATS page
char lastStop[64] = "none";

void statsTick() {
  if (!armed) return;
  if (altValid) fst.maxAlt = max(fst.maxAlt, altEst);
  if (inFlight) {
    if (battPresent) fst.minVbat = fst.minVbat == 0 ? vBat : min(fst.minVbat, vBat);
    fst.maxVib = max(fst.maxVib, vibration());
  }
  if (rcFresh) {
    if (rcRssiF != 0) fst.minRssi = fst.minRssi == 0 ? rcRssiF : min(fst.minRssi, rcRssiF);
    if (flightTime > 2) fst.minLq = min(fst.minLq, rcLq);
  }
  fst.maxCtrlUs = max(fst.maxCtrlUs, ctrlUs);
}

void saveStop(const char *why) {
  snprintf(lastStop, sizeof lastStop, "%s %.0fs gaps %u/%lums %+.0fdBm", why, flightTime, rcGapCount, (unsigned long)rcGapMaxMs, rcRssiF);
  if (prefs.begin("sfcb", false)) { prefs.putString("stop", lastStop); prefs.end(); }
}

void bootReport() {
  esp_reset_reason_t r = esp_reset_reason();
  const char *rs = (r == ESP_RST_BROWNOUT) ? "BROWNOUT" : (r == ESP_RST_PANIC) ? "CRASH" :
                   (r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT) ? "WATCHDOG" : nullptr;
  String last;
  if (prefs.begin("sfcb", false)) {
    last = prefs.getString("stop", "");
    if (rs) { last = String(rs) + " RESET"; prefs.putString("stop", last); }
    prefs.end();
  }
  Serial.printf("Reset reason %d. LAST STOP: %s\n", (int)r, last.length() ? last.c_str() : "none");
  if (last.length()) strlcpy(lastStop, last.c_str(), sizeof lastStop);
  if (last.length()) { char b[24]; snprintf(b, sizeof b, "LAST:%s", last.c_str()); setMsgFor(b, 10000); }
}

// ======================= ARMING / MODE / SAFETY =======================
void disarm(const char *why, bool needArmLow) {
  if (!armed) return;
  armed = false;
  if (needArmLow) armSeenLow = false;
  altHold = false;
  motorsOff();
  setMsg(why);
  if (needArmLow) BEEP_THREE; else BEEP_TWO;
  Serial.printf("DISARM: %s\n", why);
  saveStop(why);
  saveHover();
}

void updateArming() {
  bool armSw = ch[RX_ARM] > 1700;
  if (!rcValid || !imuOK) {
    armBlock = !rcValid ? AB_NOLINK : AB_IMU;
    if (armed) disarm(!rcValid ? (fsLanding ? "RADIO LOST-AUTOLAND" : "RADIO LOST") : "IMU FAIL", true);
    fsLanding = false;
    armSeenLow = false;
    return;
  }
  if (!armSw) {
    if (armed) disarm(inFlight && !(altHold && altLanded) ? "DISARM IN AIR" : "NORMAL", false);
    armSeenLow = true;
    armBlock = AB_NONE;
    return;
  }
  if (armed) return;
  static uint32_t lastDeny = 0;
  const char *why = nullptr;
  armBlock = AB_NONE;
  if (!armSeenLow)                               { why = "ARM SW LOW FIRST"; armBlock = AB_SWITCH; }
  else if (page == PG_SETUP)                     { why = "EXIT SETUP FIRST"; armBlock = AB_SETUP; }
  else if (levelCalRunning)                      { why = "CALIBRATING";      armBlock = AB_CAL; }
  else if (ch[RX_THR] > 1100)                    { why = "THROTTLE NOT LOW"; armBlock = AB_THROTTLE; }
  else if (fabsf(roll) > 25 || fabsf(pitch) > 25) { why = "TILT > 25 DEG";   armBlock = AB_TILT; }
  if (why) {
    if (millis() - lastDeny > 1000) { lastDeny = millis(); setMsg(why); BEEP_THREE; Serial.printf("ARM DENIED: %s\n", why); }
    return;
  }
  armed = true;
  liftedOff = false;
  resetPIDs();
  baroZero();                                     // altitude 0 = where you armed
  altHold = false; fsLanding = false;
  rcGapCount = 0; rcGapMaxMs = 0; flightTime = 0; fst = FlightStats();
  setMsg("ARMED");
  BEEP_LONG;
  Serial.println("ARMED");
}

void updateFlightState(float dt) {
  static uint32_t lastHigh = 0;
  uint32_t ms = millis();
  if (armed && ch[RX_THR] > 1100) lastHigh = ms;
  inFlight = armed && lastHigh && ms - lastHigh < 1500;
  if (armed && inFlight) flightTime += dt;
}

void updateMode() {
  bool want = armed && rcValid && ch[RX_CH6] >= 1300;   // MODE = ALT HOLD
  want |= fsLanding;
  static bool wantPrev = false;
  bool flipIn = want && !wantPrev;                      // switch moved into ALT (or armed with it there)
  wantPrev = want;
  if (!armed) altHold = false;
  else if (altHold && !want) altHoldStop("SELF-LEVEL");
  else if (altHold && !altValid) altHoldStop("ALT LOST-MANUAL");
  else if (!altHold && want) {                          // level-checked: arming zeroes the baro, so altValid can be
    if (altValid) altHoldStart(!inFlight || ch[RX_THR] < 1150);   // false for a moment and a one-shot try never came back
    else if (flipIn) { setMsg("NO BARO"); BEEP_THREE; }
  }
  mode = altHold ? MODE_ALT : MODE_SELF;
}

void crashCheck(float dt, uint32_t ms) {
  static uint32_t angSince = 0, vibSince = 0;
  static float vibBase = 5;
  if (!armed) { angSince = vibSince = 0; return; }
  if (inFlight) vibBase += (vibration() - vibBase) * dt / 5;
  bool angBad = fabsf(roll) > CRASH_ANGLE_DEG || fabsf(pitch) > CRASH_ANGLE_DEG;
  if (angBad) { if (!angSince) angSince = ms ? ms : 1; } else angSince = 0;
  if (angSince && ms - angSince > CRASH_ANGLE_MS) { disarm("CRASH: ANGLE", true); return; }
  float vf = vibrationFast();
  bool vibBad = inFlight && vf > CRASH_VIB_ABS && vf > CRASH_VIB_RATIO * max(vibBase, 3.0f);
  if (vibBad) { if (!vibSince) vibSince = ms ? ms : 1; } else vibSince = 0;
  if (vibSince && ms - vibSince > CRASH_VIB_MS) disarm("CRASH: VIBRATION", true);
}

// ======================= STICK COMMANDS (disarmed) =======================
void levelCalTick() {
  static uint32_t holdSince = 0, calStart = 0, firstStart = 0;
  static float sr = 0, sp = 0; static int n = 0;
  uint32_t ms = millis();
  if (!levelCalRunning) {
    if (!levelCalRequest) {
      bool cmd = !armed && rcValid && page != PG_SETUP && ch[RX_THR] < 1100 && ch[RX_YAW] < 1300 && ch[RX_PITCH] < 1300 &&
                 fabsf(stick(ch[RX_ROLL])) < 0.3f;
      if (!cmd) { holdSince = 0; return; }
      if (!holdSince) holdSince = ms ? ms : 1;
      if (ms - holdSince < 2000) return;
    }
    levelCalRequest = false;
    levelCalRunning = true; calStart = ms; sr = sp = 0; n = 0; holdSince = 0; firstStart = ms;
    setMsg("LEVEL CAL: STILL"); BEEP_SHORT;
    return;
  }
  if (ms - firstStart > 8000) {                              // never still for 1 s: give up, arming is free again
    levelCalRunning = false;
    setMsg("LEVEL CAL FAILED"); BEEP_THREE;
    Serial.printf("LEVEL CAL failed: gyro %.1f %.1f %.1f  |a| %.3f (ref %.3f)\n", gyro[0], gyro[1], gyro[2], accMag, accRef);
    return;
  }
  if (!imuStill()) { calStart = ms; sr = sp = 0; n = 0; }    // moved: start over
  sr += accRollRaw; sp += accPitchRaw; n++;
  if (ms - calStart < 1000) return;
  levelCalRunning = false;
  P.rollTrim = constrain(sr / n, -TRIM_LIMIT, TRIM_LIMIT);
  P.pitchTrim = constrain(sp / n, -TRIM_LIMIT, TRIM_LIMIT);
  saveTrims();
  setMsg("LEVEL CAL DONE"); BEEP_LONG;
  Serial.printf("LEVEL CAL: rollTrim %.2f pitchTrim %.2f\n", P.rollTrim, P.pitchTrim);
}

void trimStickTick() {
  static uint32_t next = 0;
  static bool changed = false;
  uint32_t now = millis();
  bool cond = !armed && rcValid && page != PG_SETUP && ch[RX_THR] > 1600 && fabsf(stick(ch[RX_YAW])) < 0.3f;
  float sr = stick(ch[RX_ROLL]), sp = stick(ch[RX_PITCH]);   // + = right / forward
  int dr = 0, dp = 0;
  if (cond) { dr = sr > 0.5f ? 1 : (sr < -0.5f ? -1 : 0); dp = sp > 0.5f ? 1 : (sp < -0.5f ? -1 : 0); }
  if (dr || dp) {
    if ((int32_t)(now - next) >= 0) {
      P.rollTrim = constrain(P.rollTrim + 0.1f * dr, -TRIM_LIMIT, TRIM_LIMIT);
      P.pitchTrim = constrain(P.pitchTrim + 0.1f * dp, -TRIM_LIMIT, TRIM_LIMIT);
      changed = true; next = now + 300;
      BEEP_SHORT;
    }
  } else if (changed) {
    changed = false; next = 0;
    saveTrims();
    setMsg("TRIM SAVED");
    Serial.printf("TRIM roll %.1f pitch %.1f\n", P.rollTrim, P.pitchTrim);
  }
}

// ======================= CONTROL =======================
void runControl(float dt) {
  float thr = ch[RX_THR];
  if (mode == MODE_ALT) thr = altHoldThrottle(dt);
  else if (exitBlendStart && millis() - exitBlendStart < EXIT_BLEND_MS)
    thr = exitBlendFrom + (thr - exitBlendFrom) * (millis() - exitBlendStart) / (float)EXIT_BLEND_MS;

  if (mode == MODE_SELF) thr = ceilingLimitThrottle(thr);
  float sR = stick(ch[RX_ROLL]), sP = stick(ch[RX_PITCH]), sY = -stick(ch[RX_YAW]);   // yaw right = CW = -gz
  // RC smoothing: 2 PT1 stages (the radio is only 50 Hz)
  static float r1 = 0, r2 = 0, p1 = 0, p2 = 0, y1 = 0, y2 = 0;
  if (P.rcSmoothHz > 0) {
    float k = 1 - expf(-2 * PI * P.rcSmoothHz * 1.414f * dt);
    r1 += k * (sR - r1); r2 += k * (r1 - r2); sR = r2;
    p1 += k * (sP - p1); p2 += k * (p1 - p2); sP = p2;
    y1 += k * (sY - y1); y2 += k * (y1 - y2); sY = y2;
  }

  // self-level: angle -> rate setpoint
  rollTgt = expoCurve(sR, P.angleExpo) * P.angleMax;
  pitchTgt = expoCurve(sP, P.angleExpo) * P.angleMax;
  float rollSet = pidLvRoll.update(rollTgt - roll, roll, dt);
  float pitchSet = pidLvPitch.update(pitchTgt - pitch, pitch, dt);
  float ye = expoCurve(sY, P.yawExpo);
  float yawSet = P.yawRateC * ye + (P.yawRateMax - P.yawRateC) * ye * fabsf(ye);

  if (thr > hoverPoint()) liftedOff = true;        // I terms only after the throttle passed the hover point once
  float oR = 0, oP = 0, oY = 0;
  bool lowThr = thr < 1100;
  if (lowThr) resetPIDs();                         // on the ground: idle, no stabilisation
  else {
    // I-term relax (roll/pitch): fast setpoint changes do not wind up I
    static float spR = 0, spP = 0, thrLp = 1000;
    float kr = 1 - expf(-2 * PI * 15.0f * dt);
    spR += kr * (rollSet - spR); spP += kr * (pitchSet - spP);
    float relR = 1, relP = 1;
    if (P.itermRelax > 0) {
      relR = max(0.0f, 1 - fabsf(rollSet - spR) / P.itermRelax);
      relP = max(0.0f, 1 - fabsf(pitchSet - spP) / P.itermRelax);
    }
    // anti-gravity (roll/pitch): I builds faster on quick throttle changes
    thrLp += (1 - expf(-2 * PI * 5.0f * dt)) * (thr - thrLp);
    float ag = 1;
    if (P.antiGrav > 0) ag = min(1 + P.antiGrav * fabsf(thr - thrLp) / 100, 1 + P.antiGrav);
    oR = pidRoll.update(rollSet - gyro[0], gyro[0], dt, liftedOff, relR * ag);
    oP = pidPitch.update(pitchSet - gyro[1], gyro[1], dt, liftedOff, relP * ag);
    oY = pidYaw.update(yawSet - gyro[2], gyro[2], dt, liftedOff);
  }

  // mixer: roll+ = right side down, pitch+ = nose down, yaw+ = CCW. CW props (M1, M4 with props in) get +yaw.
  float ym = P.propsIn > 0.5f ? 1 : -1;
  float idle = P.motorIdle;
  float base = idle + (constrain(thr, 1000.0f, 2000.0f) - 1000) * (2000 - idle) * 0.001f;
  float m[4] = {
    base - oR + oP + ym * oY,    // M1 rear right
    base - oR - oP - ym * oY,    // M2 front right
    base + oR + oP - ym * oY,    // M3 rear left
    base + oR - oP + ym * oY,    // M4 front left
  };
  for (int i = 0; i < 4; i++) outUs[i] = constrain((int)m[i], (int)idle, 2000);
}

// ======================= DJI O4 OSD (MSP DisplayPort, "canvas mode") =======================
// UART1 115200 8N1: FC TX GPIO21 -> air unit RX, FC RX GPIO10 <- air unit TX, plus GND. Power the air unit from the pack.
// The FC pretends to be Betaflight ("BTFL"), pushes MSP_STATUS (armed bit = leaves low-power mode), battery, and every
// 100 ms one OSD frame: heartbeat, clear, strings, draw. It also answers the usual requests from the air unit.
// MSPv1 frame: '$' 'M' '>' size cmd payload xor(size, cmd, payload). Requests from the air unit come as '$' 'M' '<'.
#define MSP_API_VERSION    1
#define MSP_FC_VARIANT     2
#define MSP_FC_VERSION     3
#define MSP_NAME          10
#define MSP_STATUS       101
#define MSP_RC           105
#define MSP_ATTITUDE     108
#define MSP_ALTITUDE     109
#define MSP_ANALOG       110
#define MSP_BATTERY_STATE 130
#define MSP_STATUS_EX    150
#define MSP_DISPLAYPORT  182
#define MSP_SET_OSD_CANVAS 188
#define MSP_OSD_CANVAS   189
bool osdStarted = false;

void mspSend(uint8_t cmd, const uint8_t *p, uint8_t n, char dir = '>') {
  uint8_t h[5] = {'$', 'M', (uint8_t)dir, n, cmd};
  uint8_t c = n ^ cmd;
  for (uint8_t i = 0; i < n; i++) c ^= p[i];
  Serial1.write(h, 5);
  if (n) Serial1.write(p, n);
  Serial1.write(c);
}

void dpCmd(uint8_t sub) { mspSend(MSP_DISPLAYPORT, &sub, 1); }
void dpText(int row, int col, const char *s, bool blink = false) {
  uint8_t b[4 + 30];
  int n = strlen(s);
  if (n > 30) n = 30;
  if (row < 0 || col < 0) return;
  b[0] = 3; b[1] = row; b[2] = col; b[3] = blink ? 0x40 : 0;            // attr: bit6 = blink, bank 0
  for (int i = 0; i < n; i++) { char ch = s[i]; b[4 + i] = (ch >= 'a' && ch <= 'z') ? ch - 32 : ch; }   // font: upper case
  mspSend(MSP_DISPLAYPORT, b, 4 + n);
}

void mspReply(uint8_t cmd) {           // answers (and the periodic pushes) - Betaflight layouts
  uint8_t b[32];
  memset(b, 0, sizeof b);
  switch (cmd) {
    case MSP_API_VERSION: b[0] = 0; b[1] = 1; b[2] = 46; mspSend(cmd, b, 3); break;
    case MSP_FC_VARIANT:  memcpy(b, "BTFL", 4); mspSend(cmd, b, 4); break;
    case MSP_FC_VERSION:  b[0] = 4; b[1] = 5; b[2] = 0; mspSend(cmd, b, 3); break;
    case MSP_NAME:        mspSend(cmd, (const uint8_t *)"SIMPLEFC", 8); break;
    case MSP_STATUS:
    case MSP_STATUS_EX: {                                   // 22 bytes, armed = flight mode flags bit 0
      uint16_t ct = loopUs;
      b[0] = ct; b[1] = ct >> 8;
      b[4] = 1 | (baroOK ? 2 : 0);                          // sensors: acc (+ baro)
      b[6] = armed ? 1 : 0;                                 // flightModeFlags (u32 LE) bit 0 = ARM
      b[13] = (uint8_t)LOOP_US; b[14] = LOOP_US >> 8;       // gyro cycle time
      b[16] = 0;                                            // arming disable flags count / flags = 0
      mspSend(cmd, b, 22);
      break;
    }
    case MSP_BATTERY_STATE: {                               // 11 bytes
      uint16_t mv = battPresent ? (uint16_t)(vBat * 100) : 0;
      b[0] = cellsUsed; b[3] = (uint8_t)constrain(vBat * 10, 0.0f, 255.0f);
      b[8] = battLow ? 1 : 0; b[9] = mv; b[10] = mv >> 8;
      mspSend(cmd, b, 11);
      break;
    }
    case MSP_ANALOG: {                                      // 9 bytes
      uint16_t mv = battPresent ? (uint16_t)(vBat * 100) : 0;
      uint16_t rssi = (uint16_t)(rcLq * 1023 / 100);
      b[0] = (uint8_t)constrain(vBat * 10, 0.0f, 255.0f); b[3] = rssi; b[4] = rssi >> 8; b[7] = mv; b[8] = mv >> 8;
      mspSend(cmd, b, 9);
      break;
    }
    case MSP_ATTITUDE: {
      int16_t r = (int16_t)(roll * 10), p = (int16_t)(pitch * 10);
      b[0] = r; b[1] = r >> 8; b[2] = p; b[3] = p >> 8;
      mspSend(cmd, b, 6);
      break;
    }
    case MSP_ALTITUDE: {
      int32_t cm = (int32_t)(altEst * 100); int16_t v = (int16_t)(velEst * 100);
      memcpy(b, &cm, 4); b[4] = v; b[5] = v >> 8;
      mspSend(cmd, b, 6);
      break;
    }
    case MSP_RC: {                                          // AETR order like Betaflight
      const uint8_t ord[6] = {RX_ROLL, RX_PITCH, RX_THR, RX_YAW, RX_ARM, RX_CH6};
      for (int i = 0; i < 6; i++) { b[2 * i] = ch[ord[i]]; b[2 * i + 1] = ch[ord[i]] >> 8; }
      mspSend(cmd, b, 12);
      break;
    }
    case MSP_OSD_CANVAS: b[0] = (uint8_t)P.osdCols; b[1] = (uint8_t)P.osdRows; mspSend(cmd, b, 2); break;
    default: mspSend(cmd, nullptr, 0, '!'); break;          // not supported
  }
}

void mspPoll() {                        // parse requests from the air unit
  static uint8_t st = 0, n = 0, cmd = 0, i = 0, c = 0, p[16];
  while (Serial1.available()) {
    uint8_t x = Serial1.read();
    switch (st) {
      case 0: st = (x == '$') ? 1 : 0; break;
      case 1: st = (x == 'M') ? 2 : 0; break;
      case 2: st = (x == '<') ? 3 : 0; break;
      case 3: n = x; c = x; i = 0; st = (n <= sizeof p) ? 4 : 0; break;
      case 4: cmd = x; c ^= x; st = n ? 5 : 6; break;
      case 5: p[i++] = x; c ^= x; if (i >= n) st = 6; break;
      case 6:
        st = 0;
        if (c != x) break;
        if (cmd == MSP_SET_OSD_CANVAS && n >= 2 && p[0] >= 30 && p[0] <= 60 && p[1] >= 16 && p[1] <= 22) {
          P.osdCols = p[0]; P.osdRows = p[1];               // the air unit tells its canvas size
        } else mspReply(cmd);
        break;
    }
  }
}

void osdFrame(uint32_t ms) {
  const int C = (int)P.osdCols, R = (int)P.osdRows;
  char b[40];
  bool blink = (ms / 400) & 1;
  dpCmd(0);                                                // heartbeat
  dpCmd(2);                                                // clear
  // top line: mode | flight time | link
  const char *md = fsLanding ? "AUTOLAND" : (mode == MODE_ALT ? (altLanded ? "ALT GND" : "ALT HOLD") : "SELF");
  dpText(1, 2, md, fsLanding && blink);
  if (armed) { snprintf(b, sizeof b, "%u:%02u", (unsigned)flightTime / 60, (unsigned)flightTime % 60); dpText(1, C / 2 - 2, b); }
  else dpText(1, C / 2 - 4, "DISARMED");
  if (rcValid) snprintf(b, sizeof b, "%dDB %u%%", (int)lroundf(rcRssiF), rcLq); else snprintf(b, sizeof b, "NO LINK");
  dpText(1, C - 2 - (int)strlen(b), b, !rcValid && blink);
  // bottom line: battery | altitude
  if (battPresent) {
    snprintf(b, sizeof b, "%.1fV %.2f", vBat, cellsUsed ? vBat / cellsUsed : 0);
    dpText(R - 2, 2, b, battLow && blink);
  } else dpText(R - 2, 2, "NO BATT");
  if (altValid) { snprintf(b, sizeof b, "ALT %.1fM", altEst); dpText(R - 2, C - 2 - (int)strlen(b), b); }
  // centre: horizon (9 dashes, tilt with roll, move with pitch) + aircraft mark
  int cx = C / 2, cy = R / 2;
  float tr = tanf(constrain(roll, -60.0f, 60.0f) * DEG_TO_RAD) * 0.6f;   // characters are ~1.5x taller than wide
  int off = (int)lroundf(constrain(pitch, -30.0f, 30.0f) / 6.0f);
  for (int i = -4; i <= 4; i++) {
    int x = cx + i * 2, y = cy + off + (int)lroundf(-i * 2 * tr);
    if (y >= 3 && y <= R - 4 && i != 0) dpText(y, x, "-");
  }
  dpText(cy, cx - 1, "-+-");
  // warnings / messages (row R-4, centred)
  const char *w = nullptr;
  if ((int32_t)(msgUntil - ms) > 0) w = msgText;
  else if (!rcValid && armed) w = "FAILSAFE";
  else if (battLow) w = "LOW BATTERY";
  else if (!armed && armBlock != AB_NONE) w = armBlockText(armBlock);
  if (w) dpText(R - 4, cx - (int)strlen(w) / 2, w, true);
  dpCmd(4);                                                // draw
  mspReply(MSP_FC_VARIANT);                                // pushes, as PX4 does
  mspReply(MSP_STATUS);
  mspReply(MSP_BATTERY_STATE);
}

void osdTick(uint32_t ms) {
  static uint32_t last = 0;
  if (P.osdOn < 0.5f) return;
  if (!osdStarted) {
    Serial1.setTxBufferSize(1024);                         // writes never wait for the UART
    Serial1.begin(115200, SERIAL_8N1, PIN_OSD_RX, PIN_OSD_TX);
    osdStarted = true;
  }
  mspPoll();
  if (ms - last >= 100) { last = ms; osdFrame(ms); }
}

// ======================= FLIGHT RECORDER =======================
#define REC_N  1600
#define REC_MS 20
RecSample recBuf[REC_N];
uint16_t recHead = 0, recCount = 0;
uint32_t recStart = 0, recLast = 0;
bool recWasArmed = false, recFlew = false, recFsOk = false;

int16_t c16(float v) { return (int16_t)constrain(v, -32000.0f, 32000.0f); }

void recSave() {                    // only arms that actually flew
  if (!recFsOk || recCount < 100 || !recFlew) return;
  File f = LittleFS.open("/rec.bin", "w");
  if (!f) return;
  uint16_t n = recCount, sz = sizeof(RecSample), start = recCount < REC_N ? 0 : recHead;
  f.write((const uint8_t *)&n, 2); f.write((const uint8_t *)&sz, 2);
  for (uint16_t k = 0; k < n; k++) f.write((const uint8_t *)&recBuf[(start + k) % REC_N], sizeof(RecSample));
  f.close();
  Serial.printf("REC saved: %u samples\n", n);
}

void recTick(uint32_t ms) {
  if (armed && !recWasArmed) { recHead = recCount = 0; recStart = ms; recLast = 0; recFlew = false; }
  if (armed && inFlight) recFlew = true;
  if (!armed && recWasArmed) recSave();
  recWasArmed = armed;
  if (!armed || ms - recLast < REC_MS) return;
  recLast = ms;
  RecSample &s = recBuf[recHead];
  s.t10 = (ms - recStart) / 10;
  s.roll10 = c16(roll * 10); s.pitch10 = c16(pitch * 10); s.rollT10 = c16(rollTgt * 10); s.pitchT10 = c16(pitchTgt * 10);
  s.gR = c16(gyro[0]); s.gP = c16(gyro[1]); s.gY = c16(gyro[2]);
  s.thr = ch[RX_THR]; s.rcR = ch[RX_ROLL]; s.rcP = ch[RX_PITCH]; s.rcY = ch[RX_YAW];
  for (int i = 0; i < 4; i++) s.m[i] = outUs[i];
  s.vbat100 = (uint16_t)(vBat * 100); s.ctrlUs = ctrlUs;
  s.alt100 = c16(altEst * 100); s.altT100 = c16(altTarget * 100); s.vel100 = c16(velEst * 100);
  s.flags = (mode == MODE_ALT ? 1 : 0) | (altValid ? 2 : 0) | (altLanded ? 4 : 0) | (fsLanding ? 8 : 0);
  s.vib10 = (uint8_t)constrain(vibration() * 10, 0.0f, 255.0f);
  s.rssi = (int8_t)lroundf(rcRssiF); s.lq = rcLq;
  s.accUp100 = c16(accUp * 100); s.accCorr100 = c16(accCorr * 100); s.altI10 = c16(altInteg * 10);
  recHead = (recHead + 1) % REC_N;
  if (recCount < REC_N) recCount++;
}

void recDump() {
  Serial.setTxTimeoutMs(200);       // this time wait for the PC (disarmed only)
  bool fromFlash = false;
  if (recCount == 0 && recFsOk && LittleFS.exists("/rec.bin")) {
    File f = LittleFS.open("/rec.bin", "r");
    uint16_t n = 0, sz = 0;
    if (f && f.read((uint8_t *)&n, 2) == 2 && f.read((uint8_t *)&sz, 2) == 2 && sz == sizeof(RecSample)) {
      n = min<uint16_t>(n, REC_N);
      uint16_t got = 0;
      while (got < n && f.read((uint8_t *)&recBuf[got], sizeof(RecSample)) == sizeof(RecSample)) got++;
      recCount = got; recHead = got % REC_N; fromFlash = true;
    }
    if (f) f.close();
  }
  Serial.printf("REC BEGIN %u samples, %u ms, from %s\n", recCount, REC_MS, fromFlash ? "flash" : "RAM");
  Serial.println("t,roll,pitch,rollT,pitchT,gR,gP,gY,thr,rcR,rcP,rcY,m1,m2,m3,m4,vbat,ctrl_us,alt,altT,vel,mode,altValid,landed,fsLand,vib,rssi,lq,accUp,accCorr,altI");
  uint16_t start = recCount < REC_N ? 0 : recHead;
  for (uint16_t k = 0; k < recCount; k++) {
    const RecSample &s = recBuf[(start + k) % REC_N];
    Serial.printf("%.2f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%u,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.1f,%d,%u,%.2f,%.2f,%.1f\n",
                  s.t10 / 100.0f, s.roll10 / 10.0f, s.pitch10 / 10.0f, s.rollT10 / 10.0f, s.pitchT10 / 10.0f,
                  s.gR, s.gP, s.gY, s.thr, s.rcR, s.rcP, s.rcY, s.m[0], s.m[1], s.m[2], s.m[3],
                  s.vbat100 / 100.0f, s.ctrlUs, s.alt100 / 100.0f, s.altT100 / 100.0f, s.vel100 / 100.0f,
                  s.flags & 1, (s.flags >> 1) & 1, (s.flags >> 2) & 1, (s.flags >> 3) & 1, s.vib10 / 10.0f, s.rssi, s.lq,
                  s.accUp100 / 100.0f, s.accCorr100 / 100.0f, s.altI10 / 10.0f);
    if ((k & 31) == 31) delay(2);
  }
  Serial.println("REC END");
  Serial.setTxTimeoutMs(0);
}

// ======================= SERIAL COMMANDS =======================
void printParams() {
  for (int g = 0; g < G_COUNT; g++) {
    Serial.printf("[%s]\n", GROUP_NAME[g]);
    for (int i = 0; i < N_PARAMS; i++)
      if (PARAMS[i].grp == g) Serial.printf("  %-13s = %g\n", PARAMS[i].name, P.*(PARAMS[i].m));
  }
}

// motor test over serial: one motor (or all) at a low duty for 2 s. PROPS OFF.
#define SERIAL_MOTOR_US 1150
int8_t serialMotor = -1;            // -1 off, 0..3 = M1..M4, 4 = all
uint32_t serialMotorUntil = 0;
void serialMotorTick(uint32_t ms) {   // called after motorsOff() while disarmed
  if (serialMotor < 0) return;
  if ((int32_t)(ms - serialMotorUntil) >= 0 || (rcValid && ch[RX_ARM] >= 1700)) { serialMotor = -1; Serial.println("motor test end"); return; }
  for (int i = 0; i < 4; i++) if (serialMotor == 4 || serialMotor == i) { outUs[i] = SERIAL_MOTOR_US; writeUs(i, SERIAL_MOTOR_US); }
}

void printStatus() {
  Serial.printf("%s %s R%+.1f P%+.1f | %.2fV %uS%s | ", armed ? "ARMED" : "SAFE", mode == MODE_ALT ? "ALT" : "SELF", roll, pitch, vBat, cellsUsed, battLow ? " LOW" : "");
  if (rcValid) Serial.printf("%+.0fdBm %u%% THR %u ARM %u MODE %u", rcRssiF, rcLq, ch[RX_THR], ch[RX_ARM], ch[RX_CH6]); else Serial.print("NO RC");
  Serial.printf(" | baro %s %+.2fm | hover %.0f vib %.1f loop %uus | %s\n", baroOK ? (baroIsMs ? "MS5611" : "BMP280") : "MISSING", altEst, hoverPoint(), vibration(), loopUs,
                armBlockText(armBlock));
}

void printHelp() {
  Serial.println("help | ? settings | name=value | save | defaults | cal level calibration (flat!)");
  Serial.println("m1 m2 m3 m4 mall spin 2 s PROPS OFF | stop | st status | S status on/off | stats last flight | D dump flight CSV");
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  if (!*s) return;
  if (!strcmp(s, "?"))        { printParams(); return; }
  if (!strcmp(s, "help"))     { printHelp(); return; }
  if (!strcmp(s, "st"))       { printStatus(); return; }
  if (!strcmp(s, "cal"))      { levelCalRequest = true; return; }
  if (!strcmp(s, "stop"))     { serialMotor = -1; Serial.println("motors off"); return; }
  if (!strcmp(s, "stats")) {
    Serial.printf("last flight: %.0f s, max alt %.1f m, min V %.2f, max vib %.0f, worst RSSI %.0f, LQ %u%%, gaps %u (max %lu ms), ctrl %u us\n", flightTime,
                  fst.maxAlt, fst.minVbat, fst.maxVib, fst.minRssi, fst.minLq, rcGapCount, (unsigned long)rcGapMaxMs, fst.maxCtrlUs);
    Serial.printf("stop reason: %s\n", lastStop);
    return;
  }
  if (s[0] == 'm' && (!strcmp(s, "mall") || (s[1] >= '1' && s[1] <= '4' && !s[2]))) {
    if (rcValid && ch[RX_ARM] >= 1700) { Serial.println("ARM button off first"); return; }
    serialMotor = !strcmp(s, "mall") ? 4 : s[1] - '1';
    serialMotorUntil = millis() + 2000;
    Serial.printf("spinning %s for 2 s, PROPS OFF! (stop = stop now)\n", s);
    return;
  }
  if (!strcmp(s, "D") || !strcmp(s, "d")) { recDump(); return; }
  if (!strcmp(s, "S") || !strcmp(s, "s")) { statusPrint = !statusPrint; return; }
  if (!strcmp(s, "save"))     { Serial.println(paramsSave() ? "saved" : "SAVE FAILED"); return; }
  if (!strcmp(s, "defaults")) { paramsDefault(); filtersSetup(); Serial.println("defaults loaded (not saved)"); return; }
  char *eq = strchr(s, '=');
  if (eq) {
    *eq = 0;
    for (int i = 0; i < N_PARAMS; i++) {
      if (strcasecmp(s, PARAMS[i].name)) continue;
      float v = atof(eq + 1);
      if (!(v >= PARAMS[i].mn && v <= PARAMS[i].mx)) { Serial.printf("%s: range %g..%g\n", PARAMS[i].name, PARAMS[i].mn, PARAMS[i].mx); return; }
      P.*(PARAMS[i].m) = v;
      filtersSetup();
      Serial.printf("%s = %g (type 'save' to keep)\n", PARAMS[i].name, v);
      return;
    }
  }
  Serial.println("unknown command, type help");
}

void serialTick() {
  static char line[48]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[n] = 0; n = 0;
      if (armed) { Serial.println("disarm first"); continue; }
      handleLine(line);
    } else if (n < sizeof line - 1) line[n++] = c;
  }
}

#if USE_OLED
// ======================= OLED SETUP MENU (disarmed, driven by the TX sticks) =======================
// Open: BOOT button until SETUP shows, or throttle LOW + yaw full LEFT + pitch full FORWARD for 1 s.
//   pitch fwd/back = cursor up/down      roll right/left = value +/- (hold = repeat; throttle HIGH = x10)
//   yaw right = enter group / run action yaw left = back (from the main list = leave setup)
// Changes act at once; "*" in the title = not saved yet -> SAVE. Arming is refused while SETUP is open.
enum { ACT_LEVELCAL = G_COUNT, ACT_MOTORS, ACT_ESCCAL, ACT_SAVE, ACT_DEFAULTS, ACT_EXIT, MAIN_COUNT };
#define MENU_MOTORS 100             // menuGrp value of the motor test screen
#define MOTOR_TEST_US 1150         // motor test pulse (props OFF!)
int8_t mtSel = 0;                  // motor test: 0..3 = M1..M4, 4 = ALL
bool mtSpin = false;
uint32_t escCalArmedAt = 0;
int8_t menuGrp = -1;               // -1 = main list, else the open group
int menuSel = 0, menuTop = 0;
bool dirty = false, menuReady = false;
uint32_t defaultsArmedAt = 0;
uint8_t grpIdx[N_PARAMS_MAX];      // PARAMS indices of the open group
int grpN = 0;

struct StickKey {                  // stick as a key: -1/0/+1 events with auto-repeat (faster the longer it is held)
  int last = 0; uint32_t next = 0, since = 0;
  int poll(float v) {
    int d = v > 0.4f ? 1 : (v < -0.4f ? -1 : 0), ev = 0;   // 0.4: yaw only reaches ~0.57
    uint32_t now = millis();
    if (d && d != last) { ev = d; next = now + 400; since = now; }
    else if (d && (int32_t)(now - next) >= 0) { ev = d; next = now + (now - since > 2000 ? 60 : 150); }
    last = d;
    return ev;
  }
};
StickKey keyV, keyH, keyY;

void openGroup(int g) {
  grpN = 0;
  for (int i = 0; i < N_PARAMS; i++) if (PARAMS[i].grp == g) grpIdx[grpN++] = i;
  menuGrp = g; menuSel = menuTop = 0;
}

void enterSetup() { page = PG_SETUP; menuGrp = -1; menuSel = menuTop = 0; menuReady = false; }
void leaveSetup() { page = PG_STATUS; menuGrp = -1; mtSpin = false; if (dirty) setMsg("NOT SAVED (*)"); }

void doAction(int a) {
  if (a < G_COUNT) { openGroup(a); return; }
  switch (a) {
    case ACT_LEVELCAL: levelCalRequest = true; break;   // runs in levelCalTick; arming blocked meanwhile
    case ACT_SAVE:     if (paramsSave()) { dirty = false; setMsg("SAVED"); BEEP_LONG; } else setMsg("SAVE FAILED"); break;
    case ACT_DEFAULTS:
      if (defaultsArmedAt && millis() - defaultsArmedAt < 3000) {
        paramsDefault(); filtersSetup(); dirty = true; defaultsArmedAt = 0; setMsg("DEFAULTS - SAVE?"); BEEP_TWO;
      } else { defaultsArmedAt = millis(); setMsg("AGAIN = DEFAULTS"); }
      break;
    case ACT_MOTORS:   menuGrp = MENU_MOTORS; mtSel = 0; mtSpin = false; break;
    case ACT_ESCCAL:                                   // brushed: no ESCs, nothing to calibrate
      setMsg("BRUSHED: NO ESC CAL"); BEEP_THREE;
      break;
      if (escCalArmedAt && millis() - escCalArmedAt < 3000) {
        escCalArmedAt = 0;
        if (prefs.begin("sfcb", false)) { prefs.putUChar("esccal", 1); prefs.end(); }
        setMsgFor("PROPS OFF! REPOWER", 15000); BEEP_LONG;
        Serial.println("ESC CAL armed: props off, unplug the battery and USB, then power up again (battery).");
      } else { escCalArmedAt = millis(); setMsg("PROPS OFF? AGAIN=YES"); }
      break;
    case ACT_EXIT:     leaveSetup(); break;
  }
}

void menuTick() {
  // stick command to open setup (disarmed, not open yet)
  static uint32_t openHold = 0;
  uint32_t ms = millis();
  if (page != PG_SETUP) {
    bool cmd = !armed && rcValid && ch[RX_THR] < 1100 && ch[RX_YAW] < 1300 && ch[RX_PITCH] > 1700;
    if (!cmd) openHold = 0;
    else if (!openHold) openHold = ms ? ms : 1;
    else if (ms - openHold > 1000) { openHold = 0; enterSetup(); BEEP_SHORT; }
    return;
  }
  if (armed || !rcValid) return;
  float sP = stick(ch[RX_PITCH]), sR = stick(ch[RX_ROLL]), sY = stick(ch[RX_YAW]);
  if (!menuReady) {                // wait for centred sticks after opening (the open gesture is still held)
    if (fabsf(sP) < 0.3f && fabsf(sR) < 0.3f && fabsf(sY) < 0.3f) { menuReady = true; keyV = keyH = keyY = StickKey(); }
    return;
  }
  int v = keyV.poll(sP), h = keyH.poll(sR), y = keyY.poll(sY);
  if (y > 0) keyY.next = ms + 100000;            // yaw: one event per push, no repeat
  if (y < 0) keyY.next = ms + 100000;
  int n = menuGrp < 0 ? MAIN_COUNT : grpN;
  if (menuGrp != MENU_MOTORS) {
    if (v > 0) menuSel = (menuSel + n - 1) % n;  // pitch forward = up
    if (v < 0) menuSel = (menuSel + 1) % n;
  }
  if (menuGrp == MENU_MOTORS) {                  // motor test: pitch = pick, hold roll RIGHT = spin, yaw left = back
    if (v > 0) mtSel = (mtSel + 4) % 5;
    if (v < 0) mtSel = (mtSel + 1) % 5;
    mtSpin = sR > 0.4f && ch[RX_ARM] < 1700;
    if (y < 0) { mtSpin = false; menuGrp = -1; menuSel = ACT_MOTORS; }
    return;
  }
  if (menuGrp < 0) {
    if (y > 0 || h > 0) doAction(menuSel);
    else if (y < 0) leaveSetup();
  } else {
    if (y < 0) { menuSel = menuGrp; menuTop = 0; menuGrp = -1; return; }   // back to the main list
    if (h) {
      const ParamDef &d = PARAMS[grpIdx[menuSel]];
      float &val = P.*(d.m);
      float st = d.step * (ch[RX_THR] > 1600 ? 10 : 1);
      val = constrain(roundf((val + h * st) / d.step) * d.step, d.mn, d.mx);
      filtersSetup();
      dirty = true;
    }
  }
}

// ======================= OLED PAGES (disarmed only; pushed in small slices so the loop keeps its rate) =======================
const char *armBlockText(uint8_t b) {
  switch (b) {
    case AB_THROTTLE: return "THROTTLE HIGH"; case AB_SWITCH: return "ARM SW LOW 1ST";
    case AB_IMU: return "IMU FAIL"; case AB_TILT: return "TILTED";
    case AB_NOLINK: return "NO RADIO"; case AB_CAL: return "CALIBRATING"; case AB_SETUP: return "IN SETUP";
    default: return "READY";
  }
}

// 128 x 32: four text rows, baselines at y = 7, 15, 23, 31
void drawFooter(const char *idle) {  // bottom row: a message (inverted) wins over the normal text
  if ((int32_t)(msgUntil - millis()) > 0) {
    oled->setDrawColor(0); oled->drawBox(0, 24, 128, 8); oled->setDrawColor(1);
    oled->drawBox(0, 24, 128, 8); oled->setDrawColor(0); oled->drawStr(2, 31, msgText); oled->setDrawColor(1);
  } else if (idle) oled->drawStr(0, 31, idle);
}

void drawStatus() {
  char b[32];
  // row 1: ARMED (inverted) / SAFE + mode
  if (armed) { oled->drawBox(0, 0, 28, 8); oled->setDrawColor(0); oled->drawStr(2, 7, "ARMED"); oled->setDrawColor(1); }
  else oled->drawStr(2, 7, "SAFE");
  snprintf(b, sizeof b, "%s%s", mode == MODE_ALT ? (altLanded ? "ALT-GND" : "ALT HOLD") :
           (ch[RX_CH6] >= 1300 && rcValid ? "SELF>ALT" : "SELF"), dirty ? " *" : "");
  oled->drawStr(32, 7, b);
  // row 2: attitude
  snprintf(b, sizeof b, "R%+5.1f P%+5.1f", roll, pitch);
  oled->drawStr(0, 15, b);
  // row 3: battery + link
  bool blink = battLow && ((millis() / 400) & 1);
  if (blink) { oled->drawBox(0, 16, 28, 8); oled->setDrawColor(0); }
  if (battPresent) snprintf(b, sizeof b, "%.2fV", vBat); else snprintf(b, sizeof b, "NOBAT");
  oled->drawStr(1, 23, b);
  oled->setDrawColor(1);
  if (rcValid) snprintf(b, sizeof b, "%.0fdB %u%%", rcRssiF, rcLq); else snprintf(b, sizeof b, "NO LINK");
  oled->drawStr(32, 23, b);

  // right: artificial horizon (circle), the line tilts with roll and rises when the nose goes down
  const int cx = 112, cy = 15, r = 15;
  oled->drawCircle(cx, cy, r);
  float rr = roll * DEG_TO_RAD;
  int off = (int)(constrain(pitch / 45.0f, -1.0f, 1.0f) * r * 0.5f);
  int dx = (int)(cosf(rr) * (r - 3)), dy = (int)(sinf(rr) * (r - 3));
  int hy = cy - off;
  oled->drawLine(cx - dx, hy + dy, cx + dx, hy - dy);
  oled->drawHLine(cx - 6, cy, 3); oled->drawHLine(cx + 4, cy, 3); oled->drawPixel(cx, cy);   // fixed aircraft marker
  // throttle: thin vertical bar left of the circle
  int th = (int)(constrain((ch[RX_THR] - 1000) / 1000.0f, 0.0f, 1.0f) * 30);
  oled->drawFrame(92, 0, 3, 32);
  oled->drawVLine(93, 31 - th, th);

  // row 4: message, then warnings, then why it will not arm / READY, in flight the altitude
  if ((int32_t)(msgUntil - millis()) > 0) { drawFooter(nullptr); return; }
  const char *w = !rcValid ? "NO RC SIGNAL" : !imuOK ? "IMU FAULT" : battLow ? "LOW BATTERY" :
                  armed ? nullptr : (levelCalRunning ? "LEVEL CAL: STILL" : armBlockText(armBlock));
  if (w) oled->drawStr(0, 31, w);
  else {
    if (baroOK) snprintf(b, sizeof b, "ALT %+.2fm", altEst); else snprintf(b, sizeof b, "NO BARO");
    oled->drawStr(0, 31, b);
  }
}

void drawSensors() {
  char b[32];
  snprintf(b, sizeof b, "R%+6.1f P%+6.1f vib%4.1f", roll, pitch, vibration());                  oled->drawStr(0, 7, b);
  snprintf(b, sizeof b, "acc R%+5.1f P%+5.1f", accRollRaw - P.rollTrim, accPitchRaw - P.pitchTrim); oled->drawStr(0, 15, b);
  snprintf(b, sizeof b, "gyro %+4.0f %+4.0f %+4.0f", gyro[0], gyro[1], gyro[2]);                 oled->drawStr(0, 23, b);
  if (baroOK) snprintf(b, sizeof b, "baro %+.2f v%+.2f %uus", baroAlt, velEst, loopUs); else snprintf(b, sizeof b, "baro: not found %uus", loopUs);
  drawFooter(b);
}

void drawMotorTest() {
  static const char *const NM[5] = {"M1 rear right", "M2 front right", "M3 rear left", "M4 front left", "ALL"};
  oled->drawStr(0, 7, ch[RX_ARM] >= 1700 ? "MOTORS: ARM BUTTON OFF" : "MOTORS PROPS OFF R=spin");
  int top = constrain(mtSel - 1, 0, 2);          // three rows of the five
  for (int i = top; i < top + 3; i++) {
    int y = 8 + (i - top) * 8;
    if (i == mtSel) { oled->drawBox(0, y, 128, 8); oled->setDrawColor(0); }
    oled->drawStr(2, y + 7, NM[i]);
    if (i == mtSel && mtSpin) oled->drawStr(102, y + 7, "SPIN");
    oled->setDrawColor(1);
  }
}

void drawMenu() {
  char b[32];
  if (menuGrp == MENU_MOTORS) { drawMotorTest(); return; }
  const char *title = menuGrp < 0 ? "SETUP" : GROUP_NAME[menuGrp];
  if (menuGrp == G_BATT) snprintf(b, sizeof b, "%s %.2fV%s", title, vBat, dirty ? " *" : "");   // live V to match a meter
  else snprintf(b, sizeof b, "%s%s", title, dirty ? " *" : "");
  oled->drawStr(0, 7, b);
  int n = menuGrp < 0 ? MAIN_COUNT : grpN;
  const int ROWS = 3;
  if (menuSel < menuTop) menuTop = menuSel;
  if (menuSel >= menuTop + ROWS) menuTop = menuSel - ROWS + 1;
  for (int r = 0; r < ROWS && menuTop + r < n; r++) {
    int it = menuTop + r, y = 8 + r * 8;
    bool sel = it == menuSel;
    if (sel) { oled->drawBox(0, y, 125, 8); oled->setDrawColor(0); }
    if (menuGrp < 0) {
      static const char *const ACT[] = {"LEVEL CALIBRATE", "MOTOR TEST", "(ESC CAL: n/a)", "SAVE", "DEFAULTS (x2)", "EXIT"};
      oled->drawStr(2, y + 7, it < G_COUNT ? GROUP_NAME[it] : ACT[it - G_COUNT]);
      oled->drawStr(118, y + 7, ">");
    } else {
      const ParamDef &d = PARAMS[grpIdx[it]];
      oled->drawStr(2, y + 7, d.label);
      snprintf(b, sizeof b, "%.*f", d.dec, P.*(d.m));
      oled->drawStr(123 - oled->getStrWidth(b), y + 7, b);
    }
    oled->setDrawColor(1);
  }
  if (n > ROWS) { int bh = max(24 * ROWS / n, 3), by = 8 + (24 - bh) * menuTop / (n - ROWS); oled->drawBox(126, by, 2, bh); }
  drawFooter(nullptr);
}

void drawStats() {
  char b[40];
  snprintf(b, sizeof b, "%.0fs alt %.1fm Vmin %.2f", flightTime, fst.maxAlt, fst.minVbat);        oled->drawStr(0, 7, b);
  snprintf(b, sizeof b, "vib %.0f RSSI %.0f LQ %u%%", fst.maxVib, fst.minRssi, fst.minLq);        oled->drawStr(0, 15, b);
  snprintf(b, sizeof b, "gaps %u/%lums ctrl %uus", rcGapCount, (unsigned long)rcGapMaxMs, fst.maxCtrlUs); oled->drawStr(0, 23, b);
  drawFooter(lastStop);                                   // stop reason (first 25 characters; full text on serial at boot)
}

void drawTrend() {                  // last recorded flight: altitude (line) + hold target (dots). Battery: see STATS
  char b[32];
  if (recCount < 2) { oled->drawStr(0, 15, "TREND: no flight"); oled->drawStr(0, 23, "since power-up"); return; }
  uint16_t start = recCount < REC_N ? 0 : recHead;
  float aMin = 1e9, aMax = -1e9;
  for (uint16_t k = 0; k < recCount; k++) {
    const RecSample &s = recBuf[(start + k) % REC_N];
    aMin = min(aMin, min(s.alt100, s.altT100) / 100.0f); aMax = max(aMax, max(s.alt100, s.altT100) / 100.0f);
  }
  if (aMax - aMin < 0.5f) aMax = aMin + 0.5f;
  snprintf(b, sizeof b, "ALT %.1f..%.1fm %.0fs", aMin, aMax, recCount * REC_MS / 1000.0f);
  oled->drawStr(0, 7, b);
  for (int x = 0; x < 128; x++) {
    const RecSample &s = recBuf[(start + (uint32_t)x * (recCount - 1) / 127) % REC_N];
    int ya = 31 - (int)((s.alt100 / 100.0f - aMin) / (aMax - aMin) * 22);
    int yt = 31 - (int)((s.altT100 / 100.0f - aMin) / (aMax - aMin) * 22);
    oled->drawPixel(x, ya);
    if ((s.flags & 1) && (x & 1)) oled->drawPixel(x, yt);   // target only while ALT HOLD was on
  }
}

void drawPage() {
  oled->clearBuffer();
  oled->setFont(u8g2_font_5x8_tf);
  if (page == PG_SETUP && !armed) drawMenu();
  else if (page == PG_SENSORS && !armed) drawSensors();
  else if (page == PG_STATS && !armed) drawStats();
  else if (page == PG_TREND && !armed) drawTrend();
  else drawStatus();
}

void buttonTick() {                // BOOT button: next page (disarmed)
  static bool last = HIGH; static uint32_t changed = 0;
  bool r = digitalRead(PIN_BTN);
  uint32_t ms = millis();
  if (r != last && ms - changed > 30) {
    changed = ms; last = r;
    if (r == LOW && !armed) {
      if (page == PG_SETUP) leaveSetup();
      else if (++page == PG_SETUP) enterSetup();
    }
  }
}

void oledTick(uint32_t ms) {
  static uint32_t lastDraw = 0;
  static int8_t slice = -1;          // -1 = idle, 0..7 = next half tile row to send (32 px = 4 tile rows)
  static bool frozen = false;
  if (armed) {                       // frozen in flight (drawing costs loop time): one last frame at arming
    if (!frozen && slice < 0) { page = PG_STATUS; drawPage(); slice = 0; frozen = true; }
  } else frozen = false;
  if (slice >= 0) {
    oled->updateDisplayArea((slice & 1) * 8, slice >> 1, 8, 1);   // 64 x 8 px, ~1.5 ms
    if (++slice >= 8) slice = -1;
    return;
  }
  if (!armed && ms - lastDraw >= (page == PG_SETUP ? 80u : 200u)) { lastDraw = ms; drawPage(); slice = 0; }
}
void splash(const char *l1, const char *l2) {
  oled->clearBuffer();
  oled->setFont(u8g2_font_6x10_tf);
  oled->drawStr(0, 9, FW_VERSION);
  oled->drawStr(0, 20, l1);
  oled->drawStr(0, 31, l2);
  oled->sendBuffer();
}

#else  // !USE_OLED
const char *armBlockText(uint8_t b) {
  switch (b) {
    case AB_THROTTLE: return "THROTTLE HIGH"; case AB_SWITCH: return "ARM SW LOW 1ST";
    case AB_IMU: return "IMU FAIL"; case AB_TILT: return "TILTED";
    case AB_NOLINK: return "NO RADIO"; case AB_CAL: return "CALIBRATING"; case AB_SETUP: return "IN SETUP";
    default: return "READY";
  }
}
void splash(const char *l1, const char *l2) { Serial.printf("%s %s\n", l1, l2); }
#endif

// ======================= SETUP / LOOP =======================
void setup() {
  motorsBegin();
  // ESC CALIBRATION (armed from SETUP > ESC CALIBRATE): the ESCs power up with the FC from the battery, so they must
  // see MAX right away, then MIN. The flag is cleared FIRST, so it runs only once.
  bool escCal = false;
  if (prefs.begin("sfcb", false)) { escCal = prefs.getUChar("esccal", 0) != 0; if (escCal) prefs.putUChar("esccal", 0); prefs.end(); }
  escCal = false;                    // brushed: never drive the motors to MAX at power-up
  motorsOff();                       // 0 % duty at once
  ledcAttach(PIN_BUZZ, BUZZ_HZ, 8); buzz(false);
  pinMode(PIN_BTN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);          // USB prints never block the loop
  delay(300);
  Serial.println(FW_VERSION);
  paramsLoad();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
#if USE_OLED
  oled = &oledDev;
  oled->setBusClock(I2C_HZ);
  oled->begin();
#endif
  if (escCal) {
    Serial.println("ESC CAL: MAX 5 s, then MIN 5 s. PROPS OFF!");
    splash("ESC CAL: MAX", "wait for ESC beeps");
    buzz(true); delay(200); buzz(false);
    delay(4500);
    motorsOff();
    splash("ESC CAL: MIN", "wait for confirm");
    delay(5000);
    splash("ESC CAL DONE", "starting...");
    delay(800);
  }
  splash("IMU...", "");
  if (!mpuInit()) {
    splash("MPU6050 NOT FOUND", "check wiring");
    uint32_t lastTell = 0;
    while (true) {                   // stopped, motors off: keep telling, the PC may connect later
      buzz((millis() / 100) & 1);
      if (millis() - lastTell > 2000) { lastTell = millis(); Serial.println("STOPPED: MPU6050 not found - check VCC 3V3, GND, SDA GPIO6, SCL GPIO7"); }
      delay(10);
    }
  }
  filtersSetup();
  baroOK = baroInit();
  splash("Calibrating gyro", "keep still...");
  if (!calibrateGyro()) { splash("GYRO CAL FAILED", "moving? restart"); Serial.println("Gyro cal failed"); }
  radioBegin();
  bootReport();
  recFsOk = LittleFS.begin(true);
  Serial.printf("Recorder storage %s. Hover point %.0f us. Trims R%.1f P%.1f\n",
                recFsOk ? "ok" : "N/A", hoverPoint(), P.rollTrim, P.pitchTrim);
  printHelp();
  BEEP_SHORT;
}

void loop() {
  static uint32_t nextTick = micros(), lastTick = micros(), lastPrint = 0;
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += LOOP_US;
  if ((int32_t)(now - nextTick) > (int32_t)(4 * LOOP_US)) nextTick = now + LOOP_US;
  float dt = constrain((now - lastTick) * 1e-6f, 0.0005f, 0.01f);
  lastTick = now;
  uint32_t ms = millis();

  // ---- flight-critical ----
  readRC();
  readIMU(dt);
  baroTick(ms);
  altEstTick(dt);
  batteryTick(ms);
  updateFlightState(dt);
  statsTick();
  updateArming();
  crashCheck(dt, ms);
  updateMode();
  if (armed) {
    runControl(dt);
    for (int i = 0; i < 4; i++) writeUs(i, outUs[i]);
    hoverLearn(dt);
  } else {
    resetPIDs();
    motorsOff();
    levelCalTick();
    trimStickTick();
#if USE_OLED
    menuTick();
    buttonTick();
    if (page == PG_SETUP && menuGrp == MENU_MOTORS && mtSpin && rcValid) {   // motor test (props OFF)
      for (int i = 0; i < 4; i++) if (mtSel == 4 || mtSel == i) { outUs[i] = MOTOR_TEST_US; writeUs(i, MOTOR_TEST_US); }
    } else mtSpin = false;
#endif
    serialMotorTick(ms);
  }
  ctrlUs = micros() - now;
  if (ctrlUs > ctrlMaxUs) ctrlMaxUs = ctrlUs;

  // ---- the rest ----
  recTick(ms);
  osdTick(ms);                       // DJI O4 OSD (MSP DisplayPort)
  buzzerTick(ms);
  telemetryTick(ms);
#if USE_OLED
  oledTick(ms);
#endif
  if (!armed) serialTick();

  if (Serial && statusPrint && ms - lastPrint >= 200) {   // only with a PC connected
    lastPrint = ms;
    Serial.printf("loop %u/%uus ctrl %u/%uus | %s %s R%+6.1f P%+6.1f vib %4.1f | %.2fV | ",
                  loopUs, loopMaxUs, ctrlUs, ctrlMaxUs, armed ? "ARMED" : "SAFE ", mode == MODE_ALT ? "ALT " : "SELF",
                  roll, pitch, vibration(), vBat);
    loopMaxUs = ctrlMaxUs = 0;
    if (rcValid) Serial.printf("%u %u %u %u %u %u", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]); else Serial.print("NO RC");
    Serial.printf(" | %+.0fdBm %u%% | baro %+.2f est %+.2f v%+.2f", rcRssiF, rcLq, baroAlt, altEst, velEst);
    if (altHold) Serial.printf(" | HOLD tgt %+.2f thr %.0f%s", altTarget, altThrOut, altLanded ? " LANDED" : "");
    Serial.printf(" | hover %.0f comp x%.3f\n", hoverPoint(), batComp());
  }
  loopUs = micros() - now;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}
