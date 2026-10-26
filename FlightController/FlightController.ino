/*
 * ESP32-C3 Zero flight controller (ESP-NOW) with a TRANSMITTER-DRIVEN SETUP MENU
 * MPU6050 (IMU) + ESP-NOW link to the ESP32-C3 Zero transmitter + 1.3" OLED (SH1106 / SSD1306, 128x64, I2C)
 *
 * RADIO: receives 6 channels from the ESP32-C3 transmitter over ESP-NOW (no receiver/PPM needed) and sends
 *   telemetry back (armed, flight mode, why arming was refused, attitude) so the TX OLED can show it.
 *   PKT_MAGIC / BIND_ID / WIFI_CH must match the transmitter sketch.
 *
 * FLIGHT MODE (stick commands, disarmed, throttle LOW, pitch centred), hold 1 s:
 *   yaw LEFT + roll LEFT  -> SELF STAB (self-level)      1 beep
 *   yaw LEFT + roll RIGHT -> ACRO                         2 beeps
 *   The mode is saved to flash immediately; it is also in the SETUP menu (FLIGHT MODE).
 *
 * CH6 = 3-position switch on the TX: POS1 DISARM, POS2 ARM, POS3 ARM + extra. The TX converts the switch into
 *   the ARM channel (CH5), so arming safety checks here are unchanged. DIRECTIONS > "CH6 POS3 =" picks the extra:
 *   OFF, AIRMODE (pos 3 = acro airmode on, pos 2 = off; overrides ACRO AIRMODE) or BEEPER (buzzer beeps).
 *
 * Vehicle type (QUAD / PLANE) and every tuning value are RUNTIME settings. Change them
 * with the transmitter sticks while DISARMED, then SAVE to flash.
 *
 * ---------------- HOW TO USE THE SETUP MENU (disarmed only) ----------------
 *   Open it : press the BOOT button until the SETUP page shows, OR on the sticks hold
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
 * BATTERY VOLTAGE (BATTERY menu): wire  pack+ --100k-- GPIO4 --10k-- GND  with a 100 nF capacitor from GPIO4 to GND.
 *   (GPIO4, not GPIO2: GPIO2 on this board is shorted to the crystal can. Motor 4 moved to GPIO8.)
 *   That is ratio 11 (the default) and is good for up to about 25 V (6S is too much: use 200k/10k and set ratio 21).
 *   VOLT RATIO  : (R1+R2)/R2. The BATTERY menu title shows the live voltage: nudge the ratio until it matches a multimeter.
 *   CELLS       : 0 = detect from the pack voltage at power-up (connect a charged pack before powering the FC).
 *   WARN V/CELL : "LOW BATTERY" flashes on the STATUS page when the cell voltage stays below this for 2 s.
 *   (old note, when the divider was on GPIO2) GPIO2 is a boot strapping pin: with the 10k to GND it only matters if you enter download mode by holding BOOT
 *   while pressing reset. Normal uploads from the IDE are not affected. If that ever fails, unplug the divider wire.
 *   The ADC is not very accurate (a few %), so calibrate against a multimeter. No battery = shows NO BATTERY.
 *
 * BUZZER (BATTERY menu: BUZZER on/off, TEST BUZZER): active buzzer on GPIO20 through an NPN transistor
 *   (GPIO20 -> 1k -> base, emitter -> GND, buzzer between collector and +5 V). Beeps: ready, armed, disarmed,
 *   arm refused, low battery, lost signal while armed (find-me beeps), IMU fault. Patterns are in the ALERTS block.
 *
 * OLED pages (BOOT button): STATUS, RC bars, RATE CURVE, FLIGHT STATS, TREND (estimate minus accelerometer,
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
 * ---------------- WIRING (ESP32-C3 Zero) ----------------
 *   MPU6050 SDA + OLED SDA -> GPIO6,  SCL -> GPIO7   (3.3 V, shared bus)
 *   (GPIO5 is free: the radio link is ESP-NOW, no PPM receiver)
 *   Output 1..4            -> GPIO0, GPIO1, GPIO3, GPIO8   (output 4 moved from GPIO4 to GPIO8)
 *       QUAD, BETAFLIGHT motor order (top view, front up), props in:
 *                                              M4 FL (CW)  M2 FR (CCW)
 *                                              M3 RL (CCW) M1 RR (CW)
 *       PLANE: 1 = aileron, 2 = elevator, 3 = throttle (ESC), 4 = rudder
 *   Page button            -> GPIO9 (on-board BOOT button)
 *
 * ---------------- RC CHANNELS ----------------
 *   The transmitter sends THR, ROLL, PITCH, YAW, AUX1, AUX2. They are remapped on reception to the
 *   internal order: ch[0] roll  ch[1] pitch  ch[2] throttle  ch[3] yaw  ch[4] ARM (> 1700)  ch[5] spare
 *
 * Libraries: "U8g2" by oliver. (Preferences ships with the ESP32 core.)
 * Arduino IDE: Board "ESP32C3 Dev Module", "USB CDC On Boot" = Enabled.
 *
 * !!! TEST WITH PROPS OFF. Verify directions and tune gains before flying.
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stddef.h>
#include <esp_system.h>             // esp_reset_reason(): tells a brown-out restart from a normal power-up
#include <LittleFS.h>               // flight recorder: the last flight is kept in flash after disarm
#include <VL53L1X.h>               // "VL53L1X" by Pololu (Library Manager)

// ======================= FIXED CONFIG (compile time) =======================
#define PIN_SDA  6
#define PIN_SCL  7
#define PIN_BTN  9
#define PIN_BUZZ 20        // active buzzer through an NPN transistor (GPIO20 = UART RX, free because Serial uses USB)
#define USE_BUZZER  1      // set to 0 if no buzzer is fitted
// GPIO2 is NOT used on this board: its pad is shorted to the crystal can (GND). Never configure it as an output.
// The battery divider moved to GPIO4 (one of the ADC1 pins) and motor/output 4 moved from GPIO4 to GPIO8.
#define PIN_VBAT 4         // battery voltage divider (ADC1)
const uint8_t OUT_PIN[4] = {0, 1, 3, 8};   // M1 RR, M2 FR, M3 RL, M4 FL (GPIO8: strapping pin, fine as an output)

//#define OLED_IS_SSD1306          // uncomment if your 1.3" OLED is an SSD1306 (most are SH1106)

#define FW_VERSION "v8 espnow"     // shown on the start-up screen and on serial: confirms which build is running

// ---- ESP-NOW link (MUST match the transmitter) ----
#define PKT_MAGIC   0xC3F1
#define BIND_ID     42
#define WIFI_CH     13         // away from phone hotspots / routers (1, 6, 11). MUST match the TX
#define RC_STALE_MS 250            // no packet for this long -> sticks centred, keep flying (see readRC)
#define FAILSAFE_MS 1000           // no packet for this long -> radio lost (auto-land in ALT/POS HOLD, else disarm)
#define FS_LAND_MAX_MS 15000       // failsafe auto-land: disarm anyway after this long without radio
struct RfGate { bool use; uint32_t goodSince, badSince; };   // altitude rangefinder hysteresis (ALT HOLD section)

#define LOOP_HZ   400   // 2.5 ms per loop: margin for alt hold + optical flow (filters follow LOOP_HZ)
#define LOOP_US   (1000000UL / LOOP_HZ)
#define I2C_HZ    400000UL    // I2C bus speed. 1 MHz read the IMU faster but risks bus errors with motor noise -> IMU-fail disarm
#define USE_MTF   1           // 1 = MicoAir MTF-01 optical flow + rangefinder fitted (enables POS HOLD on CH6 POS3)
                              // 0 = no MTF: alt hold uses the VL53L1X (facing down) + barometer
#define USE_AVOID 0           // 1 = VL53L1X faces forward/back for obstacle avoidance; 0 = it faces DOWN for alt hold

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
enum UiScreen : uint8_t { SCR_MENU, SCR_TXTEST, SCR_OUTTEST, SCR_ESCCAL };
enum ItemType : uint8_t { IT_FLOAT, IT_ENUM, IT_SUB, IT_ACTION };
enum { SUB_MAIN, SUB_RC, SUB_LEVEL, SUB_GYRO, SUB_DIR, SUB_TRIM, SUB_BATT, SUB_FILT, SUB_ALT, SUB_FLOW, SUB_AVOID, SUB_POS, SUB_COUNT };
enum { ACT_SAVE, ACT_DEFAULTS, ACT_TXTEST, ACT_OUTTEST, ACT_EXIT, ACT_LEVELCAL, ACT_TRIMCLEAR, ACT_ALERTTEST,
       ACT_ESCCAL, ACT_ESCBOOT, ACT_FLOWCAL };

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

// ---- ESP-NOW packets (keep identical to the transmitter sketch) ----
enum { RX_THR, RX_ROLL, RX_PITCH, RX_YAW, RX_AUX1, RX_AUX2, RX_NUM_CH };   // order the TX sends

struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  seq;
  uint16_t ch[RX_NUM_CH];          // 1000..2000 us
  uint8_t  crc;
};

enum ArmBlock : uint8_t { AB_NONE, AB_THROTTLE, AB_SWITCH, AB_IMU, AB_TILT, AB_NOLINK, AB_SETUP };

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  armed;
  uint8_t  mode;                   // 0 = self-level, 1 = acro
  uint8_t  armBlock;               // ArmBlock reason while not armed
  int16_t  roll10;                 // deg * 10
  int16_t  pitch10;
  uint16_t loopUs;
  uint16_t vbat100;                // pack voltage * 100 (0 = no battery)
  uint8_t  cells;                  // cell count in use (0 = unknown)
  uint8_t  battFlags;              // bit0 = battery present, bit1 = LOW battery
  int8_t   rssi;                   // dBm the FC receives the TX with (smoothed), 0 = unknown
  uint8_t  lq;                     // % of the TX's 50 packets/s the FC received in the last second
  uint8_t  crc;
};

Cfg  cfg;
uint8_t flightMode = 1;            // 0 = ACRO, 1 = SELF STAB. Set by stick command or menu, stored as "fmode"

// CH6 is a 3-position switch on the TX: 1000 = DISARM, 1500 = ARM, 2000 = ARM + extra.
// (The TX turns the switch into the ARM channel CH5 itself; the FC only uses CH6 for the extra.)
enum { CH6X_OFF = 0, CH6X_AIRMODE = 1, CH6X_BEEPER = 2, CH6X_ALTHOLD = 3, CH6X_POSHOLD = 4 };
uint8_t ch6Extra = CH6X_AIRMODE;   // what position 3 adds, stored as "ch6x"

// Filters adjustable in SETUP > FILTERS (stored as "filt"; the gyro and D-term low-pass live in the tune)
struct FiltCfg {
  float notchHz;                   // gyro notch centre, 0 = off (motor/frame vibration peak)
  float notchQ;                    // notch width: higher = narrower
  float accLpfHz;                  // accelerometer low-pass (self-level correction)
  float levelDLpfHz;               // self-level PID D-term low-pass
  float baroLpfHz;                 // barometer altitude smoothing
  float tofLpfHz;                  // laser range smoothing, 0 = off
};
FiltCfg filt = {0, GYRO_NOTCH_Q, ACC_LPF_HZ, LEVEL_D_LPF_HZ, 1.5f, 5.0f};   // same as defaultFilt()

// Altitude hold + ceiling, adjustable in SETUP > ALT HOLD (stored as "alth")
struct AltCfg {
  float altP;                      // m/s of climb per m of altitude error
  float velP;                      // throttle us per m/s of vertical speed error
  float velI;                      // throttle us per (m/s * s): learns the exact hover throttle
  float maxClimb;                  // m/s with the throttle stick fully up
  float maxDesc;                   // m/s with the throttle stick fully down
  float deadband;                  // us around the hover stick position that means "hold"
  float minThr;                    // lowest throttle alt hold may command (us, stick scale)
  float ceilingM;                  // max height above the arming point (m), 0 = off. SELF STAB + ALT HOLD only
  float tofMaxM;                   // use the rangefinder below this height (m)
};
AltCfg altCfg = {1.0f, 120.0f, 40.0f, 1.0f, 0.7f, 80.0f, 1150.0f, 0.0f, 2.0f};   // same as defaultAlt()
// Betaflight-style control features (SETUP > GYRO PID), stored under their own keys so the tuning is untouched
float  rcSmoothHz    = 20.0f;      // RC SMOOTH: stick low-pass between the 50 Hz radio packets (Hz), 0 = off, "rcsm"
float  itermRelaxDps = 40.0f;      // ITERM RELAX: I builds up less while the setpoint moves fast (deg/s), 0 = off, "irel"
float  antiGravGain  = 3.0f;       // ANTI GRAVITY: I builds up faster on quick throttle changes, 0 = off, "antig"
float  altHover = 1450;            // hover throttle learned while flying SELF STAB (used by the ceiling), "hover"

// Optical flow + rangefinder module (MicoAir MTF-01/02, MSP v2), adjustable in SETUP > OPT FLOW (stored as "flow")
enum { RF_AUTO = 0, RF_VL53 = 1, RF_MTF = 2 };
struct FlowCfg {
  float   scale;                   // flow raw units -> rad: rate = raw / (scale * dt). Calibrate (see FLOW page)
  uint8_t align;                   // 0/1/2/3 = sensor rotated 0/90/180/270 deg clockwise (top view)
  uint8_t flip;                    // 1 = mirror the sideways axis (sensor mounted upside down / mirrored)
  uint8_t compSign;                // gyro compensation: 0 = subtract gyro, 1 = add gyro, 2 = off
  uint8_t rfSource;                // rangefinder used by the altitude estimate: AUTO (MTF if alive) / VL53L1X / MTF
};
FlowCfg flowCfg = {10.5f, 0, 0, 0, RF_AUTO};   // same as defaultFlow()

// Position hold with the optical flow (CH6 POS3 = POS HOLD), adjustable in SETUP > POS HOLD (stored as "posh")
struct PosCfg {
  float posP;                      // m/s of correction per m of position error
  float velP;                      // deg of tilt per m/s of speed error
  float velI;                      // deg of tilt per (m/s * s) - removes steady drift (wind, CG)
  float maxSpeed;                  // m/s with full roll/pitch stick
  float maxTilt;                   // deg: strongest tilt position hold may use
  float minQuality;                // flow quality (0..255) needed to hold position
};
PosCfg posCfg = {1.5f, 8.0f, 3.0f, 2.0f, 20.0f, 60.0f};   // same as defaultPos()

// Obstacle avoidance with a FORWARD-facing VL53L1X, adjustable in SETUP > AVOIDANCE (stored as "avoid")
enum { VL53_DOWN = 0, VL53_FORWARD = 1, VL53_BACKWARD = 2 };   // BACKWARD = faces the FC board's rear
struct AvoidCfg {
  uint8_t role;                    // VL53_DOWN = altitude rangefinder, VL53_FORWARD = obstacle sensor
  uint8_t enabled;                 // 1 = limit/push back in SELF STAB + ALT HOLD (warning only when 0)
  float   avoidDist;               // m: start limiting forward tilt below this distance
  float   stopDist;                // m: no forward tilt at all; closer than this it tilts back
  float   maxBack;                 // deg: strongest automatic back tilt
};
AvoidCfg avoidCfg = {VL53_FORWARD, 0, 2.0f, 0.6f, 8.0f};   // same as defaultAvoid() - OFF until checked

Tune T;                           // working copy for the CURRENT vehicle (menu edits this)
Tune tuneStore[2];                 // stored copies, index = vehicle
bool dirty = false;                // unsaved changes

void defaultFilt(FiltCfg &f) {
  f.notchHz = 0;                   // off until you find the vibration frequency
  f.notchQ = GYRO_NOTCH_Q;
  f.accLpfHz = ACC_LPF_HZ;
  f.levelDLpfHz = LEVEL_D_LPF_HZ;
  f.baroLpfHz = 1.5f;
  f.tofLpfHz = 5.0f;
}

void sanitizeFilt(FiltCfg &f) {    // also rejects NaN from a bad save
  if (!(f.notchHz >= 0 && f.notchHz <= 240))        f.notchHz = 0;
  if (!(f.notchQ >= 0.5f && f.notchQ <= 10))        f.notchQ = GYRO_NOTCH_Q;
  if (!(f.accLpfHz >= 1 && f.accLpfHz <= 100))      f.accLpfHz = ACC_LPF_HZ;
  if (!(f.levelDLpfHz >= 2 && f.levelDLpfHz <= 100)) f.levelDLpfHz = LEVEL_D_LPF_HZ;
  if (!(f.baroLpfHz >= 0.1f && f.baroLpfHz <= 10))  f.baroLpfHz = 1.5f;
  if (!(f.tofLpfHz >= 0 && f.tofLpfHz <= 10))       f.tofLpfHz = 5.0f;
}

void defaultAlt(AltCfg &a) {
  a.altP = 1.0f;  a.velP = 120.0f;  a.velI = 40.0f;
  a.maxClimb = 1.0f;  a.maxDesc = 0.7f;
  a.deadband = 80.0f;  a.minThr = 1150.0f;
  a.ceilingM = 0.0f;  a.tofMaxM = 2.0f;
}

void sanitizeAlt(AltCfg &a) {      // also rejects NaN from a bad save
  AltCfg d; defaultAlt(d);
  if (!(a.altP >= 0.1f && a.altP <= 5))        a.altP = d.altP;
  if (!(a.velP >= 0 && a.velP <= 500))         a.velP = d.velP;
  if (!(a.velI >= 0 && a.velI <= 300))         a.velI = d.velI;
  if (!(a.maxClimb >= 0.2f && a.maxClimb <= 5)) a.maxClimb = d.maxClimb;
  if (!(a.maxDesc >= 0.2f && a.maxDesc <= 3))  a.maxDesc = d.maxDesc;
  if (!(a.deadband >= 20 && a.deadband <= 250)) a.deadband = d.deadband;
  if (!(a.minThr >= 1050 && a.minThr <= 1500)) a.minThr = d.minThr;
  if (!(a.ceilingM >= 0 && a.ceilingM <= 400)) a.ceilingM = d.ceilingM;
  if (!(a.tofMaxM >= 0.3f && a.tofMaxM <= 8.0f)) a.tofMaxM = d.tofMaxM;   // MTF-01 reaches ~8 m
}

void defaultFlow(FlowCfg &f) {
  f.scale = 10.5f;  f.align = 0;  f.flip = 0;  f.compSign = 0;  f.rfSource = RF_AUTO;
}

void sanitizeFlow(FlowCfg &f) {
  if (!(f.scale >= 0.5f && f.scale <= 5000)) f.scale = 10.5f;
  if (f.align > 3) f.align = 0;
  if (f.flip > 1) f.flip = 0;
  if (f.compSign > 2) f.compSign = 0;
  if (f.rfSource > RF_MTF) f.rfSource = RF_AUTO;
}

void defaultPos(PosCfg &p) {
  p.posP = 1.5f;  p.velP = 8.0f;  p.velI = 3.0f;
  p.maxSpeed = 2.0f;  p.maxTilt = 20.0f;  p.minQuality = 60.0f;   // 2 m/s at full stick
}

void sanitizePos(PosCfg &p) {
  PosCfg d; defaultPos(d);
  if (!(p.posP >= 0 && p.posP <= 5))          p.posP = d.posP;
  if (!(p.velP >= 0 && p.velP <= 30))         p.velP = d.velP;
  if (!(p.velI >= 0 && p.velI <= 20))         p.velI = d.velI;
  if (!(p.maxSpeed >= 0.2f && p.maxSpeed <= 3)) p.maxSpeed = d.maxSpeed;
  if (!(p.maxTilt >= 3 && p.maxTilt <= 30))   p.maxTilt = d.maxTilt;
  if (!(p.minQuality >= 0 && p.minQuality <= 250)) p.minQuality = d.minQuality;
}

void defaultAvoid(AvoidCfg &a) {
  a.role = VL53_FORWARD;  a.enabled = 0;
  a.avoidDist = 2.0f;  a.stopDist = 0.6f;  a.maxBack = 8.0f;
}

void sanitizeAvoid(AvoidCfg &a) {
  if (a.role > VL53_BACKWARD) a.role = VL53_FORWARD;
  if (a.enabled > 1) a.enabled = 0;
  if (!(a.avoidDist >= 0.5f && a.avoidDist <= 2.0f)) a.avoidDist = 2.0f;   // VL53L1X: reliable to ~2 m
  if (!(a.stopDist >= 0.2f && a.stopDist <= a.avoidDist - 0.2f)) a.stopDist = min(0.6f, a.avoidDist - 0.2f);
  if (!(a.maxBack >= 0 && a.maxBack <= 15)) a.maxBack = 8.0f;
}

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
    flightMode = prefs.getUChar("fmode", 1) ? 1 : 0;
    ch6Extra = prefs.getUChar("ch6x", CH6X_AIRMODE);
    rcSmoothHz    = prefs.getFloat("rcsm", 20.0f);  if (!(rcSmoothHz >= 0 && rcSmoothHz <= 50))     rcSmoothHz = 20.0f;
    itermRelaxDps = prefs.getFloat("irel", 40.0f);  if (!(itermRelaxDps >= 0 && itermRelaxDps <= 200)) itermRelaxDps = 40.0f;
    antiGravGain  = prefs.getFloat("antig", 3.0f);  if (!(antiGravGain >= 0 && antiGravGain <= 10))   antiGravGain = 3.0f;
    defaultFilt(filt);
    if (prefs.getBytesLength("filt") == sizeof(FiltCfg)) prefs.getBytes("filt", &filt, sizeof(FiltCfg));
    sanitizeFilt(filt);
    defaultAlt(altCfg);
    if (prefs.getBytesLength("alth") == sizeof(AltCfg)) prefs.getBytes("alth", &altCfg, sizeof(AltCfg));
    sanitizeAlt(altCfg);
    defaultFlow(flowCfg);
    if (prefs.getBytesLength("flow") == sizeof(FlowCfg)) prefs.getBytes("flow", &flowCfg, sizeof(FlowCfg));
    sanitizeFlow(flowCfg);
    defaultAvoid(avoidCfg);
    if (prefs.getBytesLength("avoid") == sizeof(AvoidCfg)) prefs.getBytes("avoid", &avoidCfg, sizeof(AvoidCfg));
    sanitizeAvoid(avoidCfg);
#if !USE_AVOID
    avoidCfg.role = VL53_DOWN;              // the VL53L1X looks down: it is the alt-hold rangefinder
    avoidCfg.enabled = 0;
#endif
    defaultPos(posCfg);
    if (prefs.getBytesLength("posh") == sizeof(PosCfg)) prefs.getBytes("posh", &posCfg, sizeof(PosCfg));
    sanitizePos(posCfg);
    if (posCfg.maxSpeed == 1.0f && posCfg.maxTilt == 15.0f) { posCfg.maxSpeed = 2.0f; posCfg.maxTilt = 20.0f; }   // old defaults -> new
    if (posCfg.posP == 1.0f && posCfg.velP == 6.0f && posCfg.velI == 2.0f) { posCfg.posP = 1.5f; posCfg.velP = 8.0f; posCfg.velI = 3.0f; }   // stronger hold
    altHover = prefs.getFloat("hover", 1450.0f);
    Serial.printf("Hover throttle (ALT HOLD hold point): %.0f us\n", altHover);
    if (!(altHover >= 1100 && altHover <= 1800)) altHover = 1450.0f;
    if (ch6Extra > CH6X_POSHOLD) ch6Extra = CH6X_AIRMODE;
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
    prefs.putUChar("fmode", flightMode);
    prefs.putUChar("ch6x", ch6Extra);
    prefs.putFloat("rcsm", rcSmoothHz);
    prefs.putFloat("irel", itermRelaxDps);
    prefs.putFloat("antig", antiGravGain);
    prefs.putBytes("filt", &filt, sizeof(FiltCfg));
    prefs.putBytes("alth", &altCfg, sizeof(AltCfg));
    prefs.putBytes("flow", &flowCfg, sizeof(FlowCfg));
    prefs.putBytes("avoid", &avoidCfg, sizeof(AvoidCfg));
    prefs.putBytes("posh", &posCfg, sizeof(PosCfg));
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

void pwmBegin(uint8_t i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(OUT_PIN[i], outHz, PWM_BITS);
#else
  ledcSetup(i, outHz, PWM_BITS);
  ledcAttachPin(OUT_PIN[i], i);
#endif
}

void pwmReinit() {                          // after the output frequency changed (ESC 250 Hz <-> servo 50 Hz)
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
  us = constrain(us, 800, 2200);
  uint32_t duty = (uint32_t)((float)us * outHz * (1 << PWM_BITS) / 1e6f);
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
float vibFastSq[3] = {0, 0, 0};             // same, but reacts within ~50 ms (crash / prop failure detection)
float vibrationFast() { return sqrtf((vibFastSq[0] + vibFastSq[1] + vibFastSq[2]) / 3.0f); }
bool notchOn = false;

void setupAdjustableFilters() {             // notch + accelerometer low-pass, from SETUP > FILTERS
  notchOn = (filt.notchHz > 1.0f && filt.notchHz < LOOP_HZ / 2.0f);
  for (int k = 0; k < 3; k++) {
    if (notchOn) gNotch[k].notch(LOOP_HZ, filt.notchHz, filt.notchQ);
    accLpf[k].lpf(LOOP_HZ, filt.accLpfHz);
  }
}

void initFilters() {                        // fixed filters (the gyro low-pass is set in applyParams)
  setupAdjustableFilters();
  for (int k = 0; k < 3; k++) {
    vibHpf[k].hpf(LOOP_HZ, VIB_HPF_HZ);
  }
}

float vibrationRms() {                      // deg/s of gyro content above VIB_HPF_HZ
  return sqrtf((vibSq[0] + vibSq[1] + vibSq[2]) / 3.0f);
}

// ======================= ESP-NOW RADIO =======================
#define RC_CH 6
// internal channel order: roll, pitch, throttle, yaw, arm, spare (the TX order is remapped on reception)
volatile uint16_t rxCh[RC_CH] = {1500, 1500, 1000, 1500, 1000, 1000};
volatile uint32_t rxLastMs = 0;
volatile bool     txMacKnown = false;
uint8_t           txMac[6];
bool              txPeerAdded = false;
portMUX_TYPE      rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile int8_t   rxRssi = 0;              // dBm of the last good packet
volatile uint16_t rxGoodCount = 0;         // good packets since the last link-quality update
float             rcRssiF = 0;             // smoothed RSSI (dBm)
uint8_t           rcLq = 0;                // link quality: % of the 50 packets/s received (last second)

uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

void handlePacket(const uint8_t *mac, const uint8_t *data, int len, int8_t rssi) {
  if (len != sizeof(ControlPacket)) return;
  ControlPacket p;
  memcpy(&p, data, sizeof p);
  if (p.magic != PKT_MAGIC || p.bindId != BIND_ID) return;
  if (crc8((const uint8_t *)&p, sizeof(p) - 1) != p.crc) return;
  if (txMacKnown && memcmp(mac, txMac, 6) != 0) return;   // locked onto the first transmitter heard

  portENTER_CRITICAL(&rxMux);
  if (!txMacKnown) { memcpy(txMac, mac, 6); txMacKnown = true; }
  rxCh[0] = constrain(p.ch[RX_ROLL],  1000, 2000);
  rxCh[1] = constrain(p.ch[RX_PITCH], 1000, 2000);
  rxCh[2] = constrain(p.ch[RX_THR],   1000, 2000);
  rxCh[3] = constrain(p.ch[RX_YAW],   1000, 2000);
  rxCh[4] = constrain(p.ch[RX_AUX1],  1000, 2000);
  rxCh[5] = constrain(p.ch[RX_AUX2],  1000, 2000);
  rxLastMs = millis();
  rxRssi = rssi;
  rxGoodCount++;
  portEXIT_CRITICAL(&rxMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePacket(info->src_addr, data, len, info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0);
}
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) { handlePacket(mac, data, len, 0); }
#endif

void radioBegin() {
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
  esp_now_register_recv_cb(onEspNowRecv);
  esp_wifi_set_max_tx_power(80);            // full power (20 dBm) for the telemetry back to the TX
  Serial.print("FC MAC: ");
  Serial.println(WiFi.macAddress());
}

uint16_t ch[RC_CH] = {1500, 1500, 1000, 1500, 1000, 1000};
extern bool armed;                          // defined further down (STATE section)
enum Mode { MODE_ACRO, MODE_SELF, MODE_ALT, MODE_POS };   // ALT = self-level + altitude hold, POS = ALT + position hold
Mode mode = MODE_SELF;
bool inFlight = false;                      // armed and the throttle has been raised (motors really working)
bool rcValid = false;                       // link alive (packets within FAILSAFE_MS): keep flying
bool rcFresh = false;                       // packet within RC_STALE_MS: the sticks are current
uint16_t rcGapCount = 0;                    // link gaps longer than RC_STALE_MS this flight
uint32_t rcGapMaxMs = 0;                    // longest gap this flight
bool     fsLanding = false;                  // radio lost in ALT/POS HOLD: descending by itself
uint32_t fsLandStart = 0;
extern bool altHoldActive, altValid, altLanded;   // defined in the ALT HOLD section
void altHoldEngage();

// Two-stage radio failsafe:
//   gap > RC_STALE_MS (0.25 s): sticks considered stale -> roll/pitch/yaw centred (self-level), throttle,
//                               arm and CH6 keep their last value (alt hold keeps holding)
//   gap > FAILSAFE_MS (1 s):    radio lost -> in ALT/POS HOLD: auto-land (descend, disarm when landed or after
//                               FS_LAND_MAX_MS); otherwise disarm. The radio coming back gives control back.
void readRC() {
  uint32_t last;
  portENTER_CRITICAL(&rxMux);
  for (int i = 0; i < RC_CH; i++) ch[i] = rxCh[i];
  last = rxLastMs;
  portEXIT_CRITICAL(&rxMux);
  uint32_t age = last ? millis() - last : 0xFFFFFFFF;
  static bool wasFresh = true;
  rcFresh = last != 0 && age < RC_STALE_MS;
  rcValid = last != 0 && age < FAILSAFE_MS;
  if (!rcFresh && rcValid) {                // short gap: fly level on the last throttle
    ch[0] = 1500; ch[1] = 1500; ch[3] = 1500;
  }
  // auto-land is possible in ALT/POS HOLD, and in SELF STAB when flying with a working altitude sensor (the FC
  // then switches alt hold on by itself). ACRO (no self-level) still disarms.
  bool canLand = altValid && !altLanded && (altHoldActive || (inFlight && mode == MODE_SELF && cfg.vehicle == VEH_QUAD));
  if (!rcValid && armed && (fsLanding || canLand)) {
    uint32_t now = millis();
    if (!fsLanding) {
      fsLanding = true; fsLandStart = now;
      if (!altHoldActive) altHoldEngage();  // from SELF STAB: hold/descend on the altitude sensor
      Serial.println("FAILSAFE: radio lost - auto landing");
    }
    ch[0] = 1500; ch[1] = 1500; ch[3] = 1500;
    ch[2] = 1000;                           // alt hold: stick fully down = descend at MAX DESCENT
    // keep flying (arm + CH6 keep their last value) until landed, the altitude is lost, or the time is up
    if (!altLanded && altValid && now - fsLandStart < FS_LAND_MAX_MS) rcValid = true;
  } else if (rcFresh) {
    if (fsLanding) Serial.println("FAILSAFE: radio back - control returned");
    fsLanding = false;
  }
  if (armed) {                              // gap statistics for the stop report
    if (wasFresh && !rcFresh) rcGapCount++;
    if (!rcFresh && age != 0xFFFFFFFF && age > rcGapMaxMs) rcGapMaxMs = age;
  }
  wasFresh = rcFresh;
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
bool armed = false;
bool armSwitchSeenLow = false;

enum { PAGE_STATUS = 0, PAGE_RC, PAGE_CURVE, PAGE_STATS, PAGE_TREND, PAGE_SENSORS, PAGE_ALT, PAGE_FLOW, PAGE_SETUP, PAGE_COUNT };
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
  int16_t v[7];
  for (int i = 0; i < 7; i++) {             // two separate reads: the order of operands of | is not guaranteed
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  r[0] = v[0]; r[1] = v[1]; r[2] = v[2];    // v[3] = temperature
  r[3] = v[4]; r[4] = v[5]; r[5] = v[6];
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
    vibFastSq[k] += 0.05f * (h * h - vibFastSq[k]);       // fast version (~50 ms) for crash detection

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
  float mv = (float)analogReadMilliVolts(PIN_VBAT);
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

// ======================= ALTITUDE SENSORS (read only, for now) =======================
// Both share the I2C bus with the MPU6050 and the OLED (SDA GPIO6, SCL GPIO7).
//   BMP280 barometer   : address 0x76 (SDO to GND) or 0x77 (SDO to 3V3) - found automatically.
//                        Gives altitude relative to the ground (zeroed at power-up and every time you arm).
//   VL53L1X laser range: address 0x29. Distance to the ground, about 4 cm .. 4 m (less in sunlight / over grass).
//                        Tilt-compensated with roll/pitch -> height above ground.
// Each sensor is read in its own loop tick and only when it has new data, so the 500 Hz flight loop is not slowed.
// A missing sensor is reported on the ALTITUDE page and simply skipped.
#define BARO_PERIOD_MS 40          // BMP280 runs free at ~23 Hz; read it at 25 Hz
#define TOF_PERIOD_MS  10          // poll the VL53L1X "data ready" flag at 100 Hz (it measures at 20 Hz)

// ---- BMP280 (own small driver: no library, never blocks) ----
uint8_t  baroAddr = 0;             // 0 = not found
bool     baroOK = false;
uint16_t bT1; int16_t bT2, bT3;
uint16_t bP1; int16_t bP2, bP3, bP4, bP5, bP6, bP7, bP8, bP9;
float    baroPa = 0, baroTempC = 0;
float    baroAlt = 0;              // m, filtered, relative to baroGroundPa
float    baroGroundPa = 0;
uint32_t baroLastMs = 0;
uint16_t baroErr = 0;

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool baroInit() {
  const uint8_t addrs[2] = {0x76, 0x77};
  for (uint8_t a : addrs) {
    uint8_t id = 0;
    if (i2cRead(a, 0xD0, &id, 1) && (id == 0x58 || id == 0x60)) { baroAddr = a; break; }   // 0x60 = BME280
  }
  if (!baroAddr) { Serial.println("BMP280 not found (0x76/0x77)"); return false; }
  uint8_t c[24];
  if (!i2cRead(baroAddr, 0x88, c, 24)) return false;
  bT1 = c[0] | (c[1] << 8);   bT2 = c[2] | (c[3] << 8);   bT3 = c[4] | (c[5] << 8);
  bP1 = c[6] | (c[7] << 8);   bP2 = c[8] | (c[9] << 8);   bP3 = c[10] | (c[11] << 8);
  bP4 = c[12] | (c[13] << 8); bP5 = c[14] | (c[15] << 8); bP6 = c[16] | (c[17] << 8);
  bP7 = c[18] | (c[19] << 8); bP8 = c[20] | (c[21] << 8); bP9 = c[22] | (c[23] << 8);
  i2cWrite8(baroAddr, 0xF5, 0x10);   // standby 0.5 ms, IIR filter x16
  i2cWrite8(baroAddr, 0xF4, 0x57);   // temp x2, pressure x16, normal mode (~23 Hz)
  Serial.printf("BMP280 found at 0x%02X\n", baroAddr);
  return true;
}

bool baroReadRaw() {                 // Bosch datasheet compensation (float version)
  uint8_t d[6];
  if (!i2cRead(baroAddr, 0xF7, d, 6)) return false;
  int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  if (adcP == 0x80000 || adcT == 0x80000) return false;          // no measurement yet
  float v1 = (adcT / 16384.0f - bT1 / 1024.0f) * bT2;
  float v2 = (adcT / 131072.0f - bT1 / 8192.0f);
  v2 = v2 * v2 * bT3;
  float tFine = v1 + v2;
  baroTempC = tFine / 5120.0f;
  float p1 = tFine / 2.0f - 64000.0f;
  float p2 = p1 * p1 * bP6 / 32768.0f;
  p2 = p2 + p1 * bP5 * 2.0f;
  p2 = p2 / 4.0f + bP4 * 65536.0f;
  p1 = (bP3 * p1 * p1 / 524288.0f + bP2 * p1) / 524288.0f;
  p1 = (1.0f + p1 / 32768.0f) * bP1;
  if (p1 == 0) return false;
  float p = 1048576.0f - adcP;
  p = (p - p2 / 4096.0f) * 6250.0f / p1;
  float q1 = bP9 * p * p / 2147483648.0f;
  float q2 = p * bP8 / 32768.0f;
  baroPa = p + (q1 + q2 + bP7) / 16.0f;
  return baroPa > 30000.0f && baroPa < 120000.0f;
}

float paToAlt(float pa) { return 44330.0f * (1.0f - powf(pa / baroGroundPa, 0.190295f)); }

void baroZero() {                    // current pressure becomes altitude 0
  if (baroOK && baroPa > 0) { baroGroundPa = baroPa; baroAlt = 0; }
}

void baroTick(uint32_t ms) {
  if (!baroOK || ms - baroLastMs < BARO_PERIOD_MS) return;
  baroLastMs = ms;
  if (!baroReadRaw()) { baroErr++; return; }
  if (baroGroundPa <= 0) { baroGroundPa = baroPa; baroAlt = 0; return; }
  float a = paToAlt(baroPa);
  // smoothing set in SETUP > FILTERS > BARO LPF (the sensor also has its own x16 IIR filter)
  float k = 1.0f - expf(-2.0f * PI * filt.baroLpfHz * (BARO_PERIOD_MS / 1000.0f));
  baroAlt += k * (a - baroAlt);
}

// ---- VL53L1X laser range finder ----
VL53L1X tof;
bool     tofOK = false;
uint16_t tofRangeMm = 0;             // raw distance along the sensor axis
float    tofHeightCm = 0;            // tilt-compensated height above ground
bool     tofValid = false;           // last reading was good
uint8_t  tofStatus = 255;
uint32_t tofLastPoll = 0, tofLastGood = 0;
uint32_t tofLastData = 0;            // any reading (valid or not): tells "sensor alive" from "nothing in range"

bool tofInit() {
  tof.setBus(&Wire);
  tof.setTimeout(50);
  if (!tof.init()) { Serial.println("VL53L1X not found (0x29)"); return false; }
  tof.setDistanceMode(VL53L1X::Long);          // up to ~4 m indoors
  tof.setMeasurementTimingBudget(50000);       // 50 ms per measurement
  tof.startContinuous(50);
  tofLastData = millis();                      // watchdog: the first reading may take a moment after boot
  Serial.println("VL53L1X found");
  return true;
}

uint16_t tofRestarts = 0;              // VL53L1X watchdog restarts (this power-up)
bool     tofEverData = false;          // it has delivered a reading since boot (only then is it watched)

void tofTick(uint32_t ms) {
  if (!tofOK || ms - tofLastPoll < TOF_PERIOD_MS) return;
  tofLastPoll = ms;
  // Watchdog: a power glitch resets the VL53L1X into standby and it never measures again (seen in flight: no data
  // at all, fine on the bench). No reading for 0.5 s -> restart it: in the air only the cheap restart of ranging
  // (a few I2C writes), on the ground the full init (blocks ~10-50 ms, not allowed while flying).
  static uint32_t lastKick = 0;
  uint32_t since = ms - tofLastData;
  if (tofEverData && since > 500 && ms - lastKick > 1000) {
    lastKick = ms;
    tofRestarts++;
    if (armed) { tof.stopContinuous(); tof.startContinuous(50); }
    else { tof.init(); tof.setDistanceMode(VL53L1X::Long); tof.setMeasurementTimingBudget(50000); tof.startContinuous(50); }
    Serial.printf("VL53L1X: no data for %lu ms - restarted (%u)\n", (unsigned long)since, tofRestarts);
    tofLastData = ms;                      // give it time to deliver before kicking again
  }
  if (!tof.dataReady()) return;
  tof.read(false);                             // non-blocking: data is already there
  tofLastData = ms;
  tofEverData = true;
  tofStatus = tof.ranging_data.range_status;
  tofRangeMm = tof.ranging_data.range_mm;
  tofValid = (tofStatus == VL53L1X::RangeValid) && tofRangeMm > 0;
  if (tofValid) {
    float tilt = cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD);
    float h = tofRangeMm * 0.1f * (tilt > 0.5f ? tilt : 0.5f);
    // smoothing set in SETUP > FILTERS > TOF LPF (0 = off). Restart it after a gap in good readings.
    if (filt.tofLpfHz <= 0 || ms - tofLastGood > 500) tofHeightCm = h;
    else tofHeightCm += (1.0f - expf(-2.0f * PI * filt.tofLpfHz * 0.05f)) * (h - tofHeightCm);   // 20 Hz updates
    tofLastGood = ms;
  }
}

const char *tofStatusText() {
  if (!tofOK) return "NOT FOUND";
  if (tofValid) return "OK";
  switch (tofStatus) {
    case VL53L1X::SignalFail:        return "NO SIGNAL";
    case VL53L1X::OutOfBoundsFail:   return "OUT OF RANGE";
    case VL53L1X::RangeValidMinRangeClipped: return "TOO CLOSE";
    case VL53L1X::SigmaFail:         return "NOISY";
    default:                         return "NO DATA";
  }
}

// ======================= OPTICAL FLOW + RANGEFINDER (MicoAir MTF-01/02, MSP v2) - read only for now ===========
// Wiring: MTF TX -> GPIO5 (FC RX), MTF RX -> GPIO10 (FC TX, not needed but harmless), 5V, GND.
// Set the MTF-01 to MSP / INAV mode, 115200 baud (MicoAir assistant). It then sends on its own:
//   MSP2_SENSOR_RANGEFINDER 0x1F01: quality(u8) distance_mm(i32, <=0 = invalid)
//   MSP2_SENSOR_OPTIC_FLOW  0x1F02: quality(u8) motionX(i32) motionY(i32)   (motion since the last message)
// Flow -> angular rate (rad/s) = raw / (FLOW SCALE * dt). Ground speed = (flow rate - body rotation) * height.
#define FLOW_RX_PIN 5
#define FLOW_TX_PIN 10
#define FLOW_BAUD   115200

// latest data
uint8_t  flowQ = 0;                 // flow quality 0..255
int32_t  flowRawX = 0, flowRawY = 0;
float    flowRateF = 0, flowRateR = 0;     // rad/s after alignment: F = forward axis, R = right axis
float    flowCompF = 0, flowCompR = 0;     // rad/s with the body rotation removed
float    flowVelF = 0, flowVelR = 0;       // m/s over ground (+F = moving forward, +R = moving right)
uint32_t flowLastMs = 0, flowLastUs = 0;
uint16_t flowMsgs = 0, flowHz = 0;         // messages per second
uint8_t  mspRfQ = 0;
int32_t  mspRfMm = -1;
float    mspRfHeightCm = 0;                // tilt-compensated, TOF LPF smoothed
uint32_t mspRfLastGood = 0, mspRfLastMs = 0;
uint16_t mspRfMsgs = 0, mspRfHz = 0;
uint32_t mspCrcErr = 0;
bool     flowCalActive = false;            // FLOW AUTO CALIBRATE running (see below)
bool     recFsOk = false;                  // flight recorder: LittleFS mounted, flights kept in flash after disarm
float    flowBiasF = 0, flowBiasR = 0;     // resting offset of the compensated flow (rad/s), learned when still

uint8_t crc8DvbS2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}

bool mspRfFresh(uint32_t ms) { return mspRfLastGood && ms - mspRfLastGood < 250; }
bool rfUseMtf(uint32_t ms) {               // which rangefinder feeds the altitude estimate
  if (avoidCfg.role != VL53_DOWN) return true;      // a forward/backward-facing VL53L1X never measures height
  if (flowCfg.rfSource == RF_MTF)  return true;
  if (flowCfg.rfSource == RF_VL53) return false;
  return mspRfFresh(ms) || !tofOK;         // AUTO: MTF when it is alive, otherwise the VL53L1X
}

void mspOnRangefinder(const uint8_t *p, uint16_t n) {
  if (n < 5) return;
  uint32_t ms = millis();
  mspRfQ = p[0];
  mspRfMm = (int32_t)((uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24));
  mspRfLastMs = ms;
  mspRfMsgs++;
  if (mspRfMm > 0) {
    float tilt = cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD);
    float h = mspRfMm * 0.1f * (tilt > 0.5f ? tilt : 0.5f);
    float dt = mspRfLastGood ? (ms - mspRfLastGood) * 0.001f : 0;
    if (filt.tofLpfHz <= 0 || !mspRfLastGood || dt > 0.5f) mspRfHeightCm = h;     // same smoothing as TOF LPF
    else mspRfHeightCm += (1.0f - expf(-2.0f * PI * filt.tofLpfHz * dt)) * (h - mspRfHeightCm);
    mspRfLastGood = ms;
  }
}

void mspOnFlow(const uint8_t *p, uint16_t n) {
  if (n < 9) return;
  uint32_t us = micros();
  flowQ = p[0];
  flowRawX = (int32_t)((uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24));
  flowRawY = (int32_t)((uint32_t)p[5] | ((uint32_t)p[6] << 8) | ((uint32_t)p[7] << 16) | ((uint32_t)p[8] << 24));
  float dt = flowLastUs ? (us - flowLastUs) * 1e-6f : 0;
  flowLastUs = us;
  flowLastMs = millis();
  flowMsgs++;
  if (dt <= 0.002f || dt > 0.2f) return;   // first message or a gap: no rate yet
  if (flowCalActive) flowCalSample(flowRawX / dt, flowRawY / dt);

  // raw -> rad/s, then rotate/mirror into the body axes (F = forward, R = right)
  float rx = flowRawX / (flowCfg.scale * dt), ry = flowRawY / (flowCfg.scale * dt);
  float f, r;
  switch (flowCfg.align) {
    case 1:  f = -ry; r =  rx; break;      // 90
    case 2:  f = -rx; r = -ry; break;      // 180
    case 3:  f =  ry; r = -rx; break;      // 270
    default: f =  rx; r =  ry; break;      // 0
  }
  if (flowCfg.flip) r = -r;
  flowRateF = f;
  flowRateR = r;

  // remove the apparent motion caused by the craft rotating: forward axis <-> pitch rate, right axis <-> roll rate
  float s = (flowCfg.compSign == 0) ? -1.0f : (flowCfg.compSign == 1 ? 1.0f : 0.0f);
  flowCompF = f + s * gyro[1] * DEG_TO_RAD;
  flowCompR = r + s * gyro[0] * DEG_TO_RAD;

  // resting offset: learned while disarmed and still over a surface (it would otherwise look like a slow drift)
  bool still = !armed && fabsf(gyro[0]) < 2 && fabsf(gyro[1]) < 2 && fabsf(gyro[2]) < 2 && flowQ > 50;
  if (still) {
    flowBiasF = constrain(flowBiasF + 0.01f * (flowCompF - flowBiasF), -0.1f, 0.1f);
    flowBiasR = constrain(flowBiasR + 0.01f * (flowCompR - flowBiasR), -0.1f, 0.1f);
  }
  flowCompF -= flowBiasF;
  flowCompR -= flowBiasR;

  // ground speed needs the height: MTF rangefinder if alive, otherwise the VL53L1X
  bool vl53Down = avoidCfg.role == VL53_DOWN && tofOK && tofValid && flowLastMs - tofLastGood < 250;
  float hM = mspRfFresh(flowLastMs) ? mspRfHeightCm * 0.01f : (vl53Down ? tofHeightCm * 0.01f : 0);
  // Minus: the MTF reports the flow caused by MOVING with the opposite sign to the flow caused by ROTATING.
  // AUTO CALIBRATE lines the axes up with the rotation (the gyro), so the speed comes out reversed without it.
  // Verified from flight data: tilting right must make the right speed grow - it went the other way
  // (correlation -0.85 roll, -0.75 pitch), which made POS HOLD push the wrong way.
  flowVelF = -flowCompF * hM;
  flowVelR = -flowCompR * hM;
}

// ---- FLOW AUTO CALIBRATE (SETUP > OPT FLOW > AUTO CALIBRATE) ----
// Hold the craft level, 30-100 cm above a textured floor, and ROCK it in roll and pitch for 10 s without
// moving it sideways. Rotating makes the floor appear to move by exactly the rotation rate (which the gyro
// measures), so a regression of flow counts against gyro rate gives: which sensor axis is forward/right, their
// signs (-> ALIGN + FLIP) and the counts per radian (-> FLOW SCALE). Rotation-flow has the same sign as the flow
// from moving in that direction, so this also fixes the direction of the ground-speed readout.
#define FLOW_CAL_MS 10000
uint32_t flowCalT0 = 0;
double   fcSxp, fcSxr, fcSyp, fcSyr, fcSpp, fcSrr;   // sums of products: raw rates (x,y) vs gyro (pitch,roll)
uint32_t fcN = 0, fcQsum = 0;

void flowCalStart() {
  flowCalActive = true;
  flowCalT0 = millis();
  fcSxp = fcSxr = fcSyp = fcSyr = fcSpp = fcSrr = 0;
  fcN = 0; fcQsum = 0;
  setMsg("ROCK IT 10 S");
  Serial.println("FLOW CAL: rock the craft in roll and pitch for 10 s, keep its position and height");
}

void flowCalSample(float rxRate, float ryRate) {   // raw counts/s, called from mspOnFlow
  float p = gyro[1] * DEG_TO_RAD, r = gyro[0] * DEG_TO_RAD;
  fcSxp += rxRate * p;  fcSxr += rxRate * r;
  fcSyp += ryRate * p;  fcSyr += ryRate * r;
  fcSpp += p * p;       fcSrr += r * r;
  fcN++; fcQsum += flowQ;
}

void flowCalTick(uint32_t ms) {
  if (!flowCalActive) return;
  if (armed || page != PAGE_SETUP) { flowCalActive = false; setMsg("FLOW CAL ABORTED"); return; }
  if (ms - flowCalT0 < FLOW_CAL_MS) return;
  flowCalActive = false;

  float rmsP = fcN ? sqrtf(fcSpp / fcN) : 0, rmsR = fcN ? sqrtf(fcSrr / fcN) : 0;   // rad/s
  float avgQ = fcN ? (float)fcQsum / fcN : 0;
  Serial.printf("FLOW CAL: %lu samples, rock rms pitch %.2f roll %.2f rad/s, quality %.0f\n",
                (unsigned long)fcN, rmsP, rmsR, avgQ);
  if (fcN < 200)                    { setMsg("CAL FAIL: NO FLOW");  alertEvent(0x15, 5); return; }
  if (rmsP < 0.3f || rmsR < 0.3f)   { setMsg("CAL FAIL: ROCK MORE"); alertEvent(0x15, 5); return; }
  if (avgQ < 40)                    { setMsg("CAL FAIL: LOW QUAL");  alertEvent(0x15, 5); return; }

  // which raw axis goes with pitch (= forward axis)?
  bool xIsFwd = (fabs(fcSxp) + fabs(fcSyr)) >= (fabs(fcSxr) + fabs(fcSyp));
  double kF, kR;                         // counts/s per rad/s, signed
  uint8_t align, flip;
  if (xIsFwd) {
    kF = fcSxp / fcSpp;  kR = fcSyr / fcSrr;
    if (kF > 0) { align = 0; flip = (kR > 0) ? 0 : 1; }      // f =  rx, r =  ry
    else        { align = 2; flip = (kR < 0) ? 0 : 1; }      // f = -rx, r = -ry
  } else {
    kF = fcSyp / fcSpp;  kR = fcSxr / fcSrr;
    if (kF > 0) { align = 3; flip = (kR < 0) ? 0 : 1; }      // f =  ry, r = -rx
    else        { align = 1; flip = (kR > 0) ? 0 : 1; }      // f = -ry, r =  rx
  }
  float scale = (float)((fabs(kF) + fabs(kR)) * 0.5);
  float ratio = (fabs(kF) > 1e-6 && fabs(kR) > 1e-6) ? (float)(fabs(kF) / fabs(kR)) : 0;
  Serial.printf("FLOW CAL: forward axis = raw %s, kF %.1f kR %.1f (ratio %.2f) -> SCALE %.1f ALIGN %u FLIP %u\n",
                xIsFwd ? "X" : "Y", kF, kR, ratio, scale, align, flip);
  if (!(scale >= 0.5f && scale <= 5000) || ratio < 0.5f || ratio > 2.0f) {
    setMsg("CAL FAIL: RETRY");                // axes disagree a lot: probably moved sideways too much
    alertEvent(0x15, 5);
    return;
  }
  flowCfg.scale = scale;
  flowCfg.align = align;
  flowCfg.flip = flip;
  flowCfg.compSign = 0;                       // subtract the gyro (correct for a downward camera)
  bool saved = saveParams();                  // save at once: an unsaved result is lost at the next restart
  char b[24];
  snprintf(b, sizeof b, saved ? "FLOW OK %.0f SAVED" : "FLOW %.0f SAVE FAIL", scale);
  setMsg(b);
  alertEvent(0x3, 3);
  Serial.printf("FLOW CAL: %s\n", saved ? "saved" : "SAVE FAILED");
}

void mspHandle(uint16_t cmd, const uint8_t *p, uint16_t n) {
  if (cmd == 0x1F01) mspOnRangefinder(p, n);
  else if (cmd == 0x1F02) mspOnFlow(p, n);
}

// MSP v2 frame: '$' 'X' dir flag cmdLo cmdHi lenLo lenHi payload crc8(dvb-s2, flag..payload)
void mspPoll() {
  static uint8_t st = 0, flag = 0, crc = 0, buf[64];
  static uint16_t cmd = 0, len = 0, idx = 0;
  uint8_t inb[64];                          // read in one block: much cheaper than byte by byte
  int avail = Serial1.available();
  if (avail <= 0) goto rates;
  if (avail > (int)sizeof(inb)) avail = sizeof(inb);   // bounded work per loop tick
  avail = Serial1.read(inb, avail);
  for (int bi = 0; bi < avail; bi++) {
    uint8_t c = inb[bi];
    switch (st) {
      case 0: st = (c == '$') ? 1 : 0; break;
      case 1: st = (c == 'X') ? 2 : 0; break;
      case 2: st = (c == '<' || c == '>' || c == '!') ? 3 : 0; break;
      case 3: flag = c; crc = crc8DvbS2(0, c); st = 4; break;
      case 4: cmd = c; crc = crc8DvbS2(crc, c); st = 5; break;
      case 5: cmd |= (uint16_t)c << 8; crc = crc8DvbS2(crc, c); st = 6; break;
      case 6: len = c; crc = crc8DvbS2(crc, c); st = 7; break;
      case 7:
        len |= (uint16_t)c << 8; crc = crc8DvbS2(crc, c); idx = 0;
        st = (len > sizeof(buf)) ? 0 : (len ? 8 : 9);
        break;
      case 8: buf[idx++] = c; crc = crc8DvbS2(crc, c); if (idx >= len) st = 9; break;
      case 9:
        if (c == crc) mspHandle(cmd, buf, len);
        else mspCrcErr++;
        st = 0;
        break;
    }
  }
  (void)flag;
rates:
  static uint32_t lastRate = 0;              // message rates, once per second
  uint32_t ms = millis();
  if (ms - lastRate >= 1000) { lastRate = ms; flowHz = flowMsgs; mspRfHz = mspRfMsgs; flowMsgs = mspRfMsgs = 0; }
}

bool flowAlive() { return flowLastMs && millis() - flowLastMs < 500; }

// ======================= OBSTACLE AVOIDANCE (forward VL53L1X, up to 2 m) =======================
// SELF STAB and ALT HOLD only (ACRO is never touched). Between AVOID DIST and STOP DIST the allowed forward
// tilt shrinks to zero; closer than STOP DIST the craft tilts BACK by up to MAX BACK. Backward / sideways
// sticks always work, so you can always fly away. The closing speed from the optical flow makes it react
// earlier (0.5 s look-ahead). When the nose dips, the beam can hit the floor: readings that match the
// floor distance (height / sin(pitch)) are ignored, so the floor is not mistaken for a wall.
// "Nothing in range" from the sensor counts as clear. A dead sensor = no avoidance, shown on the ALT page.
bool  avoidSensorOk = false;               // forward sensor alive
bool  obsSeen = false;                     // something within AVOID DIST (warning, even when avoidance is off)
bool  avoiding = false;                    // avoidance is limiting the sticks right now
bool  avoidForced = false;                 // CH6 POS3 without MTF: avoidance on regardless of the stick setting
float obsDist = 0;                         // m
float avoidFwdLimit = 90;                  // deg: max forward (nose-down) tilt allowed
float avoidBack = 0;                       // deg: automatic back tilt

void avoidTick(uint32_t ms) {
  avoidFwdLimit = 90; avoidBack = 0; obsSeen = false; avoiding = false;
  avoidSensorOk = tofOK && avoidCfg.role != VL53_DOWN && tofLastData && ms - tofLastData < 300;
  if (!avoidSensorOk) return;
  if (!(tofValid && ms - tofLastGood < 300)) return;          // nothing in range: clear
  float d = tofRangeMm * 0.001f;
  float dir = (avoidCfg.role == VL53_BACKWARD) ? -1.0f : 1.0f; // +1: sensor on the board's front, -1: on its rear
  float tipDown = pitch * dir;                                // > 0: the sensor side dips towards the floor
  if (tipDown > 3 && mspRfFresh(ms)) {                        // tilted towards the sensor side: is it just the floor?
    float floorD = (mspRfHeightCm * 0.01f) / sinf(tipDown * DEG_TO_RAD);
    if (d > 0.85f * floorD) return;
  }
  if (d > avoidCfg.avoidDist) return;
  obsSeen = true;
  obsDist = d;
  if (!avoidCfg.enabled && !avoidForced) return;
  float vTow = flowVelF * dir;                                // speed towards the sensor side
  float v = (flowAlive() && vTow > 0) ? vTow : 0;             // closing speed, m/s
  float de = d - v * 0.5f;                                    // where we will be in 0.5 s
  float span = max(avoidCfg.avoidDist - avoidCfg.stopDist, 0.1f);
  avoidFwdLimit = constrain((de - avoidCfg.stopDist) / span, 0.0f, 1.0f) * T.lvAngle;
  if (de < avoidCfg.stopDist)
    avoidBack = constrain((avoidCfg.stopDist - de) / avoidCfg.stopDist, 0.0f, 1.0f) * avoidCfg.maxBack;
  avoiding = true;
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
  float update(float err, float meas, float dt, bool useI = true, float iScale = 1.0f) {
    if (useI) integ += err * ki * dt * iScale;   // iScale: I-term relax / anti-gravity
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
  if (avoidCfg.stopDist > avoidCfg.avoidDist - 0.2f) avoidCfg.stopDist = avoidCfg.avoidDist - 0.2f;
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
  pidLevelRoll.init(LOOP_HZ, filt.levelDLpfHz); pidLevelPitch.init(LOOP_HZ, filt.levelDLpfHz);
  setupAdjustableFilters();                 // notch + accelerometer low-pass (SETUP > FILTERS)
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
const char *const NM_CH6X[]  = {"OFF", "AIRMODE", "BEEPER", "ALT HOLD", "POS HOLD"};
const char *const NM_ALIGN[] = {"0", "90 CW", "180", "270 CW"};
const char *const NM_COMP[]  = {"SUBTRACT", "ADD", "OFF"};
const char *const NM_RFSRC[] = {"AUTO", "VL53L1X", "MTF"};
const char *const NM_ROLE[]  = {"DOWN (ALT)", "FORWARD", "BACKWARD"};

const MenuItem menuMain[] = {
  E_ITEM("VEHICLE", cfg.vehicle, NM_VEH, 2),
  E_ITEM("FLIGHT MODE", flightMode, NM_CH6, 2),
  S_ITEM("RC RATES", SUB_RC),
  S_ITEM("SELF-LEVEL", SUB_LEVEL),
  S_ITEM("LEVEL TRIM", SUB_TRIM),
  S_ITEM("GYRO PID", SUB_GYRO),
  S_ITEM("DIRECTIONS", SUB_DIR),
  S_ITEM("BATTERY", SUB_BATT),
  S_ITEM("FILTERS", SUB_FILT),
  S_ITEM("ALT HOLD", SUB_ALT),
  S_ITEM("OPT FLOW", SUB_FLOW),
#if USE_AVOID
  S_ITEM("AVOIDANCE", SUB_AVOID),
#endif
  S_ITEM("POS HOLD", SUB_POS),
  A_ITEM("TX CHECK", ACT_TXTEST),
  A_ITEM("OUTPUT TEST", ACT_OUTTEST),
  A_ITEM("ESC CALIBRATE", ACT_ESCCAL),
  A_ITEM("ESC CAL AT BOOT", ACT_ESCBOOT),
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
  F_ITEM("RC SMOOTH HZ", rcSmoothHz,    0, 50, 1, 0),      // 0 = off
  F_ITEM("ITERM RELAX",  itermRelaxDps, 0, 200, 5, 0),     // deg/s, 0 = off
  F_ITEM("ANTI GRAVITY", antiGravGain,  0, 10, 0.5f, 1),   // 0 = off
};

const MenuItem menuDir[] = {
  E_ITEM("PROPS",         cfg.propsIn,       NM_OUTIN, 2),
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

// Lower Hz = smoother but more delay. The title shows the live vibration level (VIB) to judge the effect.
const MenuItem menuFilt[] = {
  F_ITEM("GYRO LPF",    T.gyroLpf,         20, 200, 5, 0),     // same as in GYRO PID
  F_ITEM("DTERM LPF",   T.dtermLpf,        10, 100, 5, 0),     // same as in GYRO PID
  F_ITEM("NOTCH HZ 0=OFF", filt.notchHz,    0, 240, 5, 0),
  F_ITEM("NOTCH Q",     filt.notchQ,      0.5f, 10, 0.5f, 1),
  F_ITEM("ACC LPF",     filt.accLpfHz,      1, 100, 1, 0),
  F_ITEM("LEVEL D LPF", filt.levelDLpfHz,   2, 100, 1, 0),
  F_ITEM("BARO LPF",    filt.baroLpfHz,  0.1f, 10, 0.1f, 1),
  F_ITEM("TOF LPF 0=OFF", filt.tofLpfHz,    0, 10, 0.5f, 1),
};

// Engage with CH6 POS3 (set DIRECTIONS > CH6 POS3 = ALT HOLD). The title shows the live fused altitude.
const MenuItem menuAlt[] = {
  F_ITEM("ALT P",        altCfg.altP,     0.1f, 5, 0.1f, 1),
  F_ITEM("VEL P",        altCfg.velP,     0, 500, 5, 0),
  F_ITEM("VEL I",        altCfg.velI,     0, 300, 5, 0),
  F_ITEM("MAX CLIMB",    altCfg.maxClimb, 0.2f, 5, 0.1f, 1),
  F_ITEM("MAX DESCENT",  altCfg.maxDesc,  0.2f, 3, 0.1f, 1),
  F_ITEM("STICK DBAND",  altCfg.deadband, 20, 250, 10, 0),
  F_ITEM("MIN THROTTLE", altCfg.minThr,   1050, 1500, 10, 0),
  F_ITEM("CEILING 0=OFF", altCfg.ceilingM, 0, 400, 1, 0),
  F_ITEM("TOF MAX M",    altCfg.tofMaxM,  0.3f, 8.0f, 0.1f, 1),
};

// MicoAir MTF-01/02 optical flow. Check on the FLOW page (BOOT button) - the title here shows flow quality.
const MenuItem menuFlow[] = {
  A_ITEM("AUTO CALIBRATE", ACT_FLOWCAL),
  F_ITEM("FLOW SCALE",   flowCfg.scale,   0.5f, 5000, 5, 0),
  E_ITEM("ALIGN",        flowCfg.align,    NM_ALIGN, 4),
  E_ITEM("FLIP SIDEWAYS", flowCfg.flip,    NM_NOYES, 2),
  E_ITEM("GYRO COMP",    flowCfg.compSign, NM_COMP, 3),
  E_ITEM("RANGEFINDER",  flowCfg.rfSource, NM_RFSRC, 3),
};

// Forward VL53L1X obstacle avoidance (SELF STAB + ALT HOLD). Title shows the live forward distance.
// Optical-flow position hold (CH6 POS3 = POS HOLD). Title shows the fused speed over ground.
const MenuItem menuPos[] = {
  F_ITEM("POS P",       posCfg.posP,       0, 5, 0.1f, 1),
  F_ITEM("VEL P",       posCfg.velP,       0, 30, 0.5f, 1),
  F_ITEM("VEL I",       posCfg.velI,       0, 20, 0.5f, 1),
  F_ITEM("MAX SPEED",   posCfg.maxSpeed,   0.2f, 3, 0.1f, 1),
  F_ITEM("MAX TILT",    posCfg.maxTilt,    3, 30, 1, 0),
  F_ITEM("MIN QUALITY", posCfg.minQuality, 0, 250, 5, 0),
};

const MenuItem menuAvoid[] = {
  E_ITEM("VL53 POINTS",  avoidCfg.role,      NM_ROLE, 3),
  E_ITEM("AVOIDANCE",    avoidCfg.enabled,   NM_OFFON, 2),
  F_ITEM("AVOID DIST M", avoidCfg.avoidDist, 0.5f, 2.0f, 0.1f, 1),
  F_ITEM("STOP DIST M",  avoidCfg.stopDist,  0.2f, 1.8f, 0.1f, 1),
  F_ITEM("MAX BACK DEG", avoidCfg.maxBack,   0, 15, 1, 0),
};

const Menu menus[SUB_COUNT] = {
  {"SETUP",      menuMain,  COUNT_OF(menuMain)},
  {"RC RATES",   menuRc,    COUNT_OF(menuRc)},
  {"SELF-LEVEL", menuLevel, COUNT_OF(menuLevel)},
  {"GYRO PID",   menuGyro,  COUNT_OF(menuGyro)},
  {"DIRECTIONS", menuDir,   COUNT_OF(menuDir)},
  {"LEVEL TRIM", menuTrim,  COUNT_OF(menuTrim)},
  {"BATTERY",    menuBatt,  COUNT_OF(menuBatt)},
  {"FILTERS",    menuFilt,  COUNT_OF(menuFilt)},
  {"ALT HOLD",   menuAlt,   COUNT_OF(menuAlt)},
  {"OPT FLOW",   menuFlow,  COUNT_OF(menuFlow)},
  {"AVOIDANCE",  menuAvoid, COUNT_OF(menuAvoid)},
  {"POS HOLD",   menuPos,   COUNT_OF(menuPos)},
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
// ESC calibration (SETUP > ESC CALIBRATE): 0 = intro, 1 = sending MAX, 2 = sending MIN (done)
uint8_t  escCalStage = 0;
uint32_t escCalMinSince = 0;
#define  ESC_CAL_MAX 2000
#define  ESC_CAL_MIN 1000

struct Repeater {                          // press detection with hysteresis + optional auto-repeat
  bool repeat;
  bool down = false;
  uint32_t tStart = 0, tNext = 0;
  explicit Repeater(bool r) : repeat(r) {}
  bool update(float x, uint32_t now) {     // x > 0 means "pushed in this direction"
    bool pressed = down ? (x > 0.25f) : (x > 0.45f);   // ~45% stick: works on TX rates LOW too
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

void onPageChange() {                      // called when the BOOT button changes the page
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
        defaultFilt(filt);
        defaultAlt(altCfg);
        { uint8_t role = avoidCfg.role; defaultAvoid(avoidCfg); avoidCfg.role = role; }   // keep how it is mounted
        defaultPos(posCfg);
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
    case ACT_FLOWCAL:                      // optical flow: rock the craft for 10 s
      flowCalStart();
      break;
    case ACT_ESCCAL:                       // step-by-step, FC on USB, battery plugged in during the procedure
      screen = SCR_ESCCAL;
      escCalStage = 0;
      break;
    case ACT_ESCBOOT:                      // FC powered by the flight battery: calibrate on the next power-up
      if (prefs.begin("fc", false)) { prefs.putUChar("esccal", 1); prefs.end(); }
      setMsg("UNPLUG+REPLUG BATT");
      alertEvent(0x3, 3);
      Serial.println("ESC calibration armed for the next power-up. PROPS OFF!");
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

  // open the setup page with a stick gesture: throttle low + yaw left + pitch forward (roll centred) for 1 s
  static uint32_t gestureStart = 0;
  bool gesture = !armed && rcValid && page != PAGE_SETUP && ch[2] < 1100 && ch[3] < 1300 && ch[1] > 1700 &&
                 fabsf(stick(ch[0])) < 0.3f;
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

  if (screen == SCR_ESCCAL) {              // yaw right = next step, yaw left = abort / leave
    if (ok) {
      if (escCalStage == 0 && ch[4] < 1500) { escCalStage = 1; Serial.println("ESC CAL: sending MAX"); }
      else if (escCalStage == 1) { escCalStage = 2; escCalMinSince = now; Serial.println("ESC CAL: sending MIN"); }
    }
    if (back) { escCalStage = 0; screen = SCR_MENU; }
    return;
  }

  if (screen == SCR_OUTTEST) {
    if (up)   testOutIdx = (testOutIdx + 3) % 4;
    if (down) testOutIdx = (testOutIdx + 1) % 4;
    if (sx > 0.45f) testDir = 1;
    else if (sx < -0.45f && cfg.vehicle == VEH_PLANE) testDir = -1;
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

// ESC calibration output: MAX then MIN on the ESC outputs (quad: all four; plane: throttle output 3 only).
// Any problem (left SETUP, link lost, ARM switch up, armed) -> outputs go back to safe.
bool escCalActive() {
  return page == PAGE_SETUP && screen == SCR_ESCCAL && !armed && rcValid && ch[4] < 1500 && escCalStage >= 1;
}

void driveEscCal() {
  int us = (escCalStage == 1) ? ESC_CAL_MAX : ESC_CAL_MIN;
  if (cfg.vehicle == VEH_QUAD) { for (int i = 0; i < 4; i++) writeUs(i, us); }
  else { outputsSafe(); writeUs(2, us); }
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
  else if (armed && obsSeen)    buzz = patternOn(0x1, (obsDist < avoidCfg.stopDist) ? 2 : 4, ms);   // obstacle: faster when close
  else if (battLow)             buzz = patternOn(0x5, 20, ms);                // two short beeps every 2 s
#if USE_BUZZER
  digitalWrite(PIN_BUZZ, (buzz && (cfg.alerts || ms < testUntil)) ? HIGH : LOW);
#else
  (void)buzz;
#endif
}

// ======================= ALTITUDE ESTIMATE + ALTITUDE HOLD =======================
// Estimate: a 3rd-order complementary filter. The accelerometer (rotated to "up" with the attitude) gives fast,
// smooth vertical speed; the barometer - or the rangefinder when it is close to the ground - corrects the slow
// drift. It also learns the accelerometer's vertical bias (accCorr).
// Source switching: below ALT HOLD > TOF MAX M with a good reading the rangefinder is used (height above ground,
// precise), otherwise the barometer. On a switch the estimate AND the hold target are shifted by the same amount,
// so the craft does not jump - it keeps its height and simply changes what it measures it with.
// Hold: altitude error -> climb rate target (ALT P) -> throttle (VEL P + VEL I) around the hover throttle.
// Throttle stick inside the deadband around the position it had when hold was engaged = hold; above/below =
// climb/descend at up to MAX CLIMB / MAX DESCENT. Ceiling (CEILING M) is enforced in SELF STAB and ALT HOLD.
float    altEst = 0, velEst = 0, accCorr = 0;  // m, m/s, m/s^2 - in the current source's frame
uint8_t  altSrc = 0;                         // 0 = none, 1 = barometer, 2 = VL53L1X, 3 = MTF (frame of altEst)
bool     altValid = false;
float    accUpMs2 = 0;                       // vertical acceleration (m/s^2), gravity removed
bool     altHoldActive = false;
bool     altLanded = false;                  // alt hold on the ground: motors idle until the stick goes above centre
uint32_t altLandSince = 0;
float    altTarget = 0, altVelTarget = 0, altInteg = 0;
float    altRefThr = 1500;                   // stick position meaning "hold" (where it was when engaged)
float    altHoverBase = 1500;                // throttle (stick us) at engagement
float    altThrOut = 1500;                   // last throttle alt hold commanded
uint32_t exitBlendStart = 0;                 // smooth hand-back to the stick after alt hold ends
float    exitBlendFrom = 0;
#define  EXIT_BLEND_MS 600

// Altitude source, best first (OPT FLOW > RANGEFINDER):
//   AUTO:    MTF rangefinder up to TOF MAX M  ->  VL53L1X (backup, if the MTF drops out)  ->  barometer
//   VL53L1X: VL53L1X up to TOF MAX M -> barometer        MTF: MTF up to TOF MAX M -> barometer
// Each rangefinder has its own hysteresis so a source does not flicker at its limit; a change of source shifts
// the estimate's frame (see altEstTick), so the hold target does not jump.

RfGate gateVl53 = {false, 0, 0}, gateMtf = {false, 0, 0};

bool rfGate(RfGate &g, bool fresh, float hM, float maxM, uint32_t ms) {
  float tilt = max(fabsf(roll), fabsf(pitch));
  bool good = fresh && tilt < 25 && hM < maxM;
  bool bad  = !fresh || tilt > 30 || hM > maxM + 0.3f;
  if (good) { if (!g.goodSince) g.goodSince = ms ? ms : 1; } else g.goodSince = 0;
  if (bad)  { if (!g.badSince)  g.badSince  = ms ? ms : 1; } else g.badSince = 0;
  if (!g.use && g.goodSince && ms - g.goodSince > 300) g.use = true;
  if (g.use && g.badSince && ms - g.badSince > 150) g.use = false;
  return g.use;
}

// 1 = barometer, 2 = VL53L1X, 3 = MTF rangefinder, 0 = nothing
// VL53L1X plausibility: when both rangefinders read but disagree by > 0.3 m for over a second, the VL53L1X is
// not trusted again until the next disarm (seen in flight: VL53L1X 0.10-0.30 m while the MTF read 0.5-1.1 m -
// something in its field of view or a dirty window). It is then never used as backup or for the cross-check.
bool vl53Distrust = false;

uint8_t altPickSource(uint32_t ms) {
  static uint32_t disagreeSince = 0;
  if (!armed) { vl53Distrust = false; disagreeSince = 0; }
  else if (!vl53Distrust && mspRfFresh(ms) && tofValid && ms - tofLastGood < 250) {
    bool off = fabsf(mspRfHeightCm - tofHeightCm) > 30.0f;
    if (off) { if (!disagreeSince) disagreeSince = ms ? ms : 1; } else disagreeSince = 0;
    if (disagreeSince && ms - disagreeSince > 1000) {
      vl53Distrust = true;
      Serial.printf("VL53L1X disagrees with MTF (%.0f vs %.0f cm): not used this flight\n", tofHeightCm, mspRfHeightCm);
    }
  }
  bool vl53Down = avoidCfg.role == VL53_DOWN && tofOK && !vl53Distrust;
  bool allowV = vl53Down && flowCfg.rfSource != RF_MTF;
  bool allowM = flowCfg.rfSource != RF_VL53 || !vl53Down;
  float maxV = altCfg.tofMaxM;
  // both gates are updated every tick (no short-circuit) so their hysteresis timers stay current
  bool v = rfGate(gateVl53, allowV && tofValid && (ms - tofLastGood) < 250, tofHeightCm * 0.01f, maxV, ms);
  bool m = rfGate(gateMtf, allowM && mspRfFresh(ms), mspRfHeightCm * 0.01f, altCfg.tofMaxM, ms);
  // Cross-check: with both rangefinders working, an MTF reading that jumps away from the current height while
  // the VL53L1X still agrees with it is a glitch (seen in flight: MTF 0.38 m, VL53L1X 0.70 m) - ride on the
  // VL53L1X until the MTF agrees again. Both moving together (a real climb or step) keeps the MTF.
  if (m && v && altValid) {
    float dM = fabsf(mspRfHeightCm * 0.01f - altEst), dV = fabsf(tofHeightCm * 0.01f - altEst);
    if (dM > 0.20f && dV < 0.12f) return 2;
  }
  if (m) return 3;                          // MTF first (this is how ALT HOLD flew well)
  if (v) return 2;
  return (baroOK && baroGroundPa > 0) ? 1 : 0;
}

float altSourceMeters(uint8_t src) {
  return src == 2 ? tofHeightCm * 0.01f : (src == 3 ? mspRfHeightCm * 0.01f : baroAlt);
}
const char *altSrcName() { return altSrc == 2 ? "TOF" : (altSrc == 3 ? "MTF" : (altSrc == 1 ? "BARO" : "none")); }
void altEstReset() { altValid = false; }     // re-start from the next measurement (e.g. when the baro is zeroed)

void altEstTick(float dt, uint32_t ms) {
  // up direction in sensor axes, from the attitude estimate (same sign conventions as readIMU)
  float r = roll * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
  float ux = -pitchSensSign * sinf(p), uy = rollSensSign * sinf(r) * cosf(p), uz = cosf(r) * cosf(p);
  float aUp = acc[0] * ux + acc[1] * uy + acc[2] * uz;          // g
  accUpMs2 = (aUp - 1.0f) * 9.80665f;

  // 1 = barometer, 2 = VL53L1X, 3 = MTF rangefinder (a change of source shifts the frame, see above)
  uint8_t src = altPickSource(ms);
  if (src == 0) { altValid = false; altSrc = 0; return; }
  float meas = altSourceMeters(src);

  if (!altValid) {
    altEst = meas; velEst = 0; altValid = true; altSrc = src;
  } else if (src != altSrc) {                // change of sensor: shift estimate and target together
    float shift = meas - altEst;
    altEst += shift;
    altTarget += shift;
    altSrc = src;
  }
  // Disarmed and sitting still: we KNOW the vertical speed is 0, so learn the accelerometer's vertical offset
  // directly (tau ~1 s) and keep the estimate pinned to the sensor. Without this the offset is only learned
  // slowly in the air and the estimate creeps (seen on the bench: +0.5 m and +0.5 m/s while standing still).
  bool still = !armed && fabsf(gyro[0]) < 3 && fabsf(gyro[1]) < 3 && fabsf(gyro[2]) < 3 && fabsf(accMag - 1.0f) < 0.05f;
  if (still) {
    accCorr += (-accUpMs2 - accCorr) * (dt / 1.0f);
    velEst = 0;
    altEst = meas;
    return;
  }

  float tau = (src >= 2) ? 0.8f : 2.5f;      // trust the rangefinder more than the barometer
  float k1 = 3.0f / tau, k2 = 3.0f / (tau * tau), k3 = 1.0f / (tau * tau * tau);
  float err = meas - altEst;
  accCorr = constrain(accCorr + err * k3 * dt, -3.0f, 3.0f);
  velEst = constrain(velEst + (accUpMs2 + accCorr) * dt + err * k2 * dt, -10.0f, 10.0f);
  altEst += velEst * dt + err * k1 * dt;
}

// BATTERY THROTTLE COMPENSATION (as iNav thr_comp_weight): the same throttle gives less thrust as the pack
// voltage drops, so the hover throttle crept up during every flight (1248 -> 1340 us seen). Alt hold therefore
// works in "reference" throttle (what the throttle would be at BATCOMP_VREF_CELL per cell) and only the output is
// scaled by Vref / Vbat. The learned hover throttle ("hover") is stored as a reference value too, so the hold
// point and feed-forward are right on a full pack and on an almost empty one.
#define BATCOMP_VREF_CELL 3.85f
float altThrRef = 1500;                      // last alt hold throttle in reference units (before the battery scale)
float batComp() {
  static float vF = 0;
  if (!battPresent || cellsUsed == 0 || vBat < cellsUsed * 3.0f) { vF = 0; return 1.0f; }
  vF = (vF == 0) ? vBat : vF + 0.002f * (vBat - vF);    // slow (~1-2 s): no loop with the sag under load
  return constrain(cellsUsed * BATCOMP_VREF_CELL / vF, 0.85f, 1.25f);
}
float thrToRef(float raw)   { return 1000.0f + (raw - 1000.0f) / batComp(); }
float thrFromRef(float ref) { return 1000.0f + (ref - 1000.0f) * batComp(); }

// learn the hover throttle while hovering in SELF STAB (used by the ceiling limiter)
void altLearnHover(float dt) {
  static bool wasArmed = false;
  if (armed && inFlight && mode == MODE_SELF && altValid && fabsf(velEst) < 0.3f &&
      fabsf(roll) < 10 && fabsf(pitch) < 10 && ch[2] > 1150)
    altHover += (thrToRef(ch[2]) - altHover) * dt / 4.0f;          // stored as reference throttle
  // in ALT / POS HOLD: learn from the throttle the hold really needs (stick in the hold zone, steady, level)
  if (armed && altHoldActive && !altLanded && fabsf(ch[2] - altRefThr) <= altCfg.deadband &&
      fabsf(velEst) < 0.15f && fabsf(roll) < 8 && fabsf(pitch) < 8 && altThrOut > 1150)
    altHover += (altThrRef - altHover) * dt / 8.0f;
  if (wasArmed && !armed) {                  // after a flight: remember it for next time
    if (prefs.begin("fc", false)) { prefs.putFloat("hover", altHover); prefs.end(); }
  }
  wasArmed = armed;
}

// Height the ceiling is checked against: the fused altitude estimate (rangefinder height above the ground below
// TOF MAX M, barometer above it). The barometer alone drifts by a metre within a flight indoors (prop wash,
// warming up) and then pushed the quad down from a real 1.1 m to 0.15 m.
float ceilHeightM() { return altValid ? altEst : baroAlt; }
bool ceilingOn() { return altCfg.ceilingM > 0.5f && (altValid || (baroOK && baroGroundPa > 0)); }

// climb-rate limit from the ceiling (height from ceilHeightM)
float ceilingVelLimit(float vt) {
  if (!ceilingOn()) return vt;
  float over = ceilHeightM() - altCfg.ceilingM;
  if (over > 0)          return min(vt, -min(altCfg.altP * over, altCfg.maxDesc));
  if (over > -0.3f)      return min(vt, 0.0f);
  return vt;
}

float tiltComp(float thr) {                  // more throttle when tilted, so the vertical thrust stays the same
  float c = cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD);
  if (c < 0.7f) c = 0.7f;
  return 1000.0f + (thr - 1000.0f) / c;
}

// throttle (stick us scale) while alt hold is active
// The HOVER POINT (hover throttle learned in SELF STAB and in alt hold, "hover") is the stick position that
// means HOLD: stick within STICK DBAND of it = hold height, above = climb, below = descend. So the stick that
// hovers in SELF STAB also holds in ALT HOLD - switching modes needs no stick change.
// Ground start (armed in POS2/POS3): "landed" = motors at idle until the stick goes above the hover point, then
// it takes off and climbs. Landing: stick fully down -> it descends; once it sits still on the ground for 0.8 s
// it is "landed" again (motors idle) - ready to disarm, or to take off again with the stick above the hover point.
float hoverPoint() { return constrain(thrFromRef(altHover), 1200.0f, 1750.0f); }   // stick position that hovers NOW

float altHoldThrottle(float dt) {
  uint32_t ms = millis();
  if (altLanded) {
    if (ch[2] > altRefThr + altCfg.deadband) {             // takeoff
      altLanded = false;
      altTarget = altEst;
      altInteg = 0;
      altLandSince = 0;
      setMsg("TAKEOFF");
      Serial.println("ALT HOLD: takeoff");
    } else {
      altThrOut = 1000;
      return 1000;                                         // stick scale 1000 = motor idle, no stabilisation
    }
  } else {                                                 // landing detection
    // On a rangefinder the ground is unmistakable (height < 15 cm): then do not wait for the throttle to fall
    // below the hover base - a large integrator kept the motors at hover thrust on the ground for >1 s.
    // LANDED needs: stick really down (< 1050), not moving, and the ground confirmed - by a rangefinder (< 15 cm)
    // when one works, otherwise by the throttle being clearly below hover for 1.5 s. (It used to accept stick < 1150
    // + slow vertical speed + throttle below hover: a slow descent at 0.63 m was taken for "landed" and the motors
    // were cut in the air - seen in flight.)
    bool rfOk = altSrc >= 2;
    bool nearGround = rfOk && altEst < 0.15f;
    bool stickDown = ch[2] < 1050;
    if (stickDown && nearGround && altInteg > -250.0f) altInteg = max(-250.0f, altInteg - 300.0f * dt);
    bool stillDown = stickDown && fabsf(velEst) < 0.1f && (rfOk ? nearGround : altThrRef < altHoverBase - 50.0f);
    if (stillDown) { if (!altLandSince) altLandSince = ms ? ms : 1; }
    else altLandSince = 0;
    if (altLandSince && ms - altLandSince > (rfOk ? 800u : 1500u)) {
      altLanded = true;
      altLandSince = 0;
      setMsg("LANDED - DISARM");
      alertEvent(0x5, 4);
      Serial.println("ALT HOLD: landed, motors idle");
      altThrOut = 1000;
      return 1000;
    }
  }

  float d = ch[2] - altRefThr, db = altCfg.deadband;
  float climb = 0;
  if (d > db)       climb =  constrain((d - db) / 300.0f, 0.0f, 1.0f) * altCfg.maxClimb;
  else if (d < -db) climb = -constrain((-d - db) / 300.0f, 0.0f, 1.0f) * altCfg.maxDesc;

  // The stick moves the TARGET height at the commanded rate; the controller always holds that target. (It used to
  // drop the target and only follow the climb rate - with the stick just outside the hold zone, alt hold then
  // had no height to come back to and drifted, seen in flight.) The target is kept within 0.5 m of the estimate
  // so it cannot run away when the quad can not follow (ground, ceiling, full throttle).
  if (climb != 0) altTarget = constrain(altTarget + climb * dt, altEst - 0.5f, altEst + 0.5f);
  altVelTarget = constrain(climb + altCfg.altP * (altTarget - altEst), -altCfg.maxDesc, altCfg.maxClimb);
  altVelTarget = ceilingVelLimit(altVelTarget);
  if (ceilingOn() && altVelTarget <= 0 && ceilHeightM() > altCfg.ceilingM - 0.3f && altTarget > altEst) altTarget = altEst;

  float e = altVelTarget - velEst;
  altInteg = constrain(altInteg + altCfg.velI * e * dt, -250.0f, 250.0f);
  altThrRef = tiltComp(altHoverBase + altCfg.velP * e + altInteg);     // reference throttle
  float out = thrFromRef(altThrRef);                                    // scaled for the pack voltage now
  altThrOut = constrain(out, altCfg.minThr, 1850.0f);
  return altThrOut;
}

// SELF STAB with a ceiling: only ever REDUCES the pilot's throttle near/above the ceiling
float ceilingLimitThrottle(float thr) {
  if (!ceilingOn() || cfg.vehicle != VEH_QUAD) return thr;
  float over = ceilHeightM() - altCfg.ceilingM;
  if (over < -0.3f) return thr;
  float vt = ceilingVelLimit(0.0f);
  float lim = tiltComp(altHover + altCfg.velP * (vt - velEst));
  lim = max(lim, altCfg.minThr);
  return min(thr, lim);
}

// armed on the ground with CH6 in POS2/POS3: wait at idle, stick above CENTRE = take off
void altHoldEngageGround() {
  altHoldActive = true;
  altLanded = true;
  altLandSince = 0;
  altTarget = altEst;
  altRefThr = hoverPoint();                  // stick at the hover point = hold
  altHoverBase = constrain(altHover, 1150.0f, 1750.0f);   // feed-forward (reference throttle)
  altInteg = 0;
  altThrOut = 1000;
  setMsg("ALT: STICK UP=TAKEOFF");
  alertEvent(0x3, 3);
  Serial.printf("ALT HOLD ground start: hover point %.0f us, stick above it to take off\n", altRefThr);
}

void altHoldEngage() {
  altHoldActive = true;
  altLanded = false;
  altLandSince = 0;
  altTarget = altEst;
  altRefThr = hoverPoint();                  // hold zone = around the hover throttle, not where the stick is now
  altHoverBase = constrain(altHover, 1150.0f, 1750.0f);   // feed-forward (reference throttle)
  altInteg = 0;
  altThrOut = ch[2];
  setMsg("ALT HOLD ON");
  alertEvent(0x3, 3);                        // one long beep
  Serial.printf("ALT HOLD ON at %.2f m, hover point %.0f (%s)\n", altEst, altRefThr, altSrcName());
}

void altHoldDisengage(const char *why) {
  if (!altHoldActive) return;
  altHoldActive = false;
  exitBlendStart = millis();                 // hand the throttle back to the stick smoothly
  exitBlendFrom = altThrOut;
  setMsg(why);
  alertEvent(0x5, 4);                        // two short beeps
  Serial.printf("ALT HOLD OFF: %s\n", why);
}

uint8_t armBlock = AB_NOLINK;               // reported to the TX: why arming is refused

// ---- why did it disarm / restart? Kept in flash so it survives a brown-out or a power cycle ----
// Shown on the FC screen for 10 s after the next boot and printed on serial ("LAST STOP: ...").
void saveDisarmReason(const char *why) {
  char b[56];
  snprintf(b, sizeof b, "%s %.0fs g%u/%lums %+.0fdBm tofR%u", why, fst.time, rcGapCount, (unsigned long)rcGapMaxMs, rcRssiF, tofRestarts);   // + flight time, radio gaps, signal
  if (prefs.begin("fc", false)) { prefs.putString("lastdis", b); prefs.end(); }
}

void bootReportReasons() {
  esp_reset_reason_t r = esp_reset_reason();
  const char *rs = (r == ESP_RST_BROWNOUT) ? "BROWNOUT" : (r == ESP_RST_PANIC) ? "CRASH" :
                   (r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT) ? "WATCHDOG" : nullptr;
  String last;
  if (prefs.begin("fc", false)) {
    last = prefs.getString("lastdis", "");
    if (rs) { last = String(rs) + " RESET"; prefs.putString("lastdis", last); }  // the restart itself was the stop
    prefs.end();
  }
  Serial.printf("Reset reason %d. LAST STOP: %s\n", (int)r, last.length() ? last.c_str() : "none recorded");
  if (last.length()) {
    char b[24];
    snprintf(b, sizeof b, "LAST:%s", last.c_str());
    setMsg(b);
    msgUntil = millis() + 10000;
  }
}

// ---- CRASH DETECTION: stop the motors when the craft is clearly out of control ----
//   ANGLE:     roll or pitch beyond 70 deg for 0.1 s (SELF / ALT / POS only - in ACRO flips are intended)
//   VIBRATION: the fast vibration level jumps above 40 AND above 4x the flight's normal level for 0.25 s
//              (prop strike, broken or lost prop, failing motor)
// After it the ARM channel must be switched off and on again to re-arm.
#define POS_RETURN_MAX   0.5f      // m/s, POS HOLD: top speed when flying back to the held spot by itself
#define CRASH_ANGLE_DEG  70.0f
#define CRASH_ANGLE_MS   100
#define CRASH_VIB_ABS    40.0f
#define CRASH_VIB_RATIO  4.0f
#define CRASH_VIB_MS     250
void crashDisarm(const char *why) {
  armed = false;
  armSwitchSeenLow = false;                 // re-arming needs the arm channel off -> on
  setMsg(why);
  alertEvent(0x15, 5);
  Serial.printf("DISARM: %s\n", why);
  saveDisarmReason(why);
}

void crashCheck(float dt, uint32_t ms) {
  static uint32_t angSince = 0, vibSince = 0;
  static float vibBase = 5.0f;              // this flight's normal vibration (slow average)
  float vf = vibrationFast();
  if (!armed) { angSince = vibSince = 0; return; }
  if (inFlight) vibBase += (vibrationRms() - vibBase) * (dt / 5.0f);

  bool angBad = (mode != MODE_ACRO) && (fabsf(roll) > CRASH_ANGLE_DEG || fabsf(pitch) > CRASH_ANGLE_DEG);
  if (angBad) { if (!angSince) angSince = ms ? ms : 1; } else angSince = 0;
  if (angSince && ms - angSince > CRASH_ANGLE_MS) { angSince = 0; crashDisarm("CRASH: ANGLE >70"); return; }

  bool vibBad = inFlight && vf > CRASH_VIB_ABS && vf > CRASH_VIB_RATIO * max(vibBase, 3.0f);
  if (vibBad) { if (!vibSince) vibSince = ms ? ms : 1; } else vibSince = 0;
  if (vibSince && ms - vibSince > CRASH_VIB_MS) { vibSince = 0; crashDisarm("CRASH: VIBRATION"); return; }
}

void updateArming() {
  bool armSw = ch[4] > 1700;

  if (!rcValid)      armBlock = AB_NOLINK;
  else if (!imuOK)   armBlock = AB_IMU;
  else if (armed)    armBlock = AB_NONE;
  if (!rcValid || !imuOK) {
    if (armed) {
      Serial.println("DISARM: failsafe");
      setMsg(!rcValid ? "FAILSAFE: NO RADIO" : "FAILSAFE: IMU");
      alertEvent(0x15, 5);
      saveDisarmReason(!rcValid ? (fsLanding ? "RADIO LOST-AUTOLAND" : "RADIO LOST") : "IMU FAIL");
    }
    armed = false;
    armSwitchSeenLow = false;               // arm switch must be cycled after a signal loss
    return;
  }
  if (!armSw) {
    if (armed) {
      Serial.println("DISARMED");
      setMsg("DISARMED");
      alertEvent(0x5, 3);
      if (inFlight && !(altValid && altSrc >= 2 && altEst < 0.15f)) saveDisarmReason("ARM CH OFF IN AIR");   // TX disarm while flying
      else saveDisarmReason("NORMAL DISARM");   // so the boot report never shows an old stop reason
    }
    armed = false;
    armSwitchSeenLow = true;
    return;
  }
  if (armed) return;

  static uint32_t lastDeny = 0;
  const char *why = nullptr;
  armBlock = AB_NONE;
  if (!armSwitchSeenLow)                           { why = "SW HIGH AT BOOT";  armBlock = AB_SWITCH; }
  else if (page == PAGE_SETUP)                     { why = "EXIT SETUP FIRST"; armBlock = AB_SETUP; }
  else if (ch[2] > 1100)                           { why = "THROTTLE NOT LOW"; armBlock = AB_THROTTLE; }
  else if (fabsf(roll) > 25 || fabsf(pitch) > 25)  { why = "TILT > 25 DEG";    armBlock = AB_TILT; }

  if (why) {
    if (millis() - lastDeny > 1000) { setMsg(why); Serial.printf("ARM DENIED: %s\n", why); lastDeny = millis(); alertEvent(0x15, 5); }
    return;
  }
  armed = true;
  resetPIDs();
  baroZero();                               // altitude 0 = where you armed
  rcGapCount = 0; rcGapMaxMs = 0;           // new flight: reset the radio-gap statistics
  altEstReset();
  altHoldActive = false;
  setMsg("ARMED");
  alertEvent(0x3, 2);
  Serial.println("ARMED");
}

// ======================= POSITION HOLD (optical flow) =======================
// Runs on top of ALT HOLD (CH6 POS3 = POS HOLD). Frames: body F (forward) / R (right) from the flow sensor;
// a "world" frame N/E fixed when hold starts, related to the body by a heading integrated from the yaw gyro,
// so the held spot stays put when you yaw. Speed over ground = smoothed flow speed. Chain: position error -> speed target (POS P)
// -> tilt angle (VEL P + VEL I) -> the normal self-level loop. Sticks: roll/pitch fly it at up to MAX SPEED;
// centred = brake, then hold the new spot. Bad flow (quality, height, no data) -> back to plain ALT HOLD.
bool  posActive = false;                   // position hold running (inside alt hold)
bool  posBraking = false;                  // sticks just released: stopping before taking a new spot
float posN = 0, posE = 0;                  // m, estimated position in the world frame
float posTgtN = 0, posTgtE = 0;            // m, held spot
float velF = 0, velR = 0;                  // m/s, fused speed over ground, body axes
float posHeading = 0;                      // rad, clockwise from the heading at engage
float posIF = 0, posIR = 0;                // deg, speed-loop integrators (body axes)
float posAngF = 0, posAngR = 0;            // deg, tilt commanded by position hold (+F = nose down, +R = right down)
uint32_t posBadSince = 0, posGoodSince = 0, posBrakeStart = 0;

bool posFlowValid(uint32_t ms) {           // can the flow be trusted for position right now?
  return flowAlive() && (ms - flowLastMs) < 150 && flowQ >= posCfg.minQuality &&
         mspRfFresh(ms) && mspRfHeightCm > 15.0f && mspRfHeightCm < altCfg.tofMaxM * 100.0f &&
         fabsf(roll) < 35 && fabsf(pitch) < 35;
}

void posReset() {                          // start holding right here
  posN = posE = 0; posTgtN = posTgtE = 0;
  posHeading = 0;
  posIF = posIR = 0;
  posBraking = false;
}

// every loop tick while armed: speed estimate + position (also while not holding, so it is ready)
void posEstimateTick(float dt, uint32_t ms) {
  // Smoothed flow speed (time constant 0.12 s). Deliberately NOT predicted from the tilt angle: with air drag
  // or wind a steady tilt does not mean acceleration, and that would show up as a false speed (a slow creep).
  if (posFlowValid(ms)) {
    float k = dt / (0.12f + dt);
    velF += k * (flowVelF - velF);
    velR += k * (flowVelR - velR);
  } else {
    velF *= (1.0f - dt * 2.0f);            // no flow: let the estimate decay instead of running away
    velR *= (1.0f - dt * 2.0f);
  }
  velF = constrain(velF, -5.0f, 5.0f);
  velR = constrain(velR, -5.0f, 5.0f);
  posHeading += -gyro[2] * DEG_TO_RAD * dt;  // gyro[2] + = counter-clockwise; heading + = clockwise
  float c = cosf(posHeading), s = sinf(posHeading);
  posN += (velF * c - velR * s) * dt;
  posE += (velF * s + velR * c) * dt;
}

// tilt targets (deg) for position hold; sRoll / sPitch are the pilot's sticks (-1..1, + = right / forward)
void posControl(float sRoll, float sPitch, float dt, uint32_t ms) {
  float c = cosf(posHeading), s = sinf(posHeading);
  bool sticks = fabsf(sRoll) > 0.08f || fabsf(sPitch) > 0.08f;
  float vtF, vtR;                                            // speed targets, body axes
  if (sticks) {                                              // pilot flies: speed command, spot follows
    vtF = sPitch * posCfg.maxSpeed;
    vtR = sRoll  * posCfg.maxSpeed;
    posTgtN = posN; posTgtE = posE;
    posBraking = true; posBrakeStart = ms;
  } else if (posBraking) {                                   // released: brake to a stop, then take the spot
    vtF = vtR = 0;
    posTgtN = posN; posTgtE = posE;
    if ((fabsf(velF) < 0.15f && fabsf(velR) < 0.15f) || ms - posBrakeStart > 2000) posBraking = false;
  } else {                                                   // hold the spot
    float eN = posTgtN - posN, eE = posTgtE - posE;
    float eF =  eN * c + eE * s;                             // world error -> body axes
    float eR = -eN * s + eE * c;
    // drifting back to the spot is slow on purpose (the sticks still get MAX SPEED): a fast return overshoots
    const float vRet = min(posCfg.maxSpeed, POS_RETURN_MAX);
    vtF = constrain(posCfg.posP * eF, -vRet, vRet);
    vtR = constrain(posCfg.posP * eR, -vRet, vRet);
  }
  float evF = vtF - velF, evR = vtR - velR;
  const float iLim = 10.0f;                                  // deg: room to cancel a steady push (trim, wind)
  posIF = constrain(posIF + posCfg.velI * evF * dt, -iLim, iLim);
  posIR = constrain(posIR + posCfg.velI * evR * dt, -iLim, iLim);
  posAngF = constrain(posCfg.velP * evF + posIF, -posCfg.maxTilt, posCfg.maxTilt);
  posAngR = constrain(posCfg.velP * evR + posIR, -posCfg.maxTilt, posCfg.maxTilt);
}

// Base mode comes from the stick command / menu (ACRO or SELF STAB). ALT HOLD / POS HOLD are added on top with
// the CH6 switch in POS3 (DIRECTIONS > CH6 POS3), quad only. They only engage when it is safe to: armed, flying
// (throttle raised), a working altitude sensor - and for POS HOLD good optical flow. Losing the altitude sensor
// hands back to SELF STAB; losing the flow hands back to plain ALT HOLD (and position hold resumes by itself
// when the flow is good again).
// CH6 3-position switch = flight mode:  POS1 = base mode (SELF STAB, or ACRO if selected with the stick
// command), POS2 = ALT HOLD, POS3 = POS HOLD (alt hold + position hold). Arming is the TX button only.
uint8_t ch6Level() { return (ch[5] < 1300) ? 0 : (ch[5] < 1700 ? 1 : 2); }

void updateMode() {
  Mode base = flightMode ? MODE_SELF : MODE_ACRO;
  uint8_t lvl = (armed && rcValid && cfg.vehicle == VEH_QUAD) ? ch6Level() : 0;
  bool want = lvl >= 1 || fsLanding;                        // POS2 or POS3: altitude hold (or failsafe landing)
  bool wantPos = (lvl == 2) && USE_MTF;                      // POS3: + position hold (needs the MTF)
  avoidForced = (lvl == 2) && !USE_MTF && USE_AVOID;         // with avoidance fitted: POS3 = alt hold + avoidance on
  static uint8_t lvlPrev = 0;
  bool flipIn = want && lvl != lvlPrev;                      // the switch was just MOVED to POS2/POS3
  lvlPrev = lvl;
  uint32_t ms = millis();

  if (!armed) { altHoldActive = false; posActive = false; }  // disarm / failsafe: silently off
  else if (altHoldActive && !want) { posActive = false; altHoldDisengage("SELF STAB"); }
  else if (altHoldActive && !altValid) { posActive = false; altHoldDisengage("ALT LOST-MANUAL"); }
  else if (altHoldActive && !wantPos && posActive) {         // POS3 -> POS2: keep altitude, drop position
    posActive = false;
    setMsg("ALT HOLD");
    alertEvent(0x5, 4);
  }
  else if (altHoldActive && wantPos && !posActive && flipIn) {   // POS2 -> POS3 in flight: add position hold
    if (posFlowValid(ms)) { posReset(); posActive = true; setMsg("POS HOLD ON"); alertEvent(0x3, 3); }
    else { setMsg("NO FLOW: ALT ONLY"); alertEvent(0x15, 5); }
    posBadSince = posGoodSince = 0;
  }
  else if (!altHoldActive && flipIn) {
    // On a switch move to POS2/POS3 (arming with the switch already up counts as a move):
    //   flying -> hold the current height, the stick position at that moment means "hold";
    //   on the ground -> ground start: motors idle, stick above centre takes off.
    const char *why = nullptr;
    if (!altValid) why = "NO ALT SENSOR";
    if (!why && (!inFlight || ch[2] < 1150)) {
      altHoldEngageGround();
    } else if (!why) {
      altHoldEngage();
      if (wantPos) {
        if (posFlowValid(ms)) { posReset(); posActive = true; setMsg("POS HOLD ON"); }
        else { posActive = false; setMsg("NO FLOW: ALT ONLY"); alertEvent(0x15, 5); }
        posBadSince = posGoodSince = 0;
      }
    } else {
      setMsg(why);
      alertEvent(0x15, 5);                                    // three beeps: refused
      Serial.printf("HOLD refused: %s\n", why);
    }
  }

  // while holding in POS HOLD: drop to ALT HOLD when the flow goes bad, resume when it is good again
  if (altHoldActive && wantPos) {
    bool good = posFlowValid(ms);
    if (good) { posBadSince = 0; if (!posGoodSince) posGoodSince = ms ? ms : 1; }
    else      { posGoodSince = 0; if (!posBadSince) posBadSince = ms ? ms : 1; }
    if (posActive && posBadSince && ms - posBadSince > 300) {
      posActive = false;
      setMsg("FLOW LOST-ALT HOLD");
      alertEvent(0x5, 4);
      Serial.println("POS HOLD: flow lost, ALT HOLD only");
    } else if (!posActive && posGoodSince && ms - posGoodSince > 1000) {
      posReset();
      posActive = true;
      setMsg("POS HOLD BACK");
      alertEvent(0x3, 3);
      Serial.println("POS HOLD: flow good again, holding here");
    }
  }
  mode = altHoldActive ? (posActive ? MODE_POS : MODE_ALT) : base;
}

// ACCELEROMETER TRIM WITH THE STICKS - removes steady drift in SELF STAB without the menu.
// Disarmed, throttle HIGH, yaw centred: push the right stick the way you want the craft to go INSTEAD of
// where it drifts (it drifts right -> push roll LEFT). Each 0.3 s held = 0.1 deg + a beep. Saved on release.
void trimStickTick() {
  static uint32_t next = 0;
  static bool changed = false;
  uint32_t now = millis();
  bool cond = !armed && rcValid && page != PAGE_SETUP && ch[2] > 1800 && fabsf(stick(ch[3])) < 0.3f;
  float sr = stick(ch[0]) * rollStickSign;                    // + = right
  float sp = stick(ch[1]) * pitchStickSign;                   // + = forward (nose down)
  int dr = 0, dp = 0;
  if (cond) {
    if (sr > 0.5f) dr = 1; else if (sr < -0.5f) dr = -1;
    if (sp > 0.5f) dp = 1; else if (sp < -0.5f) dp = -1;
  }
  if (dr || dp) {
    if ((int32_t)(now - next) >= 0) {
      cfg.rollTrim  = constrain(cfg.rollTrim  + 0.1f * dr, -TRIM_LIMIT, TRIM_LIMIT);
      cfg.pitchTrim = constrain(cfg.pitchTrim + 0.1f * dp, -TRIM_LIMIT, TRIM_LIMIT);
      changed = true;
      next = now + 300;
      alertEvent(0x1, 2);                                     // short beep per step
    }
  } else if (changed) {
    changed = false;
    next = 0;
    setMsg(saveParams() ? "TRIM SAVED" : "SAVE FAILED");
    Serial.printf("TRIM roll %.1f pitch %.1f\n", cfg.rollTrim, cfg.pitchTrim);
  }
}

// Flight-mode stick commands (disarmed, not in SETUP, throttle low, pitch centred, yaw left), hold 1 s:
//   roll LEFT -> SELF STAB,  roll RIGHT -> ACRO.  Saved to flash at once.
void modeStickTick() {
  static int8_t pending = -1;
  static uint32_t since = 0;
  static bool fired = false;
  uint32_t now = millis();
  int8_t want = -1;
  if (!armed && rcValid && page != PAGE_SETUP && ch[2] < 1100 && ch[3] < 1300 && fabsf(stick(ch[1])) < 0.3f) {
    if (ch[0] < 1300)      want = 1;        // SELF  (~60% stick, works on TX rates LOW)
    else if (ch[0] > 1700) want = 0;        // ACRO
  }
  if (want != pending) { pending = want; since = now; fired = false; return; }
  if (want < 0 || fired || now - since < 1000) return;
  fired = true;                             // once per hold: centre the sticks to do it again
  flightMode = (uint8_t)want;
  if (prefs.begin("fc", false)) { prefs.putUChar("fmode", flightMode); prefs.end(); }
  setMsg(flightMode ? "MODE: SELF STAB" : "MODE: ACRO");
  if (flightMode) alertEvent(0x1, 3);       // 1 beep
  else            alertEvent(0x5, 4);       // 2 beeps
  Serial.println(flightMode ? "MODE -> SELF STAB" : "MODE -> ACRO");
}

// Obstacle avoidance ON/OFF with the sticks (disarmed, not in SETUP, throttle low, pitch centred, yaw RIGHT),
// hold 1 s:  roll RIGHT -> ON (1 long beep),  roll LEFT -> OFF (2 beeps).  Saved to flash at once.
void avoidStickTick() {
#if !USE_AVOID
  return;                                   // no obstacle sensor: the VL53L1X looks down
#endif
  static int8_t pending = -1;
  static uint32_t since = 0;
  static bool fired = false;
  uint32_t now = millis();
  int8_t want = -1;
  if (!armed && rcValid && page != PAGE_SETUP && ch[2] < 1100 && ch[3] > 1700 && fabsf(stick(ch[1])) < 0.3f) {
    if (ch[0] > 1700)      want = 1;        // ON
    else if (ch[0] < 1300) want = 0;        // OFF
  }
  if (want != pending) { pending = want; since = now; fired = false; return; }
  if (want < 0 || fired || now - since < 1000) return;
  fired = true;                             // once per hold
  if (want == 1 && avoidCfg.role == VL53_DOWN) {     // a downward VL53L1X would see the floor as a wall
    setMsg("VL53 POINTS DOWN");
    alertEvent(0x15, 5);
    return;
  }
  avoidCfg.enabled = (uint8_t)want;
  if (prefs.begin("fc", false)) { prefs.putBytes("avoid", &avoidCfg, sizeof(AvoidCfg)); prefs.end(); }
  setMsg(want ? "AVOIDANCE ON" : "AVOIDANCE OFF");
  if (want) alertEvent(0x3, 3);             // 1 long beep
  else      alertEvent(0x5, 4);             // 2 beeps
  Serial.println(want ? "AVOIDANCE -> ON" : "AVOIDANCE -> OFF");
}

// Telemetry to the transmitter at 20 Hz (armed, mode, arm-refusal reason, attitude)
uint16_t loopTimeUs = 0;
uint16_t loopMaxUs = 0;
uint16_t ctrlMaxUs = 0;                     // worst case of the flight-critical part of the loop
void telemetryTick(uint32_t ms) {
  static uint32_t last = 0, lqLast = 0, rssiLast = 0;
  if (ms - lqLast >= 1000) {                 // link quality over the last second
    lqLast = ms;
    uint16_t n;
    portENTER_CRITICAL(&rxMux); n = rxGoodCount; rxGoodCount = 0; portEXIT_CRITICAL(&rxMux);
    rcLq = (uint8_t)min<uint16_t>(100, n * 2);   // TX sends 50 packets/s
  }
  if (ms - rssiLast >= 20) {                 // smoothed RSSI (~0.4 s)
    rssiLast = ms;
    int8_t r = rxRssi;
    if (r != 0) rcRssiF = (rcRssiF == 0) ? r : rcRssiF + 0.05f * (r - rcRssiF);
  }
  if (txMacKnown && !txPeerAdded) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMac, 6);
    peer.channel = WIFI_CH;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    txPeerAdded = (esp_now_add_peer(&peer) == ESP_OK) || esp_now_is_peer_exist(txMac);
  }
  if (!txPeerAdded || ms - last < 50) return;
  last = ms;
  TelemetryPacket t;
  t.magic = PKT_MAGIC;
  t.bindId = BIND_ID;
  t.armed = armed;
  t.mode = (mode == MODE_ACRO) ? 1 : (mode == MODE_ALT ? 2 : (mode == MODE_POS ? 3 : 0));   // 0 self, 1 acro, 2 alt, 3 pos
  t.armBlock = armed ? AB_NONE : armBlock;
  t.roll10 = (int16_t)constrain(roll * 10.0f, -32000.0f, 32000.0f);
  t.pitch10 = (int16_t)constrain(pitch * 10.0f, -32000.0f, 32000.0f);
  t.loopUs = loopTimeUs;
  t.vbat100 = battPresent ? (uint16_t)constrain(vBat * 100.0f, 0.0f, 65000.0f) : 0;
  t.cells = cellsUsed;
  t.battFlags = (battPresent ? 1 : 0) | (battLow ? 2 : 0) | (obsSeen ? 4 : 0)    // bit2 = obstacle ahead
              | ((avoidCfg.enabled || avoidForced) ? 8 : 0);                    // bit3 = avoidance ON (TX beeps only then)
  t.rssi = (int8_t)lroundf(rcRssiF);
  t.lq = rcLq;
  t.crc = crc8((const uint8_t *)&t, sizeof(t) - 1);
  esp_now_send(txMac, (const uint8_t *)&t, sizeof(t));
}

// ======================= CONTROL =======================
// Signal flow:
//   ACRO      : stick --RC rates-------------------------> rate setpoint --+
//   SELF STAB : stick --> target angle --SELF-LEVEL PID--> rate setpoint --+--> GYRO PID --> mixer
//   YAW       : always stick --RC rates--> rate setpoint
float recRollT = 0, recPitchT = 0;          // self-level target angles (deg), recorded by the flight recorder
float outRoll = 0, outPitch = 0, outYaw = 0;   // us
int   outThrottle = 1000;
float agBoostRec = 1.0f;                // last anti-gravity I boost (diagnostics)
int   outUs[4] = {1000, 1000, 1000, 1000};

void runControl(float dt) {
  float thrIn = ch[2];
  bool quad = (cfg.vehicle == VEH_QUAD);

  // ---- throttle source: alt hold, or the stick (smoothly handed back after alt hold, ceiling-limited) ----
  if (mode == MODE_ALT || mode == MODE_POS) {
    thrIn = altHoldThrottle(dt);
  } else {
    uint32_t since = millis() - exitBlendStart;
    if (exitBlendStart && since < EXIT_BLEND_MS) {
      float k = since / (float)EXIT_BLEND_MS;
      thrIn = exitBlendFrom + (ch[2] - exitBlendFrom) * k;
    }
    if (mode == MODE_SELF) thrIn = ceilingLimitThrottle(thrIn);
  }

  // ---- stick positions, -1..+1 ----
  float sRoll  = stick(ch[0]) * rollStickSign;       // +right down
  float sPitch = stick(ch[1]) * pitchStickSign;      // +nose down
  float sYaw   = -stick(ch[3]);                      // stick right = CW = negative gz

  // RC SMOOTH (as Betaflight's RC smoothing): the radio delivers 50 new stick values a second but this loop runs
  // at LOOP_HZ, so the setpoint used to jump in 20 ms steps - each step kicked the D term and felt "steppy".
  // A 2nd-order low-pass (two PT1 stages) turns the steps into a smooth curve; 20 Hz adds only ~15 ms of delay.
  {
    static float r1 = 0, r2 = 0, p1 = 0, p2 = 0, y1 = 0, y2 = 0;
    if (rcSmoothHz > 0) {
      float k = 1.0f - expf(-2.0f * PI * rcSmoothHz * 1.414f * dt);   // two stages ~ the set cutoff
      r1 += k * (sRoll - r1);  r2 += k * (r1 - r2);  sRoll = r2;
      p1 += k * (sPitch - p1); p2 += k * (p1 - p2);  sPitch = p2;
      y1 += k * (sYaw - y1);   y2 += k * (y1 - y2);  sYaw = y2;
    } else { r1 = r2 = sRoll; p1 = p2 = sPitch; y1 = y2 = sYaw; }
  }

  // ---- rate setpoints (deg/s): what the GYRO PID must achieve ----
  float yawRateSet = stickToRate(sYaw, T.rcYawC, T.rcYawMax, T.rcYawExpo);     // RC rates
  float rollRateSet, pitchRateSet;

  static bool lastLevel = true;                      // SELF STAB and ALT HOLD both self-level
  bool level = (mode != MODE_ACRO);
  if (level != lastLevel) {                          // start the self-level PIDs fresh when entering it
    pidLevelRoll.reset(); pidLevelPitch.reset();
    lastLevel = level;
  }

  if (mode == MODE_ACRO) {
    rollRateSet  = stickToRate(sRoll,  T.rcRollC,  T.rcRollMax,  T.rcRollExpo);
    pitchRateSet = stickToRate(sPitch, T.rcPitchC, T.rcPitchMax, T.rcPitchExpo);
  } else {
    float rollTarget  = expoCurve(sRoll,  T.lvExpo) * T.lvAngle;
    float pitchTarget = expoCurve(sPitch, T.lvExpo) * T.lvAngle;
    if (mode == MODE_POS) {                          // position hold: the sticks command speed, not tilt
      posControl(sRoll, sPitch, dt, millis());
      rollTarget = posAngR;
      pitchTarget = posAngF;
    }
    recRollT = rollTarget; recPitchT = pitchTarget;  // for the flight recorder (before avoidance changes them)
    if (avoiding) {                                  // obstacle on the sensor side: limit tilt towards it, push away
      if (avoidCfg.role == VL53_BACKWARD) {          // sensor on the board's rear: towards it = negative pitch
        if (pitchTarget < -avoidFwdLimit) pitchTarget = -avoidFwdLimit;
        if (avoidBack > 0 && pitchTarget < avoidBack) pitchTarget = avoidBack;
      } else {                                       // sensor on the board's front: towards it = positive pitch
        if (pitchTarget > avoidFwdLimit) pitchTarget = avoidFwdLimit;
        if (avoidBack > 0 && pitchTarget > -avoidBack) pitchTarget = -avoidBack;
      }
    }
    rollRateSet  = pidLevelRoll.update(rollTarget   - roll,  roll,  dt);
    pitchRateSet = pidLevelPitch.update(pitchTarget - pitch, pitch, dt);
  }

  outThrottle = (int)thrIn;

  // AIRMODE (quad, ACRO only): keep full roll/pitch/yaw authority even at minimum throttle, so flips and rolls can
  // start from idle. The mixer lifts all four motors together when one would drop below idle (see below).
  // The I terms stay at zero while the throttle is low, so a craft sitting on the ground cannot wind them up.
  // CH6 POS3 = AIRMODE: the CH6 switch decides (pos 3 = on, pos 2 = off); otherwise the menu setting does
  bool airmodeOn = (cfg.acroAirmode != 0);                     // DIRECTIONS > ACRO AIRMODE (CH6 now selects the mode)
  // Airmode only becomes active once the throttle has been raised past the hover point since arming (Betaflight's
  // airmode_start_throttle). Before that the quad sits on the ground: full authority at idle there made it fight
  // the ground and tip over when the sticks moved (seen in ACRO at 1100-1130 throttle).
  static bool airmodeArmed = false;
  if (!armed) airmodeArmed = false;
  else if (thrIn > hoverPoint()) airmodeArmed = true;
  // ...and it switches off again once landed: throttle at minimum and a rangefinder sees the ground (< 15 cm).
  // Airmode on the floor slammed the motors 1100-2000 and flipped the quad over (seen in ACRO). In the air -
  // mid-flip, inverted or high up - the rangefinder does not read < 15 cm, so airmode stays on there.
  if (airmodeArmed && thrIn < 1050 && altValid && altSrc >= 2 && altEst < 0.15f) airmodeArmed = false;
  bool airmode = quad && mode == MODE_ACRO && airmodeOn && airmodeArmed;
  bool lowThr  = thrIn < 1100;

  if (quad && lowThr && !airmode) {        // quad on the ground: idle, no stabilisation
    resetPIDs();
    outRoll = outPitch = outYaw = 0;
  } else {
    // I term only once the throttle has passed the hover point (lift-off): sitting on the ground with a stick held
    // (or on a slope) must not wind it up, or the quad jumps sideways at take-off
    bool useI = airmodeArmed && !(airmode && lowThr);
    // I-TERM RELAX (Betaflight "setpoint" mode, roll/pitch): while the stick moves the setpoint quickly, the I
    // term must not wind up - otherwise it "bounces back" at the end of flips, rolls and fast stick moves. The
    // fast part of the setpoint (setpoint minus its 15 Hz low-pass) scales the I build-up down to 0.
    // ANTI GRAVITY: a quick throttle change disturbs the attitude (the nose dips on a punch-out) because the I
    // term is too slow to follow; for the moment of the change the I term builds up faster.
    static float spLpR = 0, spLpP = 0, thrLp = 1000;
    float kR = 1.0f - expf(-2.0f * PI * 15.0f * dt);
    spLpR += kR * (rollRateSet - spLpR);
    spLpP += kR * (pitchRateSet - spLpP);
    float relaxR = 1, relaxP = 1;
    if (itermRelaxDps > 0) {
      relaxR = max(0.0f, 1.0f - fabsf(rollRateSet - spLpR) / itermRelaxDps);
      relaxP = max(0.0f, 1.0f - fabsf(pitchRateSet - spLpP) / itermRelaxDps);
    }
    thrLp += (1.0f - expf(-2.0f * PI * 5.0f * dt)) * (thrIn - thrLp);
    float ag = 1.0f;
    if (antiGravGain > 0) ag = min(1.0f + antiGravGain * fabsf(thrIn - thrLp) / 100.0f, 1.0f + antiGravGain);  // 100 us step = x(1+gain)
    agBoostRec = ag;
    // GYRO PID: rate error (setpoint - measured gyro rate) -> correction in us
    outRoll  = pidGyroRoll.update(rollRateSet   - gyro[0], gyro[0], dt, useI, relaxR * ag);
    outPitch = pidGyroPitch.update(pitchRateSet - gyro[1], gyro[1], dt, useI, relaxP * ag);
    outYaw   = pidGyroYaw.update(yawRateSet     - gyro[2], gyro[2], dt, useI);   // yaw: neither (as Betaflight)
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
  u8g2.drawStr(62, 10, mode == MODE_ACRO ? "ACRO" : (mode == MODE_ALT ? "ALT HOLD" : (mode == MODE_POS ? "POS HOLD" : "SELF STAB")));

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
  char b[8];
  u8g2.setFont(u8g2_font_6x10_tf);
  for (int i = 0; i < 6; i++) {
    int y = i * 10 + 2;
    snprintf(b, sizeof b, "%d", i + 1);
    u8g2.drawStr(0, y + 8, b);
    u8g2.drawFrame(10, y, 92, 8);
    int w = (int)(constrain((ch[i] - 1000) / 1000.0f, 0.0f, 1.0f) * 90);
    u8g2.drawBox(11, y + 1, w, 6);
    u8g2.drawVLine(56, y - 1, 10);
    snprintf(b, sizeof b, "%4u", ch[i]);
    u8g2.drawStr(104, y + 8, b);
  }
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
  static const char NAMES[6] = {'R', 'P', 'T', 'Y', 'A', '6'};
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

void drawEscCal() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "ESC CAL  PROPS OFF!");
  u8g2.drawHLine(0, 12, 128);
  if (ch[4] >= 1500 && escCalStage == 0) {
    u8g2.drawStr(0, 28, "LOWER THE ARM SWITCH");
  } else if (escCalStage == 0) {
    u8g2.drawStr(0, 24, "FC on USB, battery");
    u8g2.drawStr(0, 35, "UNPLUGGED. Then:");
    u8g2.drawStr(0, 48, "YAW RIGHT = send MAX");
  } else if (escCalStage == 1) {
    u8g2.drawStr(0, 24, "MAX 2000us sent.");
    u8g2.drawStr(0, 35, "PLUG IN battery, wait");
    u8g2.drawStr(0, 46, "for the ESC beeps, then");
    u8g2.drawStr(0, 57, "YAW RIGHT = send MIN");
  } else {
    u8g2.drawStr(0, 24, "MIN 1000us sent.");
    u8g2.drawStr(0, 35, "Wait for confirm beeps");
    u8g2.drawStr(0, 48, millis() - escCalMinSince > 4000 ? "DONE - YAW LEFT=exit" : "...");
  }
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 64, "YAW LEFT = abort / exit");
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
  if (screen == SCR_ESCCAL)  { drawEscCal();  return; }

  const Menu &m = menus[curMenu];
  int pos = curPos[curMenu];

  snprintf(b, sizeof b, "%s%s", m.title, dirty ? " *" : "");
  u8g2.drawStr(0, 10, b);
  if (curMenu == SUB_BATT) {                 // live reading, to set VOLT RATIO against a multimeter
    snprintf(b, sizeof b, "%5.2fV", vBat);
    u8g2.drawStr(86, 10, b);
  } else if (curMenu == SUB_POS) {           // live fused speed over ground
    snprintf(b, sizeof b, "%+3.0f%+3.0f", velF * 100, velR * 100);
    u8g2.drawStr(80, 10, b);
  } else if (curMenu == SUB_AVOID) {         // live forward distance
    if (!avoidSensorOk) snprintf(b, sizeof b, "NO SENS");
    else if (obsSeen)   snprintf(b, sizeof b, "%4.2fm", obsDist);
    else                snprintf(b, sizeof b, "CLEAR");
    u8g2.drawStr(86, 10, b);
  } else if (curMenu == SUB_FLOW) {          // live flow quality
    if (flowAlive()) snprintf(b, sizeof b, "Q%3u", flowQ);
    else             snprintf(b, sizeof b, "NO DATA");
    u8g2.drawStr(86, 10, b);
  } else if (curMenu == SUB_ALT) {           // live fused altitude
    if (altValid) snprintf(b, sizeof b, "%+5.2fm", altEst);
    else          snprintf(b, sizeof b, "NO ALT");
    u8g2.drawStr(86, 10, b);
  } else if (curMenu == SUB_FILT) {          // live vibration level, to judge the filter settings
    snprintf(b, sizeof b, "VIB%5.1f", vibrationRms());
    u8g2.drawStr(74, 10, b);
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
  else if (flowCalActive) {
    snprintf(b, sizeof b, "ROCK ROLL+PITCH  %lus", (unsigned long)((FLOW_CAL_MS - (millis() - flowCalT0)) / 1000 + 1));
    u8g2.drawStr(0, 63, b);
  }
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
// barometer + rangefinder + fused estimate + alt hold state
void drawAlt() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 9, altHoldActive ? "ALTITUDE  [HOLD]" : "ALTITUDE");
  u8g2.drawHLine(0, 11, 128);
  if (baroOK) snprintf(b, sizeof b, "BARO %+6.2fm %4.0fhPa", baroAlt, baroPa / 100.0f);
  else        snprintf(b, sizeof b, "BARO NOT FOUND");
  u8g2.drawStr(0, 22, b);
  if (avoidCfg.role != VL53_DOWN) {                       // forward/backward obstacle sensor
    if (!avoidSensorOk)  snprintf(b, sizeof b, "FWD  NO SENSOR");
    else if (obsSeen)    snprintf(b, sizeof b, "FWD %4.2fm %s", obsDist, avoiding ? "AVOID" : "WARN");
    else                 snprintf(b, sizeof b, "FWD  clear");
  } else if (tofOK && tofValid && millis() - tofLastGood < 500) snprintf(b, sizeof b, "TOF  %6.1fcm", tofHeightCm);
  else                                                    snprintf(b, sizeof b, "TOF  %s", tofStatusText());
  u8g2.drawStr(0, 33, b);
  if (altValid) snprintf(b, sizeof b, "EST%+6.2f V%+5.1f %s", altEst, velEst, altSrcName());
  else          snprintf(b, sizeof b, "EST  no sensor");
  u8g2.drawStr(0, 44, b);
  if (altHoldActive) snprintf(b, sizeof b, "TGT%+6.2f THR %4.0f", altTarget, altThrOut);
  else               snprintf(b, sizeof b, "HOVER THR %4.0f", altHover);
  u8g2.drawStr(0, 55, b);
  u8g2.setFont(u8g2_font_5x7_tf);
  if (ceilingOn()) snprintf(b, sizeof b, "CEILING %.0f m (alt est)", altCfg.ceilingM);
  else             snprintf(b, sizeof b, "CEILING OFF  (0 = arm point)");
  u8g2.drawStr(0, 64, b);
}

// optical flow + MTF rangefinder (read only for now). Use it to check directions and the scale:
//   rock the craft in place over a textured floor -> COMP stays near 0  (else change GYRO COMP)
//   slide it forward / right by hand          -> VEL F / R go positive  (else ALIGN / FLIP)
void drawFlow() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  if (!flowAlive() && !mspRfFresh(millis())) {
    u8g2.drawStr(0, 9, "OPT FLOW (MTF)");
    u8g2.drawHLine(0, 11, 128);
    u8g2.drawStr(0, 26, "NO DATA on GPIO5");
    u8g2.drawStr(0, 38, "MSP mode, 115200?");
    snprintf(b, sizeof b, "crc err %lu", (unsigned long)mspCrcErr);
    u8g2.drawStr(0, 50, b);
    return;
  }
  snprintf(b, sizeof b, "FLOW %2uHz Q%3u", flowHz, flowQ);           u8g2.drawStr(0, 9, b);
  u8g2.drawHLine(0, 11, 128);
  snprintf(b, sizeof b, "RATE F%+5.2f R%+5.2f", flowRateF, flowRateR); u8g2.drawStr(0, 22, b);
  snprintf(b, sizeof b, "COMP F%+5.2f R%+5.2f", flowCompF, flowCompR); u8g2.drawStr(0, 33, b);
  snprintf(b, sizeof b, "VEL F%+4.0f R%+4.0fcm/s", flowVelF * 100, flowVelR * 100); u8g2.drawStr(0, 44, b);
  if (mspRfFresh(millis())) snprintf(b, sizeof b, "RNG %5.1fcm Q%3u %2uHz", mspRfHeightCm, mspRfQ, mspRfHz);
  else                      snprintf(b, sizeof b, "RNG out of range");
  u8g2.drawStr(0, 55, b);
  u8g2.setFont(u8g2_font_5x7_tf);
  snprintf(b, sizeof b, "alt now %s", altSrcName());
  u8g2.drawStr(0, 64, b);
}

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
  else if (page == PAGE_CURVE) drawCurve();
  else if (page == PAGE_STATS) drawStats();
  else if (page == PAGE_TREND) drawTrend();
  else if (page == PAGE_SENSORS) drawSensors();
  else if (page == PAGE_ALT) drawAlt();
  else if (page == PAGE_FLOW) drawFlow();
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

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // never wait on USB serial: with USB plugged in but no serial monitor open,
                              // every print used to stall ~100 ms -> slow loop + frozen OLED
  // (no delay here: the ESCs, powered at the same moment from the battery, must get their signal right away)

#if USE_BUZZER
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);
#endif
  loadParams();                             // vehicle type + tuning from flash (or defaults)
  for (uint8_t i = 0; i < 4; i++) pwmBegin(i);

  // ESC CAL AT BOOT (armed from the SETUP menu): the ESCs power up together with the FC from the battery,
  // so they must see MAX as early as possible, then MIN. The flag is cleared FIRST, so it runs only once.
  bool escCalBoot = false;
  if (prefs.begin("fc", false)) {
    escCalBoot = prefs.getUChar("esccal", 0) != 0;
    if (escCalBoot) prefs.putUChar("esccal", 0);
    prefs.end();
  }
  if (escCalBoot) {
    if (cfg.vehicle == VEH_QUAD) { for (int i = 0; i < 4; i++) writeUs(i, ESC_CAL_MAX); }
    else { outputsSafe(); writeUs(2, ESC_CAL_MAX); }
  } else {
    outputsSafe();                          // ESCs see 1000 us immediately
  }
  delay(300);                               // give a serial monitor time to attach (outputs are already set)
  Serial.println("ESP32-C3 FC " FW_VERSION);

  Wire.begin(PIN_SDA, PIN_SCL);             // before u8g2.begin() so our pins are kept
  Wire.setClock(400000);
  u8g2.begin();
  u8g2.setBusClock(I2C_HZ);                // u8g2 re-applies this before each screen transfer

  if (escCalBoot) {                         // finish the at-boot ESC calibration: hold MAX, then MIN
    Serial.println("ESC CAL AT BOOT: MAX for 5 s, then MIN for 5 s. PROPS OFF!");
    splash("ESC CAL: MAX", "wait for ESC beeps");
    delay(5000);
    outputsSafe();                          // MIN (and servos centred on a plane)
    splash("ESC CAL: MIN", "wait for confirm");
    delay(5000);
    splash("ESC CAL DONE", "starting...");
    Serial.println("ESC CAL AT BOOT: done");
    delay(800);
  }
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_VBAT, INPUT);
  {                                         // battery pin self-test (serial only): what is connected to GPIO2?
    int rNormal = analogRead(PIN_VBAT);      // first read also sets the pin up for the ADC
    gpio_pullup_en((gpio_num_t)PIN_VBAT);    // weak internal pull-up (~45k) to 3.3 V
    delay(20);
    int rPull = analogRead(PIN_VBAT);
    gpio_pullup_dis((gpio_num_t)PIN_VBAT);
    delay(20);
    Serial.printf("VBAT pin test: normal raw %d, with pull-up raw %d  ", rNormal, rPull);
    if (rPull > 3500)                        Serial.println("-> NOTHING connected to the battery pin");
    else if (rPull < 100)                    Serial.println("-> battery pin SHORTED to GND");
    else if (rNormal < 100 && rPull < 1600)  Serial.println("-> only the resistor to GND: the battery side is open");
    else                                     Serial.println("-> divider connected");
  }

  splash("ESP32-C3 FC " FW_VERSION, "IMU init...");
  if (!mpuInit()) {
    splash("MPU6050 NOT FOUND", "check wiring");
    while (true) { outputsSafe(); delay(500); }
  }

  applyParams();
  initFilters();

  splash("Altitude sensors", "BMP280 / VL53L1X...");
  baroOK = baroInit();
  tofOK = tofInit();
  Wire.setClock(I2C_HZ);                    // fast bus for the 500 Hz loop (IMU read was 0.8 ms at 400 kHz)
#if USE_MTF
  Serial1.setRxBufferSize(512);             // optical flow module (MSP) - must be set before begin()
  Serial1.begin(FLOW_BAUD, SERIAL_8N1, FLOW_RX_PIN, FLOW_TX_PIN);
#endif

  splash("Calibrating gyro", "keep still...");
  calibrateGyro();

  radioBegin();                             // ESP-NOW link to the transmitter
  resetPIDs();
  alertEvent(0x1, 3);                       // one beep: ready
  Serial.printf("Vehicle: %s. Open SETUP: BOOT button, or throttle low + yaw left + pitch forward for 1 s.\n",
                cfg.vehicle == VEH_QUAD ? "QUAD" : "PLANE");
  Serial.println("Flight mode: throttle low + yaw left + roll left (SELF) / roll right (ACRO) for 1 s.");
  bootReportReasons();                      // why the last flight ended (disarm reason or a brown-out restart)
  recFsOk = LittleFS.begin(true);           // first boot formats the storage area once (a few seconds)
  Serial.printf("Flight recorder storage: %s\n", recFsOk ? "ok" : "NOT AVAILABLE (RAM only)");
}

// ======================= FLIGHT RECORDER =======================
// While armed, 50 times a second, into RAM (ring buffer, the last REC_N samples = ~32 s). Cleared at every arm,
// kept after disarm until the next arm. Read it out over USB: send the letter D (Claude does this) and it
// prints a CSV. Nothing is written to flash, nothing is sent over the radio.
#define REC_N  1600
#define REC_MS 20
struct __attribute__((packed)) RecSample {
  uint16_t t10;                 // time since arming, 10 ms units
  int16_t  roll10, pitch10;     // attitude estimate, deg x10
  int16_t  rollT10, pitchT10;   // self-level targets, deg x10
  int16_t  gR, gP, gY;          // gyro rates, deg/s
  uint16_t thr, rcR, rcP, rcY;  // sticks, us
  uint16_t m[4];                // motor outputs, us
  uint16_t vbat100;             // pack voltage x100
  uint16_t ctrlUs;              // flight-critical loop time
  int16_t  alt100;              // altitude estimate, cm
  int16_t  posErrF, posErrR;    // POS HOLD: distance from the held spot, body axes, cm (+ = spot is ahead / right)
  int16_t  velF100, velR100;    // fused ground speed, cm/s
  uint8_t  flags;               // bits 0-1 mode, 2 avoiding, 3 alt valid, 4-5 alt source (1 baro 2 VL53 3 MTF)
  uint8_t  vib10;               // vibration level x10
  int16_t  accR10, accP10;      // angles from the accelerometer alone (what the level filter is pulled towards), deg x10
  int16_t  mtfCm, tofCm;        // raw rangefinder heights (tilt-compensated), cm; -1 = no fresh reading,
                                // tofCm -100-n = VL53L1X answers but rejects the reading with range status n
  int8_t   rssi;                // dBm the FC receives the TX with (smoothed)
  uint8_t  lq;                  // radio link quality, %
};
RecSample recBuf[REC_N];
uint16_t recHead = 0, recCount = 0;
uint32_t recStartMs = 0, recLastMs = 0;
bool     recWasArmed = false;
bool     recHadFlight = false;     // this arm actually flew (inFlight was set) - only then is it saved


// Save the flight to flash right after disarming (motors already off), oldest sample first, so it survives
// a restart, a power cycle or the USB port being opened. Takes well under a second.
void recSaveToFlash() {
  // Only real flights (at least 5 s armed AND the throttle raised): an arm on the ground must not overwrite the
  // last flight (it did: a 9 s idle arm replaced the flight before it)
  if (!recFsOk || recCount < 250 || !recHadFlight) { if (recCount) Serial.println("REC not saved (no flight in this arm)"); return; }
  File f = LittleFS.open("/rec.bin", "w");
  if (!f) return;
  uint16_t n = recCount, sz = sizeof(RecSample);
  f.write((const uint8_t *)&n, sizeof n);
  f.write((const uint8_t *)&sz, sizeof sz);          // sample layout check for the reader
  uint16_t start = (recCount < REC_N) ? 0 : recHead;
  for (uint16_t k = 0; k < recCount; k++) f.write((const uint8_t *)&recBuf[(start + k) % REC_N], sizeof(RecSample));
  f.close();
  Serial.printf("REC saved to flash: %u samples\n", n);
}

void recTick(uint32_t ms, uint16_t ctrlUs) {
  if (armed && !recWasArmed) { recHead = 0; recCount = 0; recStartMs = ms; recLastMs = 0; recHadFlight = false; }   // new flight
  if (armed && inFlight) recHadFlight = true;
  if (!armed && recWasArmed) recSaveToFlash();   // just disarmed: keep this flight
  recWasArmed = armed;
  if (!armed || ms - recLastMs < REC_MS) return;
  recLastMs = ms;
  RecSample &s = recBuf[recHead];
  s.t10 = (uint16_t)((ms - recStartMs) / 10);
  s.roll10 = (int16_t)constrain(roll * 10, -32000.0f, 32000.0f);
  s.pitch10 = (int16_t)constrain(pitch * 10, -32000.0f, 32000.0f);
  s.rollT10 = (int16_t)constrain(recRollT * 10, -32000.0f, 32000.0f);
  s.pitchT10 = (int16_t)constrain(recPitchT * 10, -32000.0f, 32000.0f);
  s.gR = (int16_t)constrain(gyro[0], -32000.0f, 32000.0f);
  s.gP = (int16_t)constrain(gyro[1], -32000.0f, 32000.0f);
  s.gY = (int16_t)constrain(gyro[2], -32000.0f, 32000.0f);
  s.thr = ch[2]; s.rcR = ch[0]; s.rcP = ch[1]; s.rcY = ch[3];
  for (int i = 0; i < 4; i++) s.m[i] = (uint16_t)outUs[i];
  s.vbat100 = (uint16_t)constrain(vBat * 100, 0.0f, 65000.0f);
  s.ctrlUs = ctrlUs;
  s.alt100 = (int16_t)constrain(altEst * 100, -32000.0f, 32000.0f);
  {
    float c = cosf(posHeading), sn = sinf(posHeading), eN = posTgtN - posN, eE = posTgtE - posE;
    s.posErrF = posActive ? (int16_t)constrain(( eN * c + eE * sn) * 100, -32000.0f, 32000.0f) : 0;
    s.posErrR = posActive ? (int16_t)constrain((-eN * sn + eE * c) * 100, -32000.0f, 32000.0f) : 0;
    s.velF100 = (int16_t)constrain(velF * 100, -32000.0f, 32000.0f);
    s.velR100 = (int16_t)constrain(velR * 100, -32000.0f, 32000.0f);
  }
  s.flags = (uint8_t)((mode & 3) | (avoiding ? 4 : 0) | (altValid ? 8 : 0) | ((altSrc & 3) << 4));
  s.vib10 = (uint8_t)constrain(vibrationRms() * 10, 0.0f, 255.0f);
  s.accR10 = (int16_t)constrain(accRollDeg * 10, -32000.0f, 32000.0f);
  s.accP10 = (int16_t)constrain(accPitchDeg * 10, -32000.0f, 32000.0f);
  s.mtfCm = mspRfFresh(ms) ? (int16_t)constrain(mspRfHeightCm, 0.0f, 32000.0f) : -1;
  s.tofCm = (tofOK && tofValid && ms - tofLastGood < 250) ? (int16_t)constrain(tofHeightCm, 0.0f, 32000.0f)
          : (tofOK && ms - tofLastData < 250) ? (int16_t)(-100 - tofStatus) : -1;
  s.rssi = (int8_t)lroundf(rcRssiF);
  s.lq = rcLq;
  recHead = (recHead + 1) % REC_N;
  if (recCount < REC_N) recCount++;
}

void recDump() {                                  // CSV over USB, oldest sample first
  Serial.setTxTimeoutMs(200);                     // this time it must wait for the PC (disarmed only)
  bool fromFlash = false;
  if (recCount == 0 && recFsOk && LittleFS.exists("/rec.bin")) {   // nothing in RAM (restarted): use the saved flight
    File f = LittleFS.open("/rec.bin", "r");
    uint16_t n = 0, sz = 0;
    if (f && f.read((uint8_t *)&n, sizeof n) == sizeof n && f.read((uint8_t *)&sz, sizeof sz) == sizeof sz &&
        sz == sizeof(RecSample)) {   // a flight saved by an older build (other layout) is skipped
      n = min<uint16_t>(n, REC_N);
      uint16_t got = 0;
      while (got < n && f.read((uint8_t *)&recBuf[got], sizeof(RecSample)) == sizeof(RecSample)) got++;
      recCount = got; recHead = got % REC_N;
      fromFlash = true;
    }
    if (f) f.close();
  }
  Serial.printf("REC BEGIN %u samples, %u ms apart, from %s\n", recCount, REC_MS, fromFlash ? "flash" : "RAM");
  Serial.println("t_s,roll,pitch,rollT,pitchT,gR,gP,gY,thr,rcR,rcP,rcY,m1,m2,m3,m4,vbat,ctrl_us,alt_m,mode,avoid,vib,posErrF_cm,posErrR_cm,velF_cms,velR_cms,accR,accP,altsrc,mtf_cm,tof_cm,rssi,lq");
  uint16_t start = (recCount < REC_N) ? 0 : recHead;
  for (uint16_t k = 0; k < recCount; k++) {
    const RecSample &s = recBuf[(start + k) % REC_N];
    Serial.printf("%.2f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%u,%.2f,%u,%u,%.1f,%d,%d,%d,%d,%.1f,%.1f,%u,%d,%d,%d,%u\n",
                  s.t10 / 100.0f, s.roll10 / 10.0f, s.pitch10 / 10.0f, s.rollT10 / 10.0f, s.pitchT10 / 10.0f,
                  s.gR, s.gP, s.gY, s.thr, s.rcR, s.rcP, s.rcY, s.m[0], s.m[1], s.m[2], s.m[3],
                  s.vbat100 / 100.0f, s.ctrlUs, s.alt100 / 100.0f, s.flags & 3, (s.flags >> 2) & 1, s.vib10 / 10.0f,
                  s.posErrF, s.posErrR, s.velF100, s.velR100, s.accR10 / 10.0f, s.accP10 / 10.0f, (s.flags >> 4) & 3, s.mtfCm, s.tofCm, s.rssi, s.lq);
    if ((k & 31) == 31) delay(2);
  }
  Serial.println("REC END");
  Serial.setTxTimeoutMs(0);
}

void recSerialTick() {                            // 'D' from the PC = dump (only while disarmed)
  while (Serial.available()) {
    int c = Serial.read();
    if ((c == 'D' || c == 'd') && !armed) recDump();
  }
}

// ---- loop profiler (serial only): average / max time of each loop section, printed with the status line ----
#define PROF_N 8
uint32_t profT[PROF_N], profSum[PROF_N - 1], profMax[PROF_N - 1], profCnt = 0;
#define PROF_MARK(i) do { profT[i] = micros(); if (i > 0) { uint32_t d_ = profT[i] - profT[(i) - 1]; profSum[(i) - 1] += d_; if (d_ > profMax[(i) - 1]) profMax[(i) - 1] = d_; if ((i) == PROF_N - 1) profCnt++; } } while (0)
void profPrint() {
  static const char *nm[PROF_N - 1] = {"rc", "imu", "baro/tof", "msp", "est", "logic", "out"};
  if (!profCnt) return;
  for (int i = 0; i < PROF_N - 1; i++) { Serial.printf("%s %lu/%lu ", nm[i], (unsigned long)(profSum[i] / profCnt), (unsigned long)profMax[i]); profSum[i] = profMax[i] = 0; }
  profCnt = 0;
  Serial.print("| ");
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
  PROF_MARK(0);
  readRC();
  batteryTick(millis());
  updateFlightState();
  PROF_MARK(1);
  readIMU(dt);
  PROF_MARK(2);
  {                                         // altitude sensors: one of them per tick, only when due
    static bool baroTurn = false;
    baroTurn = !baroTurn;
    if (baroTurn) baroTick(millis());
    else          tofTick(millis());
  }
  PROF_MARK(3);
#if USE_MTF
  mspPoll();                                // optical flow + rangefinder module (MSP)
#endif
  PROF_MARK(4);
  avoidTick(millis());                      // forward VL53L1X obstacle check
  flowCalTick(millis());                    // optical flow auto calibration (when running)
  altEstTick(dt, millis());                 // fused altitude + vertical speed
#if USE_MTF
  posEstimateTick(dt, millis());            // speed over ground + position (optical flow)
#endif
  PROF_MARK(5);
  levelCalTick();
  statsTick(dt);
  updateArming();
  crashCheck(dt, millis());                 // disarm on > 70 deg (not ACRO) or a sudden vibration jump
  modeStickTick();                          // flight-mode stick commands (disarmed only)
  avoidStickTick();                         // obstacle avoidance on/off stick command (disarmed only)
  trimStickTick();                          // accelerometer trim with the sticks (disarmed, throttle high)
  updateMode();
  alertTick(millis());
  menuTick();                               // stick-driven setup menu (does nothing while armed)
  PROF_MARK(6);

  if (armed) {
    runControl(dt);
    for (int i = 0; i < 4; i++) writeUs(i, outUs[i]);
    altLearnHover(dt);
  } else {
    altLearnHover(dt);                      // also saves the learned hover throttle right after disarming
    resetPIDs();
    if (outputTestActive()) driveTestOutput();
    else if (escCalActive()) driveEscCal();
    else outputsSafe();
  }

  PROF_MARK(7);
  uint16_t ctrlUs = (uint16_t)(micros() - now);   // flight-critical part: sensors -> control -> motors
  if (ctrlUs > ctrlMaxUs) ctrlMaxUs = ctrlUs;
  recTick(millis(), ctrlUs);                // flight recorder (armed only, 50 Hz)
  if (!armed) recSerialTick();              // 'D' over USB = print the last flight

  // ----- UI (runs after outputs are written) -----
  // The OLED goes over the slow I2C bus (shared with the IMU) and drawing a page costs milliseconds, which made
  // the 500 Hz loop late and uneven -> a fast buzz in flight. While ARMED the screen is frozen (nobody reads it
  // in flight); it continues as soon as you disarm.
  handleButton();
  uint32_t ms = millis();
  if (!armed) {
    oledPushSlice();
    if (ms - lastDraw >= 100) { lastDraw = ms; renderFrame(); }
  }
  telemetryTick(ms);

  if (Serial && ms - lastPrint >= 200) {    // only with a PC connected: formatting this line is slow
    lastPrint = ms;
    Serial.printf("LOOP %4uus max %4uus CTRL %4uus max %4uus | ", loopTimeUs, loopMaxUs, ctrlUs, ctrlMaxUs);
    loopMaxUs = 0;
    ctrlMaxUs = 0;
    profPrint();
    Serial.printf("%s %s %s R:%6.1f P:%6.1f VIB:%4.1f V:%5.2f | ",
                  cfg.vehicle == VEH_QUAD ? "QUAD " : "PLANE", armed ? "ARMED " : "DISARM",
                  mode == MODE_ACRO ? "ACRO" : (mode == MODE_ALT ? "ALT " : (mode == MODE_POS ? "POS " : "SELF")), roll, pitch, vibrationRms(), vBat);
    if (rcValid) for (int i = 0; i < 6; i++) Serial.printf("%4u ", ch[i]);
    else Serial.print("NO RC");
    Serial.printf("| RSSI %+.0fdBm LQ %u%% ", rcRssiF, rcLq);
    Serial.printf("| VBAT pin %4.0fmV raw %4d ", vPinMv, analogRead(PIN_VBAT));   // battery divider diagnostics
    if (baroOK) Serial.printf("| ALT %+.2fm ", baroAlt);
    if (altValid) Serial.printf("| EST %+.2fm %+.2fm/s %s ", altEst, velEst, altSrcName());
    if (altHoldActive) Serial.printf("| HOLD tgt %+.2f thr %.0f ", altTarget, altThrOut);
    Serial.printf("| BATCOMP x%.3f hover %.0f ", batComp(), hoverPoint());
    if (posActive) Serial.printf("| POS N%+.2f E%+.2f err %+.2f %+.2f tilt F%+.1f R%+.1f ", posN, posE, posTgtN - posN, posTgtE - posE, posAngF, posAngR);
    Serial.printf("| GVEL F%+.2f R%+.2f GZ%+.1f ", velF, velR, gyro[2]);
    if (flowAlive()) Serial.printf("| FLOW q%u raw %ld %ld comp %+.2f %+.2f vel %+.2f %+.2f ",
                                   flowQ, (long)flowRawX, (long)flowRawY, flowCompF, flowCompR, flowVelF, flowVelR);
    if (mspRfFresh(ms)) Serial.printf("| MTF %.1fcm q%u ", mspRfHeightCm, mspRfQ);
    if (tofOK) {
      if (tofRestarts) Serial.printf("| TOF restarts %u ", tofRestarts);
  if (tofValid) Serial.printf("| TOF %.1fcm", tofHeightCm);
      else          Serial.printf("| TOF %s", tofStatusText());
    }
    Serial.println();
  }
  loopTimeUs = (uint16_t)(micros() - now);
  if (loopTimeUs > loopMaxUs) loopMaxUs = loopTimeUs;       // worst case, shown in the serial line (reset 5x/s)
}
