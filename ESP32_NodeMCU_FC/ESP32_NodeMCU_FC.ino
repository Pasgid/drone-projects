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
 *   Menu: RC RATES, SELF-LEVEL, LEVEL TRIM, GYRO PID, FILTERS, DIRECTIONS, BATTERY, TX CHECK, OUTPUT TEST,
 *         ESC CALIBRATE, ESC CAL AT BOOT, VEHICLE, SAVE, DEFAULTS (press twice), EXIT
 *   VEHICLE     : QUAD <-> PLANE takes effect only after SAVE + a restart (the outputs keep driving the vehicle
 *                 the board started as; arming is refused until then). Changing it live used to send 1500 us
 *                 to three ESCs.
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
 * ---------------- FROM THE C3 FLIGHT CONTROLLER (v21) ----------------
 *   GYRO PID > RC SMOOTH HZ  : stick low-pass between the 50 Hz PPM frames (0 = off). Removes the 20 ms setpoint steps.
 *   GYRO PID > ITERM RELAX   : the I term builds up less while the setpoint moves fast (deg/s, 0 = off): no bounce-back
 *                              after flips, rolls and quick stick moves.
 *   GYRO PID > ANTI GRAVITY  : the I term builds up faster for a moment on a quick throttle change (0 = off): the nose
 *                              no longer dips on a punch-out. A 100 us throttle step = I x (1 + gain).
 *   DIRECTIONS > AIR START   : the I terms and ACRO airmode only start once the throttle has passed this stick value
 *                              since arming (lift-off). Set it a little BELOW your hover stick position.
 *   FILTERS                  : gyro LPF, D-term LPF, gyro notch (Hz + Q), accelerometer LPF, self-level D LPF.
 *   CRASH DETECTION          : disarms when roll/pitch stays beyond 70 deg for 0.1 s (not in ACRO), or when the
 *                              vibration suddenly jumps (prop strike, lost prop). Re-arm needs the ARM switch off -> on.
 *   RADIO GAPS               : PPM missing for > 0.1 s: roll/pitch/yaw centred (self-level in SELF STAB), throttle kept.
 *                              Missing for > 0.5 s: disarm (failsafe).
 *   STOP REASON              : why the last flight ended (normal, failsafe, crash, brown-out / watchdog restart) is kept
 *                              in flash, shown for 10 s after the next boot and printed as "LAST STOP:" on serial.
 *   STICK TRIM               : disarmed, throttle HIGH, yaw centred: push the right stick the way the craft should go
 *                              INSTEAD of where it drifts. 0.1 deg per 0.3 s + a beep; saved when you let go.
 *   ESC CALIBRATE            : in the menu (FC on USB, plug the battery in when told), or ESC CAL AT BOOT (runs once
 *                              at the next power-up, for an FC powered from the flight battery). PROPS OFF!
 *   FLIGHT RECORDER          : 50 samples/s while armed (last ~32 s), saved to flash after a real flight. Send the letter
 *                              D on the serial monitor (disarmed) to print it as CSV.
 *   The OLED is frozen while armed (drawing it costs milliseconds of I2C and made the loop uneven).
 *   Not ported (no such hardware on this board): altitude hold, optical flow, avoidance.
 *
 * ---------------- SERIAL CONFIGURATION (no OLED: USE_OLED 0, the default) ----------------
 *   Serial monitor 115200 baud, line ending "Newline". Type ? for the commands. Disarmed only.
 *   list                 every setting with its range, e.g.  gyro.roll_p = 0.400 (0.00..3.00)
 *   gyro.roll_p 0.45     change one (also "gyro.roll_p=0.45" / "set gyro.roll_p 0.45"); enums by name: dir.props in
 *   save                 keep the changes in flash.   status / sensors / stats / rc (TX check) / levelcal / trimclear
 *   test 1 1150          PROPS OFF: drive output 1 for 3 s.   esccal / esccal min / esccal stop / esccal boot
 *   log off              stop the 5 Hz status line (it also pauses 10 s after every command).   d = flight recorder CSV
 *   With USE_OLED 1 the OLED pages and the stick-driven SETUP menu below work as before (serial works too).
 *
 * ---------------- RC LINK (RC_LINK_ESPNOW) ----------------
 *   0 = PPM receiver on IO32.
 *   1 = ESP-NOW from SimpleTX (ESP32-C3 SuperMini): no receiver. ARM = the TX's arm button (CH5 2000), MODE button
 *       = CH6: the TX's "SELF" = SELF STAB here, its "ALT" = ACRO (this FC has no altitude hold). Telemetry back to
 *       the TX: armed, mode, why arming is refused, attitude, battery, link quality. Signal gaps: 0.25 s = sticks
 *       centred, 1 s = disarm (as on the C3 FC). Do NOT power the C3 FC at the same time (same BIND_ID).
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
#include <esp_system.h>             // esp_reset_reason(): tells a brown-out restart from a normal power-up
#include <LittleFS.h>               // flight recorder: the last flight is kept in flash after disarm

// ======================= FIXED CONFIG (compile time) =======================
// ---- RC link: where the sticks come from ----
//   0 = PPM receiver on IO32
//   1 = ESP-NOW from the SimpleTX transmitter (ESP32-C3 SuperMini). No receiver needed; telemetry goes back to the TX.
//       PKT_MAGIC / BIND_ID / WIFI_CH below must match SimpleTX.ino. Do not power this FC and the C3 FC at the same
//       time with the same BIND_ID: both would answer the transmitter.
#ifndef RC_LINK_ESPNOW
#define RC_LINK_ESPNOW 0
#endif
// ---- OLED: 0 = none fitted (configure over SERIAL, type ? in the serial monitor), 1 = 1.3" OLED + stick SETUP menu ----
#ifndef USE_OLED
#define USE_OLED       0
#endif

#if RC_LINK_ESPNOW
  #include <WiFi.h>
  #include <esp_now.h>
  #include <esp_wifi.h>
  #define PKT_MAGIC   0xC3F1
  #define BIND_ID     42
  #define WIFI_CH     13       // away from phone hotspots / routers (1, 6, 11). MUST match the TX
#endif

#define PIN_SDA  21
#define PIN_SCL  22
#define PIN_PPM  32
#define PIN_BTN  13        // external page button to GND (IO0 = the on-board BOOT button is no longer used)
#define PIN_BUZZ 26        // active buzzer through an NPN transistor
#define USE_BUZZER  1      // set to 0 if no buzzer is fitted
#define PIN_VBAT 34        // battery voltage divider: ADC1, input only, not a strapping pin
const uint8_t OUT_PIN[4] = {18, 19, 23, 25};

//#define OLED_IS_SSD1306          // uncomment if your 1.3" OLED is an SSD1306 (most are SH1106)

#define FW_VERSION "v22 nodemcu serialcfg"  // shown on the start-up screen and on serial: confirms which build is running

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
#if RC_LINK_ESPNOW                  // (values of the C3 FC: the TX sends 50 packets/s, single lost packets are normal)
#define RC_STALE_MS      250       // no packet for this long -> sticks centred, keep flying (see readRC)
#define FAILSAFE_MS      1000      // no packet for this long -> signal lost, disarm
#else
#define RC_STALE_MS      100       // no good PPM frame for this long -> sticks centred, keep flying (see readRC)
#define FAILSAFE_MS      500       // no good PPM frame for this long -> signal lost, disarm
#endif

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
enum { SUB_MAIN, SUB_RC, SUB_LEVEL, SUB_GYRO, SUB_DIR, SUB_TRIM, SUB_BATT, SUB_FILT, SUB_COUNT };
enum { ACT_SAVE, ACT_DEFAULTS, ACT_TXTEST, ACT_OUTTEST, ACT_EXIT, ACT_LEVELCAL, ACT_TRIMCLEAR, ACT_ALERTTEST,
       ACT_ESCCAL, ACT_ESCBOOT };

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

// ---- ESP-NOW packets (keep identical to SimpleTX.ino) ----
enum ArmBlock : uint8_t { AB_NONE, AB_THROTTLE, AB_SWITCH, AB_IMU, AB_TILT, AB_NOLINK, AB_CAL, AB_SETUP };
enum { RX_THR, RX_ROLL, RX_PITCH, RX_YAW, RX_ARM, RX_CH6, RX_NUM_CH };   // order the TX sends
struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  seq;
  uint16_t ch[RX_NUM_CH];          // 1000..2000 us
  uint8_t  crc;
};
struct __attribute__((packed)) TelemetryPacket {   // SimpleTX's layout (shared with SimpleFC)
  uint16_t magic;
  uint8_t  bindId;
  uint8_t  armed;
  uint8_t  mode;                   // TX shows 0 = SELF, 1 = ALT; this FC sends 0 = SELF STAB, 1 = ACRO
  uint8_t  armBlock;               // ArmBlock reason while not armed
  int16_t  roll10, pitch10;        // deg * 10
  uint16_t vbat100;                // pack voltage * 100 (0 = no battery)
  uint8_t  cells;
  uint8_t  flags;                  // bit0 batt present, bit1 batt LOW, bit2 alt valid, bit3 landed, bit4 failsafe land, bit5 no baro
  int16_t  alt100;
  int8_t   rssi;                   // dBm the FC receives the TX with (smoothed)
  uint8_t  lq;                     // % of the TX's 50 packets/s received in the last second
  uint8_t  crc;
};

Cfg  cfg;
Tune T;                            // working copy for the CURRENT vehicle (menu edits this)
Tune tuneStore[2];                 // stored copies, index = vehicle
bool dirty = false;                // unsaved changes

// Filters adjustable in SETUP > FILTERS (stored as "filt"; the gyro and D-term low-pass live in the tune)
struct FiltCfg {
  float notchHz;                   // gyro notch centre, 0 = off (motor/frame vibration peak)
  float notchQ;                    // notch width: higher = narrower
  float accLpfHz;                  // accelerometer low-pass (self-level correction)
  float levelDLpfHz;               // self-level PID D-term low-pass
};
FiltCfg filt = {GYRO_NOTCH_HZ, GYRO_NOTCH_Q, ACC_LPF_HZ, LEVEL_D_LPF_HZ};   // same as defaultFilt()

// Betaflight-style control features (SETUP > GYRO PID / DIRECTIONS), stored under their own keys so the tuning is untouched
float rcSmoothHz    = 20.0f;       // RC SMOOTH: stick low-pass between the 50 Hz PPM frames (Hz), 0 = off, "rcsm"
float itermRelaxDps = 40.0f;       // ITERM RELAX: I builds up less while the setpoint moves fast (deg/s), 0 = off, "irel"
float antiGravGain  = 3.0f;        // ANTI GRAVITY: I builds up faster on quick throttle changes, 0 = off, "antig"
float airStartThr   = 1300.0f;     // AIR START: stick value that counts as lift-off (I terms + airmode start), "airst"

void defaultFilt(FiltCfg &f) {
  f.notchHz = GYRO_NOTCH_HZ;       // off until you find the vibration frequency
  f.notchQ = GYRO_NOTCH_Q;
  f.accLpfHz = ACC_LPF_HZ;
  f.levelDLpfHz = LEVEL_D_LPF_HZ;
}

void sanitizeFilt(FiltCfg &f) {    // also rejects NaN from a bad save
  if (!(f.notchHz >= 0 && f.notchHz <= 240))         f.notchHz = 0;
  if (!(f.notchQ >= 0.5f && f.notchQ <= 10))         f.notchQ = GYRO_NOTCH_Q;
  if (!(f.accLpfHz >= 1 && f.accLpfHz <= 100))       f.accLpfHz = ACC_LPF_HZ;
  if (!(f.levelDLpfHz >= 2 && f.levelDLpfHz <= 100)) f.levelDLpfHz = LEVEL_D_LPF_HZ;
}

void defaultExtras() {             // the settings stored under their own keys
  defaultFilt(filt);
  rcSmoothHz = 20.0f;  itermRelaxDps = 40.0f;  antiGravGain = 3.0f;  airStartThr = 1300.0f;
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
// The vehicle the OUTPUTS drive: the saved one, fixed at power-up. Changing VEHICLE in the menu takes effect after
// SAVE + restart, so ESCs never get the plane's 1500 us servo-centre pulse (and servos never get 250 Hz).
uint8_t outVeh = VEH_QUAD;
uint16_t vehicleHz() { return (outVeh == VEH_QUAD) ? 250 : 50; }

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
    defaultExtras();
    if (prefs.getBytesLength("filt") == sizeof(FiltCfg)) prefs.getBytes("filt", &filt, sizeof(FiltCfg));
    sanitizeFilt(filt);
    rcSmoothHz    = prefs.getFloat("rcsm", 20.0f);   if (!(rcSmoothHz >= 0 && rcSmoothHz <= 50))       rcSmoothHz = 20.0f;
    itermRelaxDps = prefs.getFloat("irel", 40.0f);   if (!(itermRelaxDps >= 0 && itermRelaxDps <= 200)) itermRelaxDps = 40.0f;
    antiGravGain  = prefs.getFloat("antig", 3.0f);   if (!(antiGravGain >= 0 && antiGravGain <= 10))    antiGravGain = 3.0f;
    airStartThr   = prefs.getFloat("airst", 1300.0f); if (!(airStartThr >= 1100 && airStartThr <= 1700)) airStartThr = 1300.0f;
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
  outVeh = cfg.vehicle;                    // loadParams runs once, at power-up
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
    prefs.putBytes("filt", &filt, sizeof(FiltCfg));
    prefs.putFloat("rcsm", rcSmoothHz);
    prefs.putFloat("irel", itermRelaxDps);
    prefs.putFloat("antig", antiGravGain);
    prefs.putFloat("airst", airStartThr);
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
  if (outVeh == VEH_QUAD) {
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
  for (int k = 0; k < 3; k++) vibHpf[k].hpf(LOOP_HZ, VIB_HPF_HZ);
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
extern bool armed;                          // defined further down (STATE section)
bool rcValid = false;                       // signal alive (good frame within FAILSAFE_MS): keep flying
bool rcFresh = false;                       // good frame within RC_STALE_MS: the sticks are current
uint16_t rcGapCount = 0;                    // gaps longer than RC_STALE_MS this flight
uint32_t rcGapMaxMs = 0;                    // longest gap this flight
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

// Two-stage signal loss (as on the C3 FC), for both links:
//   gap > RC_STALE_MS: the sticks are stale -> roll/pitch/yaw centred (self-level), throttle/arm/mode keep their value
//   gap > FAILSAFE_MS: signal lost -> updateArming disarms
void rcLinkState(bool ever, uint32_t ageMs) {
  static bool wasFresh = true;
  rcFresh = ever && ageMs < RC_STALE_MS;
  rcValid = ever && ageMs < FAILSAFE_MS;
  if (!rcFresh && rcValid) { ch[0] = 1500; ch[1] = 1500; ch[3] = 1500; }
  if (armed) {                              // gap statistics for the stop report
    if (wasFresh && !rcFresh) rcGapCount++;
    if (!rcFresh && ever && ageMs > rcGapMaxMs) rcGapMaxMs = ageMs;
  }
  wasFresh = rcFresh;
}

#if RC_LINK_ESPNOW
// ======================= ESP-NOW LINK (SimpleTX) =======================
// The TX sends THR ROLL PITCH YAW ARM CH6 50 times a second; they are remapped to the internal order
// ch[0] roll, ch[1] pitch, ch[2] throttle, ch[3] yaw, ch[4] ARM, ch[5] MODE. The FC locks onto the first TX it hears.
volatile uint16_t rxCh[6] = {1500, 1500, 1000, 1500, 1000, 1000};
volatile uint32_t rxLastMs = 0;
volatile bool     txMacKnown = false;
uint8_t           txMac[6];
bool              txPeerAdded = false;
portMUX_TYPE      rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile int8_t   rxRssi = 0;              // dBm of the last good packet
volatile uint16_t rxGoodCount = 0;         // good packets since the last rate update
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
  rxCh[4] = constrain(p.ch[RX_ARM],   1000, 2000);
  rxCh[5] = constrain(p.ch[RX_CH6],   1000, 2000);
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

void readRC() {
  static uint32_t lastRate = 0;
  uint32_t last;
  portENTER_CRITICAL(&rxMux);
  for (int i = 0; i < 6; i++) ch[i] = rxCh[i];
  last = rxLastMs;
  portEXIT_CRITICAL(&rxMux);
  uint32_t ms = millis();
  rcLinkState(last != 0, last ? ms - last : 0xFFFFFFFFu);
  ppmStable = 6;
  ppmGoodSeen = ppmBadSeen = 0;
  if (ms - lastRate >= 1000) {              // packets per second + link quality over the last second
    uint16_t n;
    portENTER_CRITICAL(&rxMux); n = rxGoodCount; rxGoodCount = 0; portEXIT_CRITICAL(&rxMux);
    ppmHz = (uint16_t)(n * 1000UL / (ms - lastRate));
    rcLq = (uint8_t)min<uint16_t>(100, ppmHz * 2);   // TX sends 50 packets/s
    lastRate = ms;
  }
  static uint32_t rssiLast = 0;
  if (ms - rssiLast >= 20) {                // smoothed RSSI (~0.4 s)
    rssiLast = ms;
    int8_t r = rxRssi;
    if (r != 0) rcRssiF = (rcRssiF == 0) ? r : rcRssiF + 0.05f * (r - rcRssiF);
  }
}

#else
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
  rcLinkState(rcEver, rcEver ? (micros() - rcAcceptUs) / 1000 : 0xFFFFFFFFu);
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
#endif  // RC_LINK_ESPNOW

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
#if !USE_OLED
  Serial.printf("> %s\n", msgText);         // no screen: the message goes to the serial monitor
#endif
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
#if DIAG_NO_ADC
  float mv = 0.0f;
#else
  float mv = (float)analogReadMilliVolts(PIN_VBAT);
#endif
  float v = mv * 0.001f * cfg.vRatio;
  if (!vInit) { vBat = v; vInit = true; }
  else vBat += (dt / (0.5f + dt)) * (v - vBat);
  vPinMv = mv;
  static uint32_t presentSince = 0;
  battPresent = (vBat > 3.0f);
  if (!battPresent) { battLow = false; battLowSince = 0; presentSince = 0; return; }
  if (presentSince == 0) presentSince = ms ? ms : 1;

  // Detect once, after the reading has settled for 2 s: a pack plugged in while the board runs on USB makes the
  // filtered voltage rise slowly, and detecting on the first reading above 3 V counted every pack as 1 cell.
  if (cfg.cells >= 1.0f) cellsUsed = (uint8_t)cfg.cells;
  else if (cellsUsed == 0 && !armed && ms - presentSince > 2000) cellsUsed = (uint8_t)constrain((int)(vBat / 4.35f) + 1, 1, 8);

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

// QUAD <-> PLANE in the menu: swap the tuning set so the other vehicle's values can be edited. The OUTPUTS keep
// driving outVeh (frequency and safe pulses) until SAVE + restart; arming is refused while the two differ.
void applyVehicleChange(uint8_t oldVeh) {
  tuneStore[oldVeh] = T;
  T = tuneStore[cfg.vehicle];
  dirty = true;
  applyParams();
  resetPIDs();
  setMsg(cfg.vehicle == outVeh ? "VEHICLE UNCHANGED" : "SAVE + RESTART");
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

const MenuItem menuMain[] = {                // VEHICLE is not the first row on purpose: the cursor starts there
  S_ITEM("RC RATES", SUB_RC),
  S_ITEM("SELF-LEVEL", SUB_LEVEL),
  S_ITEM("LEVEL TRIM", SUB_TRIM),
  S_ITEM("GYRO PID", SUB_GYRO),
  S_ITEM("FILTERS", SUB_FILT),
  S_ITEM("DIRECTIONS", SUB_DIR),
  S_ITEM("BATTERY", SUB_BATT),
  A_ITEM("TX CHECK", ACT_TXTEST),
  A_ITEM("OUTPUT TEST", ACT_OUTTEST),
  A_ITEM("ESC CALIBRATE", ACT_ESCCAL),
  A_ITEM("ESC CAL AT BOOT", ACT_ESCBOOT),
  E_ITEM("VEHICLE", cfg.vehicle, NM_VEH, 2),
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

// Lower Hz = smoother but more delay. The title shows the live vibration level (VIB) to judge the effect.
const MenuItem menuFilt[] = {
  F_ITEM("GYRO LPF",       T.gyroLpf,        20, 200, 5, 0),     // same as in GYRO PID
  F_ITEM("DTERM LPF",      T.dtermLpf,       10, 100, 5, 0),     // same as in GYRO PID
  F_ITEM("NOTCH HZ 0=OFF", filt.notchHz,      0, 240, 5, 0),
  F_ITEM("NOTCH Q",        filt.notchQ,    0.5f, 10, 0.5f, 1),
  F_ITEM("ACC LPF",        filt.accLpfHz,     1, 100, 1, 0),
  F_ITEM("LEVEL D LPF",    filt.levelDLpfHz,  2, 100, 1, 0),
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
  F_ITEM("AIR START",     airStartThr,    1100, 1700, 10, 0),   // lift-off stick value: I terms + airmode start
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
  {"FILTERS",    menuFilt,  COUNT_OF(menuFilt)},
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
bool     escCalSerial = false;             // ESC calibration started from serial (see SERIAL CONFIGURATION)
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
        defaultExtras();                   // filters, RC smooth, I-term relax, anti gravity, air start
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
  bool gesture = USE_OLED && !armed && rcFresh && page != PAGE_SETUP && ch[2] < 1100 && ch[3] < 1300 && ch[1] > 1700 &&
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
    else if (sx < -0.45f && outVeh == VEH_PLANE) testDir = -1;
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
// Any problem (left SETUP, signal lost, ARM switch up, armed) -> outputs go back to safe.
bool escCalActive() {
  if (escCalSerial) return !armed && escCalStage >= 1 && !(rcValid && ch[4] >= 1500);   // started over serial (no TX needed)
  return page == PAGE_SETUP && screen == SCR_ESCCAL && !armed && rcValid && ch[4] < 1500 && escCalStage >= 1;
}

void driveEscCal() {
  int us = (escCalStage == 1) ? ESC_CAL_MAX : ESC_CAL_MIN;
  if (outVeh == VEH_QUAD) { for (int i = 0; i < 4; i++) writeUs(i, us); }
  else { outputsSafe(); writeUs(2, us); }
}

void driveTestOutput() {
  outputsSafe();
  int us;
  if (outVeh == VEH_QUAD)      us = (testDir > 0) ? 1000 + OUT_TEST_SPIN_US : 1000;
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

// ---- why did it disarm / restart? Kept in flash so it survives a brown-out or a power cycle ----
// Shown on the FC screen for 10 s after the next boot and printed on serial ("LAST STOP: ...").
void saveDisarmReason(const char *why) {
  char b[56];
  snprintf(b, sizeof b, "%s %.0fs gaps %u/%lums", why, fst.time, rcGapCount, (unsigned long)rcGapMaxMs);   // + flight time, PPM gaps
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
//   ANGLE:     roll or pitch beyond 70 deg for 0.1 s (SELF STAB only - in ACRO flips are intended)
//   VIBRATION: the fast vibration level jumps above 40 AND above 4x the flight's normal level for 0.25 s
//              (prop strike, broken or lost prop, failing motor)
// After it the ARM switch must be switched off and on again to re-arm.
#define CRASH_ANGLE_DEG  70.0f
#define CRASH_ANGLE_MS   100
#define CRASH_VIB_ABS    40.0f
#define CRASH_VIB_RATIO  4.0f
#define CRASH_VIB_MS     250
void crashDisarm(const char *why) {
  armed = false;
  armSwitchSeenLow = false;                 // re-arming needs the arm switch off -> on
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

// ACCELEROMETER TRIM WITH THE STICKS - removes steady drift in SELF STAB without the menu.
// Disarmed, throttle HIGH, yaw centred: push the right stick the way you want the craft to go INSTEAD of
// where it drifts (it drifts right -> push roll LEFT). Each 0.3 s held = 0.1 deg + a beep. Saved on release.
void trimStickTick() {
  static uint32_t next = 0;
  static bool changed = false;
  uint32_t now = millis();
  // SimpleTX's throttle tops out at its "Thr max" (~1700), so over ESP-NOW "high" starts lower
  bool cond = !armed && rcFresh && page != PAGE_SETUP && ch[2] > (RC_LINK_ESPNOW ? 1650 : 1800) && fabsf(stick(ch[3])) < 0.3f;
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

uint8_t armBlock = AB_NOLINK;               // reported to the TX (ESP-NOW): why arming is refused

void updateArming() {
  bool armSw = ch[4] > 1700;

  if (!rcValid)      armBlock = AB_NOLINK;
  else if (!imuOK)   armBlock = AB_IMU;
  else if (armed)    armBlock = AB_NONE;
  if (!rcValid || !imuOK) {
    if (armed) {
      Serial.println("DISARM: failsafe");
      setMsg(!rcValid ? "FAILSAFE: NO RC" : "FAILSAFE: IMU");
      alertEvent(0x15, 5);
      saveDisarmReason(!rcValid ? "RC LOST" : "IMU FAIL");
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
      saveDisarmReason(inFlight ? "ARM SW OFF IN AIR" : "NORMAL DISARM");   // so the boot report never shows an old stop reason
    }
    armed = false;
    armSwitchSeenLow = true;
    armBlock = AB_NONE;
    return;
  }
  if (armed) return;

  static uint32_t lastDeny = 0;
  const char *why = nullptr;
  if (!armSwitchSeenLow)                           { why = "ARM SW OFF FIRST"; armBlock = AB_SWITCH; }   // at boot, after a failsafe or a crash
  else if (!rcFresh)                               { why = "NO RC SIGNAL";     armBlock = AB_NOLINK; }
  else if (page == PAGE_SETUP)                     { why = "EXIT SETUP FIRST"; armBlock = AB_SETUP; }
  else if (serialBusy())                           { why = "SERIAL TEST/CAL";  armBlock = AB_SETUP; }   // output test / ESC cal over serial
  else if (cfg.vehicle != outVeh)                  { why = "VEHICLE: RESTART"; armBlock = AB_SETUP; }
  else if (levelCalLeft > 0)                       { why = "CALIBRATING";      armBlock = AB_CAL; }
  else if (ch[2] > 1100)                           { why = "THROTTLE NOT LOW"; armBlock = AB_THROTTLE; }
  else if (fabsf(roll) > 25 || fabsf(pitch) > 25)  { why = "TILT > 25 DEG";    armBlock = AB_TILT; }
  else                                             armBlock = AB_NONE;

  if (why) {
    if (millis() - lastDeny > 1000) { setMsg(why); Serial.printf("ARM DENIED: %s\n", why); lastDeny = millis(); alertEvent(0x15, 5); }
    return;
  }
  armed = true;
  resetPIDs();
  rcGapCount = 0; rcGapMaxMs = 0;           // new flight: reset the PPM-gap statistics
  setMsg("ARMED");
  alertEvent(0x3, 2);
  Serial.println("ARMED");
}

void updateMode() {
#if RC_LINK_ESPNOW
  // SimpleTX's MODE button sends CH6 = 1000 (TX shows SELF) or 1500 (TX shows ALT). Its SELF position counts as
  // "CH6 HIGH", so with the default DIRECTIONS > CH6 HIGH = SELF the TX label is right; its ALT position = ACRO here.
  bool high = (ch[5] < 1250);
#else
  bool high = (ch[5] >= 1500);
#endif
  mode = (high == (cfg.selfOnHigh != 0)) ? MODE_SELF : MODE_ACRO;
}

#if RC_LINK_ESPNOW
// Telemetry to the transmitter at 20 Hz: armed, mode, why arming is refused, attitude, battery, link quality.
// Sent unicast to the TX's MAC once a packet from it has been received.
void telemetryTick(uint32_t ms) {
  static uint32_t last = 0;
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
  memset(&t, 0, sizeof t);
  t.magic = PKT_MAGIC;
  t.bindId = BIND_ID;
  t.armed = armed;
  t.mode = (mode == MODE_ACRO) ? 1 : 0;
  t.armBlock = armed ? (uint8_t)AB_NONE : armBlock;
  t.roll10 = (int16_t)constrain(roll * 10.0f, -32000.0f, 32000.0f);
  t.pitch10 = (int16_t)constrain(pitch * 10.0f, -32000.0f, 32000.0f);
  t.vbat100 = battPresent ? (uint16_t)constrain(vBat * 100.0f, 0.0f, 65000.0f) : 0;
  t.cells = cellsUsed;
  t.flags = (battPresent ? 1 : 0) | (battLow ? 2 : 0) | 0x20;   // bit5: no barometer on this FC
  t.alt100 = 0;
  t.rssi = (int8_t)lroundf(rcRssiF);
  t.lq = rcLq;
  t.crc = crc8((const uint8_t *)&t, sizeof(t) - 1);
  esp_now_send(txMac, (const uint8_t *)&t, sizeof(t));
}
#endif

// ======================= CONTROL =======================
// Signal flow:
//   ACRO      : stick --RC rates-------------------------> rate setpoint --+
//   SELF STAB : stick --> target angle --SELF-LEVEL PID--> rate setpoint --+--> GYRO PID --> mixer
//   YAW       : always stick --RC rates--> rate setpoint
float recRollT = 0, recPitchT = 0;          // self-level target angles (deg), recorded by the flight recorder
float outRoll = 0, outPitch = 0, outYaw = 0;   // us
int   outThrottle = 1000;
float agBoostRec = 1.0f;                    // last anti-gravity I boost (diagnostics)
int   outUs[4] = {1000, 1000, 1000, 1000};

void runControl(float dt) {
  float thrIn = ch[2];
  bool quad = (outVeh == VEH_QUAD);

  // ---- stick positions, -1..+1 ----
  float sRoll  = stick(ch[0]) * rollStickSign;       // +right down
  float sPitch = stick(ch[1]) * pitchStickSign;      // +nose down
  float sYaw   = -stick(ch[3]);                      // stick right = CW = negative gz

  // RC SMOOTH (as Betaflight's RC smoothing): PPM delivers 50 new stick values a second but this loop runs
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
    recRollT = rollTarget; recPitchT = pitchTarget;  // for the flight recorder
    rollRateSet  = pidLevelRoll.update(rollTarget   - roll,  roll,  dt);
    pitchRateSet = pidLevelPitch.update(pitchTarget - pitch, pitch, dt);
  }

  outThrottle = (int)thrIn;

  // AIRMODE (quad, ACRO only): keep full roll/pitch/yaw authority even at minimum throttle, so flips and rolls can
  // start from idle. The mixer lifts all four motors together when one would drop below idle (see below).
  // The I terms stay at zero while the throttle is low, so a craft sitting on the ground cannot wind them up.
  // Airmode only becomes active once the throttle has been raised past AIR START since arming (Betaflight's
  // airmode_start_throttle). Before that the quad sits on the ground: full authority at idle there made it fight
  // the ground and tip over when the sticks moved (seen on the C3 FC in ACRO at 1100-1130 throttle).
  static bool liftedOff = false;
  if (!armed) liftedOff = false;
  else if (thrIn > airStartThr) liftedOff = true;
  bool airmode = quad && mode == MODE_ACRO && cfg.acroAirmode && liftedOff;
  bool lowThr  = thrIn < 1100;

  if (quad && lowThr && !airmode) {        // quad on the ground: idle, no stabilisation
    resetPIDs();
    outRoll = outPitch = outYaw = 0;
  } else {
    // I term only once the throttle has passed AIR START (lift-off): sitting on the ground with a stick held
    // (or on a slope) must not wind it up, or the quad jumps sideways at take-off. A plane has no lift-off gate.
    bool useI = (liftedOff || !quad) && !(airmode && lowThr);
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
  u8g2.drawStr(0, 63, "good: ~450 edges/s 50Hz");
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
  u8g2.drawStr(0, 63, rcValid ? "ALL TO BOTH ENDS YL=back" : "NO RC SIGNAL  YL=back");
}

void drawOutTest() {
  static const char *const QN[4] = {"M1 RR", "M2 FR", "M3 RL", "M4 FL"};
  static const char *const PN[4] = {"AILERON", "ELEVATOR", "THROTTLE", "RUDDER"};
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "OUT TEST PROPS OFF!");
  snprintf(b, sizeof b, "OUT %d: %s", testOutIdx + 1, outVeh == VEH_QUAD ? QN[testOutIdx] : PN[testOutIdx]);
  u8g2.drawStr(0, 28, b);
  if (ch[4] >= 1500)           u8g2.drawStr(0, 44, "LOWER THE ARM SWITCH");
  else if (outputTestActive()) { snprintf(b, sizeof b, "PULSE %d us", testUsNow); u8g2.drawStr(0, 44, b); }
  else                         u8g2.drawStr(0, 44, "idle: hold ROLL rt");
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 63, "P=select R=drive YL=back");
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
    u8g2.drawStr(0, 35, "PLUG IN battery,");
    u8g2.drawStr(0, 46, "wait for ESC beeps,");
    u8g2.drawStr(0, 57, "YAW RIGHT = send MIN");
  } else {
    u8g2.drawStr(0, 24, "MIN 1000us sent.");
    u8g2.drawStr(0, 35, "Wait: confirm beeps");
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
  } else if (curMenu == SUB_FILT) {          // live vibration level, to judge the filters
    snprintf(b, sizeof b, "V%4.1f", vibrationRms());
    u8g2.drawStr(98, 10, b);
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
  else if (cfg.vehicle != outVeh) u8g2.drawStr(0, 63, "VEHICLE: SAVE+RESTART");
  else                     u8g2.drawStr(0, 63, "UD=move LR=edit YR=ok");
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
  u8g2.drawStr(0, 9, inFlight ? "THIS FLT (d=est-acc)" : "LAST FLT (d=est-acc)");
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
#if !USE_OLED
  return;                                   // no screen: no pages to step through (and SETUP stays closed)
#endif
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
#if !USE_OLED
  (void)l1; (void)l2;
  return;
#endif
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, l1);
  u8g2.drawStr(0, 36, l2);
  u8g2.sendBuffer();
}

bool recFsOk = false;                       // flight recorder: LittleFS mounted, flights kept in flash after disarm

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

uint16_t loopTimeUs = 0, loopMaxUs = 0;     // whole loop pass (serial line)
uint16_t ctrlMaxUs = 0;                     // worst case of the flight-critical part of the loop

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.setTxBufferSize(1024);             // the status line must not wait for the UART while armed
  Serial.begin(115200);
  // (no delay before the outputs: ESCs powered at the same moment from the battery must get their signal right away)
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
    if (outVeh == VEH_QUAD) { for (int i = 0; i < 4; i++) writeUs(i, ESC_CAL_MAX); }
    else { outputsSafe(); writeUs(2, ESC_CAL_MAX); }
  } else {
    outputsSafe();                          // ESCs see 1000 us immediately
  }
  delay(300);                               // give a serial monitor time to attach (outputs are already set)
  Serial.println("ESP32 FC " FW_VERSION);

#if USE_BUZZER
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);
  for (uint8_t k = 0; k < 2; k++) {         // power-on buzzer self-test, always, whatever the saved settings say
    digitalWrite(PIN_BUZZ, HIGH); delay(80);
    digitalWrite(PIN_BUZZ, LOW);  delay(80);
  }
#endif

  Wire.begin(PIN_SDA, PIN_SCL);             // before u8g2.begin() so our pins are kept
  Wire.setClock(400000);
#if USE_OLED
  u8g2.begin();
  u8g2.setBusClock(400000);
#endif

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

#if RC_LINK_ESPNOW
  radioBegin();                             // ESP-NOW link to SimpleTX (the WiFi driver runs on core 0)
#elif PPM_ON_CORE0 && !CONFIG_FREERTOS_UNICORE
  // An interrupt runs on the core that attached it. Attach from a short task pinned to core 0.
  xTaskCreatePinnedToCore(ppmAttachTask, "ppmAttach", 4096, NULL, 1, NULL, 0);
  for (uint8_t w = 0; w < 100 && !ppmAttached; w++) delay(2);
#else
  ppmAttachTask(NULL);
#endif
#if !RC_LINK_ESPNOW
  Serial.printf("PPM interrupt on IO%u, attached from core %d (flight loop on core %d)\n",
                (unsigned)PIN_PPM, (int)ppmAttachCore, (int)xPortGetCoreID());
#endif
  resetPIDs();
  alertEvent(0x1, 3);                       // one beep: ready
  Serial.printf("Vehicle: %s. RC link: %s.\n", cfg.vehicle == VEH_QUAD ? "QUAD" : "PLANE",
                RC_LINK_ESPNOW ? "ESP-NOW (SimpleTX)" : "PPM on IO32");
#if USE_OLED
  Serial.println("Open SETUP: PAGE button (IO13), or throttle low + yaw left + pitch forward for 1 s.");
#endif
  Serial.println("Settings: type ? and Enter in the serial monitor (115200, newline).");
  bootReportReasons();                      // why the last flight ended (disarm reason or a brown-out restart)
  recFsOk = LittleFS.begin(true);           // first boot formats the storage area once (a few seconds)
  Serial.printf("Flight recorder storage: %s. Send D (disarmed) to print the last flight.\n",
                recFsOk ? "ok" : "NOT AVAILABLE (RAM only)");
}

// ======================= FLIGHT RECORDER =======================
// While armed, 50 times a second, into RAM (ring buffer, the last REC_N samples = ~32 s). Cleared at every arm,
// saved to flash (LittleFS) right after a disarm that ended a real flight. Read it out over USB: send the letter D
// (disarmed) and it prints a CSV - from RAM, or from flash after a restart.
#define REC_N  1600
#define REC_MS 20
struct __attribute__((packed)) RecSample {
  uint16_t t10;                 // time since arming, 10 ms units
  int16_t  roll10, pitch10;     // attitude estimate, deg x10
  int16_t  rollT10, pitchT10;   // self-level targets, deg x10
  int16_t  gR, gP, gY;          // gyro rates, deg/s
  uint16_t thr, rcR, rcP, rcY;  // sticks, us
  uint16_t m[4];                // outputs, us
  uint16_t vbat100;             // pack voltage x100
  uint16_t ctrlUs;              // flight-critical loop time
  uint8_t  flags;               // bit 0 = SELF STAB, bit 1 = signal stale (sticks centred)
  uint8_t  vib10;               // vibration level x10
  uint8_t  ag10;                // anti-gravity I boost x10
  int16_t  accR10, accP10;      // angles from the accelerometer alone, deg x10
};
RecSample recBuf[REC_N];
uint16_t recHead = 0, recCount = 0;
uint32_t recStartMs = 0, recLastMs = 0;
bool     recWasArmed = false;
bool     recHadFlight = false;     // this arm actually flew (inFlight was set) - only then is it saved

// Save the flight to flash right after disarming (motors already off), oldest sample first, so it survives
// a restart or a power cycle. Takes well under a second.
void recSaveToFlash() {
  // Only real flights (at least 5 s armed AND the throttle raised): an arm on the ground must not overwrite the
  // last flight
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
  s.flags = (uint8_t)((mode == MODE_SELF ? 1 : 0) | (rcFresh ? 0 : 2));
  s.vib10 = (uint8_t)constrain(vibrationRms() * 10, 0.0f, 255.0f);
  s.ag10 = (uint8_t)constrain(agBoostRec * 10, 0.0f, 255.0f);
  s.accR10 = (int16_t)constrain(accRollDeg * 10, -32000.0f, 32000.0f);
  s.accP10 = (int16_t)constrain(accPitchDeg * 10, -32000.0f, 32000.0f);
  recHead = (recHead + 1) % REC_N;
  if (recCount < REC_N) recCount++;
}

void recDump() {                                  // CSV over USB, oldest sample first (disarmed only)
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
  Serial.println("t_s,roll,pitch,rollT,pitchT,gR,gP,gY,thr,rcR,rcP,rcY,m1,m2,m3,m4,vbat,ctrl_us,self,stale,vib,ag,accR,accP");
  uint16_t start = (recCount < REC_N) ? 0 : recHead;
  for (uint16_t k = 0; k < recCount; k++) {
    const RecSample &s = recBuf[(start + k) % REC_N];
    Serial.printf("%.2f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%u,%u,%u,%.1f,%.1f,%.1f,%.1f\n",
                  s.t10 / 100.0f, s.roll10 / 10.0f, s.pitch10 / 10.0f, s.rollT10 / 10.0f, s.pitchT10 / 10.0f,
                  s.gR, s.gP, s.gY, s.thr, s.rcR, s.rcP, s.rcY, s.m[0], s.m[1], s.m[2], s.m[3],
                  s.vbat100 / 100.0f, s.ctrlUs, s.flags & 1, (s.flags >> 1) & 1, s.vib10 / 10.0f, s.ag10 / 10.0f,
                  s.accR10 / 10.0f, s.accP10 / 10.0f);
  }
  Serial.println("REC END");
}

// ======================= SERIAL CONFIGURATION =======================
// Every setting of the SETUP menu can be read and changed over the serial monitor (115200 baud, line ending
// "Newline"). This is how the board is configured when no OLED is fitted. Disarmed only. Type ? for the list.
// Setting names are made from the menu: <group>.<label>, e.g. gyro.roll_p, rc.yaw_max, filt.notch_hz; the main-menu
// value is just "vehicle". Changes act at once; "save" keeps them.
static const char *const MENU_KEY[SUB_COUNT] = {"", "rc", "level", "gyro", "dir", "trim", "batt", "filt"};

bool     logOn = true;                     // periodic status line (5 per second)
uint32_t logPauseUntil = 0;                // paused for a while after a command, so its answer stays readable
uint32_t rcCheckUntil = 0;                 // "rc" command: TX check lines instead of the status line
uint32_t confirmDefaultsSerial = 0;
// serial output test: one output driven for a few seconds
int8_t   serTestIdx = -1;
int      serTestUs = 1000;
uint32_t serTestUntil = 0;

bool serialTestActive() {
  return serTestIdx >= 0 && !armed && millis() < serTestUntil && !(rcValid && ch[4] >= 1500);
}
bool serialBusy() { return serialTestActive() || (escCalSerial && escCalStage > 0); }

// "ROLL CENTER" -> "roll_center"; words containing '=' ("0=OFF", "=") are dropped: "CELLS 0=AUTO" -> "cells"
void slugLabel(const char *s, char *out, size_t n) {
  size_t k = 0;
  bool sep = false;
  while (*s && k + 1 < n) {
    const char *w = s;                       // one word
    while (*s && *s != ' ') s++;
    bool skip = false;
    for (const char *p = w; p < s; p++) if (*p == '=') skip = true;
    if (!skip && s > w) {
      if (sep && k + 1 < n) out[k++] = '_';
      for (const char *p = w; p < s && k + 1 < n; p++) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
        out[k++] = ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) ? c : '_';
      }
      sep = true;
    }
    while (*s == ' ') s++;
  }
  out[k] = 0;
}

void itemKey(uint8_t m, uint8_t i, char *out, size_t n) {
  char lab[24];
  slugLabel(menus[m].items[i].label, lab, sizeof lab);
  if (MENU_KEY[m][0]) snprintf(out, n, "%s.%s", MENU_KEY[m], lab);
  else snprintf(out, n, "%s", lab);
}

void printItem(uint8_t m, uint8_t i) {
  const MenuItem &it = menus[m].items[i];
  char key[36];
  itemKey(m, i, key, sizeof key);
  if (it.type == IT_FLOAT) {
    Serial.printf("  %-22s = %.*f   (%.*f..%.*f)\n", key, (int)it.dec + 1, *(float *)it.ptr,
                  (int)it.dec, it.minV, (int)it.dec, it.maxV);
  } else if (it.type == IT_ENUM) {
    Serial.printf("  %-22s = %s   (", key, it.names[*(uint8_t *)it.ptr]);
    for (int v = 0; v <= (int)it.maxV; v++) Serial.printf("%s%s", v ? "/" : "", it.names[v]);
    Serial.println(")");
  }
}

void printSettings(const char *group) {    // group "" = all
  for (uint8_t m = 0; m < SUB_COUNT; m++) {
    if (group[0] && strcasecmp(group, MENU_KEY[m]) != 0 && !(m == 0 && strcasecmp(group, "main") == 0)) continue;
    bool any = false;
    for (uint8_t i = 0; i < menus[m].n; i++) if (menus[m].items[i].type == IT_FLOAT || menus[m].items[i].type == IT_ENUM) any = true;
    if (!any) continue;
    Serial.printf("[%s] %s\n", m ? MENU_KEY[m] : "main", menus[m].title);
    for (uint8_t i = 0; i < menus[m].n; i++) printItem(m, i);
  }
  if (dirty) Serial.println("(changed, not saved: type save)");
  if (cfg.vehicle != outVeh) Serial.println("(vehicle changed: save, then reboot)");
}

bool setItem(const char *key, const char *val) {
  for (uint8_t m = 0; m < SUB_COUNT; m++) {
    for (uint8_t i = 0; i < menus[m].n; i++) {
      const MenuItem &it = menus[m].items[i];
      if (it.type != IT_FLOAT && it.type != IT_ENUM) continue;
      char k[36];
      itemKey(m, i, k, sizeof k);
      if (strcasecmp(k, key) != 0) continue;
      if (it.type == IT_FLOAT) {
        char *end;
        float v = strtof(val, &end);
        if (end == val || !(v == v)) { Serial.printf("'%s' is not a number\n", val); return true; }
        if (v < it.minV || v > it.maxV) Serial.printf("limited to %.*f..%.*f\n", (int)it.dec, it.minV, (int)it.dec, it.maxV);
        *(float *)it.ptr = constrain(v, it.minV, it.maxV);
        afterEdit();
      } else {
        int n = (int)it.maxV + 1, v = -1;
        for (int j = 0; j < n; j++) if (strcasecmp(val, it.names[j]) == 0) v = j;
        if (v < 0 && val[0] >= '0' && val[0] <= '9') v = atoi(val);
        if (v < 0 || v >= n) { Serial.printf("'%s' is not one of the choices\n", val); printItem(m, i); return true; }
        uint8_t *p = (uint8_t *)it.ptr, old = *p;
        *p = (uint8_t)v;
        if (it.ptr == (void *)&cfg.vehicle) applyVehicleChange(old);
        else afterEdit();
      }
      printItem(m, i);
      if (it.ptr == (void *)&cfg.vehicle && cfg.vehicle != outVeh) Serial.println("Vehicle takes effect after: save, then reboot.");
      return true;
    }
  }
  return false;
}

void printHelp() {
  Serial.println(F(
    "---- FC serial commands (disarmed only) ----\n"
    "  list [group]        all settings with ranges (groups: main rc level gyro dir trim batt filt)\n"
    "  <name> <value>      change one, e.g.  gyro.roll_p 0.45   dir.props in   rc.yaw_max 360\n"
    "  save                keep the changes in flash          defaults yes   load the defaults (not saved)\n"
    "  status              mode, link, sticks, battery, attitude, trims\n"
    "  sensors             accelerometer / estimate / vibration / gyro bias\n"
    "  stats               last flight statistics\n"
    "  rc                  TX check for 15 s: move every stick + switch to both ends\n"
    "  levelcal            level calibration (craft on a level surface, still)\n"
    "  trimclear           roll/pitch trim back to 0\n"
    "  test <1-4> [us]     PROPS OFF! drive one output for 3 s (motor default 1150, servo 1800)\n"
    "  esccal              PROPS OFF! ESC calibration: sends MAX, then 'esccal min', 'esccal stop'\n"
    "  esccal boot         ESC calibration at the next power-up (FC powered from the flight battery)\n"
    "  buzz                buzzer test\n"
    "  log on|off          the status line 5 times a second\n"
    "  d                   print the flight recorder (CSV)\n"
    "  reboot              restart the FC (needed after changing the vehicle)"));
}

void printStatus() {
  Serial.printf("FC %s | %s %s %s | link %s %s\n", FW_VERSION, outVeh == VEH_QUAD ? "QUAD" : "PLANE",
                armed ? "ARMED" : "DISARMED", mode == MODE_ACRO ? "ACRO" : "SELF STAB",
                RC_LINK_ESPNOW ? "ESP-NOW" : "PPM", rcFresh ? "OK" : (rcValid ? "STALE" : "LOST"));
#if RC_LINK_ESPNOW
  Serial.printf("  packets %u/s  LQ %u%%  RSSI %+.0f dBm  TX %s\n", ppmHz, rcLq, rcRssiF, txMacKnown ? "locked" : "not heard yet");
#else
  Serial.printf("  PPM %u ch, %u frames/s, %lu edges/s, %lu bad frames\n", ppmStable, ppmHz, (unsigned long)ppmEdgeRate, (unsigned long)ppmBadSeen);
#endif
  Serial.printf("  CH1 roll %u  CH2 pitch %u  CH3 thr %u  CH4 yaw %u  CH5 arm %u  CH6 mode %u\n", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);
  Serial.printf("  roll %+.1f pitch %+.1f deg | trim R %+.1f P %+.1f | IMU %s | VIB %.1f\n", roll, pitch, cfg.rollTrim, cfg.pitchTrim,
                imuOK ? "ok" : "FAULT", vibrationRms());
  if (battPresent) Serial.printf("  battery %.2f V, %u cells, %.2f V/cell%s\n", vBat, cellsUsed, cellVolts(), battLow ? " LOW" : "");
  else Serial.println("  battery: none");
  if (dirty) Serial.println("  (changed settings not saved)");
  if (cfg.vehicle != outVeh) Serial.println("  (vehicle changed: save + reboot)");
}

void printSensors() {
  Serial.printf("ACC roll %+.1f pitch %+.1f | EST roll %+.1f pitch %+.1f | |a| %.2f g | VIB %.1f (fast %.1f)\n",
                accRollDeg, accPitchDeg, roll, pitch, accMag, vibrationRms(), vibrationFast());
  Serial.printf("gyro %+.1f %+.1f %+.1f deg/s | bias %+.2f %+.2f %+.2f | VBAT pin %.0f mV = %.2f V\n",
                gyro[0], gyro[1], gyro[2], gyroBias[0], gyroBias[1], gyroBias[2], vPinMv, vBat);
}

void printStats() {
  if (!fstValid) { Serial.println("No flight yet: arm and raise the throttle."); return; }
  float n = fst.n ? (float)fst.n : 1.0f, nv = fst.nValid ? (float)fst.nValid : 1.0f;
  Serial.printf("%s: %.1f s | VIB avg %.1f peak %.1f | accel rejected %.0f%% | d=est-acc roll %+.1f (|d| %.1f) pitch %+.1f (|d| %.1f) | min batt %.2f V\n",
                inFlight ? "THIS FLIGHT" : "LAST FLIGHT", fst.time, fst.vibSum / n, fst.vibPeak, 100.0f * (fst.n - fst.nValid) / n,
                fst.sumDR / nv, fst.sumAbsDR / nv, fst.sumDP / nv, fst.sumAbsDP / nv, fst.vMin);
}

void serialCommand(char *line) {
  // split "name value" or "name=value"
  for (char *p = line; *p; p++) if (*p == '=' || *p == '\t') *p = ' ';
  char *cmd = line;
  while (*cmd == ' ') cmd++;
  if (!*cmd) return;
  char *arg = cmd;
  while (*arg && *arg != ' ') arg++;
  if (*arg) { *arg++ = 0; while (*arg == ' ') arg++; }
  char *e = arg + strlen(arg);
  while (e > arg && e[-1] == ' ') *--e = 0;
  if (strcasecmp(cmd, "set") == 0) {        // "set name value"
    cmd = arg;
    arg = cmd;
    while (*arg && *arg != ' ') arg++;
    if (*arg) { *arg++ = 0; while (*arg == ' ') arg++; }
  }
  logPauseUntil = millis() + 10000;
  rcCheckUntil = 0;

  if (armed) { Serial.println("ARMED: disarm first (commands are refused in flight)."); return; }

  if (!strcmp(cmd, "?") || !strcasecmp(cmd, "help"))   printHelp();
  else if (!strcasecmp(cmd, "list") || !strcasecmp(cmd, "p")) printSettings(arg);
  else if (!strcasecmp(cmd, "save"))     Serial.println(saveParams() ? "Saved." : "SAVE FAILED");
  else if (!strcasecmp(cmd, "defaults")) {
    if (!strcasecmp(arg, "yes")) {
      uint8_t v = cfg.vehicle;
      float keepRatio = cfg.vRatio, keepCells = cfg.cells;   // the voltage calibration belongs to the wiring, not the tune
      defaultCfg(cfg);
      cfg.vehicle = v;
      cfg.vRatio = keepRatio;  cfg.cells = keepCells;
      defaultTune(T, v);
      defaultExtras();
      afterEdit();
      Serial.println("Defaults loaded (battery ratio + cells kept). Not saved: type save to keep them.");
    } else Serial.println("Type: defaults yes");
  }
  else if (!strcasecmp(cmd, "status") || !strcasecmp(cmd, "s")) printStatus();
  else if (!strcasecmp(cmd, "sensors")) printSensors();
  else if (!strcasecmp(cmd, "stats"))   printStats();
  else if (!strcasecmp(cmd, "rc"))      { resetTxCheck(); rcCheckUntil = millis() + 15000; logPauseUntil = 0;
                                          Serial.println("TX CHECK 15 s: move every stick and switch to both ends."); }
  else if (!strcasecmp(cmd, "levelcal")) { startLevelCal(); Serial.println("Level calibration: keep the craft level and still..."); }
  else if (!strcasecmp(cmd, "trimclear")) { cfg.rollTrim = cfg.pitchTrim = 0; afterEdit(); Serial.println("Trim 0/0 (not saved)."); }
  else if (!strcasecmp(cmd, "buzz"))    { alertTest(); Serial.println("Buzzer: 3 beeps."); }
  else if (!strcasecmp(cmd, "log"))     { logOn = strcasecmp(arg, "off") != 0; logPauseUntil = 0; Serial.printf("Status line %s.\n", logOn ? "on" : "off"); }
  else if (!strcasecmp(cmd, "d") || !strcasecmp(cmd, "dump")) recDump();
  else if (!strcasecmp(cmd, "reboot"))  { Serial.println("Restarting..."); Serial.flush(); outputsSafe(); delay(50); ESP.restart(); }
  else if (!strcasecmp(cmd, "test")) {
    int n = atoi(arg);
    const char *u = arg;
    while (*u && *u != ' ') u++;
    if (n < 1 || n > 4) { Serial.println("test <1-4> [us]"); return; }
    if (rcValid && ch[4] >= 1500) { Serial.println("Lower the ARM switch first."); return; }
    bool motor = (outVeh == VEH_QUAD) || n == 3;                 // plane output 3 = throttle
    int us = *u ? atoi(u) : (motor ? 1000 + OUT_TEST_SPIN_US : 1800);
    us = motor ? constrain(us, 1000, 1300) : constrain(us, 1000, 2000);   // motors: never more than 1300 us here
    serTestIdx = n - 1; serTestUs = us; serTestUntil = millis() + 3000;
    static const char *const QN[4] = {"M1 RR", "M2 FR", "M3 RL", "M4 FL"};
    static const char *const PN[4] = {"AILERON", "ELEVATOR", "THROTTLE", "RUDDER"};
    Serial.printf("OUTPUT %d (%s, IO%u) at %d us for 3 s. PROPS OFF!\n", n, outVeh == VEH_QUAD ? QN[n - 1] : PN[n - 1], OUT_PIN[n - 1], us);
  }
  else if (!strcasecmp(cmd, "esccal")) {
    if (!strcasecmp(arg, "boot")) {
      if (prefs.begin("fc", false)) { prefs.putUChar("esccal", 1); prefs.end(); }
      Serial.println("ESC calibration armed for the next power-up. PROPS OFF! Unplug and replug the battery.");
    } else if (!strcasecmp(arg, "min")) {
      if (escCalSerial && escCalStage == 1) { escCalStage = 2; escCalMinSince = millis(); Serial.println("ESC CAL: MIN 1000 us sent. Wait for the confirm beeps, then 'esccal stop'."); }
      else Serial.println("Start with: esccal");
    } else if (!strcasecmp(arg, "stop")) {
      escCalSerial = false; escCalStage = 0; Serial.println("ESC calibration ended, outputs safe.");
    } else {
      if (rcValid && ch[4] >= 1500) { Serial.println("Lower the ARM switch first."); return; }
      escCalSerial = true; escCalStage = 1;
      Serial.println("ESC CAL: MAX 2000 us now. PROPS OFF! With the battery UNPLUGGED before this, plug it in now,");
      Serial.println("wait for the ESC beeps, then type: esccal min");
    }
  }
  else if (!setItem(cmd, arg)) Serial.printf("Unknown: '%s'. Type ? for help, list for the settings.\n", cmd);
}

void serialTick() {                         // read whole lines from the serial monitor
  static char buf[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (n) { buf[n] = 0; n = 0; serialCommand(buf); }
    } else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

void printRcCheck() {                       // "rc" command: live / min / max of CH1..6
  static const char NAMES[6] = {'R', 'P', 'T', 'Y', 'A', 'M'};
  for (int i = 0; i < 6; i++) {
    if (ch[i] < chMin[i]) chMin[i] = ch[i];
    if (ch[i] > chMax[i]) chMax[i] = ch[i];
  }
  Serial.print(rcValid ? "TX " : "TX NO SIGNAL ");
  for (int i = 0; i < 6; i++) {
    bool ok = (chMin[i] <= 1100 && chMax[i] >= 1900);
    Serial.printf("| %c%d %4u [%4u..%4u] %s ", NAMES[i], i + 1, ch[i], chMin[i], chMax[i], ok ? "OK" : "--");
  }
  Serial.println();
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
  crashCheck(dt, millis());                 // disarm on > 70 deg (not ACRO) or a sudden vibration jump
  trimStickTick();                          // accelerometer trim with the sticks (disarmed, throttle high)
  alertTick(millis());
  menuTick();                               // stick-driven setup menu (does nothing while armed)

  if (armed) {
    runControl(dt);
    for (int i = 0; i < 4; i++) writeUs(i, outUs[i]);
  } else {
    resetPIDs();
    if (outputTestActive()) driveTestOutput();
    else if (serialTestActive()) { outputsSafe(); writeUs(serTestIdx, serTestUs); }
    else if (escCalActive()) driveEscCal();
    else outputsSafe();
  }
  if (serTestIdx >= 0 && !serialTestActive()) { serTestIdx = -1; Serial.println("Output test ended, outputs safe."); }
  if (escCalSerial && escCalStage == 2 && millis() - escCalMinSince > 5000) {
    escCalSerial = false; escCalStage = 0;
    Serial.println("ESC CAL: done (MIN held 5 s). Outputs safe.");
  }

  uint16_t ctrlUs = (uint16_t)(micros() - now);   // flight-critical part: sensors -> control -> outputs
  if (ctrlUs > ctrlMaxUs) ctrlMaxUs = ctrlUs;
  recTick(millis(), ctrlUs);                // flight recorder (armed only, 50 Hz)
  serialTick();                             // serial configuration (commands are refused while armed)
#if RC_LINK_ESPNOW
  telemetryTick(millis());                  // armed / mode / arm refusal / battery back to SimpleTX
#endif

  // ----- UI (runs after outputs are written) -----
  // The OLED goes over the I2C bus shared with the IMU and drawing a page costs milliseconds, which made the
  // loop late and uneven. While ARMED the screen is frozen (nobody reads it in flight); it continues on disarm.
  handleButton();
  uint32_t ms = millis();
#if USE_OLED
  if (!armed) {
    oledPushSlice();
    if (ms - lastDraw >= 100) { lastDraw = ms; renderFrame(); }
  }
#else
  (void)lastDraw;
#endif

#if !DIAG_NO_SERIAL
  if (rcCheckUntil && ms - lastPrint >= 200) {   // "rc" command running
    lastPrint = ms;
    if (ms > rcCheckUntil) { rcCheckUntil = 0; Serial.println("TX check ended."); }
    else printRcCheck();
  }
  if (logOn && !rcCheckUntil && ms > logPauseUntil && ms - lastPrint >= 200) {
    lastPrint = ms;
    Serial.printf("LOOP %4uus max %4uus CTRL max %4uus | ", loopTimeUs, loopMaxUs, ctrlMaxUs);
    loopMaxUs = 0;
    ctrlMaxUs = 0;
    Serial.printf("%s %s %s R:%6.1f P:%6.1f VIB:%4.1f V:%5.2f | ",
                  cfg.vehicle == VEH_QUAD ? "QUAD " : "PLANE", armed ? "ARMED " : "DISARM",
                  mode == MODE_ACRO ? "ACRO" : "SELF", roll, pitch, vibrationRms(), vBat);
#if RC_LINK_ESPNOW
    Serial.printf("LINK %upk/s LQ %u%% %+.0fdBm | ", (unsigned)ppmHz, (unsigned)rcLq, rcRssiF);
#else
    Serial.printf("PPM %uch %uHz bad %lu | ", (unsigned)ppmStable, (unsigned)ppmHz, (unsigned long)ppmBadSeen);
#endif
    if (rcValid) for (int i = 0; i < 6; i++) Serial.printf("%4u ", ch[i]);
    else Serial.print("NO RC");
    if (rcValid && !rcFresh) Serial.print(" STALE");
    if (armed) Serial.printf(" | AG x%.2f", agBoostRec);
    Serial.println();
  }
#else
  (void)lastPrint;
#endif
  loopTimeUs = (uint16_t)(micros() - now);
  if (loopTimeUs > loopMaxUs) loopMaxUs = loopTimeUs;       // worst case, shown in the serial line (reset 5x/s)
}
