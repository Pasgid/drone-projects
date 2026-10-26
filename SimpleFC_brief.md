# Brief: simple FC + transmitter (self-level + altitude hold)

Paste this into a new chat. It sums up what was learned building the full FlightController / Transmitter2
projects in `E:\Claude codes\`. The new build should be a clean, small rewrite, not a copy.

## Goal
Two new sketches, new folders (e.g. `E:\Claude codes\SimpleFC\` and `E:\Claude codes\SimpleTX\`):
- **FC**: quad X, ESP32-C3 Zero, MPU6050 (I2C), BMP280 (I2C), small I2C OLED for status, PWM ESCs.
  Modes: **SELF-LEVEL** and **ALT HOLD (barometer)** only. No MTF, no VL53L1X, no position hold, no acro.
- **TX**: ESP32-C3 Zero, 2 gimbals (4 pots), arm push button, 3-position switch, SH1106 1.3" OLED, passive buzzer.
- Link: ESP-NOW.

## Hardware / pins (same as the current boards)
FC (ESP32-C3 Zero):
- Motors (PWM out): M1 GPIO0, M2 GPIO1, M3 GPIO3, M4 GPIO8 — Betaflight order: M1 rear-right, M2 front-right,
  M3 rear-left, M4 front-left, props in.
- I2C: SDA 6, SCL 7, 400 kHz (1 MHz was tried and reverted). MPU6050 0x68, BMP280 0x76, OLED (SSD1306/SH1106) 0x3C.
- Battery divider: GPIO4. Buzzer: GPIO20. BOOT button GPIO9. **GPIO2 is unusable on this board (shorted to the
  crystal can).** GPIO10 drives the onboard RGB LED.
TX (ESP32-C3 Zero):
- Pots: THR GPIO1, ROLL GPIO3, PITCH GPIO4, YAW GPIO0. OLED SH1106 SDA 5, SCL 6. Arm button GPIO7.
- 3-pos switch digital: GPIO20 (POS1 leg), GPIO8 (POS3 leg), middle = neither. Buzzer GPIO21.
- GPIO2 battery pin is shorted on this board too (TX battery reading did not work).
Power: **do not use a 7805 from 3S** — it overheated and browned out the FC in flight. Use a 5 V buck/BEC
(set to 5.0 V before connecting) + 470-1000 uF on the battery leads + 100-470 uF at the FC 5 V input.
Antenna: ceramic chip removed, 31 mm wire on the feed pad (RSSI went from about -80 to about -45 dBm). Never
leave the chip antenna on with a wire added (that made it ~20 dB worse).

## Toolchain
- Arduino core esp32 3.3.x, FQBN `esp32:esp32:esp32c3:CDCOnBoot=cdc`.
- arduino-cli: `$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`,
  env `ARDUINO_DIRECTORIES_DATA=$env:LOCALAPPDATA\Arduino15`, `ARDUINO_DIRECTORIES_USER=$env:TEMP\c3sb`
  (libraries there: Adafruit SH110X/GFX/SSD1306, U8g2). Use short build paths (`--build-path $env:TEMP\...`).
- FC was COM23, TX COM24.
- Arduino IDE auto-prototypes: declare structs/enums before the first function.
- `Serial.setTxTimeoutMs(0)` so USB prints never block the loop; only print when `Serial` is connected.

## Radio (keep exactly this — it fixed in-flight disarms)
- ESP-NOW on **channel 13**, `esp_wifi_set_country` EU 1-13, `esp_wifi_set_max_tx_power(80)` (full power).
- TX broadcasts until it receives telemetry, then sends **unicast** to the FC MAC (acknowledged + retried).
- ControlPacket: magic 0xC3F1, bindId 42, seq, 6 channels (THR, ROLL, PITCH, YAW, ARM, CH6), crc8.
- TelemetryPacket (20 Hz): armed, mode, armBlock, roll/pitch, vbat, cells, flags, **rssi (dBm), lq (%)**, crc8.
- TX 50 packets/s. FC LQ = packets in last second x2. Show RSSI/LQ on both OLEDs.
- Failsafe, two stages: gap > 250 ms -> sticks centred, throttle and mode kept (alt hold keeps holding);
  gap > 1 s -> **auto-land** (alt hold, descend at max descent, disarm on the ground or after 15 s) if the
  altitude is valid; otherwise disarm. Radio back -> control returned. Count gaps for the stop report.
- TX: arming = button only; telemetry loss only warns ("NO TELEMETRY"), it never disarms.

## Control loop
- 400 Hz loop, MPU6050 DLPF, gyro LPF + D-term LPF (biquad), optional static notch.
- Attitude: gyro integration + accelerometer correction, accel ignored when |a| far from 1 g, slow accel time
  constant in flight (ACC TAU FLT ~3 s). Gyro bias calibrated at boot (keep still) and tracked while disarmed.
- Self-level: angle P (+ small I) -> rate setpoint -> rate PID (P, I, D on measurement) -> mixer.
- Proven extras (keep): **RC smoothing** (2-stage PT1 ~20 Hz on sticks, the radio is only 50 Hz),
  **I-term relax** (roll/pitch, 40 deg/s, 15 Hz setpoint HPF), **anti-gravity** (I boost on fast throttle change,
  roll/pitch only), **I term held at 0 until the throttle has passed the hover point once** (no ground wind-up).
- Trims: roll/pitch trim subtracted from the accelerometer angles (a 3 deg lean was found and trimmed out).

## Altitude hold (barometer)
- 3rd-order complementary filter: accel (rotated to earth-up, gravity removed, bias learned while disarmed and
  still) + baro (BMP280, x16 IIR, extra LPF ~1.5 Hz). Baro tau ~2.5 s.
- Controller: target height -> ALT P -> velocity target -> VEL P / VEL I -> throttle = hover feed-forward + PID,
  tilt-compensated. Values that worked (rangefinder build): ALT P 1.0, VEL P 120, VEL I 30, max climb 1.0 m/s,
  max descent 0.7 m/s, stick deadband 80 us, min throttle 1150.
- **Hold point = learned hover throttle**, not mid-stick: stick within +-deadband of the hover point = hold,
  above = climb, below = descend. Hover throttle learned in SELF (stick, steady hover) and in ALT HOLD
  (throttle needed), saved to flash at disarm.
- **Stick moves the TARGET** at the commanded rate (target kept within +-0.5 m of the estimate); never drop the
  target, or it drifts when the stick sits just outside the zone.
- **Battery throttle compensation** (iNav thr_comp): work in reference throttle at 3.85 V/cell, output x
  (Vref/Vbat, filtered, clamped 0.85-1.25); hover throttle stored as a reference value.
- Landing: stick fully down and still for 0.8 s -> "LANDED", motors idle; bleed the integrator when landing so
  the motors do not stay at hover thrust on the ground.
- Engage only on a switch move; ground start in ALT HOLD = idle until stick above the hover point.
- Expect a barometer-only hold to be looser than the rangefinder one (~+-0.2-0.3 m indoors, prop wash).
- Ceiling (optional): check against the fused estimate, never the raw baro (raw baro drift pushed it down once).

## Modes / switch / arming
- CH6: POS1 = SELF-LEVEL, POS2 = ALT HOLD, POS3 = ALT HOLD as well (or unused).
- Arm: TX button, throttle low, level; must see the arm channel low after a failsafe.
- Crash cut-off: tilt > 70 deg for 100 ms, or sudden vibration jump (> 40 and > 4x the flight's baseline for
  250 ms).

## Displays
- FC OLED: frozen while armed (drawing costs loop time); shows mode, battery, RSSI, alt, arm block reason.
- TX OLED: armed/safe, mode, RSSI + LQ (weaker direction, blinks when < -85 dBm or < 60 %), FC battery,
  channel bars. Stick calibration screen.

## Diagnostics to keep
- Serial status line (only when USB connected): loop/ctrl time, attitude, vibration, battery, channels,
  RSSI/LQ, alt estimate, hold target/throttle.
- Stop reason saved to NVS on every disarm (normal, radio lost, crash, brownout reset) and printed at boot.
- Flight recorder: ~1600 samples at 50 Hz in RAM, saved to LittleFS only for arms that actually flew; 'D' over
  serial dumps CSV. Fields: t, roll, pitch, targets, gyro, sticks, motors, vbat, ctrl_us, alt, mode, vib, rssi, lq.

## Lessons / pitfalls
- Loop overruns made the buzzer beep fast; serial printing blocked the loop once (OLED froze).
- Filters set too low caused a slow growing wobble.
- Keep wires (balance lead!) away from sensors and props; loose items raised vibration and broke sensors.
- After a crash: check props, motor shafts and mounts — idle vibration jumped from ~2 to 10-22.
- Airmode/ACRO on the ground flipped the quad — not needed in this build.
