# SimpleFC + SimpleTX

Self-level and barometer altitude hold for the ESP32-C3 quad. Sketches: `SimpleFC/SimpleFC.ino` and `../SimpleTX/SimpleTX.ino`.

## Build / upload
```
$cli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$env:ARDUINO_DIRECTORIES_DATA = "$env:LOCALAPPDATA\Arduino15"; $env:ARDUINO_DIRECTORIES_USER = "$env:TEMP\c3sb"
& $cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --build-path $env:TEMP\sfcb -u -p COM23 "E:\Claude codes\SimpleFC"
& $cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --build-path $env:TEMP\stxb -u -p COM24 "E:\Claude codes\SimpleTX"
```
On first boot each sketch imports what it can from the old sketch's flash on the same board:
- FC: sensor directions, trims, motor idle, battery ratio, gains, hover throttle.
- TX: stick calibration, trims, reverse, expo.

## Bench check (PROPS OFF)
1. FC serial (115200): `?` lists the settings. Tilt the right side down: roll goes **+**. Nose down: pitch goes **+**.
   If one is wrong: `rollSensRev=1` or `pitchSensRev=1`, then `save`.
2. TX: the bars move the right way (roll right = up, pitch forward = up, yaw right = up, throttle up = up).
   Otherwise fix it with Rev in the TX menu.
3. Arm (button, throttle low). Raise the throttle a little and tilt the quad right by hand: the right motors (M1, M2)
   must speed up. Yaw right on the stick: M2 and M3 (CCW props) must speed up.
4. Press the TX MODE button (GPIO20): the FC OLED/TX shows ALT. Armed on the ground it stays at idle until the throttle is above the hold zone (hover point to Stick mid, + Stick dband).
5. Unplug the TX while armed with the throttle up a little: after 1 s the FC disarms (it is not flying, so no auto-land).

## Setup on the FC OLED (disarmed, no PC)
- **Open it:** press BOOT to step through the pages until SETUP. Or on the sticks: throttle low, yaw full left, pitch full forward, hold 1 s.
- **Move:** pitch forward/back moves the cursor. Roll right/left changes the value (hold to repeat; throttle high gives x10 steps).
- **Enter / back:** yaw right opens a group. Yaw left goes back.
- **Actions:** LEVEL CALIBRATE, MOTOR TEST, ESC CALIBRATE (choose twice), SAVE, DEFAULTS (press twice), EXIT.
  A `*` means changed but not saved.
- **Motor test** (props off, TX ARM off): pitch picks M1-M4 or ALL, hold roll right to spin at 1150 us, yaw left goes back.
- **ESC calibration** (props off): choose it twice, unplug the battery and USB, then power up from the battery.
  The FC sends MAX for 5 s, then MIN for 5 s.
- **Ceiling:** ALT HOLD > Ceiling m (0 = off). Measured from where you armed; works in SELF-LEVEL and ALT HOLD.
- **Pages:** STATUS, SENSORS, STATS (last flight and stop reason), TREND (altitude and battery graph of the last flight), SETUP.
- **Defaults** are the figures from the current quad, read from its flash on 2026-10-21.

## DJI O4 OSD
- **Wiring:** FC GPIO21 → air unit RX, FC GPIO10 ← air unit TX, plus GND. Power the air unit from the battery, not from the FC.
- **How it works:** the FC identifies as Betaflight ("BTFL") and sends a canvas-mode OSD (MSP DisplayPort, 115200 baud) 10 times a second.
- **What it shows:**
  - Top: mode, flight time / DISARMED, signal and link quality.
  - Centre: horizon and aircraft mark.
  - Bottom: battery and volts per cell, altitude.
  - Warnings (blinking): LOW BATTERY, FAILSAFE, AUTOLAND, why it won't arm, and FC messages.
- **Arming:** being armed is reported in MSP_STATUS, which takes the O4 out of low-power mode. That is why the air unit's TX wire must go to GPIO10.
- **Settings:** SETUP > DJI OSD: on/off, canvas columns × rows (default 53 × 20; the air unit can also set it).

## USB serial (optional)
`?` lists everything. `gRollP=0.45` changes a value, then `save`. `D` dumps the last flight as CSV. `S` turns the status line on or off.

## Stick commands (disarmed)
- Level calibration: quad on a level surface, throttle low, yaw full left, pitch full back, hold for 2 s.
- Drift trim: throttle high, then push the right stick the way the quad should go. 0.1 deg per step, saved when you let go.
