# ESP32-C3 Zero Flight Controller + ESP-NOW Transmitter

Two Arduino sketches:

| Folder | Board | What it does |
|---|---|---|
| `Transmitter/` | ESP32-C3 Zero #1 | Reads 2 gimbals plus a button and sends 6 channels at 50 Hz over ESP-NOW. Shows the channels and a joystick-driven menu on its OLED. |
| `FlightController/` | ESP32-C3 Zero #2 | The full setup-menu FC (v8): QUAD or PLANE, MPU6050, its own OLED, battery monitor, buzzer, stick-driven SETUP menu. Receives over ESP-NOW and sends telemetry back to the TX. |

## Channels

| CH | Name | Source on the TX |
|---|---|---|
| 1 | THR | left stick, vertical |
| 2 | ROLL | right stick, horizontal |
| 3 | PITCH | right stick, vertical |
| 4 | YAW | left stick, horizontal |
| 5 | AUX1, ARM | push button (short press, throttle low) |
| 6 | CH6 | 3-position switch = **flight mode**: POS1 = SELF STAB (1000), POS2 = ALT HOLD (1500), POS3 = POS HOLD (2000) |

The FC remaps them internally (roll, pitch, throttle, yaw, arm, spare), so nothing needs configuring.

## Wiring

### Transmitter
| Part | ESP32-C3 Zero pin |
|---|---|
| Throttle pot wiper | GPIO1 |
| Roll pot wiper | GPIO3 |
| Pitch pot wiper | GPIO4 |
| Yaw pot wiper | GPIO0 |
| Pot outer legs | 3V3 and GND |
| OLED SDA / SCL | GPIO5 / GPIO6 (VCC to 3V3), 1.3" SH1106. For a 0.96" SSD1306, uncomment `OLED_IS_SSD1306`. |
| Push button | GPIO7 to GND (internal pull-up) |
| CH6 ON-OFF-ON switch | middle leg to GND, **POS1 (DISARM) leg to GPIO20**, **POS3 leg to GPIO8** (internal pull-ups, no resistors). Not GPIO10: that's the on-board RGB LED. |
| Passive buzzer | GPIO21 through 1k to the base of an NPN (emitter to GND), buzzer between collector and 3V3/5V. A small one can go GPIO21 → 100 Ω → buzzer → GND. |

The buzzer may click briefly at power-up, because GPIO21 also carries the boot log. That's normal.

**TX battery on GPIO2.** The pin can only read about 2.5 V, so the battery goes through a divider: battery+ → R1 → GPIO2 → R2 → GND, plus a 100 nF capacitor from GPIO2 to GND.

| TX battery | R1 | R2 | Set "TX batt ratio" to |
|---|---|---|---|
| 1S (4.2 V) | 100k | 100k | 2.00 (default) |
| 2S (8.4 V) | 100k | 33k | 4.03 |
| 3S (12.6 V) | 100k | 22k | 5.55 |

The TX menu title shows the live TX voltage. Adjust *TX batt ratio* until it matches a multimeter, then save.

If the switch positions come out backwards, swap the two outer wires. With no switch fitted, set `USE_CH6_SWITCH 0`.

The pots **must** stay on GPIO0–4 (ADC1), because ADC2 stops working while ESP-NOW is on.

> The C3 ADC saturates around 2.5 V. If a stick's reading stops changing near one end, add a series resistor (about 1.5–2.2 kΩ for a 5 kΩ pot) between 3V3 and the pot's top leg, then recalibrate.

### Flight controller
| Part | ESP32-C3 Zero pin |
|---|---|
| MPU6050 + OLED SDA | GPIO6 (shared I2C bus, 3.3 V) |
| MPU6050 + OLED SCL | GPIO7 |
| Output 1..4 | GPIO0, GPIO1, GPIO3, **GPIO8** (output 4 moved from GPIO4) |
| Battery divider | **GPIO4**: pack+ to 100k to GPIO4, then 10k to GND, plus 100 nF to GND |
| GPIO2 | **Not used.** On this board its pad is shorted to the crystal can (GND). |
| Buzzer | GPIO20 through 1k to the base of an NPN; buzzer between collector and +5 V |
| Page button | GPIO9 (on-board BOOT button) |
| MTF-01/02 optical flow + rangefinder | MTF **TX → GPIO5**, MTF RX → GPIO10, 5V, GND. Set the module to **MSP / INAV mode, 115200 baud**, and mount it pointing straight down. |

**Checking the optical flow** (FLOW page on the FC, reached with the BOOT button). Do this on the bench before any position hold:
1. If the page says `NO DATA`, check the TX→GPIO5 wire and that the module is in MSP mode at 115200 baud. Many `crc err` means the wrong baud rate.
2. Hold the craft about 30 cm above a textured floor (carpet, a patterned mat). `Q` should rise and `RNG` should read the height.
3. **Direction check:** slide it **forward** by hand. **VEL F** should go positive. Slide it **right**, and **VEL R** should go positive. If the wrong axis responds, change OPT FLOW → **ALIGN**. If one axis has the wrong sign, toggle **FLIP SIDEWAYS** or change ALIGN by 180.
4. **Rotation check:** rock it in roll and pitch **without moving it sideways**. **COMP** should stay near 0. If it grows larger than RATE, set **GYRO COMP** to the other option.
5. **Scale check:** slide it 1 m in 1 s at a steady height. VEL should read about 100 cm/s. If it reads too high, raise **FLOW SCALE**; if too low, lower it.

OPT FLOW → **RANGEFINDER = AUTO** makes alt hold use the MTF rangefinder (up to ~8 m) whenever it's sending, and the VL53L1X otherwise. With the VL53L1X set to FORWARD for obstacle avoidance, only the MTF is used for height. Raise ALT HOLD → TOF MAX M to use more of its range.

- **QUAD:** Betaflight order, props in: M1 rear-right CW, M2 front-right CCW, M3 rear-left CCW, M4 front-left CW.
- **PLANE:** output 1 = aileron, 2 = elevator, 3 = throttle (ESC), 4 = rudder.
- The FC's OLED is a 1.3" SH1106 by default. For an SSD1306, uncomment `OLED_IS_SSD1306`.
- GPIO5 is now free, because there is no PPM receiver.

## Software setup
1. In Arduino IDE, use the board **ESP32C3 Dev Module** with **USB CDC On Boot: Enabled**.
2. In Library Manager, install **Adafruit SH110X** (for the TX), plus **U8g2** and **VL53L1X by Pololu** (for the FC).
3. Flash `FlightController` to one board and `Transmitter` to the other.
4. `PKT_MAGIC`, `BIND_ID` and `WIFI_CH` must match in both sketches. The FC locks onto the first TX it hears after power-up.

Settings saved by the old PPM version (`esp32c3_zero_fc_setup.ino`) are kept, since they use the same flash layout.

## Using the transmitter

| Action | Result |
|---|---|
| Short press | **ARM / DISARM** (arming needs throttle low) |
| CH6 switch | Flight mode: POS1 SELF STAB, POS2 ALT HOLD, POS3 POS HOLD (see "Flight modes" below) |
| Long press (disarmed) | Open the TX menu |
| Right stick up / down | Move through the menu |
| Right stick left / right | Change the value (reverse NOR↔REV, trims, expo, rates) |
| Short press in menu | Select (Calibrate / Save / Exit) |
| Long press in menu | Save & exit |

**Calibrate the sticks first:** long press, go to *Calibrate sticks*, move both sticks to every corner, press, centre them, press.

The TX header shows `SAFE` / `ARM?` / `ARMED`, the FC's mode (`SELF` / `ACRO`), the **FC battery voltage** and link quality. When the FC battery is low, the voltage blinks and the TX buzzer gives two short beeps every 2 s. You can switch the buzzer off in the TX menu (*Buzzer*); turning it back on gives a test beep. The TX battery shows next to the FC battery. When it's low (below *TX warn V/cell* for 3 s), it blinks and the buzzer gives one long beep every 3 s. If the FC refuses to arm, a box shows why: `THR HIGH`, `RE-ARM`, `TILTED`, `IMU ERR`, `NO LINK` or `IN SETUP`.

## Flight controller: stick commands (disarmed, throttle LOW, hold 1 s)

| Sticks | Action |
|---|---|
| Yaw LEFT + Roll LEFT (pitch centred) | Flight mode **SELF STAB** (1 beep, saved) |
| Yaw LEFT + Roll RIGHT (pitch centred) | Flight mode **ACRO** (2 beeps, saved) |
| Yaw LEFT + Pitch FORWARD (roll centred) | Open the FC **SETUP** menu |

### FC SETUP menu (on the FC's OLED, driven by your TX sticks)
| Stick | Action |
|---|---|
| Pitch forward / back | Cursor up / down |
| Roll right / left | Value + / − (hold to repeat) |
| Throttle high | Changes become ×10 |
| Yaw right | Enter a submenu / run an action |
| Yaw left | Back / leave setup |

Menu: VEHICLE, FLIGHT MODE, RC RATES, SELF-LEVEL, LEVEL TRIM, GYRO PID, DIRECTIONS (props in/out, stick and sensor reverse, servo reverse, motor idle, acro airmode), BATTERY (ratio, cells, warn level, buzzer), FILTERS, ALT HOLD, OPT FLOW, AVOIDANCE, POS HOLD, TX CHECK, OUTPUT TEST, ESC CALIBRATE, ESC CAL AT BOOT, SAVE, DEFAULTS, EXIT.

A `*` in the title means unsaved changes, so choose SAVE. Arming is refused while SETUP is open.

The BOOT button cycles the FC OLED pages: STATUS, RC bars, RATE CURVE, FLIGHT STATS, TREND, SENSORS and SETUP.

## Buzzer
| Pattern | Meaning |
|---|---|
| 1 beep | Ready |
| 1 long beep | Armed |
| 2 beeps | Disarmed |
| 3 beeps | Arm refused / failsafe |
| 2 short beeps every 2 s | Low battery |
| Long beeps | Signal lost after being armed (find-me) |
| Fast chirping | IMU fault |

## Safety and first-flight checklist (PROPS OFF until the last step)
1. Use **TX CHECK** on the FC to confirm every channel reaches ≤1100 and ≥1900.
2. On the FC SENSORS / STATUS page: tilt right and roll should go positive; tilt nose down and pitch should go positive. Fix it with DIRECTIONS → ROLL/PITCH SNS REV.
3. Put the quad on a level surface and run LEVEL TRIM → CALIBRATE, then SAVE.
4. Use **OUTPUT TEST** to check each motor's position and spin direction.
5. Arm with throttle low. The motors should idle at MOTOR IDLE. Tilt the frame by hand with a little throttle, and the low side should speed up.
6. Failsafe: switch the TX off while armed. The outputs must go safe within 0.25 s and the find-me beeps should start.
7. Fit props and fly in SELF STAB first.

This is a hobby flight controller. Failsafe **cuts the motors**, so the quad will drop. Fly it somewhere safe.

## Flight modes: safe procedure

**The CH6 switch selects the flight mode** (it does not arm; arming is the TX button):

| CH6 | Mode | What it does |
|---|---|---|
| **POS1** | **SELF STAB** | Self-levels; the sticks set the tilt angle. You control the throttle. |
| **POS2** | **ALT HOLD** | SELF STAB + automatic throttle that holds the height |
| **POS3** | **POS HOLD** | ALT HOLD + holds its position over the ground using the optical flow |

- **POS1 uses the base mode,** which is SELF STAB unless you picked **ACRO** with the stick command (disarmed, throttle low, yaw left + roll right). ACRO doesn't self-level, so set it back to SELF (yaw left + roll left) unless you really want ACRO.
- **ALT HOLD and POS HOLD only engage when you move the switch while flying.** If you arm with the switch already in POS2 or POS3, it flies in POS1's mode until you move the switch to POS1 and back up. This prevents a surprise engage on the ground.

### Before every flight (props on, outdoors or in a large space)
1. Battery charged. The TX header shows both voltages.
2. The FC SETUP **VEHICLE** is **QUAD**, and the base mode is SELF (the TX header shows `SELF`).
3. Set the quad down **level**, power it up, and **don't touch it until the gyro calibration finishes** (one beep).
4. On the FC STATUS page, roll and pitch read about 0 with the quad level. If not, run **LEVEL TRIM → CALIBRATE** and **SAVE**.
5. Check the link: `LQ` is 90–100% on the TX.
6. **CH6 in POS1.** Stand 3 m or more behind the quad, so its front faces the same way as you.

### 1. SELF STAB (POS1)
1. **Arm:** throttle low, short press on the TX button. The motors idle and the TX shows `ARMED`.
2. Raise the throttle smoothly until it lifts. Hover at knee height for 10–20 s.
3. **If it drifts steadily one way,** land and disarm, then trim it out (see "Fixing drift" below).
4. **If it wobbles or oscillates,** land and lower the GYRO PID P values by 10–20%. If it feels sloppy, raise them.
5. **Land:** lower the throttle slowly until it touches down, then short press to **disarm**.

### 2. ALT HOLD (POS2), only after SELF STAB flies well
1. **Prepare on the ground:**
   - Put open-cell foam over the BMP280. Prop wash on the barometer is the main cause of altitude bouncing.
   - The MTF rangefinder must point straight down with a clear view.
2. **Take off in POS1** and hover steadily at 1–2 m.
3. **Flip CH6 to POS2.**
   - One long beep: hold is on, and the TX shows `ALT`.
   - Three beeps: it refused (`TAKE OFF FIRST` or `NO ALT SENSOR`).
4. Leave the throttle stick where it is. That position now means "hold".
   - Push it up to climb (up to MAX CLIMB) or down to descend (up to MAX DESCENT).
   - Return it to the same spot to hold the new height.
5. **To leave:** first bring the throttle stick back to the hover position, then flip to **POS1**. You get two beeps, and the throttle hands back to your stick smoothly over 0.6 s.
6. **Always land in POS1 (SELF STAB),** never in ALT HOLD or POS HOLD.
7. **If the altitude sensor fails in flight,** it drops back to SELF STAB by itself, with two beeps and `ALT LOST-MANUAL`. Take over the throttle.

### 3. POS HOLD (POS3), after ALT HOLD works well
1. **Prepare:** the optical flow is calibrated (OPT FLOW → AUTO CALIBRATE, done ✅). You need a **textured floor** (carpet, patterned tiles or mats; not a plain shiny floor), good light, and a height under **TOF MAX M** (ALT HOLD menu, default 2 m).
2. **Hover in ALT HOLD (POS2)** at about 1 m, then **flip to POS3**.
   - One long beep and the TX shows `POS`: it holds this spot.
   - `NO FLOW: ALT ONLY` with three beeps: the flow isn't good enough there (floor, light or height), so it stays in ALT HOLD.
3. **Hands off:** it stays over the spot, and corrects drift and gentle wind.
4. **Fly it with roll/pitch:** the sticks now set a **speed** (up to MAX SPEED, 1 m/s at full stick). Centre the sticks and it brakes, then holds the new spot. Yaw works normally, and the held spot doesn't move when you turn.
5. **If the flow is lost** (plain floor, too high, dark), it drops to ALT HOLD with two beeps (`FLOW LOST-ALT HOLD`), so you steer again. It goes back to POS HOLD by itself once the flow has been good for 1 s (`POS HOLD BACK`).
6. **To leave:** POS3 → POS2 keeps the altitude hold only. POS1 gives full manual SELF STAB.

**Tuning POS HOLD** (SETUP → POS HOLD; the title shows the live ground speed):

| Symptom | Change |
|---|---|
| Rocks back and forth around the spot | Lower **VEL P** (6 → 4) |
| Slow, lazy return to the spot | Raise **POS P** (1.0 → 1.5) |
| Slowly creeps off the spot (wind, CG) | Raise **VEL I** (2 → 3) |
| Holds well but jerky | Lower **MAX TILT** |
| Drops out to ALT HOLD too often | Lower **MIN QUALITY** (60 → 40), or improve the floor or light |

**Tuning alt hold** (SETUP → ALT HOLD; the title shows the live altitude):

| Symptom | Change |
|---|---|
| Bobs up and down | Lower **VEL P** |
| Slowly sinks or rises | Raise **VEL I** |
| Slow to return to height | Raise **ALT P** |

Near the ground (under TOF MAX M) the MTF rangefinder is used, so the hold is very tight. Above that, the barometer is used, so expect ±0.3–0.5 m.

**Ceiling:** set ALT HOLD → **CEILING** in metres above the arming point. It works in SELF STAB, ALT HOLD and POS HOLD, by limiting the throttle near the ceiling. **It doesn't work in ACRO.**

### ACRO (experienced pilots only)
1. Select ACRO with the stick command on the ground (2 beeps). It then replaces SELF STAB in POS1. Fly it in a large open area, with height.
2. **ACRO doesn't self-level.** The quad keeps whatever angle you leave it at, so you must level it yourself.
3. You can't switch the base mode in the air, but flipping to **POS2 or POS3** always gives self-levelling (ALT / POS HOLD) as a rescue.

### Emergencies
- **Anything wrong:** short press on the TX button to **disarm**. The motors stop immediately, so the quad drops.
- **TX switched off or out of range:** the FC disarms after 0.25 s, then sounds find-me beeps.
## Obstacle avoidance (forward VL53L1X, up to 2 m)
- **Mounting:** the VL53L1X faces **forward**, level, on the nose, with nothing in front of the sensor window. In SETUP → AVOIDANCE set **VL53 POINTS = FORWARD** (the default). Height then comes only from the MTF-01.
- **How it works** (SELF STAB and ALT HOLD only; ACRO is never limited):
  - Closer than **AVOID DIST** (2 m): the forward tilt you're allowed shrinks.
  - At **STOP DIST** (0.6 m): no forward tilt at all.
  - Closer than STOP DIST: it tilts back by itself, up to **MAX BACK** (8°).
  - Backward and sideways sticks always work, so you can always fly away.
  - When the optical flow sees you approaching fast, it reacts earlier.
- **Warnings:** within AVOID DIST the FC beeps (faster when very close), and the TX shows `OBSTACLE` and beeps. This happens even with AVOIDANCE OFF.
- **It starts OFF.** First check it on the bench:
  1. On the ALTITUDE page, the `FWD` line should read `clear` with nothing in front.
  2. Walk a hand or a box towards the nose. It should show the distance, then `WARN`.
  3. Only then set **AVOIDANCE = ON** and SAVE.
- **Limits:**
  - **Detected:** solid, matte things straight ahead in a narrow ~27° cone.
  - **Missed:** glass, mirrors, thin wires, branches, and anything off to the side or behind.
  - **Range:** in bright sunlight it drops well below 2 m.
  - Treat it as a safety net, not autopilot.

## Fixing drift (SELF STAB)
A self-levelling quad holds an **angle**, not a **position**. Wind always moves it, so only GPS or optical-flow position hold can stop that. A steady drift in calm air, though, is fixable:

1. **Level calibration:** LEVEL TRIM → CALIBRATE on a truly level surface. Check it with a spirit level on the frame, not the table.
2. **Stick trim:**
   - Disarmed, **throttle HIGH**, yaw centred. Push the right stick **against** the drift; if it drifted right, push roll **left**.
   - Each 0.3 s held adds 0.1° and beeps. It saves when you let go.
   - Take off and repeat until it hovers in place.
3. **Vibration:** on the SENSORS page, VIB with props on should be low (under about 5). High vibration makes the accelerometer read a false tilt. Balance the props, soft-mount the FC, and use FILTERS (ACC LPF, NOTCH).
4. **Centre of gravity:** the battery must sit in the exact centre.
5. **ACC TAU FLT** (LEVEL TRIM): if it slowly leans more during long hovers, raise it.
6. **SELF-LEVEL I:** after P is tuned, a small ROLL I / PITCH I (0.05–0.2) removes the last leftover tilt.
