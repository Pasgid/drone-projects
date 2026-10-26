# SimpleFC Brushed

A copy of SimpleFC (self-level + barometer altitude hold, OLED SETUP menu, ESP-NOW to SimpleTX, failsafe auto-land,
flight recorder, DJI O4 OSD) for a **1S brushed micro quad** (8520 / 716 motors) on an **ESP32-C3 SuperMini**.

## What is different from SimpleFC
| | SimpleFC | SimpleFC Brushed |
|---|---|---|
| Motor output | 250 Hz servo pulses to ESCs (1000-2000 us) | **20 kHz PWM duty** straight to a MOSFET per motor (1000 us = off, 2000 us = 100 %) |
| ESC calibration | yes | none (menu item shows "n/a") |
| Battery defaults | 3S, divider ratio 10.65 | **1S, divider 100k/100k (ratio 2.0)** |
| Motor idle | 1100 us | **1050 us** (about 5 % duty) |
| Hover default | 1250 us | **1500 us** (brushed micros hover near 50 %; it is learned anyway) |
| Settings | NVS "sfc", imports the old FC on first boot | NVS **"sfcb"**, starts from defaults, never imports the brushless tuning |

Everything else (pins, sensors, OLED menu, TX, OSD) is the same as SimpleFC.

## Pins (SuperMini)
- Motors: M1 GPIO0 (rear right, CW), M2 GPIO1 (front right, CCW), M3 GPIO3 (rear left, CCW), M4 GPIO5 (front left, CW)
  - props in; swap the two motor wires of a brushed motor to reverse it.
- I2C SDA GPIO6, SCL GPIO7: MPU6050, BMP280, OLED.
- Battery divider on GPIO4, buzzer GPIO20, DJI O4 OSD UART TX GPIO21 / RX GPIO10.

## Motor driver (build four)
```
GPIO ──100Ω──┬── gate   AO3400 / SI2302 (logic-level N-MOSFET, SOT-23)
             └─10kΩ── GND          (keeps the motor OFF while the C3 boots)
source ── GND
drain  ── motor (−)
motor (+) ── battery (+)
Schottky SS14 / 1N5819 across the motor: cathode → battery (+), anode → drain
optional 100 nF ceramic across the motor terminals
```

## Power (the most important part)
- 470-1000 uF low-ESR capacitor across the battery, close to the MOSFETs.
- A 1S cell sags to ~3.3 V under full throttle. Do **not** run the C3 and sensors straight from the cell:
  - best: a small 1S → 5 V boost converter into the SuperMini's 5 V pin (100 uF on its output), or
  - a low-dropout 3.3 V regulator (ME6211 / RT9013) with a big capacitor.
  Otherwise the FC browns out and the quad drops (we saw exactly that on the 3S quad with a 7805).
- Keep the motor current loop (battery → motors → MOSFETs → battery) short and thick; keep it away from the IMU and I2C wires.

## First flight checklist
1. Props off. Power from USB only: motors must stay still while booting (gate pull-downs).
2. SETUP > MOTOR TEST: each motor spins at the right position and direction.
3. Set Volt ratio so the OLED voltage matches a multimeter.
4. Props on, SELF-LEVEL, short hover. The PID defaults come from the brushless quad: expect to retune
   (brushed motors respond slower; start by lowering D if it buzzes, raise P if it feels soft).
5. Then ALT HOLD. Hover throttle is learned automatically in the first hovers.
