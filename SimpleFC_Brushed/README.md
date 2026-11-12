# SimpleFC Brushed

A copy of SimpleFC (self-level + barometer altitude hold, OLED SETUP menu, ESP-NOW to SimpleTX, failsafe auto-land,
flight recorder, DJI O4 OSD) for a **2S brushed micro quad** (820 coreless, 7.4 V motors) on an **ESP32-C3 SuperMini**.

## What is different from SimpleFC
| | SimpleFC | SimpleFC Brushed |
|---|---|---|
| Motor output | 250 Hz servo pulses to ESCs (1000-2000 us) | **20 kHz PWM duty** straight to a MOSFET per motor (1000 us = off, 2000 us = 100 %) |
| ESC calibration | yes | none (menu item shows "n/a") |
| Battery defaults | 3S, divider ratio 10.65 | **2S, divider 100k/33k (ratio 4.03)** |
| Motor idle | 1100 us | **1050 us** (about 5 % duty) |
| Hover default | 1250 us | **1500 us** (brushed micros hover near 50 %; it is learned anyway) |
| Settings | NVS "sfc", imports the old FC on first boot | NVS **"sfcb"**, starts from defaults, never imports the brushless tuning |
| Configuration | OLED menu + sticks | **USB serial, no OLED** (weight). `USE_OLED 1` brings a 0.91 in SSD1306 menu back |

Everything else (pins, sensors, TX, telemetry) is the same as SimpleFC. The OSD is off by default.

## Configuration: USB serial, 115200 (disarmed)
| Command | Does |
|---|---|
| `help` | command list |
| `?` | all settings, by group |
| `gRollP=0.45` | change a setting, it acts at once; then `save` |
| `save` / `defaults` | store in flash / factory settings (then `save`) |
| `cal` | level calibration: quad flat and still |
| `m1` `m2` `m3` `m4` `mall` | spin a motor for 2 s, **props off**; `stop` stops it |
| `st` | one status line; `S` turns the live status line on or off |
| `stats` | last flight and stop reason |
| `D` | dump the last flight as CSV |

The stick commands still work (level calibration: throttle low + yaw left + pitch back 2 s; drift trim: throttle high + right stick) and the TX shows battery, mode and why it won't arm.

## Pins (SuperMini)
- Motors: M1 GPIO0 (rear right, CW), M2 GPIO1 (front right, CCW), M3 GPIO3 (rear left, CCW), M4 GPIO5 (front left, CW)
  - props in; swap the two motor wires of a brushed motor to reverse it.
- I2C SDA GPIO6, SCL GPIO7: MPU6050, BMP280.
- Battery divider on GPIO4, passive buzzer GPIO20 (through 100 ohm to GND; optional, no pads on the PCB: use the socket's G20 pad), DJI O4 OSD UART TX GPIO21 / RX GPIO10.

## Barometer: BMP280 or GY-63 (MS5611)
The FC detects either one at boot; serial `st` and the boot log say which. Fit only one.
GY-63 wiring, by the labels (its pin order differs from the BMP280 header): VCC to 3V3, GND, SCL to GPIO7, SDA to GPIO6,
**PS to VCC** (selects I2C), **CSB to GND**. Cover it with open-cell foam: it is sensitive to light and prop wash.

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
- **Never connect 2S to the SuperMini's 5 V pin** (about 6 V maximum). Use a small 2S → 5 V buck converter, set to 5.0 V before connecting, with 100 uF on its output.
- C1 must be rated 16 V or more.
- If the C3 browns out the quad drops (we saw exactly that on the 3S quad with a 7805); the boot log then shows `BROWNOUT RESET`.
- Keep the motor current loop (battery → motors → MOSFETs → battery) short and thick; keep it away from the IMU and I2C wires.

## First flight checklist
1. Props off. Power from USB only: motors must stay still while booting (gate pull-downs).
2. Serial `m1` ... `m4`: each motor spins at the right position and direction.
3. Serial `st` shows the voltage; set `vRatio=` until it matches a multimeter, then `save`.
4. Props on, SELF-LEVEL, short hover. The PID defaults come from the brushless quad: expect to retune
   (brushed motors respond slower; start by lowering D if it buzzes, raise P if it feels soft).
5. Then ALT HOLD. Hover throttle is learned automatically in the first hovers.
