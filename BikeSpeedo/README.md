# BikeSpeedo

A wheel-sensor speedometer for a motorbike or e-bike. It runs on an ESP32-C3 SuperMini with a 1.3" OLED.

- Shows speed, trip distance, riding time, average and max speed, odometer and supply voltage.
- Saves each ride as a CSV file and serves them over its own Wi-Fi network.
- Sleeps when parked and wakes when the wheel moves or the button is pressed.

**Status:** compiles, but has not been run on hardware yet.

## Parts

| Part | Notes |
|---|---|
| ESP32-C3 SuperMini | Any C3 board with GPIO 0, 1, 4, 6 and 7 free |
| SH1106 1.3" OLED, I2C | For an SSD1306, uncomment `OLED_IS_SSD1306` |
| A3144 hall sensor (bare, 3 pins) | Switches on the magnet's south pole, so try the magnet both ways round |
| Neodymium magnet, about 10 mm | Bond it to the brake disc carrier or a wheel spoke |
| Momentary push button | Waterproof if the unit is outside |
| 5 V buck converter | Rated for your supply voltage (see Power) |
| Resistors | 100k, 15k, 10k x 2, 1k |
| Capacitors | 10 nF, 100 nF |
| Sealed box and screened cable | Screened cable for the sensor run |

## Wiring

```
Bike switched + --[fuse 1 A]--+-- buck IN+ ...... buck OUT 5 V --+-- C3 5V pin
                              |                                  +-- A3144 VCC (needs 4.5 V or more)
                              +-- 100k --+-- GPIO4
                                         +-- 15k --- GND
                                         +-- 100 nF - GND

A3144 OUT --+-- 10k --- 3V3
            +-- 1k ---+-- GPIO1
                      +-- 10 nF --- GND

Button:  GPIO0 --+-- 10k --- 3V3
                 +-- button --- GND

OLED:    VCC 3V3, GND, SDA GPIO6, SCL GPIO7
```

All grounds connect together. Keep the sensor cable away from ignition coils and HT leads.

The pull-ups must go to 3V3. A KY-003 style hall module has its own pull-up to its VCC, which would put 5 V on
GPIO1, so use a bare A3144.

The 10k pull-ups must be fitted even though the code enables the internal ones, because the wake-up from deep
sleep relies on them.

## Power

**Motorbike (12 V):** use a buck converter that tolerates at least 40 V, or a sealed automotive 12 V to USB adapter,
which is the easier choice because it is built for the spikes on a vehicle's supply. A 7805 is not suitable; it
overheats, as it did on your quad.

Take power from a circuit that goes off with the ignition. Deep sleep only stops the C3 and the OLED: the buck
converter and A3144 keep drawing an estimated 5 to 15 mA. On a permanent feed that could flatten a small bike
battery in a few weeks. On an ignition feed, the board simply starts with the key.

**E-bike:** the buck converter must be rated above the pack's full-charge voltage. A 48 V pack reaches 54.6 V, so an
LM2596 (40 V) is not enough. Also change the divider, for example to 220k over 10k with `vratio=23`, and set `lowv`
for your pack.

## Sensor mounting

1. Bond the magnet to the brake disc carrier or a spoke and secure it properly. A loose magnet on a wheel is dangerous.
2. Mount the A3144 on the fork or caliper bracket so the magnet passes 3 to 5 mm from its marked face.
3. Route the cable so it can't reach the wheel or disc and has slack for the suspension and steering.

More magnets give a quicker response at low speed. Space them evenly, and set `magnets` to match.

## Setting up

1. In the Arduino IDE choose ESP32C3 Dev Module, USB CDC On Boot: Enabled, Partition: Default 4MB (with spiffs).
2. Install U8g2 and upload.
3. Measure the wheel: mark the tyre and the ground, roll one full turn with the rider on, and measure in mm.
4. Set it with `wheel=1920` over serial, or on the web page.
5. Compare with GPS on a phone at a steady speed, then correct it: new wheel = wheel x GPS speed / shown speed.

Bench test without a wheel: send `sim=80` over serial and the display should show 80 km/h. Send `sim=0` to stop.
Sim pulses count towards the trip and odometer, so reset the trip and set `odo` afterwards.

## Using it

| Page | Short press | Hold 1.5 s (bike stopped) |
|---|---|---|
| Speed | Next page | Nothing |
| Trip | Next page | Reset the trip |
| Total | Next page | Nothing |
| Wi-Fi | Next page | Wi-Fi on or off |

**Getting ride logs:** stop the bike and hold the button on the Wi-Fi page. Join the `BikeSpeedo` network (password
`bikespeedo`, set by `AP_PASS` in the sketch) and open `192.168.4.1` on your phone. The same page has the settings.
Wi-Fi turns off when the bike moves, or after 10 minutes with no phone joined.

Files are `ride_0001.csv` and so on, with columns `t_s,kmh,trip_km,vin`. A new file starts each time the board powers
up or wakes and the bike reaches 5 km/h. There is no clock on the board, so files have numbers, not dates. Space
for about 15 hours of riding, with the oldest rides deleted first.

## Settings

| Name | Default | Meaning |
|---|---|---|
| `wheel` | 1900 | Rolling circumference, mm |
| `magnets` | 1 | Pulses per wheel turn |
| `maxkmh` | 300 | Pulses faster than this are rejected as noise |
| `mph` | 0 | 1 shows mph and miles (logs stay in km/h) |
| `vratio` | 7.667 | Divider ratio, (R1 + R2) / R2 |
| `lowv` | 11.5 | Low-supply warning in volts, 0 = off |
| `sleep` | 10 | Minutes before deep sleep, 0 = never |
| `odo` | 0 | Set the odometer, km |

## How it behaves

- Speed is the median of the last 5 wheel pulse periods, so a single noise spike doesn't show. The cost is about
  2 pulses of lag: with one magnet at 20 km/h, roughly 0.7 s.
- With one magnet and a 1.9 m wheel, it reads 0 below 3 km/h and takes about 2.3 s to reach 0 after you stop.
- Flash writes pause the sensor interrupt for a moment, which can delay or lose a pulse every 30 s or so. The median
  covers the speed reading, and the effect on distance is negligible.
- The odometer and trip save 3 s after stopping and every 2 km while moving. A key-off while rolling loses under 2 km of
  distance and up to 30 s of log.

Keep the bike's original speedometer working if the bike is road legal. This one is uncalibrated and unapproved.
