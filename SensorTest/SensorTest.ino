/*
 * SensorTest - reads an MPU6050 and an MS5611 (GY-63) on one I2C bus. No flight code, motors are not touched.
 * ESP32-C3 SuperMini: SDA GPIO6, SCL GPIO7. Serial 115200.
 * Prints the devices on the bus every 3 s, and the readings 5 times a second. A sensor that appears later is picked up.
 */
#include <Wire.h>

#define PIN_SDA 6
#define PIN_SCL 7

uint8_t mpuAddr = 0, msAddr = 0;
uint16_t msC[8];
bool msCrcGood = false;

bool cmd(uint8_t addr, uint8_t c) { Wire.beginTransmission(addr); Wire.write(c); return Wire.endTransmission() == 0; }

bool readAfterStop(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {   // MS5611: STOP between command and read
  if (!cmd(addr, reg)) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool readRepeated(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {    // MPU6050: repeated start
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void scanBus() {
  Serial.print("I2C bus:");
  int n = 0;
  for (uint8_t a = 8; a < 120; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  if (!n) Serial.print(" nothing answers");
  Serial.printf("   | idle levels SDA %s SCL %s\n", digitalRead(PIN_SDA) ? "high" : "LOW", digitalRead(PIN_SCL) ? "high" : "LOW");
}

bool mpuStart() {
  for (uint8_t a : {(uint8_t)0x68, (uint8_t)0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    uint8_t id = 0;
    readRepeated(a, 0x75, &id, 1);
    Wire.beginTransmission(a); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission();   // wake, PLL
    delay(50);
    Wire.beginTransmission(a); Wire.write(0x1B); Wire.write(0x10); Wire.endTransmission();   // +-1000 dps
    Wire.beginTransmission(a); Wire.write(0x1C); Wire.write(0x18); Wire.endTransmission();   // +-16 g
    mpuAddr = a;
    Serial.printf("MPU at 0x%02X, WHO_AM_I 0x%02X %s\n", a, id, id == 0x68 ? "(MPU6050)" : "(not a genuine 6050 ID)");
    return true;
  }
  return false;
}

bool msCrcOk() {
  uint16_t rem = 0;
  for (int i = 0; i < 16; i++) {
    uint16_t w = msC[i >> 1];
    if (i >= 14) w &= 0xFF00;
    rem ^= (i & 1) ? (w & 0xFF) : (w >> 8);
    for (int b = 0; b < 8; b++) rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
  }
  return ((rem >> 12) & 0xF) == (msC[7] & 0xF);
}

bool msStart() {
  for (uint8_t a : {(uint8_t)0x77, (uint8_t)0x76}) {
    if (!cmd(a, 0x1E)) continue;    // reset
    delay(4);
    bool ok = true;
    for (int i = 0; i < 8 && ok; i++) {
      uint8_t b[2];
      ok = readAfterStop(a, 0xA0 + 2 * i, b, 2);
      if (ok) msC[i] = (b[0] << 8) | b[1];
    }
    if (!ok) { Serial.printf("MS5611 at 0x%02X: PROM read failed\n", a); continue; }
    msCrcGood = msCrcOk();
    msAddr = a;
    Serial.printf("MS5611 at 0x%02X, PROM CRC %s, C1..C6 = %u %u %u %u %u %u\n", a, msCrcGood ? "good" : "BAD", msC[1], msC[2], msC[3], msC[4], msC[5], msC[6]);
    return true;
  }
  return false;
}

bool msRead(float &pa, float &degC) {
  uint8_t b[3];
  if (!cmd(msAddr, 0x58)) return false;   // temperature, OSR 4096
  delay(10);
  if (!readAfterStop(msAddr, 0x00, b, 3)) return false;
  uint32_t d2 = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  if (!cmd(msAddr, 0x48)) return false;   // pressure, OSR 4096
  delay(10);
  if (!readAfterStop(msAddr, 0x00, b, 3)) return false;
  uint32_t d1 = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  if (d1 == 0 || d2 == 0) return false;
  int32_t dT = (int32_t)d2 - ((int32_t)msC[5] << 8);
  int32_t temp = 2000 + (int32_t)(((int64_t)dT * msC[6]) >> 23);
  int64_t off = ((int64_t)msC[2] << 16) + (((int64_t)msC[4] * dT) >> 7);
  int64_t sens = ((int64_t)msC[1] << 15) + (((int64_t)msC[3] * dT) >> 8);
  if (temp < 2000) {
    int64_t tt = (int64_t)(temp - 2000) * (temp - 2000);
    off -= 5 * tt / 2; sens -= 5 * tt / 4;
  }
  pa = (float)((((int64_t)d1 * sens >> 21) - off) >> 15);
  degC = temp / 100.0f;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(800);
  Serial.println("\nSensorTest: MPU6050 + MS5611, SDA GPIO6, SCL GPIO7");
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
}

void loop() {
  static uint32_t lastScan = 0, lastRead = 0;
  static float groundPa = 0;
  uint32_t ms = millis();
  if (lastScan == 0 || ms - lastScan > 3000) {
    lastScan = ms ? ms : 1;
    scanBus();
    if (!mpuAddr) mpuStart();
    if (!msAddr) msStart();
  }
  if (ms - lastRead < 200) return;
  lastRead = ms;
  if (mpuAddr) {
    uint8_t d[14];
    if (readRepeated(mpuAddr, 0x3B, d, 14)) {
      int16_t v[7];
      for (int i = 0; i < 7; i++) v[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]);
      float ax = v[0] / 2048.0f, ay = v[1] / 2048.0f, az = v[2] / 2048.0f;
      Serial.printf("MPU acc %+.2f %+.2f %+.2f g (|a| %.2f) gyro %+6.1f %+6.1f %+6.1f dps %.1f C | ", ax, ay, az, sqrtf(ax * ax + ay * ay + az * az),
                    v[4] / 32.8f, v[5] / 32.8f, v[6] / 32.8f, v[3] / 340.0f + 36.53f);
    } else Serial.print("MPU READ FAILED | ");
  } else Serial.print("MPU missing | ");
  if (msAddr) {
    float pa, c;
    if (msRead(pa, c)) {
      if (groundPa <= 0) groundPa = pa;
      Serial.printf("MS %.0f Pa %.2f C alt %+.2f m\n", pa, c, 44330.0f * (1 - powf(pa / groundPa, 0.190295f)));
    } else Serial.println("MS READ FAILED");
  } else Serial.println("MS missing");
}
