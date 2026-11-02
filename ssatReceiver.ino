
#include <Wire.h>
#include <math.h>

// ESP32 DevKit V1 wiring
#define SDA_PIN 23
#define SCL_PIN 22

uint8_t bmeAddress = 0x76;

// BME280 calibration data
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3, dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

double tFine;

// Write a register
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(bmeAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read one byte
uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(bmeAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;

  if (Wire.requestFrom(bmeAddress, (uint8_t)1) == 1)
    return Wire.read();

  return 0;
}

// Read unsigned 16-bit little-endian
uint16_t read16(uint8_t reg) {
  uint16_t l = read8(reg);
  uint16_t h = read8(reg + 1);
  return (h << 8) | l;
}

// Read signed 16-bit little-endian
int16_t readS16(uint8_t reg) {
  return (int16_t)read16(reg);
}

// Load factory calibration values
void readCalibration() {
  dig_T1 = read16(0x88);
  dig_T2 = readS16(0x8A);
  dig_T3 = readS16(0x8C);

  dig_P1 = read16(0x8E);
  dig_P2 = readS16(0x90);
  dig_P3 = readS16(0x92);
  dig_P4 = readS16(0x94);
  dig_P5 = readS16(0x96);
  dig_P6 = readS16(0x98);
  dig_P7 = readS16(0x9A);
  dig_P8 = readS16(0x9C);
  dig_P9 = readS16(0x9E);

  dig_H1 = read8(0xA1);
  dig_H2 = readS16(0xE1);
  dig_H3 = read8(0xE3);

  uint8_t e4 = read8(0xE4);
  uint8_t e5 = read8(0xE5);
  uint8_t e6 = read8(0xE6);

  dig_H4 = ((int16_t)(int8_t)e4 << 4) | (e5 & 0x0F);
  dig_H5 = ((int16_t)(int8_t)e6 << 4) | (e5 >> 4);

  if (dig_H4 & 0x0800) dig_H4 -= 4096;
  if (dig_H5 & 0x0800) dig_H5 -= 4096;

  dig_H6 = read8(0xE7);
}

// Read all 8 raw data bytes
bool readRaw(uint32_t &rawP, uint32_t &rawT, uint16_t &rawH) {
  Wire.beginTransmission(bmeAddress);
  Wire.write(0xF7);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bmeAddress, (uint8_t)8) != 8)
    return false;

  uint8_t d[8];
  for (int i = 0; i < 8; i++) d[i] = Wire.read();

  rawP = ((uint32_t)d[0] << 12) |
         ((uint32_t)d[1] << 4) | (d[2] >> 4);

  rawT = ((uint32_t)d[3] << 12) |
         ((uint32_t)d[4] << 4) | (d[5] >> 4);

  rawH = ((uint16_t)d[6] << 8) | d[7];

  return true;
}

// Temperature in Celsius
double getTemperature(uint32_t adcT) {
  double v1 = ((double)adcT / 16384.0 - dig_T1 / 1024.0)
              * dig_T2;

  double v2 = ((double)adcT / 131072.0 - dig_T1 / 8192.0);
  v2 = v2 * v2 * dig_T3;

  tFine = v1 + v2;
  return tFine / 5120.0;
}

// Pressure in hPa
double getPressure(uint32_t adcP) {
  double v1 = tFine / 2.0 - 64000.0;
  double v2 = v1 * v1 * dig_P6 / 32768.0;

  v2 += v1 * dig_P5 * 2.0;
  v2 = v2 / 4.0 + dig_P4 * 65536.0;

  v1 = (dig_P3 * v1 * v1 / 524288.0
        + dig_P2 * v1) / 524288.0;

  v1 = (1.0 + v1 / 32768.0) * dig_P1;

  if (v1 == 0.0) return NAN;

  double p = 1048576.0 - adcP;
  p = (p - v2 / 4096.0) * 6250.0 / v1;

  v1 = dig_P9 * p * p / 2147483648.0;
  v2 = p * dig_P8 / 32768.0;

  p += (v1 + v2 + dig_P7) / 16.0;

  return p / 100.0;
}

// Relative humidity in percent
double getHumidity(uint16_t adcH) {
  double h = tFine - 76800.0;

  h = (adcH - (dig_H4 * 64.0 + dig_H5 / 16384.0 * h))
      * (dig_H2 / 65536.0
      * (1.0 + dig_H6 / 67108864.0 * h
      * (1.0 + dig_H3 / 67108864.0 * h)));

  h *= 1.0 - dig_H1 * h / 524288.0;

  if (h > 100.0) h = 100.0;
  if (h < 0.0) h = 0.0;

  return h;
}

void setup() {
  Serial.begin(115200);

  // Custom I2C pins: SDA 23, SCL 22
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  delay(100);

  // Check BME280 chip ID (must be 0x60)
  if (read8(0xD0) != 0x60) {
    bmeAddress = 0x77;

    if (read8(0xD0) != 0x60) {
      Serial.println("BME280 NOT DETECTED!");
      Serial.println("Check SDA=23, SCL=22, VIN and GND.");
      while (true) delay(1000);
    }
  }

  Serial.print("BME280 found at address 0x");
  Serial.println(bmeAddress, HEX);

  // Reset sensor
  writeReg(0xE0, 0xB6);
  delay(10);

  // Wait for calibration data copy
  while (read8(0xF3) & 0x01) delay(2);

  readCalibration();

  // Humidity oversampling x1
  writeReg(0xF2, 0x01);

  // Temperature x1, pressure x1, normal mode
  writeReg(0xF4, 0x27);

  // Standby 1000 ms, filter disabled
  writeReg(0xF5, 0xA0);

  Serial.println("BME280 initialization complete!");
  Serial.println();
}

void loop() {
  uint32_t rawP, rawT;
  uint16_t rawH;

  if (!readRaw(rawP, rawT, rawH)) {
    Serial.println("Sensor read error!");
    delay(2000);
    return;
  }

  double temperature = getTemperature(rawT);
  double pressure = getPressure(rawP);
  double humidity = getHumidity(rawH);

  // Approximate altitude using standard sea-level pressure
  const double seaLevelPressure = 1013.25; // hPa

  double altitude = 44330.0 *
      (1.0 - pow(pressure / seaLevelPressure, 0.1903));

  Serial.println("======= BME280 DATA =======");

  Serial.print("Temperature : ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  Serial.print("Temperature : ");
  Serial.print(temperature * 9.0 / 5.0 + 32.0, 2);
  Serial.println(" °F");

  Serial.print("Pressure    : ");
  Serial.print(pressure, 2);
  Serial.println(" hPa");

  Serial.print("Pressure    : ");
  Serial.print(pressure * 100.0, 2);
  Serial.println(" Pa");

  Serial.print("Humidity    : ");
  Serial.print(humidity, 2);
  Serial.println(" %");

  Serial.print("Altitude    : ");
  Serial.print(altitude, 2);
  Serial.println(" m");

  Serial.println("===========================");
  Serial.println();

  delay(1000);
}
