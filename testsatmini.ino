
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ESP32 DevKit V1 pins
#define SDA_PIN 23
#define SCL_PIN 22
#define GPS_RX 16
#define GPS_TX 17
#define HC12_RX 26
#define HC12_TX 27
#define MPU_ADDR 0x68

// ================= BME280 =================
uint8_t bmeAddress = 0;
bool bmeOK = false;

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3, dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

double tFine = 0;
double temperatureC = NAN;
double pressureHPa = NAN;
double humidityPct = NAN;
double bmeAltitudeM = NAN;

void bmeWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(bmeAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t bmeRead8(uint8_t reg) {
  Wire.beginTransmission(bmeAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(bmeAddress, (uint8_t)1) != 1) return 0;
  return Wire.read();
}

uint16_t bmeRead16(uint8_t reg) {
  uint16_t lo = bmeRead8(reg);
  uint16_t hi = bmeRead8(reg + 1);
  return (hi << 8) | lo;
}

int16_t bmeReadS16(uint8_t reg) {
  return (int16_t)bmeRead16(reg);
}

bool bmeInit() {
  bmeAddress = 0x76;

  if (bmeRead8(0xD0) != 0x60) {
    bmeAddress = 0x77;
    if (bmeRead8(0xD0) != 0x60) {
      bmeAddress = 0;
      return false;
    }
  }

  bmeWrite(0xE0, 0xB6);
  delay(10);

  unsigned long start = millis();
  while ((bmeRead8(0xF3) & 1) &&
         millis() - start < 1000) {
    delay(2);
  }

  dig_T1 = bmeRead16(0x88);
  dig_T2 = bmeReadS16(0x8A);
  dig_T3 = bmeReadS16(0x8C);

  dig_P1 = bmeRead16(0x8E);
  dig_P2 = bmeReadS16(0x90);
  dig_P3 = bmeReadS16(0x92);
  dig_P4 = bmeReadS16(0x94);
  dig_P5 = bmeReadS16(0x96);
  dig_P6 = bmeReadS16(0x98);
  dig_P7 = bmeReadS16(0x9A);
  dig_P8 = bmeReadS16(0x9C);
  dig_P9 = bmeReadS16(0x9E);

  dig_H1 = bmeRead8(0xA1);
  dig_H2 = bmeReadS16(0xE1);
  dig_H3 = bmeRead8(0xE3);

  uint8_t e4 = bmeRead8(0xE4);
  uint8_t e5 = bmeRead8(0xE5);
  uint8_t e6 = bmeRead8(0xE6);

  dig_H4 = ((int16_t)(int8_t)e4 << 4) | (e5 & 0x0F);
  dig_H5 = ((int16_t)(int8_t)e6 << 4) | (e5 >> 4);

  if (dig_H4 & 0x0800) dig_H4 -= 4096;
  if (dig_H5 & 0x0800) dig_H5 -= 4096;

  dig_H6 = bmeRead8(0xE7);

  if (dig_T1 == 0 || dig_P1 == 0) return false;

  bmeWrite(0xF2, 0x01);
  bmeWrite(0xF4, 0x27);
  bmeWrite(0xF5, 0xA0);
  delay(50);

  return true;
}

bool bmeReadRaw(uint32_t &rawP,
                uint32_t &rawT,
                uint16_t &rawH) {
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

double bmeGetTemperature(uint32_t adcT) {
  double v1 = ((double)adcT / 16384.0 -
               (double)dig_T1 / 1024.0) * dig_T2;

  double v2 = ((double)adcT / 131072.0 -
               (double)dig_T1 / 8192.0);

  v2 = v2 * v2 * dig_T3;
  tFine = v1 + v2;

  return tFine / 5120.0;
}

double bmeGetPressure(uint32_t adcP) {
  double v1 = tFine / 2.0 - 64000.0;
  double v2 = v1 * v1 * dig_P6 / 32768.0;

  v2 += v1 * dig_P5 * 2.0;
  v2 = v2 / 4.0 + (double)dig_P4 * 65536.0;

  v1 = (dig_P3 * v1 * v1 / 524288.0 +
        dig_P2 * v1) / 524288.0;

  v1 = (1.0 + v1 / 32768.0) * dig_P1;

  if (v1 == 0) return NAN;

  double p = 1048576.0 - adcP;
  p = (p - v2 / 4096.0) * 6250.0 / v1;

  v1 = dig_P9 * p * p / 2147483648.0;
  v2 = p * dig_P8 / 32768.0;
  p += (v1 + v2 + dig_P7) / 16.0;

  return p / 100.0;
}

double bmeGetHumidity(uint16_t adcH) {
  double h = tFine - 76800.0;

  h = (adcH -
       (dig_H4 * 64.0 + dig_H5 / 16384.0 * h)) *
      (dig_H2 / 65536.0 *
       (1.0 + dig_H6 / 67108864.0 * h *
        (1.0 + dig_H3 / 67108864.0 * h)));

  h *= 1.0 - dig_H1 * h / 524288.0;

  if (h > 100) h = 100;
  if (h < 0) h = 0;

  return h;
}

void updateBME() {
  if (!bmeOK) return;

  uint32_t rawP, rawT;
  uint16_t rawH;

  if (!bmeReadRaw(rawP, rawT, rawH)) return;

  temperatureC = bmeGetTemperature(rawT);
  pressureHPa = bmeGetPressure(rawP);
  humidityPct = bmeGetHumidity(rawH);

  if (pressureHPa > 0 && isfinite(pressureHPa)) {
    bmeAltitudeM =
      44330.0 * (1.0 -
      pow(pressureHPa / 1013.25, 0.1903));
  }
}

// ================= MPU6050 =================
bool mpuOK = false;
double ax = 0, ay = 0, az = 0;
double gx = 0, gy = 0, gz = 0;

bool mpuReadBlock(uint8_t reg,
                  uint8_t *data,
                  uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, count) != count)
    return false;

  for (uint8_t i = 0; i < count; i++)
    data[i] = Wire.read();

  return true;
}

bool mpuInit() {
  // Wake up the detected MPU6050 at address 0x68.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 wake-up failed");
    return false;
  }

  delay(100);

  // Accelerometer range: +/-2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // Gyroscope range: +/-250 degrees/second
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  uint8_t id = 0;

  if (!mpuReadBlock(0x75, &id, 1)) {
    Serial.println("Cannot read MPU6050 identity");
    return false;
  }

  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(id, HEX);

  uint8_t test[14];
  if (!mpuReadBlock(0x3B, test, 14)) {
    Serial.println("MPU6050 data read failed");
    return false;
  }

  Serial.println("MPU6050 initialized.");
  return true;
}

bool updateMPU() {
  uint8_t d[14];

  if (!mpuReadBlock(0x3B, d, 14)) return false;

  int16_t axRaw = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
  int16_t ayRaw = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
  int16_t azRaw = (int16_t)(((uint16_t)d[4] << 8) | d[5]);

  int16_t gxRaw = (int16_t)(((uint16_t)d[8] << 8) | d[9]);
  int16_t gyRaw = (int16_t)(((uint16_t)d[10] << 8) | d[11]);
  int16_t gzRaw = (int16_t)(((uint16_t)d[12] << 8) | d[13]);

  ax = axRaw / 16384.0;
  ay = ayRaw / 16384.0;
  az = azRaw / 16384.0;

  gx = gxRaw / 131.0;
  gy = gyRaw / 131.0;
  gz = gzRaw / 131.0;

  return true;
}

// ================= NEO-6M GPS =================
char nmeaLine[150];
size_t nmeaIndex = 0;

bool gpsFix = false;
bool gpsAltitudeValid = false;
bool speedValid = false;

double latitude = 0, longitude = 0;
double gpsAltitude = 0, speedKmph = 0;
int satellites = 0;

double gpsCoordinate(const char *value,
                     const char *direction) {
  if (!value || !direction || value[0] == '\0')
    return 0;

  double v = atof(value);
  int degrees = (int)(v / 100.0);
  double minutes = v - degrees * 100.0;

  double result = degrees + minutes / 60.0;

  if (direction[0] == 'S' || direction[0] == 'W')
    result = -result;

  return result;
}

void parseGPSLine(char *line) {
  if (line[0] != '$') return;

  char *fields[16];
  int count = 0;

  char *token = strtok(line, ",");

  while (token && count < 16) {
    fields[count++] = token;
    token = strtok(NULL, ",");
  }

  if (count < 2) return;

  if (strstr(fields[0], "GGA") && count >= 10) {
    gpsFix = atoi(fields[6]) > 0;
    satellites = atoi(fields[7]);

    if (gpsFix) {
      latitude = gpsCoordinate(fields[2], fields[3]);
      longitude = gpsCoordinate(fields[4], fields[5]);

      if (fields[9][0] != '\0') {
        gpsAltitude = atof(fields[9]);
        gpsAltitudeValid = true;
      }
    } else {
      gpsAltitudeValid = false;
    }
  }

  if (strstr(fields[0], "RMC") && count >= 9) {
    if (fields[2][0] == 'A') {
      latitude = gpsCoordinate(fields[3], fields[4]);
      longitude = gpsCoordinate(fields[5], fields[6]);
      speedKmph = atof(fields[7]) * 1.852;
      speedValid = true;
    } else {
      speedValid = false;
    }
  }
}

void updateGPS() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (c == '\r') continue;

    if (c == '\n') {
      nmeaLine[nmeaIndex] = '\0';
      parseGPSLine(nmeaLine);
      nmeaIndex = 0;
    } else if (nmeaIndex < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaIndex++] = c;
    } else {
      nmeaIndex = 0;
    }
  }
}

// ================= SETUP =================
unsigned long lastOutput = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\nESP32 SENSOR SYSTEM");
  Serial.println("BME280 + MPU6050 + NEO-6M + HC-12");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial1.begin(9600, SERIAL_8N1, HC12_RX, HC12_TX);

  bmeOK = bmeInit();

  if (bmeOK) {
    Serial.print("BME280 found at 0x");
    Serial.println(bmeAddress, HEX);
  } else {
    Serial.println("BME280 not detected");
  }

  mpuOK = mpuInit();
  Serial.println(mpuOK ? "MPU6050 OK" : "MPU6050 init failed");

  Serial.println("GPS: RX=16 TX=17");
  Serial.println("HC-12: RX=26 TX=27");
}

// ================= MAIN LOOP =================
void loop() {
  updateGPS();

  if (millis() - lastOutput >= 1000) {
    lastOutput = millis();

    updateBME();
    bool mpuReadOK = mpuOK && updateMPU();

    char report[500];
    char t[18], p[18], h[18], ba[18], ga[18], sp[18];

    if (bmeOK && isfinite(temperatureC))
      dtostrf(temperatureC, 0, 2, t);
    else strcpy(t, "NA");

    if (bmeOK && isfinite(pressureHPa))
      dtostrf(pressureHPa, 0, 2, p);
    else strcpy(p, "NA");

    if (bmeOK && isfinite(humidityPct))
      dtostrf(humidityPct, 0, 2, h);
    else strcpy(h, "NA");

    if (bmeOK && isfinite(bmeAltitudeM))
      dtostrf(bmeAltitudeM, 0, 2, ba);
    else strcpy(ba, "NA");

    if (gpsAltitudeValid)
      dtostrf(gpsAltitude, 0, 2, ga);
    else strcpy(ga, "NA");

    if (speedValid)
      dtostrf(speedKmph, 0, 2, sp);
    else strcpy(sp, "NA");

    snprintf(report, sizeof(report),
      "BME:T=%sC,P=%shPa,H=%s%%,ALT=%sm;"
      "GPS:FIX=%d,LAT=%.6f,LON=%.6f,GALT=%sm,SPD=%skmh,SATS=%d;"
      "MPU:OK=%d,AX=%.3fg,AY=%.3fg,AZ=%.3fg,"
      "GX=%.2fdps,GY=%.2fdps,GZ=%.2fdps",
      t, p, h, ba,
      gpsFix ? 1 : 0, latitude, longitude,
      ga, sp, satellites,
      mpuReadOK ? 1 : 0, ax, ay, az, gx, gy, gz);

    Serial.println(report);
    Serial1.println(report);
  }
}
