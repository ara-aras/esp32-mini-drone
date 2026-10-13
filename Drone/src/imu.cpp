#include "imu.h"
#include "config.h"
#include "logbuf.h"
#include <Wire.h>

namespace {
const uint8_t REG_SMPLRT_DIV    = 0x19;
const uint8_t REG_CONFIG        = 0x1A;
const uint8_t REG_GYRO_CONFIG   = 0x1B;
const uint8_t REG_ACCEL_CONFIG  = 0x1C;
const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
const uint8_t REG_PWR_MGMT_1    = 0x6B;
const uint8_t REG_WHO_AM_I      = 0x75;

const float GYRO_LSB_PER_DPS = 16.4f;   // +/-2000 dps
const float ACC_LSB_PER_G    = 4096.0f; // +/-8 g

float biasX = 0, biasY = 0, biasZ = 0;
bool calibrated = false;
float lpfX = 0, lpfY = 0, lpfZ = 0;
float lpfAlpha = 1.0f;
bool lpfPrimed = false;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)IMU_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Raw read, mapped into body frame, no bias removed.
bool readBody(ImuSample &s) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  // b[6..7] = temperature, unused
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  IMU_MAP(ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G,
          s.ax, s.ay, s.az);
  IMU_MAP(gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS,
          s.gx, s.gy, s.gz);
  return true;
}
}  // namespace

bool imuBegin() {
  // Bus recovery: a reset mid-transfer can leave the MPU holding SDA low.
  // Clocking SCL up to 9 times lets it finish the byte and release the bus.
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);  // default is tens of ms: too long to hold motors

  uint8_t who = 0;
  bool got = false;
  for (int attempt = 0; attempt < 5 && !got; attempt++) {
    got = readRegs(REG_WHO_AM_I, &who, 1);
    if (!got) delay(20);
  }
  if (!got) {
    dlog("[imu] no response on I2C - check wiring/address\n");
    return false;
  }
  dlog("[imu] WHO_AM_I = 0x%02X\n", who);
  if (who != 0x68) {
    // Many clone chips report 0x70/0x72/0x98 and still work.
    dlog("[imu] warning: not a genuine MPU6050 ID, continuing anyway\n");
  }

  writeReg(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);   // wake, PLL on gyro X
  delay(50);
  ok &= writeReg(REG_SMPLRT_DIV, 0x00);   // 1 kHz internal rate
  ok &= writeReg(REG_CONFIG, 0x02);       // DLPF ~94 Hz
  ok &= writeReg(REG_GYRO_CONFIG, 0x18);  // +/-2000 dps
  ok &= writeReg(REG_ACCEL_CONFIG, 0x10); // +/-8 g
  if (!ok) {
    dlog("[imu] register config failed\n");
    return false;
  }

  float dt = 1.0f / LOOP_HZ;
  float rc = 1.0f / (2.0f * PI * GYRO_LPF_HZ);
  lpfAlpha = dt / (rc + dt);
  return true;
}

bool imuCalibrateGyro() {
  double sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;
  int n = 0;
  ImuSample s;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (readBody(s)) {
      sx += s.gx; sy += s.gy; sz += s.gz;
      qx += s.gx * s.gx; qy += s.gy * s.gy; qz += s.gz * s.gz;
      n++;
    }
    delay(2);
  }
  if (n < GYRO_CAL_SAMPLES / 2) {
    dlog("[imu] calibration failed: too many read errors\n");
    return false;
  }
  double mx = sx / n, my = sy / n, mz = sz / n;
  double dx = sqrt(max(0.0, qx / n - mx * mx));
  double dy = sqrt(max(0.0, qy / n - my * my));
  double dz = sqrt(max(0.0, qz / n - mz * mz));
  dlog("[imu] gyro bias %.2f %.2f %.2f  std %.2f %.2f %.2f\n",
                (float)mx, (float)my, (float)mz, (float)dx, (float)dy, (float)dz);
  if (dx > GYRO_CAL_MAX_STD_DPS || dy > GYRO_CAL_MAX_STD_DPS ||
      dz > GYRO_CAL_MAX_STD_DPS) {
    dlog("[imu] calibration rejected: drone was moving\n");
    return false;
  }
  biasX = mx; biasY = my; biasZ = mz;
  lpfPrimed = false;
  calibrated = true;
  return true;
}

bool imuIsCalibrated() { return calibrated; }

bool imuRead(ImuSample &s) {
  if (!readBody(s)) return false;
  float gx = s.gx - biasX, gy = s.gy - biasY, gz = s.gz - biasZ;
  if (!lpfPrimed) {
    lpfX = gx; lpfY = gy; lpfZ = gz;
    lpfPrimed = true;
  } else {
    lpfX += lpfAlpha * (gx - lpfX);
    lpfY += lpfAlpha * (gy - lpfY);
    lpfZ += lpfAlpha * (gz - lpfZ);
  }
  s.gx = lpfX; s.gy = lpfY; s.gz = lpfZ;
  return true;
}
