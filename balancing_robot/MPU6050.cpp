#include <Wire.h>
#include "Arduino.h"
#include "MPU6050.h"
#include "global.h"

// Define the class constructor
MPU6050::MPU6050(int16_t GyX_offset, int16_t GyY_offset, int16_t GyZ_offset) {
  GyX_offset = GyX_offset;
  GyY_offset = GyY_offset;
  GyZ_offset = GyZ_offset;
}

// read MPU6050 sensor data
void MPU6050::sensor_read(int MPU_addr) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 7 * 2, true); // Total 7x2=14 bytes to be requested
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// correct the offset of the gyro readings
void MPU6050::gyro_corr_offset(void) {
  GyX_deg = double(GyX - GyX_offset) * gy_conv_factor ; // gyro x velocity (deg/s)
  GyY_deg = double(GyY - GyY_offset) * gy_conv_factor ; // gyro y velocity (deg/s)
  GyZ_deg = double(GyZ - GyZ_offset) * gy_conv_factor ; // gyro z velocity (deg/s)
}

// ISSUE: How not to use global variable "s" in this method?
void MPU6050::state_est_GOSI(unsigned long dt) {
  // integrate angular velocity to find angular positions
  // GOSI = Gyro Only Simply Integrator
  s[0] = s[0] + GyX_deg * (double(dt)) / 1000.0;
  s[1] = s[1] + GyY_deg * (double(dt)) / 1000.0;
  s[2] = s[2] + GyZ_deg * (double(dt)) / 1000.0;

  // directly assign values to angular velocities
  s[3] = GyX_deg;
  s[4] = GyY_deg;
  s[5] = GyZ_deg;
}

