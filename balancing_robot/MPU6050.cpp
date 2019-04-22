#include <Wire.h>
#include "Arduino.h"
#include "MPU6050.h"
#include "global.h"

// Define the class constructor
MPU6050::MPU6050(double AcX_offset_in, double AcY_offset_in, double AcZ_offset_in,
                 double GyX_offset_in, double GyY_offset_in, double GyZ_offset_in) {
  AcX_offset = AcX_offset_in;
  AcY_offset = AcY_offset_in;
  AcZ_offset = AcZ_offset_in;
  GyX_offset = GyX_offset_in;
  GyY_offset = GyY_offset_in;
  GyZ_offset = GyZ_offset_in;
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

  // Once new raw data is available, clear estimate lock to allow new estimation
  // Using estimate lock is to avoid programming mistakes
  estimate_complete = 0;
}

// correct the offset of the gyro readings
void MPU6050::gyro_corr_offset(void) {
  GyX_deg_s = double(GyX - GyX_offset) * gy_conv_factor ; // gyro x velocity (deg/s)
  GyY_deg_s = double(GyY - GyY_offset) * gy_conv_factor ; // gyro y velocity (deg/s)
  GyZ_deg_s = double(GyZ - GyZ_offset) * gy_conv_factor ; // gyro z velocity (deg/s)
}

// ISSUE: How not to use global variable "s" in this method?
void MPU6050::state_est_GOSI(unsigned long dt) {
  // integrate angular velocity to find angular positions
  // GOSI = Gyro Only Simply Integrator

  if (GOSI_init_flag == 0) {
    s_GOSI[0] = s_AO[0];
    s_GOSI[1] = s_AO[1];
    s_GOSI[2] = s_AO[2];
    GOSI_init_flag = 1;
  }

  // Accumulate angular velocities to angular positions
  s_GOSI[0] = s_GOSI[0] + GyX_deg_s * (double(dt)) / 1000.0;
  s_GOSI[1] = s_GOSI[1] + GyY_deg_s * (double(dt)) / 1000.0;
  s_GOSI[2] = s_GOSI[2] + GyZ_deg_s * (double(dt)) / 1000.0;

  // directly assign values to angular velocities
  s_GOSI[3] = GyX_deg_s;
  s_GOSI[4] = GyY_deg_s;
  s_GOSI[5] = GyZ_deg_s;
}

void MPU6050::state_est_AO(unsigned long dt) {
  // Estimate robot state using accelerometer data only
  // AO = Accelerometer Only
  // Use rotation/scaling matrix to scale and rotate axis
  s_AO[0] =  0.005308863 * AcX + 0.000101228 * AcY - 0.000236367 * AcZ;
  s_AO[1] = -0.000017130 * AcX - 0.005480948 * AcY - 0.000103338 * AcZ;
  s_AO[2] = -0.000843155 * AcX + 0.000314773 * AcY - 0.005117289 * AcZ;
}

void MPU6050::state_est_CF(unsigned long dt) {
  // Estimate robot state using complementary filter
  // CF = Complementary Filter
  double CF_factor = 0.98; // "Trust" on gyro reading
  double gy_acc[3] = {0.0, 0.0, 0.0};

  gy_acc[0] = s_CF[0] + GyX_deg_s * (double(dt)) / 1000.0;
  gy_acc[1] = s_CF[1] + GyY_deg_s * (double(dt)) / 1000.0;
  gy_acc[2] = s_CF[2] + GyZ_deg_s * (double(dt)) / 1000.0;
  
  for (int i = 0; i <= 2; i++) {    
    s_CF[i] =  CF_factor * gy_acc[i] + (1 - CF_factor) * s_AO[i];
  }
}

