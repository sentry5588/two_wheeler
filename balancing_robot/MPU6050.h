/*
  MPU6050.h - Library for creating a MPU6050 sensor object
  Created by sentry5588, Apr 2019
  MIT license
*/
#ifndef MPU6050_h
#define MPU6050_h

#include <Wire.h>
#include "Arduino.h"

class MPU6050 {
  public:
    // variables for raw data
    int16_t AcX = 0, AcY = 0, AcZ = 0, Tmp = 0, GyX = 0, GyY = 0, GyZ = 0;
    // offset corrected data in degree
    double AcX_deg = 0.0, AcY_deg = 0.0, AcZ_deg = 0.0,
           GyX_deg = 0.0, GyY_deg = 0.0, GyZ_deg = 0.0;
    // Define the class constructor
    MPU6050(int16_t GyX_offset, int16_t GyY_offset, int16_t GyZ_offset);
    // read MPU6050 sensor data
    void sensor_read(int MPU_addr);
    // correct the offset of the gyro readings
    void gyro_corr_offset(void);
    // ISSUE: How not to use global variable "s" in this method?
    void state_est_GOSI(unsigned long dt);

  private:
    // convertion rate for gyro (total range is +/-250 deg/s
    const double gy_conv_factor = 250.0 / 32767.0;
    int16_t GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;
};

#endif
