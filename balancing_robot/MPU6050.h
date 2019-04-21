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
    double AcX_deg_s2 = 0.0, AcY_deg_s2 = 0.0, AcZ_deg_s2 = 0.0,
           GyX_deg_s = 0.0, GyY_deg_s = 0.0, GyZ_deg_s = 0.0;
    // Define the class constructor
    MPU6050(double AcX_offset_in, double AcY_offset_in, double AcZ_offset_in,
            double GyX_offset_in, double GyY_offset_in,  double GyZ_offset_in);
    // read MPU6050 sensor data
    void sensor_read(int MPU_addr);
    // correct the offset of the gyro readings
    void gyro_corr_offset(void);
    // Estimate robot state using gyro data only
    void state_est_GOSI(unsigned long dt);
    // Estimate robot state using accelerometer data only
    void state_est_AO(unsigned long dt);
    // Estimate robot state using complementary filter
    void state_est_CF(unsigned long dt);

  private:
    // convertion rate for gyro (total range is +/-250 deg/s
    const double gy_conv_factor = 250.0 / 32767.0;
    int16_t AcX_offset = 0, AcY_offset = 0, AcZ_offset = 0,
            GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;
    // Using following estimate lock is to avoid programming mistakes
    bool estimate_complete = 0;
    // To flag whether GOSI method finished the initilization using accel. data
    bool GOSI_init_flag = 0;
};

#endif
