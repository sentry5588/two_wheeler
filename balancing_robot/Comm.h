/*
  Comm.h - Library for creating communications between Arduino and PC
  Created by sentry5588, Apr 2019
  MIT license
*/
#ifndef Comm_h
#define Comm_h

#include "Arduino.h"

class Comm {
  public:
    // Define the class constructor
    Comm(uint8_t scheduled_intv);
    // read MPU6050 sensor data
    void scheduled_send(void);

  private:
    uint8_t scheduled_intv = 0;
    uint8_t i = 0;
};

#endif
