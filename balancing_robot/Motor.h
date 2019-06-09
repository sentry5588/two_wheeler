/*
  Motor.h - Library for interfacing with stepper motors
  Created by sentry5588, Apr 2019
  MIT license
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    // Define the class constructor
    Motor(unsigned int Dir_Pin, unsigned int Step_Pin);
    // Set motor speed
    void set_speed(double omega, bool d, long dt);

  private:
    const unsigned int STEPS_PER_REV = 200;
    unsigned int dir_pin = 0;
    unsigned int step_pin = 0;
};

#endif
