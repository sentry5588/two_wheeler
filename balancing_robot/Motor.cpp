#include <Wire.h>
#include "Motor.h"
#include "Arduino.h"
#include "global.h"

Motor::Motor(unsigned int Dir_Pin, unsigned int Step_Pin) {
  dir_pin = Dir_Pin;
  step_pin = Step_Pin; 
}

void Motor::set_speed(double omega, bool d, long dt) {
  // omega: required angular velocity (rad/s)
  // dt: controller cycle, in milli-second (ms)
  unsigned int s = 0; // run steps in one control cycle
  unsigned int s_period = 1200; // delay between motor steps (micro-second)
  s = int(double(dt) * 0.001 * omega / (2.0 * PI) * double(STEPS_PER_REV));
  debug_list.add(s);
  debug_list.add(double(dir_pin));
  debug_list.add(double(step_pin));
  debug_list.add(double(s_period));
  if (d == FORWARD){
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }

  for (int i = 0; i < s; i++) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(s_period);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(s_period);
  }
}

