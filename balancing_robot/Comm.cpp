#include <Wire.h>
#include "Comm.h"
#include "Arduino.h"
#include "MPU6050.h"
#include "global.h"

Comm::Comm(uint8_t s) {
  scheduled_intv = s;
  i = 0; // Initialize the counter
}

void Comm::scheduled_send(void) {
  i = i + 1;
  if (i >= scheduled_intv) {
    i = 0; // reset serial communication counter
    while (debug_list.size() > 0) {
      Serial.print(debug_list.shift()); Serial.print(", ");
    }
    Serial.println(debug_list.size());
  }
}

