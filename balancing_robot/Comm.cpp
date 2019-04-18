#include <Wire.h>
#include "Comm.h"
#include "Arduino.h"
#include "MPU6050.h"
#include "global.h"

Comm::Comm(uint8_t s) {
  scheduled_intv = s;
  i = 0; // Initialize the counter
  Serial.print(debug_list.size()); Serial.print(", ");
  Serial.print(i); Serial.print(", ");
  Serial.println(scheduled_intv);
}

void Comm::scheduled_send(unsigned long current_millis) {
  i = i + 1;
  if (i >= scheduled_intv) {
    i = 0; // reset serial communication counter
    // Try to limit the data via serial, it's slow. It'll limit loop time
    Serial.print(s[0]); Serial.print(", ");
    Serial.print(s[1]); Serial.print(", ");
    Serial.print(s[2]); Serial.print(", ");
    Serial.print(debug_list.size()); Serial.print(", ");
    Serial.print(i); Serial.print(", ");
    Serial.print(scheduled_intv); Serial.print(", ");
    Serial.println(current_millis);
  }
}

