/*
   The main program for controlling the self-balancing robot
   Created by sentry5588, Apr 2019
   MIT License
*/
#include<Wire.h>
#include <LinkedList.h>
#include "global.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Comm.h"

const int MPU = 0x68; // The address for MPU6050 when AD0 is not set
unsigned long previous_millis = 0;
const long loop_time_step = 10; // milli-second (ms)
unsigned int serial_comm_i = 0;  // counter for serial communication

// instantiation of MPU6050 for position and velocity
// using offset values obtained for my particular chip
MPU6050 pv_sensor(double(16829.33), double(-0.31663), double(-2772.92),
                  double(477.0), double(124.0), double(182.0));
// Instantiation of left stepper motor and right stepper motor
Motor lm, rm; // lm: left motor; rm: right motor

// instantiation of debug list
Comm send_debug(uint8_t(1));

// To do: move setup into class definition
void setup() { // setup communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200); // baud rate of 9600 is too slow
}

void loop() {
  // declare current time and time step from last execution
  unsigned long current_millis = millis(), this_time_step = 0;
  // calc how many ms passed since last time the code was ran
  this_time_step = current_millis - previous_millis;

  // Main control loop
  if (this_time_step >= loop_time_step) { // if longer than the time step
    previous_millis = current_millis; // save the last time the code ran

    // Sensor data reading, processing and state estimation ======================
    pv_sensor.sensor_read(MPU); // read sensor raw data
    pv_sensor.gyro_corr_offset(); // correct sensor offset
    pv_sensor.state_est_AO(this_time_step); // use AO to estimate robot states
    pv_sensor.state_est_GOSI(this_time_step); // use GOSI to estimate robot states
    pv_sensor.state_est_CF(this_time_step); // use GOSI to estimate robot states

    // Compute controller output using different control algorithms ==============
    // To do

    // Send controller output to actuators =======================================
    // To do

    // Control debug: 1) live value display, 2) data log, 3) troubleshooting ====
    // Reset CPU idle time indicator
    idle_loop_count++;
    debug_data_manage(current_millis);
    send_debug.scheduled_send();
    debug_list.clear();
    idle_loop_count = 0; // Reset CPU idle time indicator 
  } else {
    idle_loop_count++; // Increase CPU idle time counter by 1
  } // if (this_time_step >= loop_time_step) { // if longer than the time step
} // void loop(){

void debug_data_manage(unsigned long current_millis) {
  // Try to limit the data via serial, it's slow. It'll limit loop time
  debug_list.add(double(current_millis));
  debug_list.add(double(idle_loop_count));
  debug_list.add(s_GOSI[0]);
  debug_list.add(s_GOSI[1]);
  debug_list.add(s_GOSI[2]);
  debug_list.add(s_AO[0]);
  debug_list.add(s_AO[1]);
  debug_list.add(s_AO[2]);
  debug_list.add(s_CF[0]);
  debug_list.add(s_CF[1]);
  debug_list.add(s_CF[2]);
}

