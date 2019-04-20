/*
  global.h - Library for creating global variables
  Created by sentry5588, Apr 2019
  MIT license
*/
#ifndef global_h
#define global_h

#include <LinkedList.h>

// (m/s^2) local gravitational acceleration
extern const double g;

////////////////////// control variables declaration

// ====================================================================
// the measurement from sensor to the controller
// --------------------------------------------------------------------
// the state variable for the robot [x, y, z, dx, dy, dz]
// x, y, z: the angle positions of the robot in x, y, z directions
// dx, dy, dz: the angle velocities of the robot in x, y, z directions
extern double s[6];
extern double s_GOSI[6];
extern double s_AO[6];

// ====================================================================
// the control actions from controller to actuators
// --------------------------------------------------------------------
// To do

// To do: external double s[6];

// ====================================================================
// Debug variables
// --------------------------------------------------------------------
// Output data in linked list
extern LinkedList<double> debug_list;
// An counter as an indicator of the CPU idle time
extern unsigned long idle_loop_count;

#endif


