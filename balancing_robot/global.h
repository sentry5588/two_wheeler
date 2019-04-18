/*
  global.h - Library for creating global variables
  Created by sentry5588, Apr 2019
  MIT license
*/
#ifndef global_h
#define global_h

#include <LinkedList.h>

////////////////////// control variables declaration

// ====================================================================
// the measurement from sensor to the controller
// --------------------------------------------------------------------
// the state variable for the robot [x, y, z, dx, dy, dz]
// x, y, z: the angle positions of the robot in x, y, z directions
// dx, dy, dz: the angle velocities of the robot in x, y, z directions
extern double s[6];

// ====================================================================
// the control actions from controller to actuators
// --------------------------------------------------------------------
// To do

// To do: external double s[6];

// ====================================================================
// Debug variables
// --------------------------------------------------------------------
extern LinkedList<double> debug_list;

#endif


