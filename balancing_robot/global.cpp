#include "global.h"
#include <LinkedList.h>
#include "Arduino.h"

const double g = 9.82077;
double s[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0
double s_GOSI[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0
double s_AO[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0
double s_CF[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0
LinkedList<double> debug_list = LinkedList<double>();
unsigned long idle_loop_count = 0;

