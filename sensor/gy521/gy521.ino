#include<Wire.h>

const int MPU=0x68; // The address for MPU6050 when AD0 is not set
// Following offset value is obtained for my particular chip
const int16_t GyX_offset=477, GyY_offset=124, GyZ_offset=182;
double x_total_deg=0, y_total_deg=0, z_total_deg=0;
unsigned long previous_millis = 0;
const long loop_time_step = 10; // milli-second (ms)
unsigned int serial_comm_i = 0;  // counter for serial communication
////////////////////// control variables declaration
// the state variable for the robot [x, y, z, dx, dy, dz]
// x, y, z: the angle positions of the robot in x, y, z directions
// dx, dy, dz: the angle velocities of the robot in x, y, z directions
double s[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0

void setup(){ // setup communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200); // baud rate of 9600 is too slow
}

void loop(){
  // variables for raw data
  int16_t AcX=0, AcY=0, AcZ=0, Tmp=0, GyX=0, GyY=0, GyZ=0;
  // variables for converted data in deg
  double AcX_deg, AcY_deg, AcZ_deg, GyX_deg, GyY_deg, GyZ_deg;
  // declare current time and time step from last execution
  unsigned long current_millis = millis(), this_time_step = 0;
  // convertion rate for gyro (total range is +/-250 deg/s
  const double gy_conv_factor = 250.0 / 32767.0;

  // calc how many ms passed since last time the code was ran
  this_time_step = current_millis - previous_millis;
  if (this_time_step >= loop_time_step) { // if longer than the time step
    previous_millis = current_millis; // save the last time the code ran

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,7*2,true); // Total 7x2=14 bytes will be requested
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    Tmp=Wire.read()<<8|Wire.read();
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();

    GyX_deg = double(GyX - GyX_offset) * gy_conv_factor ; // gyro x velocity (deg/s)
    GyY_deg = double(GyY - GyY_offset) * gy_conv_factor ; // gyro y velocity (deg/s)
    GyZ_deg = double(GyZ - GyZ_offset) * gy_conv_factor ; // gyro z velocity (deg/s)

    state_est_GOSI(GyX_deg, GyY_deg, GyZ_deg, this_time_step);
    
    serial_comm_i = serial_comm_i + 1; // increase serial counter by 1
    if (serial_comm_i >= 10) { // only send data via serial port every 10 loops
      serial_comm_i = 0; // reset serial communication counter
      // Try to limit the data via serial, it's slow. It'll limit loop time
      Serial.print(s[0]); Serial.print(", ");
      Serial.print(s[1]); Serial.print(", ");
      Serial.print(s[2]); Serial.print(", ");
      Serial.println(current_millis);
    } // if (serial_comm_i >= 10) { // only send data via serial port every 10 loops
  } // if (this_time_step >= loop_time_step) { // if longer than the time step
} // void loop(){

void state_est_GOSI(double x_GOSI, double y_GOSI, double z_GOSI, unsigned long dt) {
  // integrate angular velocity to find angular positions
  // GOSI = Gyro Only Simply Integrator
  s[0] = s[0] + x_GOSI * (double(dt)) / 1000.0;
  s[1] = s[1] + y_GOSI * (double(dt)) / 1000.0;
  s[2] = s[2] + z_GOSI * (double(dt)) / 1000.0;

  // directly assign values to angular velocities
  s[3] = x_GOSI;
  s[4] = y_GOSI;
  s[5] = z_GOSI;
}
