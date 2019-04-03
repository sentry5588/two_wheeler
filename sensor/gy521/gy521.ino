#include<Wire.h>
#include "state_est_gyro_only_simple_int.c"
const int MPU=0x68; // The address for MPU6050 when AD0 is not set
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // declare variables

// Following offset value is obtained for my particular chip
const int16_t GyX_offset=477, GyY_offset=124, GyZ_offset=182;
double x_total_deg=0, y_total_deg=0, z_total_deg=0;
unsigned long previous_millis = 0;
const long loop_time_step = 10; // milli-second (ms)
unsigned int serial_comm_i = 0;  // counter for serial communication

////////////////////// control variables declaration
// the state variable for the robot [ax, ay, az, dax, day, daz]
// ax, ay, az: the angle position of robot in x, y, z directions
// dax, day, daz: the angle velocity of robot in x, y, z directions
struct state{
  double ax, ay, az, dax, day, daz;
} s = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0

void setup(){ // setup communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200); // baud rate of 9600 is too slow
}
void loop(){
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

    GyX = GyX - GyX_offset; // gyro x direction calibration
    GyY = GyY - GyY_offset; // gyro y direction calibration
    GyZ = GyZ - GyZ_offset; // gyro z direction calibration

    s = 
    
    serial_comm_i = serial_comm_i + 1; // increase serial counter by 1
    if (serial_comm_i >= 10) { // only send data via serial port every 10 loops
      serial_comm_i = 0; // reset serial communication counter
      // Try to limit the data via serial, it's slow. It'll limit loop time
//      Serial.print(int16_t(gy_conv_factor)); Serial.print(", ");
//      Serial.print(AcX); Serial.print(", ");
//      Serial.print(AcY); Serial.print(", ");
//      Serial.print(AcZ); Serial.print(", ");
//      Serial.print(GyX); Serial.print(", ");
//      Serial.print(GyY); Serial.print(", ");
//      Serial.print(GyZ); Serial.print(", ");
//      Serial.print(Tmp); Serial.print(", ");
      Serial.print(int16_t(state[0])); Serial.print(", ");
      Serial.print(int16_t(state[1])); Serial.print(", ");
      Serial.print(int16_t(state[2])); Serial.print(", ");
      Serial.println(current_millis);
    } // if (serial_comm_i >= 10) { // only send data via serial port every 10 loops
  } // if (this_time_step >= loop_time_step) { // if longer than the time step
} // void loop(){

