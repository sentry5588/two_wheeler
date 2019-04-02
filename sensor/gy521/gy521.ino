#include<Wire.h>
const int MPU=0x68; // The address for MPU6050 when AD0 is not set
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // declare variables

// Following offset value is obtained for my particular chip
const int16_t GyX_offset=477, GyY_offset=124, GyZ_offset=182;
double x_total_deg=0, y_total_deg=0, z_total_deg=0;
unsigned long previous_millis = 0;
const long loop_time_step = 10; // milli-second (ms)

void setup(){ // setup communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200); // baud rate of 9600 is too slow
}
void loop(){
  unsigned long current_millis = millis(), this_time_step = 0;
  this_time_step = current_millis - previous_millis;
  if (this_time_step >= loop_time_step) {
    // save the last time the code ran
    previous_millis = current_millis;
    // printf("%ld\n", current_millis);

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

    GyX = GyX - GyX_offset;
    GyY = GyY - GyY_offset;
    GyZ = GyZ - GyZ_offset;

    // with current setup, the gyro range is +/-250 deg/s
    x_total_deg += double(GyX) /32767.0 * 250.0 * double(this_time_step) / 1000.0;
    y_total_deg += double(GyY) /32767.0 * 250.0 * double(this_time_step) / 1000.0;
    z_total_deg += double(GyZ) /32767.0 * 250.0 * double(this_time_step) / 1000.0;

    // Try to limit the data via serial, it's slow. It'll limit loop time
    Serial.print(AcX); Serial.print(", ");
    Serial.print(AcY); Serial.print(", ");
    Serial.print(AcZ); Serial.print(", ");
    Serial.print(GyX); Serial.print(", ");
    Serial.print(GyY); Serial.print(", ");
    Serial.print(GyZ); Serial.print(", ");
    Serial.print(Tmp); Serial.print(", ");
    Serial.print(int16_t(x_total_deg)); Serial.print(", ");
    Serial.print(int16_t(y_total_deg)); Serial.print(", ");
    Serial.print(int16_t(z_total_deg)); Serial.print(", ");
    Serial.println(current_millis);
  }
//  Serial.println(current_millis);
}


