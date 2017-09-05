#include "freeram.h"
#include "MPU9250.h"
#include "mpu.h"
#include "I2Cdev.h"

void serialFloatPrint(float);
const int pin = A1;
const int FLEX_PIN = A2;
float valueRead;
const float voltage = 5.01;
const float resistor = 99700;
const float R_DIV = 45900.0; // Measured resistance of 3.3k resistor
const float STRAIGHT_RESISTANCE = 22150.0; // resistance when straight
const float BEND_RESISTANCE = 110500.0; // resistance at 90 deg
const float fixed_angle_tangent = -0.27559;
const float fixed_angle_bias = 64.10787;
float force;

MPU9250 myIMU;

int ret;
void setup() {
    Fastwire::setup(400,0);
    pinMode(pin, INPUT);
    pinMode(FLEX_PIN, INPUT);
//    valueRead = 0.00f; //This is for EEPROM
    Serial.begin(115200);
    ret = mympu_open(200);
    freeRam();
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    myIMU.initMPU9250();
    myIMU.initAK8963(myIMU.magCalibration);
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
   mympu_update();
   int reading = analogRead(pin);
   int flexADC = analogRead(FLEX_PIN);
   float flexV = flexADC * voltage / 1023.0;
   float flexR = R_DIV * (voltage / flexV - 1.0);
   float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 180.0);
   angle = fixed_angle_tangent * angle + fixed_angle_bias;
   if(angle < 0){ angle = 0.0; }
   
   if (reading != 1 && reading != 0) {
      float fsrVoltage = reading * voltage / 1023.0;
      float fsrResistance = resistor * (voltage / fsrVoltage - 1.0);
      float conductance = 1 / fsrResistance;

      if (fsrResistance <= 600) 
        force = (conductance - 0.00075) / 0.00000032639 ;
      else
        force =  conductance / 0.000000642857;
      }

    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

      Serial.print('$');
      Serial.print(mympu.ypr[0]); Serial.print(',');
      Serial.print(mympu.ypr[1]); Serial.print(',');
      Serial.print(mympu.ypr[2]); Serial.print(',');
      Serial.print(mympu.gyro[0]); Serial.print(',');
      Serial.print(mympu.gyro[1]); Serial.print(',');
      Serial.print(mympu.gyro[2]); Serial.print(',');
      Serial.print(mympu.accel[0]); Serial.print(',');
      Serial.print(mympu.accel[1]); Serial.print(',');
      Serial.print(mympu.accel[2]); Serial.print(',');
      Serial.print(myIMU.mx); Serial.print(',');
      Serial.print(myIMU.my); Serial.print(',');
      Serial.print(myIMU.mz); Serial.print(',');
      Serial.print(mympu.q[0]); Serial.print(',');
      Serial.print(mympu.q[1]); Serial.print(',');
      Serial.print(mympu.q[2]); Serial.print(',');
      Serial.print(mympu.q[3]); Serial.print(',');
      Serial.print(force); Serial.print(',');
      Serial.println(angle);
}

