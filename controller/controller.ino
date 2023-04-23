/* Arduino NANO sketch to stabilize and control a self-balancing robot (similar to a segway).
 * It commands two DC motors via and h-bridge to rotate and translate the vehicle.
 * Author: RCmags https://github.com/RCmags
 * 
 * NOTE: 
 * This sketch uses an MPU6050 IMU (gyroscope and accelerometer) to stabilize pitch axis. It 
 * does so by constantly accelerating into the direction of tilt to remain upright.
 * 
 * Important! Check the header files to configure the controller. See the "parameters" sections.
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 2  -> Receiver CH1
// Pin 3  -> Receiver CH2
// Pin A5 -> Sensor SCL
// Pin A4 -> Sensor SDA
  // Outputs:
// Pin 5  -> Left  motor pin
// Pin 6  -> Left  motor pin 
// Pin 9  -> Right motor pin
// Pin 10 -> Right motor pin

//=================== Code ===================
#include "fusion.h"
#include "hbridge.h"
#include "input.h"
#include "pid.h"

void setup() {     
  // RX signals
  setPwmInputs();
  delay(1000);      // allow receiver to initialize
  
  // Calibrate imu
  imu.setup();
  imu.setBias();
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     

  // setup h-bridge
  setupPulses();
}

void loop() {
  // update attitude
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN, SD_ACCEL );  

  // get receiver inputs
  float input[2]; filterInputs(input);

  // set motors
  float rate[2]; PIDcontrollers(input, rate);
  motors.setRate(rate);
}
