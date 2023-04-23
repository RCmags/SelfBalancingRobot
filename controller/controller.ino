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
  // get attitude
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN, SD_ACCEL );  

  // get rx input
  float input[2]; filterInputs(input);

  // set motors
  float rate[2]; PIDcontrollers(input, rate);
  motors.setRate(rate);
}
