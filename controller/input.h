//============ Receiver Inputs =============
/* PWM signals from receiver to command vehicle */

#include <PulseInput.h> 

//=============== Parameters ===============

// pin change interrupt pins
constexpr int   RX_PIN1   = 2;
constexpr int   RX_PIN2   = 3;

// signal filter
constexpr int   BAND_MID  = 30;       // deadband at mid stick           
constexpr int   PWM_MID   = 1550;     // PWM at mid stick

// output 
constexpr float GAIN_RX[] = { -2.0 ,  // yaw 
                             -0.01 }; // pitch

//========== Pin change interrupt ==========

volatile unsigned int pwm_raw1, pwm_raw2;      

void setPwmInputs() {
  attachPulseInput(RX_PIN1, pwm_raw1);
  attachPulseInput(RX_PIN2, pwm_raw2);
}

//=============== Filter ==================

float deadband(float x) {
  if( x > BAND_MID ) {
    return x - BAND_MID;
  } else if ( x < -BAND_MID ) {
    return x + BAND_MID;
  } else {
    return 0;
  }
}

void filterInputs(float *input) {
  // get update
  input[0] = pwm_raw1;
  input[1] = pwm_raw2;
  
  // filter and scale
  constexpr float SCALE = 1.0 / 500.0; 
  
  for( int i = 0; i < 2; i += 1 ) {
    input[i] = input[i] - PWM_MID;
    input[i] = deadband( input[i] ); 
    input[i] *= SCALE * GAIN_RX[i];
  }
}
