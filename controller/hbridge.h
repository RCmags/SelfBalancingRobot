//========== H-bridge controller ===========
/* Pulse frequency modulation to control DC motor like stepper motor */

#include <TimerOne.h>

//=============== Parameters ===============

// Timing
constexpr uint32_t TIME_ON              = 4000;                 // Width of duty cycle
constexpr uint32_t N_STEP               = 10;                   // Discretization steps of duty cycle 
constexpr uint32_t UPDATE_PERIOD        = TIME_ON / N_STEP;     // Interval at which signal is updated 

// Signal output
constexpr uint16_t N_WAVE               = 2;                    // Number of PFM waves to generate (1 per motor)
constexpr uint8_t  WAVE_PINS[N_WAVE][2] = { {5,6}, {9,10} };    // Digital pins for each motor

//============ Wave generator ==============

class FrequencyPulse {
  private:
    uint32_t _wave_period[N_WAVE] = {TIME_ON};
    uint32_t _dt[N_WAVE] = {0};
    uint8_t _pin[N_WAVE] = {false};
  
  public:
    void setRate( float rate[] ) {
      for( int i = 0; i < N_WAVE; i += 1 ) {
          // turn off
        if( rate[i] == 0 ) {
          _pin[i] == false;
           digitalWrite( WAVE_PINS[i][0], LOW );
           digitalWrite( WAVE_PINS[i][1], LOW );

          // modulate period
        } else {
          float rate_i = constrain(rate[i], -1, 1);
          
          if( rate_i > 0 ) {
            _pin[i] = WAVE_PINS[i][0];
            digitalWrite( WAVE_PINS[i][1], LOW );
          
          } else if ( rate_i < 0 ) {
            _pin[i] = WAVE_PINS[i][1]; 
            digitalWrite( WAVE_PINS[i][0], LOW ); 
            rate_i = -rate_i;
          }
       
          _wave_period[i] = TIME_ON * (1.0/rate_i - 1);
        }  
      }
    }

    // interrupt function
    void update() {
      for( int i = 0; i < N_WAVE; i += 1 ) {
        // infinite period
        if( _pin[i] == false ) {
          continue;
        } 

        // finite period
        _dt[i] += UPDATE_PERIOD;
        
        if( _dt[i] > TIME_ON ) {
          digitalWrite( _pin[i], LOW );
        }
        if( _dt[i] > _wave_period[i] ) {
          _dt[i] = 0;
          digitalWrite( _pin[i], HIGH ); 
        }
      }
    }
};

//============ Helper functions =============

FrequencyPulse motors;

void timerFunc() {
  motors.update();
}

void setupPulses() {
  // set pins
  for( int i = 0; i < N_WAVE; i += 1 ) {  // scan motors
    for( int k = 0; k < 2; k += 1 ) {     // scan H-bridge pins
      int pin = WAVE_PINS[i][k];
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }
  // timer interrupt
  Timer1.initialize(UPDATE_PERIOD);
  Timer1.attachInterrupt(timerFunc); 
}
