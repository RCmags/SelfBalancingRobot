/* Sketch to stabilize a self-balacing robot with an MPU-6050 */

#include <Wire.h>

//-- Constants

#define MOTOR_A_PIN_1     3   // PWM-enabled pins for motor H-bridge.
#define MOTOR_A_PIN_2     5
#define MOTOR_B_PIN_1     6
#define MOTOR_B_PIN_2     9

#define CALIB_N         100   // Number of readings taken to calibrate the gyro
#define CALIB_DELAY     2     // Millisecond delay between readings

#define GYRO_DEG_S        131.0   // Scale factor of gyroscope
#define COMP_FILTER_GAIN  0.95    // Gain of 1D complementray filter

#define PWM_MAX       255     // Maximum value of PWM

#define K_PITCH_INT   0.1     // Angular restoration Spring, damps K_PITCH_INT 
#define K_PITCH_ANGLE 0.1     // Angular damping, damps K_PITCH
#define K_PITCH_RATE  0.1     // Integral of angle, slightly damps BIAS_RATE.

#define BIAS_GAIN     0.001   // Wheel decceleration correction, damps EXP_RATE
#define BIAS_EXP      0.001   // velocity estimate to retard motion, Damps K_DISP
#define BIAS_DISP     0.001   // displacement estimate to counter static loads

#define K_PROP_YAW    0.1     // Proportional gain about yaw, adds damping
#define K_INT_YAW     0.1     // Adds a spring to the heading

#define DEADBAND_PITCH  4     // Deadband per axis
#define DEADBAND_YAW    4

//-- Global variables

int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;
int16_t GyroX_init  = 0;
int16_t GyroY_init  = 0;
int16_t GyroZ_init  = 0;

float pitch_angle   = 0;
float pitch_int     = 0;
float velocity      = 0;
float displacement  = 0;
float integral_yaw  = 0;

uint32_t last_time  = micros();

//-- Functions

void motorDirection( int input, uint8_t pinA, uint8_t pinB ) {
  if( input > 0 ) {
    analogWrite( pinA, input );
    digitalWrite( pinB, LOW );
  } else {
    analogWrite( pinB, -input );
    digitalWrite( pinA, LOW );
  }
}

void setupGyro(){
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Low pass filter: Gyro and Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                     // write to address 26 of the register
  Wire.write(0x06);                     // options here are 0x00 which is off, and 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
  Wire.endTransmission(true);       // 0x06 being the highest filter setting
}

void readGyro( void ) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();  

  GyroX -= GyroX_init;
  GyroY -= GyroY_init;
  GyroZ -= GyroZ_init;
}

void calibGyro( void ) {  
  float averageX = 0;
  float averageY = 0;
  float averageZ = 0;
  
  for( int index = 0; index < CALIB_N; index += 1 ) {
    readGyro();
    averageX += GyroX;
    averageY += GyroY;
    averageZ += GyroZ;
    delay(CALIB_DELAY);
  }
  GyroX_init = int(averageX/CALIB_N);
  GyroY_init = int(averageY/CALIB_N);
  GyroZ_init = int(averageZ/CALIB_N);
}

float deadband( float input, float band ) {
  if( input >= band ) {
    return input - band;
  } else if ( input <= -band ) {
    return input + band; 
  } else {
    return 0;
  }
}

//------- Main program

void setup() {
  pinMode( MOTOR_A_PIN_1, OUTPUT );
  pinMode( MOTOR_A_PIN_2, OUTPUT );
  pinMode( MOTOR_B_PIN_1, OUTPUT );
  pinMode( MOTOR_B_PIN_2, OUTPUT );

  setupGyro();
  calibGyro();
}

void loop() {
  // update time interval
  float change_time = (micros() - last_time) * 1E-6;
  last_time = micros();

  // Update vehicle pitch angle (horizontal angle) 
  readGyro();

  // complementary filter for gyro.
  float angle_slice = float(GyroX)/GYRO_DEG_S * change_time;  
  float accel_angle = atan2( AccY, AccZ ) * 180.0/PI;  
  
  pitch_angle = (angle_slice + pitch_angle)*COMP_FILTER_GAIN + (1 - COMP_FILTER_GAIN)*accel_angle;

  // PID controller for pitch
  pitch_int += pitch_angle * change_time * K_PITCH_INT;
  pitch_int = constrain(pitch_int, -PWM_MAX, PWM_MAX);
  
  float input_pitch = pitch_angle*K_PITCH_ANGLE + GyroX*K_PITCH_RATE + pitch_int; 

  // Torque function - modify all command with: output = function(time, output) <- Function depends on past values and time; Determined by vehicle behavior  
  input_pitch += displacement;
  
  velocity += (velocity*BIAS_EXP + input_pitch*BIAS_GAIN) * change_time;
  velocity = constrain(velocity, -PWM_MAX, PWM_MAX);

  input_pitch += velocity;
  
  displacement += velocity * BIAS_DISP * change_time;
  displacement = constrain(displacement, -PWM_MAX, PWM_MAX);

  // Stabilization to maintan vehicle heading
  integral_yaw += GyroZ * change_time * K_INT_YAW;
  integral_yaw = constrain( integral_yaw, -PWM_MAX, PWM_MAX ); 
                                  
  float input_yaw = GyroZ*K_PROP_YAW + integral_yaw;

  // Add deadbands
  input_pitch = deadband(input_pitch, DEADBAND_PITCH);
  input_yaw = deadband(input_yaw, DEADBAND_YAW);

  // Set PWM signals to motors with net inputs 
  int inputA = constrain( input_pitch - input_yaw, -PWM_MAX, PWM_MAX );
  int inputB = constrain( input_pitch + input_yaw, -PWM_MAX, PWM_MAX );
  
  motorDirection( inputA , MOTOR_A_PIN_1, MOTOR_A_PIN_2 ); 
  motorDirection( inputB , MOTOR_B_PIN_1, MOTOR_B_PIN_2 );  
}
