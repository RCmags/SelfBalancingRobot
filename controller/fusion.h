#include <basicMPU6050.h>     
#include <imuFilter.h>

// Gyro settings:
#define         LP_FILTER   6           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   2           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET =  552;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -241;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -3185;

//-- Set the template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;
   
// =========== Settings ===========
imuFilter fusion;

#define GAIN          0.5     /* Fusion gain, value between 0 and 1 */
#define SD_ACCEL      0.2     /* Standard deviation of acceleration. */                          
                                 
