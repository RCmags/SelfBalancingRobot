# Controller for self-balancing robot
This is an arduino sketch for an inverted pendulum with two wheels (a [self balancing robot](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html)). Due to the unstable nature of this platform, it must be actively stabilized to maintain balance.

## Operation
The program provides stabilization through an MPU6050 IMU and two brushed DC motors driven by an H-bridge. It uses three PID loops acting simultaneously:

- __Pitch stabilization:__ This is the dominant PID controller of the system. It uses the pitch angle of the vehicle as a proportional term to keep the vehicle upright by accelerating into gravity. 

- __Position stabilization:__ It estimates the velocity by integrating pitch angle, which is proportional to the _lateral acceleration_ (at small angles), and this is integrated again to approximate _position_ (the double integral of the pitch angle). Both are used as the derivative and proportional term respectively. Their combined effect prevent the vehicle from drifting laterally and counters any change in center of gravity.  

$$ a_x \propto \theta $$

$$ v_x = \int{a_x}dt \propto \int{ \theta dt } $$

$$ x = \int{v_x}dt \propto \int{ \int{ \theta dt^2 } } $$

- __Yaw stabilization:__ A controller that maintains heading by using as a proportional term the angular velocity of the yaw axis. It counters the open-loop nature of the motor control, wherein one motor can spin faster and cause the vehicle to turn.

Ultimately, the unstable axis is only stabilized by the measured pitch angle. 

__Note:__ Both motors are driven with _pulse frequency modulation_ to counter magnetic cogging and stiction. The duty is cycle is long enough so that each pulse causes the motors to rotate one magnetic pole. Such a waveform provides precise control over low-speed rotation at the cost of jerky movement. This precision is necessary to reduce under and over correction when the vehicle is nearly vertical. 

Driving the motors like this causes them to behave similar to a stepper motor, albeit with much less precision, larger steps, and less torque.  

## Schematic   
<img width="80%" src="/diagram/self-balance.png">
