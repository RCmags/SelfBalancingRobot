# Controller for self-balancing robot
This is an arduino sketch for a [self balancing robot](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html). Essentally, its an inverted pendulum with two wheels. Due to this instability it must be stabilized to maintain balance

## Operation
This sketch provides the stabilization through an MPU6050 IMU, and a set of inexpensive DC brushed motors controlled through an H-bridge. 

There are three PID loops acting at the same time: One that stabilizes the heading of the vehicle, and one that attempts to keep the vehicle from sliding back and forth. This allows the center of gravity to change in real time and the vehicle will bank itself to maintain equilibrium. The last PID controller maintains heading.

- __Pitch stabilization:__ It is the dominant PID controller of the system. It uses the pitch angle (relative to gravity) of the vehicle as a proportional term. This loop allows the vehicle to stay upright by accelerating into a fall. 

- __Position stabilization:__ This loop uses an estimate for _velocity_ as the proportional term. We estimate the velocity by integrating pitch angle, which is proportional to the _lateral acceleration_ at small angles. We integrate again to approximate the _position_ (double integral of the pitch angle), which is used as the integral term of the PID controller. Both terms cause the vehicle to drift laterally at a constant speed.  

- __Yaw stabilization:__ A controller whose proportional term is the yaw-axis angular velocity. It maintains the heading of the vehicle despite the constant corrections. It counters the open-loop nature of the DC motor control. 

__Note:__ To counter magnetic cogging and stiction at low RPMs, the motors are driven using _pulse frequency modulation_. The duty is cycle is long enough so that each pulse causes the motors to rotate one magnetic pole. This means the motors are driven like a single-phase stepper motor. Despite it being jerky at low speeds, such a waveform provides much more precise control over low-speed rotation. Such precision is necessary to reduce the oscillation of the vehicle when its nearly vertical.  

## Schematic   
<img src="/diagram/self-balance.png"></img>
