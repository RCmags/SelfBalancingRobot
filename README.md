# SelfBalancingRobot
This is an arduino sketch for a [self balancing robot](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html). 
This nothing more than a two wheeled platform that is unstable in one direction and must be stabilized to maintain balance.
The code provides the nessesary stabilization through a single MPU6050 gyroscope and does not use of rotary encoders.
Doin so allows the robot to be use inexpensive DC motors controlled through an H-bridge. 

The code works as follows: a complementrary filter is used to estimate the (pitch) angle of the vehicle relative to the horizontal. 
A PID controller is driven by this angle and attemps to drive towards zero. 
The output of the PID controller is then fed into a "torque function" that modifies the command. 
This function attemps to counter friction in the motors 
to ensure constant inputs generate constant torque on the vehicle.   

To accomplish this, the function integrates the control inputs at a specified rate. 
This causes the motors to stay on even after a control input is zero.
As the control inputs are analogous to the acceleration felt by the vehicle, 
this integral acts as an estimate for the velocity.
By the same analogy, integrating this term provides an estimate for the position. 
One can use the position and velocity estimates to laterally stabilize the vehicle. 
While this is not as precise as rotary encoders it is a simple and effective workaround.  

This allows the center of gravity of the vehicle to change in real time and the vehicle will bank itself to maintain equilibrium. 
In essence, one has two PID loops acting simulatenously. 
One that stabilizes the heading of the vehicle, and one that attempts to keep the vehicle form sliding back and forth.

Here is a schematic of the required circuit:
<img src="https://raw.githubusercontent.com/RCmags/selfBalancingRobot/main/self_balacing_robot_schem.png" width = "75%"></img>

Finally, here are images of the of platform this is intended for:  
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/side_view_res.jpg" width = "20%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/bottom_view_res.jpg" width = "20%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/top_view_res.jpg" width = "20%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/balance_motion.gif" width = "20%"></img>
