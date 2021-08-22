# SelfBalancingRobot
This is a simple arduino sketch for a self balancing robot. 
This is a two wheeled platform that is unstable in one direction and must be stabilized to maintain balance.
The code provides the nessesary stabilization through a single MPU6050 gyroscope and does not make use of rotary encoders on the wheels.
This allows the robot to be driven by inexpensive DC motors controlled by an H-bridge. 

The code works as follows: a complementrary filter is used to estimate the (pitch) angle of the vehicle relative to the horizontal. 
A PID controller is built around this angle and attemps to keep it near zero. 
The output of the PID controller is then fed into a "torque function". 
This is nothing more than a function that attemps to counter friction in the motors 
to ensure that a constant control input generates a constant torque on the vehicle.   

To accomplish this, it integrates the control inputs at a specified rate. 
This causes the motors to stay on even after a control input is zero.
As the control inputs are analogous to the torque and hence the acceleration felt by the vehicle, 
this integral acts as an estimate for the velocity.
By the same analogy integrating this term provides an estimate for the position of the vehicle. 
One can then use the position and velocity estimates to laterally stabilize the vehicle
and makes rotary encoders unnessesary.  
While this is not as precise as rotary encoders, it is a simple and effective workaround.  

In essence, one has two PID loops acting simulatenously. 
One that stabilizes the heading of the vehicle, and one that attempts to keep the vehicle form sliding back and forth.
This allows the center of gravity of the vehicle to change in real time and the vehicle will bank itself to maintain equilibrium.  

Here is a schematic of the required circuit:
<img src="https://raw.githubusercontent.com/RCmags/selfBalancingRobot/main/self_balacing_robot_schem.png" width = "75%"></img>

Finally, here are images of the kind of platform this code is intended for:  
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/side_view_res.jpg" width = "25%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/balance_motion.gif" width = "25%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/bottom_view_res.jpg" width = "25%"></img>
<img src="https://raw.githubusercontent.com/RCmags/SelfBalancingRobot/main/images/top_view_res.jpg" width = "25%"></img>
