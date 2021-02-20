# Goals Achieved

->In this code the Quadrotor is controlled with a nonlinear Backstepping based on Quaternions for attitude parametrization. 

-> A new DoubleSpinBox has been added in "myTab" in which the new yaw angle can be added to change the attitude while the position control is active.

# Problems detected
-> The yaw angle must be in 
$\psi \in (-40,40)$
 (degrees)

-> The Quadrotor attitude is obtained by mixing up the data from the Optitrack and IMU, perhaps if the attitude is obtained only from optitrack the rotation could be the entire circle. 


