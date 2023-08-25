# RevynArm
This repository houses the source code for controlling my Revyn Arm robot.
It is a 6DOF arm that is small enough to sit on a desk and can be made for
an affordable price. I am trying to see how good a complex robot arm I can
make with inexpensive materials. I am constantly making updates and changing
things to see how much better I can make it. Check out the robot here at my website.
https://brevinbanks.github.io/

In this repository I demonstrate some very basic robotics and controls principals.
My code takes us through the process of finding and calculating the forward kinematics
using DH parameters. Then we find the jacobian for the arm. I also have found the 
geometric inverse kinematics for the robot arm.

I also impliment some nice features that help in control. I put checks in the 
inverse kinematics scripts so we are always returned an elbow up solution. I also
have convienient debugging plotters in the scripts that can be turned on or off.

I also developed a linear trajectory path planner and executer. The robot end
effector will follow a straight line path while maintaining the orientation of 
the end effector, assuming the orrientation can be achieved by the robot at that position.


# The core scripts of interest are listed here:

FK_Revyn: Calculates desired frame for any link of the robot given 6 joint angles

IK_Revyn: Calculates a possible set of 6 joint angles for a desired end effector frame within reach of the robot

Jac_Revyn: Calculates the spacial jacobian of the end effector relative to the base of the robot for a given set of joint angles

Revyn_Controller: Controls the robot with an Arduino and moves it in a rectangular pattern

Revyn_Controller_FK_Proving: Controls the robot with an Arduino and moves it to several spots defined by FK and IK

RevynMat2ArdController: Arduino script for interpreting joint angles sent over serial communication and moves the robot accordingly


Stay tuned for more updates.
