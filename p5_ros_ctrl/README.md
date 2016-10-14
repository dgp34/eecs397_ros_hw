# p5_ros_ctrl

This package addresses P5:  Add an additional link and joint to the robot description of P4 (three DOF in total) and modify the package to use ROS controller plugins for Gazebo.  Afterwards, create a commander that sends unique sinusoidal position commands to each joint.

## How to Run

Use the launch file to start the ROS controllers, the commander, and Gazebo with the robot model.  Enjoy the resulting motion.
In the event a different motion is desired, navigate through the package and find sin_commander.cpp.  In this file, edit the values of the defined FREQ1, FREQ2, and FREQ3.  Save, remake, and relaunch.
