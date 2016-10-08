# p4_sin_two_dof

This package addresses P4:  Add an additional link and joint to the provided minimal robot description and modify the controller to address this change.  Afterwards, create a commander that sends unique sinusoidal position commands to each joint.

## How to Run

Use the launch file to start the controller, the commander, and Gazebo with the robot model.  Enjoy the resulting motion.
In the event a different motion is desired, navigate through the package and find sin_commander.cpp.  In this file, edit the values of the defined FREQ1 and FREQ2.  Save, remake, and relaunch.
