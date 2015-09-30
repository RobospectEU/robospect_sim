# robospect_sim
robospect Simulation Packages

The robospect_sim is composed by the following packages:

## robospect_control

This package contains all the configuration files needed simulate the motor controllers in Gazebo (using skid_steering plugin).

## robospect_gazebo

It contains the launch and config files to launch Gazebo with the robot.

## robospect_robot_control

It’s the robot’s Gazebo plug-in controller. It implements the control of the ackerman kinematics of the robot, controlling the traction and steering motors. This component publishes the robot’s odometry.

## robospect_sim_bringup

It contains several launch files in order to launch some or all the components of the robot.
