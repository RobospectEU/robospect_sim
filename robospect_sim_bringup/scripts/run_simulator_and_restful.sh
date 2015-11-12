#!/bin/sh

roslaunch robospect_sim_bringup robospect_complete.launch &

sleep 15

roslaunch robospect_sim_bringup rostful_server.launch &

sleep 2

google-chrome http://localhost:8080/robospect_platform_controller/odom
