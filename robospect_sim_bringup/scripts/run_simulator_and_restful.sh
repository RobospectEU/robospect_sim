#!/bin/sh

roslaunch robospect_sim_bringup robospect_complete.launch &

sleep 15

roslaunch robospect_sim_bringup mapping_default.launch &

sleep 2

roslaunch robospect_platform_mission_manager mission_manager.launch &

sleep 2

roslaunch robospect_sim_bringup rostful_server.launch &

sleep 2

google-chrome http://localhost:8080/mission_state
