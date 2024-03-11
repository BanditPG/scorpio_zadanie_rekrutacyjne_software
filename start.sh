#!/bin/bash
source /opt/ros/noetic/setup.bash
source /opt/ros/catkin_ws/devel/setup.bash
roscore & # Start roscore in the background
sleep 5 # Give roscore time to start
roslaunch virtual_dc_motor virtual_dc_motor.launch # Replace with your launch file