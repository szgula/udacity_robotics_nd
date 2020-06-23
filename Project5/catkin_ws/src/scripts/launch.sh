#!/bin/sh
source devel/setup.bash
xterm  -e  "source devel/setup.bash &  gazebo " &
sleep 5
xterm  -e  "source devel/setup.bash &  source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  "source devel/setup.bash &  rosrun rviz rviz" 