#!/bin/sh
source devel/setup.bash
xterm -e "source devel/setup.bash & roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash & roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "source devel/setup.bash & roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5 
xterm -e "source devel/setup.bash & rosrun add_markers home_services" &
sleep 5
xterm -e "source devel/setup.bash & rosrun pick_objects pick_objects"