### Overview

In this project the goal was to use create a robot that can navigate in a virtual environment with guardrails. The task of the robot is to navigate to:
  1) go to initial goal position, 
  2) pick up the object,
  3) drop it off,
  4) go to goal position.
  
It had to do that while navigating the environment on it own. The robot is able to perform Simultaneous Localisation and Mapping or SLAM.

### Setup

  1) clone project repository
  2) go to /repository_path/Project5/catkin_ws/src
  3) clone dependencies:
  

    git clone https://github.com/ros-perception/slam_gmapping.git  
    git clone https://github.com/turtlebot/turtlebot.git  
    git clone https://github.com/turtlebot/turtlebot_interactions.git  
    git clone https://github.com/turtlebot/turtlebot_simulator.git  

  4) go to /repository_path/Project5/catkin_ws
  5) catkin_make & source devel/setup.bash
  
  ### Run the project
  
 The project contains multiple shell scripts to launch different scope of project's functionalities - select one: 
  1) home_service.sh - (MAIN) script to launch home service robot demo
  2) pick_objects.sh - script to send multiple goals  
  3) test_navigation.sh - script to test localization and navigation
  4) test_slam.sh - script to test SLAM
  5) launch.sh - script to test shell script working
  6) add_marker.sh - script to model virtual objects 
  
 Launch selected shell script
  1) go to /repository_path/Project5/catkin_ws/
  2) . src/scripts/selected_script.sh
  
