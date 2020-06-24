#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float pickUp[3] = {-1.0, 1.4, 0.0};
float dropOff[3] = {-2.6, -0.4, 0.0};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pickUp[0];
  goal.target_pose.pose.position.y = pickUp[1];
  goal.target_pose.pose.orientation.w = pickUp[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Reached the pickup loaction");
    
    ros::Duration(5.0).sleep();

    goal.target_pose.pose.position.x = dropOff[0];
    goal.target_pose.pose.position.y = dropOff[1];
    goal.target_pose.pose.orientation.w = dropOff[2];
    ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
  }
  else {
    ROS_INFO("Failed - Reached the pickup loaction");
  }
  

  ac.waitForResult();


  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Reached the drop loaction");
  }
  else {
    ROS_INFO("Failed - Reached the drop loaction");
  }

  ros::Duration(5.0).sleep();
  return 0;
}
