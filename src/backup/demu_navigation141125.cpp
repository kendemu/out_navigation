#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const int wp_total_num = 6;
double wp[wp_total_num][3] = { 
  // x, y, theta
  { 5.0, 0.0,  90}, { 5.0, 20.0, 0}, {12.0, 20.0, 270},
  {12.0, 0.0,  0}, {25.0,  0.0, 270}, {25.0, -4.0, 270}} 
;


int main(int argc, char** argv){
  ros::init(argc, argv, "demu_navigation");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  int wp_num =0;
  ROS_INFO("Start");
  while (wp_num < wp_total_num) {
    ROS_INFO("%d loops",wp_num); 
    goal.target_pose.pose.position.x = wp[wp_num][0];
    goal.target_pose.pose.position.y = wp[wp_num][1];
    double target_dir = wp[wp_num][2];
    double radians    = target_dir * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);
    
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    goal.target_pose.pose.orientation = qMsg;
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Arrived Waypoint No.%d",wp_num+1);
    }
    else {
      ROS_INFO("Failed: The base failed to navigation");
    }
    wp_num++;
    ROS_INFO("Go to  Waypoint No.%d",wp_num+1);
  }

  return 0;
}
