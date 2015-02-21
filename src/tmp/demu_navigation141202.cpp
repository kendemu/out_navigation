#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// subscribeした自己位置情報
geometry_msgs::PoseStamped robotPose; //ロボットの自己位置 

const double DIST_LIMIT = 1.0;

// Waypoint 
// 角度777が終了の合図
const int wp_total_num = 6;
const double ox  = 0.0;   // offset_x
const double oy  = -15.0; // offset_y
double wp[wp_total_num+1][3] = { 
  // x, y, theta
  {ox+4.2,  oy+0.0, 90}, {ox+4.5, oy+20.5,   0}, {ox+12.0, oy+20.5, 270},
  {ox+12.0, oy+0.0,  0}, {ox+25.0, oy+0.0, 270}, {ox+25.0, oy-4.0, 270}, 
  {ox+0.0, oy+0.0, 777}}
;

// Callback function
// Get the present robot position 
void amcl_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  robotPose.header = msg->header;
  robotPose.pose   = msg->pose.pose;
  ROS_INFO("robotPose.pose.position.x = %.2f y=%.2f",
	   robotPose.pose.position.x,robotPose.pose.position.y);
}


int main(int argc, char** argv){
  // initialize ROS
  ros::init(argc, argv, "demu_navigation");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Set the subscriber
  ros::Subscriber amcl_pose_sub_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1, amcl_poseCB);
  
  // Valid the callback
  ros::spinOnce();

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot 
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  int wp_num =0;
  ROS_INFO("Start");
  while (wp[wp_num][2] !=777) {
    goal.target_pose.pose.position.x = wp[wp_num][0];
    goal.target_pose.pose.position.y = wp[wp_num][1];
    double target_dir = wp[wp_num][2];
    double radians    = target_dir * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);
    
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    goal.target_pose.pose.orientation = qMsg;
    
    ROS_INFO("Sending the next waypoint No.%d",wp_num+1);
    ac.sendGoal(goal);
    //ros::spinOnce();
    //ac.waitForResult();

    double dist = 10000.0;
    ros::Rate rate(10); // 10hz

    while (dist > DIST_LIMIT*DIST_LIMIT) {
      ros::spinOnce();
      rate.sleep();
      double x = robotPose.pose.position.x;
      double y = robotPose.pose.position.y;
      //ROS_INFO("WP(%.1f,%.1f) Pos(%.1f,%.1f)\n",
      //	       wp[wp_num][0],wp[wp_num][1],x,y);
      dist = (wp[wp_num][0] - x) * (wp[wp_num][0] - x)
	   + (wp[wp_num][1] - y) * (wp[wp_num][1] - y);
      double diff = dist - DIST_LIMIT*DIST_LIMIT;
      if (diff > 0) ROS_INFO("Last distance[wp:%d]: %f [m]\n",
      			     wp_num+1,sqrt(diff));
      else          ROS_INFO("Last distance[wp:%d]: %f [m]\n",
      			     wp_num+1,sqrt(-diff));
    }
    ROS_INFO("Arrived at waypoint No.%d",wp_num+1);
    wp_num++;
    ROS_INFO("Go to the next waypoint No.%d",wp_num+1);
    
    //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //ROS_INFO("Arrived Waypoint No.%d",wp_num+1);
    //}
    //else {
    //ROS_INFO("Failed: The base failed to navigation");
    // }
  }
  ROS_INFO("GoaoooooooooooooooooL!\n");

  return 0;
}
