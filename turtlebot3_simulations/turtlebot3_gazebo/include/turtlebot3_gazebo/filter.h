/*******************************************************************************
* This is the header file for gazebo_ros_turtlebot3_ssa
*******************************************************************************/

/* Authors: Changliu Liu */

#ifndef GAZEBO_ROS_TURTLEBOT3_SSA_H_
#define GAZEBO_ROS_TURTLEBOT3_SSA_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "turtlebot3_msgs/goal.h"


#include <iostream>
#include <fstream>


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos_a.h"

#include <sstream>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.2
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

#define N_DIREC 61

#define SAMPLE_TIME 125 //ms


class GazeboRosTurtleBot3SSA //: GazeboRosTurtleBot3
{
 public:
  GazeboRosTurtleBot3SSA();
  ~GazeboRosTurtleBot3SSA();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  bool is_debug_;

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher filteredGPS_pub;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber sub;



  std::ofstream ssaLog_;

  double turning_radius_;
  double rotate_angle_;
  double front_distance_limit_;
  double side_distance_limit_;

  double direction_vector_[N_DIREC];
  uint16_t scan_angle_[N_DIREC];
  double distance_limit_[N_DIREC];
  double margin_ = 0.2;

  double right_joint_encoder_;
  double priv_right_joint_encoder_;
  double left_joint_encoder_;
  double priv_left_joint_encoder_;

  double ref_cmd_[2] = {LINEAR_VELOCITY,0};
  double cmd_[2] = {0,0};

  double state_[5] = {0,0,0,0,0}; // x,y,theta,v,omega
  double state_offset_[3] = {0,0,0}; // x,y,theta

  int64_t goal_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void jointStateMsgCallBack(const sensor_msgs::JointState::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void goalCallBack(const turtlebot3_msgs::goal &msg);
  void chatterCallback(const marvelmind_nav::hedge_pos_a &msg);

};
#endif // GAZEBO_ROS_TURTLEBOT3_SSA_H_