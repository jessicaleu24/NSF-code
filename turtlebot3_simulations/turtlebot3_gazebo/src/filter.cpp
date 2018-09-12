/*******************************************************************************
* This script implements the safe set algorithm using the LDS data
*******************************************************************************/

/* Authors: Changliu Liu */

#include "turtlebot3_gazebo/filter.h"
#include <cmath>
#include <cstdio>

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
double gps_state_[4] = {0,0,0,0}; // timestamp, X, Y, Z
double Pre_gps_state_[4] = {0,0,0,0}; // timestamp, X, Y, Z
double filteredGPS_[4] = {0,0,0,0}; // timestamp, X, Y, Z
//marvelmind_nav::hedge_pos_a new_msg;
sensor_msgs::NavSatFix new_msg;
double time_ = 0; 
double Ts = 0; 
double time_new_ =0;
double GPSoffset[2] = {0};
int count_init = 0;
int count_num = 0;
bool initbool = false;


GazeboRosTurtleBot3SSA::GazeboRosTurtleBot3SSA()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 SSA Node Init");
  ROS_ASSERT(init());
  state_offset_[0] = state_[0];
  state_offset_[1] = state_[1];
  state_offset_[2] = state_[2];
  std::cout << "Offset:" << state_offset_[0] << "," << state_offset_[1] <<std::endl; 


}

GazeboRosTurtleBot3SSA::~GazeboRosTurtleBot3SSA()
{
  ssaLog_.close();
  
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool GazeboRosTurtleBot3SSA::init()
{
  // initialize ROS parameter
  nh_.param("is_debug", is_debug_, is_debug_);
  std::string robot_model = nh_.param<std::string>("tb3_model", "");

  if (!robot_model.compare("burger"))
  {
    turning_radius_ = 0.08;
    rotate_angle_ = 50.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
    side_distance_limit_  = 0.4;
  }
  else if (!robot_model.compare("waffle"))
  {
    turning_radius_ = 0.1435;
    rotate_angle_ = 40.0 * DEG2RAD;
    front_distance_limit_ = 0.7;
    side_distance_limit_  = 0.6;
  }
  ROS_INFO("robot_model : %s", robot_model.c_str());
  ROS_INFO("turning_radius_ : %lf", turning_radius_);
  ROS_INFO("front_distance_limit_ = %lf", front_distance_limit_);
  ROS_INFO("side_distance_limit_ = %lf", side_distance_limit_);

  for (int i = 0; i < N_DIREC; i++)
  {
    if (i < (N_DIREC+1)/2)
    {
      scan_angle_[i] = 90 - i * 180 / (N_DIREC-1);
    }
    else
    {
      scan_angle_[i] = 450 - i * 180 / (N_DIREC-1);
    }
    
    if ((scan_angle_[i] < 45) || (scan_angle_[i] > 315))
    {
      distance_limit_[i] = 0.15/std::abs(cos(double(scan_angle_[i]) * DEG2RAD));
    }
    else
    {
      distance_limit_[i] = 0.15/std::abs(sin(double(scan_angle_[i]) * DEG2RAD));
    }
    std::cout << scan_angle_[i] << ";"  << distance_limit_[i] << std::endl; 
  }

  

  // initialize variables
  right_joint_encoder_ = 0.0;
  priv_right_joint_encoder_ = 0.0;
  // initialize publishers
  
  //filteredGPS_pub = nh_.advertise<marvelmind_nav::hedge_pos_a>("filteredGPS", 1000);
  filteredGPS_pub = nh_.advertise<sensor_msgs::NavSatFix>("filteredGPS", 1000);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &GazeboRosTurtleBot3SSA::laserScanMsgCallBack, this);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &GazeboRosTurtleBot3SSA::jointStateMsgCallBack, this);
  odom_sub_ = nh_.subscribe("/odom", 10, &GazeboRosTurtleBot3SSA::odomMsgCallBack, this);
  sub = nh_.subscribe("/hedge_pos_a", 1000, &GazeboRosTurtleBot3SSA::chatterCallback,this);
  

  // initialize log file
  
  

  

  return true;
}

void GazeboRosTurtleBot3SSA::chatterCallback(const marvelmind_nav::hedge_pos_a &msg)
{
  
  double offset_x = 0;
  double offset_y = 0;
  int flag = 0;

  if (count_init >20){

    offset_x = GPSoffset[0];
    offset_y = GPSoffset[1];
    flag = 1;

  }

  // Save past information
  time_ = time_new_;
  time_new_ =ros::Time::now().toSec();
  Ts = time_new_-time_;                         // Time duration between two GPS message
  Pre_gps_state_[0] = filteredGPS_[0];
  Pre_gps_state_[1] = filteredGPS_[1]; 
  Pre_gps_state_[2] = filteredGPS_[2];
  Pre_gps_state_[3] = filteredGPS_[3];


  // Get new information
  gps_state_[0] = msg.timestamp_ms;
  gps_state_[1] = msg.x_m ; 
  gps_state_[2] = msg.y_m ;
  gps_state_[3] = msg.z_m;
  
  // 
  filteredGPS_[1] = gps_state_[1] - offset_x;
  filteredGPS_[2] = gps_state_[2] - offset_y;

  //Kalman filter


  
  // Assign result to publishing message
  /*
  new_msg.address= 0;
  new_msg.timestamp_ms = gps_state_[0];
  new_msg.x_m = filteredGPS_[1];
  new_msg.y_m = filteredGPS_[2];
  new_msg.z_m = filteredGPS_[3];
  new_msg.flags = flag;                        // '0' not yet initailized. '1' initialized.
  */
  new_msg.latitude = filteredGPS_[2];          // y-axis
  new_msg.longitude = filteredGPS_[1];         // x-axis
  new_msg.altitude = gps_state_[3];            // z-axis

  count_init = count_init +1;
  


 
  
  //ROS_INFO("I heard: [%f] and [%f] to [%f] , [%f]", gps_state_[1],gps_state_[2], filteredGPS_[1] ,filteredGPS_[2]);
}

void GazeboRosTurtleBot3SSA::jointStateMsgCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{
  right_joint_encoder_ = msg->position.at(0);
  left_joint_encoder_ = msg->position.at(1);
}

void GazeboRosTurtleBot3SSA::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  for (int num = 0; num < N_DIREC; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle_[num])))
    {
      direction_vector_[num] = 20;//msg->range_max;
    }
    else
    {
      direction_vector_[num] = msg->ranges.at(scan_angle_[num]);
    }
    if (direction_vector_[num] == 0)
    {
      direction_vector_[num] = 20;
    }
  }
}


void GazeboRosTurtleBot3SSA::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  state_[0] = msg->pose.pose.position.x;
  state_[1] = msg->pose.pose.position.y;
  state_[2] = std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  state_[3] = msg->twist.twist.linear.x;
  state_[4] = msg->twist.twist.angular.z;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool GazeboRosTurtleBot3SSA::controlLoop()
{
  
  // Get gps initial position offset
  if (0<count_init && count_init < 20) {

    GPSoffset[0] = GPSoffset[0] + gps_state_[1];
    GPSoffset[1] = GPSoffset[1] + gps_state_[2];
    //std::cout << "GPSstate:" << gps_state_[1] << "," << gps_state_[2]<<std::endl;
    //std::cout << count_init << std::endl;
    ++ count_num;

    
  } 
  if (initbool == false && count_init > 20){

    GPSoffset[0] = GPSoffset[0]/count_num;
    GPSoffset[1] = GPSoffset[1]/count_num;
    initbool = true;
    std::cout << "GPSoffset get!!" <<std::endl;
    std::cout << "GPSoffset:" << GPSoffset[0] << "," << GPSoffset[1]<<std::endl;

  }
    
  
  //std::cout << count_init << std::endl;
  //std::cout << "GPSoffset:" << GPSoffset[0] << "," << GPSoffset[1]<<std::endl;

  double wheel_radius = 0.033;
  double turtlebot3_rotation = 0.0;

  turtlebot3_rotation = (rotate_angle_ * turning_radius_ / wheel_radius);

  // Critical direction
  double min_d = 10;
  uint8_t critical_index = -1;
  for (int i=0; i<N_DIREC; i++)
  {
    double flag = 1;
    //std::cout << direction_vector_[i] << ",";
    if ((scan_angle_[i] < 70 && scan_angle_[i] > 40) || (scan_angle_[i] < 320 && scan_angle_[i] > 290))
    {
      if (0 > direction_vector_[i] - distance_limit_[i])
      {
        flag = 0;
      }
    }
    if ((min_d > direction_vector_[i] - distance_limit_[i]) && flag)
    {
      min_d = direction_vector_[i] - distance_limit_[i];
      critical_index = i;
    }
  }
  //std::cout << min_d << std::endl;

  //compute ref_cmd_
  
  //compute cmd_
  
  
  filteredGPS_pub.publish(new_msg);

  // log
  
  
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  //ros::init(argc, argv, "gazebo_ros_turtlebot3_ssa");
  ros::init(argc, argv, "filter");
  GazeboRosTurtleBot3SSA gazeboRosTurtleBot3SSA;

  ros::Rate loop_rate(SAMPLE_TIME);

  while (ros::ok())
  {
    gazeboRosTurtleBot3SSA.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  return 0;
}

