/*******************************************************************************
* This script implements the safe set algorithm using the LDS data
*******************************************************************************/

/* Authors: Changliu Liu */

#include "turtlebot3_gazebo/gazebo_ros_turtlebot3_ssa.h"
#include <cmath>
#include <cstdio>

#include <iostream>
#include <fstream>

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
  updatecommandVelocity(0.0, 0.0);
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
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("/scan", 10, &GazeboRosTurtleBot3SSA::laserScanMsgCallBack, this);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &GazeboRosTurtleBot3SSA::jointStateMsgCallBack, this);
  odom_sub_ = nh_.subscribe("/odom", 10, &GazeboRosTurtleBot3SSA::odomMsgCallBack, this);

  // initialize log file
  
  ssaLog_.open ("ssaLog.txt");
  ssaLog_ << "x,y,theta,v,w,cmd_v,cmd_w,min_d,min_theta,rel_v;\n";

  return true;
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

void GazeboRosTurtleBot3SSA::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  //std::cout << "vel: " << linear << "," << angular << std::endl;

  cmd_vel_pub_.publish(cmd_vel);
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
  double target_angle = std::atan2(state_offset_[1]-state_[1], state_offset_[0]-state_[0]);
  if (std::abs(target_angle - state_[3]) > 5)
  {
    ref_cmd_[0] = 0;
    if (target_angle > state_[3])
    {
      ref_cmd_[1] = 1.5;
    }
    else
    {
      ref_cmd_[1] = -1.5;
    }
  }
  else
  {
    ref_cmd_[0] = 0.2;
    ref_cmd_[1] = 0;
  }

  //compute cmd_
  if ((critical_index > -1) && (min_d*min_d - state_[3]*cos(scan_angle_[critical_index] * DEG2RAD) < margin_*margin_))
  {
    //ROS_INFO("Danger");
    //std::cout << "Min Distance:" << min_d << "; Angle: " << scan_angle_[critical_index] << "; Relative Vel: " << state_[3]*cos(scan_angle_[critical_index] * DEG2RAD) << std::endl;
    double L[2];
    double S;
    L[0] = cos(scan_angle_[critical_index] * DEG2RAD);
    L[1] = -state_[3] * sin(scan_angle_[critical_index] * DEG2RAD);
    S = -0.1 -2 * min_d * state_[3] * cos(scan_angle_[critical_index] * DEG2RAD);

    if (S - L[0]*(ref_cmd_[0] - state_[3])* SAMPLE_TIME/1000 - L[1]*ref_cmd_[1] >= 0)
    {
      cmd_[0] = ref_cmd_[0];
      cmd_[1] = ref_cmd_[1];
    }
    else
    {
      double error = S - L[0]*(ref_cmd_[0] - state_[3])* SAMPLE_TIME/1000 - L[1]*ref_cmd_[1];
      double factor = error / (L[0]*L[0] + L[1]*L[1]);
      cmd_[0] = state_[3] + factor*L[0] * SAMPLE_TIME/1000;
      cmd_[1] = ref_cmd_[1] - factor*L[1] / DEG2RAD; 
      if (cmd_[0] > 2)
      {
        cmd_[0] = 2;
      } 
      if (cmd_[0] < 0)
      {
        cmd_[0] = 0;
        if (critical_index > N_DIREC/2)
        {
          cmd_[1] = 1.5;
        }
        else
        {
          cmd_[1] = -1.5;
        }
      } 
      if (std::abs(cmd_[1]) > 1.5)
      {
        cmd_[1] = cmd_[1] / std::abs(cmd_[1]) * 1.5;
      }
      //updatecommandVelocity(cmd[0], cmd[1]);
      //std::cout << "factor: " << factor << std::endl;
      //std::cout << "cmd: " << cmd[0] << "," << cmd[1] << std::endl;
    }
  }
  else
  {
    cmd_[0] = ref_cmd_[0];
    cmd_[1] = ref_cmd_[1];
    //ROS_INFO("Reference");
  }
  updatecommandVelocity(cmd_[0], cmd_[1]);

  // log
  ssaLog_ << state_[0];
  for (int i = 1; i<5; i++)
  {
    ssaLog_ << "," << state_[i];
  }
  for (int i = 0; i<2; i++)
  {
    ssaLog_ << "," << cmd_[i];
  }
  ssaLog_ << "," << min_d << "," <<  scan_angle_[critical_index] << "," << state_[3]*cos(scan_angle_[critical_index] * DEG2RAD) << ";" << std::endl;
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gazebo_ros_turtlebot3_ssa");
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

