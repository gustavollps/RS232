#include <ros/ros.h>

#include "../include/heading/EKF_IMU.h"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "heading_ekf");  
  EKF_IMU ekf_node(new ros::NodeHandle);
  ros::spin();
  ROS_INFO("Hello world!");
}
