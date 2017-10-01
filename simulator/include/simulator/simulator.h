#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <ros/ros.h>

class simulator
{
public:
  simulator();

private:

  ros::Publisher odom_ekf_pub_;
  ros::Publisher odom_pub_;
  ros::Subscriber move_sub_;

};

#endif // SIMULATOR_H
