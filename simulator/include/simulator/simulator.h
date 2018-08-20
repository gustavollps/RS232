#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tcc_msgs/cmd_vel_msg.h>
#include <tcc_msgs/interrupt_counter.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>

#define PI 3.14159265

class Simulator
{
public:
  Simulator(ros::NodeHandle *nh);

private:

  struct Velocity
  {
    float Vx;
    float Vy;
  };

  struct intDeriv
  {
    int m1;
    int m2;
    int m3;
  };

  struct Twist
  {
    float y;
    float x;
    float w;
  };

  Twist velocity_;

  intDeriv m_speeds_[50];

  void pwmCallback(tcc_msgs::cmd_vel_msg msg);
  void headingCallBack(sensor_msgs::Imu msg);
  void odomCallback(nav_msgs::Odometry msg);

  void timerCallback(const ros::TimerEvent &event);

  ros::NodeHandle *nh_;
  ros::Publisher odom_ekf_pub_;
  ros::Publisher int_pub_;
  ros::Publisher heading_pub_;

  ros::Timer int_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber pwm_sub_;

  tcc_msgs::interrupt_counter int_msg_;

  sensor_msgs::Imu imu_data_;

  float PULSE_PER_METER_;
  float motor_speed_sim;
  float heading_;

};

#endif // SIMULATOR_H
