#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <ros/ros.h>

// Ros messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// Custom msgs and srvs
#include <tcc_msgs/pose2D.h>
#include <tcc_msgs/changePID.h>
#include <tcc_msgs/ResetOdom.h>
#include <tcc_msgs/interrupt_counter.h>
#include <tcc_msgs/movement_msg.h>
#include <tcc_msgs/ZeroAngle.h>

// Personal Library
#include <control_loop/PID.h>

// System tools
#include <boost/thread/thread.hpp>
#include <math.h>

// Ros external libraries
#include <tf/transform_datatypes.h>

#define PI 3.14159265

//#define PULSE_PER_METER 344.0000*(1.0/(0.07*PI))*4000 //344.0* 7 * PI //344
// pulses per minute (encoder) / 7cm diameter * PI
#define CONTROL_FREQ 50.0000
//#define PPM_CONSTANT CONTROL_FREQ/PULSE_PER_METER

class movementController
{
public:
  movementController(ros::NodeHandle* nh_);

  struct Velocity
  {
    float Vx;
    float Vy;
  };

  struct intCounter
  {
    long int m1_counter;
    long int m2_counter;
    long int m3_counter;
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

  struct position
  {
    float x;
    float y;
  };

  float constrain(float value, float max, float min);

private:
  ros::NodeHandle* nh_;

  position pose_, last_pose_, d_pose_;
  intCounter new_int_, old_int_;
  intDeriv m_spds_;
  Twist velocity_;
  Velocity setpoint_velocity_;
  Velocity velocity_msg_;

  PID* PID_rotation_;

  tcc_msgs::pose2D odom_msg_;
  nav_msgs::Odometry odom_msg_ekf_;
  geometry_msgs::Twist cmd_vel_msg_;  
  tf::Quaternion quaternion_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher ekf_pub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber int_sub_;
  ros::Subscriber remote_control_sub_;
  ros::Subscriber movement_sub_;
  ros::Subscriber point_sub_;
  ros::Timer control_timer_;
  ros::Timer odom_ekf_timer_;
  ros::ServiceServer pid_change_server_;
  ros::ServiceServer reset_odom_server_;
  ros::ServiceClient zeroAngle_client_;

  void headingCallBack(sensor_msgs::Imu msg);

  void intCallBack(tcc_msgs::interrupt_counter msg);
  void remotecontrolCallBack(geometry_msgs::Twist msg);
  void movementCallBack(tcc_msgs::movement_msg msg);

  bool changepidCallBack(tcc_msgs::changePID::Request& req,
                         tcc_msgs::changePID::Response& res);

  bool resetodomCallBack(tcc_msgs::ResetOdom::Request& req,
                         tcc_msgs::ResetOdom::Response& res);

  void odomekfCallBack(const ros::TimerEvent& event);
  void controltimerCallBack(const ros::TimerEvent& event);

  void getParams();

  double PULSE_PER_METER_;
  float Kx_;
  float Ky_;

  float angle_, last_angle_;
  float angle_setpoint_;

  float accel_ramp_;
  float decel_ramp_;

  float radius_;

  float freq_;
  float Kp_;
  float Ki_;
  float Kd_;
  float pid_min_;
  float pid_max_;
  float dead_zone_;

  bool cmd_vel_;
  bool int_start_;
  int cmd_vel_counter_;
};

#endif // MOVEMENTCONTROLLER_H
