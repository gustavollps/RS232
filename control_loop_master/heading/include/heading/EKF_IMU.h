#ifndef EKF_IMU_H
#define EKF_IMU_H
#include "../../include/heading/mpu.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tcc_msgs/CalibrateInt.h"
#include "tcc_msgs/ZeroAngle.h"
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>

#define PI 3.14159265
#define N_SAMPLES 2000
//#define CALIBRATION

class EKF_IMU
{
public:
  EKF_IMU(ros::NodeHandle* nh);

private:
  MPU mpu_;
  ros::NodeHandle* nh_;
  ros::Publisher imu_pub_;
  ros::Publisher battery_level_;
  ros::Publisher input_level_;
  ros::Timer gyro_timer_;
  ros::Timer levels_timer_;
  ros::ServiceServer zeroAngle_server_;


  void timerCallBack(const ros::TimerEvent& event);
  void levels_timerCallBack(const ros::TimerEvent &event);
  bool zeroangleCallBack(tcc_msgs::ZeroAngle::Request& req,
                         tcc_msgs::ZeroAngle::Response& res);

  tf::Quaternion quaternion_;
  sensor_msgs::Imu imu_data_;

  double accel_offset_x;
  double accel_offset_y;
  double accel_offset_z;

  double gyro_offset_;
  double gyro_angle_;

  bool publish_now_;
};

#endif // EKF_IMU_H
