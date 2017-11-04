#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64MultiArray.h"


ros::Publisher cam_republisher;

void camcallBack(sensor_msgs::CameraInfo msg){
  sensor_msgs::CameraInfo msg_new;

  msg_new = msg;
  msg_new.D.clear();

  std::vector<double> vec;
  vec.resize(5);
  vec.at(0) = 0.277446;
  vec.at(1) = -0.756147;
  vec.at(2) = 0.007242;
  vec.at(3) = -0.002223;
  vec.at(4) = 0.000000;
  msg_new.D = vec;

  msg_new.distortion_model = "narrow_stereo";

  msg_new.height = 480;

  msg_new.width = 640;

  msg_new.K.at(0) = 868.235558;
  msg_new.K.at(1) = 0.000000;
  msg_new.K.at(2) = 321.212317;
  msg_new.K.at(3) = 0.000000;
  msg_new.K.at(4) = 868.287863;
  msg_new.K.at(5) = 251.356640;
  msg_new.K.at(6) = 0.000000;
  msg_new.K.at(7) = 0.000000;
  msg_new.K.at(8) = 1.000000;

  msg_new.R.at(0) = 1.000000;
  msg_new.R.at(1) = 0.000000;
  msg_new.R.at(2) = 0.000000;
  msg_new.R.at(3) = 0.000000;
  msg_new.R.at(4) = 1.000000;
  msg_new.R.at(5) = 0.000000;
  msg_new.R.at(6) = 0.000000;
  msg_new.R.at(7) = 0.000000;
  msg_new.R.at(8) = 1.000000;

  msg_new.P.at(0) = 893.476196;
  msg_new.P.at(1) = 0.000000;
  msg_new.P.at(2) = 320.346635;
  msg_new.P.at(3) = 0.000000;
  msg_new.P.at(4) = 0.000000;
  msg_new.P.at(5) = 891.344666;
  msg_new.P.at(6) = 253.475964;
  msg_new.P.at(7) = 0.000000;
  msg_new.P.at(8) = 0.000000;
  msg_new.P.at(9) = 0.000000;
  msg_new.P.at(10) = 1.000000;
  msg_new.P.at(11) = 0.000000;

  cam_republisher.publish(msg_new);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_republisher");
  ros::NodeHandle nh;

  cam_republisher = nh.advertise<sensor_msgs::CameraInfo>("/republished/camera_info",10);

  ros::Subscriber cam_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera/camera_info",10,camcallBack);

  ros::spin();

  return 0;
}
