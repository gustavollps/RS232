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
  vec.at(0) = 0;
  vec.at(1) = 0;
  vec.at(2) = 0;
  vec.at(3) = 0;
  vec.at(4) = 0;
  msg_new.D = vec;

  ROS_INFO("%d",msg_new.D.size());
  cam_republisher.publish(msg_new);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_republisher");
  ros::NodeHandle nh;

  cam_republisher = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info/",10);

  ros::Subscriber cam_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera_info",10,camcallBack);

  ros::spin();

  return 0;
}
