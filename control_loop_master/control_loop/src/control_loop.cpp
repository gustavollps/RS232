#include <ros/ros.h>
#include "../include/control_loop/ControlLoop.h"

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "Control_Loop");
  ControlLoop controlNode(new ros::NodeHandle(),CONTROL_FREQ);
  ros::spin();
  return 0;
}
