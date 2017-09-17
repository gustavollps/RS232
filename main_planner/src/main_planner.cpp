#include <ros/ros.h>
#include "../include/main_planner/Planner.h"

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "Main_Planner");
  Planner *main_planner = new Planner(new ros::NodeHandle);
  ros::spin();
}
