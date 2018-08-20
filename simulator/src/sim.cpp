#include "../include/simulator/simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim");
  ros::NodeHandle nh;

  Simulator *sim = new Simulator(&nh);
  ros::spin();
  return 0;
}
