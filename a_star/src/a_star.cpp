#include "../include/a_star_test/LibAstar.h"
#include "../include/a_star_test/AStarNode.h"
#include "std_msgs/String.h"
#include "tcc_msgs/location_goal.h"


int main(int argc, char *argv[]){

	ros::init(argc, argv, "A_STAR");
  AStarNode node(new ros::NodeHandle(),1);
  node.spin();
	return 0;
}
