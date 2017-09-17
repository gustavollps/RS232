#include "../../include/a_star_test/AStarNode.h"

AStarNode::AStarNode(ros::NodeHandle *nh, float dt)
{  

  nh_ = nh;
  dt_ = dt;
  start_ = new Point(0,0);
  goal_ = new Point(9,9);  
  pub_route_ = nh->advertise<std_msgs::String>("/Route",10);
  sub_positions_ = nh->subscribe<tcc_msgs::location_goal>("/Goal", 10, &AStarNode::goalCallback, this);

}

AStarNode::~AStarNode()
{
  pub_route_.shutdown();
  sub_positions_.shutdown();
}

void AStarNode::spin() const
{
  ros::Rate loop(10);
  ROS_INFO("Node Running");
  while(nh_->ok()){
    ROS_INFO("Node Loop");

    ros::spinOnce();
    loop.sleep();
  }
}

void AStarNode::goalCallback(tcc_msgs::location_goal msg)
{
  ROS_INFO("Running pathFinder");

  Point start(msg.start.x,msg.start.y);
  Point goal(msg.goal.x,msg.goal.y);
  nav_msgs::OccupancyGrid map;


  pathRouter_ = new LibAstar(goal,start,map, 10, 10);

  std_msgs::String route;
  route.data = pathRouter_->pathFinder();

  if(!route.data.empty()){
    pub_route_.publish(route);
  }
  else{
    route.data = "0";
    pub_route_.publish(route);
  }

  delete pathRouter_;
}
