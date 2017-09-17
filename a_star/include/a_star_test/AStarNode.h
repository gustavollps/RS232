#ifndef ASTARNODE_H
#define ASTARNODE_H

#include "LibAstar.h"
#include "std_msgs/String.h"
#include "tcc_msgs/location_goal.h"
#include <ros/ros.h>
class AStarNode
{
  public:
    Point *start_,*goal_;
    AStarNode(ros::NodeHandle *nh, float dt);
    ~AStarNode();
    virtual void spin() const;

  private:

    LibAstar *pathRouter_;
    ros::Publisher pub_route_;
    ros::Subscriber sub_positions_;
    ros::Timer main_timer_;
    ros::NodeHandle *nh_;

    float dt_;

    void timerCallback(const ros::TimerEvent &event);
    void goalCallback(tcc_msgs::location_goal msg);

};

#endif // ASTARNODE_H
