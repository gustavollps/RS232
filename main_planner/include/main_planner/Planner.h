#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <tcc_msgs/pose2D.h>
#include <tcc_msgs/movement_msg.h>
#include <tcc_msgs/location_goal.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "routeInterpreter.h"
#include "../../include/a_star/LibAstar.h"
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

#define PI 3.14159265
//#define DEBUG
#define P_VELOCITY

struct Pose
{
  float x;
  float y;
  float theta;
};

class Planner
{
public:
  Planner(ros::NodeHandle* nh);
  ~Planner();

private:
  void odometryCallBack(geometry_msgs::PoseWithCovarianceStamped msg);
  void goalCallBack(geometry_msgs::PoseStamped msg);
  void timerCallBack(const ros::TimerEvent& event);
  void mapDataCallBack(nav_msgs::MapMetaData msg);
  void mapCallBack(nav_msgs::OccupancyGrid map);
  void amclCallBack(geometry_msgs::PoseWithCovarianceStamped msg);
  void pointCallBack(geometry_msgs::PoseWithCovarianceStamped msg);
  void arucoCallBack(geometry_msgs::PoseWithCovarianceStamped msg);


  float constrain(float value, float min, float max);
  void getParams();

  ros::NodeHandle* nh_;

  ros::Subscriber amcl_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber route_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber point_sub_;
  ros::Subscriber aruco_sub_;


  ros::Subscriber map_data_sub_;
  ros::Subscriber map_sub_;

  ros::Publisher move_pub_;
  ros::Publisher marker_pub_;

  /*FPB*/
  ros::Publisher fpb_pub_;
  geometry_msgs::PoseWithCovarianceStamped fpb_msg_;
  void fpbCallBack(const ros::TimerEvent& event);
  ros::Timer fpb_timer_;

  ros::Timer position_timer_;

  ros::ServiceServer goal_server_;

  routeInterpreter* routeInterpreter_;

  Point2D start_, goal_, next_goal_;
  Pose pose_estimated_, pose_raw_, pose_raw_offset_, pose_offset_, pose_raw_estimated_;
  /*ARUCO*/
  Pose pose_aruco_,odom_ekf_0,odom_ekf_1,odom_ekf_diff;
  /*ARUCO*/
  Point2D xyPose_;

  LibAstar* pathFinder_;
  boost::thread* thread_astar_;
  static void thread_astar_callBack(Point2D start, routeInterpreter* Astar,
                                    LibAstar* pathFinder, bool* new_route);
  bool new_route_;
  int path_mode_;

  float point_tolerance_;
  float top_speed_;
  float min_speed_;
  float Kp_;
  float Kd_;
  float dx_;
  float dx_last_;
  float dy_;
  float dy_last_;

  bool amcl_feedback_;
  bool remote_control_;

  tf::Quaternion quat_;

  int map_height_;
  int map_width_;
  float map_resolution_;

  nav_msgs::OccupancyGrid map_;
  visualization_msgs::Marker path_;
};

#endif // PLANNER_H
