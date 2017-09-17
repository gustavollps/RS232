#include "../../include/main_planner/Planner.h"

/**
 * @brief Planner::Planner
 * @param nh
 */
Planner::Planner(ros::NodeHandle* nh)
{
  nh_ = nh;

  start_.x = 0.0;
  start_.y = 0.0;

  goal_.x = 0.0;
  goal_.y = 0.0;

  pose_estimated_.x = 0.0;
  pose_estimated_.y = 0.0;
  pose_estimated_.theta = 0;

  pose_raw_.x = 0.0;
  pose_raw_.y = 0.0;
  pose_raw_.theta = 0;

  pose_raw_estimated_.x = 0.0;
  pose_raw_estimated_.y = 0.0;
  pose_raw_estimated_.theta = 0;

  pose_raw_offset_.x = 0;
  pose_raw_offset_.y = 0;
  pose_raw_offset_.theta = 0;

  pose_offset_.x = 0;
  pose_offset_.y = 0;
  pose_offset_.theta = 0;

  point_tolerance_ = 0.02;
  map_resolution_ = 0.02;

  Kp_ = 1;
  Kd_ = 0;

  amcl_pose_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/amcl_pose", 1, &Planner::amclCallBack, this);
  odom_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/robot_pose_ekf/odom_combined", 10, &Planner::odometryCallBack, this);

  goal_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, &Planner::goalCallBack, this);

  map_data_sub_ = nh_->subscribe<nav_msgs::MapMetaData>(
      "/map_metadata", 1, &Planner::mapDataCallBack, this);
  map_sub_ = nh_->subscribe<nav_msgs::OccupancyGrid>(
      "/map_inflated", 1, &Planner::mapCallBack, this);

  move_pub_ = nh_->advertise<tcc_msgs::movement_msg>("/cmd_movement", 10);
  // goal_server_ =
  // nh_->advertiseService("/ChangeGoal",&Planner::goalCallBack,this);
  position_timer_ =
      nh_->createTimer(ros::Duration(0.02), &Planner::timerCallBack, this);
  marker_pub_ = nh_->advertise<visualization_msgs::Marker>("/path", 1);

  point_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, &Planner::pointCallBack, this);

  static tf::TransformBroadcaster broadcaster;

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1),
                    tf::Vector3(pose_offset_.x, pose_offset_.y, 0)),
      ros::Time::now(), "map", "odom"));

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::createQuaternionFromYaw(0), tf::Vector3(0, 0, 0.04)),
      ros::Time::now(), "odom", "base_link"));

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0.707106, 0.707106),
                    tf::Vector3(0.0, 0.04, 0.15)),
      ros::Time::now(), "base_link", "laser"));

  path_mode_ = 1;

  getParams();

  routeInterpreter_ = new routeInterpreter(point_tolerance_, map_resolution_);

  path_.header.frame_id = "map";
  path_.ns = "route_frame";
  path_.action = visualization_msgs::Marker::ADD;
  path_.pose.orientation.w = 1.0;
  path_.id = 0;
  path_.type = visualization_msgs::Marker::LINE_STRIP;
  path_.scale.x = 0.01;
  path_.color.a = 1;
  path_.color.r = 1;

  quat_.setW(1);
  quat_.setX(0);
  quat_.setY(0);
  quat_.setZ(0);
}

/**
 * @brief Planner::~Planner
 */
Planner::~Planner()
{
  odom_sub_.shutdown();
  route_sub_.shutdown();

  move_pub_.shutdown();

  goal_server_.shutdown();
}

/**
 * @brief Planner::mapDataCallBack
 * @param msg
 */
void Planner::mapDataCallBack(nav_msgs::MapMetaData msg)
{
  map_height_ = msg.height;
  map_width_ = msg.width;
  map_resolution_ = msg.resolution;
}

/**
 * @brief Planner::mapCallBack
 * @param map
 */
void Planner::mapCallBack(nav_msgs::OccupancyGrid map) { map_ = map; }

/**
 * @brief Planner::amclCallBack
 * @param msg
 */
void Planner::amclCallBack(geometry_msgs::PoseWithCovarianceStamped msg)
{
  if (amcl_feedback_)
  {
    pose_offset_.x = msg.pose.pose.position.x - pose_raw_estimated_.x;
    pose_offset_.y = msg.pose.pose.position.y - pose_raw_estimated_.y;

    pose_offset_.theta = (tf::getYaw(msg.pose.pose.orientation) * 180 / PI) -
                         pose_raw_estimated_.theta;
    if (pose_offset_.theta > 180)
    {
      pose_offset_.theta -= 360;
    }
    else if (pose_offset_.theta < -180)
    {
      pose_offset_.theta += 360;
    }
  }
  else
  {
    pose_offset_.x = 0;
    pose_offset_.y = 0;
    pose_offset_.theta = 0;
    ROS_WARN("TF Published, but AMCL_POSE not used");
  }

  static tf::TransformBroadcaster broadcaster;

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(
          tf::Quaternion(
              msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
          tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0)),
      ros::Time::now(), "map", "amcl_pose"));
  /*
   ROS_INFO("Last estimated Pose: %f | %f  | %f", pose_estimated_.x,
   pose_estimated_.y,
            pose_estimated_.theta);
   ROS_INFO("AMCL pose: %f | %f  | %f", msg.pose.pose.position.x,
            msg.pose.pose.position.y,
   (tf::getYaw(msg.pose.pose.orientation)*180/PI));
   ROS_INFO("Offset: %f | %f  | %f", pose_offset_.x,
            pose_offset_.y, pose_offset_.theta);
   ROS_INFO("----------------------------------------------------------------");
   */
}

void Planner::pointCallBack(geometry_msgs::PoseWithCovarianceStamped msg)
{
  pose_raw_offset_.x = msg.pose.pose.position.x - pose_raw_.x;
  pose_raw_offset_.y = msg.pose.pose.position.y - pose_raw_.y;
  pose_raw_offset_.theta =
      tf::getYaw(msg.pose.pose.orientation) * 180.0 / PI - pose_raw_.theta;
  ROS_WARN("OFFSET SET");
}

/**
 * @brief Planner::odometryCallBack
 * @param msg
 */
void Planner::odometryCallBack(geometry_msgs::PoseWithCovarianceStamped msg)
{
  pose_raw_.x = msg.pose.pose.position.x;
  pose_raw_.y = msg.pose.pose.position.y;
  pose_raw_.theta = tf::getYaw(msg.pose.pose.orientation) * 180 / PI;

  pose_raw_estimated_.x = pose_raw_.x + pose_raw_offset_.x;
  pose_raw_estimated_.y = pose_raw_.y + pose_raw_offset_.y;
  pose_raw_estimated_.theta =
      tf::getYaw(msg.pose.pose.orientation) * 180.0 / PI +
      pose_raw_offset_.theta;

  pose_estimated_.x = pose_raw_estimated_.x + pose_offset_.x;
  pose_estimated_.y = pose_raw_estimated_.y + pose_offset_.y;
  pose_estimated_.theta = pose_raw_estimated_.theta + pose_offset_.theta;

  if (pose_estimated_.theta > 180)
  {
    pose_estimated_.theta -= 360;
  }
  else if (pose_estimated_.theta < -180)
  {
    pose_estimated_.theta += 360;
  }

  /*
    ROS_INFO("RawPose: %f - %f  - %f", pose_raw_.x,
             pose_raw_.y, pose_raw_.theta);
    ROS_INFO("Estimated Pose: %f - %f  - %f", pose_estimated_.x,
    pose_estimated_.y,
             pose_estimated_.theta);
    ROS_INFO("----------------------------------------------------------------");
  */
  quat_.setW(msg.pose.pose.orientation.w);
  quat_.setX(msg.pose.pose.orientation.x);
  quat_.setY(msg.pose.pose.orientation.y);
  quat_.setZ(msg.pose.pose.orientation.z);
}

/**
 * @brief Planner::goalCallBack
 * @param msg
 */
void Planner::goalCallBack(geometry_msgs::PoseStamped msg)
{
  ROS_INFO("GOAL SELECTED! X: %f, Y: %f", msg.pose.position.x,
           msg.pose.position.y);

  goal_.x = msg.pose.position.x / map_resolution_;
  goal_.y = msg.pose.position.y / map_resolution_;

  Point goal_param(goal_.x, goal_.y);
  Point start_param(pose_estimated_.x / map_resolution_,
                    pose_estimated_.y / map_resolution_);

  pathFinder_ = new LibAstar(goal_param, start_param, map_, map_width_,
                             map_height_, path_mode_);
  Point2D start;
  start.x = pose_estimated_.x;
  start.y = pose_estimated_.y;
  thread_astar_ =
      new boost::thread(this->Planner::thread_astar_callBack, start,
                        routeInterpreter_, pathFinder_, &new_route_);

  path_.points.clear();
  path_.action = visualization_msgs::Marker::DELETEALL;
  path_.header.stamp = ros::Time::now();
  marker_pub_.publish(path_);
}

/**
 * @brief Planner::thread_astar_callBack
 * @param start
 * @param Astar
 * @param pathFinder
 * @param new_route
 */
void Planner::thread_astar_callBack(Point2D start, routeInterpreter* Astar,
                                    LibAstar* pathFinder, bool* new_route)
{

  std::string a_route = pathFinder->pathFinder();

  std::cout << "Route: " << a_route << std::endl;

  Astar->loadNewRoute(a_route, start);

  if (Astar->hasRoute())
  {
    new_route[0] = true;
  }
}

/**
 * @brief Planner::timerCallBack
 * @param event
 */
void Planner::timerCallBack(const ros::TimerEvent& event)
{

  static tf::TransformBroadcaster broadcaster;

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::createQuaternionFromYaw(0), tf::Vector3(0, 0, 0)),
      ros::Time::now(), "map", "odom"));

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(
          tf::createQuaternionFromYaw(pose_raw_estimated_.theta * PI / 180.0),
          tf::Vector3(pose_raw_estimated_.x, pose_raw_estimated_.y, 0.02)),
      ros::Time::now(), "odom", "base_link"));

  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::createQuaternionFromYaw(90 * PI / 180),
                    tf::Vector3(0.0, 0.04, 0.10)),
      ros::Time::now(), "base_link", "laser"));

  xyPose_.x = pose_estimated_.x;
  xyPose_.y = pose_estimated_.y;

  if (routeInterpreter_->hasRoute())
  {
    next_goal_ = routeInterpreter_->nextGoal(xyPose_);
    if (new_route_)
    {
      geometry_msgs::Point point_path;
      for (int i = 0; i < routeInterpreter_->route_points_.size(); i++)
      {
        point_path.x = routeInterpreter_->route_points_.at(i).x;
        point_path.y = routeInterpreter_->route_points_.at(i).y;
        path_.points.push_back(point_path);
      }

      path_.action = visualization_msgs::Marker::ADD;
      path_.header.stamp = ros::Time::now();
      marker_pub_.publish(path_);
      new_route_ = false;
    }
  }
  else
  {
    next_goal_.x = pose_estimated_.x;
    next_goal_.y = pose_estimated_.y;
  }

  float goal_dist = sqrt(
      (next_goal_.x - pose_estimated_.x) * (next_goal_.x - pose_estimated_.x) +
      (next_goal_.y - pose_estimated_.y) * (next_goal_.y - pose_estimated_.y));
  dx_last_ = dx_;
  dy_last_ = dy_;
  dx_ = next_goal_.x - pose_estimated_.x;
  dy_ = next_goal_.y - pose_estimated_.y;

  float phi = atan2(dy_, dx_);

#ifdef DEBUG
  ROS_INFO("\nDX: %f \nDY: %f \nGoal X: %f\nPose X: %f\nGoal Y: %f\nPose Y: %f "
           "\nTheta: %f\nGoalDist: %f",
           dx, dy, next_goal_.x, pose_.x, next_goal_.y, pose_.y,
           phi * 180.0 / PI, goal_dist);
#endif
  geometry_msgs::Twist cmd_vel_msg;
  tcc_msgs::movement_msg mov_cmd;

#ifdef P_VELOCITY
  if (fabs(dx_) > point_tolerance_)
  {
    if (dx_ > 0)
    {
      mov_cmd.x_spd = dx_ * Kp_ + (dx_ - dx_last_) * Kd_ + min_speed_;
    }
    else
    {
      mov_cmd.x_spd = dx_ * Kp_ + (dx_ - dx_last_) * Kd_ - min_speed_;
    }
  }
  else
  {
    mov_cmd.x_spd = 0;
  }

  if (fabs(dy_) > point_tolerance_)
  {
    if (dy_ > 0)
    {
      mov_cmd.y_spd = dy_ * Kp_ + (dy_ - dy_last_) * Kd_+ min_speed_;
    }
    else
    {
      mov_cmd.y_spd = dy_ * Kp_ + (dy_ - dy_last_) * Kd_- min_speed_;
    }
  }
  else
  {
    mov_cmd.y_spd = 0;
  }

  mov_cmd.x_spd = constrain(mov_cmd.x_spd, -top_speed_, top_speed_);
  mov_cmd.y_spd = constrain(mov_cmd.y_spd, -top_speed_, top_speed_);

  mov_cmd.angle = pose_offset_.theta;

#else // CONSTANT VELOCITY
  if (goal_dist > point_tolerance_)
  {
    mov_cmd.y_spd = top_speed_ * sin(phi);
    mov_cmd.x_spd = top_speed_ * cos(phi);
  }

  mov_cmd.angle = pose_offset_.theta;
#endif

#ifdef DEBUG
  ROS_INFO("\nXspd: %f\nYspd: %f ", mov_cmd.x_spd, mov_cmd.y_spd);
#endif

  if (!remote_control_)
  {
    move_pub_.publish(mov_cmd);
  }
}

/**
 * @brief Planner::constrain
 * @param value
 * @param min
 * @param max
 * @return
 */
float Planner::constrain(float value, float min, float max)
{
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}

/**
 * @brief Planner::getParams
 */
void Planner::getParams()
{
  nh_->getParam("/Main_Planner/point_tolerance", point_tolerance_);
  nh_->getParam("/Main_Planner/amcl_feedback", amcl_feedback_);
  if (amcl_feedback_)
  {
    ROS_WARN("AMCL FEEDBACK ON");
  }
  else
  {
    ROS_WARN("BLIND MODE ON");
  }

  nh_->getParam("/Main_Planner/remote_control", remote_control_);
  if (remote_control_)
  {
    ROS_WARN("REMOTE CONTROL MODE");
  }
  else
  {
    ROS_WARN("AUTONOMOUS MODE");
  }

  nh_->getParam("/Main_Planner/path_mode", path_mode_);

#ifdef P_VELOCITY
  nh_->getParam("/Main_Planner/top_speed", top_speed_);
  nh_->getParam("/Main_Planner/Kp_dist", Kp_);
  nh_->getParam("/Main_Planner/Kd_dist", Kd_);
  nh_->getParam("/Main_Planner/min_speed", min_speed_);
#else
  nh_->getParam("/Main_Planner/cte_speed", top_speed_);
#endif
}
