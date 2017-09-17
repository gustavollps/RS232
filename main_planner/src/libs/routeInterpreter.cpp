#include "../../include/main_planner/routeInterpreter.h"

routeInterpreter::routeInterpreter(float tolerance, float resolution)
{
  has_route_ = false;

  goal_.x = 0;
  goal_.y = 0;
  pose_.x = 0;
  pose_.y = 0;

  route_ = "";
  route_index_ = 0;
  route_size_ = 0;

  tolerance_ = tolerance;
  map_resolution_ = resolution;
}

bool routeInterpreter::loadNewRoute(std::string Route, Point2D pose)
{
  //std::cout << "load route" << std::endl;
  if(Route == ""){
    has_route_ = false;
    return false;
  }
  route_ = Route;

  bool end = false;
  pose_ = pose;
  route_index_ = 0;
  route_points_.clear();

  //route_points_.push_back(pose);

  char current_dir ='z';
  char previous_dir;

  while (1)
  {    

    if (route_index_ < route_.size() && !end)
    {
      previous_dir = current_dir;
      current_dir = route_.at(route_index_);
      if(current_dir != previous_dir){
        route_points_.push_back(pose_);
      }
      switch (current_dir)
      {
      case 'a': // Upper Left
        pose_.x -= map_resolution_;
        pose_.y += map_resolution_;
        break;
      case 'b': // Up
        pose_.y += map_resolution_;
        break;
      case 'c': // Upper right
        pose_.x += map_resolution_;
        pose_.y += map_resolution_;
        break;
      case 'd': // Left
        pose_.x -= map_resolution_;
        break;
      case 'e': // Right
        pose_.x += map_resolution_;
        break;
      case 'f': // Lower left
        pose_.x -= map_resolution_;
        pose_.y -= map_resolution_;
        break;
      case 'g': // Down
        pose_.y -= map_resolution_;
        break;
      case 'h': // Lower right
        pose_.x += map_resolution_;
        pose_.y -= map_resolution_;
        break;
      default: // default (end of route)
        end = true;
        break;
      }

      if(route_index_ == route_.size()-1){ //End point of the route
        route_points_.push_back(pose_);
      }

      route_index_++;
    }
    else
    {
      break;
    }



  }

  route_index_ = 0;

  if (end)
  {
    has_route_ = false;
    return false;
  }
  else
  {
    has_route_ = true;
    for(int i=0;i<route_points_.size();i++){
      std::cout << route_points_.at(i).x << " - " << route_points_.at(i).y << std::endl;
    }
    return true;
  }
}

void routeInterpreter::updatePose(Point2D pose) { pose_ = pose; }

bool routeInterpreter::hasRoute() { return has_route_; }

float routeInterpreter::fabs(float value)
{
  if (value < 0)
    return -value;
  else
    return value;
}

Point2D routeInterpreter::nextGoal(Point2D pose)
{
  pose_ = pose;
  Point2D next_goal = pose;
  // std::cout << "Next Goal" << std::endl;  
  for (int i = route_index_ + 1; i < route_points_.size() - 1; i++)
  {
    float dx = route_points_.at(i).x - pose_.x;
    float dy = route_points_.at(i).y - pose_.y;
    // std::cout << "Distances: " << dx << " -- " << dy << std::endl;
    if (fabs(dx) < tolerance_ && fabs(dy) < tolerance_)
    {
      if (i + 2 < route_points_.size())
      {
        next_goal = route_points_.at(i + 2);
        // std::cout << "Updated to element %d" << i + 2 << std::endl;
      }
      if (i + 1 == route_points_.size())
      {
        has_route_ = false;
      }
      route_index_ = i;
    }

    // std::cout << "Next GOAL: " << next_goal.x << " - " << next_goal.y << " -
    // " << route_index_ << std::endl;
    // std::cout << "POINT: " << next_goal.x << " - " << next_goal.y << " - " <<
    // route_index_ << std::endl;
  }

  // In the case the robot leaves the A* track the next goal its still the last
  // valid one
  if (next_goal.x == pose.x && next_goal.y == pose.y)
  {
    next_goal = route_points_.at(route_index_ + 1);
  }

  // std::cout << "End next goal" << std::endl;
  //  for(int i=0;i<route_points_.size();i++){
  //    std::cout << route_points_.at(i).x << " - " << route_points_.at(i).y <<
  //    std::endl;
  //  }
  // std::cout << next_goal.x << " - " << next_goal.y << std::endl;
  return next_goal;
}

void routeInterpreter::setTolerance(float dist_meters)
{
  tolerance_ = dist_meters;
}
