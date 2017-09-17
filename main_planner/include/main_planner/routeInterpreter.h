#ifndef ROUTEINTERPRETER_H
#define ROUTEINTERPRETER_H

#include <string>
#include <vector>
#include <stdlib.h>
#include <iostream>

struct Point2D{
  float x;
  float y;
};

class routeInterpreter
{
public:
  routeInterpreter(float tolerance, float resolution);
  bool loadNewRoute(std::string Route, Point2D pose);
  Point2D nextGoal(Point2D pose);
  void setTolerance(float dist_meters);
  void updatePose(Point2D pose);
  bool hasRoute();

  std::vector<Point2D> route_points_;

private:
  Point2D goal_;
  Point2D pose_;
  Point2D route_pose_;

  std::string route_;

  int32_t route_size_;
  int32_t route_index_;

  float tolerance_;
  float map_resolution_;

  bool has_route_;

  float fabs(float value);

};

#endif // ROUTEINTERPRETER_H
