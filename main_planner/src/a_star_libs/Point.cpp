#include "../../include/a_star/Point.h"

Point::Point(int x, int y)
{
	x_=x;
	y_=y;
}

Point::~Point(){}

Point Point::operator+(Point& point)
{
	x_ += point.x_;
	y_ += point.y_;
	return Point(x_,y_);
}

Point Point::operator-(Point& point)
{
	x_ -= point.x_;
	y_ -= point.y_;
	return Point(x_,y_);
}

bool Point::operator ==(Point& point)
{
  if((x_ != point.x_) || (y_ != point.y_))
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool Point::operator !=(Point& point)
{
  if((x_ != point.x_) || (y_ != point.y_))
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::ostream& operator<<(std::ostream& os, const Point& point)
{
   os << "x: " << point.x_ << " - y:" << point.y_;
   return os;
}
