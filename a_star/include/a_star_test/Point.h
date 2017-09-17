#ifndef POINT
#define POINT

#include <iostream>

class Point{
	
	public:
		int x_;
		int y_;
		Point(int x, int y);
		~Point();
		Point operator+(Point& point);
		Point operator-(Point& point);
    bool operator==(Point& point);
    bool operator!=(Point& point);

    friend std::ostream& operator<<(std::ostream& os, const Point& point);
};

//int abs(int a){
//  return (a>0)?a:-a;
//}

#endif
