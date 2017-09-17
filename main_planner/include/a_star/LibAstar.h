#ifndef _LIBASTAR_
#define _LIBASTAR_

#include "ros/ros.h"
#include "GraphNode.h"
#include <queue>
#include <stdio.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

/*
#define MATRIX_HEIGHT 10
#define MATRIX_WIDTH 10
#define CURSE_LENGHT (MATRIX_WIDTH + MATRIX_HEIGHT) / 2 * 1
*/


//#define DEBUG
//#define DEBUG2

class LibAstar
{

public:
  LibAstar(Point& goal, Point& start, nav_msgs::OccupancyGrid the_map,
           int width, int height, int mode);
  ~LibAstar();

  std::string pathFinder();

  Point goal_;
  Point start_;
  std::priority_queue<GraphNode> open_list[2];

  std::vector<std::vector<bool> > map_;
  std::vector<std::vector<float> > opened_list;
  std::vector<std::vector<float> > closed_list;
  std::vector<std::vector<char> > dir_map;

  /*
  unsigned int closed_list[MATRIX_HEIGHT][MATRIX_WIDTH];
  unsigned int opened_list[MATRIX_HEIGHT][MATRIX_WIDTH];
  bool map_[MATRIX_HEIGHT][MATRIX_WIDTH];
  char dir_map[MATRIX_HEIGHT][MATRIX_WIDTH];
  */

  char getDir(int, int);

private:
  bool validPoint(Point point);
  void clearLists();
  void clearMap();
  void clearDirMap();
  void printClosedList();
  void printOpenedList();
  void printDirMap();
  void printMap();
  void debug();

  int MATRIX_HEIGHT;
  int MATRIX_WIDTH;
  int MODE_;
};

#endif
