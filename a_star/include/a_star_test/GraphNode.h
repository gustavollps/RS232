#ifndef GRAPH_NODE
#define GRAPH_NODE

#include "Point.h"
#include <math.h>

#define MANHATTAN 1
#define EUCLIDIAN 2
#define CHEBYSHEV 3

class GraphNode{
	public:		    
    Point position_;

    GraphNode(int type, Point goal, Point position, Point father,unsigned int length);
    ~GraphNode();

    void setH(int type);
    void setG(unsigned int lenght);

    void setFather(Point father);
    bool operator<(const GraphNode & a) const;

    unsigned int F_;
    unsigned int G_;
    unsigned int H_;

    friend std::ostream& operator<<(std::ostream& os, const GraphNode& node);

    Point father_;
  private:


    unsigned int lenght_;

    Point goal_;   

    int heuristic_;

};

unsigned int abs(unsigned int a);

int abs(int a);

#endif
