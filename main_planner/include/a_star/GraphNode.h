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

    GraphNode(int type, Point goal, Point position, Point father,float length);
    ~GraphNode();

    void setH(int type);
    void setG(float lenght);

    void setFather(Point father);
    bool operator<(const GraphNode & a) const;

    float F_;
    float G_;
    float H_;

    friend std::ostream& operator<<(std::ostream& os, const GraphNode& node);

    Point father_;
  private:


    float lenght_;

    Point goal_;   

    int heuristic_;

};

unsigned int abs(unsigned int a);

int abs(int a);

#endif
