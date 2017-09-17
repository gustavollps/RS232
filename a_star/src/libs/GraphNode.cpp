#include "../../include/a_star_test/GraphNode.h"

unsigned int abs(unsigned int a){
  return (a>0)?a:-a;
}

int abs(int a){
  return (a>0)?a:-a;
}

bool GraphNode::operator<(const GraphNode & a) const
{
  if(a.F_ < F_)
  {
    return true;
  }
  else if(a.F_ == F_)
  {
    if(a.G_ < G_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else{
    return false;
  }

}

GraphNode::GraphNode(int heuristic, Point goal, Point position, Point father, unsigned int length)
      : heuristic_{heuristic},goal_{goal}, position_{position}, father_{father}, lenght_{length}
{
  G_=0;
  H_=0;
  setG(length);
  setH(heuristic_);
}

GraphNode::~GraphNode()
{

}

void GraphNode::setG(unsigned int lenght){
  lenght_ = lenght;

  G_ = ((abs(father_.x_-position_.x_) + abs(father_.y_-position_.y_)) == 2) ? lenght_+14:lenght_+10;
  //std::cout << "Passou no G:" << G_ << "Length:" << lenght << std::endl;
  F_ = G_+H_;
}

void GraphNode::setH(int type){
  heuristic_ = type;
  //Manhattan
  if(heuristic_ == MANHATTAN)
    H_ = abs(goal_.x_ - position_.x_) + abs(goal_.y_ - position_.y_);

  //Euclidian
  if(heuristic_ == EUCLIDIAN)
    H_ = (int)(sqrt((goal_.x_ - position_.x_)*(goal_.x_ - position_.x_)+(goal_.y_ - position_.y_)*(goal_.y_ - position_.y_)));

  //Chebyshev distance
  if(heuristic_ == CHEBYSHEV){
    if(abs(goal_.x_-position_.x_)>abs(goal_.y_-position_.y_))
      H_ = abs(goal_.x_-position_.x_);
    else
      H_ = abs(goal_.y_-position_.y_);
  }

  H_ *=10;

  F_ = G_+H_;
}

void GraphNode::setFather(Point father){
  father_ = father;
}

std::ostream& operator<<(std::ostream& os, const GraphNode& node)
{
   os << "G: " << node.G_ << " - H: " << node.H_ <<" - x: " << node.position_.x_ << " - y:" << node.position_.y_;
   return os;
}

