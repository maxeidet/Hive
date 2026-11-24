#include "rrtnode.h"

RrtNode::RrtNode () {
  parent = 0;
}

RrtNode::RrtNode(RrtNode * parent_) : parent(parent_) {

}

RrtNode::RrtNode (double x, double y, double z) {
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
  parent = 0;
}

RrtNode::~RrtNode () {
  for (typename std::vector<RrtNode *>::iterator node_it = children.begin();
       node_it != children.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
}

double RrtNode::cost() {
  if (parent)
    return (pos - parent->pos).norm() + parent->cost();
  return 0;
}

double RrtNode::time_cost(double drone_linear_velocity, double drone_angular_velocity) {
  if (this->parent) {
    return this->time_to_reach(this->parent->pos, drone_linear_velocity, drone_angular_velocity) + this->parent->time_cost(drone_linear_velocity, drone_angular_velocity);
  } else {
    return 0;
  }
}

double RrtNode::time_to_reach(const Eigen::Vector3d& other, double drone_linear_velocity, double drone_angular_velocity) {
  
    //1. Compute angle from current to direction of other
    Eigen::Vector3d dir_to_other = (other - this->pos).normalized();
    double angle = acos(dir_to_other.dot(this->get_dir()));
    double rotation_time = angle / drone_angular_velocity;

    //2. Compute distance from current to other
    double euclidean_distance = (other - this->pos).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / drone_linear_velocity;

    return rotation_time + linear_time;
}




