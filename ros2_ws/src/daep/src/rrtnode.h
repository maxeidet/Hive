#ifndef _RRT_NODE_H_
#define _RRT_NODE_H_

#include <eigen3/Eigen/Dense>
#include <vector>

class RrtNode {
private:
  Eigen::Vector3d pos;
  Eigen::Vector3d dir;
  RrtNode *parent{NULL};
  std::vector<RrtNode *> children;

public:
  RrtNode();
  RrtNode(RrtNode * parent_);
  RrtNode(double x, double y, double z);
  ~RrtNode();
  double cost();
  double time_cost(double drone_linear_velocity, double drone_angular_velocity);
  double time_to_reach(const Eigen::Vector3d& pos, double drone_linear_velocity, double drone_angular_velocity);

  double x() {
    return pos[0];
  }

  double y() {
    return pos[1];
  }

  double z() {
    return pos[2];
  }

  RrtNode * get_parent() {
    return parent;
  }

  void set_parent(RrtNode * par) {
    parent = par;
  }

  Eigen::Vector3d get_pos() {
    return pos;
  }
  void set_pos(Eigen::Vector3d pos_) {
    pos = pos_;
  }

  void add_child(RrtNode * child) {
    children.push_back(child);
  }

  Eigen::Vector3d get_dir() {
    return dir;
  }

  void set_dir(Eigen::Vector3d dir_) {
    dir = dir_;
  }


};



#endif

