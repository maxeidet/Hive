#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <octomap/OcTree.h>
#include "param.h"
#include <cmath>

class RRTNode
{
public:
  Eigen::Vector4d state_;
  RRTNode* parent_;
  std::vector<RRTNode*> children_;
  double gain_;
  double dynamic_gain_;
  double dfm_score_;
  bool gain_explicitly_calculated_;
  double drone_angular_velocity;
  double drone_linear_velocity;
  bool dynamic_collision_;

  RRTNode(double ang_vel, double lin_vel) : parent_(NULL), gain_(0.0), dynamic_gain_(0.0), dfm_score_(0.0), gain_explicitly_calculated_(false), drone_angular_velocity(ang_vel), drone_linear_velocity(lin_vel), dynamic_collision_(false)
  {
  }

  ~RRTNode()
  {
    for (typename std::vector<RRTNode*>::iterator node_it = children_.begin();
         node_it != children_.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  RRTNode* getCopyOfParentBranch()
  {
    RRTNode* current_node = this;
    //RRTNode* current_child_node = NULL;
    RRTNode* new_node;
    RRTNode* new_child_node = NULL;

    while (current_node)
    {
      new_node = new RRTNode(drone_angular_velocity, drone_linear_velocity);
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->dynamic_gain_ = current_node->dynamic_gain_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      //current_child_node = current_node;
      current_node = current_node->parent_;
      new_child_node = new_node;
    }

    return new_node;
  }

  // Dynamic score
  double dynamic_score(double lambda, double zeta)
  { 
    // zeta * frequency map
    if (this->parent_){
      return this->parent_->dynamic_score(lambda, zeta) +
             this->dynamic_gain_ * exp(-lambda * this->distance(this->parent_)) * (1 + (zeta * this->dfm_score_));
             }
    else  {
      return this->dynamic_gain_ * (1 + (zeta * this->dfm_score_));     
    } 
  }

  // Static score
  double score(double lambda)
  { 
    if (this->parent_)
      return this->parent_->score(lambda) +
             this->gain_ * exp(-lambda * this->distance(this->parent_));
    else
      return this->gain_;
  }

  double cost()
  {
    if (this->parent_)
      return this->distance(this->parent_) + this->parent_->cost();
    else
      return 0;
  }

  double distance(RRTNode* other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }

  double time_cost(Eigen::Vector4d robot_state, bool rotating_while_walking)
  {
    if (this->parent_)
      return this->time_to_reach(this->parent_, robot_state, rotating_while_walking) + this->parent_->time_cost(robot_state, rotating_while_walking);
    else
      return 0;
  }

    /*
  !Note!
  * We need the correct linear and angular speed from the Turtlebot. (Currently 1 m/s and 1 rad/s)
  * This will be an estimate since the Turtlebot is currently slowing down when it approaches the goal.
  * The Turtlebot is also adjusting itself as it travels, which adds time.
  * 
  * the bool complete_time is used to compute the time to reach the node if we are not checking for collision.
  * When collision checking, we don't need rotation_time_2 since we only care about when we are in the goal_state.
  */
  double time_to_reach(RRTNode* other, Eigen::Vector4d robot_state, bool rotating_while_walking){
 
    double current_yaw{};
    double target_yaw = other->state_[3];

    if (this->parent_)
      current_yaw = this->state_[3];
    else
      current_yaw = robot_state[3];

    // ** 1. Time to turn to *direction* of other **

    //Find direction vector from current to target
    Eigen::Vector3d current(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d target(other->state_[0], other->state_[1], other->state_[2]);
    Eigen::Vector3d direction = (target - current).normalized();

    //Find vector from current_yaw (Polar form to rectangular, r = 1)
    Eigen::Vector3d yaw_vector(cos(current_yaw), sin(current_yaw), this->state_[2]);
    yaw_vector = yaw_vector.normalized();
  
    //Compute angle difference between yaw_vector and direction.
    double angle1 = acos(direction.dot(yaw_vector));

    double rotation_time_1 = fabs(angle1) / drone_angular_velocity;

    // ** 2. Time to travel to other from this (nearest) **
    
    // Calculate Euclidean distance in x-y plane
    double euclidean_distance = (target - current).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / (drone_linear_velocity);

    // ** 3. Time to turn to target_angle from *direction* ** 
    Eigen::Vector3d tar_vector(cos(target_yaw), sin(target_yaw), other->state_[2]);
    tar_vector = tar_vector.normalized();
    double angle2 = acos(direction.dot(tar_vector));

    double rotation_time_2 = fabs(angle2) / drone_angular_velocity;

    /* Debugging
    std::cout << "Time to turn ang1 " << angle1 << " takes " << rotation_time_1 << " seconds" << std::endl;
    std::cout << "Time to travel distance " << euclidean_distance << " takes " << linear_time << " seconds" << std::endl;
    std::cout << "Time to turn ang2 " << angle2 << " takes " << rotation_time_2 << " seconds" << std::endl;
    */

    double total_time = rotation_time_1 + linear_time + rotation_time_2;

    if(rotating_while_walking)
    {
      total_time = rotation_time_1 + linear_time;
    }
    else
    {
      total_time = rotation_time_1 + linear_time + rotation_time_2;
    }
      
    return total_time;
  }
};

#endif
