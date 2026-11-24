#ifndef _EVALUATION_H
#define _EVALUATION_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "daep_msgs/srv/coverage.hpp"

#include "octomap_msgs/msg/octomap.hpp"

#include <vector>

class CoverageEvaluator : public rclcpp::Node {
private:
  std::vector<double> min_vals;
  std::vector<double> max_vals;
  double volume_;
  double coverage_m3_, coverage_p_, free_, occupied_, unmapped_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription;

  rclcpp::Service<daep_msgs::srv::Coverage>::SharedPtr coverage_service;
  
public:
  CoverageEvaluator();
  ~CoverageEvaluator();

  void octomapCallback(const octomap_msgs::msg::Octomap& msg);  

  void service_coverage(const std::shared_ptr<daep_msgs::srv::Coverage::Request> request,
                        std::shared_ptr<daep_msgs::srv::Coverage::Response> response);
  
};




#endif

