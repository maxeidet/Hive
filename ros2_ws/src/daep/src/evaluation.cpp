#include "evaluation.h"

#include <algorithm>

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include "octomap_msgs/conversions.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

CoverageEvaluator::CoverageEvaluator() : Node("coverage_evaluator") {

  this->declare_parameter("boundary/volume", -1.0);

  volume_ = this->get_parameter("boundary/volume").get_parameter_value().get<double>();
  if (volume_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Volume parameter not found... (and no longer needed since we compute it automatically)");
  }

  this->declare_parameter("boundary/min", rclcpp::PARAMETER_DOUBLE_ARRAY);    
  this->declare_parameter("boundary/max", rclcpp::PARAMETER_DOUBLE_ARRAY);
  
  rclcpp::Parameter min_vals_param;
  rclcpp::Parameter max_vals_param;
  
  if (this->get_parameter("boundary/min", min_vals_param)) {
    min_vals = min_vals_param.as_double_array();
  } else {
    RCLCPP_ERROR(this->get_logger(), "No minimum boundary values found...");
  }
  
  if (this->get_parameter("boundary/max", max_vals_param)) {
    max_vals = max_vals_param.as_double_array();
  } else {
    RCLCPP_ERROR(this->get_logger(), "No maximum boundary values found...");
  }
  
  
  // The octomap is reliable, not sensor_data: a full map is megabytes, and best effort
  // drops it whole when one fragment is lost, so with Fast DDS it never arrived.
  octomap_subscription = this->create_subscription<octomap_msgs::msg::Octomap>("octomap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                                                               std::bind(&CoverageEvaluator::octomapCallback,
                                                                                         this, std::placeholders::_1));
  
  coverage_service = this->create_service<daep_msgs::srv::Coverage>("get_coverage", std::bind(&CoverageEvaluator::service_coverage, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Minimum boundary values: %f, %f, %f", min_vals[0], min_vals[1], min_vals[2]);
  RCLCPP_INFO(this->get_logger(), "Maximum boundary values: %f, %f, %f", max_vals[0], max_vals[1], max_vals[2]);

  volume_ = (max_vals[0] - min_vals[0]) * (max_vals[1] - min_vals[1]) * (max_vals[2] - min_vals[2]);
  RCLCPP_INFO(this->get_logger(), "Volume [m^3]: %f", volume_);

}

CoverageEvaluator::~CoverageEvaluator() {

}

void CoverageEvaluator::octomapCallback(const octomap_msgs::msg::Octomap& msg) {
  auto basic_octree = octomap_msgs::fullMsgToMap(msg);
  if (basic_octree == 0) {
    RCLCPP_ERROR(this->get_logger(), "empty octree");
    return;
  }
  std::shared_ptr<octomap::OcTree> octree =
    std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (basic_octree));
  double occupied_space = 0.0;
  double free_space = 0.0;

  // Map limits
  octomap::point3d min(min_vals[0], min_vals[1], min_vals[2]);
  octomap::point3d max(max_vals[0], max_vals[1], max_vals[2]);

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max), end = octree->end_leafs_bbx(); it != end; ++it) {
    const octomap::point3d& pos = it.getCoordinate(); // get the position of the leaf node
    // The part of the leaf inside the bounding box. A leaf can be a large cube (octomap
    // prunes free space) that sticks out of the box; counting all of it gave coverage
    // above 100 %.
    double half = it.getSize() / 2.0;
    double volume = 1.0;
    for (int i = 0; i < 3; i++) {
      volume *= std::max(0.0, std::min<double>(pos(i) + half, max(i)) - std::max<double>(pos(i) - half, min(i)));
    }
    if (octree->isNodeOccupied(*it)){ // occupied leaf node
      occupied_space += volume;
    } else { // free leaf node
      free_space += volume;
    }
  }

  double mapped_space = occupied_space + free_space;
  double unmapped_space = volume_ - mapped_space;
  double coverage = 100.0 * mapped_space / volume_;

  // Set global variables
  coverage_m3_ = mapped_space;
  coverage_p_ = coverage;
  free_ = free_space;
  occupied_ = occupied_space;
  unmapped_ = unmapped_space;
}


void CoverageEvaluator::service_coverage(const std::shared_ptr<daep_msgs::srv::Coverage::Request> request,
                                         std::shared_ptr<daep_msgs::srv::Coverage::Response> response) {
  response->coverage_m3 = coverage_m3_;
  response->coverage_p = coverage_p_;
  response->free     = free_;
  response->occupied = occupied_;
  response->unmapped = unmapped_;
}


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<CoverageEvaluator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);

  RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;

  //  rclcpp::spin(std::make_shared<ManagerM300>());
  //  rclcpp::shutdown();
  
}
