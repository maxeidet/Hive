#include "laser_filter.h"

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

LaserFilter::LaserFilter() : Node("laser_filter") {

  this->declare_parameter("lower_angle", -0.523);
  this->declare_parameter("upper_angle", 0.523);

  double angle_offset = 1.55;

  lower_angle = this->get_parameter("lower_angle").get_parameter_value().get<double>() - angle_offset;
  upper_angle = this->get_parameter("upper_angle").get_parameter_value().get<double>() - angle_offset;

  RCLCPP_INFO(this->get_logger(), "Lower Angle: %f", lower_angle*180.0/M_PI);
  RCLCPP_INFO(this->get_logger(), "Upper Angle: %f", upper_angle*180.0/M_PI);
  
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  
  scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1,
                                                                             std::bind(&LaserFilter::scan_callback,
                                                                                       this, std::placeholders::_1));

  scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 1);  
}

LaserFilter::~LaserFilter() {

}

void LaserFilter::scan_callback(const sensor_msgs::msg::LaserScan& msg) {
  float angle_min = msg.angle_min;
  float angle_max = msg.angle_max;
  float angle_increment = msg.angle_increment;
  float time_increment = msg.time_increment;
  float scan_time = msg.scan_time;
  float range_min = msg.range_min;
  float range_max = msg.range_max;

  range_max = 5.0;

  bool have_intensity;
  if (msg.intensities.size() > 0) {
    have_intensity = true;
  }
  
  RCLCPP_INFO(this->get_logger(), "%f %f %f - %f %f - %f %f - %d",
              angle_min*180.0/M_PI, angle_max*180.0/M_PI, angle_increment*180.0/M_PI,
              time_increment, scan_time,
              range_min, range_max, have_intensity);

  sensor_msgs::msg::LaserScan out;
  out.angle_min = lower_angle;
  out.angle_max = upper_angle;
  out.angle_increment = angle_increment;
  out.time_increment = time_increment;
  out.range_min = range_min;
  out.range_max = range_max;

  out.header = msg.header;

  float current_scan_time = 0.0;
  float current_angle = angle_min;
  for (unsigned int i=0; i<msg.ranges.size(); i++) {
    if ((current_angle > lower_angle) && (current_angle < upper_angle)) {
      // RCLCPP_INFO(this->get_logger(), "PUSH: %f", current_angle*180.0/M_PI);
      out.ranges.push_back(msg.ranges[i]);
      if (have_intensity) {
        out.intensities.push_back(msg.intensities[i]);
      }
    }
    current_angle += angle_increment;
    current_scan_time += time_increment;
  }
  out.scan_time = current_scan_time;
  RCLCPP_INFO(this->get_logger(), "PUB FILTERED");
  scan_pub->publish(out);
}


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<LaserFilter>();
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
