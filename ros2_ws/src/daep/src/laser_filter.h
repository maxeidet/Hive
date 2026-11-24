#ifndef _LASER_FILTER_H
#define _LASER_FILTER_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>

class LaserFilter : public rclcpp::Node {
private:
  double lower_angle;
  double upper_angle;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

public:
  LaserFilter();
  ~LaserFilter();

  void scan_callback(const sensor_msgs::msg::LaserScan& msg);  

  
};




#endif

