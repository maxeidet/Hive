#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include "octomap_msgs/conversions.h"

#include <octomap/OcTree.h>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

using namespace std::placeholders;

class CoverageStandalone : public rclcpp::Node {
public:
  CoverageStandalone() : Node("coverage_standalone") {

    // ====================================================================
    // HARDCODED PARAMETERS — ändra dessa efter behov
    // ====================================================================
    min_vals_ = {-6.0, -6.0, 0.0};   // boundary min [x, y, z]
    max_vals_ = { 6.0,  6.0,  2.0};   // boundary max [x, y, z]

    output_file_ = "coverage.csv";   // filnamn för output
    octomap_topic_ = "/husky0/octomap_binary";  // topic att lyssna på
    // ====================================================================

    volume_ = (max_vals_[0] - min_vals_[0]) *
              (max_vals_[1] - min_vals_[1]) *
              (max_vals_[2] - min_vals_[2]);

    RCLCPP_INFO(this->get_logger(), "Boundary min: [%.2f, %.2f, %.2f]",
                min_vals_[0], min_vals_[1], min_vals_[2]);
    RCLCPP_INFO(this->get_logger(), "Boundary max: [%.2f, %.2f, %.2f]",
                max_vals_[0], max_vals_[1], max_vals_[2]);
    RCLCPP_INFO(this->get_logger(), "Volume: %.2f m^3", volume_);
    RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", octomap_topic_.c_str());

    // Skriv CSV-header
    {
      std::ofstream ofs(output_file_, std::ios::trunc);
      ofs << "Time, Coverage (m3), Coverage (%), Free space, Occupied Space, Unmapped Space\n";
    }

    start_time_ = this->now();

    // Reliable, not sensor_data: best effort drops a megabyte map whole (see aeplanner.cpp).
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        octomap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&CoverageStandalone::octomapCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Coverage standalone node started.");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap& msg) {
    auto basic_octree = octomap_msgs::msgToMap(msg);
    if (basic_octree == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Received empty octree!");
      return;
    }

    std::shared_ptr<octomap::OcTree> octree(
        dynamic_cast<octomap::OcTree*>(basic_octree));

    if (!octree) {
      RCLCPP_ERROR(this->get_logger(), "Failed to cast to OcTree!");
      return;
    }

    double occupied_space = 0.0;
    double free_space = 0.0;

    octomap::point3d min_pt(min_vals_[0], min_vals_[1], min_vals_[2]);
    octomap::point3d max_pt(max_vals_[0], max_vals_[1], max_vals_[2]);

    for (auto it = octree->begin_leafs_bbx(min_pt, max_pt),
              end = octree->end_leafs_bbx(); it != end; ++it) {
      const octomap::point3d& pos = it.getCoordinate();
      if (pos.x() > min_pt.x() && pos.y() > min_pt.y() && pos.z() > min_pt.z() &&
          pos.x() < max_pt.x() && pos.y() < max_pt.y() && pos.z() < max_pt.z()) {
        double side_length = it.getSize();
        double cell_vol = std::pow(side_length, 3);
        if (octree->isNodeOccupied(*it)) {
          occupied_space += cell_vol;
        } else {
          free_space += cell_vol;
        }
      }
    }

    double mapped_space = occupied_space + free_space;
    double unmapped_space = volume_ - mapped_space;
    double coverage_pct = 100.0 * mapped_space / volume_;

    // Tid sedan start (sekunder)
    double elapsed = (this->now() - start_time_).seconds();

    RCLCPP_INFO(this->get_logger(),
                "[t=%.1fs] Coverage: %.2f%% | Mapped: %.2f m3 | "
                "Free: %.2f m3 | Occupied: %.2f m3 | Unmapped: %.2f m3",
                elapsed, coverage_pct, mapped_space, free_space,
                occupied_space, unmapped_space);

    // Skriv till CSV
    {
      std::ofstream ofs(output_file_, std::ios::app);
      ofs << std::fixed << std::setprecision(4)
          << elapsed << ","
          << mapped_space << ","
          << coverage_pct << ","
          << free_space << ","
          << occupied_space << ","
          << unmapped_space << "\n";
    }
  }

  // Parametrar
  std::vector<double> min_vals_;
  std::vector<double> max_vals_;
  double volume_;
  std::string output_file_;
  std::string octomap_topic_;
  rclcpp::Time start_time_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CoverageStandalone>();
  RCLCPP_INFO(node->get_logger(), "Spinning... Ctrl+C to stop.");
  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
