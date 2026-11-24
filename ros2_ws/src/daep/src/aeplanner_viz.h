#ifndef _AEPVIZ_H_
#define _AEPVIZ_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <tuple>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include "param.h"

#include "aeplanner.h"

#include "data_structures.h"

#include <octomap_msgs/conversions.h>  // for binaryMapToMsg


class AEPlannerViz {
private:
  AEPlanner * aep_node;
  std::string world_frame_;

public:
  AEPlannerViz(AEPlanner * aep, std::string world_frame);
  ~AEPlannerViz();
  
  visualization_msgs::msg::MarkerArray createRRTMarkerArray(RRTNode * root, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time);
  void recurse(RRTNode * node, visualization_msgs::msg::MarkerArray * marker_array, int * id, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time);

  visualization_msgs::msg::Marker createNodeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns);
  visualization_msgs::msg::Marker createEdgeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns);
  visualization_msgs::msg::Marker createTextMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns);
  visualization_msgs::msg::Marker createTimeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time);

  void visualizePose(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub, std::vector<daep_msgs::msg::HumanUpdate> humans, std::string ns);
  
  void visualizePrediction(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub, 
                           const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                           const std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>>& all_ellipses,
                           std::string ns,
                           double dt = 0.1);
  void visualizeBestNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_node_pub, std::vector<Eigen::Vector4d> nodes);

  void visualizeGhostPedestrian(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ghost_marker_pub, std::vector<std::tuple<double, double, double>> positions);

  visualization_msgs::msg::Marker createRayMarker(int ray_id, std::string color);

  void VisualizeOldPath(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr old_path_pub, RRTNode* root);
  
  //Debug nodes

  void visualizeRejectedNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampled_node_pub, std::vector<Eigen::Vector4d> rejectedNodes, std::vector<int> rejectedReasons);

  void visualizeNearestNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nearest_node_pub, std::vector<Eigen::Vector4d> neighbors);

  void visualizeValidNodes(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr valid_nodes_pub, std::vector<Eigen::Vector4d> validNodes);

  // Same as line box
  void visualizeOctomapSlice(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr octomap_leafs_pub, octomap::point3d min, octomap::point3d max);

  void visualizeFilteredOctomap(rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_filtered_pub, std::shared_ptr<octomap::OcTree> octree);

  void visualizeCylinders(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinders_pub, Eigen::Vector4d start, Eigen::Vector4d end, double x_diameter1, double y_diameter1, double z_diameter1, double x_diameter2, double y_diameter2, double z_diameter2);

  void visualizeTunnel(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinders_pub, Eigen::Vector4d start, Eigen::Vector4d end, double radius);

};

#endif
