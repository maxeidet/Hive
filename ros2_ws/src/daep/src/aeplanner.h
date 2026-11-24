#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "daep_msgs/msg/human_update.hpp"
#include "daep_msgs/msg/human_list_update.hpp"
#include "daep_msgs/msg/pigain_node.hpp"
#include "daep_msgs/msg/pigain_node_list.hpp"
#include "daep_msgs/srv/best_node.hpp"
#include "daep_msgs/srv/query_dfm.hpp"
#include "daep_msgs/srv/query.hpp"
#include "daep_msgs/srv/reevaluate.hpp"
#include "daep_msgs/action/aeplanner.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "param.h"
#include "kalman.h"
#include "kdtree.h"


#include <octomap/OcTree.h>

#include "data_structures.h"

// synch this shit
#include <mutex>

class AEPlannerViz;

class AEPlanner : public rclcpp::Node {
private:
  using Aeplanner = daep_msgs::action::Aeplanner;
  using GoalHandleAeplanner = rclcpp_action::ServerGoalHandle<Aeplanner>;


  rclcpp::CallbackGroup::SharedPtr client_cb_group;
  rclcpp::CallbackGroup::SharedPtr action_cb_group;

  AEPlannerViz * aep_viz;
  rclcpp::Time last_msg_time_;

  std::map<std::string, int> uuid_to_human_id_map;
  
  aeplanner::Params params_;

  bool current_state_initialized_{false};

  int free_human_id;
  Eigen::Vector4d current_state_;

  std::shared_ptr<octomap::OcTree> ot_;

  std::map<std::string, std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Twist>> dynamic_objects;
  std::mutex vecMutex;
  
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data;
  std::map<std::string, KalmanFilter> kalman_filters;
  std::map<std::string, rclcpp::Time> last_measurement_time_;  // Track last measurement time for each human

  std::vector<std::tuple<double, double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);
  
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories(double dt);

  // kd tree for finding nearest neighbours
  kdtree* kd_tree_;

  // Keep track of the best node and its score
  RRTNode* best_node_;
  RRTNode* best_branch_root_;

  bool dynamic_mode_;
  double dt_;

  bool first_iteration_{true};


  // Clients

  rclcpp::Client<daep_msgs::srv::QueryDFM>::SharedPtr query_dfm_client;  
  rclcpp::Client<daep_msgs::srv::Query>::SharedPtr nn_yaw_query_client;  
  rclcpp::Client<daep_msgs::srv::BestNode>::SharedPtr best_node_client;  

  
  // publishers

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr covariance_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr old_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ghost_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_rays_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_rays_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr human_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr daep_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr daep_marker_array_pub_;  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predictions_pub_;
  rclcpp::Publisher<daep_msgs::msg::PigainNodeList>::SharedPtr gain_pub_;
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampled_node_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nearest_node_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr octomap_leafs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr valid_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinders_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr max_volume_bound_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_filtered_pub_;
  
  
  // subscriptions

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dji_pose_subscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription;
  rclcpp::Subscription<daep_msgs::msg::HumanListUpdate>::SharedPtr human_update_subscription;

  // services

  rclcpp::Service<daep_msgs::srv::Reevaluate>::SharedPtr reevaluate_server_;  

  // Aeplanner
  rclcpp_action::GoalResponse handle_aeplanner_goal(const rclcpp_action::GoalUUID & uuid,
                                                    std::shared_ptr<const Aeplanner::Goal> goal);
  rclcpp_action::CancelResponse handle_aeplanner_cancel(const std::shared_ptr<GoalHandleAeplanner> goal_handle);
  void handle_aeplanner_accepted(const std::shared_ptr<GoalHandleAeplanner> goal_handle);
  void execute_aeplanner(const std::shared_ptr<GoalHandleAeplanner> goal_handle);
  rclcpp_action::Server<Aeplanner>::SharedPtr action_server_aeplanner;

public:
  AEPlanner();
  ~AEPlanner();

  double get_yaw(geometry_msgs::msg::Quaternion quat);
  void declare_params();
  void read_params();
  aeplanner::Params get_params();
  void pose_callback(const geometry_msgs::msg::PoseStamped & msg);
  void dji_pose_callback(const geometry_msgs::msg::PoseStamped & msg);
  void odometry_callback(const nav_msgs::msg::Odometry & msg);
  void octomapCallback(const octomap_msgs::msg::Octomap& msg);

  // ---------------- Initialization ----------------
  RRTNode* initialize();
  void initializeKDTreeWithPreviousBestBranch(RRTNode* root);

  void reevaluatePotentialInformationGainRecursive(RRTNode* node);
  
  std::tuple<double, double, double> gainCubature(Eigen::Vector4d state, double time_of_arrival);

  double computeVolumeEstimation(double r_min, double r_max, double dr, double hfov, double vfov, double dtheta, double dphi);

  double get_dfm_score(double x, double y, double z);
  double get_nn_yaw(double x, double y, double z);
  std::shared_ptr<daep_msgs::srv::BestNode::Response> get_best_node(double x, double y, double z, double threshold);

  int getCovarianceIndex(double max_time_step, double time_step, double t);

  bool willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts);

  bool isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z);

  bool isInsideBoundaries(Eigen::Vector4d point);

  bool pointOnXYBoundaries(Eigen::Vector4d point);

  std::pair<RRTNode*, bool> pathIsSafe(RRTNode* node, 
                                       std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                                       std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses);

  void expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses);
  
  Eigen::Vector4d sampleNewPoint(double max_sampling_radius);
  
  RRTNode* chooseParent(RRTNode* node, double l);
  
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  int collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r, bool use_sphere_collision_checking, double x_radius, double y_radius, double z_radius, std::shared_ptr<octomap::OcTree> ot);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);


  std::shared_ptr<octomap::OcTree> filterGroundFromOctomap(std::shared_ptr<octomap::OcTree> octree, double ground_filter_z_dist);

  bool isSmoothedUnknown(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing);

  bool isSmoothedOccupied(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing);

  bool checkCollision(double t, 
                      Eigen::Vector4d point, 
                      std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                      std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses);

  std::tuple<double, double, double> getGain(RRTNode* node, double time_of_arrival);

  void rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r);

  bool isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
                             std::tuple<double, double, double, Eigen::MatrixXd> covariance_ellipse);

  geometry_msgs::msg::PoseArray getFrontiers();

  void publishEvaluatedNodesRecursive(RRTNode* node);

  geometry_msgs::msg::Pose vecToPose(Eigen::Vector4d state);

  void reevaluate(const std::shared_ptr<daep_msgs::srv::Reevaluate::Request> request,
                  std::shared_ptr<daep_msgs::srv::Reevaluate::Response> response);

  void updateHumanPositions(const daep_msgs::msg::HumanListUpdate& msg);

  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> circlePredictTrajectories(double dt);

  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> linearPredictTrajectories(double dt);

  

};

//////////////////////#################################################### OLD CODE BELOW

#if 0

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <aeplanner/data_structures.h>
#include <aeplanner/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner/aeplannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <pigain/Node.h>
#include <pigain/Query.h>
#include <pigain/BestNode.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// DAEP
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <pigain/QueryDFM.h>
#include <pigain/Score.h>



namespace aeplanner
{
class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;


  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;

  // DAEP
  ros::Subscriber human_sub_;
  ros::Publisher human_marker_pub_;
  // Publishers

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceClient nn_yaw_query_client_;
  ros::ServiceClient dfm_client_;

  // DAEP

  //Global variables


  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();

  // Service server callback

  // ---------------- Expand RRT Tree ----------------



  // ---------------- Helpers ----------------
  //


  // ---------------- Frontier ----------------

public:
  AEPlanner(const ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  // DAEP

  //void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
  //void agentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);

};

}  // namespace aeplanner
#endif


#endif
