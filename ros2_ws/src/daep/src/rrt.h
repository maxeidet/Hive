#ifndef _RRT_H_
#define _RRT_H_


#include <cmath>
#include <mutex>

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "daep_msgs/action/rrt.hpp"
#include "daep_msgs/srv/safe_path.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "daep_msgs/msg/human_list_update.hpp"
 
#include <tf2/LinearMath/Matrix3x3.h>
#include <octomap/OcTree.h>

#include "rrtnode.h"
#include "kdtree.h"
#include "rrt_kalman.h"

using namespace std::placeholders;  
using namespace std::chrono_literals;


class Rrt : public rclcpp::Node {
private:
  using RrtAction = daep_msgs::action::Rrt;
  using GoalHandleRrtAction = rclcpp_action::ServerGoalHandle<RrtAction>;

  std::shared_ptr<octomap::OcTree> ot_;
  double dt_;
  std::map<std::string, std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Twist>> dynamic_objects;
  int KFiterations_;
  double min_nodes_;
  double extension_range_;
  double bounding_radius_;
  std::vector<double> boundary_min_;
  std::vector<double> boundary_max_;
  double drone_linear_velocity;
  double drone_angular_velocity;
  std::string frame_id_;
  bool dynamic_mode_;
  std::mutex vecMutex;


  std::map<std::string, KalmanFilter1> kalman_filters_;
  double position_uncertainty_;
  double velocity_uncertainty_;
  double process_noise_;
  double measurement_noise_;
  int safety_iteration_;
  double z_offset;
  double human_height_;
  double x_diameter_;
  double y_diameter_;
  double z_diameter_;
  double location_tolerance_;
  bool debug_text_;
  bool visualize_predictions_;
  bool use_sphere_collision_checking_;
  rclcpp::Time last_msg_time_;
  std::map<std::string, int> uuid_to_human_id_map;
  int free_human_id;
  double human_width_;
  double flying_obstacle_diameter_;
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data;
  bool use_smoothing_;
  double smoothing_factor_;
  bool filter_out_ground_;
  double ground_filter_z_dist_;


  
  // Clients

  // publishers

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_star_pub;

  // subscriptions

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription;
  rclcpp::Subscription<daep_msgs::msg::HumanListUpdate>::SharedPtr human_update_subscription;

  // Services

  rclcpp::Service<daep_msgs::srv::SafePath>::SharedPtr safe_path_service;

  // Action

  rclcpp_action::GoalResponse handle_rrt_goal(const rclcpp_action::GoalUUID & uuid,
                                              std::shared_ptr<const RrtAction::Goal> goal);
  rclcpp_action::CancelResponse handle_rrt_cancel(const std::shared_ptr<GoalHandleRrtAction> goal_handle);
  void handle_rrt_accepted(const std::shared_ptr<GoalHandleRrtAction> goal_handle);
  void execute_rrt(const std::shared_ptr<GoalHandleRrtAction> goal_handle);
  rclcpp_action::Server<RrtAction>::SharedPtr action_server_rrt;

  void declare_params();
  void read_params();

public:  
  Rrt();
  ~Rrt();
  double get_yaw(geometry_msgs::msg::Quaternion quat);

  void octomapCallback(const octomap_msgs::msg::Octomap& msg);

  void service_safe_path(const std::shared_ptr<daep_msgs::srv::SafePath::Request> request,
                         std::shared_ptr<daep_msgs::srv::SafePath::Response> response);

  void updateHumanPositions(const daep_msgs::msg::HumanListUpdate& msg);
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories(double dt);

  bool isCollision(const geometry_msgs::msg::PoseStamped& posestamped_parent,
                   const geometry_msgs::msg::PoseStamped& posestamped,
                   double time_of_arrival,
                   std::vector<std::pair<std::tuple<std::vector<double>,
                   std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data_input);
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);

  bool checkCollision(double t, 
                      Eigen::Vector3d point, 
                      std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                      std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                      Eigen::Vector3d parent);
                          
  bool isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse);

  int getCovarianceIndex(double max_time_step, double time_step, double t);
  
  bool lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                            const Eigen::Vector3d center, double radius);

  Eigen::Vector3d sample();
  RrtNode *chooseParent(kdtree *kd_tree, Eigen::Vector3d, double l);
  std::pair<nav_msgs::msg::Path, std::vector<double>> getBestPath(std::vector<RrtNode*> goals);

  void visualizePath(RrtNode *node);
  void visualizeChosenFrontier(RrtNode* node);
  visualization_msgs::msg::Marker createNodeMarker(RrtNode *node, int id);
  void visualizePrediction(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub, 
                          const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                          const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses,
                          std::string ns                           
                                  );

  Eigen::Vector3d getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
  
  bool isInsideBoundaries(Eigen::Vector3d point);

  RrtNode *addNodeToTree(kdtree *kd_tree, RrtNode *parent, Eigen::Vector3d new_pos, Eigen::Vector3d dir);

  int collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r, bool use_sphere_collision_checking, double x_diameter, double y_diameter, double z_diameter, std::shared_ptr<octomap::OcTree> ot);
  
  void rewire(kdtree* kd_tree, RrtNode* new_node, double l, double r, geometry_msgs::msg::Point current_state);

  RrtNode* getGoal(kdtree* goal_tree, RrtNode* new_node, double r, geometry_msgs::msg::Point current_state);

  float CylTest_CapsFirst(const octomap::point3d &pt1,
                        const octomap::point3d &pt2,
                        float lsq, float rsq, const octomap::point3d &pt);

  bool isSmoothedUnknown(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing);
  bool isSmoothedOccupied(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing);

  std::shared_ptr<octomap::OcTree> filterGroundFromOctomap(std::shared_ptr<octomap::OcTree> octree, double ground_filter_z_dist, geometry_msgs::msg::Point current_state);

};

#endif  // _RRT_H_