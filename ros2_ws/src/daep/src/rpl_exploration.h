#ifndef RPL_EXPLORATION_H
#define RPL_EXPLORATION_H

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "daep_msgs/msg/goal.hpp"
#include "daep_msgs/srv/coverage.hpp"
#include "daep_msgs/srv/safe_path.hpp"
#include "daep_msgs/action/rrt.hpp"
#include "daep_msgs/action/aeplanner.hpp"

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"

using namespace std::placeholders;  
using namespace std::chrono_literals;


class RplExploration : public rclcpp::Node {
private:
  using RrtAction = daep_msgs::action::Rrt;
  using GoalHandleRrtAction = rclcpp_action::ClientGoalHandle<RrtAction>;
  using AepAction = daep_msgs::action::Aeplanner;
  using GoalHandleAepAction = rclcpp_action::ClientGoalHandle<AepAction>;

  rclcpp::CallbackGroup::SharedPtr client_cb_group;
  rclcpp::CallbackGroup::SharedPtr action_cb_group;
  rclcpp::CallbackGroup::SharedPtr reentrant_cb_group;

  rclcpp::TimerBase::SharedPtr timer_;
  
  geometry_msgs::msg::PoseStamped current_pose_stamped;
  std_msgs::msg::Bool exploration_completed_msg;
  bool drone_freeze;
  bool vicon;
  bool goal_reached;
  bool prev_goal_reached;
  bool prev_avoidance;
  bool avoidance;
  bool have_pose;
  double path_length;
  geometry_msgs::msg::PoseStamped new_goal;
  geometry_msgs::msg::PoseStamped prev_goal;
  int max_avoidance_counter;
  int global_planner_counter;
  int look_ahead_horizon;

  bool aep_have_result;
  bool rrt_have_result;

  AepAction::Result aep_result;
  RrtAction::Result rrt_result;
  bool new_plan;

  int iteration_limit; // for debug
  double coverage_threshold; // percent; stop when coverage reaches it, 0 = off
  int coverage_stall_rounds; // stop when coverage grew less than coverage_stall_gain
  double coverage_stall_gain; // percentage points over this many rounds; 0 rounds = off

  bool is_docked_flag;
  bool have_dock_status;
  bool use_dock_station;
  std::string world_frame;
  bool husky;
  // Clients

  rclcpp::Client<daep_msgs::srv::Coverage>::SharedPtr coverage_client;  
  rclcpp::Client<daep_msgs::srv::SafePath>::SharedPtr safe_path_client;

  rclcpp_action::Client<daep_msgs::action::Rrt>::SharedPtr rrt_action_client;
  rclcpp_action::Client<daep_msgs::action::Aeplanner>::SharedPtr aeplanner_action_client;

  
  // Publishers

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_completion_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_init_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;
  rclcpp::Publisher<daep_msgs::msg::Goal>::SharedPtr goal_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clock_start_pub;

  // subscriptions

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gr_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr avoid_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr new_goal_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dji_pose_sub;
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_status_sub;

  void poseCallback(const geometry_msgs::msg::PoseStamped & msg);
  void poseCallbackDji(const geometry_msgs::msg::PoseStamped & msg);
  void goalReachedCallback(const std_msgs::msg::Bool & msg);
  void avoidanceCallback(const std_msgs::msg::Bool & msg);
  void newGoalCallback(const geometry_msgs::msg::PoseStamped & msg);
  void dock_status_callback(const irobot_create_msgs::msg::DockStatus & msg);

  void rrt_goal_response_callback(std::shared_future<GoalHandleRrtAction::SharedPtr> future);
  void rrt_feedback_callback(GoalHandleRrtAction::SharedPtr, const std::shared_ptr<const RrtAction::Feedback> feedback);
  void rrt_result_callback(const GoalHandleRrtAction::WrappedResult & result);
  
  void aep_goal_response_callback(std::shared_future<GoalHandleAepAction::SharedPtr> future);
  void aep_feedback_callback(GoalHandleAepAction::SharedPtr, const std::shared_ptr<const AepAction::Feedback> feedback);
  void aep_result_callback(const GoalHandleAepAction::WrappedResult & result);

  std::ofstream pathfile, logfile, coveragefile;

public:
  RplExploration();
  ~RplExploration();

  void start_exploration();
  void start_real_exploration();

  bool get_coverage(daep_msgs::srv::Coverage::Response & response);  
  std::shared_ptr<daep_msgs::srv::SafePath::Response> safe_path(nav_msgs::msg::Path path, std::vector<double> time_steps);

  void rrt_send_goal(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseArray goal_poses);
  void aep_send_goal(int actions_taken, int iteration);
  void publishGoalAndWait(int const& freq, daep_msgs::msg::Goal const& goal);
  void add_distance(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);
  double get_yaw(geometry_msgs::msg::Quaternion quat);
  geometry_msgs::msg::Quaternion get_quat(double yaw_rad);

  rclcpp::Duration move_to_pose(geometry_msgs::msg::Pose pose);
  void initial_motion(geometry_msgs::msg::Pose pose);
};



#endif
