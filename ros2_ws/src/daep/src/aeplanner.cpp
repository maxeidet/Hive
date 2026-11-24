#include "aeplanner.h"
#include <cstring> // Lägg till denna rad för strlen

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

//#include "octomap_ros/conversions.hpp"
#include "octomap_msgs/conversions.h"
#include "aeplanner_viz.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

AEPlanner::AEPlanner() : Node("aeplanner"), current_state_initialized_(false), free_human_id(0),
                         ot_(0), best_node_(0), best_branch_root_(0),
                         dynamic_mode_(false), first_iteration_(true),
                         // Set from the dynamic obstacle messages; without them the time
                         // window loop in checkCollision would never end on dt_ = 0.
                         dt_(0.1)
 {

  declare_params();
  read_params();
  
  RCLCPP_ERROR(this->get_logger(), "AEPlanner STARTING...");     

  client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  aep_viz = new AEPlannerViz(this, params_.world_frame);
  last_msg_time_ = this->get_clock()->now();

  // publishers

  daep_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("daep_marker_visualization", 100);
  daep_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("daep_marker_array_visualization", 100);
  predictions_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("predictions", 100);
  
  human_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("human_poses", 100);
  rrt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrtree", 100);
  best_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("best_node", 100);
  pred_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("predicted_trajectory", 100);
  covariance_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cov_ellipse", 100);
  old_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("old_path", 100);
  ghost_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ghosts", 100);
  static_rays_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_rays", 100);
  dynamic_rays_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_rays", 100);
  gain_pub_ = this->create_publisher<daep_msgs::msg::PigainNodeList>("gain_node", 100);

  sampled_node_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sampled_node", 100);
  nearest_node_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sampled_node", 100);
  valid_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sampled_node", 100);
  octomap_leafs_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("bounds_visualization", 100);
  cylinders_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sampled_node", 100);

  // Latched (transient local): sent once at start-up, and pig.py may subscribe after that.
  max_volume_bound_pub_ = this->create_publisher<std_msgs::msg::Float64>("max_volume_bound", rclcpp::QoS(1).transient_local());
  octomap_filtered_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("filtered_octomap", 10);

  // services

  query_dfm_client = create_client<daep_msgs::srv::QueryDFM>("dfm_query_server",
                                                             rmw_qos_profile_services_default,
                                                             client_cb_group);

  nn_yaw_query_client = create_client<daep_msgs::srv::Query>("nn_yaw_query_server",
                                                             rmw_qos_profile_services_default,
                                                             client_cb_group);

  best_node_client = create_client<daep_msgs::srv::BestNode>("best_node_server",
                                                             rmw_qos_profile_services_default,
                                                             client_cb_group);
  
  // subscriptions

  dji_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", qos,
                                                                                std::bind(&AEPlanner::dji_pose_callback,
                                                                                      this, std::placeholders::_1));

  pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("simple_pose", qos,
                                                                                 std::bind(&AEPlanner::pose_callback,
                                                                                           this, std::placeholders::_1));
  odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>("sim_ground_truth_pose", qos,
                                                                             std::bind(&AEPlanner::odometry_callback,
                                                                                       this, std::placeholders::_1));
  octomap_subscription = this->create_subscription<octomap_msgs::msg::Octomap>("octomap", qos,
                                                                               std::bind(&AEPlanner::octomapCallback,
                                                                                         this, std::placeholders::_1));
  human_update_subscription = this->create_subscription<daep_msgs::msg::HumanListUpdate>("dynamic_objects", 100,
                                                                                     std::bind(&AEPlanner::updateHumanPositions,
                                                                                               this, std::placeholders::_1));

  
  //reevaluate_server_ = this->create_service<daep_msgs::srv::Reevaluate>("reevaluate", std::bind(&AEPlanner::reevaluate, this, _1, _2));
  
  // Action Servers
  
  this->action_server_aeplanner = rclcpp_action::create_server<Aeplanner>(
      this,
      "make_plan",
      std::bind(&AEPlanner::handle_aeplanner_goal, this, _1, _2),
      std::bind(&AEPlanner::handle_aeplanner_cancel, this, _1),
      std::bind(&AEPlanner::handle_aeplanner_accepted, this, _1));  

  // Wait for pig.py to subscribe to max_volume_bound topic before publishing
  RCLCPP_INFO(this->get_logger(), "Waiting for pig.py to subscribe to max_volume_bound topic...");
  auto start_time = this->get_clock()->now();
  auto timeout = rclcpp::Duration::from_seconds(10.0);
  bool subscriber_found = false;
  
  while (rclcpp::ok() && (this->get_clock()->now() - start_time) < timeout) {
    if (max_volume_bound_pub_->get_subscription_count() > 0) {
      subscriber_found = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Update pig.py with max_volume_bound 
  auto msg = std_msgs::msg::Float64();
  msg.data = params_.max_volume;
  max_volume_bound_pub_->publish(msg);
  
  if (subscriber_found) {
    RCLCPP_INFO(this->get_logger(), "Sent max_volume_bound (%f) to pig.py", params_.max_volume);
  } else {
    RCLCPP_WARN(this->get_logger(), "No subscriber found for max_volume_bound topic after 10 seconds, published anyway (pig.py may subscribe later)");
  }

  RCLCPP_ERROR(this->get_logger(), "AEPlanner WAITING FOR RPL_EXPLORATION");     

}

AEPlanner::~AEPlanner() {

}

double AEPlanner::get_yaw(geometry_msgs::msg::Quaternion quat) {
  tf2::Quaternion current_quat;
  current_quat[0] = quat.x;
  current_quat[1] = quat.y;
  current_quat[2] = quat.z;
  current_quat[3] = quat.w;
  double roll, pitch, yaw;
  tf2::Matrix3x3(current_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

void AEPlanner::declare_params() {
  this->declare_parameter("camera/horizontal_fov", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("camera/vertical_fov", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("resolution", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raycast/dr", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raycast/dphi", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("raycast/dtheta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/gain/lambda", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/tree/extension_range", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/tree/max_sampling_radius", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/gain/sigma_thresh", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/tree/initial_iterations", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("aep/gain/r_max", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/gain/inside_boundary", false);
  this->declare_parameter("aep/gain/r_min", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("boundary/min", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("boundary/max", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("system/bbx/r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("aep/tree/cutoff_iterations", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("aep/gain/zero", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("world_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("robot_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("visualize_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("visualize_exploration_area", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("daep/rrt/max_sampled_initial_nodes", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("daep/rrt/max_sampled_nodes", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("daep/gain/cache_node_threshold", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("daep/gain/node_gain_threshold", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("daep/dfm/zeta", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/iterations", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("daep/kf/time_step", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/position_uncertainty", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/velocity_uncertainty", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/process_noise", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/measurement_noise", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/safety_iteration", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("visualize_rays", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("visualize_tree", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("visualize_static_and_dynamic_rays", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("human_height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("human_width", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("human_linear_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("human_angular_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_width", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_linear_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_angular_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("boosted_boundary_length", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("boost_magnitude", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_freeze", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("global_planner_counter", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("look_ahead_horizon", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("z_offset", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/x", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/y", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/z", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/use_sphere_collision_checking", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("debug_text", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("add_rejected_nodes", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("flying_obstacle_diameter", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("visualize_predictions", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("add_to_frontier_ratio", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("reeval_ratio", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("zero_gain_ratio", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("filter_out_ground", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("ground_filter_z_dist", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("viz_filtered_octomap", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("use_smoothing", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("rotating_while_walking", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("smoothing_factor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("best_branch_threshold", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("ignore_first_iteration", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("husky", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("time_window", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("prediction_mode", rclcpp::PARAMETER_STRING);
  this->declare_parameter("human_measurement_timeout", rclcpp::PARAMETER_DOUBLE);

}



void AEPlanner::read_params() {
  rclcpp::Parameter param;
  if (this->get_parameter("camera/horizontal_fov", param)) {
    params_.hfov = param.as_double();
  } else {
    params_.hfov = 60.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No horizontal fov specified. Default: " << params_.hfov);    
  }

  if (this->get_parameter("camera/vertical_fov", param)) {
    params_.vfov = param.as_double();
  } else {
    params_.vfov = 45.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No vertical fov specified. Default: " << params_.vfov);    
  }

  if (this->get_parameter("resolution", param)) 
  {
    params_.dr = param.as_double();
  } 
  else 
  {
  
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not read octomap resolution. Looking for resolution");
    RCLCPP_WARN_STREAM(this->get_logger(), "Using resolution specified by raycast/dr instead");
  
    if (this->get_parameter("raycast/dr", param)) 
    {
      params_.dr = param.as_double();
    } 
    else 
    {
      params_.dr = 0.2;
      RCLCPP_WARN_STREAM(this->get_logger(), "No dr specified. Default: " << params_.dr);
    }
  }

  if (this->get_parameter("raycast/dphi", param)) {
    params_.dphi = param.as_double();
  } else {
    params_.dphi = 10.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No dphi specified. Default: " << params_.dphi);    
  }

  if (this->get_parameter("raycast/dtheta", param)) {
    params_.dtheta = param.as_double();
  } else {
    params_.dtheta = 10.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No dtheta specified. Default: " << params_.dtheta);    
  }

  if (this->get_parameter("aep/gain/lambda", param)) {
    params_.lambda = param.as_double();
  } else {
    params_.lambda = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No lambda specified. Default: " << params_.lambda);    
  }

  if (this->get_parameter("aep/tree/extension_range", param)) {
    params_.extension_range = param.as_double();
  } else {
    params_.extension_range = 1.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No extension_range specified. Default: " << params_.extension_range);    
  }

  if (this->get_parameter("aep/tree/max_sampling_radius", param)) {
    params_.max_sampling_radius = param.as_double();
  } else {
    params_.max_sampling_radius = 10.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No max_sampling_radius specified. Default: " << params_.max_sampling_radius);    
  }

  if (this->get_parameter("aep/gain/sigma_thresh", param)) {
    params_.sigma_thresh = param.as_double();
  } else {
    params_.sigma_thresh = 0.2;
    RCLCPP_WARN_STREAM(this->get_logger(), "No sigma_thresh specified. Default: " << params_.sigma_thresh);    
  }

  if (this->get_parameter("aep/tree/initial_iterations", param)) {
    params_.init_iterations = param.as_int();
  } else {
    params_.init_iterations = 15;
    RCLCPP_WARN_STREAM(this->get_logger(), "No init_iterations specified. Default: " << params_.init_iterations);    
  }

  params_.gain_inside_boundary = this->get_parameter("aep/gain/inside_boundary").as_bool();
  if (this->get_parameter("aep/gain/r_max", param)) {
    params_.r_max = param.as_double();
  } else {
    params_.r_max = 4.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No r_max specified. Default: " << params_.r_max);
  }
  if (this->get_parameter("aep/gain/r_min", param)) {
    params_.r_min = param.as_double();
  } else {
    params_.r_min = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No r_min specified. Default: " << params_.r_min);
  }
  if (this->get_parameter("boundary/min", param)) {
    params_.boundary_min = param.as_double_array();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No boundary_min specified.");
  }
  if (this->get_parameter("boundary/max", param)) {
    params_.boundary_max = param.as_double_array();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No boundary_max specified.");
  }

  if (this->get_parameter("system/bbx/r", param)) {
    params_.bounding_radius = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No /system/bbx/r specified.");
  }

  if (this->get_parameter("system/bbx/x", param)) {
    params_.x_diameter = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/x specified.");
  }

  if (this->get_parameter("system/bbx/y", param)) {
    params_.y_diameter = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/y specified.");
  }

  if (this->get_parameter("system/bbx/z", param)) {
    params_.z_diameter = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No /system/bbx/z specified.");
  }

  if (this->get_parameter("system/bbx/use_sphere_collision_checking", param)) {
    params_.sphere_bb = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/use_sphere_collision_checking specified.");
  }

  if (this->get_parameter("debug_text", param)) {
    params_.debug_text = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No debug_text specified.");
  }

  if (this->get_parameter("add_rejected_nodes", param)) {
    params_.add_rejected_nodes = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No add_rejected_nodes specified.");
  }

  if (this->get_parameter("visualize_predictions", param)) {
    params_.visualize_predictions = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No visualize_predictions specified.");
  }

  if (this->get_parameter("aep/tree/cutoff_iterations", param)) {
    params_.cutoff_iterations = param.as_int();
  } else {
    params_.cutoff_iterations = 200;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /aep/tree/cutoff_iterations specified. Default: " << params_.cutoff_iterations);
  }
  if (this->get_parameter("aep/gain/zero", param)) {
    params_.zero_gain = param.as_double();
  } else {
    params_.zero_gain = 0.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /aep/gain/zero specified. Default: " << params_.zero_gain);
  }
  if (this->get_parameter("world_frame", param)) {
    params_.world_frame = param.as_string();
  } else {
    params_.world_frame = "world"; 
    RCLCPP_WARN_STREAM(this->get_logger(), "No /world_frame specified. Default: " << params_.world_frame);   
  }
  if (this->get_parameter("robot_frame", param)) {
    params_.robot_frame = param.as_string();
  } else {
    params_.robot_frame = "base_link";
    RCLCPP_WARN_STREAM(this->get_logger(), "No /robot_frame specified. Default: " << params_.robot_frame);
  }
  if (this->get_parameter("visualize_tree", param)) {
    params_.visualize_tree = param.as_bool();
  } else {
    params_.visualize_tree = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /visualize_tree specified. Default: " << params_.visualize_tree);
  }
  if (this->get_parameter("visualize_rays", param)) {
    params_.visualize_rays = param.as_bool();
  } else {
    params_.visualize_rays = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /visualize_rays specified. Default: " << params_.visualize_rays);
  }
  if (this->get_parameter("visualize_exploration_area", param)) {
    params_.visualize_exploration_area = param.as_bool();
  } else {
    params_.visualize_exploration_area = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /visualize_exploration_area specified. Default: " << params_.visualize_exploration_area);
  }
  if (this->get_parameter("daep/rrt/max_sampled_initial_nodes", param)) {
    params_.max_sampled_initial_nodes = param.as_int();
  } else {
    params_.max_sampled_initial_nodes = 30;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /daep/rrt/max_sampled_initial_nodes specified. Default: " << params_.max_sampled_initial_nodes);
  }
  if (this->get_parameter("daep/rrt/max_sampled_nodes", param)) {
    params_.max_sampled_nodes = param.as_int();
  } else {
    params_.max_sampled_nodes = 50;
    RCLCPP_WARN_STREAM(this->get_logger(), "No /daep/rrt/max_sampled_nodes specified. Default: " << params_.max_sampled_nodes);
  }
  if (this->get_parameter("daep/gain/cache_node_threshold", param)) {
    params_.cache_node_threshold = param.as_int();
  } else {
    params_.cache_node_threshold = 30;
    RCLCPP_WARN_STREAM(this->get_logger(), "No cache_node_threshold specified. Default: " << params_.cache_node_threshold);
  }
  if (this->get_parameter("daep/gain/node_gain_threshold", param)) {
    params_.node_gain_threshold = param.as_int();
  } else {
    params_.node_gain_threshold = 10;
    RCLCPP_WARN_STREAM(this->get_logger(), "No node_gain_threshold specified. Default: " << params_.node_gain_threshold);
  }
  if (this->get_parameter("daep/dfm/zeta", param)) {
    params_.zeta = param.as_double();
  } else {
    params_.zeta = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No zeta specified. Default: " << params_.zeta);
  }
  if (this->get_parameter("daep/kf/iterations", param)) {
    params_.KFiterations = param.as_int();
  } else {
    params_.KFiterations = 20;
    RCLCPP_WARN_STREAM(this->get_logger(), "No KFiterations specified. Default: " << params_.KFiterations);
  }
  if (this->get_parameter("daep/kf/time_step", param)) {
    params_.time_step = param.as_double();
  } else {
    params_.time_step = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No time_step specified. Default: " << params_.time_step);
  }

  if (this->get_parameter("daep/kf/position_uncertainty", param)) {
    params_.position_uncertainty = param.as_double();
  } else {
    params_.position_uncertainty = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No position_uncertainty specified. Default: " << params_.position_uncertainty);
  }
  if (this->get_parameter("daep/kf/velocity_uncertainty", param)) {
    params_.velocity_uncertainty = param.as_double();
  } else {
    params_.velocity_uncertainty = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No velocity_uncertainty specified. Default: " << params_.velocity_uncertainty);
  }

  if (this->get_parameter("daep/kf/process_noise", param)) {
    params_.process_noise = param.as_double();
  } else {
    params_.process_noise = 0.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No process_noise specified. Default: " << params_.process_noise);
  }

  if (this->get_parameter("daep/kf/measurement_noise", param)) {
    params_.measurement_noise = param.as_double();
  } else {
    params_.measurement_noise = 0.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No measurement_noise specified. Default: " << params_.measurement_noise);
  }

  if (this->get_parameter("daep/kf/safety_iteration", param)) {
    params_.safety_iteration = param.as_int();
  } else {
    params_.safety_iteration = 1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No safety_iteration specified. Default: " << params_.safety_iteration);
  }

  if (this->get_parameter("visualize_static_and_dynamic_rays", param)) {
    params_.visualize_static_and_dynamic_rays = param.as_bool();
  } else {
    params_.visualize_static_and_dynamic_rays = true;
    RCLCPP_WARN_STREAM(this->get_logger(), "No visualize_static_and_dynamic_rays specified. Default: " << params_.visualize_static_and_dynamic_rays);
  }
  if (this->get_parameter("human_height", param)) {
    params_.human_height = param.as_double();
  } else {
    params_.human_height = 1.8;
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_height specified. Default: " << params_.human_height);
  }
  if (this->get_parameter("human_width", param)) {
    params_.human_width = param.as_double();
  } else {
    params_.human_width = 1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_width specified. Default: " << params_.human_width);
  }
  if (this->get_parameter("human_linear_velocity", param)) {
    params_.human_linear_velocity = param.as_double();
  } else {
    params_.human_linear_velocity = 0.35;
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_linear_velocity specified. Default: " << params_.human_linear_velocity);
  }
  if (this->get_parameter("human_angular_velocity", param)) {
    params_.human_angular_velocity = param.as_double();
  } else {
    params_.human_angular_velocity = 1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_angular_velocity specified. Default: " << params_.human_angular_velocity);
  }
  if (this->get_parameter("drone_height", param)) {
    params_.drone_height = param.as_double();
  } else {
    params_.drone_height = 0.4;
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_height specified. Default: " << params_.drone_height);
  }
  if (this->get_parameter("drone_width", param)) {
    params_.drone_width = param.as_double();
  } else {
    params_.drone_width = 0.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_width specified. Default: " << params_.drone_width);
  }
  if (this->get_parameter("drone_linear_velocity", param)) {
    params_.drone_linear_velocity = param.as_double();
  } else {
    params_.drone_linear_velocity = 0.3;
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_linear_velocity specified. Default: " << params_.drone_linear_velocity);
  }
  if (this->get_parameter("drone_angular_velocity", param)) {
    params_.drone_angular_velocity = param.as_double();
  } else {
    params_.drone_angular_velocity = 0.35;
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_angular_velocity specified. Default: " << params_.drone_angular_velocity);
  }
  if (this->get_parameter("boosted_boundary_length", param)) {
    params_.boosted_boundary_length = param.as_double();
  } else {
    params_.boosted_boundary_length = 1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No boosted_boundary_length specified. Default: " << params_.boosted_boundary_length);
  }
  if (this->get_parameter("boost_magnitude", param)) {
    params_.boost_magnitude = param.as_double();
  } else {
    params_.boost_magnitude = 5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No boost_magnitude specified. Default: " << params_.boost_magnitude);
  }
  if (this->get_parameter("drone_freeze", param)) {
    params_.drone_freeze = param.as_bool();
  } else {
    params_.drone_freeze = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_freeze specified. Default: " << params_.drone_freeze);
  }
  if (this->get_parameter("global_planner_counter", param)) {
    params_.global_planner_counter = param.as_int();
  } else {
    params_.global_planner_counter = 25;
    RCLCPP_WARN_STREAM(this->get_logger(), "No global_planner_counter specified. Default: " << params_.global_planner_counter);
  }
  if (this->get_parameter("look_ahead_horizon", param)) {
    params_.look_ahead_horizon = param.as_int();
  } else {
    params_.look_ahead_horizon = 5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No look_ahead_horizon specified. Default: " << params_.look_ahead_horizon);
  }

  if (this->get_parameter("z_offset", param)) {
    params_.z_offset = param.as_double();
  } else {
    params_.z_offset = 0.025;
    RCLCPP_WARN_STREAM(this->get_logger(), "No z_offset specified. Default: " << params_.z_offset);
  }

  if (this->get_parameter("flying_obstacle_diameter", param)) {
    params_.flying_obstacle_diameter = param.as_double();
  } else {
    params_.flying_obstacle_diameter = 1.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No flying_obstacle_diameter specified. Default: " << params_.flying_obstacle_diameter);
  }

  if(this->get_parameter("add_to_frontier_ratio", param))
  {
    params_.add_to_frontier_ratio = param.as_double();
  }
  else
  {
    params_.add_to_frontier_ratio = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No add_to_frontier_ratio specified. Default: " << params_.add_to_frontier_ratio);
  }

  if(this->get_parameter("reeval_ratio", param))
  {
    params_.reeval_ratio = param.as_double();
  }
  else
  {
    params_.reeval_ratio = 0.7;
    RCLCPP_WARN_STREAM(this->get_logger(), "No reeval_ratio specified. Default: " << params_.reeval_ratio);
  }

  if(this->get_parameter("zero_gain_ratio", param))
  {
    params_.zero_gain_ratio = param.as_double();
  }
  else
  {
    params_.zero_gain_ratio = 0.9;
    RCLCPP_WARN_STREAM(this->get_logger(), "No zero_gain_ratio specified. Default: " << params_.zero_gain_ratio);
  }

  if(this->get_parameter("filter_out_ground", param))
  {
    params_.filter_out_ground = param.as_bool();
  }
  else
  {
    params_.filter_out_ground = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No filter_out_ground specified. Default: " << params_.filter_out_ground);
  }
  
  if(this->get_parameter("ground_filter_z_dist", param))
  {
    params_.ground_filter_z_dist = param.as_double();
  }
  else
  {
    params_.ground_filter_z_dist = 0.31; //m (Spot Max Step Height 0.3m)
    RCLCPP_WARN_STREAM(this->get_logger(), "No ground_filter_z_dist specified. Default: " << params_.ground_filter_z_dist);
  }

  if(this->get_parameter("viz_filtered_octomap", param))
  {
    params_.viz_filtered_octomap = param.as_bool();
  }
  else
  {
    params_.viz_filtered_octomap = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No viz_filtered_octomap specified. Default: " << params_.viz_filtered_octomap);
  }

  if(this->get_parameter("use_smoothing", param))
  {
    params_.use_smoothing = param.as_bool();
  }
  else
  {
    params_.use_smoothing = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No use_smoothing specified. Default: " << params_.use_smoothing);
  }

  if(this->get_parameter("smoothing_factor", param))
  {
    params_.smoothing_factor = param.as_double();
  }
  else
  {
    params_.smoothing_factor = 0.9;
    RCLCPP_WARN_STREAM(this->get_logger(), "No smoothing_factor specified. Default: " << params_.smoothing_factor);
  }

  if(this->get_parameter("rotating_while_walking", param))
  {
    params_.rotating_while_walking = param.as_bool();
  }
  else
  {
    params_.rotating_while_walking = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No rotating_while_walking specified. Default: " << params_.rotating_while_walking);
  }

  if(this->get_parameter("best_branch_threshold", param))
  {
    params_.best_branch_threshold = param.as_double();
  }
  else
  {
    params_.best_branch_threshold = 1.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No best_branch_threshold specified. Default: " << params_.best_branch_threshold);
  }

  if(this->get_parameter("ignore_first_iteration", param))
  {
    params_.ignore_first_iteration = param.as_bool();
  }
  else
  {
    params_.ignore_first_iteration = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No ignore_first_iteration specified. Default: " << params_.ignore_first_iteration);
  }

  if(this->get_parameter("husky", param))
  {
    params_.husky = param.as_bool();
  }
  else
  {
    params_.husky = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "Husky-mode not specified. Default: " << params_.husky);
  }

  if(this->get_parameter("time_window", param))
  {
    params_.time_window = param.as_double();
  }
  else
  {
    params_.time_window = 3.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No time_window specified. Default: " << params_.time_window);
  }

  if(this->get_parameter("prediction_mode", param))
  {
    params_.prediction_mode = param.as_string();
  }
  else
  {
    params_.prediction_mode = "kalman";
    RCLCPP_WARN_STREAM(this->get_logger(), "No prediction_mode specified. Default: " << params_.prediction_mode);
  }

  if (this->get_parameter("human_measurement_timeout", param)) {
    params_.human_measurement_timeout = param.as_double();
  } else {
    params_.human_measurement_timeout = 5.0;  // Default: 5 seconds
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_measurement_timeout specified. Default: " << params_.human_measurement_timeout);
  }

  RCLCPP_ERROR(this->get_logger(), "BOUNDARY MIN: %f %f %f", params_.boundary_min[0], params_.boundary_min[1], params_.boundary_min[2]);
  RCLCPP_ERROR(this->get_logger(), "BOUNDARY MAX: %f %f %f", params_.boundary_max[0], params_.boundary_max[1], params_.boundary_max[2]);

  // Compute maximum volumetric bound, used for adding and reevaluating nodes
  double max_volume = computeVolumeEstimation(params_.r_min, params_.r_max, params_.dr, params_.hfov, params_.vfov, params_.dtheta, params_.dphi);

  RCLCPP_INFO_STREAM(this->get_logger(), "Maximum volumetric bound for gain calculation: " << max_volume);

  params_.max_volume = max_volume;
  params_.add_node_to_frontier_threshold = max_volume * params_.add_to_frontier_ratio; 
  params_.reevaluate_node_threshold = max_volume * params_.reeval_ratio;  

  RCLCPP_INFO_STREAM(this->get_logger(), "Node addition threshold: " << params_.add_node_to_frontier_threshold << " Ratio: " << params_.add_to_frontier_ratio);
  RCLCPP_INFO_STREAM(this->get_logger(), "Node reevaluation threshold: " << params_.reevaluate_node_threshold << " Ratio: " << params_.reeval_ratio);
  RCLCPP_INFO_STREAM(this->get_logger(), "Zero gain threshold: " << params_.max_volume * params_.zero_gain_ratio << " Ratio: " << params_.zero_gain_ratio);

}

aeplanner::Params AEPlanner::get_params() {
  return params_;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float AEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1,
                                   const octomap::point3d& pt2, float lsq, float rsq,
                                   const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}

double AEPlanner::computeVolumeEstimation(double r_min, double r_max, double dr, double hfov, double vfov, double dtheta, double dphi)
{
  
  double total_volume = 0.0;

  // convert steps to radians
  double dphi_rad   = M_PI * dphi / 180.0;
  double dtheta_rad = M_PI * dtheta / 180.0;

  // Loop over theta (yaw) inside the horizontal field of view
  for (double theta = -hfov / 2.0; theta < hfov / 2.0; theta += dtheta)
  {
      // Loop over phi (pitch) inside the vertical field of view
      for (double phi = (90.0 - vfov / 2.0); phi < (90.0 + vfov / 2.0); phi += dphi)
      {
          double phi_rad = M_PI * phi / 180.0;

          // Integrate along radius
          for (double r = r_min; r < r_max; r += dr)
          {
              // same volume element formula as in your gainCubature
              double current_gain =
                  (2.0 * r * r * dr + (1.0 / 6.0) * dr * dr * dr) *
                  dtheta_rad * std::sin(phi_rad) * std::sin(dphi_rad / 2.0);

              total_volume += current_gain;
          }
      }
  }

  return total_volume;
}


RRTNode* AEPlanner::initialize()
{
  // Initialize kd-tree
  /* create a kd-tree for 3-dimensional data */
  kd_tree_ = kd_create(3);
  best_node_ = NULL;
  RRTNode* root = NULL;
  if(best_branch_root_){
    RCLCPP_INFO(this->get_logger(), "Previous best branch exists, reevaluate it...");
    reevaluatePotentialInformationGainRecursive(best_branch_root_);
    best_branch_root_ = best_branch_root_->children_[0];
    aep_viz->VisualizeOldPath(old_path_marker_pub_, best_branch_root_);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "best branch root : (" << best_branch_root_->state_(0) << ", " << best_branch_root_->state_(1) << ", " << best_branch_root_->state_(2) << ", " << best_branch_root_->state_(3) << ")");

    if(best_branch_root_->children_.size() > 0){
        best_branch_root_->parent_ = NULL;
    }
    else{
      delete best_branch_root_;
      best_branch_root_ = NULL;
    }
  }
  

  // Initialize without any previous branch
  root = new RRTNode(params_.drone_angular_velocity, params_.drone_linear_velocity);
  root->state_[0] = current_state_[0];
  root->state_[1] = current_state_[1];
  root->state_[2] = current_state_[2];
  RCLCPP_DEBUG_STREAM(this->get_logger(), "kd_insert3 x: " << current_state_[0] << " y: "<< current_state_[1] << " z: "<< current_state_[2] << " yaw: "<< current_state_[3]);
  kd_insert3(kd_tree_, root->state_[0], root->state_[1], root->state_[2], root);
  return root;
}

double AEPlanner::get_dfm_score(double x, double y, double z) {
  double score = -1.0;
  
  //RCLCPP_INFO(this->get_logger(), "get_dfm_score");

  while (!query_dfm_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return score;
    }
    RCLCPP_INFO(this->get_logger(), "dfm service not available, waiting again...");
  }

  auto req = std::make_shared<daep_msgs::srv::QueryDFM::Request>();
  req->point.x = x;
  req->point.y = y;
  req->point.z =  z;

  auto result_future = query_dfm_client->async_send_request(req);
  
  std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  
  if (status == std::future_status::ready) {
    //RCLCPP_INFO(this->get_logger(), "Received response get_dfm_score");
    std::shared_ptr<daep_msgs::srv::QueryDFM::Response> response = result_future.get();
    score = response->score;
    //RCLCPP_INFO(this->get_logger(), "Received get_dfm response result: %f", score);
  } else {
    RCLCPP_INFO(this->get_logger(), "Something went wrong with service call in get_dfm_score");
  }
  return score;
}


/*
This function will call pig.py to get an estimated yaw angle for a given (x,y,z) position.
This is used to estimate the future time for a node, and thus check if its going to be in dynamic collision.
If it is not gonna be in collision, we use gainCubature to get the actual gain and yaw angle.
*/
double AEPlanner::get_nn_yaw(double x, double y, double z) {

  while (!nn_yaw_query_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0.0;
    }
    RCLCPP_INFO(this->get_logger(), "nn_yaw service not available, waiting again...");
  }

  auto req = std::make_shared<daep_msgs::srv::Query::Request>();
  req->point.x = x;
  req->point.y = y;
  req->point.z = z;

  auto result_future = nn_yaw_query_client->async_send_request(req);
  
  std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  
  if (status == std::future_status::ready) {
    std::shared_ptr<daep_msgs::srv::Query::Response> response = result_future.get();
    return response->yaw;
  } else {
    RCLCPP_INFO(this->get_logger(), "Something went wrong with service call in get_nn_yaw");
  }
  return 0.0;
}

std::shared_ptr<daep_msgs::srv::BestNode::Response> AEPlanner::get_best_node(double x, double y, double z, double threshold) {
  
  RCLCPP_INFO(this->get_logger(), "Get Frontiers from pig.py...");

  while (!best_node_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "get_best_node service not available, waiting again...");
  }

  auto req = std::make_shared<daep_msgs::srv::BestNode::Request>();
  req->pos.x = x;
  req->pos.y = y;
  req->pos.z = z;
  req->threshold = threshold;

  auto result_future = best_node_client->async_send_request(req);
  
  std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  
  if (status == std::future_status::ready) {
    std::shared_ptr<daep_msgs::srv::BestNode::Response> response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Frontier with highest static gain is: %f", response->gain);
    return response;
  } else {
    RCLCPP_INFO(this->get_logger(), "Something went wrong with service call in get_best_node");
  }
  return 0;
}


/*
On every iteration, this function reevalutes the potential information gain explicplity in
the previous best branch (from the previous planning iteration). Note that it only does this in the tree, not on frontiers. 
*/
void AEPlanner::reevaluatePotentialInformationGainRecursive(RRTNode* node)
{

  std::tuple<double, double, double> ret = gainCubature(node->state_, node->time_cost(current_state_, params_.rotating_while_walking));
  node->gain_ = std::get<0>(ret); // Assign static gain
  node->dynamic_gain_ = std::get<1>(ret); //Assign dynamic gain
  node->state_[3] = std::get<2>(ret); // Assign yaw angle that maximizes static gain

  //Acquire DFM score
  double score = get_dfm_score(node->state_[0], node->state_[1], 0.0);  
  node->dfm_score_ = score;
  
  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it) {
    reevaluatePotentialInformationGainRecursive(*node_it);
  }
}

bool AEPlanner::pointOnXYBoundaries(Eigen::Vector4d point)
{ 
  if(point[0] < params_.boundary_min[0] + params_.boosted_boundary_length){
    return true;
  }
  if(point[0] > params_.boundary_max[0] - params_.boosted_boundary_length){
    return true;
  }
  if(point[1] < params_.boundary_min[1] + params_.boosted_boundary_length){
    return true;
  }
  if(point[1] > params_.boundary_max[1] - params_.boosted_boundary_length){
    return true;
  }
  return false;
}


bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{  
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

std::shared_ptr<octomap::OcTree> AEPlanner::filterGroundFromOctomap(std::shared_ptr<octomap::OcTree> octree, double ground_filter_z_dist)
{
  if (!current_state_initialized_) {
        RCLCPP_WARN(this->get_logger(), 
                    "Current robot state not initialized, returning original octree");
        return octree;
    }

    double robot_z = current_state_[2];
    double ground_cutoff = robot_z - ground_filter_z_dist;

    if(params_.debug_text)
      RCLCPP_INFO(this->get_logger(), "Robot z: %f, Ground cutoff: %f", robot_z, ground_cutoff);

    // Make a deep copy of the input octree
    std::shared_ptr<octomap::OcTree> filtered_octree =
        std::make_shared<octomap::OcTree>(*octree);

    // Collect leaf nodes to mark as free space
    // Store both keys and coordinates for more reliable updates
    struct NodeToFilter {
        octomap::OcTreeKey key;
        octomap::point3d coord;
        bool is_occupied;
    };
    std::vector<NodeToFilter> nodes_to_filter;

    int total_leafs = 0;
    int ground_leafs = 0;
    
    for (auto it = filtered_octree->begin_leafs(), end = filtered_octree->end_leafs();
         it != end; ++it)
    {
        total_leafs++;
        // begin_leafs() already iterates only over leaf nodes
        if (it.getZ() < ground_cutoff) {
            ground_leafs++;
            NodeToFilter node_info;
            node_info.key = it.getKey();
            node_info.coord = it.getCoordinate();
            node_info.is_occupied = filtered_octree->isNodeOccupied(*it);
            nodes_to_filter.push_back(node_info);
        }
    }

    if(params_.debug_text)
      RCLCPP_INFO(this->get_logger(), "Total leaf nodes: %d, Ground nodes to filter: %d", total_leafs, ground_leafs);

    // Mark ground nodes as free space - use setLogOdds() for reliable updates
    int updated_count = 0;
    int already_free_count = 0;
    double free_log_odds = filtered_octree->getClampingThresMinLog();  // Minimum log odds = free space
    
    for (const auto& node_info : nodes_to_filter) {
        try {
            // Get the node at this coordinate
            octomap::OcTreeNode* node = filtered_octree->search(node_info.coord);
            
            if (node != nullptr) {
                if (filtered_octree->isNodeOccupied(node)) {
                    // Node is occupied - set it to free space using log odds directly
                    // This is more reliable than updateNode() for changing occupied->free
                    node->setLogOdds(free_log_odds);
                    updated_count++;
                } else {
                    // Already free - just count it
                    already_free_count++;
                }
            } else {
                // Node doesn't exist - create it as free space
                filtered_octree->updateNode(node_info.coord, false);
                updated_count++;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to update node at (%f, %f, %f): %s", 
                       node_info.coord.x(), node_info.coord.y(), node_info.coord.z(), e.what());
        }
    }
    
    if(params_.debug_text)
      RCLCPP_INFO(this->get_logger(), "Ground filtering: %d nodes updated to free, %d already free, %d total processed", 
                updated_count, already_free_count, nodes_to_filter.size());

    // Update the octree structure after modifications
    filtered_octree->updateInnerOccupancy();
    
    // Prune the tree to remove unnecessary nodes after marking as free
    filtered_octree->prune();

    return filtered_octree;
}


/*
This function returns true if the voxel pt is deemed unknown.
*/
bool AEPlanner::isSmoothedUnknown(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing) {
  

  if(!use_smoothing) // Strict unknown check
  {
    return (ot->search(pt) == nullptr);
  }
  else
  {
    // We look at the surrounding voxels to determine if the voxel is unknown, should work better with sparse pointsclouds.
   
      int known_count = 0;
      int neighbor_count = 0;

      double res = ot->getResolution();
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            octomap::point3d neighbor(pt.x() + dx * res,
                                      pt.y() + dy * res,
                                      pt.z() + dz * res);
            octomap::OcTreeNode* n = ot->search(neighbor);
            if (n != nullptr){
              known_count++;
            }
            neighbor_count++;
          }
        }
      }

      double frac_occ = static_cast<double>(known_count) / neighbor_count;
      if (params_.debug_text)
      {
        //RCLCPP_INFO(this->get_logger(), "Known count: %d", known_count);
        //RCLCPP_INFO(this->get_logger(), "Neighbor count: %d", neighbor_count);
        //RCLCPP_INFO(this->get_logger(), "Frac known: %f", frac_occ);
      }
      return (frac_occ < params_.smoothing_factor); // adjustable threshold
    }
}

/*
This function returns true if the voxel pt is deemed occupied by a majority vote among the neighbor voxels.
*/
bool AEPlanner::isSmoothedOccupied(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing) {
  
  octomap::OcTreeNode* node = ot->search(pt);
  
  if(node)
  {
    // Voxel is known, check if it is occupied
    return ot->isNodeOccupied(node);
  }
  else
  {
    //Voxels is unknown, do majority vote among neighbors if use_smoothing is true
    //or return true if use_smoothing is false (assuming unknown voxels are occupied)
    if(use_smoothing)
    {
      int occ_count = 0;
      int neighbor_count = 0;

      double res = ot->getResolution();
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            octomap::point3d neighbor(pt.x() + dx * res,
                                      pt.y() + dy * res,
                                      pt.z() + dz * res);
            octomap::OcTreeNode* n = ot->search(neighbor);
            if (n && ot->isNodeOccupied(n)) {
              occ_count++;
            }
            neighbor_count++;
          }
        }
      }

      double frac_occ = static_cast<double>(occ_count) / neighbor_count;
      if (params_.debug_text)
      {
        RCLCPP_INFO(this->get_logger(), "Occupied count: %d", occ_count);
        RCLCPP_INFO(this->get_logger(), "Neighbor count: %d", neighbor_count);
        RCLCPP_INFO(this->get_logger(), "Frac occupied: %f", frac_occ);
      }
      return (frac_occ > params_.smoothing_factor); // adjustable threshold
    }
    else
    {
      //Default assume occupied if unknown
      return true;
    }
  }
}


/*
This function should return true if there is collision with the octomap between p1 and p2, or if there are any unknown voxels in between p1 and p2 that disables movement.
If we use a bounding sphere, use r as the radius of the sphere.
If we use a bounding box, use x_diameter, y_diameter, z_diameter as the dimensions of the box.
*/
int AEPlanner::collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r, bool use_sphere_collision_checking, double x_diameter, double y_diameter, double z_diameter, std::shared_ptr<octomap::OcTree> ot)
{

  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  //Use collision checking logic for a sphere
  if(use_sphere_collision_checking)
  { 
        
    if (params_.debug_text)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Sphere collision checking, radius: " << r);
    }

    // Slice out a cube with corners in min/max from octomap
    octomap::point3d min(std::min(p1[0], p2[0]) - r, 
                        std::min(p1[1], p2[1]) - r, 
                        std::min(p1[2], p2[2]) - r);
    octomap::point3d max(std::max(p1[0], p2[0]) + r, 
                        std::max(p1[1], p2[1]) + r, 
                        std::max(p1[2], p2[2]) + r);

    if (params_.debug_text)
    {
      aep_viz->visualizeOctomapSlice(octomap_leafs_pub_, min, max);
    }

    // Travel cylinder collision checking
    double lsq_travel = (end - start).norm_sq();
    double rsq_travel = r * r;

    // First pass: check for unknown voxels inside start sphere, end sphere and travel cylinder
    for (float x = min.x(); x <= max.x(); x += ot->getResolution()) {
      for (float y = min.y(); y <= max.y(); y += ot->getResolution()) {
        for (float z = min.z(); z <= max.z(); z += ot->getResolution()) {
          octomap::point3d pt(x, y, z);

          bool inside_start         = false; //(start - pt).norm() <= r;
          bool inside_sphere_travel = CylTest_CapsFirst(start, end, lsq_travel, rsq_travel, pt) > 0;
          bool inside_end           = (end - pt).norm() <= r;

          if(first_iteration_)
          {
            // On the first iteration, do not consider collisions with the start and travel cylinder.
            // This is since we do not have complete coverage of voxels with Turtlebot4/M100 pointcloud.
            inside_start = false;
            inside_sphere_travel = false;
          }

          if (inside_sphere_travel || inside_start || inside_end) {
            if (isSmoothedUnknown(ot.get(), pt, params_.use_smoothing)) {
              if (params_.debug_text) {
                if (inside_start) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *start* sphere at: (" << x << ", " << y << ", " << z << ")");
                }
                if (inside_end) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *end* sphere at: (" << x << ", " << y << ", " << z << ")");
                }
                if (inside_sphere_travel) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *travel* cylinder at: (" << x << ", " << y << ", " << z << ")");
                }
              }
              return -1; // Unknown voxel → abort
            }
          }
        }
      }
    }

    // Second pass: check for occupied voxels inside sphere region
    for (auto voxel = ot->begin_leafs_bbx(min, max), voxel_end = ot->end_leafs_bbx(); voxel != voxel_end; ++voxel) {
      octomap::point3d voxel_center(voxel.getX(), voxel.getY(), voxel.getZ());

      if (isSmoothedOccupied(ot.get(), voxel_center, params_.use_smoothing)) {
        
        /*
        // Start sphere collision checking - commented out to match bounding box behavior
        if ((start - voxel_center).norm() <= r) {
          if(first_iteration_)
          {
            return 0;
          }

          if (params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with start sphere");
          return 1;
        }
        */

        if ((end - voxel_center).norm() <= r) {
          if (params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with end sphere");
          return 2;
        }

        if (CylTest_CapsFirst(start, end, lsq_travel, rsq_travel, voxel_center) > 0) {
          if(first_iteration_)
          {
            return 0;
          }

          if (params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with travel cylinder");
          return 3;
        }
      }
    }
  }
  else
  {
    if(params_.debug_text) 
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Box collision checking");
        RCLCPP_INFO_STREAM(this->get_logger(), "x_diameter: " << x_diameter << " y_diameter: " << y_diameter << " z_diameter: " << z_diameter);
    }

    // Start / End / Travel cylinder radius
    double major_length = std::max(x_diameter, y_diameter);
    double outer_radius = (std::sqrt(2) * (major_length / 2.0));
    double major_length_travel = std::max(y_diameter, z_diameter);
    double outer_radius_travel = (std::sqrt(2) * (major_length_travel / 2.0));
   
    // Octomap slice
    octomap::point3d min(std::min(p1[0], p2[0]) - outer_radius, std::min(p1[1], p2[1]) - outer_radius, std::min(p1[2], p2[2]) - z_diameter/2.0);
    octomap::point3d max(std::max(p1[0], p2[0]) + outer_radius, std::max(p1[1], p2[1]) + outer_radius, std::max(p1[2], p2[2]) + z_diameter/2.0);
       
    // Start cylinder collision checking
    Eigen::Vector4d start_center(p1[0], p1[1], p1[2], 0.0);
    octomap::point3d start_center_octomap(p1[0], p1[1], p1[2]);
    octomap::point3d start_bottom(p1[0], p1[1], p1[2] - z_diameter/2.0);
    octomap::point3d start_top(p1[0], p1[1], p1[2] + z_diameter/2.0);
    double lsq_start = (start_top - start_bottom).norm_sq();
    double rsq_start = outer_radius * outer_radius;

    // End cylinder collision checking
    Eigen::Vector4d end_center(p2[0], p2[1], p2[2], 0.0);
    octomap::point3d end_center_octomap(p2[0], p2[1], p2[2]);
    octomap::point3d end_bottom(p2[0], p2[1], p1[2] - z_diameter/2.0);
    octomap::point3d end_top(p2[0], p2[1], p1[2] + z_diameter/2.0);
    double lsq_end = (end_top - end_bottom).norm_sq();  
    double rsq_end = outer_radius * outer_radius;

    // Travel cylinder collision checking
    double lsq_travel = (end - start).norm_sq();
    double rsq_travel = outer_radius_travel * outer_radius_travel;

    // Visualize start and end cylinders and travel cylinder
    if(params_.debug_text)
    {
      aep_viz->visualizeOctomapSlice(octomap_leafs_pub_, min, max);
      aep_viz->visualizeCylinders(cylinders_pub_, start_center, end_center, (2 * outer_radius), (2 * outer_radius), z_diameter, (2 * outer_radius), (2 * outer_radius), z_diameter);
      aep_viz->visualizeTunnel(cylinders_pub_, start_center, end_center, outer_radius_travel);
    }
   

    // First pass: check for any unknown voxels *inside* the cylinders
    for (float x = min.x(); x <= max.x(); x += ot->getResolution()) {
      for (float y = min.y(); y <= max.y(); y += ot->getResolution()) {
        for (float z = min.z(); z <= max.z(); z += ot->getResolution()) {
          octomap::point3d pt(x, y, z);

          bool inside_start   = false; //CylTest_CapsFirst(start_bottom, start_top, lsq_start, rsq_start, pt) > 0;
          bool inside_travel  = CylTest_CapsFirst(start_center_octomap, end_center_octomap, lsq_travel, rsq_travel, pt) > 0;
          bool inside_end     = CylTest_CapsFirst(end_bottom, end_top, lsq_end, rsq_end, pt) > 0;

          if(first_iteration_)
          {
            // On the first iteration, do not consider collisions with the start and travel cylinder.
            // This is since we do not have complete coverage of voxels with Turtlebot4 pointcloud.
            inside_start = false;
            inside_travel = false;
          }
          
          if (inside_start || inside_end || inside_travel) {
            
            //octomap::OcTreeNode* node = ot->search(x, y, z);
            //if (node == nullptr) {
            if(isSmoothedUnknown(ot.get(), pt, params_.use_smoothing)){  
              
              if (params_.debug_text) {
                if (inside_start) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *start* cylinder at: (" << x << ", " << y << ", " << z << ")");
                }
                if (inside_end) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *end* cylinder at: (" << x << ", " << y << ", " << z << ")");
                }
                if (inside_travel) {
                  RCLCPP_WARN_STREAM(this->get_logger(),
                    "Unknown voxel inside *travel* cylinder at: (" << x << ", " << y << ", " << z << ")");
                }

              }
              return -1;  // Unknown voxel inside relevant area — abort
            }
          }
        }
      }
    }

    // Second pass: Check for any occupied voxels inside the cylinders
    for (auto voxel = ot->begin_leafs_bbx(min, max), voxel_end = ot->end_leafs_bbx(); voxel != voxel_end; ++voxel)
    {
      octomap::point3d voxel_center(voxel.getX(), voxel.getY(), voxel.getZ());

      //if (voxel->getLogOdds() > 0) // occupied
      if(isSmoothedOccupied(ot.get(), voxel_center, params_.use_smoothing))
      {

        /*
        if(CylTest_CapsFirst(start_bottom, start_top, lsq_start, rsq_start, voxel_center) > 0)
        {

          if(first_iteration_)
          {
            return 0;
          }

          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node in collision with start cylinder");
          return 1;
        }
        */

        if(CylTest_CapsFirst(end_bottom, end_top, lsq_end, rsq_end, voxel_center) > 0)
        {
          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node in collision with end cylinder");
          return 2;
        }

        if(CylTest_CapsFirst(start_center_octomap, end_center_octomap, lsq_travel, rsq_travel, voxel_center) > 0)
        {

          if(first_iteration_)
          {
            return 0;
          }

          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node in collision with travel cylinder");
          return 3;
        }
      }
    }  
  }

  // If we made it here, no static collision was detected
  if(params_.debug_text)
    RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node not in static collision with OctoMap");
  return 0;
}


Eigen::Vector4d AEPlanner::restrictDistance(Eigen::Vector4d nearest,
                                            Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1], new_pos[2] - origin[2]);
  
  //if (direction.norm() > params_.extension_range)
  
  direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

RRTNode* AEPlanner::chooseParent(RRTNode* node, double l)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  // Find nearest neighbour
  kdres* nearest = kd_nearest_range3(kd_tree_, node->state_[0], node->state_[1],
                                     node->state_[2], l + 0.5); // FIXME why +0.5? We have 0.5 location tolerance I assume?

  if (kd_res_size(nearest) <= 0)
    nearest = kd_nearest3(kd_tree_, node->state_[0], node->state_[1], node->state_[2]);
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RRTNode* node_nn = (RRTNode*)kd_res_item_data(nearest);

  RRTNode* best_node = node_nn;
  double best_node_cost = best_node->cost();
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    double node_cost = node_nn->cost();
    if (best_node and node_cost < best_node_cost)
    {
      best_node = node_nn;
      best_node_cost = node_cost;
    }

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}


void AEPlanner::expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  if(!params_.ignore_first_iteration)
  {
    first_iteration_ = false;
  }
  
  RCLCPP_WARN_STREAM(this->get_logger(),
                    "First iteration flag is: " << std::boolalpha << first_iteration_);

  if(params_.filter_out_ground)
  {
    ot = filterGroundFromOctomap(ot_, params_.ground_filter_z_dist);
        
    if(params_.viz_filtered_octomap)
    {
      aep_viz->visualizeFilteredOctomap(octomap_filtered_pub_, ot);
    }
  }

  double estimated_yaw = 0.0;

  // Expand an RRT tree and calculate information gain in every node
  RCLCPP_INFO_STREAM(this->get_logger(), "Entering expanding RRT");
  std::vector<int> rejectedReasons{};
  std::vector<Eigen::Vector4d> rejectedNodes{};
  std::vector<Eigen::Vector4d> neighbors{};
  std::vector<Eigen::Vector4d> validNodes{};
  std::vector<Eigen::Vector4d> candidateNodes{};
  int N_valid = 0; // Valid sampled nodes counter
  int N_sampled_nodes = 0; 

   // Create MarkerArray
   visualization_msgs::msg::MarkerArray marker_array;
    
   // Clear existing markers
   visualization_msgs::msg::Marker delete_marker;
   delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
   marker_array.markers.push_back(delete_marker);
   
   // Publish the delete marker
   sampled_node_pub_->publish(marker_array);

  // (1) Sample until N_valid > init_iterations and total sampled_nodes < max_sampled_initial_nodes
  // Idea (1): Gain enough coverage (at least init_iterations) but stop sampling at max_sampled_initial_nodes
  // (2) If more than max_sampled_initial_nodes are sampled: continue until max_sampled_nodes as long as dynamic score is not high enough or until we reach max_sampled_nodes
  while ((N_valid < params_.init_iterations and N_sampled_nodes < params_.max_sampled_initial_nodes) or 
        ((N_valid > 0 and N_sampled_nodes < params_.max_sampled_nodes and best_node_->dynamic_score(params_.lambda, params_.zeta) < (params_.max_volume * params_.zero_gain_ratio) ) ))

    {
      RRTNode* new_node = new RRTNode(params_.drone_angular_velocity, params_.drone_linear_velocity);
      RRTNode* nearest;
      octomap::OcTreeNode* ot_result;
      bool isInside = false;
      int collisionBetweenNodes = 1; // 1 = no collision, 2 = collision with start cylinder, 3 = collision with end cylinder, 4 = collision with cube along line
   
      // Sample new point around agent and check that
      // (1) it is within the boundaries
      // (2) it is in known space
      // (3) the path between the new node and it's parent does not contain any
      // obstacles

      do
        {
          Eigen::Vector4d offset = sampleNewPoint(params_.max_sampling_radius);
          
          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "offset (" << offset[0] << ", " << offset[1] << ", " << offset[2] << ")");

          new_node->state_ = current_state_ + offset;
          
          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "current_state_ + offset (" << new_node->state_[0] << ", " << new_node->state_[1] << ", " << new_node->state_[2] << ")");

          nearest = chooseParent(new_node, params_.extension_range);

          if (nearest == NULL)
          {
            // Nearest node could be not be found, continue sampling.
            continue;
          }

          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "nearest (" << nearest->state_[0] << ", " << nearest->state_[1] << ", " << nearest->state_[2] << ")");

          new_node->state_ = restrictDistance(nearest->state_, new_node->state_);

          if(params_.debug_text)
            RCLCPP_INFO_STREAM(this->get_logger(), "restricted (" << new_node->state_[0] << ", " << new_node->state_[1] << ", " << new_node->state_[2] << ")");

          
          // Is new_node->state in known space (free or occupied)?
          // If in known space, return a pointer to the octomap node, otherwise return NULL
          // NOTE: This only checks the (x,y,z) position, not the bounding box around it.
          ot_result = ot->search(octomap::point3d(new_node->state_[0], new_node->state_[1], new_node->state_[2]));          
          
          if (ot_result == NULL)
          {
            if(params_.debug_text){
              RCLCPP_INFO_STREAM(this->get_logger(), "Node to be added to tree is not in known space (Free or Occupied), continue sampling...");
            }
            continue;
          }

          // COLLISION CHECKING **
          double ot_thres = ot_->getOccupancyThres();
          double prob = ot_result->getOccupancy();
          bool occupance = ot_->isNodeOccupied(ot_result);

          if(params_.debug_text)
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "Threshold for occupancy: " << ot_thres);
            RCLCPP_INFO_STREAM(this->get_logger(), "Is sampled node in known space (Free or Occupied)? Gibberish = Yes, Otherwise No -- " << ot_result);
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupancy-prob: " << prob);
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied: " << std::boolalpha << occupance);
          }
          
          isInside = isInsideBoundaries(new_node->state_);
          collisionBetweenNodes = collisionLine(nearest->state_, new_node->state_, params_.bounding_radius, params_.sphere_bb, params_.x_diameter, params_.y_diameter, params_.z_diameter, ot);

          // ----------------------------------

          if(params_.debug_text)
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node inside world boundaries? " << std::boolalpha << isInside );
            RCLCPP_INFO_STREAM(this->get_logger(), "Is sampled node in known space (Free or Occupied)? Gibberish = Yes, Otherwise No -- " << ot_result);
            RCLCPP_INFO_STREAM(this->get_logger(), "Static collision between nearest and sampled node? " << std::boolalpha << collisionBetweenNodes);
          }
          
          /*
          bool static_collision = !isInside or !ot_result or collisionBetweenNodes;
          
          if(static_collision)
          {
            // Visualize rejected node as red
            if (collisionBetweenNodes) {
              rejectedReasons.push_back(collisionBetweenNodes);
            } else {
              rejectedReasons.push_back(10);
            }
            if (collisionBetweenNodes == 1) {
              rejectedNodes.push_back(nearest->state_);
            } else {
              rejectedNodes.push_back(new_node->state_);
            }
            neighbors.push_back(nearest->state_);
          }

          aep_viz->visualizeRejectedNode(sampled_node_pub_, rejectedNodes, rejectedReasons);
          aep_viz->visualizeNearestNode(nearest_node_pub_, neighbors);
          aep_viz->visualizeValidNodes(valid_nodes_pub_, validNodes);
          */


        } while (!isInside or collisionBetweenNodes);

      estimated_yaw = get_nn_yaw(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
      new_node->state_[3] = estimated_yaw;

      double expand_rrt_time = 3.0; // From observation, it will take RRT expansion 3 seconds to complete

      // Estimate time to reach node
      double time_to_reach_node = nearest->time_cost(current_state_, params_.rotating_while_walking) + nearest->time_to_reach(new_node, current_state_, params_.rotating_while_walking) + expand_rrt_time;

      if(params_.debug_text)
        RCLCPP_ERROR_STREAM(this->get_logger(), "time_to_reach_node from current_state: " << time_to_reach_node);

      bool collision = checkCollision(time_to_reach_node, new_node->state_, trajectories, covarianceEllipses) && dynamic_mode_;
      
      if (!collision) 
        {
          double time_to_reach_node_complete = nearest->time_cost(current_state_, true) + nearest->time_to_reach(new_node, current_state_, params_.rotating_while_walking + expand_rrt_time);

          ///RCLCPP_ERROR_STREAM(this->get_logger(), "Get gain");
          std::tuple<double, double, double> ret = getGain(new_node, time_to_reach_node_complete);
          // Update node
          new_node->gain_ = std::get<0>(ret);
          new_node->dynamic_gain_ = std::get<1>(ret);
          new_node->state_[3] = std::get<2>(ret); //Set new yaw that maximizes gain

          //Aquire DFM score
          double score = get_dfm_score(new_node->state_[0], new_node->state_[1], 0.0);
          new_node->dfm_score_ = score;
          ///RCLCPP_ERROR_STREAM(this->get_logger(), "Get gain2");
          
          N_valid++;
          // new_node is now ready to be added to tree
          new_node->parent_ = nearest; 
      
          nearest->children_.push_back(new_node);

          // rewire tree with new node
          rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius);
          
          kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
                     new_node);

          // Update best node
          if (!best_node_ or new_node->dynamic_score(params_.lambda, params_.zeta) > best_node_->dynamic_score(params_.lambda, params_.zeta))
          {
            best_node_ = new_node;
          }
            
          // Estimate time to reach node (Now we know the yaw)
          Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
          validNodes.push_back(p4);
        } 
      else 
        {

          if(params_.add_rejected_nodes)
          {
          // Add rejected node to tree for planning purposes, however we can not go to that node.
          new_node->dynamic_collision_ = true;

          N_valid++;
          // new_node is now ready to be added to tree
          new_node->parent_ = nearest; 
      
          nearest->children_.push_back(new_node);

          // rewire tree with new node
          rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius);
          
          kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
                     new_node);

            // Update best node
            if (!best_node_ or new_node->dynamic_score(params_.lambda, params_.zeta) > best_node_->dynamic_score(params_.lambda, params_.zeta))
            {
              best_node_ = new_node;
              best_node_->dynamic_collision_ = true;
            }
          }
            
          // Estimate time to reach node (Now we know the yaw)
          Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
          validNodes.push_back(p4);
          RCLCPP_ERROR_STREAM(this->get_logger(), "Dynamic collision, node not valid !");
        }
        
      N_sampled_nodes++;
      if(params_.debug_text)
        RCLCPP_INFO_STREAM(this->get_logger(), "N_SAMPLED_NODES: " << N_sampled_nodes);
    }

  if(N_valid > 0)
  {
    // Visualize the best node as green (=2)
    Eigen::Vector4d p4(best_node_->state_[0], best_node_->state_[1], best_node_->state_[2], 2);
    validNodes.push_back(p4);
    assert(aep_viz);
    aep_viz->visualizeBestNode(best_marker_pub_, validNodes);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No valid nodes found!");
  }

  first_iteration_ = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "RRT expansion finished");
}

std::tuple<double, double, double> AEPlanner::getGain(RRTNode* node, double time_of_arrival)
{
  //pigain::Query srv;
  //srv.request.point.x = node->state_[0];
  //srv.request.point.y = node->state_[1];
  //srv.request.point.z = node->state_[2];
  
  // GP currently deactivated
  //if (gp_query_client_.call(srv))
  //{
  //  double gain = srv.response.mu;
  //  double yaw = srv.response.yaw;
  //  ROS_DEBUG_STREAM("gain impl: " << gain);
  //  ROS_DEBUG_STREAM("sigma: " << srv.response.sigma);
   // if (srv.response.sigma < params_.sigma_thresh)
   // {
   //   double gain = srv.response.mu;
   //   double yaw = srv.response.yaw;
   //
   //   ROS_DEBUG_STREAM("gain impl: " << gain);
   //   return std::make_pair(gain, yaw);
   // }
  //}
  
  node->gain_explicitly_calculated_ = true;  
  std::tuple<double, double, double> ret = gainCubature(node->state_, time_of_arrival);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "gain expl: " << std::get<0>(ret));
  return ret;
}

void AEPlanner::rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  if(params_.filter_out_ground)
  {
    ot = filterGroundFromOctomap(ot_, params_.ground_filter_z_dist);
        
    if(params_.viz_filtered_octomap)
    {
      aep_viz->visualizeFilteredOctomap(octomap_filtered_pub_, ot);
    }
  }

  RRTNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->state_[0], new_node->state_[1],
                                     new_node->state_[2], l + 0.5); // FIXME why +0.5?
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    Eigen::Vector3d p1(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
    Eigen::Vector3d p2(node_nn->state_[0], node_nn->state_[1], node_nn->state_[2]);
    
    if (node_nn->cost() > new_node->cost() + (p1 - p2).norm())
    {
      if (!collisionLine(new_node->state_, 
                        node_nn->state_, 
                        r, 
                        params_.sphere_bb, 
                        params_.x_diameter, 
                        params_.y_diameter, 
                        params_.z_diameter, ot))
        node_nn->parent_ = new_node;
    }
    
    kd_res_next(nearest);
  }
}

/*
Get frontiers from the best node
*/
geometry_msgs::msg::PoseArray AEPlanner::getFrontiers()
{
  geometry_msgs::msg::PoseArray frontiers;

  auto response = get_best_node(0.0, 0.0, 0.0, params_.reevaluate_node_threshold);
  
  if (response)
  {
    for (size_t i = 0; i < response->best_node.size(); ++i)
    {
      geometry_msgs::msg::Pose frontier;
      frontier.position = response->best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  return frontiers;
}

/*
void AEPlanner::publishEvaluatedNodesRecursive(RRTNode* node)
{
  if (!node) {
    return;
  }

  /// RCLCPP_ERROR(this->get_logger(), "publishEvaluatedNodesRecursive: Enter");

  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      daep_msgs::msg::PigainNode pig_node;
      pig_node.gain = (*node_it)->gain_;
      pig_node.dynamic_gain = (*node_it)->dynamic_gain_;
      pig_node.position.x = (*node_it)->state_[0];
      pig_node.position.y = (*node_it)->state_[1];
      pig_node.position.z = (*node_it)->state_[2];
      pig_node.yaw = (*node_it)->state_[3];
      RCLCPP_INFO(this->get_logger(), "publishEvaluatedNodesRecursive: Publish on topic gain_node");
      RCLCPP_INFO(this->get_logger(), "sendtopig: %f %f - %f %f %f", pig_node.gain, pig_node.dynamic_gain,
                   pig_node.position.x, pig_node.position.y, pig_node.position.z);
      gain_pub_->publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);

  }
}
*/

void AEPlanner::publishEvaluatedNodesRecursive(RRTNode* node)
{
  if (!node) {
    return;
  }

  // Create a list to store all nodes
  daep_msgs::msg::PigainNodeList pig_node_list;

  // Helper function to gather all nodes recursively
  std::function<void(RRTNode*)> gatherNodes = [&](RRTNode* current_node) {
    if (!current_node) {
      return;
    }

    for (typename std::vector<RRTNode*>::iterator node_it = current_node->children_.begin();
         node_it != current_node->children_.end(); ++node_it)
    {
      if ((*node_it)->gain_explicitly_calculated_ && (*node_it)->gain_ > params_.add_node_to_frontier_threshold)
      {
        daep_msgs::msg::PigainNode pig_node;
        pig_node.gain = (*node_it)->gain_;
        pig_node.dynamic_gain = (*node_it)->dynamic_gain_;
        pig_node.position.x = (*node_it)->state_[0];
        pig_node.position.y = (*node_it)->state_[1];
        pig_node.position.z = (*node_it)->state_[2];
        pig_node.yaw = (*node_it)->state_[3];
        
        // Add the node to the list instead of publishing it
        pig_node_list.nodes.push_back(pig_node);
        
        if(params_.debug_text) {

        RCLCPP_INFO(this->get_logger(), "Adding to list: %f %f - %f %f %f", 
                    pig_node.gain, pig_node.dynamic_gain,
                    pig_node.position.x, pig_node.position.y, pig_node.position.z);
        }
      }

      // Recurse on each child
      gatherNodes(*node_it);
    }
  };

  // Gather all nodes starting from the input node
  gatherNodes(node);

  // Publish the entire list at once
  if (!pig_node_list.nodes.empty()) {
    RCLCPP_INFO(this->get_logger(), "Publishing list with %zu frontiers to pig.py from RRT-search", pig_node_list.nodes.size());
    gain_pub_->publish(pig_node_list);
  }
}

geometry_msgs::msg::Pose AEPlanner::vecToPose(Eigen::Vector4d state)
{
  // tf2::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);

  //tf2::Pose pose_tf(quat, origin);

  geometry_msgs::msg::Pose pose;
  // tf2::poseTFToMsg(pose_tf, pose);

  pose.position.x = state[0];
  pose.position.y = state[1];
  pose.position.z = state[2];
  pose.orientation.x = quat[0];
  pose.orientation.y = quat[1];
  pose.orientation.z = quat[2];
  pose.orientation.w = quat[3];

  return pose;
}


/*
* Check intersection between cylinder at point and a cylinder at center.
*/
bool AEPlanner::isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, double, Eigen::MatrixXd> covariance_ellipse) 
{ 
  bool collision{};
  double circle_radius, covariance_height;

  if(params_.sphere_bb)
  {
    circle_radius = std::get<0>(covariance_ellipse) + params_.flying_obstacle_diameter/2;
    covariance_height = params_.flying_obstacle_diameter;
  }
  else
  {
    // Extract covariance ellipse (cylinder bottom centered in center, cylinder (and bottom) as radius circle radius) and totalt height of cylinder from bottom is covariance_height
    circle_radius = std::get<0>(covariance_ellipse) + params_.human_width/2; // Blow up radius to include human width (Worst-case scenario)
    covariance_height = params_.human_height;
  }
  
  if(params_.prediction_mode == "circle")
  {
    circle_radius = std::get<0>(covariance_ellipse);
  }
  
  // Extract robot bounding box cylinder
  // This cylinder is centered in point with radius outer_radius and height robot_height
  double major_length, outer_radius, robot_height, agent_radius;
  if(params_.sphere_bb)
  {
    agent_radius = params_.flying_obstacle_diameter / 2.0;
  }
  else
  {
    major_length = std::max(params_.x_diameter, params_.y_diameter);
    outer_radius = (std::sqrt(2) * (major_length)) / 2.0;
    robot_height = params_.z_diameter;
  }

  if(params_.sphere_bb)
  {
    // Sphere Collision check (Flying obstacles)
    // Calculate the 3D distance between sphere centers
    double distance = (point - center).norm();
    
    // Calculate sum of radii for the two spheres
    double radii_sum = circle_radius + agent_radius;
    
    // Spheres intersect if the distance between centers is less than or equal to the sum of radii
    collision = (distance <= radii_sum);
    //RCLCPP_ERROR_STREAM(this->get_logger(), "Dynamic (Flying) collision: " << std::boolalpha << collision);
  }
  else
  {
    // Cylinder Collision check (Human obstacles)
    // First check if cylinders are separated along the z-axis
    double z_distance = std::abs(point.z() - (center.z() + covariance_height / 2.0));
    double z_separation = (robot_height + covariance_height) / 2.0;
    
    if (z_distance > z_separation) {
      // Cylinders are separated along z-axis, no collision
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cylinders are separated along z-axis, no collision");
      return false;
    }
    
    // Check horizontal (XY plane) distance between cylinder centers
    double xy_distance = std::sqrt(std::pow(point.x() - center.x(), 2) + 
                                  std::pow(point.y() - center.y(), 2));
    
    // Sum of radii
    double radii_sum = circle_radius + outer_radius;
    
    // Cylinders intersect if the distance between centers is less than or equal to the sum of radii
    collision = (xy_distance <= radii_sum);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cylinders (Human) collision in xy-plane: " << std::boolalpha << collision);
  }

  return collision;
}    



/* 
* Calculate if there is a potential collision with any moving obstacle in time t.
*/
bool AEPlanner::checkCollision(double t, 
                              Eigen::Vector4d point, 
                              std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                              std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses)
{

  int N_persons = trajectories.size();

  // If time window is 0.0, we don't check for collisions in the time window, but only at a single time step t.
  if(params_.time_window == 0.0)
  {
    
    // Index the correct circle
    int covariance_index = getCovarianceIndex(params_.KFiterations * dt_, dt_, t);
  
    if(covariance_index == -1)
    { 
      // Outside our prediction
      return false;
    }
  
    if(params_.debug_text)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "time point collision check: " << t);
    }

    for(int i = 0; i < N_persons; i++)
    { 
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];
  
      const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
  
  
      if(isCircleInsideEllipse(point.head(3), center, covarianceEllipse)) {
        return true;
      }
    }
    return false;
  }
  else
  {
    // Check for collision in time window <t-time_window, t+time_window>

    if(params_.debug_text)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "time window collision check: " << t - params_.time_window << " to " << t + params_.time_window);
    }

    for(double time = t - params_.time_window; time <= t + params_.time_window; time += dt_)
    {
      int covariance_index = getCovarianceIndex(params_.KFiterations * dt_, dt_, time);
      if(covariance_index == -1)
      {
        // Outside prediction range
        continue;
      }

      for(int i = 0; i < N_persons; i++)
      {
        auto personEllipses = covarianceEllipses[i];
        auto personTrajectory = trajectories[i];
        
        std::tuple<double, double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
        double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
        double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
        double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];
        const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);

        if(isCircleInsideEllipse(point.head(3), center, covarianceEllipse))
        {
          return true;
        }
      }
    }
    return false;
  }
}


/*
Check if a branch is predicted to be collision free. It will return (nullptr, false) if a collision is detected
or (leaf_node, true) if the whole branch is collision free (leaf_node is the last node in the branch).
*/
std::pair<RRTNode*, bool> AEPlanner::pathIsSafe(RRTNode* node, 
                                                std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                                                std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  if(node == NULL){
    return std::make_pair(nullptr, false);
  }

  RRTNode* curr = node;
  // Traverse until root
  while(!curr->children_.empty()){

    double expand_rrt_time = 3.0; // From observation, it will take RRT expansion 3 seconds to complete

    // if any node has a potential collision return false
    double time_to_reach_node = curr->time_cost(current_state_, params_.rotating_while_walking) + expand_rrt_time;

    bool collision = checkCollision(time_to_reach_node, curr->state_, trajectories, covarianceEllipses) && dynamic_mode_;
    if (collision){
      return std::make_pair(nullptr, false);  // Collision detected, return false
    }
    else{
      curr = curr->children_[0];
    }
  }  
  return std::make_pair(curr, true);  // No collision, return the leaf node and true
}


Eigen::Vector4d AEPlanner::sampleNewPoint(double max_sampling_radius)
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point;
  do
    {
      for (int i = 0; i < 3; i++) 
      {
        point[i] = max_sampling_radius * 2.0 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      }
  } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) > pow(max_sampling_radius, 2.0));
  
  
  if(!params_.sphere_bb)
  {
    // If we are using a bounding box (ground vehicle), restrict the z-coordinate
    point[2] = 0.0; //Force z-coordinate to not change ==> we are only sampling in the XY-plane centered at the height of the robot
  }

  return point;
}


/**
 * Check if a point will intersect with a dynamic obstacle at time_of_arrival
 * if that is the case, the view will be blocked and we return true. 
*/
bool AEPlanner::willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts)
{
  // Race condition
  std::lock_guard<std::mutex> lock(vecMutex);

  if(visualize_ghosts)
  {
    std::vector<std::tuple<double, double, double>> positions;
    for(auto const& person : predicted_data)
    { 
      double x = std::get<0>(person.first)[index];
      double y = std::get<1>(person.first)[index];
      double z = std::get<2>(person.first)[index];
      std::tuple<double, double, double> position = std::make_tuple(x,y,z);
      positions.push_back(position);
    }
    aep_viz->visualizeGhostPedestrian(ghost_marker_pub_, positions);
  }
  
  for(auto const& person : predicted_data)
  { 
    double x = std::get<0>(person.first)[index];
    double y = std::get<1>(person.first)[index];
    double z = std::get<2>(person.first)[index];
    if(isCollisionWithBoundingBox(point, x, y, z))
    {
      return true;
    }
    
  }
  return false;
}

/**
 * Check if a certain point collides with a bounding box, in this case the square
 * of a human obstacle.
*/
bool AEPlanner::isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z)
{
  return point[0] >= (x - params_.human_width/2) and point[0] <= (x + params_.human_width/2) and
         point[1] >= (y - params_.human_width/2) and point[1] <= (y + params_.human_width/2) and
         point[2] >= z and point[2] <= (z + params_.human_height);
}



/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int AEPlanner::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  if(t <= 0)
  {
    return -1;
  }
  
  if(params_.prediction_mode == "circle")
  {
    return 0;
  }

  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}



/**
 * This function calculates the gain explicitly from the given state (point).
 * It calculates the dynamic gain as well as the static gain and returns them in a tuple
 * with the best yaw angle.
 */
std::tuple <double, double, double> AEPlanner::gainCubature(Eigen::Vector4d state, double time_of_arrival)
{
  visualization_msgs::msg::MarkerArray static_rays;
  visualization_msgs::msg::MarkerArray dynamic_rays;

  // **Declare intenal variables**
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double static_gain = 0.0;
  double dynamic_gain = 0.0;
  
  // For Husky: accumulate total gain across all 360 degrees
  double total_static_gain = 0.0;
  double total_dynamic_gain = 0.0;

  //Field of View
  double fov_y = params_.hfov, fov_p = params_.vfov; //Horizontoal 103.2 deg and Vertical 77.4 deg

  //Radius variables
  double r;
  double r_min = params_.r_min, r_max = params_.r_max;

  //Angles
  int phi, theta;
  double phi_rad, theta_rad;
  
  //Step size for r, phi and theta. In degrees and radians.
  double dr = params_.dr;
  double dphi = params_.dphi;
  double dtheta = params_.dtheta;
  
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;

  //Maps
  std::map<int, double> gain_per_yaw;
  std::map<int, double> dynamic_gain_per_yaw;

  //Set the origin to the point to be examined
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;

  double max_time_step = params_.KFiterations * dt_;

  int index = getCovarianceIndex(max_time_step, dt_, time_of_arrival);
  int ray_id = 0;
  bool blocked = false;

  //For each theta in a circle
  for (theta = -180; theta < 180; theta += dtheta)
  {
    theta_rad = M_PI * theta / 180.0f;
    
    //For each phi in pitch
    for (phi = (90 - fov_p / 2); phi < (90 + fov_p / 2); phi += dphi)
    {
      phi_rad = M_PI * phi / 180.0f;

      //Gain for a specific theta and phi over all r
      double static_g = 0.0;
      double dynamic_g = 0.0;
      visualization_msgs::msg::Marker static_ray;
      visualization_msgs::msg::Marker dynamic_ray;
      blocked = false;

      if(params_.visualize_static_and_dynamic_rays)
      {
        static_ray = aep_viz->createRayMarker(ray_id, "BLUE");
        dynamic_ray = aep_viz->createRayMarker(ray_id, "RED");
      }
      
      //For each r in distance from origin to r_max
      for (r = r_min; r < r_max; r += dr)
      {
        //A point in x,y,z from spherical coordinates
        //So from this volume we are looping over, we extract one point
        vec[0] = origin[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = origin[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = origin[2] + r * cos(phi_rad);

        //Search for the x,y,z point in the OctomapTree
        octomap::point3d query(vec[0], vec[1], vec[2]);
        octomap::OcTreeNode* result = ot->search(query);

        //Construct a Vector4d from the point but with yaw = 0
        Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);

        geometry_msgs::msg::Point point;
        point.x = vec[0];
        point.y = vec[1];
        point.z = vec[2];

        if (not blocked and dynamic_mode_ and index != -1)
        {
          blocked = willViewBeBlocked(vec, index, params_.visualize_static_and_dynamic_rays);
        }

        //If the point is outside the map, stop computing gain in that direction.
        //Off by default (aep/gain/inside_boundary). Needed where nothing closes the area
        //in (outdoors): unknown space outside the boundary would otherwise always give
        //gain, and the exploration would never complete.
        if (params_.gain_inside_boundary and !isInsideBoundaries(v))
        {
          break;
        }

        if (result) //If the point is in known space
        {
          //Iterating forward with r, if the current point is occupied it means 
          //that we have hit a static obstacle
          if (result->getLogOdds() > 0)
          {
            //So break the r loop
            break;
          }

          //If the point was free, do not break the loop so continue to loop over r
          //since there might be more unknown space to be found after the known space.
        }
        else 
        { 
          double current_gain = (2.0 * r * r * dr + (1.0 / 6.0) * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2.0); 
          
          if(pointOnXYBoundaries(v)){
            current_gain *= params_.boost_magnitude;
          }
               
          if(blocked)
          {
            //The view is blocked by a dynamic obstacle
            //Stop adding gain to the dynamic gain, but continue to add to the static one
            static_g += current_gain;
          }
          else
          {
            //The view is not blocked, add the same gain to static and dynamic
            static_g += current_gain;
            dynamic_g += current_gain;
          }
        }

        if(params_.visualize_static_and_dynamic_rays)
        {
          if(blocked)
          {
            //Visualize static rays behind obstacle
            static_ray.points.push_back(point);
          }
          else
          {
            //Visualize both dynamic and static rays
            static_ray.points.push_back(point);
            dynamic_ray.points.push_back(point);
          }
        }
      }
  
      if(params_.husky)
      {
        // For Husky: accumulate total gain from all directions (360 deg LiDAR)
        total_static_gain += static_g;
        total_dynamic_gain += dynamic_g;
      }
      else
      {
        // For regular use: store gain per yaw for later best yaw selection
        gain_per_yaw[theta] += static_g; //Add for each theta what the current gain is (Yaw)
        dynamic_gain_per_yaw[theta] += dynamic_g; //Add for each theta what the current dymamic_gain is.
      }
      
      if(params_.visualize_static_and_dynamic_rays)
      {
        static_rays.markers.push_back(static_ray); //Add ray to rays for visualize
        if(blocked)
        {
          dynamic_rays.markers.push_back(dynamic_ray); //Add ray to rays for visualize  
        }
        else
        {
          dynamic_ray = aep_viz->createRayMarker(ray_id, "RED");
          dynamic_rays.markers.push_back(dynamic_ray);
        }
        ray_id++;
      }
    }
  }

  if(params_.visualize_static_and_dynamic_rays)
  {
    static_rays_marker_pub_->publish(static_rays);
    dynamic_rays_marker_pub_->publish(dynamic_rays);
  }

  if(params_.husky)
  {
    // For Husky: return total gain accumulated from all 360 degrees
    // No need to select best yaw since we have 360 deg LiDAR coverage
    double yaw = 0.0;
    state[3] = yaw;
    RCLCPP_INFO(this->get_logger(), "Total static gain: %f, Total dynamic gain: %f", total_static_gain, total_dynamic_gain);
    return std::make_tuple(total_static_gain, total_dynamic_gain, yaw);
  }
  else
  {
    //Calculate the best yaw angle
    int best_yaw = 0;
    double best_yaw_score = 0.0, best_dynamic_yaw_score = 0.0;
    
    //For each yaw in one spin
    for (int yaw = -180; yaw < 180; yaw++)
    {
      double yaw_score = 0;
      double dynamic_yaw_score = 0;
      //For each field of view in the y-direction
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
      {
        int theta = yaw + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        yaw_score += gain_per_yaw[theta];
        dynamic_yaw_score += dynamic_gain_per_yaw[theta];
      }

      if (best_yaw_score < yaw_score)
      {
        best_yaw_score = yaw_score;
        best_dynamic_yaw_score = dynamic_yaw_score;
        best_yaw = yaw;
      }
    }

    static_gain = best_yaw_score;
    dynamic_gain = best_dynamic_yaw_score;
    double yaw = M_PI * best_yaw / 180.f; //deg2rad
    state[3] = yaw;

    return std::make_tuple(static_gain, dynamic_gain, yaw);
  }
}


/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, double, Eigen::MatrixXd>> AEPlanner::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, double, Eigen::MatrixXd>> ellipses{};

  for (const auto& cov_matrix : cov_matrices)
  {
      // Validate matrix size before processing
      if (cov_matrix.rows() == 0 || cov_matrix.cols() == 0)
      {
          RCLCPP_ERROR(this->get_logger(), "createCovarianceEllipse: Empty covariance matrix, skipping");
          continue;
      }

      Eigen::MatrixXd cov_xyz;

      if (params_.sphere_bb)
      {
          // Extract full 3D covariance - need at least 3x3
          if (cov_matrix.rows() < 3 || cov_matrix.cols() < 3)
          {
              RCLCPP_ERROR(this->get_logger(), "createCovarianceEllipse: Covariance matrix too small for 3D (got %dx%d, need 3x3)", 
                          cov_matrix.rows(), cov_matrix.cols());
              continue;
          }
          cov_xyz = cov_matrix.block<3,3>(0,0);
      }
      else
      {
          // Extract 2D covariance and embed in 3×3 - need at least 2x2
          if (cov_matrix.rows() < 2 || cov_matrix.cols() < 2)
          {
              RCLCPP_ERROR(this->get_logger(), "createCovarianceEllipse: Covariance matrix too small for 2D (got %dx%d, need 2x2)", 
                          cov_matrix.rows(), cov_matrix.cols());
              continue;
          }
          cov_xyz = Eigen::MatrixXd::Zero(3,3);
          cov_xyz.block<2,2>(0,0) = cov_matrix.block<2,2>(0,0);
      }

      // Compute eigen decomposition
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov_xyz);
      Eigen::VectorXd eigenvalues  = solver.eigenvalues();   // sorted ascending
      Eigen::MatrixXd eigenvectors = solver.eigenvectors();

      // Radii of ellipse or ellipsoid
      double r1 = std::sqrt(eigenvalues(0));
      double r2 = std::sqrt(eigenvalues(1));
      double r3 = std::sqrt(eigenvalues(2));   // 0 in 2D case because we embed Z = 0

      ellipses.emplace_back(r1, r2, r3, eigenvectors);
  }

  return ellipses;
}


/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> AEPlanner::KFpredictTrajectories(double dt)
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian/UAV, we need to build a new Kalman filter
  int n = 0;
  int m = 0;
  if(params_.sphere_bb)
  {
    //Flying obstacles, x,y,z,vx,vy,vz
    n = 6; //Number of states
    m = 3; //Number of states we measure, x,y,z
  }
  else
  {
    //Walking obstacles, x,y,vx,vy
    n = 4;
    m = 2;
  }
  

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Observation matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  if(params_.sphere_bb)
  {
    //Flying obstacles, x,y,z,vx,vy
    A <<  1, 0, 0, dt, 0, 0,
          0, 1, 0, 0, dt, 0,
          0, 0, 1, 0, 0, dt,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1;

    //Observation matrix (only measure x and y position)
    //Flying obstacles, x,y,z,vx,vy,vz
    C << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;
   
    double q = params_.process_noise;  // acceleration variance

    Q << q*std::pow(dt,4)/4.0, 0,                 0,                 q*std::pow(dt,3)/2.0, 0,                 0,
        0,                 q*std::pow(dt,4)/4.0, 0,                 0,                 q*std::pow(dt,3)/2.0, 0,
        0,                 0,                 q*std::pow(dt,4)/4.0, 0,                 0,                 q*std::pow(dt,3)/2.0,
        q*std::pow(dt,3)/2.0, 0,                 0,                 q*std::pow(dt,2),       0,                 0,
        0,                 q*std::pow(dt,3)/2.0, 0,                 0,                 q*std::pow(dt,2),       0,
        0,                 0,                 q*std::pow(dt,3)/2.0, 0,                 0,                 q*std::pow(dt,2);
    

    R << params_.measurement_noise, 0, 0,
          0, params_.measurement_noise, 0,
          0, 0, params_.measurement_noise;

    P << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
  }
  else
  {
    //Walking obstacles, x,y,vx,vy
    A <<  1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    //Walking obstacles, x,y,vx,vy
    C << 1, 0, 0, 0,
         0, 1, 0, 0;


    //Process noise covariane (only used in predict to compensate for model uncertainty)
    double q = params_.process_noise;  //acceleration variance
    Q << q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0, 0,
        0,                       q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0,
        q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2),       0,
        0,                       q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2);
  
    // Measurement noise covariance (only used in update to compensate for sensor noise)
    R << params_.measurement_noise, 0,
    0, params_.measurement_noise;

  //Will be overwritten by P0 but needed to create filter.
  P << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1; 
  
  }

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  KalmanFilter kf;

  double t = 0;
  int safety_iteration = params_.safety_iteration; // Bounding covariance at iteration safety_iteration

  if(safety_iteration > params_.KFiterations)
  {
    safety_iteration = params_.KFiterations - 1;
  }
  
  for (const auto& dynamic_obstacle : dynamic_objects) 
  {
    const std::string& key = dynamic_obstacle.first;
    const geometry_msgs::msg::Pose& pose = dynamic_obstacle.second.first;
    const geometry_msgs::msg::Twist& twist = dynamic_obstacle.second.second;
    
    const double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
    const double vx = twist.linear.x, vy = twist.linear.y, vz = twist.linear.z;
    const double std_x = params_.position_uncertainty, std_y = params_.position_uncertainty, std_z = params_.position_uncertainty; //[m]
    const double std_vx = params_.velocity_uncertainty, std_vy = params_.velocity_uncertainty, std_vz = params_.velocity_uncertainty; //[m/s]
    
    std::vector<double> xcoords{};
    std::vector<double> ycoords{};
    std::vector<double> zcoords{};
    std::vector<Eigen::MatrixXd> covariance_matrices{};

    //Initalize kalman filter for the first time or from memory.
    if(kalman_filters.count(key) > 0)
    {

      kf = kalman_filters[key];
    
      //Perform one round of the Kalman filter (Update internal step one step forward in time)
      kf.predict();
      
      if(params_.sphere_bb)
      {
        y << xcoord, ycoord, zcoord;
      }
      else
      {
        y << xcoord, ycoord;
      }

      kf.update(y, dt, A, Q);
    }
    else
    {
      kf = KalmanFilter(dt, A, C, Q, R, P);
      
      if(params_.sphere_bb)
      {
        x0 << xcoord, ycoord, zcoord, vx, vy, vz;
        P0 << (std_x*std_x), 0, 0, 0, 0, 0,
            0, (std_y*std_y), 0, 0, 0, 0,
            0, 0, (std_z*std_z), 0, 0, 0,
            0, 0, 0, (std_vx*std_vx), 0, 0,
            0, 0, 0, 0, (std_vy*std_vy), 0,
            0, 0, 0, 0, 0, (std_vz*std_vz);
      }
      else
      {
        x0 << xcoord, ycoord, vx, vy;
        P0 << (std_x*std_x), 0, 0, 0,
            0, (std_y*std_y), 0, 0,
            0, 0, (std_vx*std_vx), 0,
            0, 0, 0, (std_vy*std_vy); 
      }      
      kf.init(t, x0, P0);
    }

    kalman_filters[key] = kf; //Save current state of xk and Pk

    Eigen::VectorXd x_hat(n);
    Eigen::MatrixXd P(n, n);

    x_hat = kf.state();
    P = kf.covariance();
    xcoords.push_back(x_hat(0));
    ycoords.push_back(x_hat(1));
    
    if(params_.sphere_bb)
    {
      zcoords.push_back(x_hat(2));
    }
    else
    {
      zcoords.push_back(zcoord);
    }

    covariance_matrices.push_back(P);
    
    for (int i = 0; i < params_.KFiterations; i++)
    {
      auto res = kf.predict(x_hat, P);
      x_hat = res.first;
      P = res.second;
      
      xcoords.push_back(res.first(0));
      ycoords.push_back(res.first(1));
      if(params_.sphere_bb)
      {
        zcoords.push_back(res.first(2));
      }
      else
      {
        zcoords.push_back(zcoord);
      }
      covariance_matrices.push_back(res.second);
    }

    kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices));
  }
  return kf_data;
}


std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> AEPlanner::linearPredictTrajectories(double dt)
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian/UAV, we need to build a new Kalman filter
  int n = 4; //Number of states
  int m = 2; //Number of states we measure

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Observation matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  A <<  1, 0, dt, 0, 
        0, 1, 0, dt, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

  //Observation matrix (only measure x and y position)
  C << 1, 0, 0, 0, 
       0, 1, 0, 0;

  //Process noise covariane (only used in predict to compensate for model uncertainty)

  double q = params_.process_noise;  //acceleration variance
  Q << q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0, 0,
      0,                       q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0,
      q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2),       0,
      0,                       q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2);
   
  
  // Measurement noise covariance (only used in update to compensate for sensor noise)
  R << params_.measurement_noise, 0,
        0, params_.measurement_noise;

  //Will be overwritten by P0 but needed to create filter.
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  Eigen::MatrixXd P_safety(n, n);

  KalmanFilter kf;

  double t = 0;
  int safety_iteration = params_.safety_iteration; // Bounding covariance at iteration safety_iteration

  if(safety_iteration > params_.KFiterations)
  {
    safety_iteration = params_.KFiterations - 1;
  }
  
  for (const auto& dynamic_obstacle : dynamic_objects) 
  {
    const std::string& key = dynamic_obstacle.first;
    const geometry_msgs::msg::Pose& pose = dynamic_obstacle.second.first;
    const geometry_msgs::msg::Twist& twist = dynamic_obstacle.second.second;
    
    const double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
    const double vx = twist.linear.x, vy = twist.linear.y;
    const double std_x = params_.position_uncertainty, std_y = params_.position_uncertainty; //[m]
    const double std_vx = params_.velocity_uncertainty, std_vy = params_.velocity_uncertainty; //[m/s]
    
    std::vector<double> xcoords{};
    std::vector<double> ycoords{};
    std::vector<double> zcoords{};

    //Init with certain size
    std::vector<Eigen::MatrixXd> covariance_matrices(params_.KFiterations + 1);

    //Initalize kalman filter for the first time or from memory.
    if(kalman_filters.count(key) > 0)
    {
      kf = kalman_filters[key];
    
      //Perform one round of the Kalman filter (Update internal step one step forward in time)
      kf.predict();
      y << xcoord, ycoord;
      kf.update(y, dt, A, Q);
    }
    else
    {
      kf = KalmanFilter(dt, A, C, Q, R, P);
      x0 << xcoord, ycoord, vx, vy;
      P0 << (std_x*std_x), 0, 0, 0,
          0, (std_y*std_y), 0, 0,
          0, 0, (std_vx*std_vx), 0,
          0, 0, 0, (std_vy*std_vy); 
      kf.init(t, x0, P0);
    }

    kalman_filters[key] = kf; //Save current state of xk and Pk

    Eigen::VectorXd x_hat(n);
    Eigen::MatrixXd P(n, n);

    x_hat = kf.state();
    P = kf.covariance();
    xcoords.push_back(x_hat(0));
    ycoords.push_back(x_hat(1));
    zcoords.push_back(zcoord);
    
    for (int i = 0; i < params_.KFiterations; i++)
    {
      auto res = kf.predict(x_hat, P);
      x_hat = res.first;
      P = res.second;
      
      xcoords.push_back(res.first(0));
      ycoords.push_back(res.first(1));
      zcoords.push_back(zcoord);

      if(i == safety_iteration)
      {
        P_safety = res.second;
      }
    }

    std::fill(covariance_matrices.begin(), covariance_matrices.end(), P_safety);
    kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices));
  }
  
  return kf_data;
}


std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> AEPlanner::circlePredictTrajectories(double dt)
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  int n = 4; //Number of states
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant covariance around object
  //Based on human-aware motion planning paper from mariusz
  P << 5.76, 0, 0, 0,
       0, 5.76, 0, 0,
       0, 0, 5.76, 0,
       0, 0, 0, 5.76; 
  
  for (const auto& dynamic_obstacle : dynamic_objects) 
  {
    const geometry_msgs::msg::Pose& pose = dynamic_obstacle.second.first;    
    const double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
 
    std::vector<double> xcoords{};
    std::vector<double> ycoords{};
    std::vector<double> zcoords{};
    std::vector<Eigen::MatrixXd> covariance_matrices{};
    
    xcoords.push_back(xcoord);
    ycoords.push_back(ycoord);
    zcoords.push_back(zcoord);
    covariance_matrices.push_back(P);

    kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices));
  }
  return kf_data;
}

void AEPlanner::dji_pose_callback(const geometry_msgs::msg::PoseStamped & msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  ///  current_state_[2] = msg.pose.position.z + 0.5; //Need to compensate for LIDAR sitting 0.5 meter above the base-link
  current_state_[2] = msg.pose.position.z + 0.5; //Need to compensate for LIDAR sitting 0.5 meter above the base-link
  current_state_[3] = get_yaw(msg.pose.orientation);
  current_state_initialized_ = true;
}


// Subscriptions
void AEPlanner::pose_callback(const geometry_msgs::msg::PoseStamped & msg) {
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = get_yaw(msg.pose.orientation);
  current_state_initialized_ = true;
}

void AEPlanner::odometry_callback(const nav_msgs::msg::Odometry & msg) {
  current_state_[0] = msg.pose.pose.position.x;
  current_state_[1] = msg.pose.pose.position.y;
  current_state_[2] = msg.pose.pose.position.z;
  current_state_[3] = get_yaw(msg.pose.pose.orientation);
  current_state_initialized_ = true;
}

void AEPlanner::octomapCallback(const octomap_msgs::msg::Octomap& msg)
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::fullMsgToMap(msg);
  if (aot == 0) {
    RCLCPP_ERROR(this->get_logger(), "empty octree");
    return;
  }
  octomap::OcTree* ot = dynamic_cast<octomap::OcTree*>(aot);
  if (ot == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to cast AbstractOcTree to OcTree");
    delete aot;
    return;
  }
  
  // Create shared_ptr from copy of the octree to ensure proper memory management
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  // Use octomap's proper delete function instead of raw delete
  delete ot;  // This should work, but if issues persist, use: deleteAbstractOcTree(aot);
  
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Freeing ot_ done:");
}

// Actions

// Aeplanner

rclcpp_action::GoalResponse AEPlanner::handle_aeplanner_goal(const rclcpp_action::GoalUUID & uuid,
                                                             std::shared_ptr<const Aeplanner::Goal> goal) {
  RCLCPP_ERROR(this->get_logger(), "Received goal request Aeplanner: %d", goal->actions_taken);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AEPlanner::handle_aeplanner_cancel(const std::shared_ptr<GoalHandleAeplanner> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel Aeplanner goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AEPlanner::handle_aeplanner_accepted(const std::shared_ptr<GoalHandleAeplanner> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&AEPlanner::execute_aeplanner, this, _1), goal_handle}.detach();
}


void AEPlanner::execute_aeplanner(const std::shared_ptr<GoalHandleAeplanner> goal_handle) {
  RCLCPP_ERROR(this->get_logger(), "Executing Aeplanner goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Aeplanner::Feedback>();

  auto result = std::make_shared<Aeplanner::Result>();
  
  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    RCLCPP_WARN(this->get_logger(), "Agent's pose not yet received");
    RCLCPP_WARN(this->get_logger(), "Make sure it is being published and correctly mapped");
    goal_handle->abort(result);
    return;
  }

  if (!ot_)
  {
    RCLCPP_WARN(this->get_logger(), "No octomap received");
    goal_handle->abort(result);    
    return;
  }

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> all_ellipses{};

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }
 
  RRTNode* root = initialize();

  rclcpp::Time start_time = this->get_clock()->now(); // Start time of expand RRT

  // Check if we have a previus branch
  if (best_branch_root_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Previous branch exists...");
    
    // Check if previous branch is collision free
    std::pair<RRTNode*, bool> old_branch = pathIsSafe(best_branch_root_, trajectories, all_ellipses);
    if(old_branch.second)
    {
      // Get alternative branch
      RCLCPP_INFO_STREAM(this->get_logger(), "Get alternative branch...");
      expandRRT(trajectories, all_ellipses);

      // Check if we got a valid candidate branch
      if(best_node_ && old_branch.first)
      {
        // Compare candidate goal with previous goal
        double old_score = old_branch.first->dynamic_score(params_.lambda, params_.zeta);
        double new_score = best_node_->dynamic_score(params_.lambda, params_.zeta);
        
        // if new goal is less than params_.best_branch_threshold % better, choose old branch
        if(new_score/old_score < params_.best_branch_threshold){
          RCLCPP_ERROR_STREAM(this->get_logger(), "Don't switch branch, follow old branch...");
          best_node_ = old_branch.first;

        }
      }
    }
    else 
    {
      // Old branch contains collision, search for a new branch
      RCLCPP_ERROR_STREAM(this->get_logger(), "previous branch contains collision");
      expandRRT(trajectories, all_ellipses);
    }
  }
  else 
  {
    //No previous branch exists, search for a new branch
    RCLCPP_ERROR_STREAM(this->get_logger(), "No previous branch available");    
    expandRRT(trajectories, all_ellipses);
  }

  //expand RRT took time
  rclcpp::Duration expand_rrt_time = this->get_clock()->now() - start_time;
  RCLCPP_ERROR_STREAM(this->get_logger(), "expand RRT took " << expand_rrt_time.seconds() << " seconds");

  // No valid nodes found in expandRRT --> Go into global planning
  if (best_node_ == NULL){
    
    RCLCPP_ERROR_STREAM(this->get_logger(), "GLOBAL PLANNING");
    result->is_clear = false;
    result->frontiers = getFrontiers();
    goal_handle->succeed(result);
    RCLCPP_ERROR(this->get_logger(), "Free resources...");
    delete root;
    kd_free(kd_tree_);
    RCLCPP_ERROR(this->get_logger(), "Free resources done!");
    return;
  }

  if(params_.debug_text) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Found plan in with local planner...");
  }
  
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  if(params_.debug_text)
    RCLCPP_INFO_STREAM(this->get_logger(), "Create visualization for Local Planner");

  // Clear previous markers
  visualization_msgs::msg::MarkerArray clear_array;
  visualization_msgs::msg::Marker clear_marker;

  clear_marker.header.frame_id = params_.world_frame;
  clear_marker.header.stamp = this->get_clock()->now();
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  clear_array.markers.push_back(clear_marker);
  daep_marker_array_pub_->publish(clear_array);

  daep_marker_array_pub_->publish(aep_viz->createRRTMarkerArray(root, 1.0, "rrtree", current_state_, expand_rrt_time.seconds())); // Visualize local RRT-tree
  daep_marker_array_pub_->publish(aep_viz->createRRTMarkerArray(best_branch_root_, 1.0, "best_branch", current_state_, expand_rrt_time.seconds())); // Visualize best-branch in local-tree

  if(params_.debug_text)
    RCLCPP_INFO_STREAM(this->get_logger(), "publishRecursive");
  
  // Add nodes to frontier
  publishEvaluatedNodesRecursive(root);

  // If pose is not a valid node (meaning that is in dynamic collision). Remain in current state and do a replan next time and hope the dynamic obstacle have moved
  if (best_branch_root_->children_[0]->dynamic_collision_ and params_.add_rejected_nodes)
  {
    if(params_.debug_text)
      RCLCPP_ERROR_STREAM(this->get_logger(), "Best node is in dynamic collision, stay in current state");
    result->pose.pose = vecToPose(current_state_);
    result->is_clear = true;
    result->frontiers = getFrontiers();
    goal_handle->succeed(result);
    RCLCPP_INFO_STREAM(this->get_logger(), "Deleting/Freeing!");
    delete root;
    kd_free(kd_tree_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Done!");
    return;
  }

  // Only one action along best branch is executed
  result->pose.pose = vecToPose(best_branch_root_->children_[0]->state_);
  
  // If we find a final node in the branch that has (an accumulated) dynamic score higher than the threshold, select it for exploration
  // This should already be handled in expandRRT to make sure it is fullfilled.
  if (best_node_->dynamic_score(params_.lambda, params_.zeta) > (params_.max_volume * params_.zero_gain_ratio))
  {
    result->is_clear = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "LOCAL PLANNING DONE");
  }
  else
  {
    // We could not find a node that is good enough locally, go into global planning
    RCLCPP_INFO_STREAM(this->get_logger(), "GO INTO GLOBAL PLANNING");
    result->is_clear = false;
    result->frontiers = getFrontiers();
    delete best_branch_root_;
    best_branch_root_ = NULL;
  }

  goal_handle->succeed(result);
  RCLCPP_INFO_STREAM(this->get_logger(), "Freeing resources...");
  delete root;
  kd_free(kd_tree_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Freeing resources done!");
  return;
}

/*
Give a reevaluate call from pig.py, reevaluate takes in a list of frontier (usually all frontiers)
for reevaluation. It computes the potential inoformation gain for each one explicitly and returns the list to pig.py.
*/
void AEPlanner::reevaluate(const std::shared_ptr<daep_msgs::srv::Reevaluate::Request> request,
                           std::shared_ptr<daep_msgs::srv::Reevaluate::Response> response) {

  for (std::vector<geometry_msgs::msg::Point>::iterator it = request->points.begin();
       it != request->points.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);

    //Compute the current gain in each cached node
    std::tuple <double, double, double> gain_response = gainCubature(pos, 0);
    RCLCPP_INFO_STREAM(this->get_logger(), "gain reeval expl: " << std::get<0>(gain_response));

    //Add the result for each node in a response list
    response->gain.push_back(std::get<0>(gain_response));
    response->dynamic_gain.push_back(std::get<1>(gain_response));
    response->yaw.push_back(std::get<2>(gain_response));
  }
}


/**
 * Subscribe on the dynamic_objects topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void AEPlanner::updateHumanPositions(const daep_msgs::msg::HumanListUpdate& msg) {


  if(msg.humans.empty())
  {
    //RCLCPP_WARN(this->get_logger(), "No humans in message");
    return;
  }

  // Get current message time from the first human (assumed to be the same for all)
  rclcpp::Time current_time = msg.humans[0].header.stamp;

  // Compute dt (make sure last_msg_time_ is initialized at first)
  dt_ = 0.0;
  if (last_msg_time_.nanoseconds() != 0) { // skip first message
    dt_ = (current_time - last_msg_time_).seconds();
  }

  // Update stored time
  last_msg_time_ = current_time;

  // ---------------------------------------------------------------------
  // 1. Clamp dt to prevent covariance explosion
  // 2. Reset filter if dt is absurdly large
  // ---------------------------------------------------------------------
  const double DT_CLAMP = 0.1;   // max allowed dt for stable CV model
  const double DT_RESET = 1.0;   // if we miss >1 sec, reset the filters

  if (dt_ > DT_RESET) {
      //RCLCPP_WARN(this->get_logger(),
      //            "Large dt (%.2f s) — resetting all human filters!", dt_);
      //resetAllHumanKalmanFilters();   // <-- You must implement this
      dt_ = DT_CLAMP;                 
  }
  else if (dt_ > DT_CLAMP) {
      //RCLCPP_WARN(this->get_logger(),
      //            "Large dt (%.2f s) — clamping to %.2f s", dt_, DT_CLAMP);
      dt_ = DT_CLAMP;
  }
  // ---------------------------------------------------------------------

  std::set<std::string> current_human_names;

  for(auto const& human : msg.humans)
  {
    if(human.name != "person_unknown")
    {

    // Check if the human is already in the map
    if (uuid_to_human_id_map.find(human.uuid) == uuid_to_human_id_map.end()) {
      uuid_to_human_id_map[human.uuid] = free_human_id;
      RCLCPP_INFO(this->get_logger(), "NEW HUMAN: %s - %s - %d", human.uuid.c_str(), human.name.c_str(), free_human_id);
      free_human_id += 1;
      dynamic_mode_ = true;
    }
    
    std::string model_name = human.name; // need to be unique...
    geometry_msgs::msg::Pose person_pose = human.pose;
    geometry_msgs::msg::Twist person_twist = human.twist;
    dynamic_objects[model_name] = std::make_pair(person_pose, person_twist);

    // Update last measurement time for this human
    last_measurement_time_[model_name] = current_time;
    current_human_names.insert(model_name);
    }
  }

  // Remove humans that haven't received measurements for more than timeout seconds
  std::vector<std::string> humans_to_remove;
  for (const auto& [model_name, last_time] : last_measurement_time_) {
    // Skip if this human is in the current message
    if (current_human_names.find(model_name) != current_human_names.end()) {
      continue;
    }
    
    // Check if timeout has been exceeded
    double time_since_last_measurement = (current_time - last_time).seconds();
    if (time_since_last_measurement > params_.human_measurement_timeout) {
      humans_to_remove.push_back(model_name);
    }
  }
  
  // Remove timed-out humans from all maps
  for (const auto& model_name : humans_to_remove) {
    dynamic_objects.erase(model_name);
    kalman_filters.erase(model_name);
    last_measurement_time_.erase(model_name);
    
    RCLCPP_INFO(this->get_logger(), "Removed human %s from tracking (timeout: %.2f seconds)", 
                  model_name.c_str(), params_.human_measurement_timeout);
  }

  // Visualize dynamic obstacle ground truth
  aep_viz->visualizePose(daep_marker_pub_, msg.humans, "dynamic_objects");

  // Perform prediction of human obstacles future positions 
  // using the chosen prediction mode

  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> new_vec;

  if(params_.prediction_mode == "kalman")
  {
    //RCLCPP_ERROR_STREAM(this->get_logger(), "Predicting Kalman trajectories");
    new_vec = KFpredictTrajectories(dt_);
  }
  else if(params_.prediction_mode == "linear")
  {
    //RCLCPP_ERROR_STREAM(this->get_logger(), "Predicting Linear trajectories");
    new_vec = linearPredictTrajectories(dt_);
  }
  else if(params_.prediction_mode == "circle")
  {
    //RCLCPP_ERROR_STREAM(this->get_logger(), "Predicting Circle trajectories");
    new_vec = circlePredictTrajectories(dt_);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid prediction mode: " << params_.prediction_mode);
    return;
  }
    
  { 
    // Race condition
    std::lock_guard<std::mutex> lock(vecMutex);
    predicted_data = new_vec;
  }
 
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>> all_ellipses{};

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }
  
  // Visualize covariance and predicted trajectoryf for dynamic obstacles
  if(params_.visualize_predictions)
    aep_viz->visualizePrediction(predictions_pub_, trajectories, all_ellipses, "predicted", dt_);
} 



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<AEPlanner>();
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
