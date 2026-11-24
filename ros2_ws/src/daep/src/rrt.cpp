#include "rrt.h"

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

///#include <tf2/utils.h>
///#include <cmath>

#include "octomap_msgs/conversions.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

Rrt::Rrt() : rclcpp::Node("rrt"), dt_(0.5), KFiterations_(15), min_nodes_(100), extension_range_(1.0),
             bounding_radius_(0.5), drone_linear_velocity(0.5), drone_angular_velocity(1.0),
             frame_id_("world"), dynamic_mode_(false) {

  declare_params();
  read_params();

  RCLCPP_INFO(this->get_logger(), "RRT INITIALIZED WITH WORLD FRAME: %s", frame_id_.c_str());
  
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  ot_ = std::make_shared<octomap::OcTree>(1);  // Create dummy OcTree to prevent crash due to ot_ tree not initialized

  last_msg_time_ = this->get_clock()->now();
    
  // publishers
  path_pub = this->create_publisher<visualization_msgs::msg::Marker>("rrt_path", 100);
  rrt_star_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_star_marker_array", 100);


  // subscriptions

  octomap_subscription = this->create_subscription<octomap_msgs::msg::Octomap>("octomap", qos,
                                                                               std::bind(&Rrt::octomapCallback,
                                                                                         this, std::placeholders::_1));

  human_update_subscription = this->create_subscription<daep_msgs::msg::HumanListUpdate>("dynamic_objects", 100,
                                                                                     std::bind(&Rrt::updateHumanPositions,
                                                                                               this, std::placeholders::_1));


  // services

  safe_path_service = this->create_service<daep_msgs::srv::SafePath>("/rrtplanner/safe_path", std::bind(&Rrt::service_safe_path, this, _1, _2));

  // Action server

  this->action_server_rrt = rclcpp_action::create_server<RrtAction>(
      this,
      "rrt",
      std::bind(&Rrt::handle_rrt_goal, this, _1, _2),
      std::bind(&Rrt::handle_rrt_cancel, this, _1),
      std::bind(&Rrt::handle_rrt_accepted, this, _1));  

}

Rrt::~Rrt() {

}

double Rrt::get_yaw(geometry_msgs::msg::Quaternion quat) {
  tf2::Quaternion current_quat;
  current_quat[0] = quat.x;
  current_quat[1] = quat.y;
  current_quat[2] = quat.z;
  current_quat[3] = quat.w;
  double roll, pitch, yaw;
  tf2::Matrix3x3(current_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

void Rrt::declare_params() {
  this->declare_parameter("world_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("rrt/min_nodes", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("system/bbx/r", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("rrt/extension_range", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("boundary/min", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("boundary/max", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("daep/kf/time_step", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/iterations", rclcpp::PARAMETER_INTEGER);  
  this->declare_parameter("drone_linear_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("drone_angular_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/position_uncertainty", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/velocity_uncertainty", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/process_noise", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/measurement_noise", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("daep/kf/safety_iteration", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("z_offset", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("human_height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/x", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/y", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/z", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("system/bbx/use_sphere_collision_checking", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("debug_text", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("rrt/location_tolerance", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("human_width", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("flying_obstacle_diameter", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("visualize_predictions", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("use_smoothing", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("smoothing_factor", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("filter_out_ground", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("ground_filter_z_dist", rclcpp::PARAMETER_DOUBLE);
}

void Rrt::read_params() {
  rclcpp::Parameter param;
  
  if (this->get_parameter("rrt/min_nodes", param)) {
    min_nodes_ = param.as_int();
  } else {
    min_nodes_ = 100;
    RCLCPP_WARN_STREAM(this->get_logger(), "No minimum nodes specified default is " << min_nodes_);    
  }
  
  if (this->get_parameter("system/bbx/r", param)) {
    bounding_radius_ = param.as_double();
  } else {
    bounding_radius_ = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No bounding radius specified default is " << bounding_radius_);    
  }
  
  if (this->get_parameter("rrt/extension_range", param)){
    extension_range_ = param.as_double();
  } else {
    extension_range_ = 1.0;
    RCLCPP_WARN_STREAM(this->get_logger(), "No extension_range_ specified default is " << extension_range_);    
  }

  if (this->get_parameter("boundary/min", param)) {
    boundary_min_ = param.as_double_array();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No boundary_min specified.");
  }
  if (this->get_parameter("boundary/max", param)) {
    boundary_max_ = param.as_double_array();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No boundary_max specified.");
  }
  if (this->get_parameter("daep/kf/time_step", param)) {
    dt_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No time_step specified. Default: " << dt_);
  }
  if (this->get_parameter("daep/kf/iterations", param)) {
    KFiterations_ = param.as_int();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No KFiterations specified. Default: " << KFiterations_);
  }
  if (this->get_parameter("drone_linear_velocity", param)) {
    drone_linear_velocity = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_linear_velocity specified. Default: " << drone_linear_velocity);
  }
  if (this->get_parameter("drone_angular_velocity", param)) {
    drone_angular_velocity = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No drone_angular_velocity specified. Default: " << drone_angular_velocity);
  }
  if (this->get_parameter("daep/kf/position_uncertainty", param)) {
    position_uncertainty_ = param.as_double();
  } else {
    position_uncertainty_ = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No position_uncertainty specified. Default: " << position_uncertainty_);
  }
  if (this->get_parameter("daep/kf/velocity_uncertainty", param)) {
    velocity_uncertainty_ = param.as_double();
  } else {
    velocity_uncertainty_ = 0.5;
    RCLCPP_WARN_STREAM(this->get_logger(), "No velocity_uncertainty specified. Default: " << velocity_uncertainty_);
  }

  if (this->get_parameter("daep/kf/process_noise", param)) {
    process_noise_ = param.as_double();
  } else {
    process_noise_ = 0.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No process_noise specified. Default: " << process_noise_);
  }

  if (this->get_parameter("daep/kf/measurement_noise", param)) {
    measurement_noise_ = param.as_double();
  } else {
    measurement_noise_ = 0.1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No measurement_noise specified. Default: " << measurement_noise_);
  }

  if (this->get_parameter("daep/kf/safety_iteration", param)) {
    safety_iteration_ = param.as_int();
  } else {
    safety_iteration_ = 1;
    RCLCPP_WARN_STREAM(this->get_logger(), "No safety_iteration specified. Default: " << safety_iteration_);
  }

  if (this->get_parameter("z_offset", param)) {
    z_offset = param.as_double();
  } else {
    z_offset = 0.025;
    RCLCPP_WARN_STREAM(this->get_logger(), "No z_offset specified. Default: " << z_offset);
  }

  if (this->get_parameter("system/bbx/x", param)) {
    x_diameter_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/x specified.");
  }

  if (this->get_parameter("system/bbx/y", param)) {
    y_diameter_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/y specified.");
  }

  if (this->get_parameter("system/bbx/z", param)) {
    z_diameter_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No /system/bbx/z specified.");
  }

  if (this->get_parameter("human_height", param)) {
    human_height_ = param.as_double();
  } else {
    human_height_ = 1.8;
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_height specified. Default: " << human_height_);
  }

  if (this->get_parameter("debug_text", param)) {
    debug_text_ = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No debug_text specified.");
  }

  if (this->get_parameter("visualize_predictions", param)) {
    visualize_predictions_ = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No visualize_predictions specified.");
  }

  if (this->get_parameter("system/bbx/use_sphere_collision_checking", param)) {
    use_sphere_collision_checking_ = param.as_bool();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No system/bbx/use_sphere_collision_checking_ specified.");
  }

  if (this->get_parameter("rrt/location_tolerance", param)) {
    location_tolerance_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No rrt/location_tolerance specified.");
  }

  if (this->get_parameter("human_width", param)) {
    human_width_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No human_width specified. Default");
  }

  if (this->get_parameter("flying_obstacle_diameter", param)) {
    flying_obstacle_diameter_ = param.as_double();
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "No flying_obstacle_diameter specified. Default");
  }

  if (this->get_parameter("use_smoothing", param)) {
    use_smoothing_ = param.as_bool();
  } else {
    use_smoothing_ = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No use_smoothing specified. Default: " << use_smoothing_);
  }

  if( this->get_parameter("smoothing_factor", param)) {
    smoothing_factor_ = param.as_double();
  } else {
    smoothing_factor_ = 0.9;
    RCLCPP_WARN_STREAM(this->get_logger(), "No smoothing_factor specified. Default: " << smoothing_factor_);
  }

  if(this->get_parameter("filter_out_ground", param))
  {
    filter_out_ground_ = param.as_bool();
  }
  else
  {
    filter_out_ground_ = false;
    RCLCPP_WARN_STREAM(this->get_logger(), "No filter_out_ground specified. Default: " << filter_out_ground_);
  }

  if(this->get_parameter("ground_filter_z_dist", param))
  {
    ground_filter_z_dist_ = param.as_double();
  }
  else
  {
    ground_filter_z_dist_ = 0.31; //m (Spot Max Step Height 0.3m)
    RCLCPP_WARN_STREAM(this->get_logger(), "No ground_filter_z_dist specified. Default: " << ground_filter_z_dist_);
  }

  if(this->get_parameter("world_frame", param))
  {
    frame_id_ = param.as_string();
  }
  else
  {
    frame_id_ = "world";
    RCLCPP_WARN_STREAM(this->get_logger(), "No world_frame specified. Default: " << frame_id_);
  }
}

rclcpp_action::GoalResponse Rrt::handle_rrt_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RrtAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request RrtAction");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Rrt::handle_rrt_cancel(const std::shared_ptr<GoalHandleRrtAction> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel RrtAction goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Rrt::handle_rrt_accepted(const std::shared_ptr<GoalHandleRrtAction> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Rrt::execute_rrt, this, _1), goal_handle}.detach();
}

/*
Search for the given goal (frontiers) with the Global Planner utilizing the RRT* search algorithm.
*/
void Rrt::execute_rrt(const std::shared_ptr<GoalHandleRrtAction> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing RrtAction goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<RrtAction::Feedback>();
  auto result = std::make_shared<RrtAction::Result>();  
  
  if (!ot_)
  {
    RCLCPP_WARN(this->get_logger(), "No octomap received");
    goal_handle->succeed(result);
    return;
  }

  if (!goal->goal_poses.poses.size())
  {
    RCLCPP_WARN(this->get_logger(), "No goals received");
    goal_handle->succeed(result);    
    return;
  }

  std::shared_ptr<octomap::OcTree> ot = ot_;

  if(filter_out_ground_)
  {
    ot = filterGroundFromOctomap(ot_, ground_filter_z_dist_, goal->start.pose.position);
  }

  //Get predictions of dynamic obstacles
  //auto predicted_data = KFpredictTrajectories(dt_);
  
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

   for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }
  if(debug_text_)
    RCLCPP_INFO(this->get_logger(), "Got predictions");

  std::vector<RrtNode*> found_goals;

  kdtree* kd_tree = kd_create(3);    // Initalize tree
  kdtree* goal_tree = kd_create(3);  // kd tree with all goals
  for (size_t i = 0; i < goal->goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(goal->goal_poses.poses[i].position.x,
                                             goal->goal_poses.poses[i].position.y,
                                             goal->goal_poses.poses[i].position.z);
    kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  }
  if(debug_text_)
    RCLCPP_INFO(this->get_logger(), "Inserted goals");

  // Initialize root position
  RrtNode* root = new RrtNode(goal->start.pose.position.x, goal->start.pose.position.y, goal->start.pose.position.z);
  Eigen::Vector3d root_dir (std::cos(get_yaw(goal->start.pose.orientation)), std::sin(get_yaw(goal->start.pose.orientation)), goal->start.pose.position.z);
  root->set_dir(root_dir.normalized());

  kd_insert3(kd_tree, root->x(), root->y(), root->z(), root);

  visualization_msgs::msg::MarkerArray nodes;
  octomap::OcTreeNode* ot_result;

  // Clear pub
  visualization_msgs::msg::MarkerArray deleteMarkers;
  visualization_msgs::msg::Marker deleteMarker;

  // Set the marker properties for deletion
  deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  deleteMarker.header.frame_id = frame_id_; // Use your frame ID
  deleteMarker.header.stamp = this->now();

  // Add the delete marker to the array
  deleteMarkers.markers.push_back(deleteMarker);
  rrt_star_pub->publish(deleteMarkers);
  if(debug_text_)
    RCLCPP_INFO(this->get_logger(), "Clear previuous RRT*-tree");

 
  RCLCPP_INFO(this->get_logger(), "Enter RRT* expansion"); 
  for (int i = 0; i < min_nodes_; ++i)
  {

    // Sample new position
    Eigen::Vector3d sampled_node = sample();
    // Get nearest neighbour
    RrtNode * nearest = chooseParent(kd_tree, sampled_node, extension_range_);
    if (!nearest)
      continue;

    // Calculate position for new node
    Eigen::Vector3d normalised_node = getNewPosNormalized(sampled_node, nearest->get_pos(), extension_range_);
    Eigen::Vector3d direction = normalised_node - nearest->get_pos();
    
    //###################### DAEP ########################

    // Static collision checking
      ot_result = ot->search(octomap::point3d(normalised_node[0], normalised_node[1], normalised_node[2]));
      if (ot_result == NULL)
        continue;

    // Dynamic Collision checking

    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost(drone_linear_velocity, drone_angular_velocity) + nearest->time_to_reach(normalised_node, drone_linear_velocity, drone_angular_velocity);

    // Check if the sampled node will collide in the future
    bool collision = checkCollision(time_to_reach_node, normalised_node, 
                                    trajectories, all_ellipses, nearest->get_pos());

    //###################### END DAEP #####################

    if(debug_text_)
    {
    RCLCPP_INFO_STREAM(this->get_logger(), "Checking node: " << normalised_node);
    RCLCPP_INFO_STREAM(this->get_logger(), "collisionLine? " << std::boolalpha << collisionLine(nearest->get_pos(), (normalised_node + direction.normalized()), bounding_radius_, use_sphere_collision_checking_, x_diameter_, y_diameter_, z_diameter_, ot));
    RCLCPP_INFO_STREAM(this->get_logger(), "dynamic collision? " << std::boolalpha << collision);
    RCLCPP_INFO_STREAM(this->get_logger(), "isInsideBoundaries? " << std::boolalpha << isInsideBoundaries(normalised_node));
    RCLCPP_INFO_STREAM(this->get_logger(), "ot_result (in known space)? " << std::boolalpha << ot_result);
    }
    
    if(!collisionLine(nearest->get_pos(), normalised_node, bounding_radius_, use_sphere_collision_checking_, x_diameter_, y_diameter_, z_diameter_, ot) and !collision and isInsideBoundaries(normalised_node) and ot_result)
    {
      // Add node to tree
      RrtNode* valid_node = addNodeToTree(kd_tree, nearest, normalised_node, (normalised_node - nearest->get_pos()).normalized());
      rewire(kd_tree, valid_node, extension_range_, bounding_radius_, goal->start.pose.position);
      nodes.markers.push_back(createNodeMarker(valid_node, i));

      // Check if goal has been reached
      RrtNode * tmp_goal = getGoal(goal_tree, valid_node, bounding_radius_, goal->start.pose.position);
      if (tmp_goal)
      {
        if(debug_text_)
          RCLCPP_INFO(this->get_logger(), "Found a goal!"); 
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  RCLCPP_INFO(this->get_logger(), "RRT* - Expansion complete, send to rpl_exploration..."); 

  rrt_star_pub->publish(nodes);

  std::pair<nav_msgs::msg::Path, std::vector<double>> path_and_time = getBestPath(found_goals);

  result->path = path_and_time.first;
  result->time_steps = path_and_time.second;

  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);

  goal_handle->succeed(result);
}

bool Rrt::isInsideBoundaries(Eigen::Vector3d point)
{  
  return point[0] > boundary_min_[0] and point[0] < boundary_max_[0] and
         point[1] > boundary_min_[1] and point[1] < boundary_max_[1] and
         point[2] > boundary_min_[2] and point[2] < boundary_max_[2];
}


std::shared_ptr<octomap::OcTree> Rrt::filterGroundFromOctomap(std::shared_ptr<octomap::OcTree> octree, double ground_filter_z_dist, geometry_msgs::msg::Point current_state)
{
    double robot_z = current_state.z;
    double ground_cutoff = robot_z - ground_filter_z_dist;

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
    
    RCLCPP_INFO(this->get_logger(), "Ground filtering: %d nodes updated to free, %d already free, %d total processed", 
                updated_count, already_free_count, nodes_to_filter.size());

    // Update the octree structure after modifications
    filtered_octree->updateInnerOccupancy();
    
    // Prune the tree to remove unnecessary nodes after marking as free
    filtered_octree->prune();

    return filtered_octree;
}


/*
Creates a ARROW-marker that points from node-parent() to node. Used
to visualize the Global Planner RRT*-tree.
*/
visualization_msgs::msg::Marker Rrt::createNodeMarker(RrtNode * node, int id){
  
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = get_clock()->now();
  marker.header.frame_id = frame_id_;
  marker.id = id;
  marker.ns = "rrt_expansion";
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  //Set start/end point for ARROW
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  start.x = node->get_parent()->x();
  start.y = node->get_parent()->y();
  start.z = node->get_parent()->z();
  end.x = node->x();
  end.y = node->y();
  end.z = node->z();
  marker.points.push_back(start);
  marker.points.push_back(end);

  marker.scale.x = 0.1;
  marker.scale.y = 0.15;
  marker.scale.z = 0;
  marker.color.r = 0.22; //Green
  marker.color.g = 1;
  marker.color.b = 0.161;
  marker.color.a = 0.5;
  marker.lifetime = rclcpp::Duration(5, 0);
  marker.frame_locked = false;

  return marker;
}


/*
Update the OctoMap ot.
*/
void Rrt::octomapCallback(const octomap_msgs::msg::Octomap& msg)
{
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::fullMsgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  if (ot) {
    delete ot;
  }
  //  RCLCPP_DEBUG_STREAM(this->get_logger(), "Freeing ot_ done:");
}


/*
Check if the comming nodes in the chosen path are safe (no collision with dynamic obtacles).
*/
void Rrt::service_safe_path(const std::shared_ptr<daep_msgs::srv::SafePath::Request> request,
                                std::shared_ptr<daep_msgs::srv::SafePath::Response> response) {

  //auto predicted_data = KFpredictTrajectories(dt_);
  RCLCPP_ERROR(this->get_logger(), "Service safe_path called and path size: %zu", request->path.poses.size());

  for(size_t i = 0; i < request->path.poses.size(); i++)
  {
    if(i == request->path.poses.size() - 1)
    {
      //Avoid to index outside of path
      break;
    }

    if(isCollision(request->path.poses[i], request->path.poses[i + 1], request->time_steps[i + 1], predicted_data))
    {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), 
        "Testing points: (" 
        << request->path.poses[i].pose.position.x << ", "
        << request->path.poses[i].pose.position.y << ", "
        << request->path.poses[i].pose.position.z << ") and ("
        << request->path.poses[i + 1].pose.position.x << ", "
        << request->path.poses[i + 1].pose.position.y << ", "
        << request->path.poses[i + 1].pose.position.z << ")"
    );
    
    RCLCPP_ERROR_STREAM(
        this->get_logger(), 
        "Service safe_path with response: " << isCollision(
            request->path.poses[i], 
            request->path.poses[i + 1], 
            request->time_steps[i + 1], 
            predicted_data
        )
    );
      response->safe = false;
      return;
    }
  }
  response->safe = true;
}


/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> Rrt::KFpredictTrajectories(double dt)
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian, we need to build a new Kalman filter
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

  //Process noise covariance (only used in predict to compensate for model uncertainty)

  double q = process_noise_;  //acceleration variance
  Q << q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0, 0,
      0,                       q * std::pow(dt, 4) / 4.0, 0,                       q * std::pow(dt, 3) / 2.0,
      q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2),       0,
      0,                       q * std::pow(dt, 3) / 2.0, 0,                       q * std::pow(dt, 2);
  
  // Measurement noise covariance (only used in update to compensate for sensor noise)
  R << measurement_noise_, 0,
        0, measurement_noise_;

  //Will be overwritten by P0 but needed to create filter.
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  KalmanFilter1 kf;

  double t = 0;
  int safety_iteration = safety_iteration_; // Bounding covariance at iteration safety_iteration

  if(safety_iteration > KFiterations_)
  {
    safety_iteration = KFiterations_ - 1;
  }

  for (const auto& dynamic_obstacle : dynamic_objects) 
  {
    const std::string& key = dynamic_obstacle.first;
    const geometry_msgs::msg::Pose& pose = dynamic_obstacle.second.first;
    const geometry_msgs::msg::Twist& twist = dynamic_obstacle.second.second;
    
    const double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
    const double vx = twist.linear.x, vy = twist.linear.y;
    const double std_x = position_uncertainty_, std_y = position_uncertainty_; //[m]
    const double std_vx = velocity_uncertainty_, std_vy = velocity_uncertainty_; //[m/s]

    std::vector<double> xcoords{};
    std::vector<double> ycoords{};
    std::vector<double> zcoords{};
    std::vector<Eigen::MatrixXd> covariance_matrices{};

    //Initalize kalman filter for the first time or from memory.
    if(kalman_filters_.count(key) > 0)
    {
      kf = kalman_filters_[key];
    
      //Perform one round of the Kalman filter (Update internal step one step forward in time)
      kf.predict();
      y << xcoord, ycoord;
      kf.update(y, dt, A, Q);
    }
    else
    {
      kf = KalmanFilter1(dt, A, C, Q, R, P);
      x0 << xcoord, ycoord, vx, vy;
      P0 << (std_x*std_x), 0, 0, 0,
            0, (std_y*std_y), 0, 0,
            0, 0, (std_vx*std_vx), 0,
            0, 0, 0, (std_vy*std_vy); 
      kf.init(t, x0, P0);
    }

    kalman_filters_[key] = kf; //Save current state of xk and Pk

    Eigen::VectorXd x_hat(n);
    Eigen::MatrixXd P(n, n);

    x_hat = kf.state();
    P = kf.covariance();
    xcoords.push_back(x_hat(0));
    ycoords.push_back(x_hat(1));
    zcoords.push_back(zcoord);
    covariance_matrices.push_back(P);

    for (int i = 0; i < KFiterations_; i++)
    {
      auto res = kf.predict(x_hat, P);
      x_hat = res.first;
      P = res.second;

      xcoords.push_back(res.first(0));
      ycoords.push_back(res.first(1));
      zcoords.push_back(zcoord);
      covariance_matrices.push_back(res.second);
    }

    kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices));
  }
  return kf_data;
}


/*
Check if there is gonna be a collision between posestamped_parent and posestamped at time time_of_arrival
utilizing the predicted_data (Prediction from Kalman Filter). Used to create a safe future path in the Global Planner.
*/
bool Rrt::isCollision(const geometry_msgs::msg::PoseStamped& posestamped_parent,
                          const geometry_msgs::msg::PoseStamped& posestamped, double time_of_arrival,
                          std::vector<std::pair<std::tuple<std::vector<double>,
                          std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data_input)
{

  RCLCPP_ERROR_STREAM(this->get_logger(), "predicted_data size: " << predicted_data_input.size());

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  //Extract each person's trajectory and covariance trajectory
  for(auto const& person : predicted_data_input)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  Eigen::Vector3d next_pose(posestamped.pose.position.x, 
                            posestamped.pose.position.y, 
                            posestamped.pose.position.z);

  Eigen::Vector3d next_pose_parent(posestamped_parent.pose.position.x, 
                                   posestamped_parent.pose.position.y, 
                                   posestamped_parent.pose.position.z);

  bool collision = checkCollision(time_of_arrival, next_pose, 
                                  trajectories, all_ellipses, next_pose_parent);

  return collision;
}

/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, Eigen::MatrixXd>> Rrt::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}

/* 
* Calculate if there is a potential collision with any moving obstacle in point at time t.
*/
bool Rrt::checkCollision(double t, 
                        Eigen::Vector3d point, 
                        std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                        std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                        Eigen::Vector3d parent)
{
  
  (void) parent;
  
  double max_time_step = KFiterations_ * dt_;
  int N_persons = trajectories.size();
  
  // Index the correct circle
  int covariance_index = getCovarianceIndex(max_time_step, dt_, t);

  if(covariance_index == -1)
  { 
    // Outside our prediction
    return false;
  }

  for(int i = 0; i < N_persons; i++)
  { 
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];
      //double radius = std::get<0>(covarianceEllipse);

      const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
      //if(lineIntersectsCircle(parent, point, center, radius))
      if(isCircleInsideEllipse(point, center, covarianceEllipse))
      {
        return true;
      }
  }
  return false;
}


/*
* Check intersection between cylinder at point and a cylinder at center.
*/
bool Rrt::isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
  std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse) 
{ 

  bool collision{};
  double circle_radius, covariance_height;

  if(use_sphere_collision_checking_)
  {
    circle_radius = std::get<0>(covariance_ellipse) + flying_obstacle_diameter_/2;
    covariance_height = flying_obstacle_diameter_;
  }
  else
  {
    // Extract covariance ellipse (cylinder bottom centered in center, cylinder (and bottom) as radius circle radius) and totalt height of cylinder from bottom is covariance_height
    circle_radius = std::get<0>(covariance_ellipse) + human_width_/2; // Blow up radius to include human width (Worst-case scenario)
    covariance_height = human_height_;
  }
  
  // Extract robot bounding box cylinder
  // This cylinder is centered in point with radius outer_radius and height robot_height
  double major_length, outer_radius, robot_height, agent_radius;
  if(use_sphere_collision_checking_)
  {
    agent_radius = flying_obstacle_diameter_ / 2.0;
  }
  else
  {
    major_length = std::max(x_diameter_, y_diameter_);
    outer_radius = (std::sqrt(2) * (major_length)) / 2.0;
    robot_height = z_diameter_;
  }

  if(use_sphere_collision_checking_)
  {
    // Sphere Collision check (Flying obstacles)
    // Calculate the 3D distance between sphere centers
    double distance = (point - center).norm();
    
    // Calculate sum of radii for the two spheres
    double radii_sum = circle_radius + agent_radius;
    
    // Spheres intersect if the distance between centers is less than or equal to the sum of radii
    collision = (distance <= radii_sum);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Spheres collision check: " << collision);
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
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cylinders collision in xy-plane: " << collision);
  }

  return collision;

}   


/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int Rrt::getCovarianceIndex(double max_time_step, double time_step, double t){
  
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


/*
Optimized function to check if a line between p1 and p2 intersects with a circle 
with certain center and radius.
*/
bool Rrt::lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                                   const Eigen::Vector3d center, double radius)
{
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double a = dx * dx + dy * dy;
  double b = 2 * (dx * (p1.x() - center.x()) + dy * (p1.y() - center.y()));
  double c = center.x() * center.x() + center.y() * center.y();
  c += p1.x() * p1.x() + p1.y() * p1.y();
  c -= 2 * (center.x() * p1.x() + center.y() * p1.y());
  c -= radius * radius;
  double discriminant = b * b - 4 * a * c;

  if(discriminant >= 0)
  {
    double t1 = (-b - sqrt(discriminant)) / (2*a);
    double t2 = (-b + sqrt(discriminant)) / (2*a);
  
    if((t1 >= 0 and t1 <= 1) or (t2 >= 0 and t2 <= 1))
    {
      return true;
    }
  }
  return false;

}


/*
This function randomizes three coordinates (x,y,z) and returns them as a point.
The point is sampled within the world bounding box.
NOTE: z-coordinate is hardcoded to the height of (ground) Robot.
*/
Eigen::Vector3d Rrt::sample()
{
  Eigen::Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
  {
    x_samp[i] = boundary_min_[i] + (((double)rand()) / ((double)RAND_MAX)) * (boundary_max_[i] - boundary_min_[i]);
  }

  if(!use_sphere_collision_checking_)
  {
    x_samp[2] = z_offset; //Force z-coordinate to be at the height of Turtle4
  }
  
  return x_samp;
}


/*
Choose the closest neighbour to a node given a kd_tree with extension range l and 
return that node as a suggestion on parent.
*/
RrtNode* Rrt::chooseParent(kdtree* kd_tree, Eigen::Vector3d node, double l)
{
  kdres* nearest = kd_nearest_range3(kd_tree, node[0], node[1], node[2], l); //Why + 0.5 here?
  if (kd_res_size(nearest) <= 0)
  {
    nearest = kd_nearest3(kd_tree, node[0], node[1], node[2]);
  }
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RrtNode* node_nn = (RrtNode*)kd_res_item_data(nearest);
  RrtNode* best_node = node_nn;
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (best_node and node_nn->cost() < best_node->cost())
      best_node = node_nn;

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}


/*
Extract the best (shortest) path to any of the found goals (frontiers) and visualize the path and the chosen frontier.
*/
std::pair<nav_msgs::msg::Path, std::vector<double>> Rrt::getBestPath(std::vector<RrtNode*> goals)
{

  std::vector<double> time_steps{};
  nav_msgs::msg::Path path;

  if (goals.size() == 0)
  {
    return std::make_pair(path, time_steps);
  }

  RrtNode* best_node = goals[0];

  for (size_t i = 0; i < goals.size(); ++i)
    if (best_node->cost() > goals[i]->cost())
      best_node = goals[i];

  RrtNode* n = best_node;

  //Extract path to the best node
  for (int id = 0; n->get_parent(); ++id)
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = n->x();
    p.pose.position.y = n->y();
    p.pose.position.z = n->z();
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    // Zero out rotation along
    // x and y axis so only
    // yaw is kept
    Eigen::Vector3d dir(n->x() - n->get_parent()->x(), n->y() - n->get_parent()->y(),
                        0);
    q.setFromTwoVectors(init, dir);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
    time_steps.push_back(n->time_cost(drone_linear_velocity, drone_angular_velocity)); //Add time it takes to reach this node from root
    n = n->get_parent();
  }

  std::reverse(time_steps.begin(), time_steps.end());
  std::reverse(path.poses.begin(), path.poses.end());

  visualizePath(best_node);
  visualizeChosenFrontier(best_node);
  return std::make_pair(path, time_steps);
}


/*
Visualize the chosen (best) path by the RRT*-planner (Global Planner) as a green path.
*/
void Rrt::visualizePath(RrtNode* node)
{
  // Create a marker array to hold all path segments
  visualization_msgs::msg::MarkerArray markerArray;
  
  // Add a delete marker to clear existing markers in the "best_path" namespace
  visualization_msgs::msg::Marker deleteMarker;
  deleteMarker.header.stamp = get_clock()->now();
  deleteMarker.header.frame_id = frame_id_;
  deleteMarker.ns = "best_path";
  deleteMarker.id = 9999;  // Use a high ID number that won't conflict with path markers
  deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  markerArray.markers.push_back(deleteMarker);
  
  // Create arrow markers for each segment of the path
  for (int id = 0; node->get_parent(); id++)
  {
    visualization_msgs::msg::Marker a;
    a.header.stamp = get_clock()->now();
    a.header.frame_id = frame_id_;
    a.id = id;
    a.ns = "best_path";
    a.type = visualization_msgs::msg::Marker::ARROW;
    a.action = visualization_msgs::msg::Marker::ADD;
    a.pose.position.x = node->get_parent()->x();
    a.pose.position.y = node->get_parent()->y();
    a.pose.position.z = node->get_parent()->z();
    
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->x() - node->get_parent()->x(),
                        node->y() - node->get_parent()->y(),
                        node->z() - node->get_parent()->z());
    q.setFromTwoVectors(init, dir);
    q.normalize();
    
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = dir.norm();
    a.scale.y = 0.07;
    a.scale.z = 0.07;
    a.color.r = 0.22; // Green
    a.color.g = 1;
    a.color.b = 0.161;
    a.color.a = 1.0;
    a.lifetime = rclcpp::Duration(0, 0);
    
    // Add the marker to the array
    markerArray.markers.push_back(a);
    
    node = node->get_parent();
  }
  
  // Publish the entire marker array at once on rrt_star_pub
  rrt_star_pub->publish(markerArray);
}


/*
Visualize the chosen frontier as a green sphere. Note that this 
sphere is not exactly located on the frontier that was chosen since 
we can be 1.5 meter from it.
*/
void Rrt::visualizeChosenFrontier(RrtNode* node)
{

     // First, publish a delete message to clear existing markers
     visualization_msgs::msg::Marker deleteMarker;
     deleteMarker.header.stamp = get_clock()->now();
     deleteMarker.header.frame_id = frame_id_;
     deleteMarker.ns = "chosen_frontier";
     deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;
     path_pub->publish(deleteMarker);  
  
    visualization_msgs::msg::Marker a;
    a.header.stamp = get_clock()->now();
    a.header.frame_id = frame_id_;
    a.id = 0;
    a.ns = "chosen_frontier";
    a.type = visualization_msgs::msg::Marker::SPHERE;
    a.action = visualization_msgs::msg::Marker::ADD;
    a.pose.position.x = node->x();
    a.pose.position.y = node->y();
    a.pose.position.z = node->z();
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->x() - node->get_parent()->x(),
                        node->y() - node->get_parent()->y(),
                        node->z() - node->get_parent()->z());
    q.setFromTwoVectors(init, dir);
    q.normalize();
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = 0.7;
    a.scale.y = 0.7;
    a.scale.z = 0.7;
    a.color.r = 0.22; //Green
    a.color.g = 1;
    a.color.b = 0.161;
    a.color.a = 1.0;
    a.lifetime = rclcpp::Duration(0, 0);
    a.frame_locked = false;

    path_pub->publish(a);
}

void Rrt::visualizePrediction(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                                       const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses,
                                       std::string ns                                       
                        ) 
{
  visualization_msgs::msg::MarkerArray line_marker_array;
  visualization_msgs::msg::MarkerArray ellipse_marker_array;
  visualization_msgs::msg::MarkerArray extended_ellipse_marker_array;
  
    int line_id = 0; // Line Marker ID counter
    int ellipse_id = 0; // Ellipse Marker ID counter
    int extended_ellipse_id = 0; // Extended Ellipse Marker ID counter
    int N_persons = all_ellipses.size();

    for (int i = 0; i < N_persons; i++){

      // Visualize trajectory
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = frame_id_;
      line_marker.header.stamp = get_clock()->now();
      line_marker.ns = ns + "_position";
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.id = line_id++;
      line_marker.scale.x = 0.1;  
      line_marker.color.r = 0.5;  
      line_marker.color.g = 0.4;  
      line_marker.color.b = 0.4;  
      line_marker.color.a = 1.0; 
      line_marker.pose.orientation.x = 0.0;
      line_marker.pose.orientation.y = 0.0;
      line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;

      // Extract trajectory and ellipses for a certain dynamic obstacle (person)
      const std::vector<double>& xCoords = std::get<0>(trajectories[i]);
      const std::vector<double>& yCoords = std::get<1>(trajectories[i]);
      const std::vector<double>& zCoords = std::get<2>(trajectories[i]);
      
      const std::vector<std::tuple<double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

      int N_trajectory_steps = xCoords.size();
      
      for (int j = 0; j < N_trajectory_steps; ++j) {
         
          // Visualize trajectory by filling a line with points
        geometry_msgs::msg::Point point;
        point.x = xCoords[j];
        point.y = yCoords[j];
        if(use_sphere_collision_checking_)
        {
          point.z = zCoords[j];
        }
        else
        {
          point.z = zCoords[j] + (human_height_/2);
        }
        line_marker.points.push_back(point);

        //Extract the values to build the ellipse
        double major_length = std::get<0>(ellipses[j]);
        double minor_length = std::get<1>(ellipses[j]);
        Eigen::MatrixXd eigenvectors = std::get<2>(ellipses[j]);

        //Visualize Ellipse
        visualization_msgs::msg::Marker ellipse_marker;
        ellipse_marker.header.frame_id = frame_id_;
        ellipse_marker.header.stamp = get_clock()->now();
        ellipse_marker.ns = ns + "_cov_ellipse";
        ellipse_marker.id = ellipse_id++;
        ellipse_marker.type = visualization_msgs::msg::Marker::SPHERE;
        ellipse_marker.pose.position.x = xCoords[j];
        ellipse_marker.pose.position.y = yCoords[j];
        ellipse_marker.pose.position.z = zCoords[j];
        ellipse_marker.color.a = 1.0f - (0.9f * j / (N_trajectory_steps - 1));
        ellipse_marker.color.r = 0.0;
        ellipse_marker.color.g = 0.0;
        ellipse_marker.color.b = 1.0;

        // Compute orientation quaternion of covariance ellipse
        Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
        q.x() = 0;
        q.y() = 0;
        q.normalize();
        ellipse_marker.pose.orientation.x = q.x();
        ellipse_marker.pose.orientation.y = q.y();
        ellipse_marker.pose.orientation.z = q.z();
        ellipse_marker.pose.orientation.w = q.w();

        // Give the marker the right dimensions according to the major and minor length
        ellipse_marker.scale.x = 2.0 * major_length;
        ellipse_marker.scale.y = 2.0 * minor_length;
        ellipse_marker.scale.z = 0.1;
        
        ellipse_marker_array.markers.push_back(ellipse_marker);

        // Visualize Extended Ellipse (with human width compensation)
        visualization_msgs::msg::Marker extended_ellipse_marker = ellipse_marker;
        extended_ellipse_marker.ns = ns + "_extended_cov_ellipse";
        extended_ellipse_marker.id = extended_ellipse_id++;
        extended_ellipse_marker.color.r = 0.0;
        extended_ellipse_marker.color.g = 1.0;
        extended_ellipse_marker.color.b = 1.0;
        
        // Extend the ellipse by human_width/2
        double max_major = std::max(major_length, minor_length);
        if(use_sphere_collision_checking_)
        {
          extended_ellipse_marker.scale.x = 2.0 * (max_major + flying_obstacle_diameter_/2);
          extended_ellipse_marker.scale.y = 2.0 * (max_major + flying_obstacle_diameter_/2);  
        }
        else
        {
          extended_ellipse_marker.scale.x = 2.0 * (max_major + human_width_/2);
          extended_ellipse_marker.scale.y = 2.0 * (max_major + human_width_/2);  
        }
        extended_ellipse_marker_array.markers.push_back(extended_ellipse_marker);
        
      }
      line_marker_array.markers.push_back(line_marker);    
      
      }      
    // Publish the marker arrays (all dynamic obstacles)
    marker_array_pub->publish(line_marker_array);
    marker_array_pub->publish(ellipse_marker_array);
    marker_array_pub->publish(extended_ellipse_marker_array);
}


/*
Given a sampled node and a parent, return the normalized vector that points from parent to sampled.
So l > direction > 0 in this case.
*/
Eigen::Vector3d Rrt::getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent,
                               double l)
{
  Eigen::Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}

/*
Add a new node to the kd_tree and connect it to its parent.
Note that we when we insert in the kd_tree, key == x,y,z and value == new_node.
*/
RrtNode* Rrt::addNodeToTree(kdtree* kd_tree, RrtNode* parent,
                            Eigen::Vector3d new_pos, Eigen::Vector3d dir)
{
  RrtNode* new_node = new RrtNode(parent);
  new_node->set_pos(new_pos);
  new_node->set_dir(dir);

  parent->add_child(new_node);
  kd_insert3(kd_tree, new_node->x(), new_node->y(), new_node->z(),
             new_node);

  return new_node;
}


/*
This function returns true if the voxel pt is deemed unknown by a majority vote among the neighbor voxels.
*/
bool Rrt::isSmoothedUnknown(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing) {
  if (!use_smoothing) {
    // strict: unknown if no node at this coordinate
    return (ot->search(pt) == nullptr);
  }

  // smoothing: check neighbors
  int unknown_count = 0;
  int neighbor_count = 0;

  double res = ot->getResolution();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        octomap::point3d neighbor(pt.x() + dx * res,
                                  pt.y() + dy * res,
                                  pt.z() + dz * res);
        octomap::OcTreeNode* n = ot->search(neighbor);
        if (n == nullptr) {
          unknown_count++;
        }
        neighbor_count++;
      }
    }
  }

  // Threshold: if majority unknown → treat as unknown
  double frac_unknown = static_cast<double>(unknown_count) / neighbor_count;
  return (frac_unknown > smoothing_factor_); // adjustable threshold
}


/*
This function returns true if the voxel pt is deemed occupied by a majority vote among the neighbor voxels.
*/
bool Rrt::isSmoothedOccupied(octomap::OcTree* ot, const octomap::point3d& pt, bool use_smoothing) {
  octomap::OcTreeNode* node = ot->search(pt);
  if (!node) return false; // unknown handled separately

  if (!use_smoothing) {
    return ot->isNodeOccupied(node);
  }

  // smoothing: majority vote among neighbors
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
  return (frac_occ > smoothing_factor_); // adjustable threshold
}


/*
This function should return true if there is collision with the octomap between p1 and p2, 
or if there are any unknown voxels in between p1 and p2 that disables movement.
If we use a bounding sphere, use r as the radius of the sphere.
If we use a bounding box, use x_diameter, y_diameter, z_diameter as the dimensions of the box.
*/
int Rrt::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r, bool use_sphere_collision_checking, double x_diameter, double y_diameter, double z_diameter, std::shared_ptr<octomap::OcTree> ot)
{

  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  //Use collision checking logic for a sphere
  if(use_sphere_collision_checking)
  { 
        
    if (debug_text_)
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

    // Travel cylinder collision checking
    double lsq_travel = (end - start).norm_sq();
    double rsq_travel = r * r;

    // First pass: check for unknown voxels inside start sphere, end sphere and travel cylinder
    for (float x = min.x(); x <= max.x(); x += ot->getResolution()) {
      for (float y = min.y(); y <= max.y(); y += ot->getResolution()) {
        for (float z = min.z(); z <= max.z(); z += ot->getResolution()) {
          octomap::point3d pt(x, y, z);

          bool inside_sphere_travel = CylTest_CapsFirst(start, end, lsq_travel, rsq_travel, pt) > 0;
          bool inside_start         = (start - pt).norm() <= r;
          bool inside_end           = (end - pt).norm() <= r;

          if (inside_sphere_travel || inside_start || inside_end) {
            if (isSmoothedUnknown(ot.get(), pt, use_smoothing_)) {
              if (debug_text_) {
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

      if (isSmoothedOccupied(ot.get(), voxel_center, use_smoothing_)) {
        
        if (CylTest_CapsFirst(start, end, lsq_travel, rsq_travel, voxel_center) > 0) {
          if (debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with travel cylinder");
          return 3;
        }

        if ((start - voxel_center).norm() <= r) {
          if (debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with start sphere");
          return 1;
        }

        if ((end - voxel_center).norm() <= r) {
          if (debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Occupied voxel collides with end sphere");
          return 2;
        }
      }
    }
  }
  else
  {
    if(debug_text_) 
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

    // First pass: check for any unknown voxels *inside* the cylinders
    for (float x = min.x(); x <= max.x(); x += ot->getResolution()) {
      for (float y = min.y(); y <= max.y(); y += ot->getResolution()) {
        for (float z = min.z(); z <= max.z(); z += ot->getResolution()) {
          octomap::point3d pt(x, y, z);

          bool inside_start   = CylTest_CapsFirst(start_bottom, start_top, lsq_start, rsq_start, pt) > 0;
          bool inside_end     = CylTest_CapsFirst(end_bottom, end_top, lsq_end, rsq_end, pt) > 0;
          bool inside_travel  = CylTest_CapsFirst(start_center_octomap, end_center_octomap, lsq_travel, rsq_travel, pt) > 0;

          if (inside_start || inside_end || inside_travel) {
            
            //octomap::OcTreeNode* node = ot->search(x, y, z);
            //if (node == nullptr) {
            if(isSmoothedUnknown(ot.get(), pt, use_smoothing_)){  
              
              if (debug_text_) {
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
      if(isSmoothedOccupied(ot.get(), voxel_center, use_smoothing_))
      {

        if(CylTest_CapsFirst(start_bottom, start_top, lsq_start, rsq_start, voxel_center) > 0)
        {
          if(debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node in collision with start cylinder");
          return 1;
        }

        if(CylTest_CapsFirst(end_bottom, end_top, lsq_end, rsq_end, voxel_center) > 0)
        {
          if(debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node in collision with end cylinder");
          return 2;
        }

        if(CylTest_CapsFirst(start_center_octomap, end_center_octomap, lsq_travel, rsq_travel, voxel_center) > 0)
        {
          if(debug_text_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Point in collision with travel cylinder");
          return 3;
        }
      }
    }  
  }

  // If we made it here, no static collision was detected
  if(debug_text_)
    RCLCPP_INFO_STREAM(this->get_logger(), "Sampled node not in static collision with OctoMap");
  return 0;
}


/*
Rewire the RRT* kd_tree.
*/
void Rrt::rewire(kdtree* kd_tree, RrtNode* new_node, double l, double r, geometry_msgs::msg::Point current_state)
{
  
  std::shared_ptr<octomap::OcTree> ot = ot_;

  if(filter_out_ground_)
  {
    ot = filterGroundFromOctomap(ot_, ground_filter_z_dist_, current_state);
  }

  RrtNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->x(), new_node->y(), new_node->z(), l); //+ 0.5 Why this here?
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (node_nn->cost() > new_node->cost() + (node_nn->get_pos() - new_node->get_pos()).norm())
    {
      if (!collisionLine(
                         new_node->get_pos(),
                         (node_nn->get_pos() + (node_nn->get_pos() - new_node->get_pos()).normalized()), r, use_sphere_collision_checking_, x_diameter_, y_diameter_, z_diameter_, ot))
        node_nn->set_parent(new_node);
    }
    kd_res_next(nearest);
  }
}


/*
Check if new_node is close enough to any frontier in the goal_tree. Here, new_node must be
at most 1.5 meter from a frontier for it to be considered as goal node.
*/
RrtNode* Rrt::getGoal(kdtree* goal_tree, RrtNode* new_node, double r, geometry_msgs::msg::Point current_state)
{
  
  std::shared_ptr<octomap::OcTree> ot = ot_;

  if(filter_out_ground_)
  {
    ot = filterGroundFromOctomap(ot_, ground_filter_z_dist_, current_state);
  }
  
  kdres* nearest_goal = kd_nearest3(goal_tree, new_node->x(), new_node->y(), new_node->z());
  
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return NULL;
  }

  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->get_pos()).norm() < location_tolerance_) //If goal node within 1.5 meter from valid node, we have found it.
    if (!collisionLine(new_node->get_pos(),
                       *g_nn + (*g_nn - new_node->get_pos()).normalized(), r, use_sphere_collision_checking_, x_diameter_, y_diameter_, z_diameter_, ot_))
      return new_node;

  return NULL;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James -
// gjames@NVIDIA.com Lisc:
// Free code - no warranty &
// no money back.  Use it all
// you want Desc:
//    This function tests if
//    the 3D point 'pt' lies
//    within an arbitrarily
// oriented cylinder.  The
// cylinder is defined by an
// axis from 'pt1' to 'pt2',
// the axis having a length
// squared of 'lsq'
// (pre-compute for each
// cylinder to avoid repeated
// work!), and radius squared
// of 'rsq'.
//    The function tests
//    against the end caps
//    first, which is cheap ->
//    only
// a single dot product to
// test against the parallel
// cylinder caps.  If the
// point is within these, more
// work is done to find the
// distance of the point from
// the cylinder axis.
//    Fancy Math (TM) makes
//    the whole test possible
//    with only two
//    dot-products
// a subtract, and two
// multiplies.  For clarity,
// the 2nd mult is kept as a
// divide.  It might be faster
// to change this to a mult by
// also passing in 1/lengthsq
// and using that instead.
//    Elminiate the first 3
//    subtracts by specifying
//    the cylinder as a base
// point on one end cap and a
// vector to the other end cap
// (pass in {dx,dy,dz} instead
// of 'pt2' ).
//
//    The dot product is
//    constant along a plane
//    perpendicular to a
//    vector. The magnitude of
//    the cross product
//    divided by one vector
//    length is
// constant along a cylinder
// surface defined by the
// other vector as axis.
//
// Return:  -1.0 if point is
// outside the cylinder
// Return:  distance squared
// from cylinder axis if point
// is inside.
//
//-----------------------------------------------------------------------------
float Rrt::CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                             float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy,
      dz;  // vector d  from
           // line segment
           // point 1 to point
           // 2
  float pdx, pdy,
      pdz;  // vector pd from
            // point 1 to test
            // point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate
                           // so pt1 is
                           // origin.
                           // Make vector
                           // from
  dy = pt2.y() - pt1.y();  // pt1 to
                           // pt2.  Need
                           // for this
                           // is easily
                           // eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from
                           // pt1 to test
                           // point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors
  // to see if point lies
  // behind the cylinder cap
  // at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero
  // the point is behind the
  // pt1 cap. If greater than
  // the cylinder axis line
  // segment length squared
  // then the point is outside
  // the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
    // Point lies within the
    // parallel caps, so find
    // distance squared from
    // point to line, using
    // the fact that sin^2 +
    // cos^2 = 1 the dot =
    // cos() * |d||pd|, and
    // cross*cross = sin^2 *
    // |d|^2 * |pd|^2
    // Carefull: '*' means
    // mult for scalars and
    // dotproduct for vectors
    // In short, where dist is
    // pt distance to cyl
    // axis: dist = sin( pd to
    // d ) * |pd| distsq = dsq
    // = (1 - cos^2( pd to d))
    // * |pd|^2 dsq = ( 1 -
    // (pd * d)^2 / (|pd|^2 *
    // |d|^2) ) * |pd|^2 dsq =
    // pd * pd - dot * dot /
    // lengthsq
    //  where lengthsq is d*d
    //  or |d|^2 that is
    //  passed into this
    //  function

    // distance squared to the
    // cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return
                     // distance
                     // squared
                     // to
                     // axis
    }
  }
}


/**
 * Subscribe on the dynamic_objects topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void Rrt::updateHumanPositions(const daep_msgs::msg::HumanListUpdate& msg) {

    // Get current message time from the first human (assumed to be the same for all)
    rclcpp::Time current_time = msg.humans[0].header.stamp;

  
    // Compute dt (make sure last_msg_time_ is initialized at first)
    dt_ = 0.0;
    if (last_msg_time_.nanoseconds() != 0) { // skip first message
      dt_ = (current_time - last_msg_time_).seconds();
    }
  
    // Update stored time
    last_msg_time_ = current_time;

  for(auto const& human : msg.humans)
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
  }

  // Perform prediction of human obstacles future positions 
  // using the Kalman Filter with Constant Velocity Model
  auto new_vec = KFpredictTrajectories(dt_);
  { 
    // Race condition
    std::lock_guard<std::mutex> lock(vecMutex);
    predicted_data = new_vec;
  }
 
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  // Visualize covariance and predicted trajectory
  if(visualize_predictions_)
    visualizePrediction(rrt_star_pub, trajectories, all_ellipses, "predicted");

} 


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<Rrt>();
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