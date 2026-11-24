#include "rpl_exploration.h"

#include "rclcpp/executor.hpp"

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

#include <tf2/LinearMath/Matrix3x3.h>

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>


using namespace std::placeholders;
using namespace std::chrono_literals;

int const GOAL_PUB_FREQ = 15;

RplExploration::RplExploration() : Node("rpl_exploration"), drone_freeze(false), goal_reached(false),
                                   prev_goal_reached(false), prev_avoidance(false), avoidance(false), have_pose(false),
                                   path_length(0.0), aep_have_result(false), rrt_have_result(false), new_plan(false),
                                   iteration_limit(1000000), is_docked_flag(false), have_dock_status(false),
                                   use_dock_station(false), world_frame("world"), husky(false) {


  this->declare_parameter("max_avoidance_counter", 12);
  this->declare_parameter("global_planner_counter", 25);
  this->declare_parameter("look_ahead_horizon", 5);
  this->declare_parameter("iteration_limit", 1000000);
  this->declare_parameter("coverage_threshold", 0.0);
  this->declare_parameter("coverage_stall_rounds", 0);
  this->declare_parameter("coverage_stall_gain", 1.0);
  this->declare_parameter("drone_freeze", false);
  this->declare_parameter("vicon", true);
  this->declare_parameter("dock_station", false);
  this->declare_parameter("world_frame", "world");
  this->declare_parameter("husky", false);

  client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  reentrant_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  max_avoidance_counter = this->get_parameter("max_avoidance_counter").as_int();
  global_planner_counter = this->get_parameter("global_planner_counter").as_int();
  look_ahead_horizon = this->get_parameter("look_ahead_horizon").as_int();
  iteration_limit = this->get_parameter("iteration_limit").as_int();
  coverage_threshold = this->get_parameter("coverage_threshold").as_double();
  coverage_stall_rounds = this->get_parameter("coverage_stall_rounds").as_int();
  coverage_stall_gain = this->get_parameter("coverage_stall_gain").as_double();
  drone_freeze = this->get_parameter("drone_freeze").as_bool();
  vicon = this->get_parameter("vicon").as_bool();
  use_dock_station = this->get_parameter("dock_station").as_bool();
  world_frame = this->get_parameter("world_frame").as_string();
  husky = this->get_parameter("husky").as_bool();
  RCLCPP_ERROR(this->get_logger(), "WORLD FRAME: %s", world_frame.c_str());

  goal_init_pub = this->create_publisher<std_msgs::msg::Bool>("/init_motion", 1);  
  goal_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);  
  goal_pub = this->create_publisher<daep_msgs::msg::Goal>("goal", 1);
  collision_pub = this->create_publisher<std_msgs::msg::Bool>("/write_log", 1);
  clock_start_pub = this->create_publisher<std_msgs::msg::Bool>("/clock_start", 1);
  exploration_completion_pub = this->create_publisher<std_msgs::msg::Bool>("/exploration_completed", 1);

  // clients
  coverage_client = create_client<daep_msgs::srv::Coverage>("get_coverage",
                                                             rmw_qos_profile_services_default,
                                                             client_cb_group);

  safe_path_client = create_client<daep_msgs::srv::SafePath>("/rrtplanner/safe_path",
                                                             rmw_qos_profile_services_default,
                                                             client_cb_group);


  rrt_action_client = rclcpp_action::create_client<daep_msgs::action::Rrt>(this, "rrt");

  aeplanner_action_client = rclcpp_action::create_client<daep_msgs::action::Aeplanner>(this, "make_plan");

  // subscriptions

  gr_sub = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 1,
                                                          std::bind(&RplExploration::goalReachedCallback,
                                                                    this, std::placeholders::_1));

  avoid_sub = this->create_subscription<std_msgs::msg::Bool>("/avoidance", 1,
                                                             std::bind(&RplExploration::avoidanceCallback,
                                                                       this, std::placeholders::_1));

  new_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/new_goal", 1,
                                                                            std::bind(&RplExploration::newGoalCallback,
                                                                                      this, std::placeholders::_1));

  pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("simple_pose", 1,
                                                                        std::bind(&RplExploration::poseCallback,

                                                                                  this, std::placeholders::_1));

  dji_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 1,
                                                                          std::bind(&RplExploration::poseCallbackDji,

                                                                                    this, std::placeholders::_1));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  dock_status_sub = this->create_subscription<irobot_create_msgs::msg::DockStatus>("/dock_status", qos,
                                                                        std::bind(&RplExploration::dock_status_callback,
                                                                                  this, std::placeholders::_1));

  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RplExploration::start_exploration, this));
}

RplExploration::~RplExploration() {

}


double RplExploration::get_yaw(geometry_msgs::msg::Quaternion quat) {
  tf2::Quaternion current_quat;
  current_quat[0] = quat.x;
  current_quat[1] = quat.y;
  current_quat[2] = quat.z;
  current_quat[3] = quat.w;
  double roll, pitch, yaw;
  tf2::Matrix3x3(current_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion RplExploration::get_quat(double yaw_rad) {
  tf2::Quaternion quat;
  tf2::Matrix3x3 mat;
  mat.setRPY(0, 0, yaw_rad);
  mat.getRotation(quat);
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.x = quat[0];
  quat_msg.y = quat[1];
  quat_msg.z = quat[2];
  quat_msg.w = quat[3];
  return quat_msg;
}


void RplExploration::rrt_send_goal(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseArray goal_poses) {

  rrt_have_result = false;

  if (!this->rrt_action_client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "RRT Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = RrtAction::Goal();
  goal_msg.start = start;
  goal_msg.goal_poses = goal_poses;

  RCLCPP_INFO(this->get_logger(), "RRT Sending goal");

  auto send_goal_options = rclcpp_action::Client<RrtAction>::SendGoalOptions();
  /// send_goal_options.goal_response_callback =
  /// std::bind(&RplExploration::rrt_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&RplExploration::rrt_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&RplExploration::rrt_result_callback, this, _1);
  this->rrt_action_client->async_send_goal(goal_msg, send_goal_options);  
}

void RplExploration::aep_send_goal(int actions_taken, int iteration) {

  aep_have_result = false;
  
  if (!this->aeplanner_action_client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "AEP Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = AepAction::Goal();
  goal_msg.header.stamp = get_clock()->now();
  goal_msg.header.frame_id = world_frame;
    ///     aep_goal.header.seq = iteration;
  goal_msg.actions_taken = actions_taken;

  RCLCPP_INFO(this->get_logger(), "Requesting goal from AEPlanner");

  auto send_goal_options = rclcpp_action::Client<AepAction>::SendGoalOptions();
  //send_goal_options.goal_response_callback =
  //std::bind(&RplExploration::aep_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&RplExploration::aep_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&RplExploration::aep_result_callback, this, _1);
  this->aeplanner_action_client->async_send_goal(goal_msg, send_goal_options);  

}

void RplExploration::rrt_goal_response_callback(std::shared_future<GoalHandleRrtAction::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "RRT Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "RRT Goal accepted by server, waiting for result");
  }
}

void RplExploration::aep_goal_response_callback(std::shared_future<GoalHandleAepAction::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "RRT Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "RRT Goal accepted by server, waiting for result");
  }
}

void RplExploration::rrt_feedback_callback(GoalHandleRrtAction::SharedPtr,
                                           const std::shared_ptr<const RrtAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "rrt_feedback_callback");
}

void RplExploration::rrt_result_callback(const GoalHandleRrtAction::WrappedResult & result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "got result");
  rrt_result = *result.result;
  rrt_have_result = true;
}


void RplExploration::aep_feedback_callback(GoalHandleAepAction::SharedPtr,
                                           const std::shared_ptr<const AepAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "aep_feedback_callback");
}

void RplExploration::aep_result_callback(const GoalHandleAepAction::WrappedResult & result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "got result");

  aep_result = *result.result;
  aep_have_result = true;
}


bool RplExploration::get_coverage(daep_msgs::srv::Coverage::Response & response) {
  RCLCPP_INFO(this->get_logger(), "get_coverage");

  while (!coverage_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "service available!");  

  auto req = std::make_shared<daep_msgs::srv::Coverage::Request>();
  
  auto result_future = coverage_client->async_send_request(req);
  
  std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Received response get_coverage");
    std::shared_ptr<daep_msgs::srv::Coverage::Response> resp = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Received response result: %f - %f", resp->occupied, resp->unmapped);
    response = *resp;
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Something went wrong with service call in get_coverage");
  }
  RCLCPP_ERROR(this->get_logger(), "RETURN false");
  return false;
}

std::shared_ptr<daep_msgs::srv::SafePath::Response> RplExploration::safe_path(nav_msgs::msg::Path path, std::vector<double> time_steps) {
  RCLCPP_INFO(this->get_logger(), "safe_path");

  while (!safe_path_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto req = std::make_shared<daep_msgs::srv::SafePath::Request>();

  // Set the data in the request
  req->path = path;
  req->time_steps = time_steps;
  
  auto result_future = safe_path_client->async_send_request(req);
  
  std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Received response safe_path");
    std::shared_ptr<daep_msgs::srv::SafePath::Response> response = result_future.get();
    /// RCLCPP_INFO(this->get_logger(), "Received response result: %f", response->yaw);
    return response;
  } else {
    RCLCPP_INFO(this->get_logger(), "Something went wrong with service call in get_coverage");
  }
  return 0;
}

void RplExploration::poseCallbackDji(const geometry_msgs::msg::PoseStamped & msg)
{
  ///RCLCPP_INFO(this->get_logger(), "poseCallback got msg");
  current_pose_stamped = msg;

  
  have_pose = true;
  exploration_completion_pub->publish(exploration_completed_msg);
  ///RCLCPP_INFO(this->get_logger(), "return from poseCallback");
}


void RplExploration::poseCallback(const geometry_msgs::msg::PoseStamped & msg)
{
  ///RCLCPP_INFO(this->get_logger(), "poseCallback got msg");
  current_pose_stamped = msg;
  have_pose = true;
  exploration_completion_pub->publish(exploration_completed_msg);
  ///RCLCPP_INFO(this->get_logger(), "return from poseCallback");
}

/*
Control when the motion planner has reached a goal
*/
void RplExploration::goalReachedCallback(const std_msgs::msg::Bool & msg)
{
  RCLCPP_INFO(this->get_logger(), "goalReachedCallback: %d", msg.data);
  goal_reached = msg.data;
  return;
  
    // Lock 'goal_reached' so that we don't send consecutive 'True' and miss a fly
    if(drone_freeze)
    {
      RCLCPP_INFO(this->get_logger(), "goalReachedCallback goal reachedset to true");
      goal_reached = true;
    }
    else
    {
      if (msg.data != prev_goal_reached)
      {
          goal_reached = msg.data;
          RCLCPP_INFO(this->get_logger(), "goalReachedCallback goal reached set to msg data");
          prev_goal_reached = goal_reached;
      }
    }
}

/*
Control if avoidance mode is activated
*/
void RplExploration::avoidanceCallback(const std_msgs::msg::Bool & msg)
{
    if (msg.data != prev_avoidance)
    { 
        avoidance = msg.data;
        prev_avoidance = avoidance;
    }
}

/*
Control if a new goal is set by avoidance mode
*/
void RplExploration::newGoalCallback(const geometry_msgs::msg::PoseStamped & msg)
{    
    if(msg.pose != prev_goal.pose)
    {
      prev_goal = new_goal;
      new_goal = msg;
    }
}

void RplExploration::dock_status_callback(const irobot_create_msgs::msg::DockStatus & msg) {
  is_docked_flag = msg.is_docked;
  have_dock_status = true;
}

void RplExploration::start_exploration() {
  this->timer_->cancel();
  std::thread{std::bind(&RplExploration::start_real_exploration, this)}.detach();
  RCLCPP_INFO(this->get_logger(), "Started exploration");
}


rclcpp::Duration RplExploration::move_to_pose(geometry_msgs::msg::Pose pose) {
  daep_msgs::msg::Goal goal;  
  goal.x = pose.position.x;
  goal.y = pose.position.y;
  goal.z = pose.position.z;
  double yaw = get_yaw(pose.orientation);
  if (yaw < 0) yaw += 2 * M_PI;
  goal.yaw = yaw;
  
  
  // Write path to file
  pathfile << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", aep" << std::endl;

  
  rclcpp::Time fly_start = get_clock()->now();
  
  // Continuously publish the goal pose until goal is reached
  goal.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
  publishGoalAndWait(GOAL_PUB_FREQ, goal);
  
  rclcpp::Duration fly_time = get_clock()->now() - fly_start;
  return fly_time;
}

void RplExploration::initial_motion(geometry_msgs::msg::Pose pose) {
  daep_msgs::msg::Goal goal;  

  // Perfoming initial motion (Turn 360 degrees to explore around robot)
  RCLCPP_INFO(this->get_logger(), "Performing initial motion: %d - %d - %d", is_docked_flag, use_dock_station, have_dock_status);
  
  // Used with Turtlebot dock station
  if (is_docked_flag || ((std::fabs(pose.position.x) < 0.1) && (std::fabs(pose.position.y) < 0.1))) {
    goal.x = -1.0;
  } else {
    goal.x = pose.position.x;
  }
  
  goal.y = pose.position.y;
  goal.z = pose.position.z;
  goal.yaw = 2.0;
  goal.initial_motion = true;
  goal.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
  publishGoalAndWait(GOAL_PUB_FREQ, goal);

  
  //goal.uuid = boost::uuids::to_string(boost::uuids::random_generator()());  
  //goal.yaw = 4.0;
  //publishGoalAndWait(GOAL_PUB_FREQ, goal);

  //goal.uuid = boost::uuids::to_string(boost::uuids::random_generator()());  
  //goal.yaw = get_yaw(pose.orientation);
  //publishGoalAndWait(GOAL_PUB_FREQ, goal);
}

void RplExploration::start_real_exploration() {

  exploration_completed_msg.data = false;


  // Get the path one directory up from current directory with "/data" appended
  std::string dataPath = std::filesystem::current_path() / "data";
  RCLCPP_INFO_STREAM(this->get_logger(), "data path: " << dataPath);

  // Check if the data directory exists, if not create it
  if (!std::filesystem::exists(dataPath)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Creating data directory: " << dataPath);
      std::filesystem::create_directory(dataPath);
  }

  // Define file paths
  std::string logfilePath = dataPath + "/logfile.csv";
  std::string pathfilePath = dataPath + "/path.csv";
  std::string coveragefilePath = dataPath + "/coverage.csv";

  // Check if files exist, create them if they don't
  for (const auto& filePath : {logfilePath, pathfilePath, coveragefilePath}) {
      if (!std::filesystem::exists(filePath)) {
          RCLCPP_INFO_STREAM(this->get_logger(), "Creating file: " << filePath);
          std::ofstream tempFile(filePath);
          tempFile.close();
      }
  }

  // Open the files
  logfile.open(logfilePath);
  pathfile.open(pathfilePath);
  coveragefile.open(coveragefilePath);

  
  // File Headers
  logfile << "Iteration, " << "Path length, " << "Time, " << "Planning, " << "Flying, " << std::endl;
  pathfile << "Goal x, " << "Goal y, " << "Goal z, " << "Planner" << std::endl;
  coveragefile << "Time, " << "Coverage (m3), " << "Coverage (%), " << "Free space, " << "Occupied Space, " << "Unmapped Space" << std::endl;

  daep_msgs::msg::Goal goal; // Goal sent to motion planner



  // Create a request message
  ///rrtplanner::SafePathRequest safe_path_request;

  std::shared_ptr<daep_msgs::srv::SafePath::Response> safe_path_response;
  // Call the service and store the response in 'res'
  ///aeplanner_evaluation::CoverageResponse srv_response;
  ///rrtplanner::SafePathResponse safe_path_response;


  // wait for aep server to start
  RCLCPP_INFO(this->get_logger(), "Waiting for aeplanner action server");
  aeplanner_action_client->wait_for_action_server();  // will wait for infinite time
  RCLCPP_INFO(this->get_logger(),"aeplanner action server started!");
  
  RCLCPP_INFO(this->get_logger(), "Waiting for rrt action server");
  rrt_action_client->wait_for_action_server();
  RCLCPP_INFO(this->get_logger(), "rrt Action server started!");

  RCLCPP_INFO(this->get_logger(), "Wait for pose");
  while (!have_pose) {
    sleep(1);
  }
  RCLCPP_INFO(this->get_logger(), "Got pose");

  geometry_msgs::msg::Pose last_pose = current_pose_stamped.pose;
  RCLCPP_INFO(this->get_logger(), "init position: x=%f, y=%f, z=%f, yaw=%f", last_pose.position.x,
              last_pose.position.y, last_pose.position.z, get_yaw(last_pose.orientation));
  
  RCLCPP_ERROR(this->get_logger(), "iteration_limit := %d", iteration_limit);

  // collision
  std_msgs::msg::Bool msg;
  msg.data = true;

  // Initial Coverage
  daep_msgs::srv::Coverage::Response coverage_response;
  if (get_coverage(coverage_response)) 
    {
        // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
        double coverage_m3 = coverage_response.coverage_m3;
        double coverage_p = coverage_response.coverage_p;
        double free_space = coverage_response.free;
        double occupied_space = coverage_response.occupied;
        double unmapped_space = coverage_response.unmapped;
        coveragefile << 0 << ", " << 0 << ", " <<  0 << ", " << 0 << ", "  << 0 << ", " <<  unmapped_space+occupied_space+free_space << std::endl;
    } 
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service get_coverage 1");
      return;
    }
  logfile << 0 << ", " 
          << 0 << ", "
          << 0 << ", "
          << 0 << ", "
          << 0 << std::endl;


  if (use_dock_station) {
    // wait for dock status
    while (!have_dock_status) {
      RCLCPP_INFO(this->get_logger(), "WAIT FOR DOCK STATUS");
      sleep(1);
    }
  }
  
  if(!husky)
  {
    initial_motion(last_pose);
  }

  int iteration = 1;
  int actions_taken = 1;

  rclcpp::Duration planning_time = rclcpp::Duration(0, 0);
  rclcpp::Duration elapsed = rclcpp::Duration(0, 0);

  double total_fly_time = 0.0;
  double total_planning_time = 0.0;

  //Start the Collision clock
  std_msgs::msg::Bool planning_started;
  planning_started.data = true;
  clock_start_pub->publish(planning_started);
  int global_counter = 0;

  rclcpp::Time start = get_clock()->now();

  bool ok_flag = true;
  std::vector<double> coverage_history;  // coverage_p after each iteration
  
  while (ok_flag)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning iteration " << iteration);
    
    rclcpp::Time planning_start = get_clock()->now();
    aep_send_goal(actions_taken, iteration);

   
    while(!aep_have_result) {
      usleep(50000);
    }

    /*
    RCLCPP_INFO(this->get_logger(), "Result aeplanner is_clear: %d", aep_result.is_clear);
    RCLCPP_INFO(this->get_logger(), "Result aeplanner pose: %f %f %f - %f", aep_result.pose.pose.position.x,
                aep_result.pose.pose.position.y, aep_result.pose.pose.position.z, get_yaw(aep_result.pose.pose.orientation)*180.0/M_PI);
    
    for (auto el : aep_result.frontiers.poses) {
      RCLCPP_INFO(this->get_logger(), "Result aeplanner frontiers: %f %f %f - %f", el.position.x, el.position.y, el.position.z,
                  get_yaw(el.orientation)*180.0/M_PI);
    }
    */

    rclcpp::Duration fly_time(0, 0);

    //Hack to force into global planning after one iteration
    //One iteration is needed to store some initial frontiers.
    if (aep_result.is_clear && iteration <= iteration_limit) {
      
      /* LOCAL PLANNER */
      RCLCPP_ERROR(this->get_logger(), "LOCAL PLANNER");
      planning_time = get_clock()->now() - planning_start; 
      geometry_msgs::msg::PoseStamped goal_pose = aep_result.pose;
      actions_taken = 0;
      fly_time = move_to_pose(goal_pose.pose);
      last_pose = goal_pose.pose;
    
    } else {

      /* GLOBAL PLANNER */
      RCLCPP_ERROR(this->get_logger(), "GLOBAL PLANNER");
      geometry_msgs::msg::PoseArray goal_poses;      
      geometry_msgs::msg::PoseStamped start;      
      start.header.stamp = get_clock()->now();
      start.header.frame_id = world_frame;
      start.pose = last_pose;

      RCLCPP_ERROR(this->get_logger(), "Number of cached frontiers: %i",  aep_result.frontiers.poses.size());


      if (!aep_result.frontiers.poses.size() or global_counter == global_planner_counter)
      {
        RCLCPP_WARN(this->get_logger(), "Exploration complete!");
        //Stop the path length timer for the last time
        break;
      }

      for (auto it = aep_result.frontiers.poses.begin();
           it != aep_result.frontiers.poses.end(); ++it)
      {
        goal_poses.poses.push_back(*it);
      }

      rrt_send_goal(start, goal_poses);

      //while (!rrt_ac.waitForResult(ros::Duration(0.05))){}

      while(!rrt_have_result) {
        usleep(50000);
      }

      nav_msgs::msg::Path path = rrt_result.path;
      std::vector<double> time_steps = rrt_result.time_steps;

      planning_time = get_clock()->now() - planning_start;

      if (path.poses.size() != 0)
      {
        global_counter = 0;
      }
      else
      {
        global_counter++;
      }

      rclcpp::Time fly_start = get_clock()->now();      

      for(int i = 0; i < path.poses.size(); i++)
      {

        //################ DAEP ##################

        if(path.poses.size() < look_ahead_horizon)
        {
          look_ahead_horizon = path.poses.size();
        }

        //If we are near the end, make sure the subpath and sub-time-steps does
        //not go out of bounds, instead we should look at the remaining poses
        int size_of_remaining_path = path.poses.size() - i;
        if(size_of_remaining_path < look_ahead_horizon)
        {
          look_ahead_horizon--;
        }
        
        //Extract correct future subpath
        nav_msgs::msg::Path subpath;
        std::vector<geometry_msgs::msg::PoseStamped> subposes(path.poses.begin() + i, path.poses.begin() + i + look_ahead_horizon);
        subpath.poses = subposes;

        //Extract correct future time stamps
        std::vector<double> sub_time_steps(time_steps.begin() + i, time_steps.begin() + i + look_ahead_horizon);

        std::shared_ptr<daep_msgs::srv::SafePath::Response> resp = safe_path(subpath, sub_time_steps);        

        if (resp) 
        {
          RCLCPP_ERROR(this->get_logger(), "safe_path: %d", resp->safe);
          if(not resp->safe)
          {
            //Return to the local planner
            break;
          }
        } 
        else 
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to call service /rrtplanner/safe_path");
          return;
        }

        double time_to_remove = time_steps[i];
        for(auto& element : time_steps)
          {
            if(i <= time_steps.size() - 1)
              {
                element = element - time_to_remove;
              }
          }   

        //############## END DAEP ###############


        geometry_msgs::msg::Pose goal_pose = path.poses[i].pose;
        last_pose = goal_pose;

        // Write path to file
        pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", "
                 << goal_pose.position.z << ", rrt" << std::endl;

        goal.x = goal_pose.position.x;
        goal.y = goal_pose.position.y;
        goal.z = goal_pose.position.z;
        double yaw = get_yaw(goal_pose.orientation);
        if (yaw < 0) {
          yaw += 2 * M_PI;
        }
        goal.yaw = yaw;

        // Continuously publish the goal pose until goal is reached
        goal.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
        publishGoalAndWait(GOAL_PUB_FREQ, goal);
        if(new_plan) break;
      }
      fly_time = get_clock()->now() - fly_start;
      actions_taken = -1;
    }

    elapsed = get_clock()->now() - start;
    total_fly_time += fly_time.seconds();
    total_planning_time += planning_time.seconds();

    
    logfile << iteration << ", " 
            << path_length << ", "    
            << elapsed.seconds() << ", "
            << total_planning_time << ", "
            << total_fly_time << std::endl;


    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Iteration: "       << iteration << "  " <<
                       "Path Length: "       << path_length << "  " <<
                       "Time: "            << elapsed.seconds() << "  " <<
                       "Planning: "        << total_planning_time << "  " <<
                       "Flying: "          << total_fly_time);


    iteration++;
    collision_pub->publish(msg);

    daep_msgs::srv::Coverage::Response response;
    
    if (get_coverage(response)) {
      // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
      double coverage_m3 = response.coverage_m3;
      double coverage_p = response.coverage_p;
      double free_space = response.free;
      double occupied_space = response.occupied;
      double unmapped_space = response.unmapped;
      coveragefile << elapsed.seconds() << ", " <<coverage_m3 << ", " <<  coverage_p << ", " << free_space << ", "  << occupied_space << ", " <<  unmapped_space << std::endl;
      if (coverage_threshold > 0.0 && coverage_p >= coverage_threshold) {
        RCLCPP_WARN(this->get_logger(), "Coverage %.1f %% reached the threshold (%.1f %%), exploration complete!",
                    coverage_p, coverage_threshold);
        break;
      }
      // Areas with obstacles never reach the threshold (inside and behind them cannot be
      // seen), so also stop when the coverage has stopped growing.
      coverage_history.push_back(coverage_p);
      int n = coverage_stall_rounds;
      if (n > 0 && static_cast<int>(coverage_history.size()) > n &&
          coverage_p - coverage_history[coverage_history.size() - 1 - n] < coverage_stall_gain) {
        RCLCPP_WARN(this->get_logger(), "Coverage %.1f %% grew less than %.1f percentage points in %d rounds, exploration complete!",
                    coverage_p, coverage_stall_gain, n);
        break;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service get_coverage 2");
    }
  }

  
  //Extract the current time and calculate the distance flown
  // and add it to the extra column
  logfile << iteration-1 << ", " 
          << path_length << ", "
          << elapsed.seconds() << ", "
          << total_planning_time << ", "
          << total_fly_time << std::endl;

  if(get_coverage(coverage_response)) {
        // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
        double coverage_m3 = coverage_response.coverage_m3;
        double coverage_p = coverage_response.coverage_p;
        double free_space = coverage_response.free;
        double occupied_space = coverage_response.occupied;
        double unmapped_space = coverage_response.unmapped;
        coveragefile << elapsed.seconds() << ", " << coverage_m3 << ", " <<  coverage_p << ", " << free_space << ", "  << occupied_space << ", " <<  unmapped_space << std::endl;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service get_coverage 3");
  }

  // Completion
  collision_pub->publish(msg);

  exploration_completed_msg.data = true;
  exploration_completion_pub->publish(exploration_completed_msg);

  pathfile.close();
  logfile.close();
  coveragefile.close();
}


/*
  Publishes goal continously and waits for motion planner /goal_reached topic.
*/
void RplExploration::publishGoalAndWait(int const& freq, daep_msgs::msg::Goal const& goal) {

  RCLCPP_INFO(this->get_logger(), "publishGoalAndWait:: %f %f %f - %f - %s", goal.x, goal.y, goal.z, goal.yaw*180.0/M_PI, goal.uuid.c_str());
 
  int avoidance_counter = 0;
  if(drone_freeze)
  {
    return;
  }

  geometry_msgs::msg::Pose start_pose = current_pose_stamped.pose;

  geometry_msgs::msg::Pose last_pose = current_pose_stamped.pose;

  ///  ros::Rate rate(freq);

  bool loop_flag = true;

  if (goal.initial_motion) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    goal_init_pub->publish(msg);
  } else {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = world_frame;
    goal_pose.pose.position.x = goal.x;
    goal_pose.pose.position.y = goal.y;
    goal_pose.pose.position.z = goal.z;
    goal_pose.pose.orientation = get_quat(goal.yaw);
    goal_pose_pub->publish(goal_pose);

    double x0 = current_pose_stamped.pose.position.x;
    double y0 = current_pose_stamped.pose.position.y;
    double z0 = current_pose_stamped.pose.position.z;

    double x1 = goal.x;
    double y1 = goal.y;
    double z1 = goal.z;

    double dx = x1-x0;
    double dy = y1-y0;
    double dz = z1-z0;

    double dist2 = sqrt(dx*dx + dy*dy);
    double dist = sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO(this->get_logger(), "Start %f %f %f", x0, y0, z0);    
    RCLCPP_INFO(this->get_logger(), "End %f %f %f", x1, y1, z1);    
    RCLCPP_INFO(this->get_logger(), "Dist dist2d: %f %f", dist, dist2);    
  }
   

  while (loop_flag) {

    // Publish the goal pose
    goal_pub->publish(goal);
    new_plan = false;

#if 0    
    if(avoidance) { 
      //Avoidance mode is active, a new goal has been published
      if(new_goal.pose != prev_goal.pose) {
        avoidance_counter++;
        
        //To avoid oscillative behaviour and getting stuck, break the avoidance and
        //replan after a certain amount of tries
        if (avoidance_counter > max_avoidance_counter) { 
          rpl_exploration::Goal stop_goal;
          add_distance(last_pose, current_pose_stamped.pose);
          stop_goal.x = current_pose_stamped.pose.position.x;
          stop_goal.y = current_pose_stamped.pose.position.y;
          stop_goal.z = current_pose_stamped.pose.position.z;
          goal_pub.publish(stop_goal);   
          new_plan = true;    
          break;
        }
        //If the new goal differs from the previous one, we add it to the path length
        //This to make sure we add each new goal only once
        add_distance(last_pose, current_pose_stamped.pose);
        add_distance(current_pose_stamped.pose, new_goal.pose);
        prev_goal = new_goal;
        last_pose = new_goal.pose;
      }
    }
#endif      
    // Check if the goal has been reached
    if (goal_reached) {
      geometry_msgs::msg::Pose final_pose;
      final_pose.position.x = goal.x;
      final_pose.position.y = goal.y;
      final_pose.position.z = goal.z;
      
      if(start_pose == last_pose) {
        //No avoidance was needed, add the straight path
        //from start pose to final pose
        add_distance(start_pose, final_pose);
      } else {
        //Avoidance was needed, and the final distance traveled
        //from the latest pose to the final pose
        add_distance(last_pose, final_pose);          
      }
      
      // Stop publishing the goal pose
      loop_flag = false;
    }
    usleep(100000);
  }
  goal_reached = false;
}

// Calculate distance between two poses and add to the dynamic path length
void RplExploration::add_distance(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  double distance = sqrt((dx*dx) + (dy*dy) + (dz*dz));
  path_length += distance;
}



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<RplExploration>();
  auto exec_args = rclcpp::ExecutorOptions();
  
  rclcpp::executors::MultiThreadedExecutor executor(exec_args, 8);
  executor.add_node(client_node);

  RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;

  //  rclcpp::spin(std::make_shared<ManagerM300>());
  //  rclcpp::shutdown();
  
}
