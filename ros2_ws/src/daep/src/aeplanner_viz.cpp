#include "aeplanner_viz.h"

#include <iostream>
#include <chrono>


AEPlannerViz::AEPlannerViz(AEPlanner * aep, std::string world_frame) : aep_node(aep), world_frame_(world_frame) {

}

AEPlannerViz::~AEPlannerViz() {

}


/*
Creates markers for the rays in information gain estimation.
*/
visualization_msgs::msg::Marker AEPlannerViz::createRayMarker(int ray_id, std::string color)
{
    // Visualize trajectory
  visualization_msgs::msg::Marker ray;
    ray.header.frame_id = world_frame_;
    ray.header.stamp = aep_node->get_clock()->now();   
    ray.ns = "ray";
    ray.action = visualization_msgs::msg::Marker::ADD;
    ray.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ray.id = ray_id;
    ray.scale.x = 0.05;  
    if(color == "BLUE")
    {
      ray.color.r = 0.0;  
      ray.color.g = 0.0;  
      ray.color.b = 1.0;  
    }
    else
    {
      //Must be RED
      ray.color.r = 1.0;  
      ray.color.g = 0.0;  
      ray.color.b = 0.0;  
    }
    ray.color.a = 1.0; 
    ray.pose.orientation.x = 0.0;
    ray.pose.orientation.y = 0.0;
    ray.pose.orientation.z = 0.0;
    ray.pose.orientation.w = 1.0;
    return ray;
}

  /*
  Visualizes the old best branch from the Local Planner (RRT-tree). This is to compare witht he previous best one to see if they differ.
  */
  void AEPlannerViz::VisualizeOldPath(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr old_path_pub, RRTNode* root) {
  
  visualization_msgs::msg::MarkerArray markerArray;
    
    // Create a line strip marker
  visualization_msgs::msg::Marker line_strip_marker;
    line_strip_marker.header.frame_id = world_frame_;  
    line_strip_marker.header.stamp = aep_node->get_clock()->now();   
    line_strip_marker.ns = "rrt_line_strip";
    line_strip_marker.action = visualization_msgs::msg::Marker::ADD;
    line_strip_marker.pose.orientation.w = 1.0;           
    line_strip_marker.id = 0;
    line_strip_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_marker.scale.x = 0.05;                     
    line_strip_marker.color.r = 0.961;      
    line_strip_marker.color.g = 0.596;  
    line_strip_marker.color.b = 0.137;             
    line_strip_marker.color.a = 1.0;                  
    
    // Loop until root
    RRTNode* curr = root;
    while (!curr->children_.empty()) {
        // Add the state point to the line strip marker
      geometry_msgs::msg::Point point;
        point.x = curr->state_(0);
        point.y = curr->state_(1);
        point.z = curr->state_(2);
        line_strip_marker.points.push_back(point);
        
        // Move to the child node
        curr = curr->children_[0];
    }
    
    // Add the line strip marker to the marker array
    markerArray.markers.push_back(line_strip_marker);
    
    // Publish the marker array
    old_path_pub->publish(markerArray);
}


/**
 * Visualize the ghost pedestrian that will block our view in the future.
*/
  void AEPlannerViz::visualizeGhostPedestrian(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ghost_marker_pub, std::vector<std::tuple<double, double, double>> positions) 
{

  aeplanner::Params params_ = aep_node->get_params();
  
  visualization_msgs::msg::MarkerArray ghosts;
    int id = 0;
    for(auto const& pos : positions)
    {
      // Create visualization marker
      visualization_msgs::msg::Marker ghost;
      ghost.header.frame_id = world_frame_;  
      ghost.header.stamp = aep_node->get_clock()->now();
      ghost.ns = "dynamic_objects";
      ghost.id = id++;
      ghost.action = visualization_msgs::msg::Marker::ADD;
      ghost.pose.position.x = std::get<0>(pos);
      ghost.pose.position.y = std::get<1>(pos);

      if(params_.sphere_bb)
      {
        ghost.type = visualization_msgs::msg::Marker::SPHERE;
        ghost.pose.position.z = std::get<2>(pos);
        ghost.scale.x = params_.flying_obstacle_diameter;  
        ghost.scale.y = params_.flying_obstacle_diameter;  
        ghost.scale.z = params_.flying_obstacle_diameter; 

      }
      else
      {
        ghost.type = visualization_msgs::msg::Marker::CUBE;
        ghost.pose.position.z = std::get<2>(pos)  + (params_.human_height/2);
        ghost.scale.x = params_.human_width;  
        ghost.scale.y = params_.human_width;  
        ghost.scale.z = params_.human_height; 
      }

      //GREY
      ghost.color.r = 0.769;  
      ghost.color.g = 0.769;  
      ghost.color.b = 0.769;
      ghost.color.a = 1.0;

      ghost.pose.orientation.x = 0.0;
      ghost.pose.orientation.y = 0.0;
      ghost.pose.orientation.z = 0.0;
      ghost.pose.orientation.w = 1.0;

      ghosts.markers.push_back(ghost);
    }
    
    // Publish the marker
    ghost_marker_pub->publish(ghosts);
}

/**
 * This function visualizes the mean trajectory (line) for each dynamic obstacle
 * as well as the corresponding covariance ellipses corresponding to the mean trajectory.
 * This visualization can be seen in RViz.
*/
void AEPlannerViz::visualizePrediction(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub, 
  const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                const std::vector<std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>>& all_ellipses,
                std::string ns, double dt) 
{
visualization_msgs::msg::MarkerArray line_marker_array;
visualization_msgs::msg::MarkerArray ellipse_marker_array;
visualization_msgs::msg::MarkerArray extended_ellipse_marker_array;
visualization_msgs::msg::MarkerArray time_text_marker_array; 
aeplanner::Params params_ = aep_node->get_params();

int line_id = 0; // Line Marker ID counter
int ellipse_id = 0; // Ellipse Marker ID counter
int extended_ellipse_id = 0; // Extended Ellipse Marker ID counter
int time_text_id = 0; // Time Text Marker ID counter
int N_persons = all_ellipses.size();

for (int i = 0; i < N_persons; i++){

// Visualize trajectory
visualization_msgs::msg::Marker line_marker;
line_marker.header.frame_id = world_frame_;
line_marker.header.stamp = aep_node->get_clock()->now();
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

const std::vector<std::tuple<double, double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

int N_trajectory_steps = xCoords.size();

bool x_constant = false;

if(!xCoords.empty() && xCoords[0] == xCoords[1])
{
  x_constant = true;
}


for (int j = 0; j < N_trajectory_steps; ++j) {

// Visualize trajectory by filling a line with points
geometry_msgs::msg::Point point;
point.x = xCoords[j];
point.y = yCoords[j];



if(params_.sphere_bb)
{
  point.z = zCoords[j];
}
else
{
  point.z = zCoords[j] + (params_.human_height/2);
}
line_marker.points.push_back(point);

//Extract the values to build the ellipse
double major_length = std::get<0>(ellipses[j]);
double minor_length = std::get<1>(ellipses[j]);
double z_length = std::get<2>(ellipses[j]);
Eigen::MatrixXd eigenvectors = std::get<3>(ellipses[j]);

// Validate eigenvectors matrix size
if (eigenvectors.rows() < 3 || eigenvectors.cols() < 3)
{
    RCLCPP_ERROR(aep_node->get_logger(), "eigenvectors matrix has invalid size: %dx%d (expected 3x3)", 
                eigenvectors.rows(), eigenvectors.cols());
    continue; // Skip this ellipse
}

//Visualize Ellipse
visualization_msgs::msg::Marker ellipse_marker;
ellipse_marker.header.frame_id = world_frame_;
ellipse_marker.header.stamp = aep_node->get_clock()->now();
ellipse_marker.ns = ns + "_cov_ellipse";
ellipse_marker.id = ellipse_id++;
ellipse_marker.type = visualization_msgs::msg::Marker::SPHERE;
ellipse_marker.pose.position.x = xCoords[j];
ellipse_marker.pose.position.y = yCoords[j];
ellipse_marker.pose.position.z = zCoords[j];

if(N_trajectory_steps > 1)
{
  ellipse_marker.color.a = 1.0f - (0.9f * j / (N_trajectory_steps - 1));
}
else
{
  ellipse_marker.color.a = 1.0f;
}

ellipse_marker.color.r = 0.0;
ellipse_marker.color.g = 1.0;
ellipse_marker.color.b = 0.0;

// Compute orientation quaternion of covariance ellipse
// eigenvectors is a 3x3 matrix, we want the first column (first eigenvector)
// Access as eigenvectors.col(0) to get the first column vector
Eigen::Vector3d first_eigenvector = eigenvectors.col(0);
Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), 
                                                          Eigen::Vector3d(first_eigenvector(0), first_eigenvector(1), 0.0)));
q.x() = 0;
q.y() = 0;
q.normalize();
ellipse_marker.pose.orientation.x = q.x();
ellipse_marker.pose.orientation.y = q.y();
ellipse_marker.pose.orientation.z = q.z();
ellipse_marker.pose.orientation.w = q.w();

// Give the marker the right dimensions according to the major and minor length

// Check x is constant or y is constant and depending on that switch order of major and minor length
if(x_constant)
{
  ellipse_marker.scale.x = 2.0 * minor_length;
  ellipse_marker.scale.y = 2.0 * major_length;
}
else
{
  ellipse_marker.scale.x = 2.0 * major_length;
  ellipse_marker.scale.y = 2.0 * minor_length;
}
ellipse_marker.scale.z = 2.0 * z_length; 

ellipse_marker_array.markers.push_back(ellipse_marker);

// Visualize Extended Ellipse (with human width compensation)
visualization_msgs::msg::Marker extended_ellipse_marker = ellipse_marker;
extended_ellipse_marker.ns = ns + "_extended_cov_ellipse";
extended_ellipse_marker.id = extended_ellipse_id++;
extended_ellipse_marker.color.r = 1.0;
extended_ellipse_marker.color.g = 0.0;
extended_ellipse_marker.color.b = 0.0;

// Extend the ellipse by human_width/2
double max_major = std::max(major_length, minor_length);

if(params_.sphere_bb)
{
  extended_ellipse_marker.scale.x = 2.0 * (max_major + params_.flying_obstacle_diameter/2);
  extended_ellipse_marker.scale.y = 2.0 * (max_major + params_.flying_obstacle_diameter/2);  
}
else
{
  extended_ellipse_marker.scale.x = 2.0 * (max_major + params_.human_width/2);
  extended_ellipse_marker.scale.y = 2.0 * (max_major + params_.human_width/2);  
}

extended_ellipse_marker_array.markers.push_back(extended_ellipse_marker);

if (j == N_trajectory_steps - 1) {

// Create time text marker
visualization_msgs::msg::Marker time_text_marker;
time_text_marker.header.frame_id = world_frame_;
time_text_marker.header.stamp = aep_node->get_clock()->now();
time_text_marker.ns = ns + "_time_labels";
time_text_marker.id = time_text_id++;
time_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
time_text_marker.action = visualization_msgs::msg::Marker::ADD;

// Position the text marker above the prediction point
time_text_marker.pose.position.x = xCoords[j];
time_text_marker.pose.position.y = yCoords[j];
time_text_marker.pose.position.z = zCoords[j] + params_.human_height + 0.3; // Place text above the human/DJI

// Set the time text content (time in seconds)
double prediction_time = j * dt;
std::stringstream ss;
ss << std::fixed << std::setprecision(1) << prediction_time << "s";
time_text_marker.text = ss.str();

// Set text appearance
time_text_marker.scale.z = 0.3; // Text height
time_text_marker.color.r = 1.0;
time_text_marker.color.g = 1.0;
time_text_marker.color.b = 0.0;
time_text_marker.color.a = 1.0;

time_text_marker_array.markers.push_back(time_text_marker);
} // Close condition if you only want the last prediction to have a time marker

}
line_marker_array.markers.push_back(line_marker);    

}      
// Publish the marker arrays (all dynamic obstacles)
marker_array_pub->publish(line_marker_array);
marker_array_pub->publish(ellipse_marker_array);
marker_array_pub->publish(extended_ellipse_marker_array);
marker_array_pub->publish(time_text_marker_array); // Publish time text markers
}

/**
 * Visualize the ground truth for each dynamic obstacle in RViz as a cylinder.
*/
void AEPlannerViz::visualizePose(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub, std::vector<daep_msgs::msg::HumanUpdate> humans, std::string ns)
{
  aeplanner::Params params_ = aep_node->get_params();

  visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for(auto const& human : humans)
    {
      // Create visualization marker for each human
      visualization_msgs::msg::Marker cylinder_marker;
      cylinder_marker.header.frame_id = world_frame_;  
      cylinder_marker.header.stamp = aep_node->get_clock()->now();
      cylinder_marker.ns = ns;
      cylinder_marker.id = id++;

      // If use_sphere_collision_checking==True we are flying, dynamic obstacles are UAVs. Draw them as spheres.
      if(params_.sphere_bb)
      {
        cylinder_marker.type = visualization_msgs::msg::Marker::SPHERE;
        cylinder_marker.pose.position.x = human.pose.position.x;
        cylinder_marker.pose.position.y = human.pose.position.y;
        cylinder_marker.pose.position.z = human.pose.position.z;
        cylinder_marker.scale.x = params_.flying_obstacle_diameter; 
        cylinder_marker.scale.y = params_.flying_obstacle_diameter;  
        cylinder_marker.scale.z = params_.flying_obstacle_diameter; 
      }
      else
      {
        //If not, we are on the ground. Dynamic obstacles are humans. Draw them as cylinders.
        cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder_marker.pose.position.x = human.pose.position.x;
        cylinder_marker.pose.position.y = human.pose.position.y;
        cylinder_marker.pose.position.z = human.pose.position.z + (params_.human_height/2);
        cylinder_marker.scale.x = params_.human_width; 
        cylinder_marker.scale.y = params_.human_width;  
        cylinder_marker.scale.z = params_.human_height; 
      }
      cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
      cylinder_marker.color.r = 1.0;  
      cylinder_marker.color.g = 1.0;  
      cylinder_marker.color.b = 1.0;  
      cylinder_marker.color.a = 1.0;  

      // Add the marker to the array
      marker_array.markers.push_back(cylinder_marker);
    }
    
    // Publish the marker array
    marker_pub->publish(marker_array);
}

/*
Create a visualization of the RRT-tree (Local Planner) by putting togehter edges and nodes in the RRT-tree.
*/
visualization_msgs::msg::MarkerArray AEPlannerViz::createRRTMarkerArray(RRTNode * root, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time) {
    int id = 0;
    if (alpha < 0.8) {
      id = 1000;
    }
    visualization_msgs::msg::MarkerArray marker_array;
    recurse(root, &marker_array, &id, alpha, ns, robot_state, expand_rrt_time);

    return marker_array;
  }
  
void AEPlannerViz::recurse(RRTNode * node, visualization_msgs::msg::MarkerArray * marker_array, int * id, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time){
    for(std::vector<RRTNode*>::iterator child_it  = node->children_.begin();
                                        child_it != node->children_.end(); ++child_it){
      RRTNode * child = (*child_it);
      if(child) recurse(child, marker_array, id, alpha, ns, robot_state, expand_rrt_time);
      marker_array->markers.push_back(createEdgeMarker(child, (*id), world_frame_, alpha, ns));
      marker_array->markers.push_back(createNodeMarker(child, (*id)++, world_frame_, alpha, ns));
      marker_array->markers.push_back(createTextMarker(child, (*id)++, world_frame_, alpha, ns));
      marker_array->markers.push_back(createTimeMarker(child, (*id)++, world_frame_, alpha, ns, robot_state, expand_rrt_time));
    }
  }

  visualization_msgs::msg::Marker AEPlannerViz::createTimeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns, Eigen::Vector4d robot_state, double expand_rrt_time) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = aep_node->get_clock()->now();
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = ns + "_time_text";
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->state_[0];
    marker.pose.position.y = node->state_[1];
    marker.pose.position.z = node->state_[2] + 0.2; // Slightly above the node
    marker.scale.z = 0.2; // Text size
    marker.color.r = 1.0; // White color for visibility
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;
    marker.lifetime = rclcpp::Duration(20, 0);
    marker.frame_locked = false;
    
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << (node->time_cost(robot_state, true) + expand_rrt_time);
    marker.text = "TOA:" + stream.str();
    
    return marker;
}

visualization_msgs::msg::Marker AEPlannerViz::createTextMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = aep_node->get_clock()->now();
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = ns + "_gain_text";
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->state_[0];
    marker.pose.position.y = node->state_[1];
    marker.pose.position.z = node->state_[2] + 0.2; // Slightly above the node
    marker.scale.z = 0.2; // Text size
    marker.color.r = 1.0; // White color for visibility
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;
    marker.lifetime = rclcpp::Duration(20, 0);
    marker.frame_locked = false;
    
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << node->gain_;
    marker.text = "Gain: " + stream.str();
    
    return marker;
}


visualization_msgs::msg::Marker AEPlannerViz::createNodeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns){
  visualization_msgs::msg::Marker marker;
    marker.header.stamp = aep_node->get_clock()->now();
    /// marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = ns + "_nodes";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->state_[0];
    marker.pose.position.y = node->state_[1];
    marker.pose.position.z = node->state_[2];
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = std::max(node->gain_ / 72.0, 0.05);
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1; //Yellow
    marker.color.g = 1;
    marker.color.b = 0.0;
    marker.color.a = alpha;

    if(ns == "rrtree")
    {
    marker.color.r = 1; //Orange
    marker.color.g = 0.698;
    marker.color.b = 0.0;
    marker.color.a = alpha;
    }
    
    if(ns == "best_branch"){
      marker.color.r = 0.0; //GREEN
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = alpha;
    }

    marker.lifetime = rclcpp::Duration(20, 0);
    marker.frame_locked = false;

    return marker;
  }

visualization_msgs::msg::Marker AEPlannerViz::createEdgeMarker(RRTNode * node, int id, std::string frame_id, double alpha, std::string ns){
  visualization_msgs::msg::Marker marker;
    marker.header.stamp = aep_node->get_clock()->now();
    /// marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = ns + "_edges";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = node->parent_->state_[0];
    marker.pose.position.y = node->parent_->state_[1];
    marker.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->state_[0] - node->parent_->state_[0],
                        node->state_[1] - node->parent_->state_[1],
                        node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = dir.norm();
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = 1; //Orange
    marker.color.g = 0.698;
    marker.color.b = 0.0;
    marker.color.a = alpha;

    if(ns == "rrtree")
    {
    marker.color.r = 1; //Orange
    marker.color.g = 0.698;
    marker.color.b = 0.0;
    marker.color.a = alpha;
    }
    
    if(ns == "best_branch"){
      marker.color.r = 0.0; //GREEN
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = alpha;
    }

    marker.lifetime = rclcpp::Duration(20, 0);
    marker.frame_locked = false;

    return marker;
  }


/*
Visualizes the best node and the other sampled nodes. It highlights the best node as green, 
valid nodes as red, and collision nodes as black.
*/
void AEPlannerViz::visualizeBestNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_node_pub,
                                     std::vector<Eigen::Vector4d> nodes)
{
  assert(aep_node);
  // Clear the current markers because of different amount of nodes
  visualization_msgs::msg::MarkerArray clearMarkerArray;
  visualization_msgs::msg::Marker clearMarker;
  clearMarker.header.frame_id = world_frame_;
  clearMarker.header.stamp = aep_node->get_clock()->now();
  clearMarker.ns = "cube";  
  clearMarker.type = visualization_msgs::msg::Marker::CUBE;  
  clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  clearMarkerArray.markers.push_back(clearMarker);
  best_node_pub->publish(clearMarkerArray);
  
  visualization_msgs::msg::MarkerArray markerArray;
  int idd = 0;

  for(auto node : nodes){
    auto probability = node(3);
    // Create a marker message
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;  
    marker.header.stamp = aep_node->get_clock()->now();
    marker.ns = "cube";  
    marker.id = idd++;  
    marker.type = visualization_msgs::msg::Marker::CUBE;  
    marker.action = visualization_msgs::msg::Marker::ADD; 
    marker.pose.position.x = node(0);  
    marker.pose.position.y = node(1);  
    marker.pose.position.z = node(2);  
    marker.pose.orientation.x = 0.0;  
    marker.pose.orientation.y = 0.0;  
    marker.pose.orientation.z = 0.0;  
    marker.pose.orientation.w = 1.0; 
    marker.scale.x = 0.5;  
    marker.scale.y = 0.5;  
    marker.scale.z = 0.5;  
    if (probability == 2){
      // Set best node as green
      marker.color.r = 0.2;  
      marker.color.g = 0.8;  
      marker.color.b = 0.0;  
      marker.color.a = 1.0;  
      marker.scale.x = 0.7;  
      marker.scale.y = 0.7;  
      marker.scale.z = 0.7;  
    }else {
      // Set valid nodes as red (prob = 0) and collision nodes as black (prob = 1)
      marker.color.r = 1.0-probability;  
      marker.color.g = 0.0;  
      marker.color.b = 0.0;  
      marker.color.a = 1.0;  
    }
    markerArray.markers.push_back(marker);
  }
  best_node_pub->publish(markerArray);
}

void AEPlannerViz::visualizeRejectedNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampled_node_pub, std::vector<Eigen::Vector4d> rejectedNodes, std::vector<int> rejectedReasons)
{
    // Create MarkerArray
    visualization_msgs::msg::MarkerArray marker_array;
    
    /*
    // Clear existing markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // Publish the delete marker
    sampled_node_pub->publish(marker_array);
    
    // Small sleep to ensure deletion
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    
    // Clear the marker array for new markers
    marker_array.markers.clear();
    */
    
    // Iterate through sampled nodes and create markers
    for (size_t i = 0; i < rejectedNodes.size(); ++i) {
        // Create a new Marker message
        visualization_msgs::msg::Marker marker;
        int reason = rejectedReasons[i];
        
        // Set the frame ID (typically "map" or the appropriate reference frame)
        marker.header.frame_id = world_frame_;
        
        // Set the timestamp to now
        marker.header.stamp = rclcpp::Clock().now();
        
        // Set the namespace and id to uniquely identify the marker
        marker.ns = "rejected_nodes";
        marker.id = i;  // Unique ID for each marker
        
        // Set marker type to SPHERE
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        
        // Set marker action to ADD (create/modify)
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set the position using x, y, z from the input vector
        marker.pose.position.x = rejectedNodes[i][0];
        marker.pose.position.y = rejectedNodes[i][1];
        marker.pose.position.z = rejectedNodes[i][2];
        
        // Set default orientation (no rotation)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the sphere
        marker.scale.x = 0.2;  // diameter of the sphere
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // Set color to red
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue
        if (reason == 1) {
          marker.color.g = 1.0;
        }
        if (reason == 2) {
          marker.color.r = 1.0;
        }
        if (reason == 3) {
          marker.color.b = 1.0;
        }
        if (reason == 10) {
          marker.color.r = 1.0;
          marker.color.g = 1.0;
        }
        marker.color.a = 1.0;  // Full opacity
        
        // Ensure the marker doesn't automatically disappear
        marker.lifetime = rclcpp::Duration(0, 0);
        
        // Add marker to MarkerArray
        marker_array.markers.push_back(marker);
    }
    
    // Publish the MarkerArray
    sampled_node_pub->publish(marker_array);
}

void AEPlannerViz::visualizeNearestNode(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nearest_node_pub, std::vector<Eigen::Vector4d> neighbors)
{
  // Create MarkerArray
  visualization_msgs::msg::MarkerArray marker_array;
    
  /*
  // Clear existing markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);
  
  // Publish the delete marker
  nearest_node_pub->publish(marker_array);
  
  // Small sleep to ensure deletion
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  
  // Clear the marker array for new markers
  marker_array.markers.clear();
  */

  // Iterate through neighbors and create markers
  for (size_t i = 0; i < neighbors.size(); ++i) {
      // Create a new Marker message
      visualization_msgs::msg::Marker marker;
      
      // Set the frame ID (typically "map" or the appropriate reference frame)
      marker.header.frame_id = world_frame_;
      
      // Set the timestamp to now
      marker.header.stamp = rclcpp::Clock().now();
      
      // Set the namespace and id to uniquely identify the marker
      marker.ns = "nearest_nodes";
      marker.id = i;  // Unique ID for each marker
      
      // Set marker type to SPHERE
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      
      // Set marker action to ADD (create/modify)
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Set the position using x, y, z from the input vector
      marker.pose.position.x = neighbors[i][0];
      marker.pose.position.y = neighbors[i][1];
      marker.pose.position.z = neighbors[i][2];
      
      // Set default orientation (no rotation)
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      // Set the scale of the sphere
      marker.scale.x = 0.2;  // diameter of the sphere
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      
      // Set color to blue
      marker.color.r = 0.0;  // Red
      marker.color.g = 0.0;  // Green
      marker.color.b = 1.0;  // Blue
      marker.color.a = 1.0;  // Full opacity
      
      // Ensure the marker doesn't automatically disappear
      marker.lifetime = rclcpp::Duration(0, 0);
      
      // Add marker to MarkerArray
      marker_array.markers.push_back(marker);
  }
  
  // Publish the MarkerArray
  nearest_node_pub->publish(marker_array);
}

void AEPlannerViz::visualizeValidNodes(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr valid_nodes_pub, std::vector<Eigen::Vector4d> validNodes)
{
    // Create a MarkerArray message
    visualization_msgs::msg::MarkerArray marker_array;

    /*
    // Create a single marker with DELETEALL action
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    
    marker_array.markers.push_back(delete_marker);
    
    // Publish the delete marker
    valid_nodes_pub->publish(marker_array);
    
    // Small sleep to ensure deletion
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    */

    // Iterate through all valid nodes
    for (size_t i = 0; i < validNodes.size(); ++i)
    {
        // Create a new Marker for each node
        visualization_msgs::msg::Marker marker;
        
        // Set the frame ID (typically "map" or the appropriate reference frame)
        marker.header.frame_id = world_frame_;
        
        // Set the timestamp to now
        marker.header.stamp = rclcpp::Clock().now();
        
        // Set the namespace and unique id for each marker
        marker.ns = "valid_nodes";
        marker.id = i;  // Unique ID for each marker
        
        // Set marker type to SPHERE
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        
        // Set marker action to ADD (create/modify)
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set the position using x, y, z from the input vector
        marker.pose.position.x = validNodes[i][0];
        marker.pose.position.y = validNodes[i][1];
        marker.pose.position.z = validNodes[i][2];
        
        // Set default orientation (no rotation)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // Set the scale of the sphere
        marker.scale.x = 0.2;  // diameter of the sphere
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // Set color to green
        marker.color.r = 0.0;  // Red
        marker.color.g = 1.0;  // Green
        marker.color.b = 0.0;  // Blue
        marker.color.a = 1.0;  // Full opacity
        
        // Ensure the marker doesn't automatically disappear
        marker.lifetime = rclcpp::Duration(0, 0);
        
        // Add the marker to the MarkerArray
        marker_array.markers.push_back(marker);
    }
    
    // Publish the MarkerArray
    valid_nodes_pub->publish(marker_array);
}

void AEPlannerViz::visualizeOctomapSlice(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr octomap_leafs_pub, octomap::point3d min, octomap::point3d max)
{
    // Create a new Marker message
    visualization_msgs::msg::Marker marker;
    
    // Set the frame ID (typically "map" or the appropriate reference frame)
    marker.header.frame_id = world_frame_;
    
    // Set the timestamp to now
    marker.header.stamp = rclcpp::Clock().now();
    
    // Set the namespace and id to uniquely identify the marker
    marker.ns = "octomap_collision_slice";
    marker.id = 0;
    
    // Set marker type to CUBE
    marker.type = visualization_msgs::msg::Marker::CUBE;
    
    // Set marker action to ADD (create/modify)
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Calculate the center of the cube
    marker.pose.position.x = (min.x() + max.x()) / 2.0;
    marker.pose.position.y = (min.y() + max.y()) / 2.0;
    marker.pose.position.z = (min.z() + max.z()) / 2.0;
    
    // Set default orientation (no rotation)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Calculate the dimensions of the cube
    marker.scale.x = std::abs(max.x() - min.x());
    marker.scale.y = std::abs(max.y() - min.y());
    marker.scale.z = std::abs(max.z() - min.z());
    
    // Set color to green
    marker.color.r = 0.0;  // Red
    marker.color.g = 1.0;  // Green
    marker.color.b = 0.0;  // Blue
    marker.color.a = 0.3;  // Partially transparent
    
    // Ensure the marker doesn't automatically disappear
    marker.lifetime = rclcpp::Duration(0, 0);
    
    // Publish the marker
    octomap_leafs_pub->publish(marker);
}

void AEPlannerViz::visualizeCylinders(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinders_pub, Eigen::Vector4d start, Eigen::Vector4d end, double x_diameter1, double y_diameter1, double z_diameter1, double x_diameter2, double y_diameter2, double z_diameter2)
{
      // Create a MarkerArray message
      visualization_msgs::msg::MarkerArray marker_array;
      
      // Create markers for start and end points
      for (int i = 0; i < 2; ++i) 
      {
          // Create a new Marker for each point
          visualization_msgs::msg::Marker marker;
          
          // Set the frame ID (typically "map" or the appropriate reference frame)
          marker.header.frame_id = world_frame_;
          
          // Set the timestamp to now
          marker.header.stamp = rclcpp::Clock().now();
          
          // Set the namespace and unique id for each marker
          marker.ns = "collision_cylinders";
          marker.id = i;  // 0 for start, 1 for end
          
          // Set marker type to CYLINDER
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          
          // Set marker action to ADD (create/modify)
          marker.action = visualization_msgs::msg::Marker::ADD;
          
          // Set the position 
          if (i == 0) {
              // Start point
              marker.pose.position.x = start[0];
              marker.pose.position.y = start[1];
              marker.pose.position.z = start[2];
              double major_length = std::max(x_diameter1, y_diameter1);
              marker.scale.x = major_length;  // diameter in x direction
              marker.scale.y = major_length;  // diameter in y direction
              marker.scale.z = z_diameter1;  // height of the cylinder
          } else {
              // End point
              marker.pose.position.x = end[0];
              marker.pose.position.y = end[1];
              marker.pose.position.z = end[2];
              double major_length = std::max(x_diameter2, y_diameter2);
              marker.scale.x = major_length;  // diameter in x direction
              marker.scale.y = major_length;  // diameter in y direction
              marker.scale.z = z_diameter2;  // height of the cylinder
          }
          
          // Set orientation 
          tf2::Quaternion q;
          q.setRPY(0, 0, (i == 0) ? start[3] : end[3]);  // Use yaw from input vector
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();
          
          // Set the scale of the cylinder

          // Set color (using a distinct color for visibility)
          marker.color.r = (i == 0) ? 1.0 : 0.0;  // Red for start, Green for end
          marker.color.g = (i == 0) ? 0.0 : 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;  // Full opacity
          
          // Ensure the marker doesn't automatically disappear
          marker.lifetime = rclcpp::Duration(0, 0);
          
          // Add the marker to the MarkerArray
          marker_array.markers.push_back(marker);
      }
      
      // Publish the MarkerArray
      cylinders_pub->publish(marker_array);
}

void AEPlannerViz::visualizeTunnel(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cylinders_pub, Eigen::Vector4d start, Eigen::Vector4d end, double radius)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker cylinder_marker;

    // Basic marker setup
    cylinder_marker.header.frame_id = world_frame_; // or "odom" or whatever your frame is
    cylinder_marker.header.stamp = rclcpp::Clock().now();
    cylinder_marker.ns = "tunnel_cylinders";
    cylinder_marker.id = 0; // You might want to increment this for multiple cylinders
    cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder_marker.action = visualization_msgs::msg::Marker::ADD;

    // Calculate the center point of the cylinder
    Eigen::Vector3d start_pos = start.head<3>();
    Eigen::Vector3d end_pos = end.head<3>();
    Eigen::Vector3d center = (start_pos + end_pos) / 2.0;

    cylinder_marker.pose.position.x = center.x();
    cylinder_marker.pose.position.y = center.y();
    cylinder_marker.pose.position.z = center.z();

    // Calculate the orientation of the cylinder
    // The cylinder axis vector
    Eigen::Vector3d axis = end_pos - start_pos;
    double length = axis.norm();
    axis.normalize();

    // RViz cylinders are oriented along the Z-axis by default
    // We need to find the rotation that aligns the Z-axis with our cylinder axis
    Eigen::Vector3d z_axis(0, 0, 1);
    
    // Calculate the rotation quaternion
    Eigen::Quaterniond rotation;
    
    // Check if the axis is already aligned with z-axis
    if (axis.dot(z_axis) > 0.9999) {
        // Already aligned, no rotation needed
        rotation = Eigen::Quaterniond::Identity();
    }
    else if (axis.dot(z_axis) < -0.9999) {
        // Opposite direction, rotate 180 degrees around any perpendicular axis
        rotation = Eigen::Quaterniond(0, 1, 0, 0); // 180 degree rotation around x-axis
    }
    else {
        // General case: rotate z-axis to align with cylinder axis
        Eigen::Vector3d rotation_axis = z_axis.cross(axis).normalized();
        double angle = acos(z_axis.dot(axis));
        rotation = Eigen::AngleAxisd(angle, rotation_axis);
    }

    cylinder_marker.pose.orientation.x = rotation.x();
    cylinder_marker.pose.orientation.y = rotation.y();
    cylinder_marker.pose.orientation.z = rotation.z();
    cylinder_marker.pose.orientation.w = rotation.w();

    // Set the scale (dimensions)
    cylinder_marker.scale.x = radius * 2.0; // diameter in x
    cylinder_marker.scale.y = radius * 2.0; // diameter in y
    cylinder_marker.scale.z = length;       // height (length along z-axis)

    // Set color (semi-transparent blue)
    cylinder_marker.color.r = 0.0;
    cylinder_marker.color.g = 0.5;
    cylinder_marker.color.b = 1.0;
    cylinder_marker.color.a = 0.6; // semi-transparent

    // Set lifetime (0 means it persists until deleted)
    cylinder_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    // Add the marker to the array
    marker_array.markers.push_back(cylinder_marker);

    // Publish the marker array
    cylinders_pub->publish(marker_array);
}


void AEPlannerViz::visualizeFilteredOctomap(rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_filtered_pub, std::shared_ptr<octomap::OcTree> octree)
{
 if (!octomap_filtered_pub || !octree) {
        RCLCPP_WARN(rclcpp::get_logger("AEPlannerViz"),
                    "Octomap publisher or octree is null, cannot visualize.");
        return;
    }

    octomap_msgs::msg::Octomap msg;

    // Convert to ROS msg (binary is more compact & standard for visualization)
    if (!octomap_msgs::binaryMapToMsg(*octree, msg)) {
        RCLCPP_ERROR(rclcpp::get_logger("AEPlannerViz"),
                     "Failed to convert octree to Octomap message.");
        return;
    }

    // Fill header
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = world_frame_;

    // Publish
    octomap_filtered_pub->publish(msg);
  }
