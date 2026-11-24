#include <memory>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <daep_msgs/srv/reevaluate.hpp>
#include <eigen3/Eigen/Dense>

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include <octomap/OcTree.h>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

using namespace std::placeholders;


class ReevalFrontierNode : public rclcpp::Node 
{

    public:
        ReevalFrontierNode() : Node("reeval_frontier") 
        {

            // Declare parameters
            this->declare_parameter("camera/horizontal_fov", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("camera/vertical_fov", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("aep/gain/r_min", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("aep/gain/r_max", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("raycast/dr", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("raycast/dphi", rclcpp::PARAMETER_DOUBLE);
            this->declare_parameter("raycast/dtheta", rclcpp::PARAMETER_DOUBLE);

            // Read parameters
            rclcpp::Parameter param;
            if (this->get_parameter("camera/horizontal_fov", param)) {
                horizontal_fov_ = param.as_double();
            } else {
                horizontal_fov_ = 60.0;
                RCLCPP_WARN_STREAM(this->get_logger(), "No horizontal fov specified. Default: " << horizontal_fov_);    
            }

            if (this->get_parameter("camera/vertical_fov", param)) {
                vertical_fov_ = param.as_double();
            } else {
                vertical_fov_ = 45.0;
                RCLCPP_WARN_STREAM(this->get_logger(), "No vertical fov specified. Default: " << vertical_fov_);    
            }

            if (this->get_parameter("aep/gain/r_min", param)) {
                r_min_ = param.as_double();
            } else {
                r_min_ = 0.5;
                RCLCPP_WARN_STREAM(this->get_logger(), "No r_min specified. Default: " << r_min_);    
            }

            if (this->get_parameter("aep/gain/r_max", param)) {
                r_max_ = param.as_double();
            } else {
                r_max_ = 5.0;
                RCLCPP_WARN_STREAM(this->get_logger(), "No r_max specified. Default: " << r_max_);    
            }

            if (this->get_parameter("raycast/dr", param)) {
                dr_ = param.as_double();
            } else {
                dr_ = 0.1;
                RCLCPP_WARN_STREAM(this->get_logger(), "No dr specified. Default: " << dr_);    
            }

            if (this->get_parameter("raycast/dphi", param)) {
                dphi_ = param.as_double();
            } else {
                dphi_ = 5.0;
                RCLCPP_WARN_STREAM(this->get_logger(), "No dphi specified. Default: " << dphi_);    
            }

            if (this->get_parameter("raycast/dtheta", param)) {
                dtheta_ = param.as_double();
            } else {
                dtheta_ = 5.0;
                RCLCPP_WARN_STREAM(this->get_logger(), "No dtheta specified. Default: " << dtheta_);    
            }


            // The octomap is reliable, not sensor_data: a full map is megabytes, and best effort
            // drops it whole when one fragment is lost, so with Fast DDS it never arrived.
            octomap_subscription = this->create_subscription<octomap_msgs::msg::Octomap>("octomap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                                                                        std::bind(&ReevalFrontierNode::octomapCallback,
                                                                                                  this, std::placeholders::_1));

            service_ = this->create_service<daep_msgs::srv::Reevaluate>(
            "reevaluate",
            std::bind(&ReevalFrontierNode::reevaluate, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        rclcpp::Service<daep_msgs::srv::Reevaluate>::SharedPtr service_;
        std::shared_ptr<octomap::OcTree> ot_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription;

        double horizontal_fov_;
        double vertical_fov_;
        double r_min_;
        double r_max_;
        double dr_;
        double dphi_;
        double dtheta_;


    void octomapCallback(const octomap_msgs::msg::Octomap& msg)
    {    
    octomap::AbstractOcTree* aot = octomap_msgs::fullMsgToMap(msg);
    if (aot == 0) {
        RCLCPP_ERROR(this->get_logger(), "Empty octree");
        return;
    }
    octomap::OcTree* ot = (octomap::OcTree*)aot;
    ot_ = std::make_shared<octomap::OcTree>(*ot);

    delete ot;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Freeing Octree done:");
    }


std::tuple <double, double, double> gainCubature(Eigen::Vector3d state)
{
 
  // **Declare intenal variables**
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double static_gain = 0.0;
  double dynamic_gain = 0.0;

  //Radius variables
  double r;

  //Angles
  int phi, theta;
  double phi_rad, theta_rad;
  
  //Step size for r, phi and theta. In degrees and radians.
  double dphi_rad = M_PI * dphi_ / 180.0f, dtheta_rad = M_PI * dtheta_ / 180.0f;

  //Maps
  std::map<int, double> gain_per_yaw;
  std::map<int, double> dynamic_gain_per_yaw;

  //Set the origin to the point to be examined
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;

  //For each theta in a circle
  for (theta = -180; theta < 180; theta += dtheta_)
  {
    theta_rad = M_PI * theta / 180.0f;
    
    //For each phi in pitch
    for (phi = (90 - vertical_fov_ / 2); phi < (90 + vertical_fov_ / 2); phi += dphi_)
    {
      phi_rad = M_PI * phi / 180.0f;

      //Gain for a specific theta and phi over all r
      double static_g = 0.0;
      double dynamic_g = 0.0;
      
      //For each r in distance from origin to r_max
      for (r = r_min_; r < r_max_; r += dr_)
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
          static_g += (2.0 * r * r * dr_ + (1.0 / 6.0) * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2.0);
        }
      }
  
      gain_per_yaw[theta] += static_g; //Add for each theta what the current gain is (Yaw)
      dynamic_gain_per_yaw[theta] += dynamic_g; //Add for each theta what the current dymamic_gain is.
      
    }
  }

  //Calculate the best yaw angle
  int best_yaw = 0;
  double best_yaw_score = 0.0, best_dynamic_yaw_score = 0.0;
  
  //For each yaw in one spin
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    double dynamic_yaw_score = 0;
    //For each field of view in the y-direction
    for (int fov = -horizontal_fov_ / 2; fov < horizontal_fov_ / 2; fov++)
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

  return std::make_tuple(static_gain, dynamic_gain, yaw);
}


  void reevaluate(
    const std::shared_ptr<daep_msgs::srv::Reevaluate::Request> request,
    std::shared_ptr<daep_msgs::srv::Reevaluate::Response> response)
  {

    if (!ot_)
    {
        RCLCPP_WARN(this->get_logger(), "Not octomap received yet, cannot reevaluate frontiers.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received reevaluation request with %zu frontiers", request->points.size());
    auto start = std::chrono::high_resolution_clock::now();

    for (const auto& pt : request->points) {
      
      Eigen::Vector3d position(pt.x, pt.y, pt.z);
      auto gain_response = gainCubature(position);      
      
      response->gain.push_back(std::get<0>(gain_response));
      response->dynamic_gain.push_back(0.0);
      response->yaw.push_back(std::get<2>(gain_response));
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    RCLCPP_INFO(this->get_logger(), "Reevaluation done, responded with %zu gains. Reevaluation took %zu milliseconds", response->gain.size(), duration.count());

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReevalFrontierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}