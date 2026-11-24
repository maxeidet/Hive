#ifndef READ_PARAMS_H
#define READ_PARAMS_H

namespace aeplanner
{
  struct Params
  {
    double hfov;
    double vfov;
    double r_max;
    double r_min;

    double dr;
    double dphi;
    double dtheta;

    double lambda;
    double zero_gain;
    double extension_range;
    double max_sampling_radius;
    double sigma_thresh;
    double zeta;
    double time_step;
    
    double bounding_radius;
    bool sphere_bb;
    bool gain_inside_boundary;
    bool debug_text;
    bool add_rejected_nodes;
    bool visualize_predictions;
    double x_diameter; 
    double y_diameter; 
    double z_diameter;

    int init_iterations;
    int cutoff_iterations;
    int cache_node_threshold;
    int node_gain_threshold;
    int KFiterations;
    double position_uncertainty;
    double velocity_uncertainty;
    double process_noise;
    double measurement_noise;
    int safety_iteration;
    
    std::vector<double> boundary_min;
    std::vector<double> boundary_max;

    std::string robot_frame;
    std::string world_frame;

    bool visualize_tree;
    bool visualize_rays;
    bool visualize_exploration_area;
    bool visualize_static_and_dynamic_rays;

    double human_height;
    double human_width;
    double human_linear_velocity;
    double human_angular_velocity;
    double human_measurement_timeout;  // Timeout in seconds before removing human from kalman filter

    double drone_height;
    double drone_width;
    double drone_linear_velocity;
    double drone_angular_velocity;

    double boosted_boundary_length;
    double boost_magnitude;

    double max_sampled_initial_nodes;
    double max_sampled_nodes;

    bool drone_freeze;

    int global_planner_counter;

    int look_ahead_horizon;

    double z_offset;
    double flying_obstacle_diameter;

    double max_volume;
    double add_node_to_frontier_threshold;
    double reevaluate_node_threshold;

    double add_to_frontier_ratio;
    double reeval_ratio;
    double zero_gain_ratio;

    bool filter_out_ground;
    double ground_filter_z_dist;
    bool viz_filtered_octomap;

    bool use_smoothing;
    double smoothing_factor;
    bool rotating_while_walking;

    double best_branch_threshold;

    bool ignore_first_iteration;

    bool husky;

    double time_window;
    std::string prediction_mode;

  };
};

#endif
