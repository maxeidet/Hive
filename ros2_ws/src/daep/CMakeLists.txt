cmake_minimum_required(VERSION 3.8)
project(daep)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(daep_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(tf2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(aeplanner_node
  src/kdtree.c
  src/aeplanner_viz.cpp
  src/kalman.cpp
  src/aeplanner.cpp
  )

ament_target_dependencies(aeplanner_node rclcpp rclcpp_action daep_msgs nav_msgs octomap_msgs visualization_msgs octomap_ros octomap tf2)
# target_link_libraries(manager_m300 -lproj)

add_executable(rpl_exploration
  src/rpl_exploration.cpp
  )

ament_target_dependencies(rpl_exploration rclcpp rclcpp_action std_msgs daep_msgs tf2 irobot_create_msgs)
# target_link_libraries(manager_m300 -lproj)

add_executable(coverage_evaluation
  src/evaluation.cpp
  )

ament_target_dependencies(coverage_evaluation rclcpp rclcpp_action std_msgs daep_msgs octomap_msgs octomap_ros octomap)
# target_link_libraries(manager_m300 -lproj)

add_executable(coverage_standalone
  src/coverage_standalone.cpp
  )

ament_target_dependencies(coverage_standalone rclcpp octomap_msgs octomap_ros octomap)

add_executable(laser_filter
  src/laser_filter.cpp
  )

ament_target_dependencies(laser_filter rclcpp sensor_msgs)

add_executable(rrt
  src/rrtnode.cpp
  src/kdtree.c  
  src/rrt_kalman.cpp
  src/rrt.cpp
  )

ament_target_dependencies(rrt rclcpp rclcpp_action std_msgs daep_msgs octomap_msgs octomap_ros octomap visualization_msgs tf2)
# target_link_libraries(manager_m300 -lproj)

add_executable(reeval_frontier
  src/reeval_frontier.cpp
  )

ament_target_dependencies(reeval_frontier rclcpp rclcpp_action daep_msgs nav_msgs octomap_msgs visualization_msgs octomap_ros octomap tf2)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  aeplanner_node
  rpl_exploration
  coverage_evaluation
  coverage_standalone
  rrt
  laser_filter
  reeval_frontier
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/daeptest.py
  src/pig.py
  src/dfm.py
  src/animate_objects.py
  src/frustum.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  launch/camera_with_cloud.launch.py
  launch/load_camera.launch.py
  launch/pig.launch.py
  launch/reeval_frontier.launch.py
  launch/dfm.launch.py
  launch/animate.launch.py
  launch/octomap_server.launch.py
  launch/octomap_server_vicon.launch.py
  launch/octomap_server_husky.launch.py
  launch/octomap_server_spot.launch.py
  launch/octomap_server_m100_sim.launch.py
  launch/octomap_server_turtle4.launch.py
  launch/scan_to_pointcloud.launch.py
  launch/coverage_evaluation.launch.py
  launch/rrtplanner.launch.py
  launch/aeplanner.launch.py
  launch/laser_scan_filter.launch.py
  launch/frustum.launch.py
###  launch/slam.launch.py
  DESTINATION share/${PROJECT_NAME}
  )

install(FILES
    config/pig_config.yaml
    config/dfm_config.yaml
    config/cafe_exploration.yaml
    config/granso_exploration.yaml
    config/granso_cafe_exploration.yaml
    config/granso_castle_exploration.yaml
    config/mat1_granso_exploration.yaml
    config/maze_exploration.yaml
    config/terra_exploration.yaml
    config/husky_terra_exploration.yaml
    config/spot_terra_exploration.yaml
    config/angular_filter.yaml
    config/px4_exploration.yaml
    DESTINATION share/${PROJECT_NAME}/config
    )

ament_package()
