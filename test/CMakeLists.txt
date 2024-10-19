# Minimum CMake version required
cmake_minimum_required(VERSION 3.5)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(px4_ros2 REQUIRED)

# Add the test for the waypoint receiver
ament_add_gtest(test_waypoint_receiver test_waypoint_receiver.cpp)

# Link the test with the target node (i.e., waypoint receiver)
target_link_libraries(test_waypoint_receiver waypoint_receiver_node)

# Specify dependencies for the test
ament_target_dependencies(test_waypoint_receiver rclcpp mavros_msgs px4_ros2)
