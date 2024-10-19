cmake_minimum_required(VERSION 3.5)
project(waypoint_receiver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

# Add executable for the waypoint receiver node
add_executable(waypoint_receiver_node WaypointReceiver.cpp)

# Specify dependencies for the target
ament_target_dependencies(
  waypoint_receiver_node
  rclcpp
  mavros_msgs
)

# Install the target
install(TARGETS
  waypoint_receiver_node
  DESTINATION lib/${PROJECT_NAME})

# ament package
ament_package()
