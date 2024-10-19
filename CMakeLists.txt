cmake_minimum_required(VERSION 3.5)
project(your_package_name)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(px4_ros2 REQUIRED)

# Include subdirectories
add_subdirectory(src/WaypointReceiver)
add_subdirectory(src/AutonomousMode)

# Add an executable for the main application (if any)
# add_executable(main_app src/main.cpp)

# Link dependencies to the main app (if it needs them)
# ament_target_dependencies(main_app rclcpp mavros_msgs px4_ros2)

# # Install the main app
# install(TARGETS main_app
#   DESTINATION lib/${PROJECT_NAME})

ament_package()

