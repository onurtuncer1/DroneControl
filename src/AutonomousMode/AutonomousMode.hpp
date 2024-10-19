#ifndef AUTONOMOUS_MODE_HPP
#define AUTONOMOUS_MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>  // Include waypoint list message
#include <px4_ros2/components/mode_base.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <memory>
#include <vector>

class AutonomousMode : public px4_ros2::ModeBase
{
public:
    enum class State {
        Takeoff,
        Navigate,
        ReturnToHome,
        Landing,
        Finished
    };

    AutonomousMode(rclcpp::Node& node);

    // Lifecycle event: called when the mode is activated
    void onActivate() override;

    // Lifecycle event: called when the mode is deactivated
    void onDeactivate() override;

    // Function to update trajectory setpoints based on current state
    void updateSetpoint(float dt_s) override;

private:
    // State management functions
    void executeTakeoff();
    bool takeoffCompleted() const;
    void navigateToNextWaypoint();
    void returnToHome();
    void descendAndLand();
    bool positionReached(const Eigen::Vector3f& target) const;
    void switchToState(State state);
    std::string stateName(State state);

    // Helper function to generate waypoints for the mission
    void generateWaypoints();

    // Vehicle land detection callback
    void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

    // ROS2 communication members
    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

    // Internal state management
    State _state;
    bool _land_detected;
    bool has_taken_off_;
    int _waypoint_index;
    std::vector<Eigen::Vector3f> _waypoints;
};

#endif // CUSTOM_MODE_HPP
