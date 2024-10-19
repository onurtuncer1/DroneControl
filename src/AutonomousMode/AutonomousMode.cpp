#include "AutonomousMode.hpp"
#include <Eigen/Core>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

// Constants
static const std::string kModeName = "AutonomousFlightMode";
static const bool kEnableDebugOutput = true;

AutonomousMode::AutonomousMode(rclcpp::Node& node): ModeBase(node, kModeName), _node(node)
{
    // Initialize the trajectory setpoint and vehicle position data
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Subscribe to vehicle land detected message
    _vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
        std::bind(&CustomMode::vehicleLandDetectedCallback, this, std::placeholders::_1));

     // Subscribe to the custom waypoint topic where the waypoints will be published
    waypoint_subscriber_ = node.create_subscription<mavros_msgs::msg::WaypointList>(
        "/autonomous_mode/waypoints", 10,
        std::bind(&AutonomousMode::waypoints_callback, this, std::placeholders::_1));
}

void AutonomousMode::waypoints_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
    RCLCPP_INFO(_node.get_logger(), "Received waypoints for the mission. Processing...");

    // Generate waypoints from the MAVROS waypoint list
    generateWaypointsFromMAVROS(*msg);
}

void AutonomousMode::generateWaypointsFromMAVROS(const mavros_msgs::msg::WaypointList& msg)
{
    _waypoints.clear();  // Clear any existing waypoints

    for (const auto& wp : msg.waypoints) {
        // Convert each waypoint from MAVROS format (latitude, longitude, altitude) to NED frame
        Eigen::Vector3f waypoint(wp.x_lat, wp.y_long, wp.z_alt);
        _waypoints.push_back(waypoint);
    }

    RCLCPP_INFO(_node.get_logger(), "Loaded %lu waypoints for the mission", _waypoints.size());
}

void AutonomousMode::onActivate()
{
    // // Generate waypoints upon activation
    // generateWaypoints();
    
    //TODO check if waypoints are received
    // Start with the takeoff state
    switchToState(State::Takeoff);
}

void AutonomousMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "Deactivating Autonomous Flight Mode");
}

void AutonomousMode::updateSetpoint(float dt_s)
{
    switch (_state) {
        case State::Takeoff:
            // Takeoff to the initial height
            if (!has_taken_off_) {
                executeTakeoff();
            }
            if (takeoffCompleted()) {
                switchToState(State::Navigate);
            }
            break;

        case State::Navigate:
            navigateToNextWaypoint();
            break;

        case State::ReturnToHome:
            returnToHome();
            break;

        case State::Landing:
            descendAndLand();
            break;

        case State::Finished:
            // End the mission
            ModeBase::completed(px4_ros2::Result::Success);
            break;
    }
}

void AutonomousMode::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

void AutonomousMode::executeTakeoff()
{
    // Create the trajectory setpoint for takeoff
    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = _node.now().nanoseconds() / 1000;
    setpoint.position = {NAN, NAN, -10.0}; // Takeoff to 10 meters
    setpoint.velocity = {NAN, NAN, NAN};
    setpoint.acceleration = {NAN, NAN, NAN};
    _trajectory_setpoint->update(setpoint);
    has_taken_off_ = true;
}

bool AutonomousMode::takeoffCompleted() const
{
    return _vehicle_local_position->positionNed().z() <= -9.5;  // Check if drone has reached near 10 meters altitude
}

void AutonomousMode::navigateToNextWaypoint()
{
    if (_waypoint_index < _waypoints.size()) {
        Eigen::Vector3f target_position = _waypoints[_waypoint_index];
        // Create and update the trajectory setpoint for the next waypoint
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = _node.now().nanoseconds() / 1000;
        setpoint.position = {target_position.x(), target_position.y(), target_position.z()};
        _trajectory_setpoint->update(setpoint);

        if (positionReached(target_position)) {
            _waypoint_index++;
        }
    } else {
        // Once all waypoints are completed, return home
        switchToState(State::ReturnToHome);
    }
}

void AutonomousMode::returnToHome()
{
    Eigen::Vector3f home_position = {0.0f, 0.0f, _vehicle_local_position->positionNed().z()};
    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = _node.now().nanoseconds() / 1000;
    setpoint.position = {home_position.x(), home_position.y(), home_position.z()};
    _trajectory_setpoint->update(setpoint);

    if (positionReached(home_position)) {
        switchToState(State::Landing);
    }
}

void AutonomousMode::descendAndLand()
{
    // Descend slowly to land
    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = _node.now().nanoseconds() / 1000;
    setpoint.velocity = {0.0, 0.0, 0.5}; // Descend at 0.5 m/s
    _trajectory_setpoint->update(setpoint);

    if (_land_detected) {
        switchToState(State::Finished);
    }
}

bool CustomMode::positionReached(const Eigen::Vector3f& target) const
{
    static constexpr float kDeltaPosition = 0.5f;
    auto position = _vehicle_local_position->positionNed();
    return (target - position).norm() < kDeltaPosition;
}

void CustomMode::generateWaypoints()
{
    _waypoints.clear();
    _waypoints.push_back({5.0f, 0.0f, -10.0f});
    _waypoints.push_back({5.0f, 5.0f, -10.0f});
    _waypoints.push_back({0.0f, 5.0f, -10.0f});
    _waypoint_index = 0;
}

void CustomMode::switchToState(State state)
{
    _state = state;
    RCLCPP_INFO(_node.get_logger(), "Switching to state: %s", stateName(state).c_str());
}

std::string CustomMode::stateName(State state)
{
    switch (state) {
        case State::Takeoff: return "Takeoff";
        case State::Navigate: return "Navigate";
        case State::ReturnToHome: return "ReturnToHome";
        case State::Landing: return "Landing";
        case State::Finished: return "Finished";
        default: return "Unknown";
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<CustomMode>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}
