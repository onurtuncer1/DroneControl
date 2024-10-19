#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "std_msgs/msg/string.hpp"  // You can use a custom message type for complex waypoint data

class WaypointReceiverNode : public rclcpp::Node
{
public:
    WaypointReceiverNode() : Node("waypoint_receiver_node")
    {
        // Subscribe to the MAVROS waypoints topic
        waypoint_subscriber_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
            "/mavros/mission/waypoints", 10,
            std::bind(&WaypointReceiverNode::waypoint_callback, this, std::placeholders::_1));

        // Create a publisher to share waypoints with CustomMode node
        autonomous_waypoint_publisher_ = this->create_publisher<mavros_msgs::msg::WaypointList>(
            "/autonomous_mode/waypoints", 10);

        RCLCPP_INFO(this->get_logger(), "Waiting to receive waypoints from QGroundControl...");
    }

private:
    void waypoint_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg)
    {
        if (msg->waypoints.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Received %lu waypoints from QGroundControl", msg->waypoints.size());

            // Publish the waypoints to CustomMode node
            custom_waypoint_publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "No waypoints received yet.");
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_subscriber_;
    rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr custom_waypoint_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
