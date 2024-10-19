#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>

class WaypointReceiverTest : public ::testing::Test
{
protected:
    WaypointReceiverTest()
    {
        // Initialize ROS2 and create a node
        rclcpp::init(0, nullptr);
        node_ = rclcpp::Node::make_shared("test_waypoint_receiver");
    }

    ~WaypointReceiverTest()
    {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
};

// Test that the waypoint receiver can subscribe to the waypoint list
TEST_F(WaypointReceiverTest, WaypointSubscriptionTest)
{
    // Create a subscription to the waypoint list topic
    auto subscription = node_->create_subscription<mavros_msgs::msg::WaypointList>(
        "/mavros/mission/waypoints", 10,
        [](const mavros_msgs::msg::WaypointList::SharedPtr msg)
        {
            ASSERT_GT(msg->waypoints.size(), 0);  // Ensure that waypoints are received
        });

    // Spin the node for a short time to receive messages
    rclcpp::spin_some(node_);
}

// Test that the waypoint receiver processes waypoint correctly
TEST_F(WaypointReceiverTest, WaypointProcessingTest)
{
    auto publisher = node_->create_publisher<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", 10);

    // Prepare a dummy waypoint list message
    mavros_msgs::msg::WaypointList waypoint_msg;
    mavros_msgs::msg::Waypoint waypoint;
    waypoint.x_lat = 37.7749;  // Example latitude
    waypoint.y_long = -122.4194;  // Example longitude
    waypoint.z_alt = 100.0;  // Example altitude
    waypoint_msg.waypoints.push_back(waypoint);

    // Publish the message
    publisher->publish(waypoint_msg);

    // Create a subscription to check if the waypoint message is processed
    auto subscription = node_->create_subscription<mavros_msgs::msg::WaypointList>(
        "/mavros/mission/waypoints", 10,
        [waypoint_msg](const mavros_msgs::msg::WaypointList::SharedPtr msg)
        {
            // Verify the received waypoints match the published data
            ASSERT_EQ(msg->waypoints.size(), waypoint_msg.waypoints.size());
            ASSERT_FLOAT_EQ(msg->waypoints[0].x_lat, waypoint_msg.waypoints[0].x_lat);
            ASSERT_FLOAT_EQ(msg->waypoints[0].y_long, waypoint_msg.waypoints[0].y_long);
            ASSERT_FLOAT_EQ(msg->waypoints[0].z_alt, waypoint_msg.waypoints[0].z_alt);
        });

    // Spin to let the message propagate
    rclcpp::spin_some(node_);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
