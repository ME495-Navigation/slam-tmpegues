#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace std::chrono_literals;
using namespace Catch::Matchers;

std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg;
auto is_msg_received{false};

void wheel_cmd_cb_(std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
{
  received_msg = msg;
  is_msg_received = true;
}

TEST_CASE("Test zero twist to wheel cmd", "[integration]")
{

    auto node = rclcpp::Node::make_shared("integration_test_node");

    auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10, wheel_cmd_cb_);

    const auto TEST_DURATION = 20;

    rclcpp::Time start_time = rclcpp::Clock().now();
    rclcpp::Time last_time = rclcpp::Clock().now();
    while
  (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received
  )
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) {       // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_INFO_STREAM(node->get_logger(), "Publishing twist");

      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    rclcpp::spin_some(node);
        }
    REQUIRE_THAT(received_msg->left_velocity, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(received_msg->right_velocity, WithinAbs(0.0, 0.000001));
}
