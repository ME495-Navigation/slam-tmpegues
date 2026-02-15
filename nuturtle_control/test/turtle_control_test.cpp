#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>

using namespace std::chrono_literals;
using namespace Catch::Matchers;

std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg;
auto is_msg_received{false};
const auto TEST_DURATION = 20;

void wheel_cmd_cb_(std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
{
  received_msg = msg;
  is_msg_received = true;
}

TEST_CASE("Test zero twist to wheel cmd", "[integration]")
{
  is_msg_received = false;
  auto node = rclcpp::Node::make_shared("integration_test_node");

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10, wheel_cmd_cb_);

  rclcpp::Time start_time = rclcpp::Clock().now();
  rclcpp::Time last_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) {       // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_INFO_STREAM(node->get_logger(), "Publishing zero twist");

      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
    rclcpp::spin_some(node);
        }

    REQUIRE_THAT(received_msg->left_velocity, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(received_msg->right_velocity, WithinAbs(0.0, 0.000001));
}

std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg2;

void wheel_cmd_cb2_(std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
{
  received_msg2 = msg;
  is_msg_received = true;
}

TEST_CASE("Test positive translation", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  is_msg_received = false;

  node->declare_parameter("wheel_radius", 0.033);
  node->declare_parameter("motor_cmd_max", 256);
  node->declare_parameter("motor_cmd_per_rad_sec", 0.024);

  auto wheel_radius = node->get_parameter("wheel_radius").as_double();
  auto motor_cmd_max = node->get_parameter("motor_cmd_max").as_int();
  auto motor_cmd_per_rad_sec = node->get_parameter("motor_cmd_per_rad_sec").as_double();

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10,
    wheel_cmd_cb2_);


  auto twist_msg{geometry_msgs::msg::Twist()};
  twist_msg.linear.x = wheel_radius; // 1 radian rotation to make other calculations easier

  rclcpp::Time start_time = rclcpp::Clock().now();
  rclcpp::Time last_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) { // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_INFO_STREAM(node->get_logger(), "Publishing 2nd twist");

      cmd_vel_pub_->publish(twist_msg);
    }
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(received_msg2->left_velocity, WithinAbs(received_msg2->right_velocity, 0.000001));
  REQUIRE_THAT(received_msg2->right_velocity,
    WithinAbs(static_cast<int>(1 / motor_cmd_per_rad_sec), 0.000001));
}
