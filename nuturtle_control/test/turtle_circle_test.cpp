#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace Catch::Matchers;

const auto TEST_DURATION = 20;

auto msg_count {0};
rclcpp::Time first_collected{};
rclcpp::Time last_collected{};

void cmd_vel_cb_([[maybe_unused]] std::shared_ptr<geometry_msgs::msg::Twist> msg)
{

  msg_count += 1;
  if (msg_count == 1) {
    first_collected = rclcpp::Clock().now();
  } else if (msg_count == 100) {
    last_collected = rclcpp::Clock().now();
  }
}

TEST_CASE("Check frequency of cmd_vel from circle node", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");
  node->declare_parameter("frequency", 100);
  auto frequency = node->get_parameter("frequency").as_int();

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, cmd_vel_cb_);

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    msg_count < 100)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "messages received: " << msg_count);
    rclcpp::spin_some(node);
  }


  auto time_diff{last_collected.seconds() + last_collected.nanoseconds() / 10e9 -
    first_collected.seconds() - first_collected.nanoseconds() / 10e9};
  REQUIRE_THAT(msg_count / time_diff, WithinAbs(frequency, 1.0));
}
