#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>

using namespace std::chrono_literals;
using namespace Catch::Matchers;

const auto TEST_DURATION = 20;
rclcpp::Time last_time = rclcpp::Clock().now();

TEST_CASE("Verify identity transform between odom and base_footprint frames", "[integration]")
{

  auto node = rclcpp::Node::make_shared("integration_test_node");
  RCLCPP_INFO_STREAM(node->get_logger(), "Case 1");

  auto from_frame{"odom"};
  auto tf_found{false};
  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  node->declare_parameter("body_id", "blue/base_footprint");
  auto to_frame = node->get_parameter("body_id").as_string();

  geometry_msgs::msg::TransformStamped tf{};

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !tf_found)
  {
    try {
      tf = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
      tf_found = true;
    } catch (const tf2::TransformException & ex) {
    }
    rclcpp::spin_some(node);
    }
    REQUIRE_THAT(tf.transform.translation.x, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(tf.transform.translation.y, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(tf.transform.translation.z, WithinAbs(0.0, 0.000001));

    REQUIRE_THAT(tf.transform.rotation.x, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(tf.transform.rotation.y, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(tf.transform.rotation.z, WithinAbs(0.0, 0.000001));
    REQUIRE_THAT(tf.transform.rotation.w, WithinAbs(1.0, 0.000001));
    RCLCPP_INFO_STREAM(node->get_logger(), "Case 1");
}


std::shared_ptr<nav_msgs::msg::Odometry> odom1;
std::shared_ptr<nav_msgs::msg::Odometry> odom2;

auto received_msg {false};
auto sent_srv {false};

void odom_cb_(std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
  if (!received_msg) { // Save the first received odom state
    odom1 = msg;
    received_msg = true;
  } else if (received_msg) { // Repeatedly save the next odom state until I cut off the test when the service responds
    odom2 = msg;
  }
}


TEST_CASE(
  "Verify that initial pose service works by spawning bot, then initial posing it to a different location",
  "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");
  RCLCPP_INFO_STREAM(node->get_logger(), "Case 2");

  auto odom_sub =
    node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                                                         odom_cb_);

  auto client = node->create_client<nuturtle_control_interfaces::srv::InitialPose>("initial_pose");

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !sent_srv)
  {
    if (!received_msg) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Case 2, no msg yet");
    } else if (received_msg and !sent_srv) {
      auto request = std::make_shared<nuturtle_control_interfaces::srv::InitialPose::Request>(); // Service call
      request->x0 = 1.0;
      request->y0 = -1.0;
      request->theta0 = std::numbers::pi;
      sent_srv = true;
      auto result_future = client->async_send_request(request);
      rclcpp::spin_until_future_complete(node, result_future);

    } else
    {}
    rclcpp::spin_some(node);
  }
  start_time = rclcpp::Clock().now(); // Add a bit of a delay so that the most recent message gets the odom state updates
  while (rclcpp::ok &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(0.25)))
  {
    rclcpp::spin_some(node);
      }


      RCLCPP_INFO_STREAM(node->get_logger(), "odom x: " << odom1->pose.pose.position.x);

      REQUIRE_THAT(odom1->pose.pose.position.x, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom1->pose.pose.position.y, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom1->pose.pose.position.z, WithinAbs(0.0, 0.000001));

      REQUIRE_THAT(odom1->pose.pose.orientation.x, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom1->pose.pose.orientation.y, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom1->pose.pose.orientation.z, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom1->pose.pose.orientation.w, WithinAbs(1.0, 0.000001));

      REQUIRE_THAT(odom2->pose.pose.position.x, WithinAbs(1.0, 0.000001));
      REQUIRE_THAT(odom2->pose.pose.position.y, WithinAbs(-1.0, 0.000001));
      REQUIRE_THAT(odom2->pose.pose.position.z, WithinAbs(0.0, 0.000001));

      REQUIRE_THAT(odom2->pose.pose.orientation.x, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom2->pose.pose.orientation.y, WithinAbs(0.0, 0.000001));
      REQUIRE_THAT(odom2->pose.pose.orientation.z, WithinAbs(1.0, 0.000001));
      REQUIRE_THAT(odom2->pose.pose.orientation.w, WithinAbs(0.0, 0.000001));
}
