#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>

using namespace std::chrono_literals;
using namespace Catch::Matchers;

const auto TEST_DURATION = 20;
rclcpp::Time last_time = rclcpp::Clock().now();

std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg;
auto is_msg_received{false};


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
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) {       // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_DEBUG_STREAM(node->get_logger(), "Publishing zero twist");

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
  node->declare_parameter("motor_cmd_per_rad_sec", 0.024);

  auto wheel_radius = node->get_parameter("wheel_radius").as_double();
  auto motor_cmd_per_rad_sec = node->get_parameter("motor_cmd_per_rad_sec").as_double();

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10,
    wheel_cmd_cb2_);


  auto twist_msg{geometry_msgs::msg::Twist()};
  twist_msg.linear.x = wheel_radius; // 1 radian rotation to make other calculations easier

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) { // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_DEBUG_STREAM(node->get_logger(), "Publishing 2nd twist");

      cmd_vel_pub_->publish(twist_msg);
    }
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(received_msg2->left_velocity, WithinAbs(received_msg2->right_velocity, 0.000001));
  REQUIRE_THAT(received_msg2->right_velocity,
    WithinAbs(static_cast<int>(1 / motor_cmd_per_rad_sec), 0.000001));
}


std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg3;

void wheel_cmd_cb3_(std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
{
  received_msg3 = msg;
  is_msg_received = true;
}

TEST_CASE("Test negative translation", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  is_msg_received = false;

  node->declare_parameter("wheel_radius", 0.033);
  node->declare_parameter("motor_cmd_per_rad_sec", 0.024);

  auto wheel_radius = node->get_parameter("wheel_radius").as_double();
  auto motor_cmd_per_rad_sec = node->get_parameter("motor_cmd_per_rad_sec").as_double();

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10,
                                                                      wheel_cmd_cb3_);

  auto twist_msg{geometry_msgs::msg::Twist()};
  twist_msg.linear.x = -1.0 * wheel_radius; // 1 radian rotation to make other calculations easier

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) { // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_DEBUG_STREAM(node->get_logger(), "Publishing 3rd twist");

      cmd_vel_pub_->publish(twist_msg);
    }
    rclcpp::spin_some(node);
  }


  REQUIRE_THAT(received_msg3->left_velocity, WithinAbs(received_msg3->right_velocity, 0.000001));
  REQUIRE_THAT(received_msg3->right_velocity,
               WithinAbs(static_cast<int>(-1 / motor_cmd_per_rad_sec), 0.000001));
}


std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> received_msg4;

void wheel_cmd_cb4_(std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
{
  received_msg4 = msg;
  is_msg_received = true;
}

TEST_CASE("Test positive rotation", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  is_msg_received = false;

  node->declare_parameter("wheel_radius", 0.033);
  node->declare_parameter("motor_cmd_per_rad_sec", 0.024);
  node->declare_parameter("track_width", 0.16);

  auto wheel_radius = node->get_parameter("wheel_radius").as_double();
  auto motor_cmd_per_rad_sec = node->get_parameter("motor_cmd_per_rad_sec").as_double();
  auto track_width = node->get_parameter("track_width").as_double();

  auto cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub_ =
    node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10,
                                                                      wheel_cmd_cb4_);

  auto twist_msg{geometry_msgs::msg::Twist()};
  twist_msg.angular.z = 1; // 1 radian rotation to make other calculations easier

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) { // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      RCLCPP_DEBUG_STREAM(node->get_logger(), "Publishing 4th twist");

      cmd_vel_pub_->publish(twist_msg);
    }
    rclcpp::spin_some(node);
  }

  rclcpp::spin_some(node);
  REQUIRE_THAT(received_msg4->left_velocity,
    WithinAbs(static_cast<int>(-1 * track_width / 2 / wheel_radius / motor_cmd_per_rad_sec),
    0.000001));
  REQUIRE_THAT(received_msg4->right_velocity,
    WithinAbs(static_cast<int>(track_width / 2 / wheel_radius / motor_cmd_per_rad_sec), 0.000001));
}

std::shared_ptr<sensor_msgs::msg::JointState> received_msg5;

void joint_states_cb_(std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  received_msg5 = msg;
  is_msg_received = true;
}

TEST_CASE("Test 0 encoder to 0 joint_state", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  node->declare_parameter("encoder_ticks_per_rad", 651.89864);

  auto encoder_ticks_per_rad = node->get_parameter("encoder_ticks_per_rad").as_double();

  is_msg_received = false;
  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data",
    10);
  auto joint_state_sub_ =
    node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
                                                              joint_states_cb_);

  auto sensor_msg{nuturtlebot_msgs::msg::SensorData()};
  sensor_msg.left_encoder = encoder_ticks_per_rad * 2 * std::numbers::pi;
  sensor_msg.right_encoder = encoder_ticks_per_rad * (2 * std::numbers::pi / 4);

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)) &&
    !is_msg_received)
  {
    if ((rclcpp::Clock().now() - last_time) > rclcpp::Duration::from_seconds(1)) { // Only send a message every three seconds
      last_time = rclcpp::Clock().now();
      sensor_data_pub->publish(sensor_msg);
    }
    rclcpp::spin_some(node);
  }

  REQUIRE_THAT(received_msg5->position.at(0), WithinAbs(0.0, 0.01));  // TODO: Ask Matt if this reduced accuracy is a mistake
  REQUIRE_THAT(received_msg5->position.at(1), WithinAbs(std::numbers::pi / 2, 0.01));
}
