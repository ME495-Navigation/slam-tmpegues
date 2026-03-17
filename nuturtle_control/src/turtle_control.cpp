#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/wheels.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>

class turtle_control : public rclcpp::Node
{
public:
  turtle_control()
  : Node("turtle_control")
  {
    // Create all parameters
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("motor_cmd_max", 256);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.89864);
    declare_parameter("collision_radius", 0.11);

    // Create all publishers/broadcasters and subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&turtle_control::cmd_vel_cb_, this, std::placeholders::_1));
    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&turtle_control::sensor_data_cb_, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);


    // Define all variables
    last_time = get_clock()->now();
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    motor_rad_max = motor_cmd_max * motor_cmd_per_rad_sec;
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius = get_parameter("collision_radius").as_double();
    dd_calc = turtlelib::DiffDrive(
      track_width,
      wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Turtle Control node initialized");

    // DiffDrive initial wheel positions will need to be set once the first encoder readings are received
  }

private:
  builtin_interfaces::msg::Time last_time{};
  double wheel_radius{0.0};
  double track_width{0.0};
  int motor_cmd_max{0};
  double motor_cmd_per_rad_sec{0.0};
  double encoder_ticks_per_rad{0};
  double collision_radius{0.0};
  double motor_rad_max{0.0};

  turtlelib::DiffDrive dd_calc{
    track_width,
    wheel_radius};  // DiffDrive initial wheel positions will need to be set once the first encoder readings are received

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;

  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  void cmd_vel_cb_(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
  {
    RCLCPP_DEBUG_STREAM(get_logger(),
      "Twist received (angular z, linear x): " << msg->angular.z << ", " << msg->linear.x);
    if (msg->angular.x or msg->angular.y or msg->linear.z) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please provide a twist in the x-y plane.");
      return;
    }

    turtlelib::Twist2D twist_cmd{msg->angular.z, msg->linear.x, msg->linear.y};
    turtlelib::WheelDiff wheelrad_cmd;

    try {
      wheelrad_cmd = dd_calc.ik(twist_cmd);  // These are in radians per second
    } catch (std::logic_error & error_msg) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid cmd_vel: y component must be 0");
      return;
    }

    auto left_scale = ((std::fabs(wheelrad_cmd.l()) >
      motor_rad_max) ? std::fabs(motor_rad_max / wheelrad_cmd.l()) : 1.0);
    auto right_scale = ((std::fabs(wheelrad_cmd.r()) >
      motor_rad_max) ? std::fabs(motor_rad_max / wheelrad_cmd.r()) : 1.0);
    auto scale = (left_scale < right_scale ? left_scale : right_scale);
    RCLCPP_DEBUG_STREAM(get_logger(), "Scales: " << left_scale << ", " << right_scale);
    RCLCPP_DEBUG_STREAM(get_logger(), "Scale: " << scale);
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "Initial wheel rads: " << wheelrad_cmd.l() << ", " << wheelrad_cmd.r());

    wheelrad_cmd *= scale;
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "Adjusted wheel rads: " << wheelrad_cmd.l() << ", " << wheelrad_cmd.r());

    auto wheeltick_cmd = nuturtlebot_msgs::msg::WheelCommands();

    wheeltick_cmd.left_velocity = static_cast<int>(wheelrad_cmd.l() / motor_cmd_per_rad_sec);
    wheeltick_cmd.right_velocity = static_cast<int>(wheelrad_cmd.r() / motor_cmd_per_rad_sec);
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "Adjusted wheel cmds: " << wheeltick_cmd.left_velocity << ", " <<
      wheeltick_cmd.right_velocity);

    wheel_cmd_pub_->publish(wheeltick_cmd);
  }

  void sensor_data_cb_(const std::shared_ptr<nuturtlebot_msgs::msg::SensorData> msg)
  {
    RCLCPP_DEBUG_STREAM(get_logger(),
      "SensorData received: " << msg->left_encoder << ", " << msg->right_encoder);

    dd_calc.fk(turtlelib::Wheels(msg->left_encoder / encoder_ticks_per_rad,
      msg->right_encoder / encoder_ticks_per_rad));

    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = msg->stamp;

    joint_state_msg.name.push_back("wheel_left_joint");
    joint_state_msg.name.push_back("wheel_right_joint");

    joint_state_msg.position.push_back(dd_calc.phi().l());
    joint_state_msg.position.push_back(dd_calc.phi().r());

    joint_state_msg.velocity.push_back(dd_calc.phidot().l());
    joint_state_msg.velocity.push_back(dd_calc.phidot().r());

    joint_state_pub_->publish(joint_state_msg);
  }
};

std::shared_ptr<turtle_control> my_node = nullptr;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  my_node = std::make_shared<turtle_control>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
