#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"


class odometry : public rclcpp::Node
{
public:
  odometry()
  : Node("odometry")
  {
    this->declare_parameter("body_id", "base_footprint");
    this->declare_parameter("odom_id", "odom");
    this->declare_parameter<std::string>("wheel_left");
    this->declare_parameter<std::string>("right_left");



    body_id = this->get_parameter("body_id").as_string();
    odom_id = this->get_parameter("odom_id").as_string();
    // wheel_left and wheel_right MUST be provided, there is no default value
    try {
      wheel_left = this->get_parameter("wheel_left").as_string();
    } catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException & error_msg) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter 'wheel_left' not specified");
    }

    try {
      wheel_right = this->get_parameter("wheel_right").as_string();
    } catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException & error_msg) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter 'wheel_right' not specified");
    }


    last_time = this->get_clock()->now();

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_state", 10, std::bind(&odometry::joint_state_cb_, this, std::placeholders::_1));

  }

private:
  builtin_interfaces::msg::Time last_time{};

  std::string body_id {"base_footprint"};
  std::string odom_id {"odom"};
  std::string wheel_left {"x"};
  std::string wheel_right {"x"};

  nav_msgs::msg::Odometry()

  void joint_state_cb_(const std::shared_prt<sensor_msgs::msg::JointState> msg)
  {
    // JointState includes left and right positions, velocities, and time

    // 1st, fk to get new position
    auto time_diff{
      msg->stamp.sec + msg->stamp.nanosec / 10e9 - last_time.sec - last_time.nanosec / 10e9};


  }
};

std::shared_ptr<odometry> my_node = nullptr;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  my_node = std::make_shared<odometry>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
