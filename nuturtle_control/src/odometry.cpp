#include "rclcpp/rclcpp.hpp"


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
try
{
wheel_left = this->get_parameter("wheel_left").as_string();
}
catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException &error_msg)
{      RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter 'wheel_left' not specified");
}

try
{
wheel_right = this->get_parameter("wheel_right").as_string();
}
catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException &error_msg)
{      RCLCPP_ERROR_STREAM(this->get_logger(), "Parameter 'wheel_right' not specified");
}


  }

private:
  std::string body_id {"base_footprint"};
  std::string odom_id {"odom"};
std::string wheel_left {"x"};
std::string wheel_right {"x"};
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
