#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control_interfaces/srv/control.hpp"

class circle : public rclcpp::Node
{
public:
  circle()
  : Node("circle")
  {
    declare_parameter("frequency", 100);
    frequency = get_parameter("frequency").as_int();
    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / frequency),
      std::bind(&circle::timer_cb_, this));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    control_service_ = create_service<nuturtle_control_interfaces::srv::Control>("control",
      std::bind(&circle::control_cb_, this, std::placeholders::_1, std::placeholders::_2));
    reverse_service_ = create_service<std_srvs::srv::Empty>(
            "reverse",
            std::bind(&circle::reverse_cb_, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Empty>(
            "stop",
            std::bind(&circle::stop_cb_, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<nuturtle_control_interfaces::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;

  int frequency {100};
  double velocity {1.0};
  double radius {1.0};
  bool stopped = false;
  geometry_msgs::msg::Twist circle_twist{geometry_msgs::msg::Twist()};

  void control_cb_(
    const std::shared_ptr<nuturtle_control_interfaces::srv::Control::Request> request,
    [[maybe_unused]] const std::shared_ptr<nuturtle_control_interfaces::srv::Control::Response>
    response)
  {
    // RCLCPP_DEBUG_STREAM(get_logger(), "Circle control vel, rad: " << request.velocity << ", " << request.radius);

    if (request->radius == 0.0)
    {
      circle_twist.angular.z = request->velocity;
      circle_twist.linear.x = 0;
    }
    else
    {
      circle_twist.angular.z = request->velocity / request->radius;
      circle_twist.linear.x = request->velocity;
    }
    stopped = false;
  }

  void reverse_cb_(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    stopped = false;
    circle_twist.angular.z *= -1;
    circle_twist.linear.x *= -1;
    RCLCPP_INFO_STREAM(get_logger(), "Reversed");
  }

  void stop_cb_(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    stopped = true;
  }

  void timer_cb_()
  {
    if (stopped) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    } else {
      cmd_vel_pub_->publish(circle_twist);
    }
  }
};

std::shared_ptr<circle> my_node = nullptr;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  my_node = std::make_shared<circle>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
