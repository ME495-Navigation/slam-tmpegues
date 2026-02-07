#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"


class turtle_control : public rclcpp::Node
{
public:
    turtle_control() : Node("turtle_control")

    {
        // Create all parameters
        this->declare_parameter("wheel_radius", 0.0);
        this->declare_parameter("track_width", 0.0);
        this->declare_parameter("motor_cmd_max", 0);
        this->declare_parameter("motor_cmd_per_rad_sec", 0.0);
        this->declare_parameter("encoder_tics_per_rad", 0);
        this->declare_parameter("collision_radius", 0.0);

        // Create all publishers/broadcasters and subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&turtle_control::cmd_vel_cb_, this, std::placeholders::_1));
        sensor_data_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("sensor_data", 10, std::bind(&turtle_control::sensor_data_cb_, this, std::placeholders::_1));

        // Define all variables
        wheel_radius = this->get_parameter("wheel_radius").as_double();
        track_width = this->get_parameter("track_width").as_double();
        motor_cmd_max = this->get_parameter("motor_cmd_max").as_int();
        motor_cmd_per_rad_sec = this->get_parameter("motor_cmd_per_rad_sec").as_double();
        encoder_tics_per_rad = this->get_parameter("encoder_tics_per_rad").as_int();
        collision_radius = this->get_parameter("collision_radius").as_double();

        // Define functions

        // Call setup functions
    };

private:
    double wheel_radius {0.0};
    double track_width {0.0};
    int motor_cmd_max {0};
    double motor_cmd_per_rad_sec {0.0};
    int encoder_tics_per_rad {0};
    double collision_radius {0.0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_ ;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_data_sub_ ;


    void cmd_vel_cb_(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Twist received: " << msg);
    }

    void sensor_data_cb_(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
    {
         RCLCPP_INFO_STREAM(this->get_logger(), "JointState received: " << msg);
    }
};

std::shared_ptr<turtle_control> my_node = nullptr;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    my_node = std::make_shared<turtle_control>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}