

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using Empty = std_srvs::srv::Empty;
using MarkerArray = visualization_msgs::msg::MarkerArray;

class nusim_node : public rclcpp::Node
{
public:
    nusim_node()
    : Node("nusim")
    {
    // Create parameter 'rate', default 100 Hz
    this->declare_parameter("rate", 100); // TODO: check removing the this->
    int rate = this->get_parameter("rate").as_int();

    auto timer_callback = [this]() -> void
        { // TODO: Check removing the -> void, moving the whole lambda
        RCLCPP_INFO(this->get_logger(), "Tick Tock"); // TODO: check using RCLCPP_DEBUG_STREAM
        };
    auto timer_period {std::chrono::milliseconds(1000/rate)};
    timer_ = this->create_wall_timer(timer_period, timer_callback);
    // Create ~/reset service, type std/srv/Empty
    // Resets simulation
    // Right now, just resets the ~/timestep to 0
    // At next task, the position of the robot should be reset to x0, y0, theta0
    // Read the parameters at that time, not the values at initialization

    // Walls
    // Create parameters arena_x_length, arena_y_length
    this->declare_parameter("arena_x_length", 10); // TODO: check removing the this->
    double arena_x_length = this->get_parameter("arena_x_length").as_double();
    this->declare_parameter("arena_y_length", 10); // TODO: check removing the this->
    double arena_y_length = this->get_parameter("arena_y_length").as_double();

    // Center arena at 0,0
    // Walls 0.25 tall
    // Walls are red
    // use visualization_msgs/MarkerArray on topic ~/real_walls
        // Check qos from assignment 2
    rclcpp::QoS marker_qos(10);
    marker_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto wall_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("~/real_walls", marker_qos);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MarkerArray>::SharedPtr wall_pub_;

    // The following section creates a wall marker
    double wall_thick{0.01};
    double wall_height{0.25};
    visualization_msgs::msg::Marker(0) marker;
    marker.header.frame_id = "~/world";
    marker.header.stamp = rclcpp::Clock().now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.color.r = 1.0;
    marker.a = 0.75;

    // geometry_msgs::msg::Point p1{x : arena_x_length, y : arena_y_length, z : wall_height / 2.0};
};

std::shared_ptr<nusim_node> my_node = nullptr;

// Create parameters x0, y0, theta0 for initial pose of red turtle
    // default all to 0.0
    // relative to nusim/world frame


// Obstacles
    // Not specified how to get this to work
    // Parameters
        // obstacles.x is a list of x coordinates, float64
        // obstacles.y is a list of y coordinates, float64
        // obstacles.r is the radius (all abstacles are the same radius)
    // All obstacles are .25 m tall
    // All obstacles are red
    // Publish visualization_msgs/MarkerArray on ~/real_obstacles
        // PUblish in the red namespace of Marker message


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    my_node = std::make_shared<nusim_node>();
    // auto toggle_service = my_node->create_service<Empty>("toggle", toggle_cb);
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
