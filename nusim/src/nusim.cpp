

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using Empty = std_srvs::srv::Empty;

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
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;

};

std::shared_ptr<nusim_node> my_node = nullptr;

// Create parameters x0, y0, theta0 for initial pose of red turtle
    // default all to 0.0
    // relative to nusim/world frame

// Walls
    // Create parameters arena_x_length, arena_y_length
    // Center arena at 0,0
    // Walls 0.25 tall
    // Walls are red
    // use visualization_msgs/MarkerArray on topic ~/real_walls
        // Check qos from assignment 2

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
