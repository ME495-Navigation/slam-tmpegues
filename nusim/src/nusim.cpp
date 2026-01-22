#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// #include "std_msgs/msg/uint64.hpp"

using Empty = std_srvs::srv::Empty;
using MarkerArray = visualization_msgs::msg::MarkerArray;

class nusim_node : public rclcpp::Node
{
public:
    std::vector<double> obs_x {};
    std::vector<double> obs_y {};
    double obs_r {0.25};
    nusim_node():Node("nusim")


    {
        // Create parameters x0, y0, theta0 for initial pose of red turtle
        // default all to 0.0
        // relative to nusim/world frame
        // this->declare_parameter("x0", 0.0); // TODO: check removing the this->
        // double x = this->get_parameter("x0").as_double();

        // this->declare_parameter("y0", 0.0); // TODO: check removing the this->
        // double y = this->get_parameter("y0").as_double();

        // this->declare_parameter("theta0", 0.0); // TODO: check removing the this->
        // double theta = this->get_parameter("theta0").as_double();

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        // Create ~/reset service, type std/srv/Empty
        // Resets simulation
        // Right now, just resets the ~/timestep to 0
        //   auto timestep = std_msgs::msg::UInt64();
        //   timestep.data = 0
        // At next task, the position of the robot should be reset to x0, y0, theta0
        // Read the parameters at that time, not the values at initialization


        // Create parameter 'rate', default 100 Hz
        this->declare_parameter("rate", 100); // TODO: check removing the this->
        int rate = this->get_parameter("rate").as_int();

        auto timer_callback = [this]() -> void {          // TODO: Check removing the -> void, moving the whole lambda  // TODO: read about Lambda variable capture
            RCLCPP_INFO(this->get_logger(), "Tick Tock"); // TODO: check using RCLCPP_DEBUG_STREAM

        };
        auto timer_period{std::chrono::milliseconds(1000 / rate)};
        timer_ = this->create_wall_timer(timer_period, timer_callback);

        // std::vector empty_list<double> {};

        this->declare_parameter("obstacles.x", std::vector<double>{});
        this->declare_parameter("obstacles.y", std::vector<double>{});
        this->declare_parameter("obstacles.r", 0.25);

        obs_x = this->get_parameter("obstacles.x").as_double_array();
        obs_y = this->get_parameter("obstacles.y").as_double_array();
        obs_r = this->get_parameter("obstacles.r").as_double();

        // Check lengths of the lists against each other, then loop through calling the obs_cb_ for each one


        // // Create parameters x0, y0, theta0 for initial pose of red turtle
        //     // default all to 0.0
        //     // relative to nusim/world frame
        // this->declare_parameter("x0", 0.0); // TODO: check removing the this->
        // double x0 = this->get_parameter("x0").as_double();

        // this->declare_parameter("y0", 0.0); // TODO: check removing the this->
        // double y0 = this->get_parameter("y0").as_double();

        // this->declare_parameter("theta0", 0.0); // TODO: check removing the this->
        // double theta0 = this->get_parameter("theta0").as_double();

        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        wall_creator(); // This publishes the wall marker array
        obs_cb(); // Publish the pre-loaded obstacle parameters
    };


private:
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;

    rclcpp::QoS marker_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", marker_qos);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", marker_qos);

    void wall_creator()
    { // Walls
        // Create parameters arena_x_length, arena_y_length

        this->declare_parameter("arena_x_length", 10.0); // TODO: check removing the this->
        double arena_x_length = this->get_parameter("arena_x_length").as_double();
        this->declare_parameter("arena_y_length", 10.0); // TODO: check removing the this->
        double arena_y_length = this->get_parameter("arena_y_length").as_double();

        // Center arena at 0,0
        // Walls 0.25 tall
        // Walls are red
        // use visualization_msgs/MarkerArray on topic ~/real_walls
        // Check qos from assignment 2
        // rclcpp::QoS marker_qos(10);
        //  marker_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        // wall_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", marker_qos);

        // The following section creates wall marker array
        double wall_thick{0.1};
        double wall_height{0.25};
        auto marker_array = visualization_msgs::msg::MarkerArray();
        RCLCPP_INFO(this->get_logger(), "Before wall loop");
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "nusim/world";
        marker.scale.z = wall_height;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.color.r = 1.0;
        marker.color.a = 0.75;
        marker.pose.position.z = wall_height / 2.0;

        auto x_loc = arena_x_length / 2.0 + wall_thick / 2.0;
        auto y_loc = arena_y_length / 2.0 + wall_thick / 2.0;
        marker.header.stamp = rclcpp::Clock().now();

        marker.scale.x = wall_thick;
        marker.scale.y = arena_y_length + 2.0 * wall_thick;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = std::sqrt(2) / 2.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = std::sqrt(2) / 2.0;

        // +x wall
        marker.id = 0;
        marker.pose.position.x = x_loc;
        marker_array.markers.push_back(marker);

        // -x wall
        marker.id = 1;
        marker.pose.position.x = -x_loc;
        marker_array.markers.push_back(marker);

        marker.pose.position.x = 0.0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.w = 1;
        marker.scale.y = wall_thick;
        marker.scale.x = arena_x_length + 2.0 * wall_thick;

        // +y wall
        marker.id = 2;
        marker.pose.position.y = y_loc;
        marker_array.markers.push_back(marker);

        // -y wall
        marker.id = 3;
        marker.pose.position.y = -y_loc;
        marker_array.markers.push_back(marker);

        wall_pub_->publish(marker_array);
    }
    void obs_cb()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = rclcpp::Clock().now();
        marker.header.frame_id = "nusim/world";
        marker.ns = "red";

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.color.r = 1.0;
        marker.color.a = 0.75;
        // marker.scale.x = obs_r/2.0;
        // marker.scale.y = obs_r / 2.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.25;

        // Add loop here
        marker.id = 0;
        marker.pose.position.x = 1;
        marker.pose.position.y = 3;
        marker.pose.position.z = .25/2.0;


        marker_array.markers.push_back(marker);

        obs_pub_->publish(marker_array);
    }
};

std::shared_ptr<nusim_node> my_node = nullptr;


auto reset_cb(
    [[maybe_unused]] const std::shared_ptr<Empty::Request> request,
    [[maybe_unused]] const std::shared_ptr<Empty::Response> response) -> void
{
    RCLCPP_INFO(my_node->get_logger(), "Reset acknowledged, but can't do anything get");
    // my_node.x = my_node->get_parameter("x0").as_double();
    // my_node.y = my_node->get_parameter("y0").as_double();
    // my_node.theta = my_node->get_parameter("theta0").as_double();
};



// auto pub_redbase() -> void
// {
//     geometry_msgs::msg::TransformStamped red_pose;
//     red_pose.header.stamp = my_node->get_clock()->now();
//     red_pose.header.frame_id = "~/world";
//     red_pose.child_frame_id = "red/base_footprint";

//     red_pose.transform.translation.x = x;
//     red_pose.transform.translation.y = y;
//     red_pose.transform.translation.z = 0.1; // TODO: correct this height

//     tf2::Quaternion rot;
//     rot.setRPY(0, 0, theta);
//     red_pose.transform.rotation.x = rot.x();
//     red_pose.transform.rotation.y = rot.y();
//     red_pose.transform.rotation.z = rot.z();
//     red_pose.transform.rotation.w = rot.w();

//     my_node->tf_broadcaster_->sendTransform(red_pose);
// }

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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    my_node = std::make_shared<nusim_node>();
    auto reset_service = my_node->create_service<Empty>("~/reset", reset_cb);
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}