#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/se2d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// #include "turtlelib"

class nusim_node : public rclcpp::Node
{
public:
  nusim_node()
  : Node("nusim")
  {
    // Create all parameters
    this->declare_parameter("rate", 100);  // TODO: check removing the this->
    this->declare_parameter("arena_x_length", 3.0);
    this->declare_parameter("arena_y_length", 3.0);
    this->declare_parameter("obstacles.x", std::vector<double>{});
    this->declare_parameter("obstacles.y", std::vector<double>{});
    this->declare_parameter("obstacles.r", 0.5);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);

    // Create all publishers/broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    rclcpp::QoS marker_qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", marker_qos_);
    obs_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", marker_qos_);
    reset_service_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&nusim_node::reset_cb_, this, std::placeholders::_1, std::placeholders::_2));
    timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Define all variables
    timestep.data = 0;
    rate = this->get_parameter("rate").as_int();
    timer_period = std::chrono::milliseconds(1000 / rate);

    arena_x_length = this->get_parameter("arena_x_length").as_double();
    arena_y_length = this->get_parameter("arena_y_length").as_double();

    obs_x = this->get_parameter("obstacles.x").as_double_array();
    obs_y = this->get_parameter("obstacles.y").as_double_array();
    obs_r = this->get_parameter("obstacles.r").as_double();

    red_x = this->get_parameter("x0").as_double();
    red_y = this->get_parameter("y0").as_double();
    red_theta = this->get_parameter("theta0").as_double();

    // Define functions

    auto timer_callback = [this]()
      -> void {  // TODO: Check removing the -> void, moving the whole lambda  // TODO: read about Lambda variable capture
      // RCLCPP_INFO_STREAM(this->get_logger(), "Tick Tock: " << timestep.data);
        auto t = tl_point_to_pose(red_x, red_y, red_theta);
        tf_broadcaster_->sendTransform(t);
        timestep_pub_->publish(timestep);
        timestep.data++;
      };

    timer_ = this->create_wall_timer(timer_period, timer_callback);

    // Use setup functions
    create_walls();
    publish_obstacles();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::QoS marker_qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Wall dimensions
  double arena_x_length{3};
  double arena_y_length{3};

  // Obstacle locations
  std::vector<double> obs_x{};
  std::vector<double> obs_y{};
  double obs_r{0.25};

  // Red robot location
  double red_x{0};
  double red_y{0};
  double red_theta{0};

  // Timer rate
  int rate{};
  std::chrono::milliseconds timer_period{};

  std_msgs::msg::UInt64 timestep;

  void create_walls()
  {  // The following section creates wall marker array and sets all variables that don't change
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

  void publish_obstacles()
  {
    if (obs_x.size() != obs_y.size()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Obstacle coordinate list lengths do not match:"
                              << obs_x.size() << " and " << obs_y.size());
    } else {
      auto marker_array = visualization_msgs::msg::MarkerArray();
      auto marker = visualization_msgs::msg::Marker();
      marker.header.stamp = rclcpp::Clock().now();
      marker.header.frame_id = "nusim/world";
      marker.ns = "red";

      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.color.r = 1.0;
      marker.color.a = 0.75;
      marker.scale.x = obs_r / 2.0;
      marker.scale.y = obs_r / 2.0;

      marker.scale.z = 0.25;

      for (unsigned int i = 0; i <= obs_x.size() - 1; i++) {
        marker.id = i;
        marker.pose.position.x = obs_x[i];
        marker.pose.position.y = obs_y[i];
        marker.pose.position.z = .25 / 2.0;

        marker_array.markers.push_back(marker);
      }
      obs_pub_->publish(marker_array);
    }
  }

  geometry_msgs::msg::TransformStamped tl_point_to_pose(
    const double x, const double y, const double theta)
  {
    geometry_msgs::msg::TransformStamped t{};
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
  }

  void reset_cb_(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Reset acknowledged at timestep " << timestep.data);
    timestep.data = 0;
    red_x = this->get_parameter("x0").as_double();
    red_y = this->get_parameter("y0").as_double();
    red_theta = this->get_parameter("theta0").as_double();
  }
};

std::shared_ptr<nusim_node> my_node = nullptr;

// auto reset_cb(
//     [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//     [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
// {
//     RCLCPP_INFO_STREAM(my_node->get_logger(), "Reset acknowledged, timestep " << my_node->timestep.data << "is now " << my_node->timestep.data = 0);
// };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  my_node = std::make_shared<nusim_node>();
  // auto reset_service = my_node->create_service<std_srvs::srv::Empty>("~/reset", reset_cb);
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
