#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/wheels.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

class nusim_node : public rclcpp::Node
{
public:
  nusim_node()
  : Node("nusim")
  {
    // Create all parameters
    declare_parameter("rate", 100); // TODO: check removing the
    declare_parameter("arena_x_length", 3.0);
    declare_parameter("arena_y_length", 3.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.5);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);

    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("encoder_ticks_per_rad", 651.89864);

    // Create all publishers/broadcasters
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd",
                                                                               10,
      std::bind(&nusim_node::wheel_cmd_cb_, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    rclcpp::QoS marker_qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", marker_qos_);
    obs_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", marker_qos_);
    reset_service_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&nusim_node::reset_cb_, this, std::placeholders::_1, std::placeholders::_2));
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    // Define all variables
    timestep.data = 0;
    rate = get_parameter("rate").as_int();
    timer_period = 1000 / rate;

    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();

    obs_x = get_parameter("obstacles.x").as_double_array();
    obs_y = get_parameter("obstacles.y").as_double_array();
    obs_r = get_parameter("obstacles.r").as_double();

    auto x = get_parameter("x0").as_double();
    auto y = get_parameter("y0").as_double();
    auto theta = get_parameter("theta0").as_double();

    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

    red_dd = turtlelib::DiffDrive(track_width, wheel_radius,
                                  turtlelib::Transform2D(turtlelib::Vector2D{x, y}, theta));

    last_time = get_clock()->now();

    path.header.frame_id = "nusim/world";

    // Define functions
    auto timer_callback = [this]()
      -> void
      {
        red_dd.fk(1.0 / (1000.0 / float(timer_period))); // timer_period is in milliseconds, but I need it in seconds

      // Publish SensorData
        auto sensor_msg = nuturtlebot_msgs::msg::SensorData();
        sensor_msg.stamp = get_clock()->now();
        sensor_msg.left_encoder = red_dd.phi().l() * encoder_ticks_per_rad;
        sensor_msg.right_encoder = red_dd.phi().r() * encoder_ticks_per_rad;
        sensor_pub_->publish(sensor_msg);

      // Publish robot's TF
        auto t = tf2d_to_tfstamped(red_dd.get_transform());
        tf_broadcaster_->sendTransform(t);
        timestep_pub_->publish(timestep);
        timestep.data++;

      // Add pose to path at 1 Hz?
        if (timestep.data % 10 == 0) {
          auto p = tf2d_to_posestamped(red_dd.get_transform());
          path.poses.push_back(p);
          path_pub_->publish(path);
        }


      };

    timer_ = create_wall_timer(std::chrono::milliseconds(timer_period), timer_callback);
    RCLCPP_INFO_STREAM(get_logger(), "timer: " << std::chrono::milliseconds(timer_period));

    // Use setup functions
    create_walls();

    publish_obstacles();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time{};

  rclcpp::QoS marker_qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Wall dimensions
  double arena_x_length{3};
  double arena_y_length{3};

  // Obstacle locations
  std::vector<double> obs_x{};
  std::vector<double> obs_y{};
  double obs_r{0.25};

  // Red robot info
  double wheel_radius{0.0};
  double track_width{0.0};
  turtlelib::DiffDrive red_dd{
    track_width,
    wheel_radius};
  double motor_cmd_per_rad_sec{0.0};
  double encoder_ticks_per_rad{0.0};

  // Timer rate
  int rate{};
  int timer_period{};

  std_msgs::msg::UInt64 timestep;
  nav_msgs::msg::Path path;

  void wheel_cmd_cb_(const std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
  {
    // 0301 When a wheel command is received, it gets saved as the speed in the DiffDrive
    red_dd.set_speeds(turtlelib::WheelDiff(msg->left_velocity * motor_cmd_per_rad_sec,
                                           msg->right_velocity * motor_cmd_per_rad_sec));
  }

  void create_walls()
  { // The following section creates wall marker array and sets all variables that don't change
    double wall_thick{0.1};
    double wall_height{0.25};
    auto marker_array = visualization_msgs::msg::MarkerArray();
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
    RCLCPP_INFO_STREAM(get_logger(), "Publishing obstacles");

    if (obs_x.size() != obs_y.size()) {
      RCLCPP_ERROR_STREAM(
          get_logger(), "Obstacle coordinate list lengths do not match:"
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

  geometry_msgs::msg::TransformStamped tf2d_to_tfstamped(turtlelib::Transform2D tf)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "red pose: " << tf.translation() << " " << tf.rotation());

    geometry_msgs::msg::TransformStamped t{};
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = tf.translation().x;
    t.transform.translation.y = tf.translation().y;

    tf2::Quaternion q;
    q.setRPY(0, 0, tf.rotation());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
  }

  geometry_msgs::msg::PoseStamped tf2d_to_posestamped(const turtlelib::Transform2D tf)
  {
    geometry_msgs::msg::PoseStamped p{};
    p.header.stamp = get_clock()->now();
    p.header.frame_id = "nusim/world";

    p.pose.position.x = tf.translation().x;
    p.pose.position.y = tf.translation().y;

    tf2::Quaternion q;
    q.setRPY(0, 0, tf.rotation());
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    return p;
  }

  void reset_cb_(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Reset acknowledged at timestep " << timestep.data);
    timestep.data = 0;

    auto x = get_parameter("x0").as_double();
    auto y = get_parameter("y0").as_double();
    auto theta = get_parameter("theta0").as_double();

    red_dd = turtlelib::DiffDrive(track_width, wheel_radius,
      turtlelib::Transform2D(turtlelib::Vector2D{x, y}, theta));
  }
};

std::shared_ptr<nusim_node> my_node = nullptr;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  my_node = std::make_shared<nusim_node>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
