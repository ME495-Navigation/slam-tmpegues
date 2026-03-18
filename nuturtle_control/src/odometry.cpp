#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/wheels.hpp"

#include <fstream>

class odometry : public rclcpp::Node
{
public:
  odometry()
  : Node("odometry")
  {

    declare_parameter("body_id", "base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter<std::string>("wheel_left");
    declare_parameter<std::string>("wheel_right");

    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();


    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    // wheel_left and wheel_right MUST be provided, there is no default value
    try {
      wheel_left = get_parameter("wheel_left").as_string();
    } catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException & error_msg) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter 'wheel_left' not specified");
    }

    try {
      wheel_right = get_parameter("wheel_right").as_string();
    } catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException & error_msg) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter 'wheel_right' not specified");
    }

    last_time = get_clock()->now();
    dd_calc = turtlelib::DiffDrive(track_width, wheel_radius);

    odom_state.header.frame_id = odom_id;
    odom_state.child_frame_id = body_id;

    path.header.frame_id = odom_id;

    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / 100),
                                     std::bind(&odometry::timer_cb_, this));

    initial_pose_service_ = create_service<nuturtle_control_interfaces::srv::InitialPose>(
        "initial_pose",
        std::bind(&odometry::initial_pose_cb_, this, std::placeholders::_1, std::placeholders::_2));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
    std::bind(&odometry::joint_state_cb_, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr initial_pose_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  builtin_interfaces::msg::Time last_time{};

  std::string body_id {"base_footprint"};
  std::string odom_id {"odom"};
  std::string wheel_left {"x"};
  std::string wheel_right {"x"};

  double wheel_radius{0.0};
  double track_width{0.0};

  turtlelib::DiffDrive dd_calc{track_width, wheel_radius};

  nav_msgs::msg::Odometry odom_state = nav_msgs::msg::Odometry();
  nav_msgs::msg::Path path = nav_msgs::msg::Path();

  std::ofstream odom_tf_file{"odom_tf.txt"};
  std::ofstream odom_wheel_file{"odom_phi.txt"};

  void initial_pose_cb_(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    [[maybe_unused]] const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response>
    response)
  { // Reset the internal odom state to the newly received initial position
    RCLCPP_INFO_STREAM(get_logger(),
      "Initial pose service: " << request->x0 << " " << request->y0 << " " << request->theta0);

    dd_calc = turtlelib::DiffDrive(track_width, wheel_radius,
      turtlelib::Transform2D(turtlelib::Vector2D(request->x0, request->y0), request->theta0));

    RCLCPP_INFO_STREAM(get_logger(), "New dd: " << dd_calc.get_transform().translation() );

    odom_state.pose.pose.position.x = request->x0;
    odom_state.pose.pose.position.y = request->y0;

    RCLCPP_INFO_STREAM(get_logger(),
      "Odom state: " << odom_state.pose.pose.position.x << " " << odom_state.pose.pose.position.y);

    tf2::Quaternion q;
    q.setRPY(0, 0, request->theta0);
    odom_state.pose.pose.orientation.x = q.x();
    odom_state.pose.pose.orientation.y = q.y();
    odom_state.pose.pose.orientation.z = q.z();
    odom_state.pose.pose.orientation.w = q.w();


  }

  void joint_state_cb_(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
  {
    // JointState includes left and right positions, velocities, and time
    // FK to get position and velocity based on received wheel positions

    dd_calc.fk(turtlelib::Wheels(msg->position.at(0), msg->position.at(1)));
    odom_tf_file << dd_calc.get_transform().translation() << ", " <<
      dd_calc.get_transform().rotation() << "\n";
    odom_wheel_file << dd_calc.phi().l() << ", " << dd_calc.phi().r() << "\n";
  }


  void timer_cb_()
  {
    tf_broadcaster_->sendTransform((turtlelib_transform2d_to_msg(dd_calc.get_transform())));

    // Convert turtlelib format to ROS messages
    odom_state.pose.pose = turtlelib_transform_to_pose(dd_calc.get_transform());
    odom_state.twist.twist = turtlelib_twist2d_to_msg(dd_calc.get_twist());
    odom_state.header.stamp = this->get_clock()->now();

    odom_pub_->publish(odom_state);

    // Add pose to path at lower freq?
    static int timestep = 0;
    if (timestep >= 10)
    {
      geometry_msgs::msg::PoseStamped p{};
      p.header.stamp = get_clock()->now();
      p.header.frame_id = "nusim/world";
      p.pose = turtlelib_transform_to_pose(dd_calc.get_transform());
      path.poses.push_back(p);
      path_pub_->publish(path);
    }
  }

  geometry_msgs::msg::Pose turtlelib_transform_to_pose(
    const turtlelib::Transform2D tf)
  {
    geometry_msgs::msg::Pose p{};

    p.position.x = tf.translation().x;
    p.position.y = tf.translation().y;

    tf2::Quaternion q;
    q.setRPY(0, 0, tf.rotation());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    return p;
  }

  geometry_msgs::msg::TransformStamped turtlelib_transform2d_to_msg(const turtlelib::Transform2D tf)
  {
    geometry_msgs::msg::TransformStamped t{};
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;

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

  geometry_msgs::msg::Twist turtlelib_twist2d_to_msg(const turtlelib::Twist2D tw)
  {
    geometry_msgs::msg::Twist t{};
    t.linear.x = tw.x;
    t.linear.y = tw.y;
    t.angular.z = tw.omega;

    return t;
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
