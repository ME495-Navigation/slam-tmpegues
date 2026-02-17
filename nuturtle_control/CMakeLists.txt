cmake_minimum_required(VERSION 3.22)
project(nuturtle_control)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtle_control_interfaces REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(Catch2 REQUIRED)



add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)
add_executable(turtle_control_test test/turtle_control_test.cpp)
add_executable(turtle_odom_test_node test/turtle_odom_test_node.cpp)
add_executable(turtle_circle_test test/turtle_circle_test.cpp)




target_link_libraries(turtle_control
                      PUBLIC
                      rclcpp::rclcpp
                      turtlelib::turtlelib
                      ${nuturtlebot_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${sensor_msgs_TARGETS})

target_link_libraries(odometry
                      PUBLIC
                      rclcpp::rclcpp
                      turtlelib::turtlelib
                      tf2::tf2
                      tf2_ros::tf2_ros
                      ${nav_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${sensor_msgs_TARGETS}
                      ${nuturtle_control_interfaces_TARGETS})


target_link_libraries(circle
                      PUBLIC
                      rclcpp::rclcpp
                      turtlelib::turtlelib
                      ${std_srvs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${sensor_msgs_TARGETS}
                      ${nuturtle_control_interfaces_TARGETS})

target_link_libraries(turtle_control_test
                      PUBLIC
                      Catch2::Catch2
                      rclcpp::rclcpp
                      catch_ros2::catch_ros2_with_node_main
                      ${geometry_msgs_TARGETS}
                      ${nuturtlebot_msgs_TARGETS}
                      ${sensor_msgs_TARGETS})

target_link_libraries(turtle_odom_test_node
                      PUBLIC
                      tf2::tf2
                      tf2_ros::tf2_ros
                      Catch2::Catch2
                      rclcpp::rclcpp
                      catch_ros2::catch_ros2_with_node_main
                      ${geometry_msgs_TARGETS}
                      ${nuturtlebot_msgs_TARGETS}
                      ${sensor_msgs_TARGETS}
                      ${nav_msgs_TARGETS}
                      ${nuturtle_control_interfaces_TARGETS})

target_link_libraries(turtle_circle_test
                      PUBLIC
                      Catch2::Catch2
                      rclcpp::rclcpp
                      catch_ros2::catch_ros2_with_node_main
                      ${geometry_msgs_TARGETS})

catch_ros2_add_integration_test(turtle_control_test
  LAUNCH_FILE turtle_control_test.launch.xml)


install(TARGETS
  circle
  turtle_control
  odometry
  turtle_control_test
  turtle_odom_test_node
  turtle_circle_test
  DESTINATION lib/${PROJECT_NAME})

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
