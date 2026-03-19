cmake_minimum_required(VERSION 3.20)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

  find_package(Armadillo REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(nuturtle_control_interfaces REQUIRED)
  find_package(nuturtlebot_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(turtlelib REQUIRED)
  find_package(visualization_msgs REQUIRED)

  add_executable(nusim src/nusim.cpp)
  target_link_libraries(nusim
                        PUBLIC
                        ${ARMADILLO_LIBRARIES} #TODO: cite Theo
                        rclcpp::rclcpp
                        tf2_ros::tf2_ros
                        tf2::tf2
                        turtlelib::turtlelib
                        ${nav_msgs_TARGETS}
                        ${geometry_msgs_TARGETS}
                        ${nuturtle_control_interfaces_TARGETS}
                        ${nuturtlebot_msgs_TARGETS}
                        ${sensor_msgs_TARGETS}
                        ${std_msgs_TARGETS}
                        ${std_srvs_TARGETS}
                        ${visualization_msgs_TARGETS}
                        )

install(TARGETS
  nusim
  DESTINATION lib/${PROJECT_NAME})

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
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
