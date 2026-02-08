cmake_minimum_required(VERSION 3.20)
project(nuturtle_control)

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
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)

add_executable(turtle_control src/turtle_control.cpp)


target_link_libraries(turtle_control
                      PUBLIC
                      rclcpp::rclcpp
                      turtlelib::turtlelib
                      ${nuturtlebot_msgs_TARGETS}
                      ${std_srvs_TARGETS}
                      ${std_msgs_TARGETS}
                      ${visualization_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${sensor_msgs_TARGETS})

install(TARGETS
  turtle_control
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
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
