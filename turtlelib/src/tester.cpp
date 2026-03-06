#include <cmath>
#include <iostream>
#include <print>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
using std::numbers::pi;

int main()
{
  auto wheel_radius = 0.033;
  auto track_width = 0.16;
  auto dd = turtlelib::DiffDrive(track_width, wheel_radius,
    turtlelib::Transform2D(turtlelib::Vector2D(1.0, 0.0), 0.0));

  auto motor_cmd_per_rad_sec = 0.024;

  auto time_diff {0.01};

  dd.set_speeds(motor_cmd_per_rad_sec * turtlelib::WheelDiff(0, 100));

  std::cout << dd.phi().l() << ", " << dd.phi().r() << "\n";
  int i = 0;


  while (i < 1000) {
    dd.fk(time_diff);
    i++;
  }

}
