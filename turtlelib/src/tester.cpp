#include <cmath>
#include <iostream>
#include <fstream>
#include <print>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
using std::numbers::pi;

int main()
{
  std::ofstream tester_file{"tester.txt"};

  auto wheel_radius = 0.033;
  auto track_width = 0.16;
  auto dd = turtlelib::DiffDrive(track_width, wheel_radius,
    turtlelib::Transform2D(turtlelib::Vector2D(0.0, 0.0), 0.0));

  auto motor_cmd_per_rad_sec = 0.024;

  auto time_diff {0.01};
  auto wheel_cmd = turtlelib::WheelDiff(0, 100);
  dd.set_speeds(motor_cmd_per_rad_sec * wheel_cmd);
  tester_file << "Wheel cmd\nx0, y0\ndd properties\ntime step\n";
  tester_file << wheel_cmd.l() << ", " << wheel_cmd.r() << "\n";
  tester_file << dd.get_transform().translation() << "\n";
  tester_file << track_width << ", " << wheel_radius << ", " << motor_cmd_per_rad_sec << "\n";
  tester_file << time_diff << "\n";

  printf("point, %f", dd.get_transform().translation() )
  int i = 0;


  while (i < 15000) {

    dd.fk(time_diff);

    i++;
  }

}
