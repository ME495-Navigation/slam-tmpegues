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
  auto dd = turtlelib::DiffDrive(track_width, wheel_radius);


 // Twist to wheel command code is from turtle_control.cpp
 // auto motor_cmd_per_rad_sec = 0.024;
 // auto motor_cmd_max = 256;

  // auto body_twist = .5*turtlelib::Twist2D(0.1, 0.2, 0.0);

  // auto wheelrad_cmd = dd.ik(body_twist);
  // int lefttick_cmd{static_cast<int>(wheelrad_cmd.l() / motor_cmd_per_rad_sec)};
  // int righttick_cmd{static_cast<int>(wheelrad_cmd.r() / motor_cmd_per_rad_sec)};
  // lefttick_cmd = ((lefttick_cmd > motor_cmd_max) ? motor_cmd_max : lefttick_cmd);
  // righttick_cmd = ((righttick_cmd > motor_cmd_max) ? motor_cmd_max : righttick_cmd);

  // lefttick_cmd = ((lefttick_cmd < -motor_cmd_max) ? -motor_cmd_max : lefttick_cmd);
  // righttick_cmd = ((righttick_cmd < -motor_cmd_max) ? -motor_cmd_max : righttick_cmd);

  auto time_diff {0.01};


  // lefttick_cmd = 100;
  // righttick_cmd = 150;

  // Try spinning left and right at +- pi/2 radians /sec
  dd.set_speeds(turtlelib::WheelDiff(0, pi / 2));

  // std::cout << "Before any FK:\n";
  // std::cout << "Time :" << time_diff << "\n";
  // std::cout << "x, y, theta: " << dd.get_transform().translation() << ", " << dd.get_transform().rotation() << "\n";
  // std::cout << "Wheels, phi: " << dd.phi().l() << ", " << dd.phi().r() << "\n";
  // ;
  // std::cout << "Wheelspeeds, phidot: " << dd.phidot().l() << ", " << dd.phidot().r() << "\n";
  // std::cout << "All x, y will be given after FK, except the next line\n";
  std::cout << dd.get_transform().translation() << "\n";

  int i = 0;
  while (i <= 1000) { // 1 second
    dd.fk(time_diff);
    std::cout << dd.get_transform().translation() << "\n";
    i++;
  }

}
