#include <cmath>
#include <stdexcept>

#include <iostream>
#include <fstream>

#include "turtlelib/angle.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

Transform2D DiffDrive::get_transform() const {return q;}

Wheels DiffDrive::phi() const { return wheels; }

WheelDiff DiffDrive::phidot() const { return wheelspeeds; }

double DiffDrive::get_track() const { return track; }

double DiffDrive::get_radius() const { return radius; }

Twist2D DiffDrive::get_twist() const { return body_vel; }

DiffDrive::DiffDrive() {}

DiffDrive::DiffDrive(double input_track, double input_radius)
{
  track = input_track;
  radius = input_radius;
}

DiffDrive::DiffDrive(double input_track, double input_radius, Transform2D tf)
{
  track = input_track;
  radius = input_radius;
  q = tf;
}

void DiffDrive::set_speeds(WheelDiff speed)
{
  wheelspeeds = speed;
}

void DiffDrive::fk(Wheels wheels2)
{
  // The arguments received here are actual wheel positions, we don't need to scale by time
  // 0st, update wheel positions and get how far the wheels rotated
  wheel_fk_file << "1: " << wheels.r() << ", " << wheels.l() << "\n";
  auto update = wheels.get_diff(wheels2);
  wheel_fk_file << "2: " << wheels.r() << ", " << wheels.l() << "\n";
  wheel_fk_file << "D: " << update.r() << ", " << update.l() << "\n";

  fk(update);
}

void DiffDrive::fk(double time)
{
  fk(wheelspeeds * time);
}

void DiffDrive::fk(WheelDiff diff)
{
  // 1st, calculate the resultant twist from how far the wheels rotate
  // 2nd, transform the twist into the world frame
  // 3rd, integrate the twist in the world frame
  // 4th, chain the initial position transform and the integrated twist transform

  // Calculate these with *differences*
  diff_radian_file << diff.l() << ", " << diff.r() << "\n";

  wheels += diff;

  auto omega = radius / track * (diff.r() - diff.l());
  auto x = radius / 2.0 * (diff.r() + diff.l());
  auto y = 0.0;

  body_vel = Twist2D{omega, x, y};
  body_twist_file << body_vel << "\n";

  auto tf_current_to_new = integrate_twist(body_vel);
  q *= tf_current_to_new;

  twb_file << q.translation() << ", " << q.rotation() << "\n";
}

// TODO: 0228 Update wheel handling
WheelDiff DiffDrive::ik(Twist2D body_tw) const
{
  // Do not allow twists with a y component
  if (std::abs(body_tw.y) >= 0.00001) {
    throw std::logic_error("DiffDrive::ik: Requested body twist cannot have non-zero y component.");
  } else {  // u = H*V_b
    auto left = (body_tw.x - body_tw.omega * track / 2) / radius;
    auto right = (body_tw.x + body_tw.omega * track / 2) / radius;

    return WheelDiff(left, right).normalize();
  }
}

}  // namespace turtlelib
