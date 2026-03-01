#include <cmath>
#include <stdexcept>
#include <iostream>

#include "turtlelib/angle.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

Transform2D DiffDrive::get_transform() {return q;}

Wheels DiffDrive::phi() {return wheels;}

WheelDiff DiffDrive::phidot() {return wheelspeeds;}

double DiffDrive::get_track() {return track;}

double DiffDrive::get_radius() {return radius;}

Twist2D DiffDrive::get_twist() {return body_vel;}

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


void DiffDrive::fk(Wheels wheels2)
{
  // The arguments received here are actual wheel positions, we don't need to scale by time

  // 0st, update wheel positions
  // 1st, calculate the resultant twist
  // 2nd, transform the twist into the world frame
  // 3rd, integrate the twist in the world frame
  // 4th, chain the initial position transform and the integrated twist transform

  auto diff {wheels.update(wheels2)};

  std::cout << "Pre FK update: " << q.translation() << "\n";

  // Calculate these with *differences*
  auto omega = radius / 2.0 * (diff.l() - diff.r());
  auto x = radius / 2.0 * (diff.l() + diff.r());
  auto y = 0.0;

  body_vel = Twist2D{omega, x, y};

  auto world_twist = q(body_vel);
  auto tf_current_to_new = integrate_twist(world_twist);
  q *= tf_current_to_new;
  std::cout << "Post FK update: " << q.translation() << "\n";
}

// TODO: 0228 Update wheel handling
WheelDiff DiffDrive::ik(Twist2D body_tw)
{
  // Do not allow twists with a y component
  if (std::abs(body_tw.y) >= 0.00001) {
    throw std::logic_error("DiffDrive::ik: Requested body twist cannot have non-zero y component.");
  } else {  // u = H*V_b
    auto left = (body_tw.x - body_tw.omega * track / 2) / radius;
    auto right = (body_tw.x + body_tw.omega * track / 2) / radius;

    return WheelDiff(left, right);
  }
}

}  // namespace turtlelib
