#include <cmath>
#include <stdexcept>
#include <iostream>

#include "turtlelib/angle.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

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

// TODO: 0228 Update wheel handling
// Update both the wheel positions and wheelspeeds by calculating the smallest path between
// the current wheel angles and the newly provided ones
void DiffDrive::update_wheels(Wheels new_wheels, double time)
{
  // Update new_wheels in place so it's easier to update wheels and wheelspeeds
  new_wheels.left = normalize_angle(new_wheels.left);
  new_wheels.right = normalize_angle(new_wheels.right);

  // What's the shortest path for each wheel?
  auto left_diff = shortest_angle_diff(new_wheels.left, phi.left);
  auto right_diff = shortest_angle_diff(new_wheels.right, phi.right);

  phi.update(new_wheels);

  phidot = WheelDiff{left_diff / time, right_diff / time};
}

void DiffDrive::fk(double phil2, double phir2, double time)
{
  // The arguments received here are actual wheel positions, we don't need to scale by time

  // 0st, update wheel positions
  // 1st, calculate the resultant twist
  // 2nd, transform the twist into the world frame
  // 3rd, integrate the twist in the world frame
  // 4th, chain the initial position transform and the integrated twist transform

  phi.update(Wheels{phil2, phir2});

  std::cout << "Pre FK update: " << q.translation() << "\n";

  // Calculate these with *differences*
  auto omega = radius / 2.0 * (phil2 - phir2);
  auto x = radius / 2.0 * (phir2 + phil2);
  auto y = 0.0;

  body_vel = Twist2D{omega, x, y};

  auto world_twist = q(time * body_vel);
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

    return {left, right};
  }
}


Transform2D DiffDrive::get_transform() {return q;}

Wheels DiffDrive::get_wheels() {return phi;}

// TODO: 0228 Update wheel handling

WheelDiff DiffDrive::get_wheelspeed()
{
  return phidot;
}

double DiffDrive::get_track() {return track;}

double DiffDrive::get_radius() {return radius;}

Twist2D DiffDrive::get_twist()
{
  return body_vel;
}
}  // namespace turtlelib
