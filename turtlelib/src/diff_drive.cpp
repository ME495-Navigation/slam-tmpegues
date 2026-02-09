#include "turtlelib/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

#include "turtlelib/angle.hpp"

namespace turtlelib
{

DiffDrive::DiffDrive(double input_track, double input_radius)
{
  track = input_track;
  radius = input_radius;
}

// Update both the wheel positions and wheelspeeds by calculating the smallest path between
// the current wheel angles and the newly provided ones
void DiffDrive::update_wheels(wheels new_wheels, double time)
{
  // Update new_wheels in place so it's easier to update wheels and wheelspeeds
  new_wheels.left = normalize_angle(new_wheels.left);
  new_wheels.right = normalize_angle(new_wheels.right);

  // What's the shortest path for each wheel?
  auto left_diff = shortest_angle_diff(new_wheels.left, phi.left);
  auto right_diff = shortest_angle_diff(new_wheels.right, phi.right);

  phi.update(new_wheels);

  phidot = wheelspeed{left_diff / time, right_diff / time};
}

void DiffDrive::fk(double phil2, double phir2, double time)
{
  // 0st, update wheel positions
  // 1st, calculate the resultant twist
  // 2nd, transform the twist into the world frame
  // 3rd, integrate the twist in the world frame
  // 4th, chain the initial position transform and the integrated twist transform

  // phidot = wheelspeed{(phil2 - phi.left) / time, (phir2 - phi.right) / time};

  // phi.left = normalize_angle(phil2);
  // phi.right = normalize_angle(phir2);

  update_wheels(wheels{phil2, phir2}, time);


  auto omega = radius / 2.0 * (2.0 * (phidot.right - phidot.left) / 2.0);
  auto x = radius / 2.0 * (phidot.right + phidot.left);
  auto y = 0.0;

  body_vel = Twist2D{omega, x, y};

  auto world_twist = q(time * body_vel);
  auto tf_current_to_new = integrate_twist(world_twist);
  q *= tf_current_to_new;


}

wheelspeed DiffDrive::ik(Twist2D body_tw)
{
  // Do not allow twists with a y component
  if (std::abs(body_tw.y) >= 0.00001) {
    throw std::logic_error("DiffDrive::ik: Requested body twist cannot have non-zero y component.");
  } else {  // u = H*V_b
    auto left = 1 / radius * (body_tw.x - body_tw.omega * track / 2);
    auto right = 1 / radius * (body_tw.x + body_tw.omega * track / 2);

    return {left, right};
  }
}


Transform2D DiffDrive::get_transform() {return q;}

wheels DiffDrive::get_wheels() {return phi;}

wheelspeed DiffDrive::get_wheelspeed()
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
