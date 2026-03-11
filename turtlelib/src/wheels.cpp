#include <cmath>
#include <iostream>

#include "turtlelib/wheels.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{
// Creating Wheel and WheelDiff
// Default values of left and right are already 0.0 and 0.0
Wheels::Wheels() {}

Wheels::Wheels(double left_pos, double right_pos)
{
  left = normalize_angle(left_pos);
  right = normalize_angle(right_pos);
}

// Default values of left and right are already 0.0 and 0.0
WheelDiff::WheelDiff() {}

WheelDiff::WheelDiff(double left_rot, double right_rot)
{
  left = left_rot;
  right = right_rot;
}


// Basic and frequently used operations
double Wheels::l() {return left;}
double Wheels::r() {return right;}

double WheelDiff::l() {return left;}
double WheelDiff::r() {return right;}

Wheels & Wheels::normalize()
{
  left = normalize_angle(left);
  right = normalize_angle(right);

  return *this;
}

WheelDiff & WheelDiff::normalize()
{
  left = normalize_angle(left);
  right = normalize_angle(right);

  return *this;
}

WheelDiff operator-(Wheels & final, Wheels & initial)
{ // Why can't I const these?
  return WheelDiff(final.l() - initial.l(), final.r() - initial.r()).normalize();
}

Wheels & Wheels::operator+=(const WheelDiff & rotation)
{
  left = normalize_angle(left + rotation.left);
  right = normalize_angle(right + rotation.right);

  return *this;
}

Wheels operator+(Wheels position, WheelDiff & rotation)
{
  return position += rotation;
}

// Operations to be used in kinematics

WheelDiff Wheels::get_diff(Wheels new_wheels)
{
  auto diff = new_wheels - *this;

  return diff;
}
}
