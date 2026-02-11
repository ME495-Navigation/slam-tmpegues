#include "turtlelib/se2d.hpp"

#include <cmath>
#include <sstream>

#include "turtlelib/angle.hpp"

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Twist2D & tw)
{
  // If first character is <, get rid of it
  if (is.peek() == '<') {
    is.get();
  }

  // Then read first number
  is >> tw.omega;
  is.get();  // Purge the space

  // Check if next character is r or d
  if (is.peek() == 'r') { // remove the string, but do nothing else. Keep all units in rad/sec
    std::string str;
    is >> str;
  } else if (is.peek() == 'd') { // remove the string, then convert deg/s to rad/s
    std::string str;
    is >> str;
    tw.omega = deg2rad(tw.omega);
  }

  // Check for commas between every other character
  if (is.peek() == ',') {
    is.get();
  }

  is >> tw.x;

  if (is.peek() == ',') {
    is.get();
  }

  is >> tw.y;

  if (is.peek() == '>') {
    is.get();
  }
  return is;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
{
  os << "<" << tw.omega << ", " << tw.x << ", " << tw.y << '>';
  return os;
}

Transform2D::Transform2D()
{
     // Default values for pos, theta, and costh and sinth are for an identity transform
}

Transform2D::Transform2D(Vector2D trans)
{
  pos = trans;
  // Default values for tw.omega, theta, costh, sinth are correct for no rotation
}

Transform2D::Transform2D(double radians)
{
  // Default values for tw.x and tw.y are correct for no translation
  update_theta(radians);
}

Transform2D::Transform2D(Vector2D trans, double radians)
{
  update_theta(radians);
  pos = trans;
}

void Transform2D::update_theta(double new_theta)
{
  theta = normalize_angle(new_theta);
  sinth = std::sin(theta);
  costh = std::cos(theta);
}

Point2D Transform2D::operator()(Point2D p) const
{
  // Rotate then translate the point
  return {(costh * p.x - sinth * p.y) + pos.x, (costh * p.y + sinth * p.x) + pos.y};
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  // Transforming a vector only rotates it
  return {costh * v.x - sinth * v.y, costh * v.y + sinth * v.x};
}

Twist2D Transform2D::operator()(Twist2D v) const
{
  return {
    v.omega, costh * v.x - sinth * v.y + pos.y * v.omega,
    costh * v.y + sinth * v.x - pos.x * v.omega};
}

Transform2D Transform2D::inv() const
{
  Vector2D new_vc;
  new_vc.x = -pos.x * costh - pos.y * sinth;
  new_vc.y = -pos.y * costh + pos.x * sinth;

  return {new_vc, -theta};
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  pos.x += costh * rhs.pos.x - sinth * rhs.pos.y;
  pos.y += costh * rhs.pos.y + sinth * rhs.pos.x;

  update_theta(theta + rhs.theta);

  return *this;
}

Vector2D Transform2D::translation() const {return pos;}

double Transform2D::rotation() const {return theta;}

Transform2D integrate_twist(const Twist2D & twist)
{  // TODO: Cite Theo
  if (std::abs(twist.omega) < 1e-9) {
    return Transform2D(Vector2D{twist.x, twist.y}, 0.0);
  }

  auto sinom{std::sin(twist.omega)};
  auto cosom{std::cos(twist.omega)};

  auto x = (twist.x * sinom + twist.y * (1.0 - cosom)) / twist.omega;
  auto y = (-twist.y * sinom + twist.x * (1.0 - cosom)) / twist.omega;

  Transform2D tf(Vector2D{x, y}, twist.omega);
  return tf;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  auto x{0.0};
  auto y{0.0};
  auto theta{0.0};

  // If first character is '{', get rid of it and the following '<'
  if (is.peek() == '{') {
    is.get();  // purge {
    is.get();  // purge <
  }

  double angle{0.0};
  is >> angle;

  if (is.peek() == '>') {
    is.get();  // purge >
    if (is.peek() == ' ') {
      is.get();  // Purge the ' ' space if we will be getting a unit.
                 // If we're not getting a unit, then the peek would be a comma.
    }
  }
  switch (is.peek()) { // either we get a '<', indicating that we're being given a unit,
                       // or a ',' or  ' ', or a digit indicating we aren't.
                       // Anything else is a problem.
    case ' ':  // this means that unit was not given and we can assume radians
    case ',':  // this means that unit was not given and we can assume radians
      {
        theta = angle;
        break;
      }
    case '<':  // this means we need to check the unit
      {
        is.get();            // Purge the <
        if (is.peek() == 'r') { // We have radians again, but we need to purge the string
          theta = angle;
          std::string str;
          is >> str;
          is.get();                 // Purge the ' ' space
        } else if (is.peek() == 'd') { // Convert from deg to rad, then purge string
          theta = deg2rad(angle);
          std::string str;
          is >> str;
          is.get(); // Purge the ' ' space
        }
        break;
      }
  }
  if (is.peek() == ',') {
    is.get();  // purge ,
    is.get();  // purge ' ' space
  }

  if (is.peek() == '<') {
    is.get();  // purge <
  }

  is >> x;

  if (is.peek() == '>') {
    is.get();  // purge >
    is.get();  // purge ,
    is.get();  // purge ' ' space
  }

  if (is.peek() == '<') {
    is.get();  // purge <
  }

  is >> y;

  if (is.peek() == '>') {
    is.get();  // purge >
    is.get();  // purge }
  }
  tf = Transform2D(Vector2D{x, y}, theta);
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  lhs *= rhs;
  return lhs;
}
}  // namespace turtlelib
