#include "turtlelib/geometry2d.hpp"

#include <cmath>
#include <sstream>

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Point2D & p)
{
  if (
    is.peek() ==
    '(')       // if the first character is (, then assume it's properly formatted as (x, y)
  {
    is.get();  // purge the (
    is >> p.x;
    is.get();  // purge the ,
    is >> p.y;
    is.get();  // purge the )
  } else {     // If we can break it into 2 numbers, do it
    is >> p.x >> p.y;
  }
  return is;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  if (
    is.peek() ==
    '[')       // if the first character is [, then assume it's properly formatted as [x, y]
  {
    is.get();  // Purge [
    is >> v.x;
    is.get();  // Purge ,
    is >> v.y;
    is.get();  // Purge ]
  } else {     // If we can break it into 2 numbers, do it
    is >> v.x >> v.y;
  }
  return is;
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}
Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  // vector is head - tail
  return {head.x - tail.x, head.y - tail.y};
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vector2D operator+(Vector2D lhs, Vector2D & rhs) {return lhs += rhs;}


Vector2D operator-(Vector2D lhs, Vector2D & rhs) {return lhs -= rhs;}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  return {tail.x + disp.x, tail.y + disp.y};
}

double dot(const Vector2D & vect1, const Vector2D & vect2)
{
  return {vect1.x * vect2.x + vect1.y * vect2.y};
}

double magnitude(Vector2D & vect)
{
  return std::sqrt((std::pow(vect.x, 2) + std::pow(vect.y, 2)));
}

double angle(Vector2D & vect1, Vector2D & vect2)
{
  return std::atan2(vect1.x, vect1.y) - std::atan2(vect2.x, vect2.y);
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << ", " << v.y << "]";
  return os;
}

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

Vector2D normalize(Vector2D in) {return {in.x / magnitude(in), in.y / magnitude(in)};}
}  // namespace turtlelib
