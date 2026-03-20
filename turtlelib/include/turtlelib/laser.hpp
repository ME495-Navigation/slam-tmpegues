#ifndef TURTLELIB_LASER_INCLUDE_GUARD_HPP
#define TURTLELIB_LASER_INCLUDE_GUARD_HPP

/// \file
/// \brief Calculations for line-circle and line-line intersections for simulating LIDAR

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

#include "turtlelib/wheels.hpp"

namespace turtlelib
{
class Laser
{
private:
  Point2D near_point;
  Point2D far_point;

  double sgn(double value);

public:
  Laser(double min_range, double max_range);

  /// \brief Determine if a laser at this angle will hit a circular obstacle
  /// \param angle The angle of the laser relative to the laser/robot base
  /// \param T_rob_obs The potision of the obstacle relative to the laser/robot base
  /// \param radius The radius of the obstacle
  /// \returns std::pair<bool, double> If the laser segment does not hit the obstacle, bool = false. If the laser does hit the obstacle, bool = true and double = the distance from laser to intersection point.
  std::pair<bool, double> obs_check(double angle, Transform2D T_rob_obs, double radius);

  /// \brief Determine if a laser at this angle a linear obstacle, like a wall
  /// \param angle The angle of the laser relative to the laser/robot base
  /// \param T_w_rob The potision of the robot in the world frame
  /// \param std::pair<Point2D, Point2D> The endpoints of a linear obstacle
  /// \returns std::pair<bool, double> If the laser segment does not hit the obstacle, bool = false. If the laser does hit the obstacle, bool = true and double = the distance from laser to intersection point.
  std::pair<bool, double> line_check(double angle, Transform2D T_w_rob, std::pair<Point2D, Point2D> seg_ends);


};

}
#endif
