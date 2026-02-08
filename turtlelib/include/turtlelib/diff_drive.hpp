#ifndef TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Representations of diff drive robot kinematics

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
/// \brief represent wheel positions
struct wheels
{  /// \brief the left wheel position
  double left{0.0};
  /// \brief the right wheel position
  double right = {0.0};
};
/// \brief represent wheel speeds

struct wheelspeed
{  /// \brief the left wheel speed
  double left{0.0};
  /// \brief the right wheel speed
  double right = {0.0};
};

class DiffDrive
{
private:
  Transform2D q;
  wheels phi;
  double track = 0;
  double radius = 0;

public:
  /// \brief Create a base DiffDrive
  DiffDrive();
  /// \brief  Create a DiffDrive representation with given wheel track and wheel radius
  /// \param input_track The distance between the wheel contact points
  /// \param input_radius The radius of the wheels
  explicit DiffDrive(double input_track, double input_radius);

  /// \brief Update the state of the robot based on new wheel positions
  /// \param phir2 The new right wheel angle
  /// \param phil2 The new left wheel angle
  /// \param time Time elapsed since the wheel angles were last collected
  wheelspeed fk(double phil2, double phir2, double time);

  /// \brief Calculate wheel velocities needed to achive the provided twist
  /// \param tw The desired twist
  /// \return phirdot, phildot: the right and left wheel velocities
  wheelspeed ik(Twist2D tw);

  /// \brief Get the world to body transform
  /// \return q, the world to body transform
  Transform2D get_transform();

  /// \brief Get wheel positions
  /// \return phir, phil: the left and right wheel angles
  wheels get_wheels();

  /// \brief Get track width
  /// \return Track width
  double get_track();

  /// \brief Get wheel radius
  /// \return Wheel Radius
  double get_radius();
};
}  // namespace turtlelib

#endif
