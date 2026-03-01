#ifndef TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Representations of diff drive robot kinematics

#include "turtlelib/se2d.hpp"
#include "turtlelib/wheels.hpp"

namespace turtlelib
{

class DiffDrive
{
private:
  Transform2D q;
  Wheels wheels;
  WheelDiff wheelspeeds;
  Twist2D body_vel;
  double track = 0;
  double radius = 0;

public:
  /// \brief Create a base DiffDrive
  DiffDrive();

  /// \brief  Create a DiffDrive representation with given wheel track and wheel radius
  /// \param input_track The distance between the wheel contact points
  /// \param input_radius The radius of the wheels
  explicit DiffDrive(double input_track, double input_radius);

  /// \brief  Create a DiffDrive representation with given wheel track and wheel radius, and a non-identity Transform2D
  /// \param input_track The distance between the wheel contact points
  /// \param input_radius The radius of the wheels
  /// \param tf The initial configuration of the turtlebot
  explicit DiffDrive(double input_track, double input_radius, Transform2D tf);

  /// \brief Update the state of the robot based on new wheel positions
  /// \param wheels2 The new wheel positions
  void fk(Wheels wheels2);

  // TODO: 0228 Update wheel handling
  /// \brief Calculate wheel velocities needed to achive the provided twist
  /// \param tw The desired twist
  /// \return phirdot, phildot: the right and left wheel velocities
  WheelDiff ik(Twist2D tw);

  /// \brief Get the world to body transform
  /// \return q, the world to body transform
  Transform2D get_transform();

  /// \brief Get wheel positions (phi)
  /// \return The current wheel positions
  Wheels phi();

  // TODO: 0228 Update wheel handling
  /// \brief Get wheel speeds
  /// \return a WheelDiff representing the wheelspeeds in rad/sec
  WheelDiff phidot();

  /// \brief Get track width
  /// \return Track width
  double get_track();

  /// \brief Get wheel radius
  /// \return Wheel Radius
  double get_radius();

  /// \brief Get current body twist
  /// \return Current body twist
  Twist2D get_twist();
};
}  // namespace turtlelib

#endif
