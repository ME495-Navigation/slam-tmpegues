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
  Wheels phi;
  WheelDiff phidot;
  Twist2D body_vel;
  double track = 0;
  double radius = 0;

  // Update both the phi and phidot by calculating the smallest path betweenthe current wheel angles and the newly provided ones
  void update_wheels(Wheels new_wheels, double time);

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
  /// \param phir2 The new right wheel angle
  /// \param phil2 The new left wheel angle
  /// \param time Time elapsed since the wheel angles were last collected
  void fk(double phil2, double phir2, double time);

  // TODO: 0228 Update wheel handling
  /// \brief Calculate wheel velocities needed to achive the provided twist
  /// \param tw The desired twist
  /// \return phirdot, phildot: the right and left wheel velocities
  WheelDiff ik(Twist2D tw);

  /// \brief Get the world to body transform
  /// \return q, the world to body transform
  Transform2D get_transform();

  /// \brief Get wheel positions (phi)
  /// \return phi, where phi.left and phi.right the left and right wheel angles in radians
  Wheels get_wheels();

  // TODO: 0228 Update wheel handling
  /// \brief Get wheel speeds (phidot)
  /// \return phidot, where phidot.left and phidot.right the left and right wheel speeds in rad/sec
  WheelDiff get_wheelspeed();

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
