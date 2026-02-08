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

  void update(wheels new_wheels)
  {
      left = new_wheels.left;
      right = new_wheels.right;
  }

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
  wheelspeed phidot;
  Twist2D vel;
  double track = 0;
  double radius = 0;

  // Update both the wheel positions and wheelspeeds by calculating the smallest path between
  // the current wheel angles and the newly provided ones
  void update_wheels(wheels new_wheels, double time);

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
  void fk(double phil2, double phir2, double time);

  /// \brief Calculate wheel velocities needed to achive the provided twist
  /// \param tw The desired twist
  /// \return phirdot, phildot: the right and left wheel velocities
  wheelspeed ik(Twist2D tw);

  /// \brief Get the world to body transform
  /// \return q, the world to body transform
  Transform2D get_transform();

  /// \brief Get wheel positions
  /// \return phi, where phi.left and phi.right the left and right wheel angles in radians
  wheels get_wheels();

  /// \brief Get wheel speeds
  /// \return phidot, where phidot.left and phidot.right the left and right wheel speeds in rad/sec
  wheelspeed get_wheelspeed();

  /// \brief Get track width
  /// \return Track width
  double get_track();

  /// \brief Get wheel radius
  /// \return Wheel Radius
  double get_radius();
};
}  // namespace turtlelib

#endif
