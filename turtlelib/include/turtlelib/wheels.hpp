#ifndef TURTLELIB_WHEELS_INCLUDE_GUARD_HPP
#define TURTLELIB_WHEELS_INCLUDE_GUARD_HPP

/// \file
/// \brief Representations of diff drive wheel positions, differences, and speeds
// Wheel positions and wheelspeeds/wheeldifferences are largely analogous to 2D cartesian points, except for angle wrapping behavior

namespace turtlelib
{

/// \brief Represent wheel rotation/displacement (analogous to a Vector2D). They will be normalized.
class WheelDiff
{
private:
    /// \brief the left wheel difference
  double left{0.0};
    /// \brief the right wheel difference
  double right{0.0};

public:
    /// \brief Create WheelDiff with zero rotation/difference/speed on both wheels
  WheelDiff();

  /// \brief Wrap the angles to (-pi, pi]
  /// \returns A WheelDiff struct with wrapped angles
  WheelDiff & normalize();

  /// \brief Create WheelDiff with specific rotations on both wheels
  /// \param left_rot The rotation of the left wheel
  /// \param right_rot The rotation of the right wheel
  explicit WheelDiff(double left_rot, double right_rot);

    // Scale by scalar/time
    // get
    /// \brief Get the left wheel rotation
    /// \return The left wheel rotation
  double l();

    /// \brief Get the right wheel rotation
    /// \return The right wheel rotation
  double r();

    /// \brief Scale a wheel difference (speed) by a scalar (time)
    /// \param rhs The scalar (time) to scale the diff (speed) by
    /// \return a reference to the scaled diff
  template<typename T>
  WheelDiff & operator*=(const T & rhs)
  {
    left *= rhs;
    right *= rhs;

    return *this;
  }

  friend class Wheels;

};

/// \brief Represent DiffDrive wheel positions/angles (analogous to a Point2D)
class Wheels
{
private:
    /// \brief The left wheel position
  double left {0.0};
    /// \brief The right wheel position
  double right {0.0};

    /// \brief Wrap wheel angles to (-pi, pi] and store the result in this object
    /// \returns A reference to the Wheels object with wrapped angles
  Wheels & normalize();

public:
    /// \brief Create Wheels with both wheels at zero angle
  Wheels();

    /// \brief Create Wheels with both wheels at specific orientations.
    /// \param left_pos The left wheel position
    /// \param right_pos The right wheel position
  explicit Wheels(double left_pos, double right_pos);

    /// \brief Get the left wheel position
    /// \return The left wheel position
  double l();

    /// \brief Get the right wheel position
    /// \return The right wheel position
  double r();

    /// \brief Update the wheels to a new position, returning the shortest rotation between initial and final positions
    /// \param new_wheels The new positions
    /// \return The rotations required to get to the new positions
  WheelDiff update(Wheels new_wheels);

    // += diff
    // get
};


/// \brief Scale a wheel difference (speed) by a scalar (time)
/// \param scalar The scalar (time) to scale the diff (speed) by
/// \param diff The difference to be scaled
/// \return the scaled difference
template<typename T>
WheelDiff operator*(const T & scalar, WheelDiff diff)
{
  return diff *= scalar;
}

/// \brief Scale a wheel difference (speed) by a scalar (time)
/// \param scalar The scalar (time) to scale the diff (speed) by
/// \param diff The difference to be scaled
/// \return the scaled difference
template<typename T>
WheelDiff operator*(WheelDiff diff, const T & scalar)
{
  return diff *= scalar;
}

/// \brief Get the minimum rotations between two sets of wheel positions
/// \param final The final wheel positions
/// \param initial The initial wheel positions
/// \return The minimum rotations
WheelDiff operator-(Wheels & final, Wheels & initial);

/// \brief Add rotations to a wheel positions to get the new positions
/// \param pos The initial wheel positions
/// \param rot How far the wheels rotate
/// \return The new positions
Wheels operator+(Wheels & position, WheelDiff & rotation);
}

#endif
