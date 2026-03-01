#ifndef TURTLELIB_WHEELS_INCLUDE_GUARD_HPP
#define TURTLELIB_WHEELS_INCLUDE_GUARD_HPP

/// \file
/// \brief Representations of diff drive wheel positions, differences, and speeds
// Wheel positions and wheelspeeds/wheeldifferences are largely analogous to 2D cartesian points, except for angle wrapping behavior

namespace turtlelib
{
    /// \brief Represent DiffDrive wheel positions (analogous to a Point2D)
    class Wheels
    {
        private:
            /// \brief The left wheel position
            double left{0.0};
            /// \brief The right wheel position
            double right{0.0};

            /// \brief Wrap the angles to (-pi, pi]
            /// \returns A Wheel struct with wrapped angles
            Wheels normalize();

            /// \brief Update the wheels to a new position after a rotation, returning the shortest rotation between initial and final positions
            /// \param new_wheels The new position of the wheels after the rotation
            Wheels update(Wheels new_wheels);
            // {
            //   Wheels diff{left - new_wheels.left, right - new_wheels.right};

            //   left = new_wheels.left;
            //   right = new_wheels.right;

            //   return diff;
            // }
        public:
            /// \brief Create Wheels with both wheels at zero angle
            Wheels();

            /// \brief Create Wheels with both wheels at specific orientations
            /// \param left The left wheel position
            /// \param right The right wheel position
            explicit Wheels(double left_pos, double right_pos);

            // += diff
            // get
    };


    /// \brief Represent wheel rotation/displacement (analogous to a Vector2D). These will also be used to represent speeds. If a WheelDiff is created directly, it would be wise to confine them to [-pi, pi]
    class WheelDiff
    {
        private:
            /// \brief the left wheel difference
            double left{0.0};
            /// \brief the right wheel difference
            double right{0.0};

            /// \brief Wrap the angles to (-pi, pi]
            /// \returns A WheelDiff struct with wrapped angles


        public:
            /// \brief Create WheelDiff with zero rotation/difference/speed on both wheels
            WheelDiff();

            /// \brief Create WheelDiff with specific rotations on both wheels
            explicit WheelDiff(double left_rot, double right_rot);

        // Scale by scalar/time
        // get
        // add two differences


        /// \brief Scale a wheel difference (speed) by a scalar (time)
        /// \param rhs The scalar (time) to scale the diff (speed) by
        /// \return a reference to the scaled diff
        template <typename T>
        WheelDiff &operator*=(const T &rhs)
        {
            left *= rhs;
            right *= rhs;

            return *this;
        }

        // /// \brief Add a wheel diff to this wheel diff
        // /// \param rhs The wheel diff  to add to this wheel diff
        // /// \return A reference to the summed wheel diff
        // WheelDiff &operator+=(const WheelDiff &rhs);

        // /// \brief Subtract a wheel diff from this wheel diff
        // /// \param rhs The wheel diff to subtract from this wheel diff
        // /// \return A reference to the subtracted wheel diff
        // WheelDiff &operator-=(const WheelDiff &rhs);
    };

    /// \brief Get the minimum rotations between two sets of wheel positions
    /// \param final The final wheel positions
    /// \param initial The initial wheel positions
    /// \return The minimum rotations
    WheelDiff operator-(const Wheels &final, const Wheels &initial);

    Wheels Wheels::normalize()
    {
        return {};
    }

    Wheels Wheels::update(Wheels new_wheels)
    {
        Wheels diff{left - new_wheels.left, right - new_wheels.right};

        left = new_wheels.left;
        right = new_wheels.right;

        return diff;
    }

    WheelDiff operator-(const Wheels &final, const Wheels &initial)
    {
        WheelDiff raw_diff{final.left - initial.left, final.right - initial.right};
    }

    // WheelDiff &WheelDiff::operator-=(const WheelDiff &rhs)
    // {
    //   left = left-rhs.left;
    //   right = right + rhs.right;
    //   return *this;
    // }

    // WheelDiff &WheelDiff::operator+=(const WheelDiff &rhs)
    // {
    //   left += rhs.left;
    //   right += rhs.right;
    //   return *this;
    // }
}

#endif