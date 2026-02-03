#ifndef TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Representations of diff drive robot kinematics

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    class DiffDrive
    {
        private:
            Transform2D q;
            double phir = 0;
            double phil = 0;
            double track = 0;
            double radius = 0;

        public:
            /// \brief  Create a DiffDrive representation with given wheel track and wheel radius
            /// \param inputtrack The distance between the wheel contact points
            /// \param inputradius The radius of the wheels
            explicit DiffDrive(double inputtrack, double inputradius);


            /// \brief Update the state of the robot based on new wheel positions
            /// \param phir2 The new right wheel angle
            /// \param phil2 The new left wheel angle
            void fk(double phil2, double phir2);

            /// \brief Calculate wheel velocities needed to achive the provided twist
            /// \param tw The desired twist
            /// \return phirdot, phildot: the right and left wheel velocities
            auto ik(Twist2D tw);

    }
;
}

#endif