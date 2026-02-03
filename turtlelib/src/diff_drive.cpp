#include <cmath>

#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive(double inputtrack, double inputradius)
    {
        track = inputtrack;
        radius = inputradius;
    }

    void DiffDrive::fk(double phil2, double phir2)
    {
        // 1st, calculate the resultant twist
        // 2nd, transform the twist into the world frame
        // 3rd, integrate the twist in the body frame
        // 4th, chain the initial position transform and the integrated twist transform
        Twist2D body_twist {};
        body_twist.omega = track/2 * (phil2 - phir2);
        body_twist.x = phir2+phil2 ;
        body_twist.y = 0.0;

        auto world_twist = q(body_twist);

        auto oldq_to_newq = integrate_twist(world_twist);

        q *= oldq_to_newq;


    }

    auto DiffDrive::ik(Twist2D tw)
    {  // TODO: Add citation 2
        struct phidot
        {
            double left {0};
            double right {0};
        } phidot;


        return phidot;
    }
}