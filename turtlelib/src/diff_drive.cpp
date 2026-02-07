#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{

    DiffDrive::DiffDrive(double input_track, double input_radius)
    {
        track = input_track;
        radius = input_radius;
    }

    void DiffDrive::fk(double phil2, double phir2, double time)
    {
        // 0st, update wheel positions
        // 1st, calculate the resultant twist
        // 2nd, transform the twist into the world frame
        // 3rd, integrate the twist in the body frame
        // 4th, chain the initial position transform and the integrated twist transform

        auto phidotl = (phil2 - phi.left)/time;
        auto phidotr = (phir2 - phi.right)/time;
        phi.left = normalize_angle(phil2);
        phi.right = normalize_angle(phir2);

        Twist2D body_twist {};
        body_twist.omega = radius/2 * (2*(phidotr-phidotl)/2);
        body_twist.x = radius/2 * (phidotr+phidotl);
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

    Transform2D DiffDrive::get_transform()
        {return q;}

    wheels DiffDrive::get_wheels()
        {
            return phi;
        }
}