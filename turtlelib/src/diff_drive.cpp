#include <cmath>
#include <stdexcept>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{

    DiffDrive::DiffDrive(double input_track, double input_radius)
    {
        track = input_track;
        radius = input_radius;
    }

    wheelspeed DiffDrive::fk(double phil2, double phir2, double time)
    {
        // 0st, update wheel positions
        // 1st, calculate the resultant twist
        // 2nd, transform the twist into the world frame
        // 3rd, integrate the twist in the world frame
        // 4th, chain the initial position transform and the integrated twist transform

        wheelspeed phidot { (phil2 - phi.left) / time, (phir2 - phi.right) / time };
        // auto phidotl = (phil2 - phi.left)/time;
        // auto phidotr = (phir2 - phi.right)/time;

        phi.left = normalize_angle(phil2);
        phi.right = normalize_angle(phir2);

        auto omega = radius / 2.0 * (2.0 * (phidot.right - phidot.left) / 2.0);
        auto x = radius / 2.0 * (phidot.right + phidot.left);
        auto y = 0.0;

        Twist2D body_twist{omega, x, y};

        auto world_twist = q(time*body_twist);
        auto tf_current_to_new = integrate_twist(world_twist);
        q *= tf_current_to_new;

        return phidot;
    }

    wheelspeed DiffDrive::ik(Twist2D body_tw)
    {
        // Do not allow twists with a y component
        if (std::abs(body_tw.y) >= 0.00001)
        {
            throw std::logic_error("DiffDrive::ik: Requested body twist cannot have non-zero y component.");
        }
        else
        {   // u = H*V_b
            auto left = 1/radius * (body_tw.x-body_tw.omega*track/2);
            auto right = 1/radius * (body_tw.x+body_tw.omega*track/2);

            return {left, right};
        }
    }

    Transform2D DiffDrive::get_transform()
    {
        return q;
    }

    wheels DiffDrive::get_wheels()
    {
        return phi;
    }

    double DiffDrive::get_track()
    {
        return track;
    }

    double DiffDrive::get_radius()
    {
        return radius;
    }
}