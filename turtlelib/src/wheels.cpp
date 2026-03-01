#include <cmath>

#include "turtlelib/wheels.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib
{
    // Creating Wheel and WheelDiff
    Wheels::Wheels()
    {
        ;   // Default values of left and right are already 0.0 and 0.0
    }

    Wheels::Wheels(double left_pos, double right_pos)
    {
        left = left_pos;
        right = right_pos;
    }


    WheelDiff::WheelDiff()
    {
        ;   // Default values of left and right are already 0.0 and 0.0
    }

    WheelDiff::WheelDiff(double left_rot, double right_rot)
    {
        left = left_rot;
        right = right_rot;
    }



}