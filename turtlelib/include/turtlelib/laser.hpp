#ifndef TURTLELIB_LASER_INCLUDE_GUARD_HPP
#define TURTLELIB_LASER_INCLUDE_GUARD_HPP

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

#include "turtlelib/wheels.hpp"

namespace turtlelib
{
    class Laser
    {
    private:
        Point2D near_point;
        Point2D far_point;

    public:
        /// \brief Determine if a laser at this angle will hit the obstacle
        std::pair<bool, double> obs_check(double angle, Transform2D T_rob_obs, double radius);

        explicit Laser(double min_range, double max_range);
    };

}
#endif
