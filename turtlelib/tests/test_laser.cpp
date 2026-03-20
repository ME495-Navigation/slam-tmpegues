#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>

#include "turtlelib/laser.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("Check a basic laser-obstacle hit and a miss", "Laser::obs_check")
{
    auto laser = Laser(.1, 100.0);
    Transform2D T_rob_obs {Vector2D(10.0, 1.0)};

    auto check_result = laser.obs_check(0.0, T_rob_obs, 1);
    REQUIRE(check_result.first == true);
    REQUIRE(check_result.second == 10.0);

    check_result = laser.obs_check(0.0, T_rob_obs, .5);
    REQUIRE(check_result.first == false);
}

TEST_CASE("Check laser-wall hit and miss", "Laser::line_check")
{
    auto laser = Laser(.1, 100.0);
    auto check_result = laser.line_check(0, Transform2D(), std::make_pair(Point2D(1,-1), Point2D(1, 1)));
    REQUIRE(check_result.first == true);
    REQUIRE(check_result.second == 1);

    check_result = laser.line_check(0, Transform2D(), std::make_pair(Point2D(1.0, -1.0), Point2D(1.0, -.25)));
    REQUIRE(check_result.first == false);

    check_result = laser.line_check(std::numbers::pi / 4, Transform2D(), std::make_pair(Point2D(1.0, -100.0), Point2D(1.0, 100.0)));
    REQUIRE(check_result.first == true);
    REQUIRE(almost_equal(check_result.second, sqrt(2)));
}
