#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>

#include "turtlelib/laser.hpp"
#include "turtlelib/se2d.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("Check a basic laser-obstacle hit and a miss", "laser_obs")
{
    auto laser = Laser(.1, 100.0);
    Transform2D T_rob_obs {Vector2D(10.0, 1.0)};

    auto check_result = laser.obs_check(0.0, T_rob_obs, 1);
    REQUIRE(check_result.first == true);
    REQUIRE(check_result.second == 10.0);
}