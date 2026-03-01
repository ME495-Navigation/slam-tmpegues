#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

#include <iostream>

#include "turtlelib/wheels.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("Shortest difference between two Wheels positions", "WheelDiff operator-(Wheels, Wheels)")
{
    Wheels pre {};
    Wheels post {};
    auto diff = post - pre;

    REQUIRE(diff.l() == 0.0);
    REQUIRE(diff.r() == 0.0);

    post = Wheels((5.0 / 4.0 * pi), -3.0 / 2.0 * pi);
    diff = post - pre;
    REQUIRE_THAT(diff.l(), WithinAbs(-3.0 / 4.0 * pi, 0.00001));
    REQUIRE_THAT(diff.r(), WithinAbs(1.0 / 2.0 * pi, 0.00001));
}
