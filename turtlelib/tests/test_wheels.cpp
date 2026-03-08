#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

#include <iostream>

#include "turtlelib/wheels.hpp"
#include "turtlelib/angle.hpp"
#include "turtlelib/geometry2d.hpp"

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

TEST_CASE("Add rotations to wheel positions", "Wheels operator+(Wheels, WheelDiff)")
{
    // Zero rotation should not change the wheel positions
    Wheels pre{1.0, 3.78};
    WheelDiff rot{};

    auto new_wheels = pre + rot;
    REQUIRE(new_wheels.l() == pre.l());
    REQUIRE(new_wheels.r() == pre.r());

    rot = WheelDiff{2.0, pi};

    new_wheels = pre + rot;
    REQUIRE(new_wheels.l() == 3.0);
    REQUIRE_THAT(new_wheels.r(), WithinAbs(normalize_angle(3.78 + pi), 0.00001));

    pre += rot;
    REQUIRE(pre.l() == 3.0);
    REQUIRE_THAT(pre.r(), WithinAbs(normalize_angle(3.78 + pi), 0.00001));
}


TEST_CASE("Update wheel positions to new potision and calculate the required rotations",
  "WheelDiff Wheels::update(Wheels)")
{
    Wheels initial {};
    Wheels end {pi / 2, 3.0 / 2.0 * pi};

    auto diff {initial.update(end)};
    REQUIRE_THAT(diff.l(), WithinAbs(pi / 2.0, 0.00001));
    REQUIRE_THAT(diff.r(), WithinAbs(-pi / 2.0, 0.00001));

    REQUIRE_THAT(initial.l(), WithinAbs(pi / 2.0, 0.00001));
    REQUIRE_THAT(initial.r(), WithinAbs(-pi / 2.0, 0.00001));
}
