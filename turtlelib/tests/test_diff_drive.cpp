#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <stdexcept>

#include <iostream>

#include "turtlelib/angle.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("Test fk","DiffDrive::fk")
{
        DiffDrive dd {2.0, 1.0 }; // Wheels have circumference 2*pi

        // 0 rotation on both wheels should result in no change
        dd.fk(0, 0, 1);
        REQUIRE(dd.get_transform().translation().x == 0);
        REQUIRE(dd.get_transform().translation().y == 0);
        REQUIRE(dd.get_transform().rotation() == 0);
        REQUIRE(dd.get_wheels().left == 0);
        REQUIRE(dd.get_wheels().right == 0);

        // 1 quarter rotation should move pi/2 in x direction
        dd.fk(pi/2, pi/2, 1);
        REQUIRE(dd.get_transform().translation().x == pi/2);
        REQUIRE(dd.get_transform().translation().y == 0);
        REQUIRE(dd.get_transform().rotation() == 0);
        REQUIRE(dd.get_wheels().left == pi/2);
        REQUIRE(dd.get_wheels().right == pi/2);

        // Returning to the previous position should return state to original
        dd.fk(0, 0, 1);
        REQUIRE(dd.get_transform().translation().x == 0.0);
        REQUIRE(dd.get_transform().translation().y == 0.0);
        REQUIRE(dd.get_transform().rotation() == 0.0);
        REQUIRE(dd.get_wheels().left == 0.0);
        REQUIRE(dd.get_wheels().right == 0.0);

        // Wheels rotation equal distances in opposite direction should spin with no translation
        auto rot = pi/2;
        dd.fk(-rot, rot, 1);
        REQUIRE(dd.get_transform().translation().x == 0.0);
        REQUIRE(dd.get_transform().translation().y == 0.0);
        REQUIRE(dd.get_transform().rotation() == rot);
        REQUIRE(dd.get_wheels().left == -rot);
        REQUIRE(dd.get_wheels().right == rot);

        // Back it up
        rot = 0.0;
        dd.fk(-rot, rot, 1);
        REQUIRE(dd.get_transform().translation().x == 0.0);
        REQUIRE(dd.get_transform().translation().y == 0.0);
        REQUIRE(dd.get_transform().rotation() == 0.0);
        REQUIRE(dd.get_wheels().left == 0.0);
        REQUIRE(dd.get_wheels().right == 0.0);

        // Rotate only one wheel should result in an x, y, and theta change (multiple iterations so I get to a location with easy numbers)
        DiffDrive dd2 {2.0, 1.0 }; // Wheels have circumference 2*pi
        dd2.fk(0, dd2.get_wheels().right+pi/2, 1);
        dd2.fk(0, dd2.get_wheels().right+pi/2, 1);
        dd2.fk(0, dd2.get_wheels().right+pi/2, 1);
        dd2.fk(0, dd2.get_wheels().right+pi/2, 1);

        REQUIRE(dd2.get_wheels().left == 0);
        REQUIRE(dd2.get_wheels().right == normalize_angle(2*pi));
        REQUIRE_THAT(dd2.get_transform().translation().x, WithinAbs(0, 0.00001));
        REQUIRE_THAT(dd2.get_transform().translation().y, WithinAbs(2, 0.00001));
        REQUIRE_THAT(dd2.get_transform().rotation(), WithinAbs(normalize_angle(pi), 0.00001));
}

TEST_CASE("Test ik","DiffDrive::ik")
{

    DiffDrive dd {2.0, 1.0};
    Twist2D body_tw {0.0, 1, 0.0};

    auto speeds {dd.ik(body_tw)}; // Pure translation +
    REQUIRE(speeds.left == 1);
    REQUIRE(speeds.right == 1);

    speeds = dd.ik(-1*body_tw); // Pure translation -
    REQUIRE(speeds.left == -1);
    REQUIRE(speeds.right == -1);

    Twist2D body_tw2 {1.0, 0.0, 0.0}; // Pure rotation  +
    speeds = dd.ik(body_tw2);
    REQUIRE(speeds.left == -1);
    REQUIRE(speeds.right == 1);

    speeds = dd.ik(-1*body_tw2); // Pure rotation -
    REQUIRE(speeds.left == 1);
    REQUIRE(speeds.right == -1);

    Twist2D body_tw3 {pi/2, pi/2, 0.0};
    speeds = dd.ik(body_tw3);
    REQUIRE(speeds.left == 0);
    REQUIRE(speeds.right == pi);

    Twist2D body_tw4 {-pi/2, -pi/2, 0.0};
    speeds = dd.ik(body_tw4);
    REQUIRE(speeds.left == 0);
    REQUIRE(speeds.right == -pi);

    Twist2D body_tw5 {1, 1, 1};
    speeds = dd.ik(body_tw5);
    REQUIRE_THROWS_AS(dd.ik(body_tw5), std::logic_error);
}