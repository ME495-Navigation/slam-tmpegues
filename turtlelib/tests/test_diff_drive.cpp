#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/diff_drive.hpp"
#include <cmath>

#include <iostream>

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("Test fk","DiffDrive::fk")
    {
        DiffDrive dd {1.0, 1/(pi*2) }; // Wheels have circumference 1

        // 1 full rotation should move 1 unit in x direction
        dd.fk(-2*pi, 2*pi);
        REQUIRE(dd.get_transform().translation().x == 1.0);

    }