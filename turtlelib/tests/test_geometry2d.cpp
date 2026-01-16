#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"

TEST_CASE("Subtract points (1, 1) and (0, 0) to make vector (1, 1)", "Vector2D operator-")
{
    turtlelib::Point2D origin {};
    origin.x = 0.0;
    origin.y = 0.0;

    turtlelib::Point2D pt_oo {};
    pt_oo.x = 1.0;
    pt_oo.y = 1.0;

    turtlelib::Vector2D vec_oo {};
    vec_oo.x = 1.0;
    vec_oo.y = 1.0;

    auto vec_res (pt_oo - origin);

    REQUIRE(vec_res.x == vec_oo.x);
    REQUIRE(vec_res.y == vec_oo.y);
}