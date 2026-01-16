#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"

TEST_CASE("Subtract points to make vectors", "Vector2D operator-")
{
    turtlelib::Point2D origin {};
    origin.x = 0.0;
    origin.y = 0.0;

    turtlelib::Point2D pt_oo {};
    pt_oo.x = 1.0;
    pt_oo.y = 1.0;

    turtlelib::Point2D pt_ot {};
    pt_ot.x = 1.0;
    pt_ot.y = 2.0;

    // 1 1 - 0 0 = 1 1
    turtlelib::Vector2D vec_oo {};
    vec_oo.x = 1.0;
    vec_oo.y = 1.0;
    auto vec_res_oo (pt_oo - origin);
    REQUIRE(vec_res_oo.x == vec_oo.x);
    REQUIRE(vec_res_oo.y == vec_oo.y);

    // 1 2 - 0 0 = 1 2
    turtlelib::Vector2D vec_ot{};
    vec_ot.x = 1.0;
    vec_ot.y = 2.0;
    auto vec_res_ot(pt_ot - origin);
    REQUIRE(vec_res_ot.x == vec_ot.x);
    REQUIRE(vec_res_ot.y == vec_ot.y);

    // 1 2 - 1 1 = 0 -1
    turtlelib::Vector2D vec_nno{};
    vec_nno.x = 0.0;
    vec_nno.y = -1.0;
    auto vec_res_nno(pt_oo - pt_ot);
    REQUIRE(vec_res_nno.x == vec_nno.x);
    REQUIRE(vec_res_nno.y == vec_nno.y);
}