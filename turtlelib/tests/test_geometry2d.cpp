#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include <cmath>

// TEST_CASE("Create points and with format 'x y' and '(x, y)'", "std::istream & operator>>")
// {
//     std::print("test0");

//     std::stringstream stream {"1 1\n"};
//     turtlelib::Point2D pt1;
//     std::print("test1");
//     stream >> pt1;
//     std::print("test2");

//     REQUIRE(pt1.x == 0.0);
//     REQUIRE(pt1.y == 0.0);
// }

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
    auto vec_res_oo {pt_oo - origin};
    REQUIRE(vec_res_oo.x == 1.0);
    REQUIRE(vec_res_oo.y == 1.0);
    auto vec_res_minoo(origin - pt_oo);
    REQUIRE(vec_res_minoo.x == -1);
    REQUIRE(vec_res_minoo.y == -1);

    // 1 2 - 0 0 = 1 2
    auto vec_res_ot {pt_ot - origin};
    REQUIRE(vec_res_ot.x == 1.0);
    REQUIRE(vec_res_ot.y == 2);
    auto vec_res_minot {origin - pt_ot};
    REQUIRE(vec_res_minot.x == -1);
    REQUIRE(vec_res_minot.y == -2);

    // 1 2 - 1 1 = 0 -1
    auto vec_res_nno {pt_oo - pt_ot};
    REQUIRE(vec_res_nno.x == 0.0);
    REQUIRE(vec_res_nno.y == -1.0);
    auto vec_res_minnno {pt_ot - pt_oo};
    REQUIRE(vec_res_minnno.x == -0.0);
    REQUIRE(vec_res_minnno.y == 1.0);
}

TEST_CASE("Add a point and vector to make a new point", "Point2D operator+")
{
    turtlelib::Point2D origin{};
    origin.x = 0.0;
    origin.y = 0.0;

    turtlelib::Vector2D vec_oo{};
    vec_oo.x = 1.0;
    vec_oo.y = 1.0;

    turtlelib::Point2D pt_oo{};
    pt_oo.x = 1.0;
    pt_oo.y = 1.0;

    auto pt_res_oo {origin + vec_oo};
    REQUIRE(pt_res_oo.x == 1.0);
    REQUIRE(pt_res_oo.y == 1.0);

    auto pt_res_tt {pt_oo + vec_oo};
    REQUIRE(pt_res_tt.x == 2.0);
    REQUIRE(pt_res_tt.y == 2.0);

    turtlelib::Vector2D vec_ot{};
    vec_ot.x = 1.0;
    vec_ot.y = 2.0;
    auto pt_res_ot{origin + vec_ot};
    REQUIRE(pt_res_ot.x == 1.0);
    REQUIRE(pt_res_ot.y == 2.0);

    auto pt_res_tth{pt_oo + vec_ot};
    REQUIRE(pt_res_tth.x == 2.0);
    REQUIRE(pt_res_tth.y == 3.0);
}

TEST_CASE("Normalize vectors", "Vector2D normalize")
{
    turtlelib::Vector2D vec1;
    vec1.x = 1.0;
    vec1.y = 1.0;
    auto vec2 = turtlelib::normalize(vec1);

    REQUIRE(vec2.x == 1.0/std::sqrt(2.0));
    REQUIRE(vec2.y == 1.0/std::sqrt(2.0));

    vec1.x = -1.0;
    vec1.y = -1.0;
    vec2 = turtlelib::normalize(vec1);
    REQUIRE(vec2.x == -1.0 / std::sqrt(2.0));
    REQUIRE(vec2.y == -1.0 / std::sqrt(2.0));

    vec1.x = 3;
    vec1.y = 4;
    vec2 = turtlelib::normalize(vec1);
    REQUIRE(vec2.x == 3.0/5.0);
    REQUIRE(vec2.y == 4.0/5.0);
}