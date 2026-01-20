#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"
#include <cmath>

#include <print>

TEST_CASE("Create a twist with format 'w x y' and 'x <unit> x y' and <w x y>, with radians and degrees", "std::istream & operator>>")
{
    std::stringstream stream{"1 1 1"};
    turtlelib::Twist2D tw1;
    stream >> tw1;
    REQUIRE(tw1.omega == 1.0);
    REQUIRE(tw1.x == 1.0);
    REQUIRE(tw1.y == 1.0);

    std::stringstream stream2 {"1 d 1 1"};
    turtlelib::Twist2D tw2;
    stream2 >> tw2;

    REQUIRE(tw2.omega == turtlelib::deg2rad(1.0));
    REQUIRE(tw2.x == 1.0);
    REQUIRE(tw2.y == 1.0);

    std::stringstream stream3 {"<-2 d, 3, 2>"};
    turtlelib::Twist2D tw3;
    stream3 >> tw3;

    REQUIRE(tw3.omega == turtlelib::deg2rad(-2));
    REQUIRE(tw3.x == 3.0);
    REQUIRE(tw3.y == 2.0);
}

TEST_CASE("Create transforms", "Transform2D")
{
    // Check that default constructor is correct
    turtlelib::Transform2D tf;
    REQUIRE(tf.theta == 0);
    REQUIRE(tf.x == 0);
    REQUIRE(tf.y == 0);

    // Check that rotate constructor is correct
    double rotate {1};
    turtlelib::Transform2D tf2(rotate);
    REQUIRE(tf2.theta == 1);
    REQUIRE(tf2.x == 0);
    REQUIRE(tf2.y == 0);

    // Check trans constructor
    turtlelib::Vector2D vc;
    vc.x = 1;
    vc.y = 2;
    turtlelib::Transform2D tf3(vc);
    REQUIRE(tf3.theta == 0);
    REQUIRE(tf3.x == 1);
    REQUIRE(tf3.y == 2);

    // Check rot and trans
    turtlelib::Transform2D tf4(vc, rotate);
    std::print("{}",vc.x);
    REQUIRE(tf4.theta == 1);
    REQUIRE(tf4.x == 1);
    REQUIRE(tf4.y == 2);
}

TEST_CASE("Transform a point", "Transform2D")
{
    // origin with no transformation
    turtlelib::Transform2D tf;
    turtlelib::Point2D pt;
    auto pt2 = tf(pt);
    REQUIRE(pt2.x == pt.x);
    REQUIRE(pt2.y == pt.y);

    // origin with pure rotation
    double rot = turtlelib::deg2rad(45);
    turtlelib::Transform2D tf2(rot);
    auto pt3 = tf2(pt2);
    REQUIRE(pt3.x == pt2.x);
    REQUIRE(pt3.y == pt2.y);

    // origin with pure translation
    turtlelib::Vector2D oo;
    oo.x = 1;
    oo.y = -1;
    turtlelib::Transform2D tf4(oo);
    auto pt4 = tf4(pt3);
    REQUIRE(pt4.x == oo.x);
    REQUIRE(pt4.y == oo.y);

    // from this point, translate and rotate
    turtlelib::Transform2D tf5(oo, rot);
    auto pt5 = tf5(pt4);
    REQUIRE(pt5.x == 1+std::sqrt(2));
    REQUIRE(pt5.y == -1);
}

TEST_CASE("Transform origin vector (expect no changes)", "Transform2D")
{
    // origin with no transformation
    turtlelib::Transform2D tf;
    turtlelib::Vector2D vc_origin;
    auto vc2 = tf(vc_origin);
    REQUIRE(vc2.x == vc_origin.x);
    REQUIRE(vc2.y == vc_origin.y);

    // origin with pure rotation
    double rot = turtlelib::deg2rad(45);
    turtlelib::Transform2D tf2(rot);
    auto vc3 = tf2(vc_origin);
    REQUIRE(vc3.x == vc_origin.x);
    REQUIRE(vc3.y == vc_origin.y);

    // origin with pure translation (shouldn't change)
    turtlelib::Vector2D vec_one_negone;
    vec_one_negone.x = 1;
    vec_one_negone.y = -1;
    turtlelib::Transform2D tf4(vec_one_negone);
    auto vc4 = tf4(vc_origin);
    REQUIRE(vc4.x == vc_origin.x);
    REQUIRE(vc4.y == vc_origin.y);

    // translate and rotate origin (should just rotate)
    turtlelib::Transform2D tf5(vec_one_negone, rot);
    auto vc5 = tf5(vc_origin);
    REQUIRE(vc5.x == vc_origin.x);
    REQUIRE(vc5.y == vc_origin.y);
}

TEST_CASE("Transform vector [1, 1]", "Transform2D")
{
    // origin with no transformation
    turtlelib::Transform2D tf;
    turtlelib::Vector2D vc_one_one;
    vc_one_one.x = 1.0;
    vc_one_one.y = 1.0;
    auto vc2 = tf(vc_one_one);
    REQUIRE(vc2.x == vc_one_one.x);
    REQUIRE(vc2.y == vc_one_one.y);

    // origin with pure rotation
    double rot = turtlelib::deg2rad(45);
    turtlelib::Transform2D tf2(rot);
    auto vc3 = tf2(vc_one_one);
    REQUIRE(vc3.x == Catch::Approx(0));
    REQUIRE(vc3.y == Catch::Approx(std::sqrt(2)));

    // origin with pure translation (shouldn't change)
    turtlelib::Vector2D vec_one_negone;
    vec_one_negone.x = 1;
    vec_one_negone.y = -1;
    turtlelib::Transform2D tf4(vec_one_negone);
    auto vc4 = tf4(vc_one_one);
    REQUIRE(vc4.x == vc_one_one.x);
    REQUIRE(vc4.y == vc_one_one.y);

    // translate and rotate origin (should just rotate)
    turtlelib::Transform2D tf5(vec_one_negone, rot);
    auto vc5 = tf5(vc_one_one);
    REQUIRE(vc5.x == 0.0);
    REQUIRE(vc5.y == std::sqrt(2));
}

// TODO: need test for transforming a Twist2D

