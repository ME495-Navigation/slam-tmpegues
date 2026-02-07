#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"
#include <cmath>

#include <iostream>

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

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
    vc_one_one.x = 1.0;
    vc_one_one.y = 1.0;
    auto vc3 = tf2(vc_one_one);
    REQUIRE_THAT(vc3.x, WithinAbs(0, 0.00001));
    REQUIRE_THAT(vc3.y, WithinAbs(std::sqrt(2), 0.00001));

    // REQUIRE(vc3.x == Catch::Approx(0));
    // REQUIRE(vc3.y == Catch::Approx(std::sqrt(2)));

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
    // REQUIRE(vc5.x == 0.0);
    // REQUIRE(vc5.y == std::sqrt(2));
    REQUIRE_THAT(vc5.x, WithinAbs(0, 0.00001));
    REQUIRE_THAT(vc5.y, WithinAbs(std::sqrt(2), 0.00001));
}

// TODO: need test for transforming a Twist2D
TEST_CASE("Transform a twist", "Transform2D::operator(Twist2D)")
{
    // Identity transform of twist with omega, x, and y
    Transform2D tr1 {};
    Twist2D tw_input {deg2rad(90.0), 1.0, 1.0};
    auto tw_res = tr1(tw_input);
    REQUIRE(tw_res.omega == tw_input.omega);
    REQUIRE(tw_res.x == tw_input.x);
    REQUIRE(tw_res.y == tw_input.y);

    // Pure rotation of a twist with omega, x, and y

    double rot2 {deg2rad(90.0)};
    Transform2D tr2(rot2);
    tw_res = tr2(tw_input);
    REQUIRE_THAT(tw_res.omega, WithinAbs(tw_input.omega, 0.00001));
    REQUIRE_THAT(tw_res.x, WithinAbs(-tw_input.x, 0.00001));
    REQUIRE_THAT(tw_res.y, WithinAbs(tw_input.y, 0.00001));

    // Translate a twist with omega, x, and y
    Vector2D trans2{0.0, 0.0};
    Transform2D tr3(trans2);
    tw_res = tr3(tw_input);
    REQUIRE(tw_res.omega == tw_input.omega);
    REQUIRE(tw_res.x == tw_input.x);
    REQUIRE(tw_res.y == tw_input.y);

    // Translate and rotate
    Transform2D tr4(trans2, rot2);
    tw_res = tr4(tw_input);
    REQUIRE_THAT(tw_res.omega, WithinAbs(tw_input.omega, 0.00001));
    REQUIRE_THAT(tw_res.x, WithinAbs(-tw_input.x, 0.00001));
    REQUIRE_THAT(tw_res.y, WithinAbs(tw_input.y, 0.00001));

    // Translate and rotate, but with a different y component on the Transform2D
    Vector2D trans3{1, 2};
    Transform2D tr5(trans3, rot2);
    tw_res = tr5(tw_input);
    REQUIRE_THAT(tw_res.omega, WithinAbs(tw_input.omega, 0.00001));
    REQUIRE_THAT(tw_res.x, WithinAbs(std::numbers::pi-1, 0.00001));
    REQUIRE_THAT(tw_res.y, WithinAbs((2-std::numbers::pi)/2, 0.00001));

}

TEST_CASE("Transform2D operator*", "[Conor]")
{
    // this is also implicitly testing operator*=
    SECTION("Multiply two transforms")
    {
        Vector2D trans1{2.0, 0.0};
        double angle1 = deg2rad(90);
        Transform2D tf1(trans1, angle1);

        Vector2D trans2{1.0, 0.0};
        double angle2 = deg2rad(45);
        Transform2D tf2(trans2, angle2);

        Transform2D result = tf1 * tf2;
        REQUIRE_THAT(result.rotation(), WithinAbs(deg2rad(135), 0.00001));
        REQUIRE_THAT(result.translation().x, WithinAbs(2.0, 0.00001));
        REQUIRE_THAT(result.translation().y, WithinAbs(1.0, 0.00001));
    }

    SECTION("operator* does not modify operands")
    {
        Vector2D trans1{1.0, 2.0};
        double angle1 = deg2rad(30);
        Transform2D tf1(trans1, angle1);

        Vector2D trans2{3.0, 4.0};
        double angle2 = deg2rad(60);
        Transform2D tf2(trans2, angle2);

        // Store original values
        Vector2D orig_trans1 = tf1.translation();
        double orig_angle1 = tf1.rotation();
        Vector2D orig_trans2 = tf2.translation();
        double orig_angle2 = tf2.rotation();
        // Perform multiplication
        [[maybe_unused]] Transform2D result = tf1 * tf2;

        // Verify operands were not modified
        REQUIRE_THAT(tf1.translation().x, WithinAbs(orig_trans1.x, 0.00001));
        REQUIRE_THAT(tf1.translation().y, WithinAbs(orig_trans1.y, 0.00001));
        REQUIRE_THAT(tf1.rotation(), WithinAbs(orig_angle1, 0.00001));
        REQUIRE_THAT(tf2.translation().x, WithinAbs(orig_trans2.x, 0.00001));
        REQUIRE_THAT(tf2.translation().y, WithinAbs(orig_trans2.y, 0.00001));
        REQUIRE_THAT(tf2.rotation(), WithinAbs(orig_angle2, 0.00001));
    }
}

TEST_CASE("Scale a twist by a scalar", "operator*")
{
    Twist2D tw1;
    tw1.x = 1;
    tw1.y = 1;
    tw1.omega = 1;

    auto tw2 = tw1 * 2;
    auto tw3 = tw1 * -1.5;
    REQUIRE(tw2.x == 2.0);
    REQUIRE(tw2.y == 2.0);
    REQUIRE(tw2.omega == 2.0);

    REQUIRE(tw3.x == -1.5);
    REQUIRE(tw3.y == -1.5);
    REQUIRE(tw3.omega == -1.5);
}

TEST_CASE("Integrate twists into transforms", "integrate_twist()")
{
    Twist2D tw_trans {};
    tw_trans.x = 1;
    tw_trans.y = 1;
    auto tf1 = integrate_twist(tw_trans);

    REQUIRE(tf1.translation().x == 1.0);
    REQUIRE(tf1.translation().y == 1.0);
    REQUIRE(tf1.rotation() == 0.0);

    Twist2D tw_rot {};
    tw_rot.omega = 1;
    auto tf2 = integrate_twist(tw_rot);

    REQUIRE(tf2.translation().x == 0.0);
    REQUIRE(tf2.translation().y == 0.0);
    REQUIRE(tf2.rotation() == 1.0);

    Twist2D twist {};
    twist.x = 1;
    twist.y = 0;
    twist.omega = pi/2;
    auto tf3 = integrate_twist(twist);

    REQUIRE(tf3.translation().x == 1.0);
    REQUIRE(tf3.translation().y == 1.0);
    REQUIRE(tf3.rotation() == pi/2);
}
