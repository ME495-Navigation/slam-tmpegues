#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

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
    REQUIRE(tf.tw.omega == 0);
    REQUIRE(tf.tw.x == 0);
    REQUIRE(tf.tw.y == 0);

    // Check that rotate constructor is correct
    double rotate {1};
    turtlelib::Transform2D tf2(rotate);
    REQUIRE(tf2.tw.omega == 1);
    REQUIRE(tf2.tw.x == 0);
    REQUIRE(tf2.tw.y == 0);

    // Check trans constructor
    turtlelib::Vector2D vc;
    vc.x = 1;
    vc.y = 2;
    turtlelib::Transform2D tf3(vc);
    REQUIRE(tf3.tw.omega == 0);
    REQUIRE(tf3.tw.x == 1);
    REQUIRE(tf3.tw.y == 2);

    // Check rot and trans
    turtlelib::Transform2D tf4(vc, rotate);
    REQUIRE(tf2.tw.omega == 1);
    REQUIRE(tf3.tw.x == 1);
    REQUIRE(tf3.tw.y == 2);
}

TEST_CASE("Transform a point")