#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

TEST_CASE("Create a twist with format 'w x y' and 'x <unit> x y' and <w x y>, with radians and degrees", "std::istream & operator>>")
{
    std::stringstream stream{"1 1 1\n"};
    turtlelib::Twist2D tw1;
    stream >> tw1;
    REQUIRE(tw1.omega == 1.0);
    REQUIRE(tw1.x == 1.0);
    REQUIRE(tw1.y == 1.0);

    stream << "1 d 1 1/n";
    stream >> tw1;
    REQUIRE(tw1.omega == turtlelib::rad2deg(1.0));
    REQUIRE(tw1.x == 1.0);
    REQUIRE(tw1.y == 1.0);
}