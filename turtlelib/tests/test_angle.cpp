#include <catch2/catch_test_macros.hpp>

#include "turtlelib/angle.hpp"

TEST_CASE("Independently testing turtlelib::normalize angle()", "turtlelib::normalize_angle")
{
  REQUIRE(turtlelib::normalize_angle(-5.0 * (std::numbers::pi / 2.0)) == -std::numbers::pi / 2.0);
}