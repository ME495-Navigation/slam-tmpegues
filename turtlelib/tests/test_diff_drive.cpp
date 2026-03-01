#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <stdexcept>

#include "turtlelib/angle.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("Test fk", "DiffDrive::fk")
{
  DiffDrive dd{2.0, 1.0};  // Wheels have circumference 2*pi

  // 0 rotation on both wheels should result in no change
  dd.fk(Wheels(0, 0));
  REQUIRE(dd.get_transform().translation().x == 0);
  REQUIRE(dd.get_transform().translation().y == 0);
  REQUIRE(dd.get_transform().rotation() == 0);
  REQUIRE(dd.get_wheels().left == 0);
  REQUIRE(dd.get_wheels().right == 0);

  // small equal rotation on both wheels should result in x translation of that rotation amount
  auto rot {pi / 4.0};
  dd.fk(Wheels(rot, rot));
  REQUIRE(dd.get_transform().translation().x == rot);
  REQUIRE(dd.get_transform().translation().y == 0);
  REQUIRE(dd.get_transform().rotation() == 0);
  REQUIRE(dd.get_wheels().left == rot);
  REQUIRE(dd.get_wheels().right == rot);

  // Returning to the previous position should return state to original
  dd.fk(Wheels(0, 0));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == 0.0);
  REQUIRE(dd.get_wheels().left == 0.0);
  REQUIRE(dd.get_wheels().right == 0.0);

  // Wheels rotation equal distances in opposite direction should spin with no translation
  dd.fk(Wheels(-rot, rot));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == rot);
  REQUIRE(dd.get_wheels().left == -rot);
  REQUIRE(dd.get_wheels().right == rot);

  // Back it up
  rot = 0.0;
  dd.fk(Wheels(-rot, rot));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == 0.0);
  REQUIRE(dd.get_wheels().left == 0.0);
  REQUIRE(dd.get_wheels().right == 0.0);

  // Rotate only one wheel should result in an x, y, and theta change (multiple iterations so I get to a location with easy numbers)
  DiffDrive dd2{2.0, 1.0};  // Wheels have circumference 2*pi

  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));
  dd2.fk(Wheels(0, dd2.get_wheels().right + pi / 4));


  REQUIRE(dd2.get_wheels().left == 0);
  REQUIRE(dd2.get_wheels().right == normalize_angle(2 * pi));
  REQUIRE_THAT(dd2.get_transform().translation().x, WithinAbs(0, 0.00001));
  REQUIRE_THAT(dd2.get_transform().translation().y, WithinAbs(2, 0.00001));
  REQUIRE_THAT(dd2.get_transform().rotation(), WithinAbs(normalize_angle(pi), 0.00001));
}

TEST_CASE("Test ik", "DiffDrive::ik")
{
  DiffDrive dd{2.0, 1.0};

  auto speeds{DiffDrive{2.0, 1.0}.ik({0.0, 1, 0.0})}; // Pure translation +
  REQUIRE(dd.ik({0.0, 1, 0.0}).left == 1);
  REQUIRE(speeds.right == 1);

  speeds = DiffDrive{2.0, 1.0}.ik({0.0, -1.0, 0.0}); // Pure translation -
  REQUIRE(speeds.left == -1);
  REQUIRE(speeds.right == -1);

  speeds = DiffDrive{2.0, 1.0}.ik({1.0, 0.0, 0.0});
  REQUIRE(speeds.left == -1);
  REQUIRE(speeds.right == 1);

  speeds = DiffDrive{2.0, 1.0}.ik({-1.0, 0.0, 0.0}); // Pure rotation -
  REQUIRE(speeds.left == 1);
  REQUIRE(speeds.right == -1);

  speeds = DiffDrive{2.0, 1.0}.ik({pi / 2, pi / 2, 0.0});
  REQUIRE(speeds.left == 0);
  REQUIRE(speeds.right == pi);

  // Twist2D body_tw4{-pi / 2, -pi / 2, 0.0};
  speeds = DiffDrive{2.0, 1.0}.ik({-pi / 2, -pi / 2, 0.0});
  REQUIRE(speeds.left == 0);
  REQUIRE(speeds.right == -pi);

  REQUIRE_THROWS_AS(dd.ik({1, 1, 1}), std::logic_error);
}
