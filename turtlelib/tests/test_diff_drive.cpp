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
  REQUIRE(dd.phi().l() == 0);
  REQUIRE(dd.phi().r() == 0);

  // small equal rotation on both wheels should result in x translation of that rotation amount
  auto rot {pi / 4.0};
  dd.fk(Wheels(rot, rot));
  REQUIRE(dd.get_transform().translation().x == rot);
  REQUIRE(dd.get_transform().translation().y == 0);
  REQUIRE(dd.get_transform().rotation() == 0);
  REQUIRE(dd.phi().l() == rot);
  REQUIRE(dd.phi().r() == rot);

  // Returning to the previous position should return state to original
  dd.fk(Wheels(0, 0));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == 0.0);
  REQUIRE(dd.phi().l() == 0.0);
  REQUIRE(dd.phi().r() == 0.0);

  // Wheels rotation equal distances in opposite direction should spin with no translation
  dd.fk(Wheels(-rot, rot));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == rot);
  REQUIRE(dd.phi().l() == -rot);
  REQUIRE(dd.phi().r() == rot);

  // Back it up
  rot = 0.0;
  dd.fk(Wheels(-rot, rot));
  REQUIRE(dd.get_transform().translation().x == 0.0);
  REQUIRE(dd.get_transform().translation().y == 0.0);
  REQUIRE(dd.get_transform().rotation() == 0.0);
  REQUIRE(dd.phi().l() == 0.0);
  REQUIRE(dd.phi().r() == 0.0);

  // Rotate only one wheel should result in an x, y, and theta change (multiple iterations so I get to a location with easy numbers)
  DiffDrive dd2{2.0, 1.0};  // Wheels have circumference 2*pi

  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));
  dd2.fk(Wheels(0, dd2.phi().r() + pi / 4));


  REQUIRE(dd2.phi().l() == 0);
  REQUIRE(dd2.phi().r() == normalize_angle(2 * pi));
  REQUIRE_THAT(dd2.get_transform().translation().x, WithinAbs(0, 0.00001));
  REQUIRE_THAT(dd2.get_transform().translation().y, WithinAbs(2, 0.00001));
  REQUIRE_THAT(dd2.get_transform().rotation(), WithinAbs(normalize_angle(pi), 0.00001));
}

TEST_CASE("Test ik", "DiffDrive::ik")
{
  DiffDrive dd{2.0, 1.0};

  auto speeds{DiffDrive{2.0, 1.0}.ik({0.0, 1, 0.0})}; // Pure translation +
  REQUIRE(dd.ik({0.0, 1, 0.0}).l() == 1);
  REQUIRE(speeds.r() == 1);

  speeds = DiffDrive{2.0, 1.0}.ik({0.0, -1.0, 0.0}); // Pure translation -
  REQUIRE(speeds.l() == -1);
  REQUIRE(speeds.r() == -1);

  speeds = DiffDrive{2.0, 1.0}.ik({1.0, 0.0, 0.0});
  REQUIRE(speeds.l() == -1);
  REQUIRE(speeds.r() == 1);

  speeds = DiffDrive{2.0, 1.0}.ik({-1.0, 0.0, 0.0}); // Pure rotation -
  REQUIRE(speeds.l() == 1);
  REQUIRE(speeds.r() == -1);

  speeds = DiffDrive{2.0, 1.0}.ik({pi / 2, pi / 2, 0.0});
  REQUIRE(speeds.l() == 0);
  REQUIRE(speeds.r() == pi);

  // Twist2D body_tw4{-pi / 2, -pi / 2, 0.0};
  speeds = DiffDrive{2.0, 1.0}.ik({-pi / 2, -pi / 2, 0.0});
  REQUIRE(speeds.l() == 0);
  REQUIRE(speeds.r() == pi);

  REQUIRE_THROWS_AS(dd.ik({1, 1, 1}), std::logic_error);
}


// Tests below this point were given to me by Conor

TEST_CASE("DiffDrive forward motion (forward/inverse)", "[Conor]")
{
  const auto wheel_radius = 0.1;
  const auto wheel_track = 0.5;

  auto dd = DiffDrive{wheel_radius, wheel_track};

  // initialize wheel angle
  dd.fk(Wheels(0.0, 0.0));
  dd.fk(WheelDiff(2.0 * pi, 2.0 * pi));
  auto tf = dd.get_transform();
  REQUIRE_THAT(tf.rotation(), WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.translation().x, WithinAbs(2.0 * pi * wheel_radius, 1e-6));
  REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));

  auto wheels = dd.ik(Twist2D{0.0, 0.1, 0.0});
  REQUIRE_THAT(wheels.l(), WithinAbs(1.0, 1e-6));
  REQUIRE_THAT(wheels.r(), WithinAbs(1.0, 1e-6));
}

TEST_CASE("DiffDrive pure rotation (forward/inverse)", "[Conor]")
{
  const auto wheel_radius = 0.1;
  const auto wheel_track = 0.5;

  auto dd = DiffDrive{wheel_radius, wheel_track};

  // spin around 180 degrees
  // this means 0.5/2 * pi = 0.25pi arc length for both wheels
  // so the wheels have to spin 0.25pi / 0.1 = 2.5pi radians in opposite directions
  dd.fk(Wheels(0.0, 0.0));
  dd.fk(WheelDiff(-pi * 2.5, pi * 2.5));
  auto tf = dd.get_transform();

  REQUIRE_THAT(tf.translation().x, WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.rotation(), WithinAbs(pi, 1e-6));

  // 180 degree rotation in place should use the same math in reverse
  auto wheels = dd.ik(Twist2D{pi, 0.0, 0.0});
  REQUIRE_THAT(wheels.l(), WithinAbs(-2.5 * pi, 1e-6));
  REQUIRE_THAT(wheels.r(), WithinAbs(2.5 * pi, 1e-6));
}

TEST_CASE("DiffDrive circular arc (forward/inverse)", "[Conor]")
{
  const auto wheel_radius = 0.1;
  const auto wheel_track = 0.5;

  auto dd = DiffDrive{wheel_radius, wheel_track};

  dd.fk(Wheels(0.0, 0.0));
  dd.fk(Wheels(1.0, 2.0));
  auto tf = dd.get_transform();


  const auto omega = (wheel_radius / wheel_track) * (2.0 - 1.0);
  const auto v_x = (wheel_radius / 2.0) * (1.0 + 2.0);
  const auto expected_x = (v_x / omega) * std::sin(omega);
  const auto expected_y = (v_x / omega) * (1.0 - std::cos(omega));

  REQUIRE_THAT(tf.rotation(), WithinAbs(omega, 1e-6));
  REQUIRE_THAT(tf.translation().x, WithinAbs(expected_x, 1e-6));
  REQUIRE_THAT(tf.translation().y, WithinAbs(expected_y, 1e-6));

  auto wheels = dd.ik(Twist2D{omega, v_x, 0.0});
  REQUIRE_THAT(wheels.l(), WithinAbs(1.0, 1e-6));
  REQUIRE_THAT(wheels.r(), WithinAbs(2.0, 1e-6));
}

TEST_CASE("DiffDrive inverse kinematics rejects lateral motion", "[Conor]")
{
  auto dd = DiffDrive{0.1, 0.5};

  REQUIRE_THROWS_AS(dd.ik(Twist2D{0.0, 0.1, 0.01}), std::logic_error);
}