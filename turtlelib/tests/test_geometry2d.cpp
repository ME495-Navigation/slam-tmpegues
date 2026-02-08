#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <iostream>

#include "turtlelib/geometry2d.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("Create points and with format 'x y' and '(x, y)'", "std::istream & operator>>")
{
  std::stringstream stream{"1 1\n"};
  turtlelib::Point2D pt1;
  stream >> pt1;
  REQUIRE(pt1.x == 1.0);
  REQUIRE(pt1.y == 1.0);

  std::stringstream stream2{"(-5.35, 10.78)\n"};
  turtlelib::Point2D pt2;
  stream2 >> pt2;
  REQUIRE(pt2.x == -5.35);
  REQUIRE(pt2.y == 10.78);
}

TEST_CASE("Create vectors and with format 'x y' and '[x, y]", "std::istream & operator>>")
{
  std::stringstream stream3{"1 1\n"};
  turtlelib::Vector2D vc1;
  stream3 >> vc1;
  REQUIRE(vc1.x == 1.0);
  REQUIRE(vc1.y == 1.0);

  std::stringstream stream2{"[-5.35, 10.78\n"};
  turtlelib::Vector2D vc2;
  stream2 >> vc2;
  REQUIRE(vc2.x == -5.35);
  REQUIRE(vc2.y == 10.78);
}

TEST_CASE("Subtract points to make vectors", "Vector2D operator-")
{
  turtlelib::Point2D origin{0.0, 0.0};

  turtlelib::Point2D pt_oo{1.0, 1.0};

  turtlelib::Point2D pt_ot{1.0, 2.0};

  // 1 1 - 0 0 = 1 1
  auto vec_res_oo{pt_oo - origin};
  REQUIRE(vec_res_oo.x == 1.0);
  REQUIRE(vec_res_oo.y == 1.0);
  auto vec_res_minoo(origin - pt_oo);
  REQUIRE(vec_res_minoo.x == -1);
  REQUIRE(vec_res_minoo.y == -1);

  // 1 2 - 0 0 = 1 2
  auto vec_res_ot{pt_ot - origin};
  REQUIRE(vec_res_ot.x == 1.0);
  REQUIRE(vec_res_ot.y == 2);
  auto vec_res_minot{origin - pt_ot};
  REQUIRE(vec_res_minot.x == -1);
  REQUIRE(vec_res_minot.y == -2);

  // 1 2 - 1 1 = 0 -1
  auto vec_res_nno{pt_oo - pt_ot};
  REQUIRE(vec_res_nno.x == 0.0);
  REQUIRE(vec_res_nno.y == -1.0);
  auto vec_res_minnno{pt_ot - pt_oo};
  REQUIRE(vec_res_minnno.x == -0.0);
  REQUIRE(vec_res_minnno.y == 1.0);
}

TEST_CASE("Add a point and vector to make a new point", "Point2D operator+")
{
  turtlelib::Point2D origin{0.0, 0.0};

  turtlelib::Vector2D vec_oo{1.0, 1.0};

  turtlelib::Point2D pt_oo{1.0, 1.0};

  auto pt_res_oo{origin + vec_oo};
  REQUIRE(pt_res_oo.x == 1.0);
  REQUIRE(pt_res_oo.y == 1.0);

  auto pt_res_tt{pt_oo + vec_oo};
  REQUIRE(pt_res_tt.x == 2.0);
  REQUIRE(pt_res_tt.y == 2.0);

  turtlelib::Vector2D vec_ot{1.0, 2.0};

  auto pt_res_ot{origin + vec_ot};
  REQUIRE(pt_res_ot.x == 1.0);
  REQUIRE(pt_res_ot.y == 2.0);

  auto pt_res_tth{pt_oo + vec_ot};
  REQUIRE(pt_res_tth.x == 2.0);
  REQUIRE(pt_res_tth.y == 3.0);
}

TEST_CASE(
  "Normalize vectors. Normalize uses magnitude(), so this also tests magnitude()",
  "Vector2D normalize")
{
  turtlelib::Vector2D vec1{1.0, 1.0};
  auto vec2 = turtlelib::normalize(vec1);

  REQUIRE(vec2.x == 1.0 / std::sqrt(2.0));
  REQUIRE(vec2.y == 1.0 / std::sqrt(2.0));

  vec1 *= -1;
  vec2 = turtlelib::normalize(vec1);
  REQUIRE(vec2.x == -1.0 / std::sqrt(2.0));
  REQUIRE(vec2.y == -1.0 / std::sqrt(2.0));

  vec1.x = 3;
  vec1.y = 4;
  vec2 = turtlelib::normalize(vec1);
  REQUIRE(vec2.x == 3.0 / 5.0);
  REQUIRE(vec2.y == 4.0 / 5.0);
}

TEST_CASE("Vector scaling", "operator*")
{
  turtlelib::Vector2D vec00{0.0, 0.0};
  auto vec2 = vec00 * 10;
  REQUIRE(vec2.x == 0.0);
  REQUIRE(vec2.y == 0.0);

  turtlelib::Vector2D vec11{1.0, 1.0};
  auto vec3 = vec11 * 2;
  REQUIRE(vec3.x == 2.0);
  REQUIRE(vec3.y == 2.0);

  auto vec4 = 3.5 * vec11;
  REQUIRE(vec4.x == 3.5);
  REQUIRE(vec4.y == 3.5);
}

TEST_CASE("Vector addition", "operator+")
{
  turtlelib::Vector2D vec00{0.0, 0.0};
  turtlelib::Vector2D vec11{1.0, 1.0};

  auto vec2 = vec00 + vec11;
  REQUIRE(vec2.x == 1.0);
  REQUIRE(vec2.y == 1.0);

  turtlelib::Vector2D vec3{42.3, -120.75};
  auto vec4 = vec3 + vec11;
  REQUIRE(vec4.x == 43.3);
  REQUIRE(vec4.y == -119.75);
}

TEST_CASE("Vector subtraction", "operator-")
{
  turtlelib::Vector2D vec00;
  vec00.x = 0.0;
  vec00.y = 0.0;
  turtlelib::Vector2D vec11;
  vec11.x = 1.0;
  vec11.y = 1.0;

  auto vec2 = vec00 - vec11;
  REQUIRE(vec2.x == -1.0);
  REQUIRE(vec2.y == -1.0);

  turtlelib::Vector2D vec3;
  vec3.x = 42.3;
  vec3.y = -120.75;
  auto vec4 = vec3 - vec11;
  REQUIRE(vec4.x == 41.3);
  REQUIRE(vec4.y == -121.75);
}

TEST_CASE("Vector dot product", "dot()")
{
  turtlelib::Vector2D vec00;
  vec00.x = 0.0;
  vec00.y = 0.0;
  turtlelib::Vector2D vec11;
  vec11.x = 1.0;
  vec11.y = 1.0;

  auto res1 = dot(vec00, vec11);
  REQUIRE(res1 == 0.0);

  auto res2 = dot(vec11, vec11);
  REQUIRE(res2 == 2.0);

  turtlelib::Vector2D vec3;
  vec3.x = 42.3;
  vec3.y = -120.75;

  auto res3 = dot(vec11, vec3);
  REQUIRE(res3 == 42.3 - 120.75);
}

TEST_CASE("Vector angle", "angle()")
{
  turtlelib::Vector2D vec10;
  vec10.x = 1.0;
  vec10.y = 0.0;
  turtlelib::Vector2D vec11;
  vec11.x = 1.0;
  vec11.y = 1.0;
  turtlelib::Vector2D vec01;
  vec01.x = 0.0;
  vec01.y = 1.0;

  auto ang1 = angle(vec10, vec01);
  REQUIRE_THAT(ang1, WithinAbs(std::numbers::pi / 2, 0.0001));
  auto angn1 = angle(vec01, vec10);
  REQUIRE_THAT(angn1, WithinAbs(-std::numbers::pi / 2, 0.0001));

  auto ang2 = angle(vec10, vec11);
  REQUIRE_THAT(ang2, WithinAbs(std::numbers::pi / 4, 0.0001));

  auto ang3 = angle(vec11, vec01);
  REQUIRE_THAT(ang3, WithinAbs(std::numbers::pi / 4, 0.0001));

  auto ang4 = angle(vec11, vec11);
  REQUIRE(ang4 == 0.0);

  Vector2D vec;
  vec.x = -1;
  vec.y = 0.0;
  auto ang5 = angle(vec10, vec);
  REQUIRE_THAT(ang5, WithinAbs(std::numbers::pi, 0.0001));
}