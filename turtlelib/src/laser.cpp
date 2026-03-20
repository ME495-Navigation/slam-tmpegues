#include <cmath>

#include "turtlelib/laser.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"
#include <iostream>
namespace turtlelib
{
Laser::Laser(double min_range, double max_range)
{
  near_point = Point2D(min_range, 0);
  far_point = Point2D(max_range, 0);
}
std::pair<bool, double> Laser::obs_check(double angle, Transform2D T_rob_obs, double radius)
{
        // Now with obstacle locations in robot frame, do circle line intersection to find if the laser hits a particular object
        // Using wolfram circle line. x1 = near laser point, x2 = far laser point TODO: cite
        // Poln = coordinates of laser minimum measure point in object frame
        // Polf = cordinates of laser maximum measure point in object frame
  auto prln = Transform2D(angle)(near_point);
  auto prlf = Transform2D(angle)(far_point);

  auto Poln = T_rob_obs.inv()(prln);
  auto Polf = T_rob_obs.inv()(prlf);
  auto d = turtlelib::Vector2D(Polf.x - Poln.x, Polf.y - Poln.y);
  auto D = Poln.x * Polf.y - Polf.x * Poln.y;
  auto delta = std::pow(radius, 2) * std::pow(turtlelib::magnitude(d), 2) - std::pow(D, 2);

  if (delta < 0) {
    return std::make_pair(false, 0);
  } else {
    auto p1 = turtlelib::Point2D();
    auto p2 = turtlelib::Point2D();
    auto sgn = (d.y < 0 ? -1.0 : 1.0);
    p1.x = (D * d.y + sgn * d.x * sqrt(delta)) / std::pow(turtlelib::magnitude(d), 2);
    p2.x = (D * d.y - sgn * d.x * sqrt(delta)) / std::pow(turtlelib::magnitude(d), 2);

    p1.y = (-D * d.x + fabs(d.y) * sqrt(delta)) / std::pow(turtlelib::magnitude(d), 2);
    p2.y = (-D * d.x - fabs(d.y) * sqrt(delta)) / std::pow(turtlelib::magnitude(d), 2);

            // Calculate which one is closer to Poln, as the point closer to the laser is the one that will be read
    auto Vn1 = p1 - Poln;
    auto Vn2 = p2 - Poln;
    auto P_ohit = ((turtlelib::magnitude(Vn1) <= turtlelib::magnitude(Vn2)) ? p1 : p2);

            // Transform hit_point back into the robot frame
    auto P_rhit = T_rob_obs(P_ohit);
    if ((normalize_angle(angle) > 0 && P_rhit.y > 0) ||
      (normalize_angle(angle) < 0 && P_rhit.y < 0))
    {
      auto V_rhit = turtlelib::Vector2D(P_rhit.x, P_rhit.y);

                // std::cout << "Hit at angle, obs: " << angle * 360 / (2 * std::numbers::pi) << "\n";
                // std::cout << "rob_obs, obs_rob: " << T_rob_obs.translation() << ", " << T_rob_obs.inv().translation() << "\n";
                // std::cout << "Laser near, far (r frame): " << near_point << ", " << far_point << "\n";
                // std::cout << "Laser near, far (o frame): " << Poln << ", " << Polf << "\n";
                // std::cout << "Contact points 1, 2, chosen (o frame): " << p1 << ", " << p2 << ", " << P_ohit << "\n";
                // std::cout << "Contact points 1, 2, chosen (r frame): " << T_rob_obs(p1) << ", " << T_rob_obs(p2) << ", " << T_rob_obs(P_ohit) << "\n";

      return std::make_pair(true, magnitude(V_rhit));
    } else {
      return std::make_pair(false, 0);
    }
  }
}
}
