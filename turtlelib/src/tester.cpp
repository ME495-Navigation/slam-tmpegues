#include <cmath>
#include <iostream>
#include <print>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
using std::numbers::pi;

int main()
{
  bool end = true;
  turtlelib::Twist2D tw{};
  tw.x = 1;
  tw.y = 0;
  double w = 0;
  while (w <= 2 * pi) {
    std::cout << "\n";
    tw.x = w;
    tw.omega = w;
    // std::cout << tw << "\n";
    auto tf = turtlelib::integrate_twist(tw);
    std::cout << tf.rotation() << "\n";
    std::cout << tf.translation() << "\n";
    w += pi / 8;
  }

  while (not end) {
    std::print("t(r)ansform, (t)wist, (v)ector, (p)oint?, or (e)nd: ");
    char choice{'e'};
    std::cin >> choice;
    std::cin.get();
    switch (choice) {
      case 'p': {
          turtlelib::Point2D pt;
          std::cout << "Initial point: " << pt << "\nInput:";
          std::cin >> pt;
          std::cout << "New point: " << pt << "\n";
          break;
        }
      case 'v': {
          turtlelib::Vector2D vc;
          std::cout << "Initial vector: " << vc << "\nInput:";
          std::cin >> vc;
          std::cout << "New vector: " << vc << "\n";
          break;
        }
      case 't': {
          turtlelib::Twist2D tw;
          std::print("Initial twist: <{}, {}, {}>\nInput:", tw.omega, tw.x, tw.y);
          std::cin >> tw;
          std::cout << "New twist: " << tw << "\n";
          break;
        }
      // case 'r':
      // {
      //     turtlelib::Transform2D tr;
      //     std::print("Initial transform twist: ");
      //     std::cout << tr.tw << "\nInput: ";
      //     std::cin >> tr;
      //     std::cout << "New transform twist: " << tr.tw << "\n";
      //     break;
      // }
      default: {
          end = true;
        }
    }
  }
  return 0;
}
