#include <iostream>
#include <print>

#include "turtlelib/geometry2d.hpp"


int main()
{
    while (true)
    {
        std::print("\n");
        turtlelib::Point2D pt;
        std::print("initial pt: {} _ {}\nProvide point:", pt.x, pt.y);
        std::cin >> pt;
        std::print("2nd pt: {} _ {}\n", pt.x, pt.y);
        if (pt.x == 0)
            return 0;

    }
    return 0;
}