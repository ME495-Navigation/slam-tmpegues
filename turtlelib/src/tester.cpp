#include <iostream>
#include <print>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"


int main()
{
    while (true)
    {
        std::print("\n");
        turtlelib::Twist2D tw;
        std::print("initial twist: {} _ {} _ {} \nProvide twist:", tw.omega, tw.x, tw.y);
        std::cin >> tw;

        std::print("new_twist twist: {} _ {} _ {} \n", tw.omega, tw.x, tw.y);
        if (tw.x == 0)
            return 0;

    }
    return 0;
}