#include <sstream>

#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

#include <print>

namespace turtlelib
{
    std::istream & operator>>(std::istream &is, Twist2D &tw)
    {
        // If first character is <, get rid of it
        if (is.peek() == '<')
        {
            is.get();
        }
        // Then read first number
        is >> tw.omega;
        is.get(); // Purge the space

        std::print("peek1: {}\n", is.peek());

        // Check if next character is r or d
        if (is.peek() == 'r') // remove the string, but do nothing else. Keep all units in rad/sec
        {

            std::string str;
            is >> str;
        }
        else if (is.peek() == 'd') // remove the string, then convert deg/s to rad/s
        {
            std::string str;
            is >> str;
            tw.omega = deg2rad(tw.omega);
        }

        // Check for commas between every other character
        if (is.peek() == ',')
        {
            is.get();
        }

        is >> tw.x;

        if (is.peek() == ',')
        {
            is.get();
        }

        is >> tw.y;

        if (is.peek() == '>')
        {
            is.get();
        }

        return is;
    }
}