#include <sstream>
#include <cmath>

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
        else
        {
        }
        // Then read first number
        is >> tw.omega;
        is.get(); // Purge the space

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

    std::ostream &operator<<(std::ostream &os, const Twist2D & tw)
    {
        os << "<" << tw.omega << ", " << tw.x << ", " << tw.y << '>' ;
        return os; // TODO: do I need to return?
    }

    Transform2D::Transform2D()
    {
        // Twist2D tw;
        tw.omega = 0;
        tw.x = 0;
        tw.y = 0;
    }

    Transform2D::Transform2D(Vector2D trans)
    {
        // Twist2D tw;
        tw.omega = 0;
        tw.x = trans.x;
        tw.y = trans.y;
    }

    Transform2D::Transform2D(double radians)
    {
        // Twist2D tw;
        tw.omega = radians;
        tw.x = 0;
        tw.y = 0;
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
    {
        // Twist2D tw;
        tw.omega = radians;
        tw.x = trans.x;
        tw.y = trans.y;
    }

    Point2D Transform2D::operator()(Point2D p) const
    {
        // Rotate then translate the point
        Point2D new_pt;
        new_pt.x = std::cos(tw.omega) * p.x - std::sin(tw.omega) * p.y;
        new_pt.y = std::cos(tw.omega) * p.y + std::sin(tw.omega) * p.x;

        new_pt.x += tw.x;
        new_pt.y += tw.y;

        return new_pt;
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        // This is exactly the same math as doing a point above
        Vector2D new_vc;
        new_vc.x = std::cos(tw.omega) * v.x - std::sin(tw.omega) * v.y;
        new_vc.y = std::cos(tw.omega) * v.y + std::sin(tw.omega) * v.x;

        new_vc.x += tw.x;
        new_vc.y += tw.y;

        return new_vc;
    }

    Twist2D Transform2D::operator()(Twist2D v ) const
    {
        // I think the only thing we need to deal with here is just the translation bit
        // TODO: check if this is correct. Also check how do I just use the function above instead of copy pasting it

        Vector2D new_vc;
        new_vc.x = std::cos(tw.omega) * v.x - std::sin(tw.omega) * v.y;
        new_vc.y = std::cos(tw.omega) * v.y + std::sin(tw.omega) * v.x;

        new_vc.x += tw.x;
        new_vc.y += tw.y;

    }

    Transform2D Transform2D::inv() const
    {
        // TODO: check. This seems too easy

        auto new_rot = tw.omega * -1;
        Vector2D new_vc;
        new_vc.x = -1;
        new_vc.y = -1;

        Transform2D new_tf(new_vc, new_rot);

    return new_tf;
    }

    // Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    // {  //TODO: how to return
    //     // I think we just add all the values together?
    //     tw.omega += rhs.tw.omega;
    //     tw.x += rhs.tw.x;
    //     tw.y = rhs.tw.y;

    //     return rhs ; //
    // }

}