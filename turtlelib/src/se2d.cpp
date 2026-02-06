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
        ; // Default values for twist and costh and sinth are for an identity transform
    }

    Transform2D::Transform2D(Vector2D trans) // use initializer lists
    {
        tw.x = trans.x;
        tw.y = trans.y;

        x = trans.x;
        y = trans.y;
        // Default values for tw.omega, theta, costh, sinth are correct for no rotation
    }

    Transform2D::Transform2D(double radians)
    {
        tw.omega = radians;
        // Default values for tw.x and tw.y are correct for no translation
        theta = radians;
        costh = std::cos(radians);
        sinth = std::sin(radians);
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
    {
        tw.omega = radians;
        tw.x = trans.x;
        tw.y = trans.y;

        theta = radians;
        x = trans.x;
        y = trans.y;
        costh = std::cos(radians);
        sinth = std::sin(radians);
    }

    Point2D Transform2D::operator()(Point2D p) const
    {
        // Rotate then translate the point
        Point2D new_pt;
        new_pt.x = costh * p.x - sinth * p.y;
        new_pt.y = costh * p.y + sinth * p.x;

        new_pt.x += tw.x;
        new_pt.y += tw.y;

        return new_pt;
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        // Transforming a vector only rotates it
        Vector2D new_vc;
        new_vc.x = costh * v.x - sinth * v.y;
        new_vc.y = costh * v.y + sinth * v.x;

        return new_vc;
    }

    Twist2D Transform2D::operator()(Twist2D v ) const
    {
        Twist2D new_tw;
        new_tw.omega = v.omega;
        new_tw.x = costh*v.x - sinth*v.y + tw.y*v.omega;
        new_tw.y = costh*v.y + sinth*v.x - tw.x*v.omega;

        return new_tw;
    }

    Transform2D Transform2D::inv() const
    {
        // TODO: check. This seems too easy

        Vector2D new_vc;
        new_vc.x = -x*costh-y*sinth;
        new_vc.y = -y*costh+x*sinth;

        Transform2D new_tf(new_vc, -theta);

    return new_tf;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        theta += rhs.theta;
        Vector2D new_vec; // unnecessary temporary
        new_vec.x = rhs.x;
        new_vec.y = rhs.y;
        x += costh * rhs.x - sinth * rhs.y;
        y += costh * rhs.y + sinth * rhs.x;

        return *this; //
    }

    Vector2D Transform2D::translation() const
    {
        Vector2D new_vc;
        new_vc.x = tw.x;
        new_vc.y = tw.y;
        return new_vc;
    }

    double Transform2D::rotation() const
    {
        return tw.omega;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        // there are much simpler solutions here
        // If first character is '{', get rid of it and the following '<'
        if (is.peek() == '{')
        {
            is.get(); // purge {
            is.get(); // purge <
        }

        double angle {0.0};
        is >> angle;

        if (is.peek() == '>')
        {
            is.get(); // purge >
            if (is.peek() == ' ')
            {
                is.get(); // Purge the ' ' space if we will be getting a unit.
                          // If we're not getting a unit, then the peek would be a comma.
            }
        }
        std::print("Peek 1: {}\n", static_cast<char>(is.peek()));
        switch (is.peek()) // either we get a '<', indicating that we're being given a unit,
                           // or a ',' or  ' ', or a digit indicating we aren't.
                           // Anything else is a problem.
        {
            case ' ': // this means that unit was not given and we can assume radians
            case ',': // this means that unit was not given and we can assume radians
            {

                tf.tw.omega = angle;
                break;
            }
            case '<': // this means we need to check the unit
            {
                std::print("in the unit case\n");
                is.get(); // Purge the <
                if (is.peek() == 'r') // We have radians again, but we need to purge the string
                {
                    tf.tw.omega = angle;
                    std::string str;
                    is >> str;
                    is.get(); // Purge the ' ' space
                }
                else if (is.peek() == 'd') // Convert from deg to rad, then purge string
                {
                    tf.tw.omega = deg2rad(angle);
                    std::string str;
                    is >> str;
                    is.get(); // Purge the ' ' space
                }
                break;
            }
        }
        std::print("Peek 2: {}\n", static_cast<char>(is.peek()));
        if (is.peek() == ',')
        {
            is.get(); // purge ,
            is.get(); // purge ' ' space
        }

        if (is.peek() == '<')
        {
            is.get(); // purge <
        }

        is >> tf.tw.x;

        if (is.peek() == '>')
        {
            is.get(); // purge >
            is.get(); // purge ,
            is.get(); // purge ' ' space
        }

        if (is.peek() == '<')
        {
            is.get(); // purge <
        }


        is >> tf.tw.y;

        if (is.peek() == '>')
        {
            is.get(); // purge >
            is.get(); // purge }
        }

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        // should be
        // lhs *=rhs
        // return lhs;
        // what you ahve here is subtly different and can cause problems
        return lhs*=rhs;
    }
}
