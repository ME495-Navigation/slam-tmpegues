#include <sstream>
#include <cmath>

#include"turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Point2D & p)
    {


        if (is.peek() == '(')
        {// if the first character is (, then assume it's properly formatted as (x, y)
            is.get(); // purge the (
            is >> p.x;
            is.get(); // purge the ,
            is >> p.y;
            is.get(); // purge the )
        }
        else
        {// If we can break it into 2 numbers, do it
            is >> p.x >> p.y;
        }
        return is;
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {

            if (is.peek() == '[')
            { // if the first character is [, then assume it's properly formatted as [x, y]
                is.get(); // Purge [
                is >> v.x;
                is.get(); // Purge ,
                is >> v.y;
                is.get(); // Purge ]
            }
            else
            { // If we can break it into 2 numbers, do it
                is >> v.x >> v.y;
            }
            return is;
        }

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {
        // vector is head - tail
        Vector2D vect {};

        vect.x = head.x - tail.x;
        vect.y = head.y - tail.y;
        return vect;
    }

    Point2D operator+(const Point2D &tail, const Vector2D &disp)
    {
        Point2D new_pt {};
        new_pt.x = tail.x + disp.x;
        new_pt.y = tail.y + disp.y;

        return new_pt;
    }

    std::ostream & operator<<(std::ostream &os, const Vector2D &v)
    {
        os << "[" <<  v.x << ", " << v.y << "]";
        return os; // TODO: do I need to return?
    }

    std::ostream &operator<<(std::ostream &os, const Point2D & p)
    {
        os << "(" << p.x << ", " << p.y << ")";
        return os; // TODO: do I need to return?
    }

    Vector2D normalize(Vector2D in)
    {
        auto length = std::sqrt((std::pow(in.x, 2) + std::pow(in.y, 2)));
        turtlelib::Vector2D normalized;
        normalized.x = in.x/length;
        normalized.y = in.y/length;
        return normalized;
    }
}