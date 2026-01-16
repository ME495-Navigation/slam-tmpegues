#include <iosfwd>
#include <iostream>
#include <sstream>
#include <cctype>

#include"turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        // For now, I'll just restrict input to only "x y" right now and come back to this later to
        // allow "(x, y)" and add error handling
        is >> p.x >> p.y;

        return is;
    }

    std::istream & operator>>(std::istream &is, Vector2D &v)
    {
        // For now, I'll just restrict input to only "x y" right now and come back to this later to
        // allow "[x, y]" and add error handling
        is >> v.x >> v.y;

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


}