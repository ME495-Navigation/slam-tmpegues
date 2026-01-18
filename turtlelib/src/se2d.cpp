#include <iosfwd>
#include <iostream>

std::istream &operator>>(std::istream &is, Point2D &p)
{

    // For now, I'll just restrict input to only "x y" right now and come back to this later to
    // allow "(x, y)" and add error handling
    if (std::cin.peek() != 40)
    {
        is >> p.x >> p.y;
    }
    else
    {
        std::string x_str; // Needs to be a string so I can catch the comma
        std::cin >> x_str;
        x_str.erase(0, 1);
        if (x_str.back() == 44) // 44 is value for ,
        {
            x_str.pop_back();
            p.x = std::stod(x_str);

            std::string y_str;
            std::cin >> y_str;

            if (y_str.back() == 41) // 41 is value for )
            {
                y_str.pop_back();
                p.y = std::stod(y_str);
            }
        }
    }
    return is;
}