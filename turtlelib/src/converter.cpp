#include <iostream>
#include <print>
#include <cmath>

#include"turtlelib/angle.hpp"

std::pair<double, std::string> get_input()
{
    double angle{0.0};
    std::string unit;
    bool failed {false};
    while (true)
    {
        if (failed)
        {
            std::print("Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n");
            failed = false;
        }

        std::cin >> angle >> unit;
        if (std::cin.eof())
        {
            unit = "break";
            return {angle, unit};
        }
        else if (std::cin.fail())
        {
           std::cin.clear();
           std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           failed = true;
           continue;
        }

        if (unit == "deg")
        {
            return {angle, unit};
            ;
        }
        else if (unit == "rad")
        {
            return {angle, unit};
            ;
        }
    }

}

void convert_and_print(double angle, std::string unit)
{
    double converted_normal_angle{0.0};
    std::string other_unit;

    if (unit == "deg")
    {
        angle = (std::fmod(angle + 180.0, 360.0) - 180.0);
        converted_normal_angle = turtlelib::deg2rad(angle);
        converted_normal_angle = turtlelib::normalize_angle(converted_normal_angle);
        other_unit = "rad";
    }
    else if (unit == "rad")
    {
        angle = turtlelib::normalize_angle(angle);
        converted_normal_angle = turtlelib::rad2deg(angle);
        other_unit = "deg";
    }
    std::print("{} {} is {} {}.\n\n", angle, unit, converted_normal_angle, other_unit);
}

int main()
{
    while (true)
    {
        // 1. Get angle and unit
        std::cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n";
        auto [angle, unit] = get_input();

        if (unit == "break")
        {
            break;
        }
        // 2. Convert to other unit and print
        convert_and_print(angle, unit);

    }
        return 0;
}