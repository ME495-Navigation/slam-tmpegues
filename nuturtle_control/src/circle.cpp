#include "rclcpp/rclcpp.hpp"

class circle : public rclcpp::Node
{
    public: circle() : Node("circle")
    {}
    private:

};

std::shared_ptr<circle> my_node = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    my_node = std::make_shared<circle>();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
