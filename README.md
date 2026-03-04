# ME495 Sensing, Navigation and Machine Learning For Robotics
* T. Miguel Pegues
* Winter 2025
# Package List
This repository consists of several ROS packages
- nuturtle_description - Contains modified turtlebot3 models and related functionality.
- nusim - The nusim node simulates an environment for the turtlebot3 to navigate and simulates the turtlebot itself.
- nuturtle_control - Notes interface between general commands (like a Twist message on /cmd_vel) and turtlebot specifc commands (like specific wheel velocity commands)
- nuturtle_control - Contains services for interfacing with nuturtle_control