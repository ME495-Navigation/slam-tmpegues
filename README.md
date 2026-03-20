# ME495 Sensing, Navigation and Machine Learning For Robotics
* T. Miguel Pegues
* Winter 2025
# Package List
This repository consists of several ROS packages
- nuturtle_description - Contains modified turtlebot3 models and related functionality.
- nusim - The nusim node simulates an environment for the turtlebot3 to navigate and simulates the turtlebot itself.
- nuturtle_control - Notes interface between general commands (like a Twist message on /cmd_vel) and turtlebot specifc commands (like specific wheel velocity commands)
- nuturtle_control - Contains services for interfacing with nuturtle_control

Below are 2 videos of the same test: commanding the real Turtlebot to drive in circles and tracking it with the blue simulated odometry robot. The calculated difference between initial and final poses is .24 m, which is largely due to my stopping ability. 

https://github.com/user-attachments/assets/3da914dc-451f-4573-abe0-b7032d3c6018


https://github.com/user-attachments/assets/3722678f-b899-4d1a-a516-6c3166d8bda0
