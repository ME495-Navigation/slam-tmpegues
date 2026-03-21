# ME495 Sensing, Navigation and Machine Learning For Robotics
* T. Miguel Pegues
* Winter 2025
# Package List
This repository consists of several ROS packages for implementing EKF SLAM on a Turtlebot 3
- nuturtle_description - Contains modified turtlebot3 models and related functionality.
- nusim - The nusim node simulates an environment for the turtlebot3 to navigate and simulates the turtlebot itself.
- nuturtle_control - Notes interface between general commands (like a Twist message on /cmd_vel) and turtlebot specifc commands (like specific wheel velocity commands)
- nuturtle_control - Contains services for interfacing with nuturtle_control

Below are 2 videos of the same test: commanding the real Turtlebot to drive in circles and tracking it with the blue simulated odometry robot. The calculated difference between initial and final poses is .24 m, which is largely due to my stopping ability.

https://github.com/user-attachments/assets/3da914dc-451f-4573-abe0-b7032d3c6018


https://github.com/user-attachments/assets/3722678f-b899-4d1a-a516-6c3166d8bda0

While I have not completed actual SLAM, a solid simulation framework is in place. The video below shows the following features.
* A simulated red Turtlebot can be driven using any `cmd_vel` source, such as `teleop_twist_keyboard`.
* The red Turtlebot has collision and cannot pass through obstacles. 
* Simulated LIDAR that scans the flat walls of the arena, along with cylindrical obstacles. Simulated LIDAR has noise on both range and bearing.
* Separate from LIDAR, there are noisy "detected" obstacles shown as yellow cylinders
* Odometry, shown as the blue Turtlebot, calculated from simulated wheel encoder data.
* Execution of commanded twists are subject to noise, along with the addition of physical wheel slip.


https://github.com/user-attachments/assets/b2b31ecd-0e40-4675-ad0b-d1d75be26413

To experience this yourself, run `ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim`, then in another terminal, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
