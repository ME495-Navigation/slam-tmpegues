Task 0.1
Task 0.2
Task 0.3
Task 0.4
Task A.1
Task A.2
*   Need to figure out comments in the xacros
```
<!-- ##### Begin_Citation [1] ##### -->
<!-- This file was taken from ROBOTIS-GIT's turtlebot3 repo on github (Citation #1) -->
<!-- It has been modified to do the following -->
<!-- * Add this comment -->
<!-- * Remove dependency on the turtlebot3 package by changing file references to this nuturtle_description packge-->
<!-- * Modified collision of base_link to be a cylinder -->
<!-- * Wheel radius, track width, and overall collision now read from diff_params.yaml-->
<!-- #####  End_Citation [1]  ##### -->
```
Task A.3
Task A.4
Task A.5
Task A.6
* Can I hide the arguments from included launchfiles from when you do `ros2 launch ... ... -s`?

Task B.1
Task B.2
Task B.3
* Formatters not done
Task B.4
Task B.5
* Formatters not done
* Most things really need to be tested, but I've at least got a start for everything.
Task B.6
* Probably like half done
Task B.7
* Skipping
Task B.8
* Skipping
Task B.9
* Skipping

Task C.1
Tasl C.2
* Created timer
* Did not create ~/timestep publisher
    * fatal error: std_msgs/msg/uint64.hpp: No such file or directory
* Created reset service
    * It doesn't reset the timestep since I haven't created that yet
Task C.3
* Tried to create a publisher, but it's saying various variables don't exist, but it's saying the same ones are unused3
Task C.4
* Attempted, and the topic showed up once on rqt_graph, but does not work
Task C.5
* Skipping
Task C.6
* Rviz opens with a config that shows:
    * Fixed Frame `nusim/world`
    * RobotModel in red (not correctly placed yet)
    * MarkerArray on topic nusim/real_walls
    * Shows axes `nusim_world`
Task C.7
* I'll do it when I get at least half of this node working