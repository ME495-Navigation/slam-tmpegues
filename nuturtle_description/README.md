# Nuturtle  Description
URDF files for Nuturtle <Name Your Robot>
* `<Command Here>` to see the robot in rviz.
* `<Command Here>` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)

# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.xml -s`
  ```
  Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        True will open rviz, False will not open rviz
        (default: 'True')

    'use_jsp':
        True will use joint_state_publisher, False will not use joint_state_publisher
        (default: 'True')

    'color':
        What color do you want your turtle to be?. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')```
* `<Command To Show Arguments of load_all.launch.py>`
  `<Output of the Above Command>`