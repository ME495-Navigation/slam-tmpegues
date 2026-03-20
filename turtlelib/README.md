# turtlelib
A C++ library for geometry, tailored for use in 2D SLAM

# Headers
* angle.hpp
    * Contains functionality for converting between degrees and radians, checking near-equality of angles, calculating minimum angle differences, and angle_wrapping

* diff_drive.hpp
    * Contains forward and inverse kinematic calculations for a differential dive robot.

* geometry2d.hpp
    * Contains 2d point and vector calculations that are used in se2d.hpp

* se2d.hpp
    * Contains functionality for constructing transformations and applying them to points, vectors, twists, and other transformations

* wheels.hpp
    * Contains Wheel and WheelDiff classes that are used within diff_drive.hppc

* laser.hpp
    * Contains a laser class with functions for line-circle intersection and line-line intersection