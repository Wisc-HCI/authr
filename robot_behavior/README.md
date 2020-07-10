# Robot Behavior

Robot Behavior is a ROS package with the goal of providing general robot
functionality for use in the HCI lab. Provided in this package is a suite of
arm trajectory planner wrappers written in python under a common abstraction.
Also provided is a suite of pre-built behaviors to get started on your robot
project.

## Dependencies
Currently there are no required ROS packages to build this package. However, to use
the planner interface install MoveIt.

- [MoveIt](http://wiki.ros.org/moveit)

Next install the following Python 2 modules using pip,

- scipy (`pip install scipy`)
- pyquaternion (`pip install pyquaternion`)
- numpy (`pip install numpy`)
