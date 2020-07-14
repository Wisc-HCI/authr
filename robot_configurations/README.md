# Robot Configurations
Provides a single repository for robot configurations used in lab projects/studies
with non-standard MoveIt configurations and/or robot descriptions.

Note that for the Authr release we have simplified the package to only the necessary modules.

## Workflow

To install, simply clone the repository into your catkin workspace and install
the relevant dependencies for the robot being used (see below). Note that MoveIt
should be installed.

When extending this repository, ensure that contributions do not fail to build
when dependencies are not installed. As an example, if using a UR5 then Panda
dependencies should not be required for catkin_make to successfully build. A
notable exception is for MoveIt, which if not installed may necessitate deletion of `*_moveit_config` directories.

## Robots
- UR 3 + Robotiq 85
- UR 5 + Robotiq 85
- UR 10 + Robotiq 85
- Franka Emika Panda

## Dependencies
- Universal Robots UR 3 / UR 5 / UR 10
  - [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)
    - Use the remote serial version of the driver
  - [fmauch/universal_robot](https://github.com/fmauch/universal_robot)
    - Use this fork until merged into [ros-industrial](https://github.com/ros-industrial/universal_robot) if using hardware driver
  - [industrial_core](wiki.ros.org/industrial_core) via `apt install ros-<VERSION>-industrial-core`
  - (Optional) [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

- Franka Emika Panda
  - [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)

- General
  - (Optional) [moveit](http://wiki.ros.org/moveit) via `apt install ros-<VERSION>-moveit`
