# robotiq_85_gripper

## Original README
Common packages for the Robotiq 85 Gripper provided by Stanley Innovation

Defaults to 'ttyUSB0' and 115200 baud rate

Single gripper and left gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface
Right gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface


start with:
```
roslaunch robotiq_85_bringup robotiq_85.launch run_test:=true
```

## Fork Notes
Repository forked from [waypointrobotics/robotiq_85_gripper](https://github.com/waypointrobotics/robotiq_85_gripper).

Why use this version?
- Removed Kinova arm coupler from URDF (applicable to more robots off the shelf).
- Provides a gripper_action_server to convert GripperCommandActions used by
planning pipelines like MoveIt into driver specific messages.
- Referenced by [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations) repository.
- Provides URScript-based gripper action server for use with UR robots with only embedded gripper connections
  - Note: with the release of the new Official UR ROS driver, it is recommended to use the remote serial connection instead.
- Provides a variant of the robotiq driver from remote serial communication.
  - Provides additional delay for critical communication steps instead of failing to connect.
  - Lowers polling rate to device
- Optional command time limits on ROS subscriber

## Authr Release Notes
Robot Configurations with UR robot's require this package for the Robotiq description. The driver is not being used.

## URScript Version
The URScript driver was developed specifically for the UR3e robot in the lab.
This driver provides basic gripper functionality without having to bringup the
standard robotiq driver. However, it does require the `ur_modern_driver` to
provide a URScript topic.

Note: With the release of the new UR Official ROS driver this approach is no
longer needed to command a Robotiq gripper mounted on an e-series robot. Instead
use the remote communication version of the driver.

Note: This driver is not complete. Work should be done to minimize script
length and extend functionality.

Note: The script used is based off the generated script for the robotiq urcap.
Please install and use this cap where possible. The source provided here is only
to provide the functionality for this driver and is not intended for general use.

## Remote Communication Version
In order to use this ROS driver with the UR Official Driver under remote serial
communication it was necessary to provide additional time to communicate between
the PC and the gripper. Additionally, to manage bandwidth the polling rate was
reduced on the gripper.

To use this version simply run this node in replacement of the original robotiq
driver. Can be used with the action-server.

Note: Additional work can be done to fine-tune the timing.
