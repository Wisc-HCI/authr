# Authr - Release v3.0
Provided in this repository is the release version of Authr along with additional ROS package dependencies configured to work with the Authr software.

Authr is an environment designed to allow engineers to translate existing manual processes into collaborative human-robot plans. Additionally, Authr provides the ability to design exclusively robotic plans.

What makes Authr novel is a fully defined pipeline for designing collaborative applications. The user can iteratively setup the virtual work environment model, program in a visual drag-and-drop programming environment, utilize automatic agent allocation, and simulate their plan with the built-in, ROS-based plan simulator. Authr is designed around a hierarchical task breakdown with three levels: the plan, tasks, and therbligs. Each task within a plan is a linear process of work composed of therbligs. Therbligs are the lowest-level action primitive used in Authr and originally established by Gilbreth and Gilbreth for human work. Authr currently implements a subset of the physical therbligs with the goal of future work to operationalize cognitive therbligs.

For more details on how we implemented and evaluated Authr, check out our paper [Authr: A Task Authoring Environment for Human-Robot Teams]().

If you find Authr useful for your project, please consider citing our paper,

```
FILL IN
```

To get started using Authr, follow the installation guide below along with the starting Authr guide. Or to start investigating the implementation of Authr, check out the implementation details section at the bottom of this README.

## Installation

First clone this repository into your ROS catkin workspace.

Next install the following dependencies for ROS,
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [RobotWebTools/tf2_web_republisher](https://github.com/RobotWebTools/tf2_web_republisher)
  - [RobotWebTools/interactive_marker_proxy](https://github.com/RobotWebTools/interactive_marker_proxy)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
- [ros/geometry](https://github.com/ros/geometry) (For ROS-melodic)
- [MoveIt](http://wiki.ros.org/moveit) via `apt install ros-<VERSION>-moveit`

Then install the robot dependencies,
- [industrial_core](wiki.ros.org/industrial_core) via `apt install ros-<VERSION>-industrial-core`
- (Install if using Panda) Follow their [installation guide](https://frankaemika.github.io/docs/installation_linux.html) and checkout their Github: [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)
- (Install if using Universal Robots)[ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) or `apt install ros-<VERSION>-universal-robots`

Next install the following Python 2 modules using pip,
- z3py (`pip install z3-solver`)
- scipy (`pip install scipy`)
- tornado (`pip install tornado`) OR for ros-melodic (`pip install tornado==4.5.3`) ** used for rosbridge, specific version fixes disconnect issues
- pymongo (`pip install pymongo`) ** used for rosbridge, fixes disconnect issues
- pyassimp (`pip uninstall pyassimp`) and then  (`pip install -H pyassimp==4.1.3`) ** Needs to be 4.1.3 or 3.3 version. Anything else will cause a segfault when loading custom moveit collider meshes
- scipy (`pip install scipy`)
- pyquaternion (`pip install pyquaternion`)
- numpy (`pip install numpy`)

Finally, for the frontend install Node.js and NPM. Run `npm install` inside the `authr` directory.

That's it. To start Authr follow the description below.

## Starting


- The whole deal: `roslaunch authr authr.launch robot:=(ur3 | ur5 | ur10) [verifier:=(standard | advanced)]`


- The backend: `roslaunch authr backend.launch robot:=(ur3 | ur5 | ur10) [verifier:=(standard | advanced)]`


- The frontend: `roslaunch authr frontend.launch`

Use the tool [here](https://wisc-hci.github.io/authr/)!

## Implementation Details
In this repository we included the Authr interface itself along with several support libraries used in our lab to provide robot control to Authr. Checkout each subsystem's README files listed below.

1. [Authr](./authr/README.md) is a visual programming tool to convert

2. [robot_behavior](./robot_behavior/README.md) provides a bridge between the Authr system and the low-level ROS and MoveIt systems.

3. [robot_configurations](./robot_configurations/README.md) provides standard robot configurations we use in our lab for various projects. For Authr we currently support Universal Robots UR3, UR5, and UR10 and Franka Emika Panda.

4. [robotiq_85_gripper](./robotiq_85_gripper/README.md) is a fork of the Robotiq driver that has been modified and extended by our lab. Robot configurations and robot_behavior depend on this version of the robotiq_85_gripper ROS package to supply an action server and a modified URDF.

5. [custom_meshes](./custom_meshes/README.md) provides ROS with the custom meshes used in the Authr visualization.

6. [authr_pddl](./authr_pddl/README.md) used to evaluate Authr's custom allocation algorithm against a traditional PDDL approach. This subdirectory is not needed to run the actual interface.
