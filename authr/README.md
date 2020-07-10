# Authr
ROS Package/Angular Frontend for the NRI Authoring Tool (Authr)

Use the tool [here](https://wisc-hci.github.io/authr/)!

### Installation

First clone this repository into your ROS catkin workspace.

Install the following dependencies for ROS,
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [RobotWebTools/tf2_web_republisher](https://github.com/RobotWebTools/tf2_web_republisher)
  - [RobotWebTools/interactive_marker_proxy](https://github.com/RobotWebTools/interactive_marker_proxy)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
- [ros/geometry](https://github.com/ros/geometry) (For ROS-melodic)

Install the robot dependencies,
- [industrial_core](wiki.ros.org/industrial_core) via `apt install ros-<VERSION>-industrial-core`
- (Install if using Panda) Follow their [installation guide](https://frankaemika.github.io/docs/installation_linux.html) and checkout their Github: [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)
- (Install if using Universal Robots)[ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) or `apt install ros-<VERSION>-universal-robots`

Install the following Python 2 modules using pip,
- z3py (`pip install z3-solver`)
- scipy (`pip install scipy`)
- tornado (`pip install tornado`) OR for ros-melodic (`pip install tornado==4.5.3`) ** used for rosbridge, specific version fixes disconnect issues
- pymongo (`pip install pymongo`) ** used for rosbridge, fixes disconnect issues
- pyassimp (`pip uninstall pyassimp`) and then  (`pip install -H pyassimp==4.1.3`) ** Needs to be 4.1.3 or 3.3 version. Anything else will cause a segfault when loading custom moveit collider meshes
- scipy (`pip install scipy`)
- pyquaternion (`pip install pyquaternion`)
- numpy (`pip install numpy`)

Finally, for the frontend install Node.js and NPM. Run `npm install` inside the `authr` directory.

### Starting:
- The whole deal: `roslaunch authr authr.launch robot:=(ur3 | ur5 | ur10) [verifier:=(standard | advanced)]`
- The backend: `roslaunch authr backend.launch robot:=(ur3 | ur5 | ur10) [verifier:=(standard | advanced)]`
- The frontend: `roslaunch authr frontend.launch`

### Structure:
- src_angular: Angular app for frontend. (run `ng serve --open` to start)
- launch: Launch files to initialize frontend, backend, and everything.
- src_ros: Backend code for optimizing plans and planning behaviors.

### Github Build
- angular: Build app then host on remote server. (run `npm run-script build`)
- launch: Launch backend on local machine
