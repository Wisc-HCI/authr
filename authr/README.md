# Authr
ROS Package/Angular Frontend for the NRI Authoring Tool (Authr)

## Installation
Authr was developed for ROS Melodic on Ubuntu 18.04. Installation may need to be
modified if targeting a different environment.

Note that using `rosdep install <PACKAGE>` will not install all of the dependencies for authr specific packages. Manual installation is required due to version / dependency issues.

### 1. Clone this repository
First clone this repository into your ROS catkin workspace.

### 2. Install ROS Dependencies
Next install the following ROS Bridge dependencies,
- [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
  - [RobotWebTools/tf2_web_republisher](https://github.com/RobotWebTools/tf2_web_republisher)
  - [RobotWebTools/interactive_marker_proxy](https://github.com/RobotWebTools/interactive_marker_proxy)
  - [GT-RAIL/rosauth](https://github.com/GT-RAIL/rosauth)
  
Then install the robot dependencies,
- [MoveIt](http://wiki.ros.org/moveit) via `apt install ros-<VERSION>-moveit`
- [industrial_core](http://wiki.ros.org/industrial_core) via `apt install ros-<VERSION>-industrial-core`
- (Install if using Franka Emika Panda) Follow their [installation guide](https://frankaemika.github.io/docs/installation_linux.html) and checkout their Github: [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)
- (Install if using Universal Robots) Clone [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)

### 3. Install Python Dependencies
Next install the following Python 2 modules using pip.
```
pip install z3-solver scipy numpy pyquaternion pymongo
```

We also need to confirm state of several modules in order for everything to work.
- tornado should be version 4.5.3 to fix disconnect issues with rosbridge. Install with `pip install tornado==4.5.3`.
- pymongo should be installed to fix disconnect issues with rosbridge.
- pyassimp needs to be at version 3.3 or 4.1.3 to fix Segault when using MoveIt with custom mesh collider objects. Install with `pip install pyassimp==4.1.3`.

### 4. Install Node.js
Finally, for Authr's user interface install Node.js and NPM. For ROS Melodic just installing from the package manager may cause ROS to [uninstall](https://answers.ros.org/question/329144/installing-ros-melodic-ros-base-deletes-npm-installing-npm-deletes-ros-how-can-i-have-both/). Simply enter the following into the terminal.

```
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt-get install nodejs
```

Then run `npm install` inside the `authr` directory. This step is optional as part of building the workspace CMake will invoke this command.

### 5. Build
That's it. Now just build and source the catkin workspace. To start Authr follow the description below.

## Starting
To run Authr, enter the following into the terminal,
```
roslaunch authr authr.launch robot:=(ur3 | ur5 | ur10 | panda) [verifier:=(standard | advanced)]
```

Or launch the subsystems individually,

```
roslaunch authr frontend.launch
```

and

```
roslaunch authr backend.launch robot:=(ur3 | ur5 | ur10 | panda) [verifier:=(standard | advanced)]
```

Currently our implementation supports Universal Robots UR3, UR5, and UR10 and Franka Emika Panda. We do not provide preconfigured MoveIt packages for Universal Robots E-Series or other collaborative robots like Rethink's Sawyer robot. Authr can be extended to support these robots but will require some changes to the launch file. Checkout the implementation section of this README for details.

By default the verifier parameter is set to standard. This is the version we used for technical and user evaluations in our paper. The advanced option is a new Z3 implementation that provides shorter
verification and allocation times.

We also provide a static build of our user interface [here](https://wisc-hci.github.io/authr-release/). The backend subsystem will still need to be run locally. Additionally, because Github enforces SSL, you will need to configure your backend to use a valid certificate.

## Usage

Once you have the frontend and backend running, you will need to point the browser to the hosted webpage. This should be printed in the output of NPM, but if you are running on your machine locally, this is likely `localhost:4200`. The launch page shows an entry field for the URL of the backend. The ROS connection uses port `9090` on localhost (unless you have specified otherwise), so entering `localhost:9090` should allow you access to the tool. A video of using the tool can be found below: 

[![Authr Interface](http://img.youtube.com/vi/9vS3B3UrpTU/0.jpg)](http://www.youtube.com/watch?v=9vS3B3UrpTU "Authr").


## Implementation Details
### Adding a Robot
Adding a robot to Authr is relatively straightforward though it has a few caveats. Below we detail the steps you should take to configure your robot.

#### 1. MoveIt Configuration
Authr uses MoveIt to control the robot agent for simulation and thus any robot used has to have a MoveIt Configuration. Often these are provided by the community though you may need to add a gripper to the robot arm and thus create a custom configuration or modify the existing. Refer to this [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) to use the MoveIt Configuration Wizard. Authr requires the arm move-group to be called `manipulator` and the gripper move-group to be called `gripper`. Furthermore, Authr currently constrains the gripper to have the same value for each control joint.

#### 2. Backend Integration
Next you need to update the launch files to your robot configuration. First create a new launch file for your robot which will load MoveIt with your robot, provide joint-state / robot-state publishers, and provide any robot specific nodes. After creating that launch file, add it to the lookup table in `backend.launch`. The following code should be entered,

```
<include if="$(eval arg('robot') == '<YOUR_ROBOT_NAME>')" file="$(find authr)/launch/<YOUR_ROBOT_NAME>.launch">
  <arg name='simulated' value="$(arg simulated)"/>
</include>
```

Additionally, when launching `authr_sim.py` you will need to provide the gripper linear function converting effort to position. Effort is constrained to be between 0 and 1 inclusively. Add the following inside the node launch XML.

```
<param if="$(eval arg('robot') == '<YOUR_ROBOT_NAME>')" name="robot_effort_scalar" value="<YOUR_GRIPPER_CLOSED_STATE>"/>
<param if="$(eval arg('robot') == '<YOUR_ROBOT_NAME>')" name="robot_effort_offset" value="<YOUR_GRIPPER_OPENED_STATE>"/>
```

#### 3. Frontend Integration
In order for Authr to visualize the robot within the embedded 3D workspace visualization, you will need to copy the robot's meshes into the following directory,

```
.\authr\src_angular\assets\meshes\<YOUR_ROBOT_DESCRIPTION_NAME>\*
```

The name and structure of the directory should match the convention provided in the robot's URDF.

### Github Build
To generate the Github static hosted version of our interface first run,

```
npm run-script build
```

Then deploy to host. Built files can be found in the `docs` subdirectory.
