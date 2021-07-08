# hector_spot_ros
## Overview
This package provides a [ROS](http://www.ros.org/) interface to control the [Boston Dynamics Spot](https://www.bostondynamics.com/spot). It provides most functionalities of the [spot-sdk](https://github.com/boston-dynamics/spot-sdk) via topics and services, e.g. power on/off, movement commands, robot state and sensor access.
## Installation
### spot-sdk
First, install the [BD Spot SDK](https://github.com/boston-dynamics/spot-sdk). The following is a condensed version of [quickstart.md](https://github.com/boston-dynamics/spot-sdk/blob/master/docs/python/quickstart.md).

Install PIP3 by running
```
sudo apt-get install python3-pip
pip3 --version
pip3 install --upgrade pip
```
Afterwards, clone the sdk repository into your home folder
```
git clone https://github.com/boston-dynamics/spot-sdk.git
```
and install the contents
```
pip3 install --upgrade --find-links=~/spot-sdk/prebuilt bosdyn-client
```
Since the spot sdk is using Python3, we need to install some additional ros packages:
```
pip3 install rospkg catkin_pkg opencv-python
```
### hector_spot_ros
Clone hector_spot_ros into your catkin workspace
```
git clone [TBD]
```
Install the dependencies
```
rosdep install hector_spot_ros
```
and build your workspace.
## Getting started
Make sure, that your URDF follows the [ROS standard](https://www.ros.org/reps/rep-0105.html#base-link) and has a base frame named `base_link`, which is the body frame of spot. Instructions on how to pull an URDF directly from spot can be found below.
The node `spot_ros_interface_node.py` starts this driver. Make sure to provide at least a minimum launch configuration (spot host name, credentials, apptoken) via the parameter server. 
An example launch-configuration can be found [here (TODO)](TODO).
## Nodes
### spot_ros_interface_node.py
This is the main driver node which offers topic and service interfaces for all functionalities. It acquires a lease on startup, so starting this node multiple times will lead to conflicts. When the node is shut down (e.g. via Ctrl+C), the robot will go into a safe state before cutting motor power. Currently, it is not possible to cut motor power in an unsafe state using this driver.
If the driver loses connection, the robot will go into a safe state after a few seconds.
#### Subscribed Topics
* **`/cmd_vel_raw`** ([geometry_msgs/Twist])

    Set desired base velocity
* **`/set_body_orientation`** ([geometry_msgs/Vector3])

    Set desired body orientation in RPY-Euler angles.
* **`/set_body_pose`** ([geometry_msgs/Pose])

    Set desired body orientation and position. For the position, only the z-component is considered for body height.
* **`/set_body_twist`** ([geometry_msgs/Twist])

    Set the desired body twist (speed) in m/s. For the position, only the z-component is considered for body height.

#### Published Topics
* **`/joint_states`** ([sensor_msgs/JointStates])

    Robot joint state with position, velocity and effort for each joint.
* **`~battery_state`** ([sensor_msgs/BatteryState])

    Battery info of the built-in battery.
* **`/odom_kinematic`** ([nav_msgs/Odometry])

    Kinematic Odometry.
* **`/odom_vision`** ([nav_msgs/Odometry])

    Visual Odometry.
* **`/motor_enabled`** ([std_msgs/Bool])

    State of the motors. True, if motors are powered.
* **`/tf_static`** ([tf2_msgs/TFMessage])

    TF transforms for the camera optical frames relative to `base_link`. The transforms are automatically updated after performing calibration.
* **`/cameras/[camera_name]/image_rect`** ([sensor_msgs/Image])

    Rectified mono-fisheye and depth camera images. Fisheye images are published as `mono8` and depth images as `32FC1` in meters. The image topics are created dynamically based on available camera sources. By default, spot offers the following cameras: back_depth, back_fisheye_image, frontleft_depth, frontleft_fisheye_image, frontright_depth, frontright_fisheye_image, left_depth, left_fisheye_image, right_depth, right_fisheye_image.
* **`/cameras/[camera_name]/camera_info`** ([sensor_msgs/CameraInfo])

    Respective camera info.
* **`/estop/[estop_source]/pressed`** ([std_msgs/Bool])

    State of each E-Stop source. The sources are created dynamically based on available sources. True indicates, that the respective e-stop is triggered.
#### Services
* **`~power_on`** ([std_srvs/Empty])

    Turn motor power on. Blocking until powered on.
* **`~safe_power_off`** ([std_srvs/Empty])

    Spot sits down and the motor power is cut. Blocking until powered down.
* **`~selfright`** ([std_srvs/Empty])

    Self-right spot when lying on its back. Blocking until movement finished.
* **`~stand_up`** ([std_srvs/Empty])

    Command spot to stand up. Blocking until the robot is standing.
* **`~sit_down`** ([std_srvs/Empty])

    Command spot to sit down. Blocking until the robot is sitting.
* **`~set_mobility_params`** ([hector_spot_ros_msgs/SetMobilityParams])

    Sets spot mobility parameters like gate, maximum velocity or stair mode. See [MobilityParams] for reference.

#### Actions
* **`/controller/follow_path`** ([move_base_lite_msgs/FollowPathAction])

    Command spot to follow a path.
#### Parameters
* **`~hostname`** (string, mandatory)

    IP or host name of spot.
* **`~app_token_path`** (string, default: "~/.bosdyn/dev.app_token")

    File path of a valid application token (provided by BD support).
* **`~user`** (string, mandatory)

    Login credentials, user name.
* **`~password`** (string, mandatory)

    Login credentials, user password.
* **`~auto_power_on`** (bool, default: false)

    Automatically turn on motor power after successful connection.
* **`~[camera_source_name]_desired_rate`** (double, default: 10.0)

    Desired rate of the respective camera, e.g. `~left_depth_desired_rate`.
## Retrieving the robot URDF
TODO

## Bugs & Feature Requests
Submit bugs and feature requests using the Issue Tracker. Pull Requests are welcomed.
                  
[geometry_msgs/Twist]:http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
[geometry_msgs/Vector3]:http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html
[geometry_msgs/Pose]:http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
[sensor_msgs/JointStates]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html
[sensor_msgs/BatteryState]:http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html
[nav_msgs/Odometry]:http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
[std_msgs/Bool]:http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html
[tf2_msgs/TFMessage]:http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html
[sensor_msgs/Image]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CameraInfo]:http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.htmlhttp://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[std_srvs/Empty]:http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html
[hector_spot_ros_msgs/SetMobilityParams]
[MobilityParams]
[move_base_lite_msgs/FollowPathAction]:[https://github.com/tu-darmstadt-ros-pkg/move_base_lite/blob/hector_exploration/move_base_lite_msgs/action/FollowPath.action](https://github.com/tu-darmstadt-ros-pkg/move_base_lite/blob/hector_exploration/move_base_lite_msgs/action/FollowPath.action)

