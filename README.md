# hector_spot_ros
## Overview
This package provides a [ROS](http://www.ros.org/) interface to control the [Boston Dynamics Spot](https://www.bostondynamics.com/spot). It provides most functionalities of the [spot-sdk](https://github.com/boston-dynamics/spot-sdk) via topics, services and actions, e.g. power on/off, movement commands, robot state, sensor access and GraphNav functionality.

**Latest supported Spot SDK version: 3.0.3**

## Getting started
### Docker Install
Docker is the easiest way for installation. You will need the [Docker Engine](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/).

Clone the hector_spot_ros repository somewhere onto your disk
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_spot_ros
```
Copy the credentials example file and fill it with your spot hostname and login data
```
cd hector_spot_ros/hector_spot_ros/config/
cp spot-credentials-example.yaml spot-credentials.yaml
xdg-open spot-credentials.yaml
```
Go back to the root folder and build the docker image with
```
cd ../..
docker build . -t hector_spot_ros
```
Start the image in a container with
```
docker-compose up
```
You should now be able to see the provided interface on your local host. For details, see the "Nodes" section below
```
rostopic list
rosservice list
```
The provided Dockerfile is intended for development purposes. By default, the local hector_spot_ros folder is mounted in the container. This allows for development without the need to rebuild the image on every code change. The volume should be removed for production environments.

### Manual Install
First, install the [BD Spot SDK](https://github.com/boston-dynamics/spot-sdk). The following is a condensed version of [quickstart](https://dev.bostondynamics.com/docs/python/quickstart).

Install PIP3 by running
```
sudo apt-get install python3-pip
pip3 install --upgrade pip
```
Install the spot sdk via pip
```
python3 -m pip install bosdyn-client
```

Since the spot sdk is using Python3, we need to install some additional ros packages:
```
pip3 install rospkg catkin_pkg opencv-python
```
Clone hector_spot_ros into your catkin workspace
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_spot_ros
```
Copy the credentials example file and fill it with your spot hostname and login data
```
cd hector_spot_ros/hector_spot_ros/config/
cp spot-credentials-example.yaml spot-credentials.yaml
xdg-open spot-credentials.yaml
```
Install the dependencies
```
rosdep install hector_spot_ros
```
and build your workspace.
Start the driver in a new terminal
```
roslaunch hector_spot_ros spot.launch
```

## Nodes
### spot_ros_interface_node.py
This is the main driver node which offers topic, service and action interfaces for Spot's functionalities. It acquires a lease on startup, so starting this node multiple times will lead to conflicts. When the node is shut down (e.g. via Ctrl+C), the robot will go into a safe state before cutting motor power. Currently, it is not possible to cut motor power in an unsafe state using this driver.
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
    
* **`~map_to_odom_vision_tf`** and **`~map_to_odom_kinematic_tf`** ([tf2_msgs/TFMessage])

    Publishes a transform from the fixed GraphNav-frame to either vision or kinematic odometry. Remap the respective topic to `/tf` to use the transform. Only available if the robot is properly localized with [GraphNav](https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_service). 
    
* **`/cameras/[camera_name]/image_rect`** ([sensor_msgs/Image])

    Rectified mono-fisheye and depth camera images. Fisheye images are published as `mono8` and depth images as `32FC1` in meters. The image topics are created dynamically based on available camera sources. By default, spot offers the following cameras: back_depth, back_fisheye_image, frontleft_depth, frontleft_fisheye_image, frontright_depth, frontright_fisheye_image, left_depth, left_fisheye_image, right_depth, right_fisheye_image.
    
* **`/cameras/[camera_name]/camera_info`** ([sensor_msgs/CameraInfo])

    Respective camera info.

* **`/estop/[estop_source]/pressed`** ([std_msgs/Bool])

    State of each E-Stop source. The sources are created dynamically based on available sources. True indicates, that the respective e-stop is triggered.
    
* **`local_grid_map`** ([grid_map_msgs/GridMap])

    Grid-map representations of the environment created by Spot. Multiple layers are available for elevation, texture, no_step and obstacle distance.

* **`/graph_nav/cloud`** ([sensor_msgs/PointCloud2])

    Feature cloud of the environment created by Spot. This is only published once when the current GraphNav is downloaded. See `/graph_nav/download_full_graph`.
    
 * **`/graph_nav/topology`** ([visualization_msgs/MarkerArray])

    Visualization of the GraphNav graph. This is only published once when the current GraphNav is downloaded. See `/graph_nav/download_full_graph`.

 * **`/graph_nav/global_pose`** ([geometry_msgs/PoseStamped])

    Current pose of Spot's base link relative to the fixed GraphNav frame. Only available if the robot is properly localized with [GraphNav](https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_service). 

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

* **`/graph_nav/start_recording`** ([std_srvs/Empty])

    Starts recording a new GraphNav graph.

* **`/graph_nav/stop_recording`** ([std_srvs/Empty])

    Stops recording a new GraphNav graph.
* **`/graph_nav/download_full_graph`** ([hector_spot_ros_msgs/DownloadGraph])

    Downloads the current GraphNav graph to a specified file. If the respective parameters are set to `true`, feature cloud and graph topology are published upon download (see topics `/graph_nav/cloud` and `/graph_nav/topology`).
* **`/graph_nav/upload_graph`** ([hector_spot_ros_msgs/UploadGraph])

    Uploads a graph from a file to Spot. Spot won't localize automatically to the new graph.
* **`/graph_nav/create_waypoint`** ([hector_spot_ros_msgs/CreateGraphWaypoint])

    Forces the creation of a graph waypoint. The waypoint id is returned in the result and can be used for navigation.
* **`/graph_nav/clear_graph`** ([std_srvs/Empty])

    Clears the current graph.
* **`/graph_nav/set_localization`** ([hector_spot_ros_msgs/SetLocalization])

    Localizes Spot in the current GraphNav.

#### Actions
* **`~/follow_path`** ([move_base_lite_msgs/FollowPathAction])

    Command spot to follow a path.

* **`/graph_nav/navigate_to_waypoint`** ([hector_spot_ros_msgs/NavigateToWaypointAction])

    Navigates to the given waypoint ID in the current GraphNav. Spot needs to be properly localized.
#### Parameters
* **`~hostname`** (string, mandatory)

    IP or host name of spot.
    
* **`~user`** (string, mandatory)

    Login credentials, user name.
    
* **`~password`** (string, mandatory)

    Login credentials, user password.
    
* **`~auto_power_on`** (bool, default: false)

    Automatically turn on motor power after successful connection.
    
* **`~[camera_source_name]_desired_rate`** (double, default: 10.0)

    Desired rate of the respective camera, e.g. `~left_depth_desired_rate`.

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
[hector_spot_ros_msgs/SetMobilityParams]:hector_spot_ros_msgs/srv/SetMobilityParams.srv
[MobilityParams]:hector_spot_ros_msgs/msg/MobilityParams.msg
[move_base_lite_msgs/FollowPathAction]:https://github.com/tu-darmstadt-ros-pkg/move_base_lite/blob/hector_exploration/move_base_lite_msgs/action/FollowPath.action
[grid_map_msgs/GridMap]:https://docs.ros.org/en/melodic/api/grid_map_msgs/html/msg/GridMap.html
[visualization_msgs/MarkerArray]:http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/MarkerArray.html
[geometry_msgs/PoseStamped]:https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html
[hector_spot_ros_msgs/DownloadGraph]:hector_spot_ros_msgs/srv/DownloadGraph.srv
[hector_spot_ros_msgs/UploadGraph]:hector_spot_ros_msgs/srv/UploadGraph.srv
[hector_spot_ros_msgs/CreateGraphWaypoint]:hector_spot_ros_msgs/srv/CreateGraphWaypoint.srv
[hector_spot_ros_msgs/SetLocalization]:hector_spot_ros_msgs/srv/SetLocalization.srv
[hector_spot_ros_msgs/NavigateToWaypointAction]:hector_spot_ros_msgs/action/NavigateToWaypoint.action
