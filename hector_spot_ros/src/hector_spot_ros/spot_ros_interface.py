import rospy
import actionlib
import geometry_msgs.msg
import sensor_msgs.msg
import std_srvs.srv
import nav_msgs.msg
import std_msgs.msg
import tf2_msgs.msg
import move_base_lite_msgs.msg
import hector_spot_ros_msgs.srv
import hector_spot_ros_msgs.msg
import grid_map_msgs.msg
import visualization_msgs.msg

from hector_spot_ros.utils import proto_conversions
from hector_spot_ros.spot_driver import SpotDriver
from hector_spot_ros.utils import utils, visualization
from hector_spot_ros import cv_bridge

from bosdyn.client.frame_helpers import get_vision_tform_body, get_odom_tform_body
from bosdyn.client.math_helpers import *

BODY_TWIST_DESIRED_PERIOD = 0.1


class CallbackSubscribeListener(rospy.SubscribeListener):
    def __init__(self, callback):
        super(CallbackSubscribeListener, self).__init__()
        self._callback = callback

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self._callback()

    def peer_unsubscribe(self, topic_name, num_peers):
        self._callback()


class SpotROSInterface:
    def __init__(self):
        # Load parameters
        config = utils.load_config_from_ns("~")
        self._allow_power_on_with_shore_power = rospy.get_param("~allow_power_on_with_shore_power", False)
        self._publish_identity_if_not_localized = rospy.get_param("~publish_identity_if_not_localized", True)

        # Initialize members
        # Body pose
        self._body_height = 0
        self._body_roll = 0
        self._body_pitch = 0
        self._body_yaw = 0

        self._body_height_vel = 0
        self._body_roll_vel = 0
        self._body_pitch_vel = 0
        self._body_yaw_vel = 0

        self._cv_bridge = cv_bridge.CvBridge()

        # Status
        self._connected_to_shore_power = True  # assuming true until first message, for safety

        # GraphNav
        self._map_to_odom_transform = None
        self._map_to_vision_transform = None

        # Connect to robot
        self._driver = SpotDriver()
        if not self._driver.connect(config):
            exit(0)
        self._driver.start(power_on=config.auto_power_on)

        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel_raw", geometry_msgs.msg.Twist, self.cmd_vel_callback,
                                            queue_size=10)
        self.body_orientation_sub = rospy.Subscriber("/set_body_orientation", geometry_msgs.msg.Vector3,
                                                     self.set_body_orientation_callback, queue_size=10)  # TODO service?
        self.body_pose_sub = rospy.Subscriber("/set_body_pose", geometry_msgs.msg.Pose, self.set_body_pose_callback,
                                              queue_size=10)  # TODO service?
        self.body_twist_sub = rospy.Subscriber("/set_body_twist", geometry_msgs.msg.Twist, self.set_body_twist_callback,
                                               queue_size=10)

        # Publishers
        self.joint_state_pub = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState, queue_size=10)
        self.battery_state_pub = rospy.Publisher("~battery_state", sensor_msgs.msg.BatteryState, queue_size=10)
        self.kinematic_odom_pub = rospy.Publisher("/odom_kinematic", nav_msgs.msg.Odometry, queue_size=10)
        self.vision_odom_pub = rospy.Publisher("/odom_vision", nav_msgs.msg.Odometry, queue_size=10)
        self.motor_enabled_pub = rospy.Publisher("/motor_enabled", std_msgs.msg.Bool, queue_size=10, latch=True)
        self.motor_enabled = None
        self.tf_static_pub = rospy.Publisher("/tf_static", tf2_msgs.msg.TFMessage, queue_size=100, latch=True)
        self.map_to_odom_vision_tf_pub = rospy.Publisher("~map_to_odom_vision_tf", tf2_msgs.msg.TFMessage, queue_size=100)
        self.map_to_odom_kinematic_tf_pub = rospy.Publisher("~map_to_odom_kinematic_tf", tf2_msgs.msg.TFMessage, queue_size=100)

        # Image publishers
        image_sources = self._driver.camera.list_image_sources()
        self._requested_image_sources = []
        self._driver.camera.set_requested_image_sources(self._requested_image_sources)
        self.image_pubs = dict()
        self.camera_info_pubs = dict()
        self._image_subscribe_listener = CallbackSubscribeListener(self.image_subscriber_callback)
        self.camera_transforms = dict()
        for image_source in image_sources:
            self.image_pubs[image_source] = rospy.Publisher("/cameras/" + image_source + "/image_rect",
                                                            sensor_msgs.msg.Image,
                                                            subscriber_listener=self._image_subscribe_listener,
                                                            queue_size=10)
            self.camera_info_pubs[image_source] = rospy.Publisher("/cameras/" + image_source + "/camera_info",
                                                                  sensor_msgs.msg.CameraInfo,
                                                                  queue_size=10)
            self.camera_transforms[image_source] = None

            # Set desired cam frequency
            desired_rate = rospy.get_param("~" + image_source + "_desired_rate", self._driver.camera.DEFAULT_CAM_RATE)
            self._driver.camera.set_image_rate(image_source, desired_rate)

        # Estop state publishers
        estop_sources = self._driver.state.list_estop_sources()
        self.estop_pressed_pubs = dict()
        self.estop_pressed = dict()
        for source in estop_sources:
            self.estop_pressed_pubs[source] = rospy.Publisher("/estop/" + source + "/pressed", std_msgs.msg.Bool,
                                                              queue_size=10, latch=True)
            self.estop_pressed[source] = None

        # Grid map publisher
        self.grid_map_types = self._driver.state.get_local_grid_types()
        self.grid_map_pub = rospy.Publisher("local_grid_map",
                                            grid_map_msgs.msg.GridMap,
                                            subscriber_listener=CallbackSubscribeListener(self.grid_map_subscriber_callback),
                                            queue_size=10)
        # Graph nav publisher
        self.graph_nav_cloud_pub = rospy.Publisher("/graph_nav/cloud", sensor_msgs.msg.PointCloud2, queue_size=10, latch=True)
        self.graph_nav_topology_pub = rospy.Publisher("/graph_nav/topology", visualization_msgs.msg.MarkerArray, queue_size=10, latch=True)
        self.graph_nav_localization_pub = rospy.Publisher("/graph_nav/global_pose", geometry_msgs.msg.PoseStamped,
                                                          queue_size=10)

        # Service Servers
        self.power_on_cmd_server = rospy.Service("~power_on", std_srvs.srv.Empty, self.power_on_callback)
        self.safe_power_off_cmd_server = rospy.Service("~safe_power_off", std_srvs.srv.Empty,
                                                       self.safe_power_off_callback)
        self.selfright_cmd_server = rospy.Service("~selfright", std_srvs.srv.Empty, self.selfright_cmd_callback)
        self.stand_up_cmd_server = rospy.Service("~stand_up", std_srvs.srv.Empty, self.stand_up_cmd_callback)
        self.sit_down_cmd_server = rospy.Service("~sit_down", std_srvs.srv.Empty, self.sit_down_cmd_callback)
        self.set_mobility_params_server = rospy.Service("~set_mobility_params", hector_spot_ros_msgs.srv.SetMobilityParams,
                                                        self.set_mobility_params)
        self.start_graph_recording_server = rospy.Service("/graph_nav/start_recording", std_srvs.srv.Empty, self.start_graph_recording_callback)
        self.stop_graph_recording_server = rospy.Service("/graph_nav/stop_recording", std_srvs.srv.Empty, self.stop_graph_recording_callback)
        self.download_full_graph_server = rospy.Service("/graph_nav/download_full_graph", hector_spot_ros_msgs.srv.DownloadGraph, self.download_full_graph_callback)
        self.upload_graph_server = rospy.Service("/graph_nav/upload_graph", hector_spot_ros_msgs.srv.UploadGraph, self.upload_graph_callback)
        self.create_waypoint_server = rospy.Service("/graph_nav/create_waypoint", hector_spot_ros_msgs.srv.CreateGraphWaypoint, self.create_waypoint_callback)
        self.clear_graph_server = rospy.Service("/graph_nav/clear_graph", std_srvs.srv.Empty, self.clear_graph_callback)
        self.set_localization_server = rospy.Service("/graph_nav/set_localization", hector_spot_ros_msgs.srv.SetLocalization, self.set_localization_callback)

        # Action Server
        self.follow_path_as = actionlib.SimpleActionServer("/controller/follow_path",
                                                           move_base_lite_msgs.msg.FollowPathAction,
                                                           # execute_cb=self.follow_path_execute_callback,
                                                           auto_start=False)
        self.follow_path_as.register_goal_callback(self.follow_path_execute_callback)
        self.follow_path_as.register_preempt_callback(self.follow_path_preempt_callback)
        self.follow_path_as.start()

        self.navigate_to_waypoint_as = actionlib.SimpleActionServer("/graph_nav/navigate_to_waypoint",
                                                                    hector_spot_ros_msgs.msg.NavigateToWaypointAction,
                                                                    auto_start=False)
        self.navigate_to_waypoint_as.register_goal_callback(self.navigate_to_waypoint_goal_callback)
        self.navigate_to_waypoint_as.register_preempt_callback(self.navigate_to_waypoint_preempt_callback)
        self.navigate_to_waypoint_as.start()

        # Register periodic query callbacks
        self._driver.camera.register_image_callback(self.camera_response_callback)
        self._driver.state.register_robot_metrics_callback(self.robot_metrics_callback)
        self._driver.state.register_robot_state_callback(self.robot_state_callback)
        self._driver.state.register_local_grid_callback(self.grid_map_callback)
        self._driver.graph_nav.periodic_localization_state.register_callback(self.localization_state_callback)
        self._driver.graph_nav.periodic_localization_state.enabled = True

        # Periodic body pose
        self._body_pose_timer = None
        self._periods_without_new_body_twist = 0

        # Publish camera transforms once
        for image_source in image_sources:
            images = self._driver.camera.get_images([image_source], timeout=None)
            self.publish_camera_transforms(images)

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._driver.stop()

    def power_on_callback(self, _):
        self.reset_body_pose()
        if self._connected_to_shore_power and not self._allow_power_on_with_shore_power:
            rospy.logerr("Can't power on. Connected to shore power.")
            return std_srvs.srv.EmptyResponse()
        ret = utils.try_cmd("power_on", self._driver.power_on)
        if not ret[1]:
            return None
        return std_srvs.srv.EmptyResponse()

    def safe_power_off_callback(self, _):
        self.reset_body_pose()
        self.abort_action_servers("Robot will be shut down")
        ret = utils.try_cmd("safe_power_off", self._driver.safe_power_off)
        if not ret[1]:
            return None
        return std_srvs.srv.EmptyResponse()

    def set_mobility_params(self, request):
        mobility_params = proto_conversions.mobility_params_msg_to_proto(request.mobility_params)
        self._driver.movement.mobility_params = mobility_params
        return hector_spot_ros_msgs.srv.SetMobilityParamsResponse(success=True)

    def selfright_cmd_callback(self, _):
        self.reset_body_pose()
        ret = utils.try_cmd("selfright", self._driver.movement.selfright)
        if not ret[1]:
            return None
        return std_srvs.srv.EmptyResponse()

    def stand_up_cmd_callback(self, _):
        self.reset_body_pose()
        ret = utils.try_cmd("stand_up", self._driver.movement.stand_up)
        if not ret[1]:
            return None
        return std_srvs.srv.EmptyResponse()

    def sit_down_cmd_callback(self, _):
        self.reset_body_pose()
        ret = utils.try_cmd("sit_down", self._driver.movement.sit_down)
        if not ret[1]:
            return None
        return std_srvs.srv.EmptyResponse()

    def follow_path_execute_callback(self):
        goal = self.follow_path_as.accept_new_goal()
        trajectory = proto_conversions.path_msg_to_proto(goal.target_path)
        desired_speed = goal.follow_path_options.desired_speed
        if desired_speed == 0.0:
            desired_speed = 0.5
        ret = utils.try_cmd("follow_path",
                            self._driver.movement.follow_trajectory_command,
                            [trajectory, desired_speed, self.follow_path_finished_callback])
        if not ret[1]:
            self.follow_path_as.set_aborted()

    def follow_path_preempt_callback(self):
        # print("PREEMPT CALLED")
        self._driver.movement.cancel_follow_trajectory_command()
        self.follow_path_as.set_preempted()

    def follow_path_finished_callback(self, success):
        # print("TRAJECTORY FINISHED", success)
        result = move_base_lite_msgs.msg.FollowPathResult()
        if success:
            result.result.val = move_base_lite_msgs.msg.ErrorCodes.SUCCESS
        else:
            result.result.val = move_base_lite_msgs.msg.ErrorCodes.FAILURE
        self.follow_path_as.set_succeeded(result)

    def abort_action_servers(self, reason="Aborted"):
        if self.follow_path_as.is_active():
            self._driver.movement.cancel_follow_trajectory_command()
            result = move_base_lite_msgs.msg.FollowPathResult()
            result.result.val = move_base_lite_msgs.msg.ErrorCodes.FAILURE
            self.follow_path_as.set_aborted(result, reason)
        if self.navigate_to_waypoint_as.is_active():
            self._driver.graph_nav.cancel_navigation()
            result = hector_spot_ros_msgs.msg.NavigateToWaypointResult()
            result.result = reason
            self.navigate_to_waypoint_as.set_aborted(result, reason)

    def cmd_vel_callback(self, twist_msg):
        """

        :type twist_msg: geometry_msgs.msg.Twist
        """
        self.reset_body_pose()
        self.abort_action_servers("Interrupted by user control input.")
        ret = utils.try_cmd("cmd_vel", self._driver.movement.send_velocity_command, [twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z])
        if ret is None:
            self.follow_path_as.set_aborted()

    def set_body_orientation_callback(self, vec_msg):
        """
        :type vec_msg: geometry_msgs.msg.Vector3

        """
        roll, pitch, yaw = [vec_msg.x, vec_msg.y, vec_msg.z]
        self.set_body_pose(roll, pitch, yaw, self._body_height)

    def set_body_pose_callback(self, pose_msg):
        """

        :type pose_msg: geometry_msgs.msg.Pose
        """
        height = pose_msg.position.z
        quaternion = proto_conversions.quaternion_msg_to_proto(pose_msg.orientation)
        yaw, pitch, roll = utils.quaternion_to_ypr(quaternion)
        self.set_body_pose(roll, pitch, yaw, height)

    def set_body_twist_callback(self, twist_msg):
        """

        :type twist_msg: geometry_msgs.msg.Twist
        """
        self._periods_without_new_body_twist = 0
        self._body_height_vel = twist_msg.linear.z
        self._body_roll_vel = twist_msg.angular.x
        self._body_pitch_vel = twist_msg.angular.y
        self._body_yaw_vel = twist_msg.angular.z

        # Start timer
        if self._body_pose_timer is None:
            self._body_pose_timer = rospy.Timer(rospy.Duration().from_sec(BODY_TWIST_DESIRED_PERIOD), self.update_body_twist_command)

    def update_body_twist_command(self, timer_event):
        """

        :type timer_event: rospy.TimerEvent
        """
        self._periods_without_new_body_twist += 1
        if self._periods_without_new_body_twist > 4:
            return
        if timer_event.last_real:
            dt = (timer_event.last_real - rospy.Time.now()).to_sec()
        else:
            dt = BODY_TWIST_DESIRED_PERIOD
        height = self._body_height + self._body_height_vel * dt
        roll = self._body_roll + self._body_roll_vel * dt
        pitch = self._body_pitch + self._body_pitch_vel * dt
        yaw = self._body_yaw + self._body_yaw_vel * dt
        self.set_body_pose(roll, pitch, yaw, height)

    def set_body_pose(self, roll, pitch, yaw, height):
        self._body_height = utils.clamp(height, self._driver.movement.HEIGHT_MAX)
        self._body_roll = utils.clamp(roll, self._driver.movement.ORIENTATION_ROLL_MAX)
        self._body_pitch = utils.clamp(pitch, self._driver.movement.ORIENTATION_PITCH_MAX)
        self._body_yaw = utils.clamp(yaw, self._driver.movement.ORIENTATION_YAW_MAX)

        # print("body pose", "roll", self._body_roll, "pitch", self._body_pitch, "yaw", self._body_yaw)
        utils.try_cmd("body_pose", self._driver.movement.send_body_pose_ypr,
                            [self._body_yaw, self._body_pitch, self._body_roll, self._body_height])

    def reset_body_pose(self):
        if self._body_pose_timer is not None:
            self._body_pose_timer.shutdown()
            self._body_pose_timer = None
        self._body_height = 0
        self._body_roll = 0
        self._body_pitch = 0
        self._body_yaw = 0

    def camera_response_callback(self, images):
        self.publish_camera_images(images)
        self.publish_camera_infos(images)
        self.publish_camera_transforms(images)

    def publish_camera_images(self, images):
        for stamp, image, transform, source in images:
            # Convert image to sensor msg
            image_msg = self._cv_bridge.cv2_to_imgmsg(image)

            # Fill header
            image_msg.header.frame_id = source.name + "_optical_frame"
            image_msg.header.stamp = proto_conversions.timestamp_proto_to_ros_time(stamp, self._driver.get_clock_skew())

            self.image_pubs[source.name].publish(image_msg)

    def publish_camera_infos(self, images):
        for stamp, image, transform, source in images:
            # Convert camera info and publish
            camera_info_msg = proto_conversions.camera_info_proto_to_msg(stamp,
                                                                         source,
                                                                         self._driver.get_clock_skew())
            self.camera_info_pubs[source.name].publish(camera_info_msg)

    def publish_camera_transforms(self, images):
        camera_frame_changed = False
        for stamp, image, transform, source in images:
            # check if frame changed
            new_transform = proto_conversions.transform_stamped_proto_to_msg(stamp,
                                                                             "base_link",
                                                                             source.name + "_optical_frame",
                                                                             transform,
                                                                             self._driver.get_clock_skew())
            old_transform = self.camera_transforms[source.name]
            if old_transform is None or old_transform.transform != new_transform.transform:
                camera_frame_changed = True
                self.camera_transforms[source.name] = new_transform

        if camera_frame_changed:
            tf_msg = tf2_msgs.msg.TFMessage()
            for source_name, transform in self.camera_transforms.items():
                if transform is not None:
                    tf_msg.transforms.append(transform)
            self.tf_static_pub.publish(tf_msg)

    def image_subscriber_callback(self):
        self._requested_image_sources = []
        for source, pub in self.image_pubs.items():
            if pub.get_num_connections() > 0:
                self._requested_image_sources.append(source)
        self._driver.camera.set_requested_image_sources(self._requested_image_sources)

    def robot_state_callback(self, robot_state):
        self.update_shore_power_status(robot_state)
        self.publish_motor_enabled(robot_state)
        self.publish_estop_pressed(robot_state)
        self.publish_battery_state(robot_state)
        self.publish_joint_states(robot_state)
        self.publish_kinematics_odometry(robot_state)
        self.publish_visual_odometry(robot_state)

    def update_shore_power_status(self, robot_state):
        self._connected_to_shore_power = proto_conversions.is_connected_to_shore(robot_state.power_state.shore_power_state)

    def publish_battery_state(self, robot_state):
        self.battery_state_pub.publish(
            proto_conversions.battery_state_proto_to_msg(robot_state.battery_states,
                                                         self._driver.get_clock_skew()))

    def publish_motor_enabled(self, robot_state):
        new_motor_enabled = utils.is_motor_enabled(robot_state.power_state.motor_power_state)
        if self.motor_enabled is None or new_motor_enabled != self.motor_enabled:
            bool_msg = std_msgs.msg.Bool()
            bool_msg.data = new_motor_enabled
            self.motor_enabled_pub.publish(bool_msg)
            self.motor_enabled = new_motor_enabled

    def publish_estop_pressed(self, robot_state):
        for state in robot_state.estop_states:
            new_estop_pressed = utils.is_estop_pressed(state.state)
            old_state = self.estop_pressed[state.name]
            if old_state is None or old_state != new_estop_pressed:
                bool_msg = std_msgs.msg.Bool()
                bool_msg.data = new_estop_pressed
                self.estop_pressed_pubs[state.name].publish(bool_msg)
                self.estop_pressed[state.name] = new_estop_pressed

    def publish_kinematics_odometry(self, robot_state):
        timestamp = robot_state.kinematic_state.acquisition_timestamp
        robot_pose = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)
        robot_velocity = robot_state.kinematic_state.velocity_of_body_in_odom

        odom_msg = nav_msgs.msg.Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.header.stamp = proto_conversions.timestamp_proto_to_ros_time(timestamp, self._driver.get_clock_skew())

        odom_msg.pose = proto_conversions.pose_with_covariance_proto_to_msg(robot_pose)
        odom_msg.twist = proto_conversions.twist_with_covariance_proto_to_msg(robot_velocity)
        self.kinematic_odom_pub.publish(odom_msg)
        # Publish map -> odom tf msg
        if self._map_to_odom_transform:
            self.publish_map_to_odom(self.map_to_odom_kinematic_tf_pub, self._map_to_odom_transform, odom_msg.header.stamp)

    def publish_visual_odometry(self, robot_state):
        timestamp = robot_state.kinematic_state.acquisition_timestamp
        robot_pose = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)
        robot_velocity = robot_state.kinematic_state.velocity_of_body_in_vision

        odom_msg = nav_msgs.msg.Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.header.stamp = proto_conversions.timestamp_proto_to_ros_time(timestamp, self._driver.get_clock_skew())

        odom_msg.pose = proto_conversions.pose_with_covariance_proto_to_msg(robot_pose)
        odom_msg.twist = proto_conversions.twist_with_covariance_proto_to_msg(robot_velocity)
        self.vision_odom_pub.publish(odom_msg)
        # Publish map -> odom tf msg
        if self._map_to_vision_transform:
            self.publish_map_to_odom(self.map_to_odom_vision_tf_pub, self._map_to_vision_transform, odom_msg.header.stamp)

    def publish_joint_states(self, robot_state):
        timestamp = robot_state.kinematic_state.acquisition_timestamp
        joint_state_msg = sensor_msgs.msg.JointState()
        joint_state_msg.header.stamp = proto_conversions.timestamp_proto_to_ros_time(timestamp, self._driver.get_clock_skew())
        for joint_state in robot_state.kinematic_state.joint_states:
            joint_state_msg.name.append(joint_state.name)
            joint_state_msg.position.append(joint_state.position.value)
            joint_state_msg.velocity.append(joint_state.velocity.value)
            joint_state_msg.effort.append(joint_state.load.value)
        self.joint_state_pub.publish(joint_state_msg)

    def publish_robot_info(self):
        pass

    def robot_metrics_callback(self, robot_metrics):
        pass

    def grid_map_subscriber_callback(self):
        sub_count = self.grid_map_pub.get_num_connections()
        if sub_count <= 0:
            self._driver.state.set_requested_grid_types([])
        else:
            self._driver.state.set_requested_grid_types(self.grid_map_types)

    def grid_map_callback(self, grid_map_responses):
        grid_map_msg = grid_map_msgs.msg.GridMap()
        # Find terrain mask if available
        mask_proto = next((local_grid_response.local_grid for local_grid_response in grid_map_responses
                           if local_grid_response.local_grid_type_name == "terrain_valid"), None)
        for local_grid_response in grid_map_responses:
            # Skip terrain mask
            if local_grid_response.local_grid.local_grid_type_name == "terrain_valid":
                continue
            mask = None
            if local_grid_response.local_grid.local_grid_type_name == "terrain":
                mask = mask_proto
            proto_conversions.grid_map_proto_to_msg(local_grid_response.local_grid,
                                                    self._driver.get_clock_skew(),
                                                    grid_map_msg,
                                                    mask_proto=mask)
        self.grid_map_pub.publish(grid_map_msg)

    def start_graph_recording_callback(self, _):
        success = self._driver.graph_nav.start_recording()
        if success:
            return std_srvs.srv.EmptyResponse()
        else:
            return None

    def stop_graph_recording_callback(self, _):
        success = self._driver.graph_nav.stop_recording()
        if success:
            return std_srvs.srv.EmptyResponse()
        else:
            return None

    def download_full_graph_callback(self, request):
        """

        :type request: hector_spot_ros_msgs.srv.DownloadGraphRequest
        """
        # Download graph from robot
        graph, waypoint_snapshot_dict, edge_snapshot_dict = self._driver.graph_nav.download_graph()
        if graph is None:
            response = hector_spot_ros_msgs.srv.DownloadGraphResponse()
            response.success = False
            response.message = "Failed to download graph"
            return response

        # If desired, publish to ROS
        if request.publish_data:
            global_graph = self._driver.graph_nav.convert_graph_to_global_frame(graph, waypoint_snapshot_dict,
                                                                                edge_snapshot_dict)
            # Topology
            self.publish_graph_topology(global_graph)
            # Feature cloud
            cloud_msg = visualization.waypoints_to_point_cloud_msg(global_graph.waypoints)
            self.graph_nav_cloud_pub.publish(cloud_msg)

        # If desired, save to disk
        if len(request.save_file_path) > 0:
            success = self._driver.graph_nav.save_graph_to_file(graph, waypoint_snapshot_dict, edge_snapshot_dict,
                                                                request.save_file_path)
            if not success:
                response = hector_spot_ros_msgs.srv.DownloadGraphResponse()
                response.success = False
                response.message = "Failed to save graph to disk"
                return response

        return hector_spot_ros_msgs.srv.DownloadGraphResponse(success=True)

    def publish_graph_topology(self, graph):
        # Clear all markers
        visualization.clear_all_markers(self.graph_nav_topology_pub)
        marker_array = visualization.graph_topology_to_visualization_msg(graph)
        self.graph_nav_topology_pub.publish(marker_array)

    def upload_graph_callback(self, request):
        """

        :type request: hector_spot_ros_msgs.srv.UploadGraphRequest
        """
        graph_data = self._driver.graph_nav.load_graph_from_file(request.load_file_path)
        self._driver.graph_nav.upload_graph(*graph_data)
        response = hector_spot_ros_msgs.srv.UploadGraphResponse(success=True)
        return response

    def create_waypoint_callback(self, request):
        resp = self._driver.graph_nav.create_waypoint(request.waypoint_name)
        if resp:
            return hector_spot_ros_msgs.srv.CreateGraphWaypointResponse(waypoint_id=resp.created_waypoint.id)
        else:
            return None

    def clear_graph_callback(self, _):
        self._driver.graph_nav.clear_graph()
        return std_srvs.srv.EmptyResponse()

    def localization_state_callback(self, state):
        localized = state is not None
        if localized:
            map_to_base_transform, response_proto, graph = state
            # Publish map -> base_link transform as pose
            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.header.frame_id = "world"
            pose_msg.header.stamp = proto_conversions.timestamp_proto_to_ros_time(response_proto.localization.timestamp,
                                                                                  clock_skew=self._driver.get_clock_skew())
            pose_msg.pose = proto_conversions.pose_proto_to_msg(map_to_base_transform)
            self.graph_nav_localization_pub.publish(pose_msg)

            odom_vision = get_vision_tform_body(response_proto.robot_kinematics.transforms_snapshot)
            odom_kinematic = get_odom_tform_body(response_proto.robot_kinematics.transforms_snapshot)
        else:
            map_to_base_transform = None
            odom_vision = None
            odom_kinematic = None

        self._map_to_odom_transform = self._compute_map_to_odom(map_to_base_transform, odom_kinematic)
        self._map_to_vision_transform = self._compute_map_to_odom(map_to_base_transform, odom_vision)

    def _compute_map_to_odom(self, map_to_base, odom_to_base):
        if map_to_base is not None:
            map_to_odom = map_to_base * odom_to_base.inverse()
        elif self._publish_identity_if_not_localized:
            map_to_odom = SE3Pose.from_identity()
        else:
            map_to_odom = None

        return map_to_odom

    def publish_map_to_odom(self, pub, map_to_odom, stamp):
        # Publish map -> odom transform as tf message
        transform_stamped_msg = geometry_msgs.msg.TransformStamped()
        transform_stamped_msg.header.frame_id = "world"
        transform_stamped_msg.header.stamp = stamp
        transform_stamped_msg.child_frame_id = "odom"
        transform_stamped_msg.transform = proto_conversions.transform_proto_to_msg(map_to_odom)
        tf_msg = tf2_msgs.msg.TFMessage()
        tf_msg.transforms.append(transform_stamped_msg)
        pub.publish(tf_msg)

    def set_localization_callback(self, request):
        """

        :type request: hector_spot_ros_msgs.srv.SetLocalizationRequest
        """
        initial_guess_proto = proto_conversions.localization_msg_to_proto(request.initial_guess,
                                                                          self._driver.get_clock_skew())

        result = self._driver.graph_nav.set_localization(initial_guess_proto, request.max_distance, request.max_yaw,
                                                         request.fiducial_init, request.use_fiducial_id,
                                                         request.refine_fiducial_result_with_icp,
                                                         request.do_ambiguity_check)
        response = hector_spot_ros_msgs.srv.SetLocalizationResponse()
        response.message = ""
        response.success = True
        return response

    def navigate_to_waypoint_goal_callback(self):
        goal = self.navigate_to_waypoint_as.accept_new_goal()
        rospy.loginfo("Navigating to waypoint '{}'".format(goal.waypoint_id))
        self._driver.graph_nav.navigate_to_waypoint(goal.waypoint_id,
                                                    velocity_limit=[goal.linear_velocity_limit,
                                                                    goal.angular_velocity_limit],
                                                    finished_threshold=[goal.max_distance,
                                                                        goal.max_yaw],
                                                    async_execution=True,
                                                    finished_cb=self.navigate_to_waypoint_finished_callback)

    def navigate_to_waypoint_finished_callback(self, success):
        rospy.loginfo("Navigation finished with result: {}".format("Success" if success else "Failed"))
        result = hector_spot_ros_msgs.msg.NavigateToWaypointResult()
        if success:
            result.result = "Success"
            self.navigate_to_waypoint_as.set_succeeded(result)
        else:
            result.result = "Failure"
            self.navigate_to_waypoint_as.set_aborted(result)

    def navigate_to_waypoint_preempt_callback(self):
        rospy.loginfo("Preempting navigation to waypoint")
        self._driver.graph_nav.cancel_navigation()
        self._driver.movement.stand_up()
        self.navigate_to_waypoint_as.set_preempted()



