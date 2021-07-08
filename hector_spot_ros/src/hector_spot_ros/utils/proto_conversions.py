from __future__ import division
import numpy as np

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import grid_map_msgs.msg
import std_msgs.msg

import bosdyn.util
from bosdyn.api import robot_state_pb2, geometry_pb2, trajectory_pb2, local_grid_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from google.protobuf.timestamp_pb2 import Timestamp
from bosdyn.client.frame_helpers import get_a_tform_b, BODY_FRAME_NAME, VISION_FRAME_NAME

from hector_spot_ros.utils.utils import convert_timestamp_from_robot_to_local, convert_timestamp_from_local_to_robot, \
    quaternion_to_ypr


def is_connected_to_shore(shore_power_state):
    return shore_power_state != robot_state_pb2.PowerState.STATE_OFF_SHORE_POWER


def battery_status_proto_to_msg(battery_status):
    if battery_status == robot_state_pb2.BatteryState.STATUS_CHARGING:
        return sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_CHARGING
    if battery_status == robot_state_pb2.BatteryState.STATUS_DISCHARGING:
        return sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    if battery_status == robot_state_pb2.BatteryState.STATUS_BOOTING or \
            battery_status == robot_state_pb2.BatteryState.STATUS_MISSING or \
            battery_status == robot_state_pb2.BatteryState.STATUS_UNKNOWN:
        return sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    # Unknown state
    return sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN


def pose_with_covariance_proto_to_msg(se3_pose):
    pose_msg = geometry_msgs.msg.PoseWithCovariance()
    pose_msg.pose = pose_proto_to_msg(se3_pose)
    for i in range(0, 6):
        pose_msg.covariance[i * 6 + i] = 0.001
    return pose_msg


def pose_proto_to_msg(se3_pose):
    pose_msg = geometry_msgs.msg.Pose()
    pose_msg.position = point_proto_to_msg(se3_pose.position)
    pose_msg.orientation = quaternion_proto_to_msg(se3_pose.rotation)
    return pose_msg


def pose_msg_to_proto(pose_msg):
    """

    :type pose_msg: geometry_msgs.msg.Pose
    """
    se3_pose = geometry_pb2.SE3Pose(position=vec3_msg_to_proto(pose_msg.position),
                                    rotation=quaternion_msg_to_proto(pose_msg.orientation))
    return se3_pose


def transform_proto_to_msg(se3_pose):
    transform_msg = geometry_msgs.msg.Transform()
    transform_msg.translation = vec3_proto_to_msg(se3_pose.position)
    transform_msg.rotation = quaternion_proto_to_msg(se3_pose.rotation)
    return transform_msg


def twist_with_covariance_proto_to_msg(se3_velocity):
    twist_msg = geometry_msgs.msg.TwistWithCovariance()
    twist_msg.twist = twist_proto_to_msg(se3_velocity)
    for i in range(0, 6):
        twist_msg.covariance[i * 6 + i] = 0.001
    return twist_msg


def twist_proto_to_msg(se3_velocity):
    twist_msg = geometry_msgs.msg.Twist()
    twist_msg.linear = vec3_proto_to_msg(se3_velocity.linear)
    twist_msg.angular = vec3_proto_to_msg(se3_velocity.angular)
    return twist_msg


def vec3_proto_to_msg(vec3):
    vec3_msg = geometry_msgs.msg.Vector3()
    vec3_msg.x = vec3.x
    vec3_msg.y = vec3.y
    vec3_msg.z = vec3.z
    return vec3_msg


def vec3_msg_to_proto(vec3_msg):
    vec3_proto = geometry_pb2.Vec3()
    vec3_proto.x = vec3_msg.x
    vec3_proto.z = vec3_msg.y
    vec3_proto.y = vec3_msg.z
    return vec3_proto


def point_proto_to_msg(vec3):
    point_msg = geometry_msgs.msg.Point()
    point_msg.x = vec3.x
    point_msg.y = vec3.y
    point_msg.z = vec3.z
    return point_msg


def quaternion_proto_to_msg(quaternion):
    quaternion_msg = geometry_msgs.msg.Quaternion()
    quaternion_msg.x = quaternion.x
    quaternion_msg.y = quaternion.y
    quaternion_msg.z = quaternion.z
    quaternion_msg.w = quaternion.w
    return quaternion_msg


def quaternion_msg_to_proto(quaternion_msg):
    quaternion = geometry_pb2.Quaternion()
    quaternion.x = quaternion_msg.x
    quaternion.y = quaternion_msg.y
    quaternion.z = quaternion_msg.z
    quaternion.w = quaternion_msg.w
    return quaternion


def timestamp_proto_to_ros_time(timestamp, clock_skew):
    local_timestamp = convert_timestamp_from_robot_to_local(timestamp, clock_skew)
    return rospy.Time(secs=local_timestamp.seconds, nsecs=local_timestamp.nanos)


def timestamp_ros_time_to_proto(ros_time, clock_skew):
    local_timestamp = Timestamp(seconds=ros_time.secs, nanos=ros_time.nsecs)
    robot_timestamp = convert_timestamp_from_local_to_robot(local_timestamp, clock_skew)
    return robot_timestamp


def camera_info_proto_to_msg(timestamp, source, clock_skew):
    # Extract intrinsic model
    fx = source.pinhole.intrinsics.focal_length.x
    fy = source.pinhole.intrinsics.focal_length.y
    cx = source.pinhole.intrinsics.principal_point.x
    cy = source.pinhole.intrinsics.principal_point.y
    sx = source.pinhole.intrinsics.skew.x
    sy = source.pinhole.intrinsics.skew.y

    # Create camera info message
    info_msg = sensor_msgs.msg.CameraInfo()
    info_msg.header.frame_id = source.name + "_optical_frame"
    info_msg.header.stamp = timestamp_proto_to_ros_time(timestamp, clock_skew)

    info_msg.width = source.cols
    info_msg.height = source.rows
    info_msg.K = [fx, sx, cx,
                  sy, fy, cy,
                  0, 0, 1]
    info_msg.P = info_msg.K[0:3] + [0] + info_msg.K[3:6] + [0] + info_msg.K[6:9] + [0]

    return info_msg


def battery_state_proto_to_msg(battery_states, clock_skew):
    battery_state_msg = sensor_msgs.msg.BatteryState()
    battery_state_msg.header.stamp = rospy.Time.now()  # If there is no battery, we do not get a proper timestamp

    if len(battery_states) != 0:
        battery_state = battery_states[0]
        battery_state_msg.header.stamp = timestamp_proto_to_ros_time(battery_state.timestamp, clock_skew)
        battery_state_msg.voltage = battery_state.voltage.value  # Voltage in Volts (Mandatory)
        battery_state_msg.current = battery_state.current.value  # Negative when discharging (A)  (If unmeasured NaN)
        battery_state_msg.charge = float("nan")  # Current charge in Ah  (If unmeasured NaN)
        battery_state_msg.capacity = float("nan")  # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        battery_state_msg.design_capacity = float("nan")  # Capacity in Ah (design capacity)  (If unmeasured NaN)
        battery_state_msg.percentage = battery_state.charge_percentage.value / 100.0  # Charge percentage on 0 to 1 range  (If unmeasured NaN)

        battery_state_msg.power_supply_status = battery_status_proto_to_msg(
            battery_state.status)  # The charging status as reported. Values defined above
        battery_state_msg.power_supply_health = sensor_msgs.msg.BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN  # The battery health metric. Values defined above
        battery_state_msg.power_supply_technology = sensor_msgs.msg.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION  # The battery chemistry. Values defined above
        battery_state_msg.present = True  # True if the battery is present

        battery_state_msg.cell_voltage = []  # An array of individual cell voltages for each cell in the pack. If
        # individual voltages unknown but number of cells known set each to NaN
        battery_state_msg.location = ""  # The location into which the battery is inserted. (slot number or plug)
        battery_state_msg.serial_number = battery_state.identifier  # The best approximation of the battery serial number

        # Currently unmapped proto fields
        # battery_state.estimated_runtime  # An estimate of remaining runtime. Note that this field might not be populated.
        # battery_state.temperatures  # Measured temperature measurements of battery, in Celsius. Temperatures may be measured in many locations across the battery.
    else:
        battery_state_msg.present = False

    return battery_state_msg


def transform_stamped_proto_to_msg(timestamp, frame_id, child_frame_id, transform, clock_skew):
    transform_msg = geometry_msgs.msg.TransformStamped()
    transform_msg.header.stamp = timestamp_proto_to_ros_time(timestamp, clock_skew)
    # Set frame_id according to base_frame field
    transform_msg.header.frame_id = frame_id
    transform_msg.child_frame_id = child_frame_id
    transform_msg.transform = transform_proto_to_msg(transform)
    return transform_msg


def path_msg_to_proto(path_msg):
    """

    :type path_msg: nav_msgs.msg.Path
    """
    trajectory = trajectory_pb2.SE2Trajectory()
    bosdyn.util.set_timestamp_from_now(trajectory.reference_time)  # Can be left on 0 # TODO Fill with time of first pose
    trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
    for i, pose in enumerate(path_msg.poses, start=1):
        point = trajectory_pb2.SE2TrajectoryPoint()
        point.pose.position.x = pose.pose.position.x
        point.pose.position.y = pose.pose.position.y
        quaternion_proto = quaternion_msg_to_proto(pose.pose.orientation)
        yaw = quaternion_to_ypr(quaternion_proto)[0]
        point.pose.angle = yaw
        point.time_since_reference.seconds = 0  # TODO fill with time of each pose
        trajectory.points.append(point)
    return trajectory


def mobility_params_msg_to_proto(mobility_params_msg):
    """

    :type mobility_params_msg: hector_spot_ros_msgs.msg.MobilityParams
    """

    mobility_params = spot_command_pb2.MobilityParams()
    # Velocity limit
    mobility_params.vel_limit.max_vel.linear.x = mobility_params_msg.vel_limit_linear_x
    mobility_params.vel_limit.max_vel.linear.y = mobility_params_msg.vel_limit_linear_y
    mobility_params.vel_limit.max_vel.angular = mobility_params_msg.vel_limit_angular

    # Body control
    # mobility_params.body_control # TODO

    # Gait
    mobility_params.locomotion_hint = mobility_params_msg.locomotion_hint

    # Stairs
    mobility_params.stair_hint = mobility_params_msg.stair_hint

    # Degraded perception
    mobility_params.allow_degraded_perception = mobility_params_msg.allow_degraded_perception

    # Obstacle params
    mobility_params.obstacle_params.disable_vision_foot_obstacle_avoidance = mobility_params_msg.obstacle_params.disable_vision_foot_obstacle_avoidance
    mobility_params.obstacle_params.disable_vision_foot_constraint_avoidance = mobility_params_msg.obstacle_params.disable_vision_foot_constraint_avoidance
    mobility_params.obstacle_params.disable_vision_body_obstacle_avoidance = mobility_params_msg.obstacle_params.disable_vision_body_obstacle_avoidance
    mobility_params.obstacle_params.obstacle_avoidance_padding = mobility_params_msg.obstacle_params.obstacle_avoidance_padding

    # Swing height
    mobility_params.swing_height = mobility_params_msg.swing_height

    # Terrain params
    mobility_params.terrain_params.ground_mu_hint.value = mobility_params_msg.terrain_params.ground_mu_hint
    mobility_params.terrain_params.enable_grated_floor = mobility_params_msg.terrain_params.enable_grated_floor

    mobility_params.disallow_stair_tracker = mobility_params_msg.disallow_stair_tracker

    # External force params
    mobility_params.external_force_params.external_force_indicator = mobility_params_msg.external_force_params.external_force_indicator
    mobility_params.external_force_params.frame_name = "body"  # TODO: allow arbitrary frames
    mobility_params.external_force_params.external_force_override.x = mobility_params_msg.external_force_params.external_force_override.x
    mobility_params.external_force_params.external_force_override.y = mobility_params_msg.external_force_params.external_force_override.y
    mobility_params.external_force_params.external_force_override.z = mobility_params_msg.external_force_params.external_force_override.z

    mobility_params.disallow_non_stairs_pitch_limiting = mobility_params_msg.disallow_non_stairs_pitch_limiting
    mobility_params.disable_nearmap_cliff_avoidance = mobility_params_msg.disable_nearmap_cliff_avoidance

    return mobility_params


def grid_map_proto_to_msg(local_grid_proto, clock_skew, grid_map_msg_out, mask_proto=None):
    """

    :type grid_map_msg_out: grid_map_msgs.msg.GridMap
    """
    # Set grid map info. Overrides existing entries, so layers have to be compatible (same info)
    # TODO: add warning if info doesn't match
    # Header
    grid_map_msg_out.info.header.stamp = timestamp_proto_to_ros_time(local_grid_proto.acquisition_time, clock_skew)
    grid_map_msg_out.info.header.frame_id = "odom"
    # Dimensions
    cell_size = local_grid_proto.extent.cell_size
    grid_map_msg_out.info.resolution = cell_size
    grid_map_msg_out.info.length_x = local_grid_proto.extent.num_cells_x * cell_size
    grid_map_msg_out.info.length_y = local_grid_proto.extent.num_cells_y * cell_size
    # Pose
    pose3d = get_a_tform_b(local_grid_proto.transforms_snapshot, VISION_FRAME_NAME, local_grid_proto.frame_name_local_grid_data)
    grid_map_msg_out.info.pose = pose_proto_to_msg(pose3d)  # TODO: BD pose is top-left of grid-map, msg pose is center of grid map
    grid_map_msg_out.info.pose.position.x += grid_map_msg_out.info.length_x / 2
    grid_map_msg_out.info.pose.position.y += grid_map_msg_out.info.length_y / 2

    # Append layer data
    grid_map_msg_out.layers.append(local_grid_proto.local_grid_type_name)
    data_array_msg = std_msgs.msg.Float32MultiArray()
    # Add array layout
    array_dimension_x = std_msgs.msg.MultiArrayDimension()
    array_dimension_x.label = "column_index"
    array_dimension_x.stride = local_grid_proto.extent.num_cells_x * local_grid_proto.extent.num_cells_y
    array_dimension_x.size = local_grid_proto.extent.num_cells_x
    data_array_msg.layout.dim.append(array_dimension_x)
    array_dimension_y = std_msgs.msg.MultiArrayDimension()
    array_dimension_y.label = "column_row"
    array_dimension_y.stride = local_grid_proto.extent.num_cells_x
    array_dimension_y.size = local_grid_proto.extent.num_cells_y
    data_array_msg.layout.dim.append(array_dimension_y)
    grid_unpacked = unpack_grid(local_grid_proto).astype(np.float32)
    grid_unpacked = np.flip(grid_unpacked)  # convert axis convention (corresponds to 180° rotation)

    # if local_grid_proto.local_grid_type_name == "no_step":
    #     steppable = grid_unpacked > 0
    #     grid_unpacked[steppable] = 1.0
    #     grid_unpacked[np.invert(steppable)] = -1.0

    if mask_proto is not None:
        mask_unpacked = unpack_grid(mask_proto)
        invalid_cell = mask_unpacked == 0
        grid_unpacked[invalid_cell] = float("nan")

    data_array_msg.data = list(grid_unpacked)

    grid_map_msg_out.data.append(data_array_msg)


def unpack_grid(local_grid_proto):
    """Unpack the local grid proto."""
    # Determine the data type for the bytes data.
    data_type = _get_gridmap_numpy_data_type(local_grid_proto)
    if data_type is None:
        print("Cannot determine the dataformat for the local grid.")
        return None
    # Decode the local grid.
    if local_grid_proto.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
        full_grid = np.fromstring(local_grid_proto.data, dtype=data_type)
    elif local_grid_proto.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
        full_grid = _expand_gridmap_data_by_rle_count(local_grid_proto, data_type=data_type)
    else:
        # Return nothing if there is no encoding type set.
        return None
    # Apply the offset and scaling to the local grid.
    if local_grid_proto.cell_value_scale == 0:
        return full_grid
    full_grid_float = full_grid.astype(np.float64)
    full_grid_float *= local_grid_proto.cell_value_scale
    full_grid_float += local_grid_proto.cell_value_offset
    return full_grid_float


def _get_gridmap_numpy_data_type(local_grid_proto):
    """Convert the cell format of the local grid proto to a numpy data type."""
    if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
        return np.uint16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
        return np.int16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
        return np.uint8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
        return np.int8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
        return np.float64
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
        return np.float32
    else:
        return None


def _expand_gridmap_data_by_rle_count(local_grid_proto, data_type=np.int16):
    """Expand local grid data to full bytes data using the RLE count."""
    cells_pz = np.fromstring(local_grid_proto.data, dtype=data_type)
    cells_pz_full = []
    # For each value of rle_counts, we expand the cell data at the matching index
    # to have that many repeated, consecutive values.
    for i in range(0, len(local_grid_proto.rle_counts)):
        for j in range(0, local_grid_proto.rle_counts[i]):
            cells_pz_full.append(cells_pz[i])
    return np.array(cells_pz_full)


def localization_msg_to_proto(pose_stamped, clock_skew):
    """

    :type pose_stamped: geometry_msgs.msg.PoseStamped
    """
    localization_proto = nav_pb2.Localization(timestamp=timestamp_ros_time_to_proto(pose_stamped.header.stamp, clock_skew),
                                              waypoint_tform_body=pose_msg_to_proto(pose_stamped.pose),
                                              waypoint_id=pose_stamped.header.frame_id)
    return localization_proto
