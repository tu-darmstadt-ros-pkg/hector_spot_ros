import numpy as np

import rospy
import visualization_msgs.msg
import sensor_msgs.msg
from hector_spot_ros.utils import proto_conversions


def clear_all_markers(publisher):
    """

    :type publisher: rospy.topics.Publisher
    """
    markers = visualization_msgs.msg.MarkerArray()
    marker = visualization_msgs.msg.Marker()
    marker.action = visualization_msgs.msg.Marker.DELETEALL
    markers.markers.append(marker)
    publisher.publish(markers)


def graph_topology_to_visualization_msg(graph):
    marker_array = visualization_msgs.msg.MarkerArray()
    next_marker_id = 0
    # Draw waypoints
    for waypoint in graph.waypoints.values():
        next_marker_id = add_waypoint_marker(waypoint, marker_array, next_marker_id)
    # Draw edges
    for edge in graph.edges:
        next_marker_id = add_edge_marker(edge, graph.waypoints, marker_array, next_marker_id)

    return marker_array


def add_edge_marker(edge, waypoints, marker_array, initial_marker_id):
    marker_id = initial_marker_id
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = "world"
    marker.ns = "edges"
    marker.id = marker_id
    marker_id += 1
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x = 0.03
    marker.scale.y = 0.05
    marker.scale.z = 0.07
    pose_begin = waypoints[edge.edge_proto.id.from_waypoint].global_pose
    marker.points.append(proto_conversions.point_proto_to_msg(pose_begin.position))
    pose_end = waypoints[edge.edge_proto.id.to_waypoint].global_pose
    marker.points.append(proto_conversions.point_proto_to_msg(pose_end.position))
    marker.pose.orientation.w = 1  # mute warning
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker_array.markers.append(marker)
    return marker_id


def add_waypoint_marker(waypoint, marker_array, initial_marker_id):
    marker_id = initial_marker_id
    # Sphere marker for position
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = "world"
    marker.ns = "waypoints"
    marker.id = marker_id
    marker_id += 1
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.pose = proto_conversions.pose_proto_to_msg(waypoint.global_pose)
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker_array.markers.append(marker)

    # Text marker for waypoint name
    text = visualization_msgs.msg.Marker()
    text.header.frame_id = "world"
    text.ns = "waypoint-labels"
    text.id = marker_id
    marker_id += 1
    text.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
    text.action = visualization_msgs.msg.Marker.ADD
    text.text = waypoint.waypoint_proto.id
    text.scale.x = text.scale.y = 0
    text.scale.z = 0.1
    text.pose = proto_conversions.pose_proto_to_msg(waypoint.global_pose)
    text.pose.position.z += 1.5 * text.scale.z
    text.color.a = 1.0
    text.color.r = 1.0
    text.color.g = 0.0
    text.color.b = 0.0
    marker_array.markers.append(text)

    return marker_id


def waypoints_to_point_cloud_msg(waypoints):
    """

    :type waypoints: dict
    """
    cloud_msg = sensor_msgs.msg.PointCloud2()
    cloud_msg.header.frame_id = "world"
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.height = 1
    cloud_msg.width = 0
    cloud_msg.point_step = 12  # 3 * float32
    cloud_msg.is_dense = True
    cloud_msg.is_bigendian = False
    cloud_msg.fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
    cloud_msg.fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
    cloud_msg.fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

    for waypoint in waypoints.values():
        _append_cloud_data(waypoint.global_point_cloud, cloud_msg)

    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    return cloud_msg


def _append_cloud_data(cloud, cloud_msg):
    """

    :param cloud: np.ndarray
    :type cloud_msg: sensor_msgs.msg.PointCloud2
    """
    num_points = np.shape(cloud)[0]
    cloud_msg.width += num_points
    cloud_msg.data += cloud.tobytes()
