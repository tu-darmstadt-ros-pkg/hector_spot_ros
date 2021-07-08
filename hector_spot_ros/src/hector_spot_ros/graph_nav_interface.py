import numpy as np

import rospy
import threading
import pickle

from bosdyn.api.graph_nav import recording_pb2, graph_nav_pb2
from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.recording import GraphNavRecordingServiceClient

from hector_spot_ros.periodic_tasks import PeriodicLocalizationState


class Graph:
    def __init__(self):
        self.waypoints = dict()
        self.edges = []


class Waypoint:
    def __init__(self, waypoint_proto, snapshot_proto):
        self.waypoint_proto = waypoint_proto
        self.snapshot_proto = snapshot_proto
        self.global_pose = None
        self.global_point_cloud = None


class Edge:
    def __init__(self, edge_proto, snapshot_proto):
        self.edge_proto = edge_proto
        self.snapshot_proto = snapshot_proto


class GraphNavInterface:
    def __init__(self, robot):
        self._robot = robot
        self._recording_client = self._robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)  # type: GraphNavRecordingServiceClient
        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)  # type: GraphNavClient
        self.periodic_localization_state = PeriodicLocalizationState(self, 1)

        self._navigation_cancelled = False
        self._navigate_to_waypoint_thread = None

        self._reference_waypoint_id = None

    def start(self):
        self.periodic_localization_state.start()

    def stop(self):
        self.periodic_localization_state.stop()

    def start_recording(self):
        recording_possible = self._is_recording_possible()
        if not recording_possible:
            rospy.logerr("The system is not in the proper state to start recording. Either clear the map or localize first.")
            return False
        try:
            status = self._recording_client.start_recording()
            rospy.loginfo("Successfully started recording a map.")
        except Exception as err:
            rospy.logerr("Start recording failed: " + str(err))
            return False
        return True

    def stop_recording(self):
        """Stop or pause recording a map."""
        try:
            status = self._recording_client.stop_recording()
            rospy.loginfo("Successfully stopped recording a map.")
        except Exception as err:
            rospy.logerr("Stop recording failed: " + str(err))
            return False
        return True

    def clear_graph(self):
        return self._graph_nav_client.clear_graph()

    def create_waypoint(self, name):
        """Create a waypoint at the robot's current location."""
        resp = self._recording_client.create_waypoint(waypoint_name=name)
        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            rospy.loginfo("Successfully created waypoint '{}' with id '{}'.".format(name, resp.created_waypoint.id))
            return resp
        else:
            rospy.logerr("Could not create waypoint '{}'.".format(name))
            return None

    def download_graph(self, full_graph=True):
        """Download the full graph with topology and waypoint/edge snapshots"""
        graph_proto = self._graph_nav_client.download_graph()
        if graph_proto is None:
            rospy.logerr("Failed to download the graph.")
            return None, None, None
        if len(graph_proto.waypoints) == 0:
            rospy.logerr("Graph is empty.")
            return None, None, None

        if full_graph:
            waypoint_snapshot_dict = self._download_waypoint_snapshots(graph_proto.waypoints)
            edge_snapshot_dict = self._download_edge_snapshots(graph_proto.edges)
            return graph_proto, waypoint_snapshot_dict, edge_snapshot_dict
        else:
            return graph_proto, None, None

    def _download_waypoint_snapshots(self, waypoints):
        """Download the waypoint snapshots from robot to a dictionary snapshot id -> snapshot proto"""
        snapshot_dict = dict()
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            # Only download snapshot if necessary
            if waypoint.snapshot_id not in snapshot_dict:
                try:
                    waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                        waypoint.snapshot_id)
                except Exception:
                    # Failure in downloading waypoint snapshot. Continue to next snapshot.
                    print("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                    continue

                snapshot_dict[waypoint.snapshot_id] = waypoint_snapshot
            num_waypoint_snapshots_downloaded += 1
            rospy.loginfo("Downloaded {} of the total {} waypoint snapshots.".format(
                num_waypoint_snapshots_downloaded, len(waypoints)))
        return snapshot_dict

    def _download_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to a dictionary snapshot id -> snapshot proto"""
        snapshot_dict = dict()
        num_edge_snapshots_downloaded = 0
        for edge in edges:
            if edge.snapshot_id not in snapshot_dict:
                try:
                    edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
                except Exception:
                    # Failure in downloading edge snapshot. Continue to next snapshot.
                    print("Failed to download edge snapshot: " + edge.snapshot_id)
                    continue
                snapshot_dict[edge.snapshot_id] = edge_snapshot
            num_edge_snapshots_downloaded += 1
            rospy.loginfo("Downloaded {} of the total {} edge snapshots.".format(
                num_edge_snapshots_downloaded, len(edges)))
        return snapshot_dict

    def convert_graph_to_global_frame(self, graph_proto, waypoint_snapshot_dict=None, edge_snapshot_dict=None):
        """Download the full graph and convert it to a global reference frame (typically the first waypoint)"""
        graph_object = Graph()
        for waypoint in graph_proto.waypoints:
            if waypoint_snapshot_dict:
                waypoint_snapshot_proto = waypoint_snapshot_dict[waypoint.snapshot_id]
            else:
                waypoint_snapshot_proto = None
            graph_object.waypoints[waypoint.id] = Waypoint(waypoint, waypoint_snapshot_proto)

        for edge in graph_proto.edges:
            if edge_snapshot_dict:
                edge_snapshot_proto = edge_snapshot_dict[edge.snapshot_id]
            else:
                edge_snapshot_proto = None
            graph_object.edges.append(Edge(edge, edge_snapshot_proto))

        # Graphs are only connected by relative transformations. There is no global reference frame.
        # We set the first waypoint as reference and traverse the graph to compute global poses of waypoints
        # and point clouds

        # Set pose of reference waypoint
        reference_idx = 0
        if self._reference_waypoint_id:
            found = False
            for i, waypoint in enumerate(graph_proto.waypoints):
                if waypoint.id == self._reference_waypoint_id:
                    reference_idx = i
                    found = True
                    break
            if not found:
                rospy.logwarn("Could not find reference waypoint with id '" + self._reference_waypoint_id + "'. "
                              "Creating a new reference. This is normal after creating a new map.")
        self._reference_waypoint_id = graph_proto.waypoints[reference_idx].id
        reference_waypoint = graph_object.waypoints[self._reference_waypoint_id]
        queue = [reference_waypoint]
        reference_waypoint.global_pose = SE3Pose.from_identity()
        visited = dict()

        while len(queue) > 0:
            current_element = queue.pop(0)
            visited[current_element.waypoint_proto.id] = True

            # For each adjacent edge, walk along and concatenate transform
            transform_world_to_current_waypoint = current_element.global_pose
            for edge in graph_proto.edges:
                next_waypoint = None
                transform_current_waypoint_to_next_waypoint = None
                # Directed away from current waypoint
                if edge.id.from_waypoint == current_element.waypoint_proto.id and edge.id.to_waypoint not in visited:
                    next_waypoint = graph_object.waypoints[edge.id.to_waypoint]
                    transform_current_waypoint_to_next_waypoint = SE3Pose.from_obj(edge.from_tform_to)
                    match = True
                # Directed towards current waypoint
                elif edge.id.to_waypoint == current_element.waypoint_proto.id and edge.id.from_waypoint not in visited:
                    next_waypoint = graph_object.waypoints[edge.id.from_waypoint]
                    transform_current_waypoint_to_next_waypoint = SE3Pose.from_obj(edge.from_tform_to).inverse()
                    match = True
                if next_waypoint and transform_current_waypoint_to_next_waypoint:
                    next_waypoint.global_pose = transform_world_to_current_waypoint * transform_current_waypoint_to_next_waypoint
                    queue.append(next_waypoint)

        # Transform point clouds to global frame
        if waypoint_snapshot_dict:
            for waypoint in graph_object.waypoints.values():
                cloud = waypoint.snapshot_proto.point_cloud
                transform_odom_to_cloud = get_a_tform_b(cloud.source.transforms_snapshot, ODOM_FRAME_NAME,
                                                        cloud.source.frame_name_sensor)
                transform_waypoint_to_odom = SE3Pose.from_obj(waypoint.waypoint_proto.waypoint_tform_ko)
                transform_world_to_cloud = waypoint.global_pose * transform_waypoint_to_odom * transform_odom_to_cloud

                point_cloud_data = np.frombuffer(cloud.data, dtype=np.float32).reshape(int(cloud.num_points), 3)
                waypoint.global_point_cloud = transform_world_to_cloud.transform_cloud(point_cloud_data).astype(np.float32)

        return graph_object

    def save_graph_to_file(self, graph_proto, waypoint_snapshot_dict, edge_snapshot_dict, file_path):
        with open(file_path, 'wb') as graph_file:
            pickle.dump(graph_proto, graph_file)
            pickle.dump(waypoint_snapshot_dict, graph_file)
            pickle.dump(edge_snapshot_dict, graph_file)
            pickle.dump(self._reference_waypoint_id, graph_file)
        return True

    def load_graph_from_file(self, file_path):
        with open(file_path, 'rb') as graph_file:
            graph_proto = pickle.load(graph_file)
            waypoint_snapshot_dict = pickle.load(graph_file)
            edge_snapshot_dict = pickle.load(graph_file)
            self._reference_waypoint_id = pickle.load(graph_file)
            return graph_proto, waypoint_snapshot_dict, edge_snapshot_dict

    def upload_graph(self, graph_proto, waypoint_snapshots, edge_snapshots):
        rospy.loginfo("Uploading the graph")
        self._graph_nav_client.upload_graph(graph=graph_proto)

        # Upload the snapshots to the robot.
        rospy.loginfo("Uploading waypoint snapshots")
        for waypoint_snapshot in waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            rospy.loginfo("Uploaded {}".format(waypoint_snapshot.id))
        rospy.loginfo("Uploading edge snapshots")
        for edge_snapshot in edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            rospy.loginfo("Uploaded {}".format(edge_snapshot.id))

    def set_localization(self, initial_guess_localization, max_distance=None, max_yaw=None,
                         fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST,
                         use_fiducial_id=None, refine_fiducial_result_with_icp=False,
                         do_ambiguity_check=False):
        # Simplification! Take current state as state at time of localization
        state = self._graph_nav_client.get_localization_state()
        odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot).to_proto()
        return self._graph_nav_client.set_localization(initial_guess_localization,
                                                       odom_tform_body,
                                                       max_distance,
                                                       max_yaw,
                                                       fiducial_init,
                                                       use_fiducial_id,
                                                       refine_fiducial_result_with_icp,
                                                       do_ambiguity_check)

    def get_localization_state(self):
        return self._graph_nav_client.get_localization_state()

    def get_global_localization(self, graph=None):
        """

        :type graph: Graph
        """
        # Check, if robot is localized
        state_proto = self.get_localization_state()
        if not self._is_localized(state_proto):
            return None

        # Download graph if none was provided
        if graph is None:
            graph_proto, _, _ = self.download_graph(full_graph=False)
            # Exit if no graph is available
            if graph_proto is None:
                return
            graph = self.convert_graph_to_global_frame(graph_proto)

        waypoint_to_body = SE3Pose.from_obj(state_proto.localization.waypoint_tform_body)
        try:
            world_to_waypoint = graph.waypoints[state_proto.localization.waypoint_id].global_pose
        except KeyError as e:
            rospy.logerr("Waypoint '{}' is not part of given graph.".format(state_proto.localization.waypoint_id))
            return None
        world_to_body = world_to_waypoint * waypoint_to_body
        return world_to_body, state_proto, graph

    def _navigate_to_waypoint(self, waypoint_id, velocity_limit=None, finished_threshold=None, callback=None):
        is_finished = False
        success = False
        self._navigation_cancelled = False
        travel_params = graph_nav_pb2.TravelParams()
        if velocity_limit:
            travel_params.velocity_limit.max_vel.linear.x = velocity_limit[0]
            travel_params.velocity_limit.max_vel.linear.y = velocity_limit[0]
            travel_params.velocity_limit.max_vel.angular = velocity_limit[1]
        if finished_threshold:
            travel_params.max_distance = finished_threshold[0]
            travel_params.max_yaw = finished_threshold[1]
        while not is_finished and not self._navigation_cancelled:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            command_id = self._graph_nav_client.navigate_to(waypoint_id, 1.0, travel_params=travel_params)
            rospy.sleep(0.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete
            is_finished, success = self._is_navigation_finished(command_id)

        if callback and not self._navigation_cancelled:
            callback(success)
        return success

    def navigate_to_waypoint(self, waypoint_id, velocity_limit=None, finished_threshold=None, async_execution=True, finished_cb=None):
        if async_execution:
            self._navigate_to_waypoint_thread = \
                threading.Thread(target=lambda: self._navigate_to_waypoint(waypoint_id, velocity_limit,
                                                                           finished_threshold, finished_cb))
            self._navigate_to_waypoint_thread.start()
            return None
        else:
            return self._navigate_to_waypoint(waypoint_id, velocity_limit, finished_threshold, finished_cb)

    def cancel_navigation(self):
        self._navigation_cancelled = True

    def navigate_route(self, waypoint_ids):
        pass

    def _is_navigation_finished(self, command_id):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False, False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True, True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            rospy.logerr("Robot got lost when navigating the route. Stopping execution.")
            return True, False
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            rospy.logerr("Robot got stuck when navigating the route. Stopping execution.")
            return True, False
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            rospy.logerr("Robot is impaired. Stopping execution.")
            return True, False
        else:
            # Navigation command is not complete yet.
            return False, False

    def _is_localized(self, localization_state_proto=None):
        if localization_state_proto is None:
            localization_state_proto = self._graph_nav_client.get_localization_state()
        return len(localization_state_proto.localization.waypoint_id) != 0 \
               and not localization_state_proto.lost_detector_state.is_lost

    def _is_recording_possible(self):
        # Before starting to record, check the state of the GraphNav system.
        graph = self._graph_nav_client.download_graph()
        if graph is not None:
            # Check that the graph has waypoints. If it does, then we need to be localized to the graph
            # before starting to record
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    # Not localized to anything in the map. The best option is to clear the graph or
                    # attempt to localize to the current map.
                    # Returning false since the GraphNav system is not in the state it should be to
                    # begin recording.
                    return False
        # If there is no graph or there exists a graph that we are localized to, then it is fine to
        # start recording, so we return True.
        return True
