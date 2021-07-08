#!/usr/bin/python3
import threading
from enum import Enum, auto
import rospy
import std_srvs.srv
import actionlib
import actionlib_msgs.msg
import hector_spot_ros_msgs.srv
import hector_spot_ros_msgs.msg

START_GRAPH_RECORDING_SERVICE_NAME = "/graph_nav/start_recording"
STOP_GRAPH_RECORDING_SERVICE_NAME = "/graph_nav/stop_recording"
CREATE_WAYPOINT_SERVICE_NAME = "/graph_nav/create_waypoint"


class State(Enum):
    GOING_TO_START = auto()
    GOING_TO_END = auto()
    FINISHED = auto()


class ThreadWithCallback:
    def __init__(self, target, callback):
        self._target = target
        self._callback = callback
        self._thread = threading.Thread(target=self._run_thread)
        self._thread.start()

    def _run_thread(self):
        result = self._target()
        self._callback(result)


class MissionRecorder:
    def __init__(self):
        # Members
        self._state = State.FINISHED
        self._start_waypoint_id = None
        self._end_waypoint_id = None
        self._mission_recording_active = False
        self._linear_velocity_limit = 0.0
        self._angular_velocity_limit = 0.0
        # Action Server
        self._execute_mission_as = actionlib.SimpleActionServer("~execute_mission",
                                                                hector_spot_ros_msgs.msg.ExecuteMissionAction,
                                                                auto_start=False)
        self._execute_mission_as.register_goal_callback(self._execute_mission_goal_callback)
        self._execute_mission_as.register_preempt_callback(self._execute_mission_preempt_callback)
        self._execute_mission_as.start()

        # Service server
        self._start_mission_recording_server = rospy.Service("~start_mission_recording",
                                                             std_srvs.srv.Trigger,
                                                             self._start_mission_recording_callback)
        self._stop_mission_recording_server = rospy.Service("~stop_mission_recording",
                                                             std_srvs.srv.Trigger,
                                                             self._stop_mission_recording_callback)

        rospy.loginfo("Waiting for services and actions")
        # Service Clients
        rospy.wait_for_service(START_GRAPH_RECORDING_SERVICE_NAME)
        self._start_graph_recording_client = rospy.ServiceProxy(START_GRAPH_RECORDING_SERVICE_NAME, std_srvs.srv.Empty)
        rospy.wait_for_service(STOP_GRAPH_RECORDING_SERVICE_NAME)
        self._stop_graph_recording_client = rospy.ServiceProxy(STOP_GRAPH_RECORDING_SERVICE_NAME, std_srvs.srv.Empty)
        rospy.wait_for_service(CREATE_WAYPOINT_SERVICE_NAME)
        self._create_waypoint_client = rospy.ServiceProxy(CREATE_WAYPOINT_SERVICE_NAME,
                                                          hector_spot_ros_msgs.srv.CreateGraphWaypoint)

        # Action client
        self._navigate_to_waypoint_client = actionlib.SimpleActionClient('/graph_nav/navigate_to_waypoint',
                                                                         hector_spot_ros_msgs.msg.NavigateToWaypointAction)
        self._navigate_to_waypoint_client.wait_for_server()
        self._navigate_to_waypoint_thread = None
        rospy.loginfo("Found all services and actions")

    def _start_mission_recording_callback(self, request):
        rospy.loginfo("Starting mission recording")
        # Start map recording
        try:
            result = self._start_graph_recording_client()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return None
        rospy.sleep(1.0)
        # Create waypoint
        try:
            result = self._create_waypoint_client("")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return None
        self._mission_recording_active = True
        self._start_waypoint_id = result.waypoint_id
        rospy.loginfo("Start waypoint: {}".format(self._start_waypoint_id))
        return std_srvs.srv.TriggerResponse()

    def _stop_mission_recording_callback(self, request):
        rospy.loginfo("Stopping mission recording")
        if not self._mission_recording_active:
            rospy.loginfo("Can't stop mission recording. No active recording.")
            return None

        # Create waypoint
        try:
            result = self._create_waypoint_client("")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return None
        self._end_waypoint_id = result.waypoint_id
        rospy.loginfo("End waypoint: {}".format(self._end_waypoint_id))
        rospy.sleep(1.0)
        # Stop graph recording
        try:
            result = self._stop_graph_recording_client()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return None

        self._mission_recording_active = False

        return std_srvs.srv.TriggerResponse()

    def _execute_mission_goal_callback(self):
        rospy.loginfo("Executing mission")
        # Check if execution is possible
        if not self._start_waypoint_id or not self._end_waypoint_id:
            rospy.logerr("No mission recorded yet")
            self._execute_mission_as.set_aborted()
            return

        if self._mission_recording_active:
            rospy.logerr("Mission recording is still active. Stop recording first")
            self._execute_mission_as.set_aborted()
            return
        # Accept goal
        goal = self._execute_mission_as.accept_new_goal()
        self._linear_velocity_limit = goal.linear_velocity_limit
        self._angular_velocity_limit = goal.angular_velocity_limit

        # Go to start
        self._state = State.GOING_TO_START
        if not self._execute_mission_as.is_preempt_requested():
            self._navigate_to_waypoint_threaded(self._start_waypoint_id)

    def _execute_mission_preempt_callback(self):
        rospy.loginfo("Mission preempted")
        self._navigate_to_waypoint_client.cancel_all_goals()
        self._execute_mission_as.set_preempted()

    def _navigate_to_waypoint(self, waypoint_id):
        rospy.loginfo("Going to waypoint '{}'".format(waypoint_id))
        goal = hector_spot_ros_msgs.msg.NavigateToWaypointGoal()
        goal.waypoint_id = waypoint_id
        goal.linear_velocity_limit = self._linear_velocity_limit
        goal.angular_velocity_limit = self._angular_velocity_limit
        final_state = self._navigate_to_waypoint_client.send_goal_and_wait(goal)

        if final_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            return True
        else:
            rospy.logerr("Failed to reach waypoint '{}'".format(waypoint_id))
            return False

    def _navigate_to_waypoint_threaded(self, waypoint_id):
        self._navigate_to_waypoint_thread = ThreadWithCallback(target=lambda: self._navigate_to_waypoint(waypoint_id),
                                    callback=self._navigate_to_waypoint_finished_callback)

    def _navigate_to_waypoint_finished_callback(self, success):
        if not success:
            self._execute_mission_as.set_aborted()
            return
        if self._state == State.GOING_TO_START:
            # Go to end
            self._state = State.GOING_TO_END
            if not self._execute_mission_as.is_preempt_requested():
                self._navigate_to_waypoint_threaded(self._end_waypoint_id)
        elif self._state == State.GOING_TO_END:
            # Finished
            self._state = State.FINISHED
            result = hector_spot_ros_msgs.msg.ExecuteMissionResult()
            self._execute_mission_as.set_succeeded(result)
            rospy.loginfo("Mission execution finished")


def main():
    rospy.init_node("spot_mission_recorder")
    rospy.loginfo("Starting mission recorder")
    mission_recorder = MissionRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
