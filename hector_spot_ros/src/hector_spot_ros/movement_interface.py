import time
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api import robot_command_pb2, basic_command_pb2, mobility_command_pb2, synchronized_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn import geometry
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME

from hector_spot_ros.utils import utils
from hector_spot_ros.periodic_tasks import PeriodicRobotCommandFeedback

VELOCITY_CMD_DURATION = 0.6  # seconds


class MovementInterface:
    ORIENTATION_ROLL_MAX = 0.5  # rad
    ORIENTATION_PITCH_MAX = 0.53  # rad
    ORIENTATION_YAW_MAX = 0.5  # rad
    HEIGHT_MAX = 0.3  # m

    def __init__(self, robot):
        assert robot is not None
        self._robot = robot
        self._command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)

        self._zero_velocity_commanded = True

        self._follow_trajectory_feedback_task = None
        self._follow_trajectory_finished_cb = None

        self.mobility_params = spot_command_pb2.MobilityParams()

    def selfright(self):
        cmd = RobotCommandBuilder.selfright_command()
        self._command_client.robot_command(cmd)

    def stand_up(self):
        cmd = RobotCommandBuilder.synchro_stand_command()
        self._command_client.robot_command(cmd)

    def sit_down(self):
        cmd = RobotCommandBuilder.synchro_sit_command()
        self._command_client.robot_command(cmd)

    def send_body_pose_ypr(self, yaw=0.0, pitch=0.0, roll=0.0, body_height=0.0):
        """
        Ask Spot to orient its body (e.g. yaw, pitch, roll)

        @param[in]  yaw    Rotation about Z (+up/down) [rad]
        @param[in]  pitch  Rotation about Y (+left/right) [rad]
        @param[in]  roll   Rotation about X (+front/back) [rad]
        """
        rotation = geometry.EulerZXY(yaw=yaw, pitch=pitch, roll=roll)
        self._send_body_pose(rotation, body_height)

    def send_bose_pose_quat(self, quaternion, body_height):
        rotation = quaternion.to_euler_zxy()
        self._send_body_pose(rotation, body_height)

    def _send_body_pose(self, rotation, body_height):
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=rotation, body_height=body_height)
        self._command_client.robot_command_async(cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def send_velocity_command(self, v_x, v_y, v_rot):
        zero_command = v_x == v_y == v_rot == 0.0
        if zero_command and self._zero_velocity_commanded:
            return
        self._zero_velocity_commanded = zero_command

        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=self.mobility_params)
        self._command_client.robot_command_async(cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def follow_trajectory_command(self, trajectory, desired_speed=0.5, finished_cb=None):
        # Build command
        trajectory_command = basic_command_pb2.SE2TrajectoryCommand.Request(trajectory=trajectory, se2_frame_name=ODOM_FRAME_NAME)

        mobility_params = spot_command_pb2.MobilityParams()
        mobility_params.CopyFrom(self.mobility_params)
        mobility_params.vel_limit.max_vel.linear.x = desired_speed
        mobility_params.vel_limit.max_vel.linear.y = desired_speed
        mobility_params.vel_limit.max_vel.angular = 0.5
        mobility_params_proto = utils.to_any(mobility_params)
        mobility_command = mobility_command_pb2.MobilityCommand.Request(
            se2_trajectory_request=trajectory_command, params=mobility_params_proto)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            mobility_command=mobility_command)
        command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
        # Send command
        command_id = self._command_client.robot_command(command, end_time_secs=time.time() + 30.0)  # TODO set end_time based on trajectory length

        # Pull for feedback on separate thread
        if finished_cb is not None:
            self._follow_trajectory_finished_cb = finished_cb
            self._follow_trajectory_feedback_task = PeriodicRobotCommandFeedback(self._command_client, 1, command_id)
            self._follow_trajectory_feedback_task.register_callback(self._follow_trajectory_status_callback)
            self._follow_trajectory_feedback_task.start()

    def cancel_follow_trajectory_command(self):
        self._follow_trajectory_feedback_task.stop()
        self.stand_up()

    def _follow_trajectory_status_callback(self, response):
        assert self._follow_trajectory_finished_cb is not None
        # Error occurred
        if response is None:
            self._follow_trajectory_feedback_task.stop()
            self._follow_trajectory_finished_cb(False)
            return
        status = response.feedback.mobility_feedback.se2_trajectory_feedback.status
        # print("Status", status)
        # Check if robot is at goal
        if status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
            self._follow_trajectory_feedback_task.stop()
            self._follow_trajectory_finished_cb(True)
            return
        # if status == robot_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_UNKNOWN:
        #     self._follow_trajectory_feedback_task.stop()
        #     self._follow_trajectory_finished_cb(False)
