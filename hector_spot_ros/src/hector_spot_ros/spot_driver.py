import bosdyn.client
import bosdyn.client.util
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.robot_id
import time

import rospy

from hector_spot_ros import movement_interface, camera_interface, state_interface, graph_nav_interface
from hector_spot_ros.utils.utils import wait_for_client, SpotConfig

SUPPORTED_MAJOR_VERSION = 3


class SpotDriver:
    def __init__(self):
        self._sdk = None
        # A handle to the spot robot
        self._robot = None
        # A handle to the estop endpoint
        self._external_estop = False
        self._estop_endpoint = None
        self._estop_keep_alive = None
        # A handle to the lease for the robot
        self._lease_client = None
        self._lease_keep_alive = None
        self._lease = None

        # Create high-level interfaces
        self.movement = None  # type: movement_interface.MovementInterface
        self.camera = None  # type: camera_interface.CameraInterface
        self.state = None  # type: state_interface.StateInterface
        self.graph_nav = None  # type: graph_nav_interface.GraphNavInterface

    def connect(self, config):
        """

        :type config: SpotConfig
        """
        # Create SDK
        self._sdk = bosdyn.client.create_standard_sdk(config.app_name)

        # Connect to robot
        self._robot = self._sdk.create_robot(config.hostname)

        success = False
        while not success:
            try:
                # Check if firmware version is supported
                if not self._is_firmware_supported():
                    return False
                # connect
                self._robot.authenticate(config.user, config.password, timeout=1)
                success = True
            except Exception as error:
                rospy.loginfo('SpotDriver: could not connect to robot, will reconnect in 5 sec. Error: {}'.format(error))
                time.sleep(5)

        rospy.loginfo("Authentication successful")
        self._robot.start_time_sync()

        # Register E-stop
        self._external_estop = config.external_estop
        if not self._external_estop:
            estop_name = 'ros-spot-estop'
            # Wait for e-stop service to become available
            estop_client = wait_for_client(self._robot, bosdyn.client.estop.EstopClient.default_service_name)
            self._estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=estop_client,
                                                                     name=estop_name,
                                                                     estop_timeout=9.0)
            self._estop_endpoint.force_simple_setup()

        # Handle lease keep alive
        self._lease_client = self._robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self._lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client)

        # Create clients
        self.movement = movement_interface.MovementInterface(self._robot)
        self.camera = camera_interface.CameraInterface(self._robot)
        self.state = state_interface.StateInterface(self._robot)
        self.graph_nav = graph_nav_interface.GraphNavInterface(self._robot)

        rospy.loginfo("Connection successful")
        return True

    def start(self, power_on=False):
        # Keep E-stop alive
        if not self._external_estop:
            self._estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(self._estop_endpoint)

        # Acquire lease
        self._lease = self._lease_client.acquire()

        # Establish time sync with the robot
        self._robot.time_sync.wait_for_sync()

        # Start periodic queries
        self.camera.start()
        self.state.start()
        self.graph_nav.start()

        # Power on robot
        if power_on:
            return self.power_on()
        else:
            return True

    def stop(self):
        # Stop periodic requests
        self.camera.stop()
        self.state.stop()
        self.graph_nav.stop()

        # Release estop keep alive
        if self._estop_keep_alive:
            self._estop_keep_alive.settle_then_cut() # sit down, then power off
            self._estop_keep_alive = None

        # Return lease
        if self._lease_keep_alive:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def power_on(self):
        if not self._robot.is_powered_on():
            self._robot.power_on(timeout_sec=20)
        return self._robot.is_powered_on()

    def safe_power_off(self):
        self._robot.power_off(cut_immediately=False)

    def get_clock_skew(self):
        return self._robot.time_sync.endpoint.clock_skew

    def _is_firmware_supported(self):
        if self._robot:
            id_client = self._robot.ensure_client(bosdyn.client.robot_id.RobotIdClient.default_service_name)
            robot_id = id_client.get_id()
            major_version = robot_id.software_release.version.major_version
            firmware_supported = major_version == SUPPORTED_MAJOR_VERSION
            if not firmware_supported:
                print("Firmware version {} is not supported. This driver only supports major version {}.".format(
                    major_version, SUPPORTED_MAJOR_VERSION))
                return False
            return True
        else:
            print("Can't retrieve firmware version. Not connected to robot")
            return False

