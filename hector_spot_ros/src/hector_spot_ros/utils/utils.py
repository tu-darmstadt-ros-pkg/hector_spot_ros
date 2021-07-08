import rospy
import time

from google.protobuf import any_pb2
from bosdyn.api import robot_state_pb2
import bosdyn.client.common
import bosdyn.util
import bosdyn.client.sdk


class SpotConfig:
    def __init__(self):
        self.app_name = ""
        self.hostname = "192.168.80.3"
        self.user = None
        self.password = None
        self.control_rate = 20
        self.auto_power_on = False


def load_config_from_ns(ns):
    config = SpotConfig()
    config.app_name = "spot-ros-interface"
    config.hostname = load_mandatory_parameter(ns + "hostname")
    config.user = load_mandatory_parameter(ns + "user")
    config.password = load_mandatory_parameter(ns + "password")
    config.control_rate = rospy.get_param(ns + "control_rate", config.control_rate)
    config.auto_power_on = rospy.get_param(ns + "auto_power_on", config.auto_power_on)
    return config


def load_mandatory_parameter(ns):
    try:
        return rospy.get_param(ns)
    except KeyError:
        rospy.logerr("Failed to load mandatory parameter '" + ns + "'")
        return None


def future_result_with_timeout(future, timeout=1):
    """

    :type future: bosdyn.client.common.FutureWrapper
    """
    error = future.exception(timeout=timeout)
    if error is not None:
        raise error

    base_result = future.original_future.result(timeout=timeout)

    if future._value_from_response is None:
        return base_result

    return future._value_from_response(base_result)


def quaternion_to_ypr(quaternion):
    rotation = quaternion.to_euler_zxy()
    return [rotation.yaw, rotation.pitch, rotation.roll]


def clamp(val, limit):
    return max(min(limit, val), -limit)


def convert_timestamp_from_robot_to_local(timestamp_proto, clock_skew):
    robot_nsecs = bosdyn.util.timestamp_to_nsec(timestamp_proto)
    local_nsecs = robot_nsecs - bosdyn.util.timestamp_to_nsec(clock_skew)
    local_nsecs = max(local_nsecs, 0)  # Prevent negative timestamps
    local_timestamp = bosdyn.util.nsec_to_timestamp(local_nsecs)
    return local_timestamp


def convert_timestamp_from_local_to_robot(timestamp_proto, clock_skew):
    local_nsecs = bosdyn.util.timestamp_to_nsec(timestamp_proto)
    robot_nsecs = local_nsecs + bosdyn.util.timestamp_to_nsec(clock_skew)
    robot_timestamp = bosdyn.util.nsec_to_timestamp(robot_nsecs)
    return robot_timestamp


def is_motor_enabled(motor_power_state):
    return motor_power_state == robot_state_pb2.PowerState.STATE_ON


def is_estop_pressed(estop_state):
    return estop_state == robot_state_pb2.EStopState.STATE_ESTOPPED


def to_any(params):
    any_params = any_pb2.Any()
    any_params.Pack(params)
    return any_params


def try_cmd(name, cmd, params=None, kwargs=None):
    if kwargs is None:
        kwargs = dict()
    if params is None:
        params = []
    try:
        return [cmd(*params, **kwargs), True]
    except Exception as err:
        rospy.logerr("Failed to execute '" + name + "': " + str(err))
    return [None, False]


def wait_for_client(robot, service_name, timeout=None):
    """

    :type robot: bosdyn.client.sdk.Robot
    :type timeout: float
    :type service_name: str
    """
    success = False
    start_time = time.time()
    client = None
    sleep_time = 5
    while not success and (timeout is None or time.time() - start_time < timeout):
        try:
            client = robot.ensure_client(service_name)
        except Exception as e:
            rospy.loginfo("Waiting for service '{}'. Retrying in {} second. Error: {}".format(
                service_name, sleep_time, e))
            time.sleep(sleep_time)
        else:
            success = True
    return client
