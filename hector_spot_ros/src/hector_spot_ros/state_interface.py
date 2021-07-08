from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.local_grid import LocalGridClient
from hector_spot_ros.periodic_tasks import PeriodicMetrics, PeriodicRobotState, PeriodicLocalGrid


class StateInterface:
    def __init__(self, robot):
        assert robot is not None
        self._robot = robot
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._periodic_metrics = PeriodicMetrics(self._robot_state_client, 0.2)
        self._periodic_robot_state = PeriodicRobotState(self._robot_state_client, 20.0)

        self._local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)  # type: LocalGridClient
        self._periodic_local_grid = PeriodicLocalGrid(self._local_grid_client, 1)

    def start(self):
        self._periodic_metrics.start()
        self._periodic_robot_state.start()
        self._periodic_local_grid.start()

    def stop(self):
        self._periodic_metrics.stop()
        self._periodic_robot_state.stop()
        self._periodic_local_grid.stop()

    def register_robot_metrics_callback(self, cb):
        self._periodic_metrics.register_callback(cb)

    def register_robot_state_callback(self, cb):
        self._periodic_robot_state.register_callback(cb)

    def list_estop_sources(self):
        robot_state = self._robot_state_client.get_robot_state()
        return [estop_state.name for estop_state in robot_state.estop_states]

    def register_local_grid_callback(self, cb):
        self._periodic_local_grid.register_callback(cb)

    def get_local_grid_types(self):
        return [local_grid_type.name for local_grid_type in self._local_grid_client.get_local_grid_types()]

    def set_requested_grid_types(self, grid_types):
        self._periodic_local_grid.requested_local_grid_type_names = grid_types


