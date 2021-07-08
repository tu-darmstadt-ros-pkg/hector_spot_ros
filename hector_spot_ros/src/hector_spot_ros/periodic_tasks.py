import rospy
import abc
import time
import threading

from bosdyn.client import ResponseError, RpcError, TimedOutError


class PeriodicGRPCTaskWithCallback:
    def __init__(self, name, client, update_rate):
        self._name = name
        self._client = client
        self._observer_callbacks = []
        self._period_sec = 1/update_rate
        self._last_call = 0
        self.running = False
        self._stop_signal = threading.Event()
        self._thread = threading.Thread(target=self.update)

    def start(self):
        self.running = True
        self._stop_signal.clear()
        self._thread.start()

    def stop(self):
        self.running = False
        self._stop_signal.set()

    def update(self):
        while self.running:
            now_sec = time.time()
            if self._should_query(now_sec):
                self._last_call = now_sec
                try:
                    result = self._start_query()
                except TimedOutError as error:
                    self._handle_timeout(error)
                except (ResponseError, RpcError) as error:
                    self._handle_error(error)
                else:
                    self._handle_result(result)
            exec_time = time.time() - now_sec
            self._stop_signal.wait(self._period_sec - exec_time)

    def register_callback(self, cb):
        self._observer_callbacks.append(cb)

    @abc.abstractmethod
    def _start_query(self):
        """Override to start grpc query and return client responses"""

    def _handle_result(self, result):
        for cb in self._observer_callbacks:
            cb(result)

    def _handle_error(self, exception):
        rospy.logerr_throttle(10, "Failed to retrieve '{}' with error: {}".format(self._name, exception))

    def _handle_timeout(self, exception):
        rospy.logerr("'{}' timed out with error: {}".format(self._name, exception))

    def _should_query(self, now_sec):
        return len(self._observer_callbacks) != 0


class PeriodicMetrics(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate):
        super(PeriodicMetrics, self).__init__("robot_metrics", client, update_rate)

    def _start_query(self):
        return self._client.get_robot_metrics(timeout=1)


class PeriodicRobotState(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate):
        super(PeriodicRobotState, self).__init__("robot_state", client, update_rate)

    def _start_query(self):
        return self._client.get_robot_state(timeout=1)


class PeriodicImageCapture(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate):
        super(PeriodicImageCapture, self).__init__("image_capture", client, update_rate)
        self.requested_sources = []
        self._desired_sources = []
        self.desired_period = dict()
        self._last_capture = dict()

    def _start_query(self):
        return self._client.get_images(self._desired_sources, timeout=1)

    def _should_query(self, now_sec):
        self._update_desired_sources()
        return super(PeriodicImageCapture, self)._should_query(now_sec) and len(self._desired_sources) > 0

    def _update_desired_sources(self):
        self._desired_sources = []
        now = time.time()
        for source in self.requested_sources:
            take_source = False
            # Take if no desired period specified
            desired_period = self.desired_period.get(source, None)
            if desired_period is None:
                take_source = True
            else:
                # Take if this is the first capture
                last_capture = self._last_capture.get(source, None)
                if last_capture is None:
                    take_source = True
                else:
                    # Else check the time stamp of the last capture
                    if now - last_capture >= desired_period:
                        take_source = True

            if take_source:
                self._desired_sources.append(source)
                self._last_capture[source] = now


class PeriodicRobotCommandFeedback(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate, robot_command_id):
        super(PeriodicRobotCommandFeedback, self).__init__("robot_command_feedback", client, update_rate)
        self._robot_command_id = robot_command_id

    def _start_query(self):
        return self._client.robot_command_feedback(self._robot_command_id, timeout=1)

    def _handle_error(self, exception):
        self._handle_result(None)
        super(PeriodicRobotCommandFeedback, self)._handle_error(exception)


class PeriodicLocalGrid(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate):
        super(PeriodicLocalGrid, self).__init__("local_grid", client, update_rate)
        self.requested_local_grid_type_names = []

    def _start_query(self):
        return self._client.get_local_grids(self.requested_local_grid_type_names, timeout=1)

    def _should_query(self, now_sec):
        return super(PeriodicLocalGrid, self)._should_query(now_sec) and len(self.requested_local_grid_type_names) != 0


class PeriodicLocalizationState(PeriodicGRPCTaskWithCallback):
    def __init__(self, client, update_rate):
        super(PeriodicLocalizationState, self).__init__("localization state", client, update_rate)
        self.enabled = False

    def _start_query(self):
        return self._client.get_global_localization()

    def _should_query(self, now_sec):
        return super(PeriodicLocalizationState, self)._should_query(now_sec) and self.enabled
