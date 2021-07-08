import cv2
import numpy as np

from bosdyn.client.image import ImageClient
from bosdyn.api import image_pb2
from bosdyn.client import TimedOutError
from bosdyn.client.frame_helpers import get_a_tform_b, BODY_FRAME_NAME

from hector_spot_ros.periodic_tasks import PeriodicImageCapture


def convert_image_to_cv2(image_response):
    # handle pixel format
    pixel_format = image_response.shot.image.pixel_format
    depth_image = False
    if pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
        depth_image = True
    else:
        dtype = np.uint8

    # decode to cv2 image
    img = np.fromstring(image_response.shot.image.data, dtype=dtype)
    if image_response.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(image_response.shot.image.rows, image_response.shot.image.cols)
    else:
        img = cv2.imdecode(img, -1)

    # depth images are given as uint16 with mm values
    # convert to float32 in m
    if depth_image:
        img = img.astype(np.float32)
        img = img / 1000.0

    return img


def get_sensor_transform(image_response):
    image_frame = image_response.shot.frame_name_image_sensor
    transforms_snapshot = image_response.shot.transforms_snapshot
    pose = get_a_tform_b(transforms_snapshot, BODY_FRAME_NAME, image_frame)
    return pose


class CameraInterface:
    DEFAULT_CAM_RATE = 10.0

    def __init__(self, robot):
        assert robot is not None
        self._robot = robot
        self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
        self._periodic_image_capture = PeriodicImageCapture(self, self.DEFAULT_CAM_RATE)

    def start(self):
        self._periodic_image_capture.start()

    def stop(self):
        self._periodic_image_capture.stop()

    def list_image_sources(self):
        try:
            sources = self._image_client.list_image_sources(timeout=1)
        except TimedOutError as err:
            print("Timed out listing image sources with error:", err)
            return []
        return [source.name for source in sources]

    def get_images(self, cam_names, timeout=1):
        # Get image data from robot
        image_responses = self._image_client.get_image_from_sources(cam_names, timeout=timeout)
        imgs = []
        for image_reponse in image_responses:
            stamp = image_reponse.shot.acquisition_time
            image = convert_image_to_cv2(image_reponse)
            transform = get_sensor_transform(image_reponse)
            source = image_reponse.source
            imgs.append([stamp, image, transform, source])
        return imgs

    def register_image_callback(self, cb):
        self._periodic_image_capture.register_callback(cb)

    def set_requested_image_sources(self, image_sources):
        self._periodic_image_capture.requested_sources = image_sources

    def set_image_rate(self, source, rate):
        self._periodic_image_capture.desired_period[source] = 1 / rate
