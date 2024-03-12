from abc import abstractmethod
from typing import List

import cv2 as cv
import numpy as np
from cv_bridge.core import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from raubase_msgs.srv import AskCameraImage


def toBGR(hex: str) -> np.ndarray:
    return np.array(list(int(hex[i : i + 2], 16) for i in (4, 2, 0)))


CVImage = cv.typing.MatLike


class ImageProcessingUnit:
    """
    Processing unit for the image processor
    """

    @abstractmethod
    def setup(self, node: Node) -> None:
        pass

    @abstractmethod
    def run(
        self,
        img: CVImage,
        print_debug: bool = False,
        debug_img: CVImage | None = None,
    ) -> None:
        pass


class ImageProcessorInterface(Node):
    """
    Description of a image processor node.
    It automatically setup image requesting and definition of callbacks.
    It also implement a compressed image channel decryption if possible.
    """

    CAMERA_SERVICE = "/camera/get_image"
    DEBUG_IMG = "debug"

    RAW_IMG_TOPIC = "image"
    COMPRESSED_IMG_TOPIC = "compressed"
    DEFAULT_QOS = 10

    SRV_TIMEOUT = 1.0
    REQUESTING_THROTTLE_S = 3

    IMG_ENCODING = "bgr8"

    def __init__(self, name: str) -> None:
        super().__init__(name)  # type: ignore

        # Initialize all parameters
        self.__parameters_declaration()

        # Camera subscriptions
        self.__subscribe_to_camera()
        self.__define_debug_output()
        self.__initialize_img_requesting()

        self.__units: List[ImageProcessingUnit] = []

    def __parameters_declaration(self) -> None:
        """
        Declare the parameters of this node
        """
        self._use_compressed = (
            self.declare_parameter("use_compressed", True)
            .get_parameter_value()
            .bool_value
        )
        self.__camera_srv = (
            self.declare_parameter("camera_srv", ImageProcessorInterface.CAMERA_SERVICE)
            .get_parameter_value()
            .string_value
        )
        self.__request_s = 1 / float(
            self.declare_parameter("request_hz", 10).get_parameter_value().integer_value
        )
        self.DEBUG = (
            self.declare_parameter("in_debug", True).get_parameter_value().bool_value
        )

    def __subscribe_to_camera(self) -> None:
        """
        Subscribe to the camera output.
        """
        # Import converter
        self.cv_bridge = CvBridge()
        if self._use_compressed:
            self.__img_sub = self.create_subscription(
                CompressedImage,
                ImageProcessorInterface.COMPRESSED_IMG_TOPIC,
                self._compressed_image_callback,
                ImageProcessorInterface.DEFAULT_QOS,
            )
        else:
            self.__img_sub = self.create_subscription(
                Image,
                ImageProcessorInterface.RAW_IMG_TOPIC,
                self._compressed_image_callback,
                ImageProcessorInterface.DEFAULT_QOS,
            )

    def __define_debug_output(self) -> None:
        """
        Define debug output (e.g. debug image) for the node
        """
        if self.DEBUG:
            self.__debug_pub = self.create_publisher(
                Image,
                ImageProcessorInterface.DEBUG_IMG,
                ImageProcessorInterface.DEFAULT_QOS,
            )

    def __initialize_img_requesting(self) -> None:
        """
        Initialize the image requesting client
        """
        self.__img_requested = False
        self.__req_img = AskCameraImage.Request()
        self.__req_client = self.create_client(AskCameraImage, self.__camera_srv)
        self.__req_loop = self.create_timer(self.__request_s, self.__request_img)

    def attach_processing_unit(self, unit: ImageProcessingUnit) -> None:
        """
        Attach a new processing unit to the processor
        """
        self.__units.append(unit)
        self.__units[-1].setup(self)

    def __request_img(self) -> None:
        """
        Ask the camera for a new image
        """
        if self.__img_requested:
            return

        # Get image
        if not self.__req_client.wait_for_service(ImageProcessorInterface.SRV_TIMEOUT):
            self.get_logger().warn("Waiting for for camera service ...")
            return

        self.get_logger().info(
            "Requesting image",
            throttle_duration_sec=ImageProcessorInterface.REQUESTING_THROTTLE_S,
        )
        self.__img_requested = True
        self.__req_client.call_async(self.__req_img)

    def _compressed_image_callback(self, img: CompressedImage) -> None:
        """
        Receive a new compressed image and process it
        """
        self.__img_requested = False
        self.process_img(
            self.cv_bridge.compressed_imgmsg_to_cv2(
                img, desired_encoding=ImageProcessorInterface.IMG_ENCODING
            )
        )

    def _raw_image_callback(self, img: Image) -> None:
        """
        Receive a new image and process it
        """
        self.__img_requested = False
        self.process_img(
            self.cv_bridge.imgmsg_to_cv2(
                img, desired_encoding=ImageProcessorInterface.IMG_ENCODING
            )
        )

    def publish_debug(self, img: cv.typing.MatLike) -> None:
        """
        Take the debug image and publish it
        """
        self.__debug_pub.publish(
            self.cv_bridge.cv2_to_imgmsg(
                img, encoding=ImageProcessorInterface.IMG_ENCODING
            )
        )

    def process_img(self, img: CVImage) -> None:
        """
        Process the image.
        """
        self.get_logger().info("Process image")

        # Prepare debug image
        debug_img = None
        if self.DEBUG:
            debug_img = img.copy()

        # Run
        for unit in self.__units:
            unit.run(img, self.DEBUG, debug_img)

        # Send debug image if needed
        if self.DEBUG and debug_img is not None:
            self.publish_debug(debug_img)
