from abc import abstractmethod
from typing import List

import cv2 as cv
import numpy as np
from cv_bridge.core import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty
from raubase_ros.wrappers import NodeWrapper


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


class ImageProcessor(NodeWrapper):
    """
    Description of a image processor node.
    It automatically setup image requesting and definition of callbacks.
    It also implement a compressed image channel decryption if possible.
    """

    CAMERA_TRIGGER = "trigger"
    DEBUG_IMG = "debug"

    RAW_IMG_TOPIC = "image"
    COMPRESSED_IMG_TOPIC = "compressed"
    DEFAULT_QOS = 10

    MESSAGES_THROTTLE_S = 3

    IMG_ENCODING = "bgr8"

    # =================================================================
    #                          Initializations
    # =================================================================

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
        # Mode of operations
        self._use_compressed = self.declare_parameter("use_compressed", True).get()
        self.__on_demand = self.declare_parameter("on_demand", False)

        # Request parameters
        self.__timeout = self.declare_parameter("rqst_timeout", 0.05)
        self.__request_s = 1.0 / float(self.declare_parameter("request_hz", 10).get())

        self.DEBUG = self.declare_parameter("in_debug", True)

    def __subscribe_to_camera(self) -> None:
        """
        Subscribe to the camera output.
        """
        # Import converter
        self.cv_bridge = CvBridge()
        if self._use_compressed:
            self.__img_sub = self.create_subscription(
                CompressedImage,
                ImageProcessor.COMPRESSED_IMG_TOPIC,
                self._compressed_image_callback,
                ImageProcessor.DEFAULT_QOS,
            )
        else:
            self.__img_sub = self.create_subscription(
                Image,
                ImageProcessor.RAW_IMG_TOPIC,
                self._compressed_image_callback,
                ImageProcessor.DEFAULT_QOS,
            )

    def __define_debug_output(self) -> None:
        """
        Define debug output (e.g. debug image) for the node
        """
        self.__debug_pub = self.create_publisher(
            Image,
            ImageProcessor.DEBUG_IMG,
            ImageProcessor.DEFAULT_QOS,
        )

    def __initialize_img_requesting(self) -> None:
        """
        Initialize the image requesting client
        """
        self.__img_rec = True
        self.__last_t = self.get_clock().now()
        self.__req_client = self.create_publisher(
            Empty,
            ImageProcessor.CAMERA_TRIGGER,
            ImageProcessor.DEFAULT_QOS,
        )
        self.__trigger = self.create_subscription(
            Empty,
            ImageProcessor.CAMERA_TRIGGER,
            lambda m: self.__request_img(True),
            10,
        )
        self.__req_loop = self.create_timer(
            self.__request_s,
            lambda: self.__request_img(False),
        )

    def attach_processing_unit(self, unit: ImageProcessingUnit) -> None:
        """
        Attach a new processing unit to the processor
        """
        self.__units.append(unit)
        self.__units[-1].setup(self)

    # ================================================================
    #                 ROS Image Request & Callbacks
    # =================================================================

    def __request_img(self, manual_req: bool) -> None:
        """
        Ask the camera for a new image. Check whether the last image has been received.
        """
        # If "Manual" triggering
        if self.__on_demand.get():
            # If the request was not from an external trigger, skip it
            if not manual_req:
                return

            # If image not received and timeout not met, skip call (reduce frequency)
            if not self.__img_rec and self.since(self.__last_t) < self.__timeout.get():
                return

        # Else if in "continuous"
        else:
            # If image not received and timeout not met, skip call (reduce frequency)
            if not self.__img_rec and self.since(self.__last_t) < self.__timeout.get():
                return

        # Check for timeout
        if not self.__img_rec and self.__timeout.get() > self.since(self.__last_t):
            self.get_logger().warn("Timeout while waiting for image from camera!")
            self.__img_rec = True

        # If image already requested,
        self.get_logger().info(
            "Requesting image",
            throttle_duration_sec=ImageProcessor.MESSAGES_THROTTLE_S,
        )
        self.__img_rec = False
        self.__last_t = self.get_clock().now()
        self.__req_client.publish(Empty())

    def _compressed_image_callback(self, img: CompressedImage) -> None:
        """
        Receive a new compressed image and process it
        """
        self.process_img(
            self.cv_bridge.compressed_imgmsg_to_cv2(
                img,
                desired_encoding=ImageProcessor.IMG_ENCODING,
            )
        )

    def _raw_image_callback(self, img: Image) -> None:
        """
        Receive a new image and process it
        """
        self.__img_rec = True
        self.process_img(
            self.cv_bridge.imgmsg_to_cv2(
                img,
                desired_encoding=ImageProcessor.IMG_ENCODING,
            )
        )

    def publish_debug(self, img: cv.typing.MatLike) -> None:
        """
        Take the debug image and publish it
        """
        self.__debug_pub.publish(
            self.cv_bridge.cv2_to_imgmsg(
                img,
                encoding=ImageProcessor.IMG_ENCODING,
            )
        )

    # ================================================================
    #                 Processing
    # =================================================================

    def process_img(self, img: CVImage) -> None:
        """
        Process the image.
        """
        self.get_logger().info(
            "Process image",
            throttle_duration_sec=ImageProcessor.MESSAGES_THROTTLE_S,
        )

        # Prepare debug image
        debug_img = None
        if self.DEBUG.get():
            debug_img = img.copy()

        # Run
        for unit in self.__units:
            unit.run(img, self.DEBUG.get(), debug_img)

        self.__img_rec = True

        # Send debug image if needed
        if self.DEBUG and debug_img is not None:
            self.publish_debug(debug_img)
