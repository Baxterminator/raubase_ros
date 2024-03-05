#!/bin/python3
from typing import Tuple
import rclpy
from rclpy.node import Node
from raubase_msgs.srv import AskCameraImage
import cv2 as cv
import cv2.aruco as ArUco
import numpy as np
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image


def toBGR(hex: str) -> np.ndarray:
    return np.array(list(int(hex[i : i + 2], 16) for i in (4, 2, 0)))


class ImageProcessor(Node):

    KERNEL = np.ones((1, 1), np.uint8)
    CAMERA_SERVICE = "/camera/get_image"
    IMG_OUTPUT = "/camera/analysis"

    def __init__(self) -> None:
        super().__init__("ImageProcessor")  # type: ignore
        self.timer = self.create_timer(0.5, self.image_process)
        self.client = self.create_client(AskCameraImage, ImageProcessor.CAMERA_SERVICE)
        self.img_req = AskCameraImage.Request()
        self.bridge = CvBridge()

        self._send_draw = (
            self.declare_parameter("img_result", True).get_parameter_value().bool_value
        )
        self._result_img_pub = self.create_publisher(
            Image, ImageProcessor.IMG_OUTPUT, 10
        )

        # OpenCV parameters
        self._lower_ball = toBGR(
            self.declare_parameter("lower_orange", "64640A")  # In RGB
            .get_parameter_value()
            .string_value
        )
        self._upper_ball = toBGR(
            self.declare_parameter("upper_orange", "FFFF14")  # In RGB
            .get_parameter_value()
            .string_value
        )
        self._aruco_detector = ArUco.ArucoDetector(
            ArUco.getPredefinedDictionary(ArUco.DICT_4X4_50),
            ArUco.DetectorParameters(),
        )

    def image_process(self):
        # Get image
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for for camera service ...")
        resp: AskCameraImage.Response = self.client.call(self.img_req)

        # Process image
        frame: cv.typing.MatLike = self.bridge.imgmsg_to_cv2(resp.image)
        bx, by, br = self.detect_golf_ball(frame)
        cnrs, ids, rjcts = self.detect_aruco(frame)

        if self._send_draw:
            # Draw golf ball
            cv.circle(frame, (bx, by), br, (0, 255, 255))

            # ArUco
            if ids is not None:
                ArUco.drawDetectedMarkers(frame, cnrs, ids)
            ArUco.drawDetectedMarkers(frame, rjcts, None, (255, 255, 20))

            self._result_img_pub.publish(self.bridge.cv2_to_imgmsg(frame))

    def detect_golf_ball(self, frame: cv.typing.MatLike) -> Tuple[int, int, int]:
        """
        Detect a golf ball in the frame.

        Returns: (x, y, r) of the detected ball
            or   (-1,-1,-1) if no ball was detected
        """
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.dilate(
            cv.inRange(hsv, self._lower_ball, self._upper_ball),
            ImageProcessor.KERNEL,
            iterations=2,
        )
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv.contourArea)
            (x, y), radius = cv.minEnclosingCircle(c)
            if radius > 10:  # RADIUS OF THE BALL
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                return (int(x), int(y), int(radius))
        return (-1, -1, -1)

    def detect_aruco(self, frame: cv.typing.MatLike):
        """
        Detect an ARuCo code in the frame"""
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        return self._aruco_detector.detectMarkers(gray)


# =============================================================================
# ROS Launching
# =============================================================================


def main(args=None):
    rclpy.init(args=args)

    proc = ImageProcessor()

    rclpy.spin(proc)

    proc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
