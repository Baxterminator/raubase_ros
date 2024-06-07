from enum import Enum

import numpy as np
from geometry_msgs.msg import Vector3
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo

logger = get_logger("utils")


class DimType(Enum):
    WIDTH = 0
    HEIGHT = 1
    DIAM = 2


def cam_px2cam_pos(
    cam_info: CameraInfo,
    x: float,
    y: float,
    dim_px: float,
    dim_m: float,
    dim_type: DimType,
) -> np.ndarray:
    """
    Return the equivalent position in the camera frame from the position in pixel in the image.
    """
    # Check for non valid input:
    if dim_px <= 0:
        raise ValueError("Width in px of an object can't be <= 0")
    if dim_m <= 0:
        raise ValueError("Width in m of an object can't be <= 0")
    if cam_info.k[0] <= 0 or cam_info.k[4] <= 0:
        raise ValueError("Invalid focal length for matrix K (should be >=0)")

    # Compute depth
    match dim_type:
        case DimType.WIDTH:
            fm = cam_info.k[0]
        case DimType.HEIGHT:
            fm = cam_info.k[4]
        case _:
            fm = (cam_info.k[0] + cam_info.k[4]) / 2
    Zc = (fm * dim_m) / dim_px

    cam_info.p

    Xc = (x - cam_info.k[2]) * Zc / cam_info.k[0]
    Yc = (y - cam_info.k[5]) * Zc / cam_info.k[4]
    return np.array([Xc, Yc, Zc])


def cam_pos2rob_pos(
    robot_trans: Vector3,
    robot_rot: np.ndarray,
    Xc: np.ndarray,
) -> np.ndarray:
    """
    Transform the camera coordinates into the robot frame.
    Parameters:
      - robot_trans / robot_rot: transform from the robot frame to the camera frame
        | Xc Yc Zc Pc |
        | 0  0  0  1  |
        with Xc, Yc, Zc begin the axes of the camera frame described in the robot frame
      - Xc the position of the point in the camera frame
    """
    return robot_rot @ np.array(
        [
            Xc[0] + robot_trans.x,
            Xc[1] + robot_trans.y,
            Xc[2] + robot_trans.z,
        ],
        dtype=np.float64,
    )
