from dataclasses import dataclass, field
from typing import Dict
from raubase_msgs.msg import DataEncoder, DataDistance
from raubase_msgs.msg import ResultYolo, ResultArUco
from sensor_msgs.msg import CompressedImage


@dataclass
class SharedData:
    """
    Raw structure to store the states and sensors data of the robot.
    """

    # Sensors
    encoders: DataEncoder = DataEncoder()
    distance: Dict[int, DataDistance] = field(
        default_factory=lambda: {1: DataDistance(), 2: DataDistance()}
    )

    # Camera / Images
    last_img: CompressedImage = CompressedImage()
    last_yolo: ResultYolo = ResultYolo()
    last_aruco: ResultArUco = ResultArUco()
