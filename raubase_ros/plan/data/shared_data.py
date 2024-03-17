from dataclasses import dataclass, field
from typing import Dict
from raubase_msgs.msg import DataEncoder, DataDistance
from raubase_msgs.msg import ResultYolo, ResultArUco, ResultOdometry
from sensor_msgs.msg import CompressedImage
from time import perf_counter


@dataclass
class SharedData:
    """
    Raw structure to store the states and sensors data of the robot.
    """

    # Sensors
    encoders: DataEncoder = DataEncoder()
    ir: Dict[int, DataDistance] = field(
        default_factory=lambda: {1: DataDistance(), 2: DataDistance()}
    )
    odometry: ResultOdometry = ResultOdometry()

    # Camera / Images
    last_img: CompressedImage = CompressedImage()
    last_yolo: ResultYolo = ResultYolo()
    last_aruco: ResultArUco = ResultArUco()

    # Extra
    distance: float = 0  # Depends on odometry
    time_origin: float = 0  # Since when do we start the lapsed time computation
    time_elapsed: float = 0  # The result of the elasped time
    task_time: float = 0  # The time the task has been started

    def reset_distance(self) -> None:
        """
        Reset the internal distance computation.
        """
        self.distance = 0

    def reset_time(self) -> None:
        """
        Reset the internal time counter.
        """
        self.time_origin = perf_counter()
        self.time_elapsed = 0
