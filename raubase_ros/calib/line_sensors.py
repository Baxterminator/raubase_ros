from enum import Enum, auto

import numpy as np

from raubase_ros.constants import ConfigFiles
from raubase_ros.config import ConfigFile
from raubase_ros.plan import BaseTask, Requirement
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
    StopTaskCondition,
)

COLOR_TO_CALIB = ["black", "white"]


class TaskStep(Enum):
    WAITING_FOR_LAUNCH = auto()
    LAUNCH_GATHERING = auto()
    PROCCESS_DATA = auto()


class LineSensor(BaseTask):
    """
    This calibration task provide a way to calibrate the line sensors (black & white).
    """

    N_DATA = 100

    def __init__(self, n_data: int = N_DATA) -> None:
        super().__init__()

        # Conditions
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

        # Inner Work
        self.cfg = ConfigFile(ConfigFiles.Names.LINE)
        self.n_data: int = n_data
        self.last_stamp = None
        self.buffer: np.ndarray = np.array([])
        self.buffer_size: int = 0
        self.state: TaskStep = TaskStep.WAITING_FOR_LAUNCH
        self.color: int = 0

    def requirements(self) -> Requirement:
        return Requirement.LINE

    def start_condition(self) -> StartTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition:
        return self.stop_cond

    # =========================================================================
    def resize_buffer(self) -> None:
        """
        Resize the internal buffer at the right size
        """
        self.buffer_size = len(self.data.line.data)
        self.buffer = np.zeros((self.buffer_size), dtype=float)

    # =========================================================================
    def loop(self) -> None:
        match self.state:
            case TaskStep.WAITING_FOR_LAUNCH:
                """
                Wait for input signal before launching calibration
                """

                # Initialize buffer
                self.resize_buffer()
                self.state = TaskStep.LAUNCH_GATHERING
                self.count = 0

                input(
                    f"Press [enter] to launch the {COLOR_TO_CALIB[self.color]} calibration"
                )

            case TaskStep.LAUNCH_GATHERING:
                """
                Gather the line sensor data into the buffer
                """
                # Wait for valid data message
                if len(self.buffer) == 0:
                    self.logger.warn(
                        "Waiting for line sensor reading ...",
                        throttle_duration_sec=1.0,
                    )
                    self.resize_buffer()
                    return

                # Prevent from reusing same message
                if (
                    self.last_stamp is not None
                    and self.last_stamp == self.data.line.stamp
                ):
                    return

                self.logger.info(
                    f"Calibrating {COLOR_TO_CALIB[self.color]} ... [{self.count:4d}/{self.n_data:4d}]"
                )

                # Add the values in the buffer
                self.last_stamp = self.data.line.stamp
                for i in range(self.buffer_size):
                    self.buffer[i] += self.data.line.data[i]
                    self.count += 1

                # When we got enough data, process them
                if self.count >= self.n_data:
                    self.state = TaskStep.PROCCESS_DATA

            case TaskStep.PROCCESS_DATA:
                """
                Compute the data
                """
                # Compute the mean
                self.buffer /= self.n_data

                self.logger.info(
                    f"{COLOR_TO_CALIB[self.color].title()} calibration results"
                )
                self.logger.info(self.buffer)

                # Replace in config files
                self.cfg.set_parameter(
                    f"{COLOR_TO_CALIB[self.color]}_calib", self.buffer
                )

                if self.color == len(COLOR_TO_CALIB):
                    self.stop = True
                else:
                    self.color += 1
                    self.state = TaskStep.WAITING_FOR_LAUNCH
