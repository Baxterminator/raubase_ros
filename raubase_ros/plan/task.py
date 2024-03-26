from abc import abstractmethod
from rclpy.logging import get_logger
import numpy as np

from .conditions import (
    StartTaskCondition,
    StopTaskCondition,
    FlowTaskCondition,
    FollowPreviousTask,
    Never,
)
from .data import SharedData, ControlWrapper, Requirement


def close_to(x: float, y: float, eps: float = 5e-3) -> bool:
    return np.abs(x - y) < eps


class BaseTask:
    """
    This class is an implementation of a task
    """

    def __init__(self, name: str | None = None) -> None:
        self.name = name
        self.logger = get_logger(
            self.__class__.__name__ if (name is None) or (name == "") else name
        )

    # =================================================================
    #                        Task Properties
    # =================================================================

    @abstractmethod
    def requirements(self) -> Requirement:
        """
        List the requirements of this task (in term of inputs)
        """
        self.logger.error(
            f"Requirements for task {self.__class__.__name__} not implemented!"
        )
        raise NotImplementedError(
            f"Requirements for task {self.__class__.__name__} not implemented!"
        )

    @abstractmethod
    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        """
        List the conditions to accomplish to start
        """
        self.logger.error(
            f"Start condition for task {self.__class__.__name__} not implemented!"
        )
        raise NotImplementedError(
            f"Start condition for task {self.__class__.__name__} not implemented!"
        )

    @abstractmethod
    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        """
        List the conditions to accomplish to stop.
        """
        self.logger.error(
            f"Stop condition for task {self.__class__.__name__} not implemented!"
        )
        raise NotImplementedError(
            f"Stop condition for task {self.__class__.__name__} not implemented!"
        )

    @abstractmethod
    def loop(self) -> None:
        """
        Run a loop iteration for this task
        """
        self.logger.error(
            f"Runtime loop for task {self.__class__.__name__} not implemented!"
        )
        raise NotImplementedError(
            f"Runtime loop for task {self.__class__.__name__} not implemented!"
        )

    # =================================================================
    #                             Runtime
    # =================================================================

    def setup(self, data: SharedData, control: ControlWrapper) -> None:
        """
        Setup the task
        """
        self.data = data
        self.control = control

    def can_start(self) -> bool:
        """
        Test whether the task can start.
        """
        return self.start_condition().test()

    def can_stop(self) -> bool:
        """
        Test whether the task can stop.
        """
        return self.stop_condition().test()


class DefaultTask(BaseTask):
    """
    Task that will be run as a default action no other task is running.
    """

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return Never()
