from abc import abstractmethod
from rclpy.logging import get_logger

from .conditions import StartTaskCondition, StopTaskCondition
from .data import SharedData, ControlWrapper, Requirement


class BaseTask:
    """
    This class is an implementation of a task
    """

    # =================================================================
    #                        Task Properties
    # =================================================================

    @abstractmethod
    def requirements(self) -> Requirement:
        """
        List the requirements of this task (in term of inputs)
        """
        raise RuntimeError("Requirements method not implemented!")

    @abstractmethod
    def start_conditions(self) -> StartTaskCondition:
        """
        List the conditions to accomplish to start
        """
        raise RuntimeError("Start conditions method not implemented!")

    @abstractmethod
    def stop_conditions(self) -> StopTaskCondition:
        """
        List the conditions to accomplish to stop.
        """
        raise RuntimeError("Stop conditions method not implemented!")

    @abstractmethod
    def loop(self) -> None:
        """
        Run a loop iteration for this task
        """
        raise RuntimeError("Can start method not implemented!")

    # =================================================================
    #                             Runtime
    # =================================================================

    def setup(self, data: SharedData, control: ControlWrapper) -> None:
        """
        Setup the task
        """
        self.data = data
        self.control = control
        self.logger = get_logger(self.__class__.__name__)

    def can_start(self) -> bool:
        """
        Test whether the task can start.
        """
        return self.start_conditions().test()

    def can_stop(self) -> bool:
        """
        Test whether the task can stop.
        """
        return self.stop_conditions().test()
