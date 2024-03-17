from typing import List

from raubase_ros.wrappers import NodeWrapper
from .data import IOWrapper, Requirement
from .task import BaseTask


class BehaviourPlan(NodeWrapper):

    # =================================================================
    #                             Setup
    # =================================================================
    def __init__(self) -> None:
        super().__init__("behaviour")  # type: ignore
        self.__io = IOWrapper()
        self.__tasks: List[BaseTask] = []
        self._requirements: Requirement = Requirement.NONE
        self.__task_id = 0
        self.__task_started = False
        self.__done = False
        self.__initialized = False

        # Declare loop runner
        self.__timer = self.create_timer(
            self.declare_wparameter("loop_s", 0.1).get(),
            self.loop,
        )

    def __setup(self) -> None:
        self.get_logger().info("Initializing the requirements for the tasks")
        self.__io.init_requirements(self, self._requirements)
        self.__initialized = True

    # =================================================================
    #                             Task configuration
    # =================================================================
    def add_task(self, task: BaseTask) -> None:
        """
        Add a task to the sequence and gives him references to data and methods
        """
        self.get_logger().info(f"Plugging-in task {task.__class__.__name__}")
        self.__tasks.append(task)
        self.__tasks[-1].setup(self.__io.state, self.__io.controls)
        self._requirements = self._requirements | self.__tasks[-1].requirements()

    # =================================================================
    #                             Runtime
    # =================================================================
    def loop(self) -> None:
        """
        Run an iteration of the task
        """
        # If not initialized, launch setup
        if not self.__initialized:
            self.__setup()

        # Run if there's a task
        if len(self.__tasks) == 0 or self.__done:
            return

        # Update task
        if self.__task_started:
            self.__tasks[self.__task_id].loop()

            # Test for task end
            if self.__tasks[self.__task_id].can_stop():
                self.__task_id += 1
                self.__task_started = False
        elif self.__tasks[self.__task_id].can_start():
            self.__task_started = True

        # If done, reset state
        if self.__task_id >= len(self.__tasks):
            self.__done = True
            self.__task_id = 0
            self.__task_started = False
