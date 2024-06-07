from time import perf_counter
from typing import List

from raubase_ros.wrappers import NodeWrapper
from .data import IOWrapper, Requirement
from .task import BaseTask, DefaultTask
from raubase_ros.config import get_top_namespace
from raubase_msgs.msg import DataGPIO


class BehaviourPlan(NodeWrapper):

    # =================================================================
    #                             Setup
    # =================================================================
    def __init__(
        self,
        name="behavior",
        default_loop_s=0.05,
        namespace: str = get_top_namespace(),
        require_user_input: bool = False,
    ) -> None:
        super().__init__(name, namespace=namespace)  # type: ignore
        self.__io = IOWrapper()

        # Tasks related
        self.__tasks: List[BaseTask] = []
        self._requirements: int = Requirement.NONE
        self.__default_task: DefaultTask | None = None

        # Runtime variables
        self.__task_id = 0
        self.__task_started = False
        self.done = False
        self.running = False
        self.__initialized = False
        self.__require_input = require_user_input

        # Extra variables
        self.__task_time_origin = 0

        # Declare loop runner
        self.__timer = self.create_timer(
            self.declare_wparameter("loop_s", default_loop_s).get(),
            self.loop,
        )

        # Declare start and stop button
        self.__start = self.create_subscription(
            DataGPIO, "start", self.start_callback, 10
        )
        self.__stop = self.create_subscription(DataGPIO, "stop", self.stop_callback, 10)

    def __setup(self) -> None:
        """
        Setup all requirements needed for the tasks
        """
        if self._requirements != Requirement.NONE:
            self.get_logger().info("Initializing the requirements for the tasks")
            self.__io.init_requirements(self, self._requirements)
        self.__initialized = True

    def reinitialize(self) -> None:
        """
        Launch a reinitialization of the requirements
        """
        self.__initialized = False

    def reset_state(self) -> None:
        """
        Reset the state of the task list (so begins from the start again)
        """
        self.__task_id = 0
        self.__task_started = False
        self.done = False

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

    def clear_tasks(self) -> None:
        """
        Clear the tasks of this plan.
        """
        self.__tasks = []
        self.reset_state()

    def set_default_task(self, task: DefaultTask) -> None:
        """
        Set the default task to run when no other task is running.
        """
        self.__default_task = task
        self._requirements |= self.__default_task.requirements()

    # =================================================================
    #                             Runtime
    # =================================================================

    def start_callback(self, msg: DataGPIO) -> None:
        """
        Start button state callback
        """
        if not self.running and msg.value:
            self.get_logger().info("Starting behaviour plan!")
            self.running = True

    def stop_callback(self, msg: DataGPIO) -> None:
        """
        Stop button state callback
        """
        if self.running and msg.value:
            self.get_logger().warn("Stopping behaviour plan!")
            self.running = False
            self.reset_state()

    def loop(self) -> None:
        """
        Run an iteration of the task
        """
        # If not initialized, launch setup
        if not self.__initialized:
            self.__setup()

        # Run if there's a task
        if len(self.__tasks) == 0 or self.done or not self.running:
            return

        # Update task
        if self.__task_started:
            self.get_logger().info(
                f"Running task {self.__tasks[self.__task_id].__class__.__name__}",
                throttle_duration_sec=0.5,
            )
            self.__io.state.task_time = perf_counter() - self.__task_time_origin
            self.__io.state.time_elapsed = perf_counter() - self.__io.state.time_origin
            self.__tasks[self.__task_id].loop()

            # Test for task end
            if self.__tasks[self.__task_id].can_stop():
                self.get_logger().info(
                    f"Stopping task {self.__tasks[self.__task_id].__class__.__name__}"
                )
                self.__task_id += 1
                self.__task_started = False
        elif self.__tasks[self.__task_id].can_start():
            self.get_logger().info(
                f"Starting task {self.__tasks[self.__task_id].__class__.__name__}"
            )
            self.__task_started = True
            self.__io.state.task_time = perf_counter()
            self.__task_time_origin = perf_counter()
            self.__io.state.time_origin = 0
            self.__io.state.time_elapsed = 0
            self.__io.state.reset_distance()
            if self.__require_input:
                input("Press [enter] for next task...")
        elif self.__default_task is not None:
            self.get_logger().info(f"Running default task")
            self.__default_task.loop()

        # If done, reset state
        if self.__task_id >= len(self.__tasks):
            self.get_logger().info(f"Task list done!")
            self.reset_state()
            self.done = True
