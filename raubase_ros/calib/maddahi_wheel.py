from enum import Enum, auto

from raubase_ros.plan import BaseTask
from raubase_ros.plan.conditions import (
    FollowPreviousTask,
    OnValue,
    StartTaskCondition,
    StopTaskCondition,
)
from raubase_ros.plan.data import Requirement
from raubase_msgs.msg import CmdMove
from time import perf_counter
import numpy as np


class TaskStep(Enum):
    WAITING_FOR_LAUNCH = auto()
    LAUNCH_FORWARD = auto()
    GOING_FORWARD = auto()
    STOP_FORWARD = auto()
    PROCCESS_DATA = auto()


class WheelMaddahi(BaseTask):
    """
    This calibration task provide a way to calibrate the wheels by moving along a straight line and
    measuring the difference in position in the end.
    """

    DEFAULT_DIST_TO_GO = 2.0
    DEFAULT_ROBOT_VEL = 0.7
    DEFAULT_WHEEL_D = 0.146

    def __init__(
        self,
        dist_to_go: float = DEFAULT_DIST_TO_GO,
        robot_vel: float = DEFAULT_ROBOT_VEL,
        rwheel_diam: float = DEFAULT_WHEEL_D,
        lwheel_diam: float = DEFAULT_WHEEL_D,
    ) -> None:
        super().__init__()

        # Conditions
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

        # Inner Work
        self.dist_to_go = dist_to_go
        self.robot_vel = robot_vel
        self.state = TaskStep.WAITING_FOR_LAUNCH
        self.encoders_0 = [0.0, 0.0]
        self.encoders_1 = [0.0, 0.0]
        self.cmd_launched = perf_counter()
        self.wheels_d = [lwheel_diam, rwheel_diam]

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ENCODERS | Requirement.ODOMETRY

    def start_conditions(self) -> StartTaskCondition:
        return FollowPreviousTask()

    def stop_conditions(self) -> StopTaskCondition:
        return self.stop_cond

    # =========================================================================
    @staticmethod
    def make_motor_msg(vel: float) -> CmdMove:
        out = CmdMove()
        out.move_type = CmdMove.CMD_V_ANGLE
        out.velocity = float(vel)
        out.heading = 0
        return out

    def loop(self) -> None:
        match self.state:
            case TaskStep.WAITING_FOR_LAUNCH:
                """
                Stop the motors (if they were moving), and wait for a used input signal
                """
                self.control.set_vel_h(0, 0)

                input("Press [enter] for launching the calibration")
                self.state = TaskStep.LAUNCH_FORWARD

            case TaskStep.LAUNCH_FORWARD:
                """
                Launch the movement in a straight line (both motors at the same voltage)
                """
                self.encoders_0 = [self.data.encoders.left, self.data.encoders.right]
                self.control.set_vel_h(self.robot_vel, 0)
                self.cmd_launched = perf_counter()
                self.state = TaskStep.GOING_FORWARD

            case TaskStep.GOING_FORWARD:
                """
                Robot should be moving, detect if he really moved or not.
                Also test if the robot has done its movement.
                """
                # Verify that robot has moved
                if (
                    np.abs(self.data.distance) < 0.01
                    and (perf_counter() - self.cmd_launched) > 2
                ):
                    self.logger.warn("Robot not moving, retrying!")
                    self.state = TaskStep.LAUNCH_FORWARD
                    return

                # Verify the robot has gone the right distance
                if self.data.distance >= self.dist_to_go:
                    self.encoders_1 = [
                        self.data.encoders.left,
                        self.data.encoders.right,
                    ]
                    self.state = TaskStep.STOP_FORWARD

            case TaskStep.STOP_FORWARD:
                """
                The calibration movement is done, stop the robot
                """
                self.control.set_vel_h(0, 0)
                self.state = TaskStep.PROCCESS_DATA

            case TaskStep.PROCCESS_DATA:
                """
                Compute the data
                """
                # Wait for user delta measurement:
                dx = float(input("Input dx = "))
                dy = float(input("Input dy = "))

                denc_left = float(self.encoders_1[0] - self.encoders_0[0])
                denc_right = float(self.encoders_1[1] - self.encoders_0[1])

                Flat = denc_right / denc_left
                Flon = self.data.distance / np.sqrt(
                    np.power(self.data.distance + dx, 2) + np.power(dy, 2)
                )

                self.logger.info("Calibration results :")
                self.logger.info(f"Distance = {self.data.distance}")
                self.logger.info(f"ΔEnc Right = {denc_right:.0f} pulses")
                self.logger.info(f"ΔEnc Left = {denc_left:.0f} pulses")
                self.logger.info(f"Flat = {Flat:.5f}")
                self.logger.info(f"Flon = {Flon:.5f}")
                if self.wheels_d[0] != 0.0 or self.wheels_d[1] != 0.0:
                    self.logger.info(f"∅ right wheel {self.wheels_d[1]/Flon:.4f}")
                    self.logger.info(f"∅ left wheel {self.wheels_d[0]/Flat/Flon:.4f}")

                self.stop = True
