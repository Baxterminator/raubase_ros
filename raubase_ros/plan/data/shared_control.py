from dataclasses import dataclass
from enum import Enum, unique
from typing import Callable
from raubase_msgs.msg import (
    CmdMove,
    CmdLineFollower,
    SetControllerInput,
    CmdServoPosition,
)


@dataclass
class SharedControl:
    """
    Raw structure to stored the callbacks for all available commands in the behaviour task.

    If the publisher has not been initialize, the callback is of value None.
    """

    set_cmd: Callable[[SetControllerInput], None] | None = None
    follow_edge: Callable[[CmdLineFollower], None] | None = None
    set_vel: Callable[[CmdMove], None] | None = None
    set_servo: Callable[[CmdServoPosition], None] | None = None


@unique
class ControllerMode(Enum):
    """
    Enumeration of the "normal" methods of controlling the controller.
    """

    MANUAL = SetControllerInput.MANUAL
    PLAN = SetControllerInput.PLAN
    EDGE = SetControllerInput.EDGE


class ControlWrapper:
    """
    Higher level from the raw publishing callbacks for a better user-friendly API.
    """

    USE_UNDEFINED = "Using uninitialized control command!"

    def __init__(self, obj: SharedControl) -> None:
        self._cmd = obj
        self.mode = ""

    # =========================================================================
    # Movement
    # =========================================================================
    def controller_input(self, mode: ControllerMode | str, force: bool = True):
        """
        Set the input type of the velocity controller.

        This command force the exclusitivity of the control to the given input mode.

        Param:
            - mode, the name of the mode
        """

        # Need to make the command ?
        if self._cmd.set_cmd is None:
            raise RuntimeError(ControlWrapper.USE_UNDEFINED)

        # If last controller cmd was already the same, don't send another one
        if self.mode == mode and not force:
            return

        # Make the command
        req = SetControllerInput()
        if type(mode) is str:
            req.input = mode
        elif type(mode) is ControllerMode:
            req.input = mode.value
        self._cmd.set_cmd(req)

    def follow_line(self, follow_right: bool, offset: float, velocity: float):
        """
        Send the command to follow the line.

        (Automatically set the controller input)

        Params:
            - follow_right: True if following the right edge, False for the left edge
            - offset: The horizontal offset (m)
        """
        if self._cmd.follow_edge is None:
            raise RuntimeError(ControlWrapper.USE_UNDEFINED)
        self.controller_input(ControllerMode.EDGE)

        line_cmd = CmdLineFollower()
        line_cmd.follow = bool(follow_right)
        line_cmd.offset = float(offset)
        line_cmd.speed = float(velocity)
        self._cmd.follow_edge(line_cmd)

    def set_vel_w(self, v: float, w: float):
        """
        Set the linear and angular velocities command.

        (Automatically set the controller input)

        Params:
            - v: the linear velocity (m/s)
            - w: the angular velocity (rad/s)
        """
        if self._cmd.set_vel is None:
            raise RuntimeError(ControlWrapper.USE_UNDEFINED)
        self.controller_input(ControllerMode.PLAN)

        move_cmd = CmdMove()
        move_cmd.move_type = CmdMove.CMD_V_TR
        move_cmd.velocity = float(v)
        move_cmd.turn_rate = float(w)
        self._cmd.set_vel(move_cmd)

    def set_vel_h(self, v: float, h: float):
        """
        Set the linear velocity and targeted heading.

        (Automatically set the controller input)

        Params:
            - v: the linear velocity (m/s)
            - h: the heading of the robot (rad)
        """
        if self._cmd.set_vel is None:
            raise RuntimeError(ControlWrapper.USE_UNDEFINED)
        self.controller_input(ControllerMode.PLAN)

        move_cmd = CmdMove()
        move_cmd.move_type = CmdMove.CMD_V_ANGLE
        move_cmd.velocity = float(v)
        move_cmd.heading = float(h)
        self._cmd.set_vel(move_cmd)

    # =========================================================================
    # Servo movement
    # =========================================================================
    def set_servo(self, idx: int, p: float, v: float):
        """
        Set the servo position, according to the given velocity.

        Params:
            - idx: the index of the robot
            - p: the angular position of the servo (in ticks)
            - v: the velocity of the movement (ticks/s)
        """
        if self._cmd.set_servo is None:
            raise RuntimeError(ControlWrapper.USE_UNDEFINED)

        servo_cmd = CmdServoPosition()
        servo_cmd.servo_id = idx
        servo_cmd.position = p
        servo_cmd.velocity = v
        self._cmd.set_servo(servo_cmd)
