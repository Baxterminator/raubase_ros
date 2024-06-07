from dataclasses import dataclass, field
from raubase_ros.config import get_top_namespace
from raubase_ros.wrappers.node import NodeWrapper
from raubase_msgs.msg import (
    ResultEdge,
    ResultOdometry,
    CmdMotorVoltage,
    CmdMove,
    StateVelocityController,
)

import matplotlib.pyplot as plt
import numpy as np
from rclpy import shutdown


@dataclass
class Data:
    # Plot properties
    initial_time_set: bool = False
    initial_time: float = -1.0
    display_time: list = field(default_factory=lambda: [0.0, 1.0])

    # Times
    edge_t: list = field(default_factory=list)
    odom_t: list = field(default_factory=list)
    ref_t: list = field(default_factory=list)
    cmd_t: list = field(default_factory=list)
    control_t: list = field(default_factory=list)

    # Edges
    edge_right: list = field(default_factory=list)
    edge_left: list = field(default_factory=list)

    # Velocity
    vel_cmd: list = field(default_factory=list)
    vel_control: list = field(default_factory=list)
    vel_odom: list = field(default_factory=list)

    # Turn rate
    tr_cmd: list = field(default_factory=list)
    tr_control: list = field(default_factory=list)
    tr_odom: list = field(default_factory=list)

    # Heading
    head_control: list = field(default_factory=list)
    head_odom: list = field(default_factory=list)

    # Commands
    right_cmd: list = field(default_factory=list)
    left_cmd: list = field(default_factory=list)


def ctime(sec: int, nano: int) -> float:
    return float(sec) + float(nano) * 1e-9


def dt(tfrom: float, tto: float) -> float:
    return tto - tfrom


TIME_TO_KEEP = 60  # in s
LIM_MARGIN = 1.2


class LineVelocityChecker(NodeWrapper):
    """
    Line following engine display
    """

    PLOT_WIDTH = 0.4
    PLOT_HEIGHT = 0.35

    def __init__(self) -> None:
        super().__init__("linefollow_checker", namespace=get_top_namespace())

        self.data = Data()
        self.setup_plots()

        # Set callbacks
        self.edge_sub = self.create_subscription(
            ResultEdge, "sensor/line/edge", self.edge_callback, 10
        )
        self.odom_sub = self.create_subscription(
            ResultOdometry, "state/odometry", self.odom_callback, 10
        )
        self.ref_sub = self.create_subscription(
            CmdMove, "control/move", self.reference_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            CmdMotorVoltage, "control/set_voltage", self.cmd_callback, 10
        )
        self.ctrl_sub = self.create_subscription(
            StateVelocityController, "state/vcontroller", self.control_callback, 10
        )

        self.timer = self.create_timer(0.2, self.plot_update)

    def setup_plots(self) -> None:
        self.fig = plt.figure(
            label="Line Follower - Data",
            figsize=(13, 6),
        )

        # Edges
        self.graph1 = self.fig.add_axes(
            [
                0.05,
                0.57,
                LineVelocityChecker.PLOT_WIDTH,
                LineVelocityChecker.PLOT_HEIGHT,
            ]
        )
        self.graph1.set_title("Detected edge values")
        self.graph1_2 = self.graph1.twinx()
        (self.gr1_vel_cmd,) = self.graph1.plot(
            self.data.ref_t, self.data.vel_cmd, ":", label="Velocity (Command)"
        )
        (self.gr1_vel_ctrl,) = self.graph1.plot(
            self.data.control_t, self.data.vel_control, ":", label="Velocity (Control)"
        )
        (self.gr1_vel_odom,) = self.graph1.plot(
            self.data.odom_t, self.data.vel_odom, ":", label="Velocity (Odometry)"
        )
        (self.gr1_redge,) = self.graph1_2.plot(
            self.data.edge_t, self.data.edge_right, label="Right edge"
        )
        (self.gr1_ledge,) = self.graph1_2.plot(
            self.data.edge_t, self.data.edge_left, label="Left edge"
        )
        self.graph1.set_xlabel("Time (s)")
        self.graph1.set_ylim(-LIM_MARGIN * 2, LIM_MARGIN * 2)
        self.graph1.set_ylabel("Velocity (m/s)")
        self.graph1_2.set_ylabel("Edge position (cm)")
        self.graph1_2.set_ylim(-LIM_MARGIN * 6, LIM_MARGIN * 6)
        self.graph1.legend(loc="upper right")
        self.graph1_2.legend(loc="upper left")

        # Commands
        self.graph2 = self.fig.add_axes(
            [
                0.05,
                0.1,
                LineVelocityChecker.PLOT_WIDTH,
                LineVelocityChecker.PLOT_HEIGHT,
            ]
        )
        self.graph2.set_title("Motor commands")
        self.graph2_2 = self.graph2.twinx()
        (self.gr2_vel_cmd,) = self.graph2.plot(
            self.data.ref_t, self.data.vel_cmd, ":", label="Velocity (Command)"
        )
        (self.gr2_vel_ctrl,) = self.graph2.plot(
            self.data.control_t, self.data.vel_control, ":", label="Velocity (Control)"
        )
        (self.gr2_vel_odom,) = self.graph2.plot(
            self.data.odom_t, self.data.vel_odom, ":", label="Velocity (Odometry)"
        )
        (self.gr2_rcmd,) = self.graph2_2.plot(
            self.data.cmd_t, self.data.right_cmd, label="Right command"
        )
        (self.gr2_lcmd,) = self.graph2_2.plot(
            self.data.cmd_t, self.data.left_cmd, label="Left command"
        )
        self.graph2.set_xlabel("Time (s)")
        self.graph2.set_ylabel("Velocity (m/s)")
        self.graph2.set_ylim(-LIM_MARGIN * 2, LIM_MARGIN * 2)
        self.graph2_2.set_ylabel("Command (V)")
        self.graph2_2.set_ylim(-LIM_MARGIN * 10, LIM_MARGIN * 10)
        self.graph2.legend(loc="upper right")
        self.graph2_2.legend(loc="upper left")

        # Turn rate
        self.graph3 = self.fig.add_axes(
            [
                0.57,
                0.57,
                LineVelocityChecker.PLOT_WIDTH,
                LineVelocityChecker.PLOT_HEIGHT,
            ]
        )
        self.graph3.set_title("Turn rate")
        (self.gr3_tr_cmd,) = self.graph3.plot(
            self.data.ref_t, self.data.tr_cmd, label="Command"
        )
        (self.gr3_tr_ctrl,) = self.graph3.plot(
            self.data.control_t, self.data.tr_control, label="Control"
        )
        (self.gr3_tr_odom,) = self.graph3.plot(
            self.data.odom_t, self.data.tr_odom, label="Odometry"
        )
        self.graph3.set_xlabel("Time (s)")
        self.graph3.set_ylabel("Turn rate (rad/s)")
        self.graph3.legend()

        # Heading
        self.graph4 = self.fig.add_axes(
            [
                0.57,
                0.1,
                LineVelocityChecker.PLOT_WIDTH,
                LineVelocityChecker.PLOT_HEIGHT,
            ]
        )
        self.graph4.set_title("Heading")
        (self.gr4_hd_ctrl,) = self.graph4.plot(
            self.data.control_t, self.data.head_control, label="Control"
        )
        (self.gr4_hd_odom,) = self.graph4.plot(
            self.data.odom_t, self.data.head_odom, label="Odometry"
        )
        self.graph4.set_xlabel("Time (s)")
        self.graph4.set_ylabel("Heading (rad)")
        self.graph4.legend()

        self.fig.canvas.draw()

        def close_callback(evt):
            self.fig = None
            shutdown()

        self.fig.canvas.mpl_connect("close_event", close_callback)

    def plot_update(self) -> None:
        self.get_logger().info("Updating plot ...", throttle_duration_sec=2.0)

        if self.fig is not None:
            # Graph 1
            self.gr1_vel_cmd.set_data(self.data.ref_t, self.data.vel_cmd)
            self.gr1_vel_ctrl.set_data(self.data.control_t, self.data.vel_control)
            self.gr1_vel_odom.set_data(self.data.odom_t, self.data.vel_odom)
            self.gr1_redge.set_data(self.data.edge_t, self.data.edge_right)
            self.gr1_ledge.set_data(self.data.edge_t, self.data.edge_left)
            self.graph1.set_xlim(*self.data.display_time)

            # Graph 2
            self.gr2_vel_cmd.set_data(self.data.ref_t, self.data.vel_cmd)
            self.gr2_vel_ctrl.set_data(self.data.control_t, self.data.vel_control)
            self.gr2_vel_odom.set_data(self.data.odom_t, self.data.vel_odom)
            self.gr2_rcmd.set_data(self.data.cmd_t, self.data.right_cmd)
            self.gr2_lcmd.set_data(self.data.cmd_t, self.data.left_cmd)
            self.graph2.set_xlim(*self.data.display_time)

            # Graph 3
            self.gr3_tr_cmd.set_data(self.data.ref_t, self.data.tr_cmd)
            self.gr3_tr_ctrl.set_data(self.data.control_t, self.data.tr_control)
            self.gr3_tr_odom.set_data(self.data.odom_t, self.data.tr_odom)
            self.graph3.set_xlim(*self.data.display_time)
            self.graph3.set_ylim(
                LIM_MARGIN
                * np.min(
                    [
                        np.min(self.data.tr_cmd) if len(self.data.ref_t) > 0 else 0,
                        (
                            np.min(self.data.tr_control)
                            if len(self.data.control_t) > 0
                            else 0
                        ),
                        np.min(self.data.tr_odom) if len(self.data.odom_t) > 0 else 0,
                    ]
                ),
                LIM_MARGIN
                * np.max(
                    [
                        np.max(self.data.tr_cmd) if len(self.data.ref_t) > 0 else 0,
                        (
                            np.max(self.data.tr_control)
                            if len(self.data.control_t) > 0
                            else 0
                        ),
                        np.max(self.data.tr_odom) if len(self.data.odom_t) > 0 else 0,
                    ]
                ),
            )

            # Graph 4
            self.gr4_hd_ctrl.set_data(self.data.control_t, self.data.head_control)
            self.gr4_hd_odom.set_data(self.data.odom_t, self.data.head_odom)
            self.graph4.set_xlim(*self.data.display_time)
            self.graph4.set_ylim(
                LIM_MARGIN
                * np.min(
                    [
                        (
                            np.min(self.data.head_control)
                            if len(self.data.control_t) > 0
                            else 0
                        ),
                        np.min(self.data.head_odom) if len(self.data.odom_t) > 0 else 0,
                    ]
                ),
                LIM_MARGIN
                * np.max(
                    [
                        (
                            np.max(self.data.head_control)
                            if len(self.data.control_t) > 0
                            else 0
                        ),
                        np.max(self.data.head_odom) if len(self.data.odom_t) > 0 else 0,
                    ]
                ),
            )

            # Draw the results
            self.fig.canvas.draw()

    def edge_callback(self, msg: ResultEdge) -> None:
        self.get_logger().info("Edge values callback ...", throttle_duration_sec=2.0)
        msg_time = ctime(*self.get_clock().now().seconds_nanoseconds())

        # Check initial time
        if not self.data.initial_time_set:
            self.data.initial_time = msg_time
            self.data.initial_time_set = True
        msg_time = msg_time - self.data.initial_time

        # Remove message to far away
        while (
            len(self.data.edge_t) > 0
            and dt(self.data.edge_t[0], msg_time) > TIME_TO_KEEP
        ):
            self.data.edge_t.pop(0)
            self.data.edge_left.pop(0)
            self.data.edge_right.pop(0)

        self.data.edge_t.append(msg_time)
        self.data.edge_left.append(msg.left_edge * 100)
        self.data.edge_right.append(msg.right_edge * 100)

        # Set diplay range
        self.data.display_time[0] = max(self.data.display_time[0], self.data.edge_t[0])
        self.data.display_time[1] = max(self.data.display_time[1], self.data.edge_t[-1])

    def odom_callback(self, msg: ResultOdometry) -> None:
        self.get_logger().info("Odometry callback ...", throttle_duration_sec=2.0)
        msg_time = ctime(msg.stamp.sec, msg.stamp.nanosec)

        # Check initial time
        if not self.data.initial_time_set:
            self.data.initial_time = msg_time
            self.data.initial_time_set = True
        msg_time = msg_time - self.data.initial_time

        # Remove too old messages
        while (
            len(self.data.odom_t) > 0
            and dt(self.data.odom_t[0], msg_time) > TIME_TO_KEEP
        ):
            self.data.odom_t.pop(0)
            self.data.vel_odom.pop(0)
            self.data.tr_odom.pop(0)
            self.data.head_odom.pop(0)

        self.data.odom_t.append(msg_time)
        self.data.vel_odom.append(msg.v_lin)
        self.data.tr_odom.append(msg.turn_rate)
        self.data.head_odom.append(msg.heading)

        # Set diplay range
        self.data.display_time[0] = max(self.data.display_time[0], self.data.odom_t[0])
        self.data.display_time[1] = max(self.data.display_time[1], self.data.odom_t[-1])

    def reference_callback(self, msg: CmdMove) -> None:
        self.get_logger().info("Move cmd callback ...", throttle_duration_sec=2.0)
        msg_time = ctime(*self.get_clock().now().seconds_nanoseconds())

        # Check initial time
        if not self.data.initial_time_set:
            self.data.initial_time = msg_time
            self.data.initial_time_set = True
        msg_time = msg_time - self.data.initial_time

        # Remove too old messages
        while (
            len(self.data.ref_t) > 0 and dt(self.data.ref_t[0], msg_time) > TIME_TO_KEEP
        ):
            self.data.ref_t.pop(0)
            self.data.vel_cmd.pop(0)
            self.data.tr_cmd.pop(0)

        self.data.ref_t.append(msg_time)
        self.data.vel_cmd.append(msg.velocity)
        self.data.tr_cmd.append(msg.turn_rate)

        # Set diplay range
        self.data.display_time[0] = max(self.data.display_time[0], self.data.ref_t[0])
        self.data.display_time[1] = max(self.data.display_time[1], self.data.ref_t[-1])

    def cmd_callback(self, msg: CmdMotorVoltage) -> None:
        self.get_logger().info("Motor cmd callback ...", throttle_duration_sec=2.0)
        msg_time = ctime(*self.get_clock().now().seconds_nanoseconds())

        # Check initial time
        if not self.data.initial_time_set:
            self.data.initial_time = msg_time
            self.data.initial_time_set = True
        msg_time = msg_time - self.data.initial_time

        # Remove too old messages
        while (
            len(self.data.cmd_t) > 0 and dt(self.data.cmd_t[0], msg_time) > TIME_TO_KEEP
        ):
            self.data.cmd_t.pop(0)
            self.data.right_cmd.pop(0)
            self.data.left_cmd.pop(0)

        self.data.cmd_t.append(msg_time)
        self.data.right_cmd.append(msg.right)
        self.data.left_cmd.append(msg.left)

        # Set diplay range
        self.data.display_time[0] = max(self.data.display_time[0], self.data.cmd_t[0])
        self.data.display_time[1] = max(self.data.display_time[1], self.data.cmd_t[-1])

    def control_callback(self, msg: StateVelocityController) -> None:
        self.get_logger().info("Control state callback ...", throttle_duration_sec=2.0)
        msg_time = ctime(*self.get_clock().now().seconds_nanoseconds())

        # Check initial time
        if not self.data.initial_time_set:
            self.data.initial_time = msg_time
            self.data.initial_time_set = True
        msg_time = msg_time - self.data.initial_time

        # Remove too old messages
        while (
            len(self.data.control_t) > 0
            and dt(self.data.control_t[0], msg_time) > TIME_TO_KEEP
        ):
            self.data.control_t.pop(0)
            self.data.vel_control.pop(0)
            self.data.tr_control.pop(0)
            self.data.head_control.pop(0)

        self.data.control_t.append(msg_time)
        self.data.vel_control.append((msg.vleft_ref + msg.vright_ref) / 2)
        self.data.tr_control.append(msg.turn_rate)
        self.data.head_control.append(msg.heading_ref)

        # Set diplay range
        self.data.display_time[0] = max(
            self.data.display_time[0], self.data.control_t[0]
        )
        self.data.display_time[1] = max(
            self.data.display_time[1], self.data.control_t[-1]
        )
