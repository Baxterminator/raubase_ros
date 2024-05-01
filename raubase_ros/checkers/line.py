from matplotlib.ticker import FormatStrFormatter
from raubase_ros.config import get_top_namespace
from raubase_ros.wrappers.node import NodeWrapper
from raubase_msgs.msg import DataLineSensor, ResultEdge

import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
import numpy as np


class LineSensorChecker(NodeWrapper):
    """ """

    MAX_SAMPLES = 200
    MAX_VALUE = 1000

    def __init__(self) -> None:
        super().__init__("line_checker", namespace=get_top_namespace())

        self.raw = np.zeros((8, LineSensorChecker.MAX_SAMPLES))
        self.norm = np.zeros((8, LineSensorChecker.MAX_SAMPLES))
        self.x = np.arange(0, LineSensorChecker.MAX_SAMPLES)
        self.redge = np.zeros(LineSensorChecker.MAX_SAMPLES)
        self.ledge = np.zeros(LineSensorChecker.MAX_SAMPLES)
        self.setup_plots()

        self.raw_sub = self.create_subscription(
            DataLineSensor, "sensor/line/raw", self.raw_callback, 10
        )

        self.norm_sub = self.create_subscription(
            DataLineSensor, "sensor/line/normalized", self.norm_callback, 10
        )
        self.edge_sub = self.create_subscription(
            ResultEdge, "sensor/line/edge", self.edge_callback, 10
        )

        self.timer = self.create_timer(0.2, self.plot_update)

    def setup_plots(self) -> None:
        self.fig = plt.figure(
            label="Line Sensor - Data",
            figsize=(10, 5),
        )

        # Define norm
        norm = Normalize(vmin=0, vmax=1000)
        cmap = "gray"

        X = np.arange(0, LineSensorChecker.MAX_SAMPLES)
        Y = np.arange(0, 8)

        # Raw sensor data
        self.raw_ax = self.fig.add_axes([0.05, 0.575, 0.65, 0.35])
        self.raw_img = self.raw_ax.pcolormesh(
            X, Y, self.raw, cmap=cmap, shading="nearest", norm=norm
        )
        self.raw_ax.set_yticks([0, 7])
        self.raw_ax.set_yticklabels(["Right", "Left"])
        self.raw_ax.set_title("Raw sensors values")
        self.raw_ax.set_xlabel("Time (unit)")

        # Processed sensor data
        self.processed_ax = self.fig.add_axes([0.05, 0.09, 0.65, 0.35])
        self.norm_img = self.processed_ax.pcolormesh(
            X, Y, self.norm, cmap=cmap, shading="nearest", norm=norm
        )
        self.processed_ax.set_yticks([0, 7])
        self.processed_ax.set_yticklabels(["Right", "Left"])
        self.processed_ax.set_title("Normalized sensors values")
        self.processed_ax.set_xlabel("Time (unit)")

        self.proc_ax2 = self.processed_ax.twinx()
        (self.right_edge,) = self.proc_ax2.plot(
            self.x, self.redge * 100, "b-", label="Right edge"
        )
        (self.left_edge,) = self.proc_ax2.plot(
            self.x, self.ledge * 100, "r-", label="Left edge"
        )
        self.proc_ax2.set_ylim(-12.0 / 2, 12.0 / 2)
        self.proc_ax2.set_ylabel("Edge distances")
        self.proc_ax2.yaxis.set_major_formatter(FormatStrFormatter("%.2f cm"))
        self.proc_ax2.legend()

        # Colorbar
        ticks = [LineSensorChecker.MAX_VALUE / 10 * i for i in range(0, 11)]
        self.cb_ax = self.fig.add_axes([0.83, 0.1, 0.06, 0.8])
        self.cb = self.fig.colorbar(
            ScalarMappable(norm=norm, cmap=cmap),
            cax=self.cb_ax,
            ticks=[LineSensorChecker.MAX_VALUE / 10 * i for i in range(0, 11)],  # type: ignore
            orientation="vertical",
        )
        # ticks[0] = "Black"  # type: ignore
        # ticks[-1] = "White"  # type: ignore
        # self.cb_ax.set_yticklabels(ticks)
        self.cb_ax.set_title("Detected")

        self.fig.canvas.draw()

    def plot_update(self) -> None:
        self.get_logger().info("Updating plot ...", throttle_duration_sec=2.0)
        self.raw_img.update({"array": self.raw})
        self.norm_img.update({"array": self.norm})
        self.right_edge.set_data(self.x, self.redge * 100)
        self.left_edge.set_data(self.x, self.ledge * 100)

        self.fig.canvas.draw()

    def raw_callback(self, msg: DataLineSensor) -> None:
        self.get_logger().info("Raw callback ...", throttle_duration_sec=2.0)
        self.raw = np.hstack(
            [
                self.raw[:, 1:],
                np.flip(np.array(list(msg.data), dtype=float).reshape((8, 1))),
            ]
        )

    def norm_callback(self, msg: DataLineSensor) -> None:
        self.get_logger().info("Normalized callback ...", throttle_duration_sec=2.0)
        self.norm = np.hstack(
            [
                self.norm[:, 1:],
                np.flip(np.array(list(msg.data), dtype=float).reshape((8, 1))),
            ]
        )

    def edge_callback(self, msg: ResultEdge) -> None:
        self.get_logger().info("Edge values callback ...", throttle_duration_sec=2.0)
        self.redge = np.hstack([self.redge[1:], np.array([msg.right_edge])])
        self.ledge = np.hstack([self.ledge[1:], np.array([msg.left_edge])])
