from typing import List

from matplotlib.ticker import FormatStrFormatter
from raubase_ros.config import get_top_namespace
from raubase_ros.wrappers.node import NodeWrapper
from raubase_msgs.msg import DataLineSensor

import matplotlib.pyplot as plt
import numpy as np


class LineSensorChecker(NodeWrapper):
    """ """

    def __init__(self) -> None:
        super().__init__("line_checker", namespace=get_top_namespace())

        self.raw = np.ones((8, 50))
        self.raw[0, 0] = 0
        self.norm = np.ones((8, 50))
        self.norm[0, 0] = 0
        self.setup_plots()

        self.raw_sub = self.create_subscription(
            DataLineSensor, "sensor/line/raw", self.raw_callback, 10
        )

        self.norm_sub = self.create_subscription(
            DataLineSensor, "sensor/line/normalized", self.norm_callback, 10
        )

        self.timer = self.create_timer(0.2, self.plot_update)

    def setup_plots(self) -> None:
        self.fig_raw, self.data_ax = plt.subplots(
            ncols=1, nrows=2, label="Line Sensor - Data"
        )
        self.raw_img = self.data_ax[1].imshow(
            self.raw, cmap="gray", interpolation="nearest"
        )
        self.raw_cb = plt.colorbar(
            self.raw_img,
            ticks=[0.1 * i for i in range(0, 11)],
            extend="both",
        )
        self.data_ax[1].set_title("Raw sensors values")

        self.norm_img = self.data_ax[0].imshow(
            self.norm, cmap="gray", interpolation="nearest"
        )
        self.data_ax[0].set_title("Normalized sensors values")

        self.fig_raw.canvas.draw()

    def plot_update(self) -> None:
        self.get_logger().info("Updating plot ...")
        self.raw_img.set_data(self.raw)
        self.norm_img.set_data(self.norm)
        self.fig_raw.canvas.draw()

    def raw_callback(self, msg: DataLineSensor) -> None:
        self.get_logger().info("Raw callback ...")
        self.raw = np.hstack(
            [self.raw[:, 1:], np.array(msg.data, dtype=float).reshape((8, 1)) / 1000]
        )

    def norm_callback(self, msg: DataLineSensor) -> None:
        self.get_logger().info("Normalized callback ...")
        self.norm = np.hstack(
            [self.norm[:, 1:], np.array(msg.data, dtype=float).reshape((8, 1)) / 1000]
        )
