#!/usr/bin/python3

from rclpy import init, ok, spin_once, shutdown
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DummyJointState(Node):
    def __init__(self):
        super().__init__("Control")  # type:ignore

        self.message = JointState()
        self.message.name = ["right_wheel", "left_wheel"]
        self.message.position = [0.0, 0.0]

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_transform)

    def publish_transform(self):
        # self.message.position[0] += 0.1
        # self.message.position[1] -= 0.2
        self.message.header.frame_id = "body"
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.message)


if __name__ == "__main__":
    init()

    node = DummyJointState()
    while ok():
        spin_once(node)

    shutdown()
