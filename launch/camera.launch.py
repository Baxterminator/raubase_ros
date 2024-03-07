from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package="raubase_ros", executable="camera")])
