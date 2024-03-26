from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile

# =============================================================================
#                              Odometry Launcher
#  This launch file let users launch the Odometry daemon - in charge of
#  estimating the robot position - with all the parameters described in the
#  config files (~/config/odometry.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("localization")

    return LaunchDescription(
        [
            Node(
                package="raubase_ros",
                executable="localization",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )