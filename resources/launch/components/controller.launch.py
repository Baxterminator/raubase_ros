from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile

# =============================================================================
#                         Velocity Controller Launcher
#  This launch file let users launch the Velocity Controller Node - in charge
#  of moving the robot according to the velocity commands - with all the
#  parameters described in the config files (~/config/control.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("control")

    return LaunchDescription(
        [
            Node(
                package="raubase_ros",
                executable="controller",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
