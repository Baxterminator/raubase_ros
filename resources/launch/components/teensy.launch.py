from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config.config_file import ConfigFile

# =============================================================================
#                             Teensy Board Launcher
#  This launch file let users launch the Teensy Board Node - in charge of
#  communication between the board and ROS - with all the parameters described
#  in the config files (~/config/teensy.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("teensy")

    return LaunchDescription(
        [
            Node(
                package="raubase_ros",
                executable="teensy",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
