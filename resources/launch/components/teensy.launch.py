from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile, is_on_raspberry
from raubase_ros.constants import ConfigFiles, PACKAGE_NAME

# =============================================================================
#                             Teensy Board Launcher
#  This launch file let users launch the Teensy Board Node - in charge of
#  communication between the board and ROS - with all the parameters described
#  in the config files (~/config/teensy.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile(ConfigFiles.Names.TEENSY)

    if is_on_raspberry():
        return LaunchDescription(
            [
                Node(
                    package=PACKAGE_NAME,
                    executable="teensy",
                    parameters=config.get_parameters(),
                    remappings=config.get_remaps(),
                )
            ]
        )

    # If not on raspberry, launch simulator
    return LaunchDescription(
        [
            Node(
                package=PACKAGE_NAME,
                executable="simulator",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
