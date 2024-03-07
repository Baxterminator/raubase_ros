from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config.config_file import ConfigFile

# =============================================================================
#                               Camera Launcher
#  This launch file let users launch the Camera Node configured with the file
#  at (~/config/camera.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("camera")
    return LaunchDescription(
        [
            Node(
                package="raubase_ros",
                executable="camera",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
