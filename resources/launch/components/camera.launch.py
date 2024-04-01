from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile
from raubase_ros.constants import ConfigFiles, PACKAGE_NAME

# =============================================================================
#                               Camera Launcher
#  This launch file let users launch the Camera Node configured with the file
#  at (~/config/camera.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile(ConfigFiles.Names.CAMERA)
    return LaunchDescription(
        [
            Node(
                package=PACKAGE_NAME,
                executable="camera",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ],
    )
