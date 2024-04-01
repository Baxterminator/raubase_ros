from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile
from raubase_ros.constants import ConfigFiles, PACKAGE_NAME

# =============================================================================
#                              Odometry Launcher
#  This launch file let users launch the Odometry daemon - in charge of
#  estimating the robot position - with all the parameters described in the
#  config files (~/config/odometry.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile(ConfigFiles.Names.LOCALIZATION)

    return LaunchDescription(
        [
            Node(
                package=PACKAGE_NAME,
                executable="localization",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
