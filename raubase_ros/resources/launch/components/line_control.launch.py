from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile
from raubase_ros.constants import ConfigFiles, PACKAGE_NAME

# =============================================================================
#                      Line following controller Launcher
#  This launch file let users launch the Edge controller Node - in charge of
#  following a line on the floor - with all the parameters described
#  in the config files (~/config/line.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile(ConfigFiles.Names.LINE)

    return LaunchDescription(
        [
            Node(
                package=PACKAGE_NAME,
                executable="line_follow",
                parameters=config.get_parameters(),
                remappings=config.get_remaps(),
            )
        ]
    )
