from launch import LaunchDescription
from launch_ros.actions import Node
from raubase_ros.config import ConfigFile

# =============================================================================
#                      Line following controller Launcher
#  This launch file let users launch the Edge controller Node - in charge of
#  following a line on the floor - with all the parameters described
#  in the config files (~/config/control.yaml).
# =============================================================================


def generate_launch_description():
    config = ConfigFile("control")

    return LaunchDescription(
        [
            # Node(
            #     package="raubase_ros",
            #     executable="line_follow",
            #     parameters=config.get_parameters(),
            #     remappings=config.get_remaps(),
            # )
        ]
    )
