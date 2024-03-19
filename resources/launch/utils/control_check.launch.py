from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from raubase_ros.config import get_top_namespace

# =============================================================================
#                         Full Raubase stack Launcher
#  This launch file will initialize all the stack (Level 1, 2 and 3) of the
#  Raubase Stack
# =============================================================================


def generate_launch_description():
    local_setup = LaunchConfiguration("setup") != "robot"
    decl = DeclareLaunchArgument("setup", default_value="robot")

    return LaunchDescription(
        [
            GroupAction(
                [
                    decl,
                    LogInfo(
                        condition=LaunchConfigurationNotEquals("setup", "robot"),
                        msg=[f"Running on local server"],
                    ),
                    LogInfo(
                        condition=LaunchConfigurationEquals("setup", "robot"),
                        msg=[f"Running on robot server"],
                    ),
                    PushRosNamespace(("theo" if local_setup else get_top_namespace())),
                    Node(
                        package="raubase_ros",
                        executable="control_check",
                    ),
                ]
            )
        ]
    )
