from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from raubase_ros.constants.namespaces import DRIVE_NAMESPACE

# =============================================================================
#                         Full Raubase stack Launcher
#  This launch file will initialize all the stack (Level 1, 2 and 3) of the
#  Raubase Stack
# =============================================================================


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace("theo"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("raubase_ros"),
                                "/launch",
                                "/stacks",
                                "/motor_stack.launch.py",
                            ],
                        ),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("raubase_ros"),
                                "/launch",
                                "/stacks",
                                "/mixer_stack.launch.py",
                            ],
                        ),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("raubase_ros"),
                                "/launch",
                                "/stacks",
                                "/plan_stack.launch.py",
                            ],
                        ),
                    ),
                ]
            )
        ]
    )
