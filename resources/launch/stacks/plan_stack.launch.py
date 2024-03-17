from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from raubase_ros.constants.namespaces import PLAN_NAMESPACE

# =============================================================================
#                         Motor control stack Launcher
#  This launch file will initialize all the velocity controlling stack of the
#  Raubase software.
# =============================================================================


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace(PLAN_NAMESPACE),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                FindPackageShare("raubase_ros"),
                                "/launch",
                                "/components",
                                "/camera.launch.py",
                            ],
                        ),
                    ),
                ]
            )
        ]
    )
