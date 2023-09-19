from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch


def generate_launch_description():
    return LaunchDescription(
        [
            launch.actions.ExecuteProcess(
                cmd=["cd", "~/plotjuggler/data", "&&", "ros2", "bag", "record", "-a"],
                output="screen",
            ),
        ]
    )
