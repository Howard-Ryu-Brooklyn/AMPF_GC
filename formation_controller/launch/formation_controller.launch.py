import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
config_common = os.path.join(
    get_package_share_directory("formation_controller"), "config", "param.yaml"
)


def generate_launch_description():
    desired_formation_node = Node(
        package="formation_controller",
        executable="desired_formation",
        output="screen",
        parameters=[config_common],
    )

    follower1_controller_node = Node(
        package="formation_controller",
        namespace="/f1",
        executable="follower1_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            ("/cmd_vel", "/f1/cmd_vel"),
            ("/polar_coordinate", "/f1/polar_coordinate"),
            ("/follower1info", "/f1/follower1info"),
            ("/lost_leader", "/f1/lost_leader"),
        ],
    )

    follower2_controller_node = Node(
        package="formation_controller",
        namespace="/f2",
        executable="follower2_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            ("/cmd_vel", "/f2/cmd_vel"),
            ("/polar_coordinate", "/f2/polar_coordinate"),
            ("/uwb", "/f2/uwb"),
            ("/odom", "/f2/odom"),
            ("/follower2info", "/f2/follower2info"),
            ("/lost_leader", "/f2/lost_leader"),
        ],
    )

    # get all actions here
    return LaunchDescription(
        [desired_formation_node, follower1_controller_node, follower2_controller_node]
    )
