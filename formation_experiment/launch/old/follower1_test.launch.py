import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from datetime import datetime


def generate_launch_description():
    formation_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("formation_controller"),
                "/launch/formation_controller.launch.py",
            ]
        )
    )

    follower1_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("formation_experiment"),
                "/launch/follower1.launch.py",
            ]
        )
    )

    now = datetime.now()
    strnow = now.strftime("%Y_%m_%d-%H_%M_%S")
    data_path = "/home/humble/plotjuggler/data/rosbag2_"
    data_name = data_path + strnow

    # get all actions here
    return LaunchDescription(
        [
            # formation_controller_launch,
            follower1_launch,
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a", "-o", data_name],
                output="screen",
            ),
        ]
    )
