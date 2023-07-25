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
    get_package_share_directory("pkg_name"), "folder", "file.yaml"
)


def generate_launch_description():
    # declare and get launch argument
    # get Launch Argument, this should be matched to Declared Launch Argument
    # you do this because you want to config node or launch using launch argument
    arg1 = LaunchConfiguration("arg1")

    dc_arg1 = (
        DeclareLaunchArgument(
            "arg1",
            default_value="arg1_default",
            description="description of arg1",
        ),
    )

    # set Node
    run_node_name = Node(
        package="",
        namespace="",
        executable="",
        name=arg1,
        output="screen",
        parameters=[
            # YAML files
            config_common,
        ],
    )

    # set Launch file
    run_launch_name = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("pkg"), "/folder", "/file.launch.py"]
        ),
        launch_arguments={
            "arg1": arg1,
        }.items(),
    )

    # get all actions here
    return LaunchDescription(
        [
            # Set Environment Variable
            SetEnvironmentVariable(name="", value=""),
            # Declare launch argument that you want to use
            dc_arg1,
            # Nodes that you want to run
            run_node_name,
            # Other launch files from other pkgs
            run_launch_name,
        ]
    )
