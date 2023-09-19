import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
config_path = os.path.join(get_package_share_directory("uwb"), "config", "param.yaml")


def generate_launch_description():
    namespace = "f1"

    uwb_remp = namespace + "/uwb"

    # set Node
    uwb_node = Node(
        package="uwb",
        executable="uwb_node",
        namespace=namespace,
        parameters=[config_path],
        remappings=[
            ("/uwb", uwb_remp),
        ],
    )

    # get all actions here
    return LaunchDescription(
        [
            uwb_node,
        ]
    )
