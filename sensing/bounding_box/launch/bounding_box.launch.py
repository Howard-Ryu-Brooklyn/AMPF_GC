import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

# Specify files like yaml, xacro, urdf in this format
config_path = os.path.join(
    get_package_share_directory("bounding_box"), "config", "param.yaml"
)


def generate_launch_description():
    namespace = "/f1"

    image_remap = namespace + "/zed_node/rgb/image_rect_color"
    boundingbox_remap = namespace + "/boundingbox"
    obj_detection_time_remap = namespace + "/obj_detection_time"
    lost_leader_remap = namespace + "/lost_leader"

    # set Node
    f1_yolo_node = Node(
        package="bounding_box",
        executable="bounding_box_node",
        namespace=namespace,
        parameters=[config_path],
        remappings=[
            ("/image", image_remap),
            ("/boundingbox", boundingbox_remap),
            ("/obj_detection_time", obj_detection_time_remap),
            ("/lost_leader", lost_leader_remap),
        ],
    )

    # get all actions here
    return LaunchDescription(
        [
            f1_yolo_node,
        ]
    )
