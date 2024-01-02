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

# Specify files like yaml, xacro, urdf in this format
f1_config_path = os.path.join(
    get_package_share_directory("formation_experiment"), "config", "param_f1.yaml"
)

f2_config_path = os.path.join(
    get_package_share_directory("formation_experiment"), "config", "param_f2.yaml"
)


def generate_launch_description():
    namespace = "/f1"

    boundingbox_remap = namespace + "/boundingbox"
    process_time_remap = namespace + "/process_time"
    lost_leader_remap = namespace + "/lost_leader"
    polar_coordinate_remap = namespace + "/polar_coordinate"

    # set Node
    follower1_node = Node(
        package="zed_yolo",
        executable="zed_yolo_node",
        namespace=namespace,
        parameters=[f1_config_path],
        remappings=[
            ("/boundingbox", boundingbox_remap),
            ("/process_time", process_time_remap),
            ("/lost_leader", lost_leader_remap),
            ("/polar_coordinate", polar_coordinate_remap),
        ],
    )
    
    namespace = "/f2"

    boundingbox_remap = namespace + "/boundingbox"
    process_time_remap = namespace + "/process_time"
    lost_leader_remap = namespace + "/lost_leader"
    polar_coordinate_remap = namespace + "/polar_coordinate"
    
    follower2_node = Node(
        package="zed_yolo",
        executable="zed_yolo_node",
        namespace=namespace,
        parameters=[f2_config_path],
        remappings=[
            ("/boundingbox", boundingbox_remap),
            ("/process_time", process_time_remap),
            ("/lost_leader", lost_leader_remap),
            ("/polar_coordinate", polar_coordinate_remap),
        ],
    )

    # get all actions here
    return LaunchDescription(
        [
            follower1_node,
            follower2_node,
        ]
    )
