import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
config = os.path.join(
    get_package_share_directory("point_cloud_processing"), "config", "param.yaml"
)


def generate_launch_description():
    namespace = "/f1"

    pcl_remap = namespace + "/zed2i/zed_node/point_cloud/cloud_registered"
    boundingbox_remap = namespace + "/boundingbox"
    cropped_point_cloud_remap = namespace + "/cropped_point_cloud"
    poloar_coordinate_remap = namespace + "/polar_coordinate"
    point_cloud_processing_time_remap = namespace + "/point_cloud_processing_time"

    # set Node
    f1_pcl_node = Node(
        package="point_cloud_processing",
        executable="point_cloud_processing",
        namespace=namespace,
        parameters=[config],
        remappings=[
            ("/pcl", pcl_remap),
            ("/boundingbox", boundingbox_remap),
            ("/cropped_point_cloud", cropped_point_cloud_remap),
            ("/poloar_coordinate", poloar_coordinate_remap),
            ("/point_cloud_processing_time", point_cloud_processing_time_remap),
        ],
    )
    # get all actions here
    return LaunchDescription(
        [
            f1_pcl_node,
        ]
    )
