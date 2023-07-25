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
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Specify files like yaml, xacro, urdf in this format
config_path = os.path.join(
    get_package_share_directory("my_yolo_wrapper"), "config", "param.yaml"
)


def launch_setup(context, *args, **kwargs):
    # declare and get launch argument
    # get Launch Argument, this should be matched to Declared Launch Argument
    # you do this because you want to config node or launch using launch argument

    agent1_ns = "f1"
    agent2_ns = "f2"

    sub_topic_name = "/image"
    topic_name = "/zed2i/zed_node/rgb/image_rect_color"

    agent1_sub_topic = "/" + agent1_ns + topic_name
    agent2_sub_topic = "/" + agent2_ns + topic_name

    # set Node
    f1_yolo_node = Node(
        package="my_yolo_wrapper",
        executable="boundingbox",
        namespace=agent1_ns,
        parameters=[config_path],
        remappings=[(sub_topic_name, agent1_sub_topic)],
    )

    f2_yolo_node = Node(
        package="my_yolo_wrapper",
        executable="boundingbox",
        namespace=agent2_ns,
        parameters=[config_path],
        remappings=[(sub_topic_name, agent2_sub_topic)],
    )

    return [f1_yolo_node, f2_yolo_node]
    # return [f1_yolo_node]


def generate_launch_description():
    # get all actions here
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
