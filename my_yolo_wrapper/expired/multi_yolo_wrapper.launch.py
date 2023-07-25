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

# # Specify files like yaml, xacro, urdf in this format
# config_common = os.path.join(
#     get_package_share_directory("pkg_name"), "folder", "file.yaml"
# )


def launch_setup(context, *args, **kwargs):
    # declare and get launch argument
    # get Launch Argument, this should be matched to Declared Launch Argument
    # you do this because you want to config node or launch using launch argument
    agent1_name = LaunchConfiguration("agent1_name")
    agent2_name = LaunchConfiguration("agent2_name")

    agent1_name_str = agent1_name.perform(context)
    agent2_name_str = agent2_name.perform(context)

    sub_topic_name = "/zed2i/zed_node/rgb/image_rect_color"

    agent1_sub_topic = "/" + agent1_name_str + "/zed2i/zed_node/rgb/image_rect_color"
    agent2_sub_topic = "/" + agent2_name_str + "/zed2i/zed_node/rgb/image_rect_color"

    # set Node
    f1_yolo_node = Node(
        package="my_yolo_wrapper",
        executable="my_yolo",
        namespace=agent1_name,
        remappings=[(sub_topic_name, agent1_sub_topic)],
    )

    f2_yolo_node = Node(
        package="my_yolo_wrapper",
        executable="my_yolo",
        namespace=agent2_name,
        remappings=[(sub_topic_name, agent2_sub_topic)],
    )

    # # set Launch file
    # run_launch_name = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("pkg"), "/folder", "/file.launch.py"]
    #     )
    # )

    return [f1_yolo_node, f2_yolo_node]


def generate_launch_description():
    # get all actions here
    return LaunchDescription(
        [
            # Set Environment Variable
            # SetEnvironmentVariable(name="", value=""),
            # Declare launch argument that you want to use
            DeclareLaunchArgument(
                "agent1_name",
                default_value="f1",
                description="description of agent1_name",
            ),
            DeclareLaunchArgument(
                "agent2_name",
                default_value="f2",
                description="description of agent2_name",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
