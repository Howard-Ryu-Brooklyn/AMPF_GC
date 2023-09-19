from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["-l", "~/plotjuggler/layout/posxy_z.xml"],
    )

    return LaunchDescription([plotjuggler_node])
