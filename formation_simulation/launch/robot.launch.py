import os

# launch description
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)

# launch substitutions
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

# launch actions
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from launch.conditions import LaunchConfigurationEquals

# launch event handlers
from launch.event_handlers import OnExecutionComplete
from launch.event_handlers import OnProcessExit

# launch webot
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)

from datetime import datetime

config_common = os.path.join(
    get_package_share_directory("formation_controller"), "config", "param.yaml"
)


def generate_launch_description():
    package_dir = get_package_share_directory("formation_simulation")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")

    # webot world file
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
        mode=mode,
        ros2_supervisor=True,
    )

    # Include launch file
    Turtlebot3_f1_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("formation_simulation"),
                "/launch/" + "basic_robot_f1.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "True",
            "nav": "False",
            "slam": "False",
        }.items(),
    )

    Turtlebot3_f2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("formation_simulation"),
                "/launch/" + "basic_robot_f2.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "True",
            "nav": "False",
            "slam": "False",
        }.items(),
    )

    Crazyflie_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("formation_simulation"),
                "/launch/" + "basic_robot_cf.launch.py",
            ]
        )
    )

    sim_publisher_node = Node(
        package="formation_simulation",
        executable="sim_publisher",
        # remappings=[("/follower1info", "/f1/follower1info"),],
        output="screen",
    )


    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        # arguments=["-l", "~/plotjuggler/layout/webot_simulation.xml"],
    )

    now = datetime.now()
    strnow = now.strftime("%Y_%m_%d-%H_%M_%S")
    data_path = "/home/humble/plotjuggler/data/rosbag2_"
    data_name = data_path + strnow

    print(data_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="turtlebot3_burger_example.wbt",
                description="Choose one of the world files from `/formation_simulation/world` directory",
            ),
            DeclareLaunchArgument(
                "mode", default_value="realtime", description="Webots startup mode"
            ),
            DeclareLaunchArgument(
                "record", default_value="true", description="activate ros2 bag"
            ),
            # webot simulation
            webots,
            webots._supervisor,
            Crazyflie_launch,
            Turtlebot3_f1_launch,
            Turtlebot3_f2_launch,
            sim_publisher_node,
            plotjuggler_node,
            # ros bag
            # LogInfo(condition=LaunchConfigurationEquals("record", "true")),
            launch.actions.ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a", "-o", data_name],
                # cmd=["ros2", "bag", "record", "-a"],
                output="screen",
            ),
            # This action will kill all nodes once the Webots simulation has exited
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
