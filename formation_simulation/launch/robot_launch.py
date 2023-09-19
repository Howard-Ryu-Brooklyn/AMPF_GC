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
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

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
                "/launch/" + "basic_robot_f1.py",
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
                "/launch/" + "basic_robot_f2.py",
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
                "/launch/" + "basic_robot_cf.py",
            ]
        )
    )

    sim_publisher_node = Node(
        package="formation_simulation",
        executable="sim_publisher",
        output="screen",
    )

    desired_formation_node = Node(
        package="formation_controller",
        executable="desired_formation",
        output="screen",
        parameters=[config_common],
    )

    follower1_controller_node = Node(
        package="formation_controller",
        executable="follower1_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            ("/cmd_vel", "/f1/cmd_vel"),
            ("/leader_polar", "/f1/leader_polar"),
            ("/follower1info", "/f1/follower1info"),
        ],
    )

    follower2_controller_node = Node(
        package="formation_controller",
        executable="follower2_controller",
        output="screen",
        parameters=[config_common],
        remappings=[
            ("cmd_vel", "/f2/cmd_vel"),
            ("leader_polar", "/f2/leader_polar"),
            ("uwb", "/my_uwb"),
            ("odom", "f2/odom"),
            ("/follower2info", "/f2/follower2info"),
        ],
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        # arguments=["-l", "~/plotjuggler/layout/webot_simulation.xml"],
    )

    now = datetime.now()
    strnow = now.strftime("%Y_%m_%d-%H_%M_%S")
    data_path = "/home/dcasl-icryu/plotjuggler/data/rosbag2_"
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
            # webot simulation
            webots,
            webots._supervisor,
            Crazyflie_launch,
            Turtlebot3_f1_launch,
            Turtlebot3_f2_launch,
            sim_publisher_node,
            # ros controller
            desired_formation_node,
            follower1_controller_node,
            follower2_controller_node,
            plotjuggler_node,
            # ros bag
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
