"""webots_ros2 package setup file."""

from setuptools import setup
from glob import glob
import os


package_name = "formation_simulation"
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
# data_files.append(os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
data_files.append(("share/" + package_name + "/launch", ["launch/pltjug.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/robot.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/basic_robot_f1.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/basic_robot_f2.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/basic_robot_cf.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/test.py"]))
data_files.append(
    (
        "share/" + package_name + "/resource",
        [
            "resource/turtlebot3_burger_example_map.pgm",
            "resource/turtlebot3_burger_example_map.yaml",
            "resource/turtlebot_webots.urdf",
            "resource/crazyflie_webots.urdf",
            "resource/ros2control.yml",
            "resource/nav2_params.yaml",
        ],
    )
)

data_files.append(
    (
        "share/" + package_name + "/worlds",
        [
            # "TurtleBot3Burger.proto",
            "worlds/turtlebot3_burger_example.wbt",
            "worlds/.turtlebot3_burger_example.wbproj",
        ],
    )
)
data_files.append(("share/" + package_name, ["package.xml"]))


setup(
    name=package_name,
    version="2023.1.1",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    author="Cyberbotics",
    author_email="support@cyberbotics.com",
    maintainer="Cyberbotics",
    maintainer_email="support@cyberbotics.com",
    keywords=["ROS", "Webots", "Robot", "Simulation", "Examples"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="TurtleBot3 Burger robot ROS2 interface for Webots.",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["sim_publisher = formation_simulation.sim_publisher:main"],
        "launch.frontend.launch_extension": ["launch_ros = launch_ros"],
    },
)
