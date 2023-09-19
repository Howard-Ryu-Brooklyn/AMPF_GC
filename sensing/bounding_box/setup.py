from setuptools import setup
from glob import glob
import os

package_name = "bounding_box"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # os.path.join("share", package_name, "config") 여기서 package_name뒤에 "folder"이름을 붙여줘야 정리해줌
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dcasl-icryu",
    maintainer_email="dcasl-icryu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bounding_box_node = bounding_box.bounding_box_node:main",
        ],
    },
)
