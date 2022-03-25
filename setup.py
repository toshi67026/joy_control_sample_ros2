import os
from glob import glob

from setuptools import setup

package_name = "joy_control_sample_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share/", package_name), glob("rviz/*.rviz")),
        (os.path.join("share/", package_name), glob("config/*.yaml")),
        (os.path.join("share/", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshiyuki Oshima",
    maintainer_email="toshiyuki67026@gmail.com",
    description="Sample ROS2 package for joy controller",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = joy_control_sample_ros2.controller:main",
            "agent_body = joy_control_sample_ros2.agent_body:main",
        ],
    },
)
