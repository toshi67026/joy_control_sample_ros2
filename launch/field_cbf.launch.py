#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("joy_control_sample_ros2")
    params = LaunchConfiguration("params", default=os.path.join(pkg_dir, "field_cbf.yaml"))

    rviz_config = LaunchConfiguration(
        "rviz_config", default=os.path.join(pkg_dir, "field_cbf.rviz")
    )

    joy_node = Node(package="joy", executable="joy_node", name="joy_node")

    controller_node = Node(
        package="joy_control_sample_ros2",
        executable="controller",
        name="controller",
        remappings=[
            ("cmd_vel", "cmd_vel_nom")
        ],
        parameters=[params],
    )

    agent_body_node = Node(
        package="joy_control_sample_ros2",
        executable="agent_body",
        name="agent_body",
        remappings=[
            ("cmd_vel", "cmd_vel_opt")
        ],
        parameters=[params],
    )

    cbf_optimizer_node = Node(
        package="joy_control_sample_ros2",
        executable="cbf_optimizer",
        name="cbf_optimizer",
        parameters=[params],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(joy_node)
    ld.add_action(controller_node)
    ld.add_action(agent_body_node)
    ld.add_action(cbf_optimizer_node)
    ld.add_action(rviz2_node)

    return ld