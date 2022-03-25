#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.curr_pose_sub = self.create_subscription(
            Pose, "curr_pose", self.curr_pose_callback, 10
        )
        # get frame name
        self.declare_parameter("world_frame", "world")
        self.world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        self.declare_parameter("agent_frame", "agent")
        self.agent_frame = (
            self.get_parameter("agent_frame").get_parameter_value().string_value
        )

        # set for tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.declare_parameter("timer_period", 0.01)
        self.timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.cmd_vel_in_agent_coord = Twist()
        self.cmd_vel_in_world_coord = Twist()

    def joy_callback(self, msg: Joy) -> None:
        # invert value of x to match your vision
        self.cmd_vel_in_agent_coord.linear.x = -msg.axes[0]
        self.cmd_vel_in_agent_coord.linear.y = msg.axes[1]
        self.cmd_vel_in_agent_coord.angular.z = msg.axes[3]

    def curr_pose_callback(self, msg: Pose) -> None:
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[quat.x, quat.y, quat.z, quat.w])
        theta = yaw + self.cmd_vel_in_agent_coord.angular.z
        self.cmd_vel_in_world_coord.linear.x = (
            self.cmd_vel_in_agent_coord.linear.x * np.cos(theta)
            - self.cmd_vel_in_agent_coord.linear.y * np.sin(theta)
        )
        self.cmd_vel_in_world_coord.linear.y = (
            self.cmd_vel_in_agent_coord.linear.x * np.sin(theta)
            + self.cmd_vel_in_agent_coord.linear.y * np.cos(theta)
        )
        self.cmd_vel_in_world_coord.angular.z = self.cmd_vel_in_agent_coord.angular.z

    def timer_callback(self) -> None:
        self.cmd_vel_pub.publish(self.cmd_vel_in_world_coord)


def main() -> None:
    rclpy.init()
    controller = Controller()
    try:
        rclpy.spin(controller)
    except Exception as e:
        print(e)

    rclpy.shutdown()


if __name__ == "__main__":
    main()