#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    Vector3,
    TransformStamped,
    Twist,
)
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class AgentBody(Node):
    def __init__(self) -> None:
        super().__init__("AgentBody")

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.curr_pose_pub = self.create_publisher(Pose, "curr_pose", 10)
        self.curr_pose = Pose()

        # get frame name
        self.declare_parameter("world_frame", "world")
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.declare_parameter("agent_frame", "agent")
        self.agent_frame = self.get_parameter("agent_frame").get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.declare_parameter("timer_period", 0.01)
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

    def cmd_vel_callback(self, msg: Twist) -> None:
        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + self.timer_period * msg.linear.x,
            y=position.y + self.timer_period * msg.linear.y,
            z=position.z + self.timer_period * msg.linear.z,
        )
        quat = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[quat.x, quat.y, quat.z, quat.w])

        quat = quaternion_from_euler(ai=0, aj=0, ak=yaw + self.timer_period * msg.angular.z)
        self.curr_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.curr_pose_pub.publish(self.curr_pose)

        transform_stamped = TransformStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            child_frame_id=self.agent_frame,
            transform=Transform(
                translation=Vector3(
                    x=self.curr_pose.position.x,
                    y=self.curr_pose.position.y,
                    z=self.curr_pose.position.z,
                ),
                rotation=self.curr_pose.orientation,
            ),
        )
        self.broadcaster.sendTransform(transform_stamped)


def main() -> None:
    rclpy.init()
    agent_body = AgentBody()
    try:
        rclpy.spin(agent_body)
    except Exception as e:
        print(e)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
