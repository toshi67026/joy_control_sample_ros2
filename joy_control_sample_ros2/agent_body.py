#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class AgentBody(Node):
    def __init__(self) -> None:
        super().__init__("agent_body")

        self.curr_pose = Pose()

        # get frame name
        self.declare_parameter("world_frame", "world")
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.declare_parameter("agent_frame", "agent")
        self.agent_frame = self.get_parameter("agent_frame").get_parameter_value().string_value

        # tf2
        self.tf_buffer = Buffer()
        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter("sampling_time", 0.1)
        self.sampling_time = self.get_parameter("sampling_time").get_parameter_value().double_value

        # pub
        self.curr_pose_pub = self.create_publisher(Pose, "curr_pose", 10)

        # sub
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist) -> None:
        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + self.sampling_time * msg.linear.x,
            y=position.y + self.sampling_time * msg.linear.y,
            z=position.z + self.sampling_time * msg.linear.z,
        )
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        orientation_array = quaternion_from_euler(ai=0, aj=0, ak=yaw + self.sampling_time * msg.angular.z)
        self.curr_pose.orientation = Quaternion(
            x=orientation_array[0],
            y=orientation_array[1],
            z=orientation_array[2],
            w=orientation_array[3],
        )
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
        agent_body.get_logger().error(f"{e}")
    finally:
        agent_body.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
