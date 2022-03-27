#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

from .cbf_utils.example.field_cbf_qp_optimizer import FieldCBFOptimizer


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        self.optimizer = FieldCBFOptimizer()
        self.declare_parameter("activate_field_cbf", True)
        self.declare_parameter("field.x_limit", [0.0, 1.0])
        self.declare_parameter("field.y_limit", [1.0, 2.0])
        self.declare_parameter("field.theta", 0.0)
        self.declare_parameter("field.keep_inside", False)
        x_limit = self.get_parameter("field.x_limit").get_parameter_value().double_array_value
        y_limit = self.get_parameter("field.y_limit").get_parameter_value().double_array_value

        self.cent_field = np.array([sum(x_limit) / len(x_limit), sum(y_limit) / len(y_limit)])
        self.width = np.array([(x_limit[1] - x_limit[0]) / 2.0, (y_limit[1] - y_limit[0]) / 2.0])
        self.theta = self.get_parameter("field.theta").get_parameter_value().double_value

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.curr_pose_sub = self.create_subscription(Pose, "curr_pose", self.curr_pose_callback, 10)

        # get frame name
        self.declare_parameter("world_frame", "world")
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.declare_parameter("agent_frame", "agent")
        self.agent_frame = self.get_parameter("agent_frame").get_parameter_value().string_value

        # set for tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.declare_parameter("timer_period", 0.01)
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_vel_in_agent_coord = Twist()
        self.cmd_vel_in_world_coord = Twist()

        self.field_visualization_pub = self.create_publisher(Marker, "field_visualization", 10)

    def joy_callback(self, msg: Joy) -> None:
        # invert value of x to match your vision
        self.cmd_vel_in_agent_coord.linear.x = -msg.axes[0]
        self.cmd_vel_in_agent_coord.linear.y = msg.axes[1]
        self.cmd_vel_in_agent_coord.angular.z = msg.axes[3]

    def curr_pose_callback(self, msg: Pose) -> None:
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[quat.x, quat.y, quat.z, quat.w])
        yaw = yaw + self.cmd_vel_in_agent_coord.angular.z
        self.cmd_vel_in_world_coord.linear.x = self.cmd_vel_in_agent_coord.linear.x * np.cos(
            yaw
        ) - self.cmd_vel_in_agent_coord.linear.y * np.sin(yaw)
        self.cmd_vel_in_world_coord.linear.y = self.cmd_vel_in_agent_coord.linear.x * np.sin(
            yaw
        ) + self.cmd_vel_in_agent_coord.linear.y * np.cos(yaw)
        self.cmd_vel_in_world_coord.angular.z = self.cmd_vel_in_agent_coord.angular.z

        activate_field_cbf = self.get_parameter("activate_field_cbf").get_parameter_value().bool_value

        x_limit = self.get_parameter("field.x_limit").get_parameter_value().double_array_value
        y_limit = self.get_parameter("field.y_limit").get_parameter_value().double_array_value

        self.cent_field = np.array([sum(x_limit) / len(x_limit), sum(y_limit) / len(y_limit)])
        self.width = np.array([(x_limit[1] - x_limit[0]) / 2.0, (y_limit[1] - y_limit[0]) / 2.0])
        self.theta = self.get_parameter("field.theta").get_parameter_value().double_value
        keep_inside = self.get_parameter("field.keep_inside").get_parameter_value().bool_value

        if activate_field_cbf:
            self.optimizer.set_field_parameters(self.cent_field, self.width, theta=self.theta, keep_inside=keep_inside)
            agent_position = np.array([msg.position.x, msg.position.y])
            nominal_input = np.array(
                [
                    self.cmd_vel_in_world_coord.linear.x,
                    self.cmd_vel_in_world_coord.linear.y,
                ]
            )
            _, optimal_input = self.optimizer.optimize(nominal_input, agent_position)
            self.cmd_vel_in_world_coord.linear.x = float(optimal_input[0])
            self.cmd_vel_in_world_coord.linear.y = float(optimal_input[1])

    def visualize_field(self):
        marker = Marker()
        marker.header = Header(frame_id=self.world_frame, stamp=self.get_clock().now().to_msg())
        marker.ns = "field_area"
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.CYLINDER
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        marker.pose = Pose(
            position=Point(x=self.cent_field[0], y=self.cent_field[1], z=0.0),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )
        marker.scale = Vector3(x=self.width[0] * 2, y=self.width[1] * 2, z=0.1)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        self.field_visualization_pub.publish(marker)

    def timer_callback(self) -> None:
        try:
            self.visualize_field()
        except Exception as e:
            self.get_logger().error(f"{e}")
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
