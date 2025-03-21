#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import functools
from cdf_msgs.msg import PositionMap
from localization.math_lidar import quaternion_from_euler

class MyTfBroadcaster(Node):
    def __init__(self):
        super().__init__('my_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.get_logger().info(f"UHDUUADH")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("update_position_topic", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
            ]
        )

        update_position_topic = self.get_parameter("update_position_topic").get_parameter_value().string_value
        self.robot_names = self.get_parameter("robot_names").get_parameter_value().string_array_value
        self.get_logger().info(f"POsition_topic: {update_position_topic}")
        self.get_logger().info(f"Robot_names: {self.robot_names}")
        for name in self.robot_names:
            self.sub_update_position = self.subscriber(PositionMap, update_position_topic, functools.partial(self.broadcast_tf, name=name))

    def broadcast_tf(self, msg, name):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "laser_frame"
        # Define the translation (x, y, z)
        t.transform.translation.x = msg.robot.x
        t.transform.translation.y = msg.robot.y
        t.transform.translation.z = 0.0
        # Define the rotation (using a quaternion)
        # Here, no rotation is applied (roll=pitch=yaw=0)
        q = quaternion_from_euler(0., 0., msg.robot.z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)
        self.get_logger().info('Broadcasting transform from "world" to "robot"')

def main(args=None):
    rclpy.init(args=args)
    node = MyTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()