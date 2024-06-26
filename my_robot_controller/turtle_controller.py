#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self.previous_x_ = 0.0
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.get_logger().info("Turtle Controller Node Started...")

    def pose_callback(self, msg: Pose):
        cmd = Twist()

        if msg.x > 9.0 or msg.x <2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9

        else:    
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_.publish(cmd)

        if msg.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = msg.x
            self.call_set_pen_service(255, 0, 0, 5, 0)
            self.get_logger().info("set color to Red")

        elif msg.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = msg.x
            self.call_set_pen_service(0, 0, 255, 5, 0)
            self.get_logger().info("set color to Blue")

    def call_set_pen_service(self, r, b, g, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()