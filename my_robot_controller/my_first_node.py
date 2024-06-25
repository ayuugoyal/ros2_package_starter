#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Create a new class that is a subclass of the Node class and inherits all its Node functionalities from the rclpy library
class MyNode(Node):

    # Constructor to initialize the node
    def __init__(self):
        super().__init__('first_node')
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    # Callback function that is called periodically
    def timer_callback(self):
        self.get_logger().info('Hello ji ROS2 suru krte hai! ' + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    # To initialize ROS 2 Communication using rclpy
    rclpy.init(args=args)

    # Create a node
    node = MyNode()
 
    # Keep the node running inifinitely until it is shutdown or kill it by ctrl + c
    rclpy.spin(node)

    # Shutdown the node and all the ROS 2 Communication
    rclpy.shutdown()    

if __name__ == '__main__':
    main()