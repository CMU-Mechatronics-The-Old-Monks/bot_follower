#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.duration = 5  # Move for 5 seconds
        # initialize robot position to 0,0
        self.robot_position = (0, 0)



    def timer_callback(self):
        self.get_logger().info('Timer callback triggered')
        msg = Twist()
        elapsed_time = time.time() - self.start_time

        if elapsed_time < self.duration:
            # Move forward
            msg.linear.x = 0.1
            msg.linear.y = 0.1
            msg.angular.z = 0.0
            self.get_logger().info(f'Moving forward: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        else:
            # Stop moving
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Stopping robot.')

        self.publisher_.publish(msg)
        if elapsed_time >= self.duration + 1:
            self.get_logger().info('Shutting down node.')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()