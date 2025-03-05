#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import sin, cos

class SimulateMotion(Node):
    def __init__(self):
        super().__init__('simulate_motion')
        # Subscribe to /cmd_vel for velocity commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Create a TransformBroadcaster for TF transforms
        self.br = TransformBroadcaster(self)
        # Create a publisher for odometry messages
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Initialize the robot's position and orientation
        self.x = 0.0  # x position (meters)
        self.y = 0.0  # y position (meters)
        self.theta = 0.0  # Orientation (radians)

        # Track the last time a velocity command was processed
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        """
        Callback function that processes velocity commands from /cmd_vel.
        Updates the robot's position and publishes odometry and TF transforms.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Time delta in seconds

        # Update the robot's position based on the received velocity command
        self.x += msg.linear.x * dt * cos(self.theta) - msg.linear.y * dt * sin(self.theta)
        self.y += msg.linear.x * dt * sin(self.theta) + msg.linear.y * dt * cos(self.theta)
        self.theta += msg.angular.z * dt

        # Broadcast the updated transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert theta to quaternion for rotation around Z-axis
        t.transform.rotation.z = sin(self.theta / 2.0)
        t.transform.rotation.w = cos(self.theta / 2.0)

        # Send the transform using the TransformBroadcaster
        self.br.sendTransform(t)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position and orientation in the odometry message
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta / 2.0)

        # Set linear and angular velocities in the odometry message
        odom_msg.twist.twist.linear.x = msg.linear.x
        odom_msg.twist.twist.linear.y = msg.linear.y
        odom_msg.twist.twist.angular.z = msg.angular.z

        # Log odometry data for debugging purposes
        self.get_logger().info(f'Publishing Odometry: x={self.x}, y={self.y}, theta={self.theta}')
        
        # Publish the odometry message to /odom topic
        self.odom_publisher.publish(odom_msg)

        # Update the last time for velocity integration
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimulateMotion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
