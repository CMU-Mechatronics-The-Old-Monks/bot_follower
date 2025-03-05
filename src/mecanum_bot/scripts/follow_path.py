#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class FollowPath(Node):
    def __init__(self):
        super().__init__('follow_path')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Define the path as a list of waypoints (x, y)
        self.path = [(0, 0), (3, 3)]
        self.current_waypoint_index = 0
        self.current_position = None  # Start with no odometry data
        self.tolerance = 0.5

        # Kickstart simulation with an initial velocity command
        self.kickstart_simulation()

    def kickstart_simulation(self):
        """Send an initial velocity command to start the simulation."""
        twist = Twist()
        twist.linear.x = 0.1  # Small forward motion to trigger /odom updates
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Sent initial kickstart command.')
        self.current_waypoint_index = 1

    def odom_callback(self, msg):
        """Callback function triggered when new odometry data is received."""
        # Update current position from odometry
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.get_logger().info(f'Received /odom: Current position: {self.current_position}')

        # Follow the path using updated position
        self.follow_path()

    def follow_path(self):
        """Follow the predefined path based on current position."""
        if self.current_position is None:
            # Wait for odometry data before proceeding
            self.get_logger().info('Waiting for /odom data...')
            return

        if self.current_waypoint_index >= len(self.path):
            # Stop if all waypoints have been reached
            self.get_logger().info('Path completed.')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            return

        # Get the current waypoint and current position
        target_x, target_y = self.path[self.current_waypoint_index]
        current_x, current_y = self.current_position

        # Calculate distance to the target waypoint
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        if distance < self.tolerance:
            # Move to the next waypoint if close enough to the current one
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
            self.current_waypoint_index += 1

            # Log transition to the next waypoint
            if self.current_waypoint_index < len(self.path):
                next_target_x, next_target_y = self.path[self.current_waypoint_index]
                self.get_logger().info(f'Moving to next waypoint: ({next_target_x}, {next_target_y})')
            return

        # Calculate direction to the target waypoint
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)

        # Publish velocity commands to move toward the target waypoint
        twist = Twist()
        twist.linear.x = min(0.5, distance) * math.cos(angle_to_target)
        twist.linear.y = min(0.5, distance) * math.sin(angle_to_target)
        twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing Twist: linear.x={twist.linear.x}, linear.y={twist.linear.y}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowPath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
