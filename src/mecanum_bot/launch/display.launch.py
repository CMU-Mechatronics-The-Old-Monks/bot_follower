from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/tariq/ros2_ws2/src/mecanum_bot/urdf/mecanum_bot.urdf'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]  # Set robot_description parameter
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
