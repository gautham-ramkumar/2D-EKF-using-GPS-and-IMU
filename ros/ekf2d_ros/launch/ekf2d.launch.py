"""Play the bag, run the estimators on it, and compare them in RViz.

    ros2 launch ekf2d_ros ekf2d.launch.py bag:=/path/to/my_ros2_bag_data
    ros2 launch ekf2d_ros ekf2d.launch.py bag:=... rate:=10 rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICS = ['/imu/imu_uncompensated', '/gps/fix', '/vehicle/twist']


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('ekf2d_ros'), 'rviz', 'ekf2d.rviz')
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to the ROS 2 bag folder or .db3 file'),
        DeclareLaunchArgument('rate', default_value='5.0', description='Bag playback speed'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),

        # Everything runs on the bag's clock (/clock from `ros2 bag play --clock`).
        Node(
            package='ekf2d_ros',
            executable='ekf2d_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # Give the node and RViz a moment to subscribe before data starts.
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate, '--topics', *TOPICS],
                output='screen',
            ),
        ]),
    ])
