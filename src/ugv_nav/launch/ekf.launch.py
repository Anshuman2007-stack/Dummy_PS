"""Stage 2 - EKF only (needs /odom/wheels, /imu/data, /gnss/fix from the sim)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_nav'), 'config', 'ugv_nav.yaml')
    return LaunchDescription([
        Node(package='ugv_nav', executable='ekf_node', name='ekf_node',
             output='screen', parameters=[params]),
    ])
