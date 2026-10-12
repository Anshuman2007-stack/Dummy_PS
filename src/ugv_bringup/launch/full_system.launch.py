"""Stage 6 - End-to-end system.

Gazebo world + robot + bridge + true_pose_gps   (my_bot_sim/sim.launch.py)
EKF + navigation controller + goal input        (ugv_nav/nav_stack.launch.py)

Usage:
  ros2 launch ugv_bringup full_system.launch.py
  ros2 launch ugv_bringup full_system.launch.py headless:=true goal_input:=none
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('my_bot_sim'), 'launch', 'sim.launch.py')
    nav_launch = os.path.join(
        get_package_share_directory('ugv_nav'), 'launch', 'nav_stack.launch.py')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={'goal_input': LaunchConfiguration('goal_input')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo server only (no GUI)'),
        DeclareLaunchArgument('goal_input', default_value='auto',
                              description='auto | xterm | gnome-terminal | none'),
        sim,
        nav,
    ])
