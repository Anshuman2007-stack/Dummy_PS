"""Stage 1 - Sensor bring-up.

Starts Gazebo with the world, spawns the robot, bridges the Gazebo topics to
ROS 2 and starts the GPS node that synthesises /gnss/fix from the true pose.

Published ROS topics:
  /odom/wheels  nav_msgs/Odometry    (wheel-encoder odometry, 50 Hz)
  /imu/data     sensor_msgs/Imu      (100 Hz)
  /gnss/fix     sensor_msgs/NavSatFix (5 Hz, from true_pose_gps)
  /true_odom    nav_msgs/Odometry    (ground truth - for evaluation only)
Subscribed:
  /cmd_vel      geometry_msgs/Twist
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', 'my_world.sdf')
    model = os.path.join(pkg, 'models', 'my_bot', 'model.sdf')

    headless = LaunchConfiguration('headless')

    # '-r' = start running, '-s' = server only (no GUI)
    gz_flags = PythonExpression(
        ["'-r -s ' if '", headless, "'.lower() == 'true' else '-r '"])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flags, world]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', model, '-name', 'my_bot',
                   '-x', '0', '-y', '0', '-z', '0.05'])

    # [ = Gazebo -> ROS,  ] = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/my_bot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/true_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/my_bot/odometry', '/odom/wheels'),
            ('/imu', '/imu/data'),
            ('/navsat', '/gnss/fix'),
        ])

    # GPS synthesised from the ground-truth pose + Gaussian noise
    true_pose_gps = Node(
        package='my_bot_sim', executable='true_pose_gps.py',
        name='true_pose_gps', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = run Gazebo server only (no GUI)'),
        gz, spawn, bridge, true_pose_gps,
    ])
