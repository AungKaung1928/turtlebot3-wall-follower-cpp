#!/usr/bin/env python3
"""Controller only: use when a simulator (or the real robot) is already publishing /scan."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('wall_following_cpp_project')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'wall_following_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='wall_following_cpp_project',
            executable='wall_follower_cpp',
            name='wall_follower_controller',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
            emulate_tty=True,
        ),
    ])
