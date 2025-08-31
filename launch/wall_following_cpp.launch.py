#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for C++ wall following controller
    """
    # Get package directory
    pkg_dir = get_package_share_directory('wall_following_cpp_project')
    
    # Parameters file path
    params_file = os.path.join(pkg_dir, 'config', 'wall_following_params.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
        
        # Wall follower C++ node
        Node(
            package='wall_following_cpp_project',
            executable='wall_follower_cpp',
            name='wall_follower_controller_cpp',
            parameters=[
                params_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0
        ),
    ])