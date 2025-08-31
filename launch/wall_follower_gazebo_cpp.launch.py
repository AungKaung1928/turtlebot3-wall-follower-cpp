#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Complete launch file for C++ wall follower with Gazebo simulation
    """
    # Get directories
    pkg_dir = get_package_share_directory('wall_following_cpp_project')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # World file (using TurtleBot3 world)
    world = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Full path to world file to load'
        ),
        
        DeclareLaunchArgument(
            'x_pose', 
            default_value='-2.0',
            description='Initial x position of robot'
        ),
        
        DeclareLaunchArgument(
            'y_pose', 
            default_value='-0.5',
            description='Initial y position of robot'
        ),
        
        DeclareLaunchArgument(
            'z_pose', 
            default_value='0.01',
            description='Initial z position of robot'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Launch Gazebo with TurtleBot3 world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_gazebo_dir, '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),
        
        # Wait a bit for Gazebo to load, then launch wall follower
        # Note: In practice, you might want to add a delay here
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_dir, '/launch/wall_following_cpp.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),
    ])