#!/usr/bin/env python3
"""
Launch file: Carter MOLA SLAM with Intensity
Launches:
1. Add Intensity Node (converts /carter/lidar -> /carter/lidar_with_intensity)
2. MOLA LiDAR Odometry (subscribes to /carter/lidar_with_intensity)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Launch argument for MOLA GUI
        DeclareLaunchArgument(
            'use_mola_gui',
            default_value='False',
            description='Enable MOLA GUI visualization'
        ),

        # Node 1: Add Intensity
        Node(
            package='zisaac_sim_test',  # Adjust if needed
            executable='add_intensity_node.py',
            name='add_intensity_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Node 2: MOLA LiDAR Odometry
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'mola_lidar_odometry', 'ros2-lidar-odometry.launch.py',
                'lidar_topic_name:=/carter/lidar_with_intensity',
                ['use_mola_gui:=', LaunchConfiguration('use_mola_gui')]
            ],
            output='screen'
        ),
    ])
