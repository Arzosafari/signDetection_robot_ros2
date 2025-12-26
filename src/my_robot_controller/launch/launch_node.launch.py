#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Package directory ---
    pkg_my_robot_controller = get_package_share_directory('my_robot_controller')
    
    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Camera Node ---
    camera_node = Node(
        package='my_robot_controller',
        executable='node_camera',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ]
    )

    # --- Detection Node ---
    detect_node = Node(
        package='my_robot_controller',
        executable='node_detect',
        name='detect_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/command', '/detection_command'),
        ]
    )

    # --- Motor Control Node ---
    move_node = Node(
        package='my_robot_controller',
        executable='node_controll_motor',
        name='move_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/direction_command', '/detection_command'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        
        # Launch all three nodes
        camera_node,
        detect_node,
        move_node,
    ])
