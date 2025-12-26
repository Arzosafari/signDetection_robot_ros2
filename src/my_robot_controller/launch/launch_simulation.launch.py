#!/usr/bin/env python3
# فایل: my_complete_simulation.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Directories ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_robot_controller = get_package_share_directory('my_robot_controller')

    # --- Launch configs ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # --- Path to your custom world ---
    world_path = os.path.join(pkg_my_robot_controller, 'worlds', 'roadmap.world')

    # --- Gazebo ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # --- Spawn TurtleBot ---
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # --- Your Nodes (اضافه شده) ---
    camera_node = Node(
        package='my_robot_controller',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )
    
    detect_node = Node(
        package='my_robot_controller',
        executable='detect_node',
        name='detect_node',
        output='screen'
    )
    
    move_node = Node(
        package='my_robot_controller',
        executable='move_node',
        name='move_node',
        output='screen'
    )

    # --- Combine all ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        
        gzserver,
        gzclient,
        spawn_robot,
        camera_node,
        detect_node,
        move_node,
    ])
