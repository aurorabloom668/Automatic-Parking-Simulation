#!/usr/bin/env python3
"""预写几何路径泊车入口。

整条任务在 mission.py 里解析拼接好，节点只负责跟踪与可视化。
点选地图车位 + 在线规划走 auto_park.launch.py。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_car_demo = get_package_share_directory('car_demo')
    pkg_auto_parking = get_package_share_directory('auto_parking')

    params = os.path.join(pkg_auto_parking, 'config', 'parking.yaml')
    rviz_config = os.path.join(pkg_auto_parking, 'rviz', 'planned_park.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='park.world'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.5'),
        DeclareLaunchArgument('yaw', default_value='3.14159265'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_car_demo, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': rviz_config,
                'frame': 'odom',
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
                'yaw': LaunchConfiguration('yaw'),
            }.items()
        ),

        Node(
            package='auto_parking',
            executable='gt_odom',
            name='gt_odom',
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='auto_parking',
            executable='planned_park',
            name='planned_park',
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='auto_parking',
            executable='proximity_warn',
            name='proximity_warn',
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        ),
    ])
