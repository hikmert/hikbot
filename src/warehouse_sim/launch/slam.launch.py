"""Warehouse sim + slam_toolbox online_async + SLAM RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_sim')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    slam_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    base_launch = os.path.join(pkg_share, 'launch', 'warehouse.launch.py')
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated /clock from Gazebo.')

    warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': 'false',
        }.items(),
    )

    # Use slam_toolbox's own launch which handles LifecycleNode configure+activate.
    slam = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'true',
            }.items(),
        )],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', slam_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([declare_use_sim_time, warehouse, slam, rviz])
