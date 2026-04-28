"""Warehouse sim + slam_toolbox online_async + SLAM RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_MAP_FILE = os.path.join(
    os.path.normpath(os.path.join(get_package_share_directory('warehouse_sim'),
                                  '..', '..', '..', '..')),
    'maps', 'warehouse_map'
)


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_sim')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    slam_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    base_launch = os.path.join(pkg_share, 'launch', 'warehouse.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart_mapper = LaunchConfiguration('autostart_mapper')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated /clock from Gazebo.')

    declare_autostart_mapper = DeclareLaunchArgument(
        'autostart_mapper', default_value='true',
        description='Start frontier exploration so mapping begins without teleop.')

    warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_rviz': 'false',
        }.items(),
    )

    slam_extra: dict = {'use_sim_time': use_sim_time}
    if os.path.exists(_MAP_FILE + '.posegraph'):
        slam_extra['map_file_name'] = _MAP_FILE
        slam_extra['map_start_at_dock'] = False
        print(f'[slam.launch] Loading existing map: {_MAP_FILE}')
    else:
        print('[slam.launch] No existing map — starting fresh.')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, slam_extra],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', slam_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    mapper = Node(
        package='warehouse_sim',
        executable='autonomous_mapper.py',
        namespace='hikbot',
        name='autonomous_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'linear_speed': 0.50,
            'slow_linear_speed': 0.25,
            'max_angular_speed': 0.90,
            'startup_turn_speed': 0.60,
            'front_clearance': 0.90,
            'emergency_clearance': 0.42,
            'slow_clearance': 1.35,
            'side_clearance': 0.55,
            'startup_spin_duration': 7.0,
            'obstacle_inflation_radius': 0.32,
            'min_frontier_size': 14,
            'replan_interval': 1.5,
            'max_frontier_plans': 5,
            'max_astar_expansions': 60000,
        }],
        condition=IfCondition(autostart_mapper),
    )

    map_manager = Node(
        package='warehouse_sim',
        executable='map_manager_gui.py',
        name='map_manager',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart_mapper,
        warehouse,
        slam,
        lifecycle_manager,
        mapper,
        map_manager,
        rviz,
    ])
