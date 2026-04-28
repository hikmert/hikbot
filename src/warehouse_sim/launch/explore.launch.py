"""Autonomous exploration: Gazebo + SLAM + Nav2 + frontier explorer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_sim')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    warehouse_launch = os.path.join(pkg, 'launch', 'warehouse.launch.py')
    slam_launch = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
    explorer_script = os.path.join(pkg, 'scripts', 'frontier_explorer.py')
    rviz_cfg = os.path.join(pkg, 'rviz', 'slam.rviz')

    use_sim_time = {'use_sim_time': True}

    # Nav2 nodes remap cmd_vel → /hikbot/cmd_vel so DiffDrive receives it.
    nav2_remappings = [('cmd_vel', '/hikbot/cmd_vel')]

    # ── 1. Gazebo + bridges + TF (no built-in RViz) ──────────────────
    warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(warehouse_launch),
        launch_arguments={'use_sim_time': 'true', 'launch_rviz': 'false'}.items(),
    )

    # ── 2. SLAM (3 s delay — Gazebo needs to load first) ─────────────
    slam = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'true',
            }.items(),
        )],
    )

    # ── 3. Nav2 stack (8 s delay — SLAM needs to publish /map first) ──
    nav2_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[nav2_params, use_sim_time],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                output='screen',
                parameters=[nav2_params, use_sim_time],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params, use_sim_time],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params, use_sim_time],
                remappings=nav2_remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params, use_sim_time],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ],
                }],
            ),
        ],
    )

    # ── 4. Frontier explorer (18 s delay — Nav2 lifecycle needs ~10 s) ─
    explorer = TimerAction(
        period=18.0,
        actions=[Node(
            package='warehouse_sim',
            executable='frontier_explorer.py',
            name='frontier_explorer',
            output='screen',
            parameters=[use_sim_time],
        )],
    )

    # ── 5. RViz ───────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_explore',
        arguments=['-d', rviz_cfg],
        parameters=[use_sim_time],
        output='screen',
    )

    return LaunchDescription([warehouse, slam, nav2_nodes, explorer, rviz])
