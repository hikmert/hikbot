"""Launch Gazebo Harmonic warehouse, spawn the hikbot robot, run ros_gz bridges and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Static TFs we publish ourselves so RViz can resolve sensor frames.
# Each entry: (parent, child, x, y, z, roll, pitch, yaw)
STATIC_TFS = [
    ('base_link', 'imu_link',          0.00,  0.00, 0.05, 0.0, 0.0,  0.0),
    ('base_link', 'cam0_link',         0.25,  0.03, 0.10, 0.0, 0.0,  0.0),
    ('base_link', 'cam1_link',         0.25, -0.03, 0.10, 0.0, 0.0,  0.0),
    ('base_link', 'laser_front_link',  0.27,  0.00, 0.04, 0.0, 0.0,  0.0),
    ('base_link', 'laser_back_link',  -0.27,  0.00, 0.04, 0.0, 0.0,  3.14159265),
    ('base_link', 'laser_left_link',   0.00,  0.18, 0.04, 0.0, 0.0,  1.5707963),
    ('base_link', 'laser_right_link',  0.00, -0.18, 0.04, 0.0, 0.0, -1.5707963),
    ('base_link', 'lidar_link',        0.00,  0.00, 0.13, 0.0, 0.0,  0.0),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    robot_sdf = os.path.join(pkg_share, 'models', 'hikbot', 'model.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'warehouse.rviz')
    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')
    models_dir = os.path.join(pkg_share, 'models')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated /clock from Gazebo.')

    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Launch default RViz with the warehouse config.')

    # Make our local models discoverable by Gazebo (for any future <include>s)
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hikbot',
        output='screen',
        arguments=[
            '-world', 'warehouse',
            '-file', robot_sdf,
            '-name', 'hikbot',
            '-x', '0', '-y', '0', '-z', '0.15',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Sensor + cmd_vel + odom bridge driven by YAML; topics are scoped under /hikbot.
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hikbot_bridge',
        namespace='hikbot',
        parameters=[{
            'config_file': bridge_yaml,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # Publish odom→base_link TF from the Odometry topic.
    # The gz Pose_V→TFMessage bridge is unreliable in Gazebo Harmonic.
    tf_bridge = Node(
        package='warehouse_sim',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_nodes = []
    for parent, child, x, y, z, roll, pitch, yaw in STATIC_TFS:
        static_tf_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{child}',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                '--frame-id', parent, '--child-frame-id', child,
            ],
            output='log',
        ))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_rviz,
        set_resource_path,
        gz_sim,
        spawn_robot,
        clock_bridge,
        sensor_bridge,
        tf_bridge,
        *static_tf_nodes,
        rviz,
    ])
