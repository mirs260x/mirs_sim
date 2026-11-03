"""3Dセンサの処理を指定した段階まで起動する。Nav2と清掃範囲走行は起動しない。"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'stage', default_value='scan',
            description='scan, cloud, filtered, assembled, or localization'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument(
            'database_path', default_value='/home/developer/ws/maps/simulation/mirs_gazebo_3d.db'),
        DeclareLaunchArgument('cloud_assembling_time', default_value='2.0'),
        DeclareLaunchArgument('icp_min_height', default_value='0.5'),
        DeclareLaunchArgument('enable_foxglove_bridge', default_value='false'),
        DeclareLaunchArgument('enable_monitor', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '91'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_nodding_lidar': 'true',
                'enable_slam': 'false',
                'enable_nav2': 'false',
                'enable_rviz': 'false',
                'enable_rtabmap_viz': 'false',
                'enable_foxglove_bridge': LaunchConfiguration('enable_foxglove_bridge'),
                'bridge_config_file': PathJoinSubstitution([
                    FindPackageShare('simulation'), 'config',
                    'diagnose_3d_bridge.yaml']),
                'diagnostic_stage': LaunchConfiguration('stage'),
                'world': LaunchConfiguration('world'),
                'database_path': LaunchConfiguration('database_path'),
                'cloud_assembling_time': LaunchConfiguration('cloud_assembling_time'),
                'icp_min_height': LaunchConfiguration('icp_min_height'),
                'rtabmap_initial_pose': '0 0 0 0 0 0',
            }.items()),
        Node(
            package='coverage', executable='coverage_monitor',
            name='coverage_monitor', output='screen',
            parameters=[{
                'use_sim_time': True,
                'profile': 'sensor',
                'diagnostic_stage': LaunchConfiguration('stage'),
            }],
            condition=IfCondition(LaunchConfiguration('enable_monitor'))),
    ])
