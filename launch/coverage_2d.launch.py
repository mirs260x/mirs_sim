"""Gazeboで2D地図による位置推定、Nav2、BLE入力付きの清掃範囲走行を起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_params = PathJoinSubstitution([
        FindPackageShare('simulation'), 'config', 'nav2_params.yaml'])
    behavior_tree = PathJoinSubstitution([
        FindPackageShare('simulation'), 'behavior_trees',
        'navigate_coverage_return_home.xml'])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('database_path', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params),
        DeclareLaunchArgument('enable_ble', default_value='false'),
        DeclareLaunchArgument('enable_monitor', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_slam': 'false',
                'enable_nav2': 'true',
                'enable_nodding_lidar': 'false',
                'enable_ground_segmentation': 'false',
                'enable_teleop': 'false',
                'enable_rtabmap_viz': LaunchConfiguration('enable_rtabmap_viz'),
                'enable_rviz': LaunchConfiguration('enable_rviz'),
                'world': LaunchConfiguration('world'),
                'database_path': LaunchConfiguration('database_path'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('coverage'), 'launch', 'coverage.launch.py'])),
            launch_arguments={
                'enable_mirs': 'false',
                'enable_coverage_server': 'true',
                'behavior_tree': behavior_tree,
                'enable_ble': LaunchConfiguration('enable_ble'),
                'enable_monitor': LaunchConfiguration('enable_monitor'),
                'use_rviz': 'false',
                'use_sim_time': 'true',
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            }.items()),
    ])
