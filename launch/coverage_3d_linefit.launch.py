"""Linefitの非地面点群で自己位置推定とNav2の清掃範囲走行を起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Linefit設定・地図DB・BLE入力の設定を受け取り、清掃走行を起動する。"""
    nav2_params = PathJoinSubstitution([
        FindPackageShare('simulation'), 'config', 'nodding_nav2.yaml'])
    behavior_tree = PathJoinSubstitution([
        FindPackageShare('simulation'), 'behavior_trees',
        'navigate_coverage_return_home.xml'])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('ground_method', default_value='linefit'),
        DeclareLaunchArgument('linefit_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('simulation'), 'config', 'linefit_ground.yaml'])),
        DeclareLaunchArgument(
            'database_path',
            default_value='/home/developer/ws/maps/simulation/forest_linefit_3d.db'),
        DeclareLaunchArgument(
            'rtabmap_initial_pose', default_value='0 0 0 0 0 0'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params),
        DeclareLaunchArgument('enable_ble', default_value='true'),
        DeclareLaunchArgument('enable_monitor', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'ground_method': LaunchConfiguration('ground_method'),
                'linefit_config_file': LaunchConfiguration('linefit_config_file'),
                'enable_slam': 'false',
                'enable_nav2': 'true',
                'enable_nodding_lidar': 'true',
                'enable_ground_segmentation': 'true',
                'enable_teleop': 'false',
                'enable_rtabmap_viz': LaunchConfiguration('enable_rtabmap_viz'),
                'enable_rviz': LaunchConfiguration('enable_rviz'),
                'world': LaunchConfiguration('world'),
                'database_path': LaunchConfiguration('database_path'),
                'rtabmap_initial_pose': LaunchConfiguration('rtabmap_initial_pose'),
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
                'monitor_profile': '3d',
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            }.items()),
    ])
