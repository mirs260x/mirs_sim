"""Start 3D localization, Nav2, and coverage with BLE in Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_params = PathJoinSubstitution([
        FindPackageShare('simulation'), 'config', 'nodding_nav2.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('database_path', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params),
        DeclareLaunchArgument('enable_ble', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_slam': 'false',
                'enable_nav2': 'true',
                'enable_nodding_lidar': 'true',
                'enable_icp_odometry': 'true',
                'enable_ground_segmentation': 'true',
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
                'enable_ble': LaunchConfiguration('enable_ble'),
                'use_rviz': 'false',
                'use_sim_time': 'true',
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            }.items()),
    ])
