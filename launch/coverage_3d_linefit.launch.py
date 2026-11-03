"""Linefitの非地面点群で自己位置推定とNav2の清掃範囲走行を起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Linefit用の既定地図を設定し、共通の3D清掃走行launchへ委譲する。"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path', default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'ws', 'maps', 'simulation',
                'forest_linefit_3d.db'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'coverage_3d.launch.py'])),
            launch_arguments={
                'ground_method': 'linefit',
                'database_path': LaunchConfiguration('database_path'),
            }.items()),
    ])
