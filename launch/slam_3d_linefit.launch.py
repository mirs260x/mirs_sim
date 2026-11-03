"""首振りLiDARの蓄積点群をLinefitで地面分離し、RTAB-Mapの3D地図を作成する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """地図DB・world・Linefit設定を受け取り、地図作成と手動操作を起動する。"""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument(
            'database_path', default_value='/home/developer/ws/maps/simulation/forest_linefit_3d.db'),
        DeclareLaunchArgument('linefit_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('simulation'), 'config', 'linefit_ground.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_slam': 'true',
                'enable_nav2': 'false',
                'enable_nodding_lidar': 'true',
                'enable_ground_segmentation': 'true',
                'enable_teleop': 'true',
                'enable_rtabmap_viz': 'true',
                'enable_rviz': 'true',
                'ground_method': 'linefit',
                'world': LaunchConfiguration('world'),
                'database_path': LaunchConfiguration('database_path'),
                'linefit_config_file': LaunchConfiguration('linefit_config_file'),
            }.items()),
    ])
