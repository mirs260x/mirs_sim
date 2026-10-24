"""Start 3D map localization, Nav2, and coverage in Gazebo."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_slam': 'false',
                'enable_nav2': 'true',
                'enable_coverage': 'true',
                'enable_nodding_lidar': 'true',
                'enable_icp_odometry': 'true',
                'enable_ground_segmentation': 'true',
                'enable_ble': 'true',
                'enable_teleop': 'false',
                'enable_rtabmap_viz': 'true',
                'enable_rviz': 'true',
            }.items()),
    ])
