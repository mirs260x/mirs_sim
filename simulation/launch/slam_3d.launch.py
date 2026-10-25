"""Run Gazebo with the nodding LiDAR pipeline and create a 3D RTAB-Map database."""

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
                'enable_slam': 'true',
                'enable_nav2': 'false',
                'enable_nodding_lidar': 'true',
                'enable_ground_segmentation': 'true',
                'enable_teleop': 'true',
                'enable_rtabmap_viz': 'true',
                'enable_rviz': 'true',
            }.items()),
    ])
