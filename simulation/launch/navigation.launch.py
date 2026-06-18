import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')

    params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # 1. Include SLAM Toolbox (for mapping and localization)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # 2. Include Navigation 2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        slam_toolbox,
        nav2
    ])
