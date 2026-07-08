import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')

    # 1. Gazebo + RViz + Bridge with mapping.rviz config
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'rviz_config': os.path.join(pkg_share, 'rviz', 'mapping.rviz')}.items()
    )

    # 2. RTAB-Map
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rtabmap.launch.py')
        )
    )

    # 3. Teleop for manual driving (opens in xterm)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',  # Open in new terminal
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        gazebo_launch,
        rtabmap_launch,
        teleop,
    ])
