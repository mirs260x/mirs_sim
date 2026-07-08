import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    default_robot_model_path = os.path.join(pkg_share, 'urdf', 'mirs_gazebo.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    # URDF読み込み
    robot_description = ParameterValue(
            Command(['xacro ', default_robot_model_path]),
            value_type=str
    )
    
    # robot_state_publisher 配信
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Joint State Publisher 起動
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # RViz起動
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
