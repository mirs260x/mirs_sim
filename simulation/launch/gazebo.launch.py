import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world_path = os.path.join(pkg_share, 'worlds', 'building_robot.sdf')

    rviz_launch = os.path.join(
            get_package_share_directory("simulation"),
            "launch",
            "rviz.launch.py"
    )

    # Gazebo Sim Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', default_world_path]}.items()
    )

    # Spawn Robot Node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mirs',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge between ROS 2 and Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch)
        ),
        gazebo,
        spawn_robot,
        bridge
    ])
