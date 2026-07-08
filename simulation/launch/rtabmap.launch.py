# Requirements:
#   Stereo camera SLAM using RTAB-Map with Gazebo simulation
#
# Example:
#
#   SLAM:
#     $ ros2 launch simulation mapping.launch.py
#
# Stereo camera topics (from ros_gz_bridge):
#   /camera/left/image_raw
#   /camera/right/image_raw
#   /camera/left/image_raw/camera_info
#   /camera/right/image_raw/camera_info
#   /scan
#   /odom

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters = {
          'frame_id':                  'base_footprint',
          'odom_frame_id':             'odom',
          'odom_tf_linear_variance':   0.001,
          'odom_tf_angular_variance':  0.001,
          # Stereo subscription
          'subscribe_stereo':          True,
          'subscribe_scan':            True,
          'approx_sync':               True,
          'sync_queue_size':           10,
          # RTAB-Map's internal parameters
          'RGBD/NeighborLinkRefining':    'true',
          'RGBD/ProximityBySpace':        'true',
          'RGBD/ProximityByTime':         'false',
          'RGBD/ProximityPathMaxNeighbors': '10',
          'Reg/Strategy':                 '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':               '12',
          'RGBD/OptimizeFromGraphEnd':    'false',
          'RGBD/OptimizeMaxError':        '4',
          'Reg/Force3DoF':                'true',    # 2D SLAM
          'Grid/FromDepth':               'false',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':                  '30',
          'RGBD/LocalRadius':             '5',
          'Icp/CorrespondenceRatio':      '0.2',
          'Icp/PM':                       'false',
          'Icp/PointToPlane':             'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':                '0.05',
    }

    stereo_remappings = [
        ('left/image_rect',        '/camera/left/image_raw'),
        ('right/image_rect',       '/camera/right/image_raw'),
        ('left/camera_info',       '/camera/left/image_raw/camera_info'),
        ('right/camera_info',      '/camera/right/image_raw/camera_info'),
    ]

    slam_remappings = stereo_remappings + [
        ('scan',                   '/scan'),
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),

        SetParameter(name='use_sim_time', value=True),

        # Stereo image synchronizer
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.04, 'use_sim_time': True}],
            remappings=stereo_remappings,
        ),

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=slam_remappings,
            arguments=['-d'],  # Delete previous database (~/.ros/rtabmap.db)
        ),

        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory': 'False',
               'Mem/InitWMWithAllNodes': 'True'}],
            remappings=slam_remappings,
        ),

        # Visualization (rtabmap_viz):
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[parameters],
            remappings=slam_remappings,
        ),
    ])
