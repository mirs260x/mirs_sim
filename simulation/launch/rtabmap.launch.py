# Requirements:
#   RGB-D カメラ + LiDAR を使った 3D SLAM (RTAB-Map) with Gazebo Harmonic
#
# センサー構成:
#   - RealSense D435 相当のカメラ (RGB + Depth)
#     /camera/color/image_raw        : RGBカメラ
#     /camera/color/camera_info      : RGBカメラ info
#     /camera/depth/image_raw        : Depthカメラ
#     /camera/depth/camera_info      : Depthカメラ info
#   - 2D LiDAR
#     /scan
#   - オドメトリ
#     /odom

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # ---- RTAB-Map パラメータ ----
    parameters = {
          # フレーム設定
          'frame_id':                   'base_footprint',
          'odom_frame_id':              'odom',
          'map_frame_id':               'map',
          'publish_tf':                 True,
          'odom_tf_linear_variance':    0.001,
          'odom_tf_angular_variance':   0.001,

          # センサー購読設定 (rgbd_sync同期トピックを使用)
          'subscribe_rgbd':             True,   # 同期済みRGBDトピックを購読
          'subscribe_depth':            False,
          'subscribe_rgb':              False,
          'subscribe_scan':             True,   # 2D LiDARを追加で使う
          'approx_sync':                True,
          'sync_queue_size':            10,

          # ----- 3D SLAM 設定 -----
          'Reg/Force3DoF':              'false', # 3D SLAMを有効化 (falseで3Dに)
          'Grid/3D':                    'true',  # 3Dグリッドマップを生成

          # レジストレーション戦略
          'Reg/Strategy':               '1',    # 0=Visual, 1=ICP, 2=Visual+ICP

          # RGBD設定
          'RGBD/NeighborLinkRefining':  'true',
          'RGBD/ProximityBySpace':      'true',
          'RGBD/ProximityByTime':       'false',
          'RGBD/ProximityPathMaxNeighbors': '10',
          'RGBD/OptimizeFromGraphEnd':  'false',
          'RGBD/OptimizeMaxError':      '4',
          'RGBD/LocalRadius':           '5',

          # Visual特徴量
          'Vis/MinInliers':             '12',

          # ICPパラメータ
          'Icp/CorrespondenceRatio':    '0.2',
          'Icp/PM':                     'false',
          'Icp/PointToPlane':           'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':              '0.05',

          # 占有格子マップ
          'Grid/FromDepth':             'true',  # DepthカメラからOccupancy Gridを生成
          'Grid/MaxGroundHeight':       '0.1',   # 地面とみなす最大高さ
          'Grid/MaxObstacleHeight':     '2.0',   # 障害物とみなす最大高さ

          # メモリ管理
          'Mem/STMSize':                '30',


          'cloud_output_voxel_size':    0.05,
          'cloud_noise_filtering_radius': 0.0,
          'cloud_noise_filtering_min_neighbors': 5
    }

    # ---- トピックリマッピング ----
    sync_remappings = [
        ('rgb/image',                  '/camera/color/image_raw'),
        ('rgb/camera_info',            '/camera/color/camera_info'),
        ('depth/image',                '/camera/depth/image_raw'),
        ('rgbd_image',                 '/rgbd_image'),
    ]

    slam_remappings = [
        ('rgbd_image',                 '/rgbd_image'),
        ('scan',                       '/scan'),
    ]

    return LaunchDescription([

        # Launch 引数
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),

        SetParameter(name='use_sim_time', value=True),

        # --- RGBD 同期ノード ---
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            # Gazebo の負荷で RGB/Depth の周期が揺れるため、同期許容幅を広げる
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.2, 'use_sim_time': True}],
            remappings=sync_remappings,
        ),

        # --- SLAM モード ---
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=slam_remappings,
            arguments=['-d'],  # 前回のデータベースを削除して新規マッピング
        ),

        # --- ローカライゼーション モード ---
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory': 'False',
               'Mem/InitWMWithAllNodes': 'True'}],
            remappings=slam_remappings,
        ),

        # --- rtabmap_viz (3D可視化) ---
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[parameters],
            remappings=slam_remappings,
        ),
    ])
