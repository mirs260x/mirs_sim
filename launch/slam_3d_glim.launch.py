"""Gazeboの首振り点群と機体IMUをGLIMへ渡して、CPU版の3D SLAMを起動する。

入力点群はEKFのodomで移動補償した蓄積点群で、機体IMUと同一原点・同一軸として扱う。
GLIMのTFはglim_map -> glim_odom -> glim_baseに分離し、EKFのodom TFと競合させない。
Nav2とRTAB-Mapは起動しない。設定・出力先はlaunch引数で指定する。
"""

import atexit
import json
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _glim_node(context):
    """apt版の既定設定へ指定JSONを重ね、GLIMノードの起動アクションを返す。"""
    source = os.path.expanduser(LaunchConfiguration('glim_config_path').perform(context))
    defaults = os.path.join(get_package_share_directory('glim'), 'config')
    # 配布パッケージの設定を変更せず、プロセス終了時に消す専用ディレクトリを使う。
    runtime = tempfile.mkdtemp(prefix='mirs_glim_')
    atexit.register(shutil.rmtree, runtime, ignore_errors=True)
    shutil.copytree(defaults, runtime, dirs_exist_ok=True)
    shutil.copytree(source, runtime, dirs_exist_ok=True)
    ros_file = os.path.join(runtime, 'config_ros.json')
    with open(ros_file, encoding='utf-8') as stream:
        ros_config = json.load(stream)
    if LaunchConfiguration('enable_glim_viewer').perform(context).lower() == 'true':
        ros_config['glim_ros']['extension_modules'].append('libstandard_viewer.so')
    with open(ros_file, 'w', encoding='utf-8') as stream:
        json.dump(ros_config, stream, indent=2)
    output = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('dump_path').perform(context)))
    os.makedirs(output, exist_ok=True)
    return [Node(
        package='glim_ros', executable='glim_rosnode', name='glim',
        parameters=[{'use_sim_time': True, 'config_path': runtime, 'dump_path': output}],
        # 公式PPAのGTSAM 4.3をGLIM内で優先し、ROS配布の4.2とのABI混在を防ぐ。
        additional_env={'LD_LIBRARY_PATH': '/usr/local/lib:' + os.environ.get('LD_LIBRARY_PATH', '')},
        output='screen')]


def generate_launch_description():
    """world・GLIM設定・保存先・ビューア設定を受け取り、センサとSLAMを起動する。"""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('enable_teleop', default_value='true'),
        DeclareLaunchArgument('cloud_assembling_time', default_value='2.0'),
        DeclareLaunchArgument('enable_glim_viewer', default_value='false'),
        DeclareLaunchArgument('dump_path', default_value='/home/developer/ws/maps/simulation/glim_dump'),
        DeclareLaunchArgument('glim_config_path', default_value=PathJoinSubstitution([
            FindPackageShare('simulation'), 'config', 'glim'])),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '91'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
            launch_arguments={
                'enable_slam': 'false', 'enable_nav2': 'false',
                'enable_nodding_lidar': 'true', 'diagnostic_stage': 'cloud',
                'enable_ground_segmentation': 'false', 'enable_rviz': 'false',
                'enable_rtabmap_viz': 'false',
                'enable_teleop': LaunchConfiguration('enable_teleop'),
                'world': LaunchConfiguration('world'),
            }.items()),
        Node(
            package='rtabmap_util', executable='point_cloud_assembler',
            name='glim_input_assembler', parameters=[{
                'use_sim_time': True, 'fixed_frame_id': 'odom', 'frame_id': 'base_link',
                'assembling_time': ParameterValue(
                    LaunchConfiguration('cloud_assembling_time'), value_type=float),
                'circular_buffer': False, 'voxel_size': 0.05,
                'wait_for_transform': 0.5, 'qos': 1,
            }], remappings=[('cloud', '/points'), ('assembled_cloud', '/glim/input_points')],
            output='screen'),
        OpaqueFunction(function=_glim_node),
    ])
