"""Start Gazebo, the robot, ROS/Gazebo and Foxglove bridges, and sensors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _enabled(context, name):
    return _value(context, name).lower() in ('true', '1', 'yes')


def _optional_path(context, name, default):
    return _value(context, name) or default


def _base_link_height_above_ground(context):
    """Match the base_link offset used by mirs.urdf.xacro."""
    return 2.0 * float(_value(context, 'wheel_radius')) + 0.109


def _point_cloud_assembler(name, input_topic, output_topic, circular=False):
    """Assemble one nodding sweep in the robot frame using odom for motion."""
    return Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name=name,
        parameters=[{'use_sim_time': True}, {
            'assembling_time': ParameterValue(
                LaunchConfiguration('cloud_assembling_time'), value_type=float),
            'fixed_frame_id': 'odom',
            'frame_id': 'base_link',
            'wait_for_transform': 0.5,
            'voxel_size': 0.1,
            'qos': 1,
            'circular_buffer': circular,
        }],
        remappings=[
            ('cloud', input_topic),
            ('assembled_cloud', output_topic),
        ],
        output='screen')


def _rtabmap_actions(is_3d, localization, database_path, show_viz,
                     initial_pose=''):
    """Run RTAB-Map against the same LiDAR and odometry used in Gazebo."""
    odometry_topic = '/odometry/filtered' if is_3d else '/odom'
    parameters = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'odom_frame_id': '',
        'odom_frame_id_init': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_odom': True,
        'subscribe_scan_cloud': is_3d,
        'subscribe_scan': not is_3d,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'Rtabmap/DetectionRate': '10',
        'log_to_rosout_level': 2,
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/Sensor': '0',
        'Grid/3D': 'true' if is_3d else 'false',
        'Grid/RangeMax': '10',
        'database_path': database_path,
    }
    if is_3d:
        parameters.update({
            'Icp/PointToPlane': 'false',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MaxGroundHeight': '-0.12',
            'Grid/RayTracing': 'true',
        })
    if localization:
        parameters.update({
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        })
        if initial_pose:
            # Override the last localization pose stored in the database.
            parameters['initial_pose'] = initial_pose

    remappings = [
        ('scan_cloud', '/points_assembled_icp'),
        ('scan', '/scan'),
        ('odom', odometry_topic),
        ('grid_map', '/map'),
    ]
    actions = [Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[parameters],
        arguments=[] if localization else ['--delete_db_on_start'],
        remappings=remappings,
        output='screen')]
    if show_viz:
        actions.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            parameters=[parameters],
            remappings=remappings,
            output='screen'))
    return actions


def _launch_setup(context):
    is_3d = _enabled(context, 'enable_nodding_lidar')
    mapping = _enabled(context, 'enable_slam')
    navigation = _enabled(context, 'enable_nav2')
    diagnostic_stage = _value(context, 'diagnostic_stage')
    stages = ('full', 'scan', 'cloud', 'filtered', 'assembled', 'localization')
    if diagnostic_stage not in stages:
        raise RuntimeError(f'diagnostic_stage must be one of: {", ".join(stages)}')
    if diagnostic_stage != 'full' and (not is_3d or mapping or navigation):
        raise RuntimeError(
            'diagnostic_stage requires 3D LiDAR and disables SLAM and Nav2')

    def stage_at_least(name):
        return diagnostic_stage == 'full' or stages.index(diagnostic_stage) >= stages.index(name)

    if mapping and navigation:
        raise RuntimeError('enable_slam and enable_nav2 cannot both be true')
    share = get_package_share_directory('simulation')
    sim_time = {'use_sim_time': True}
    world = _optional_path(
        context, 'world',
        os.path.join(share, 'worlds',
                     'nodding_forest.sdf' if is_3d else 'park.sdf'))

    ekf_config = _optional_path(
        context, 'ekf_config_file',
        os.path.join(share, 'config', 'nodding_ekf.yaml'))

    # Keep simulation DDS traffic isolated from the real MIRS stack. The
    # physical micro-ROS firmware uses domain 90; Gazebo uses domain 91.
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', '91')]
    if _enabled(context, 'enable_foxglove_bridge'):
        foxglove_share = get_package_share_directory('foxglove_bridge')
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                foxglove_share, 'launch', 'foxglove_bridge_launch.xml')),
            launch_arguments={
                'address': _value(context, 'foxglove_address'),
                'port': _value(context, 'foxglove_port'),
            }.items()))
    if _enabled(context, 'enable_gazebo'):
        gz_share = get_package_share_directory('ros_gz_sim')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r ', world],
            }.items()))

    if _enabled(context, 'enable_robot'):
        description = ParameterValue(
            Command([
                'xacro ', LaunchConfiguration('robot_model_file'),
                ' nodding:=', 'true' if is_3d else 'false',
                ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
                ' wheel_track:=', LaunchConfiguration('wheel_track'),
                ' wheel_mu_longitudinal:=', LaunchConfiguration('wheel_mu_longitudinal'),
                ' wheel_mu_lateral:=', LaunchConfiguration('wheel_mu_lateral'),
                ' caster_mu:=', LaunchConfiguration('caster_mu'),
            ]),
            value_type=str)
        actions.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[sim_time, {'robot_description': description}],
                output='screen'),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'mirs', '-topic', 'robot_description',
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                ],
                parameters=[sim_time],
                output='screen'),
        ])

    if _enabled(context, 'enable_bridge'):
        # 2D publishes the fixed laser scan directly as /scan. In 3D,
        # nodding_io republishes /scan_raw after adding the servo state.
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[sim_time, {
                'config_file': LaunchConfiguration('bridge_config_file'),
            }],
            # Keep Gazebo's raw odometry TF available only on a diagnostic
            # topic in 3D mode; robot_localization owns odom -> base_footprint.
            remappings=[('/tf', '/tf_gazebo')] if is_3d else [('/scan_raw', '/scan')],
            output='screen'))

    if is_3d:
        if stage_at_least('cloud'):
            actions.extend([
                Node(
                    package='simulation',
                    executable='sensor_covariance_adapter',
                    parameters=[sim_time, {
                        'wheel_linear_velocity_variance': ParameterValue(
                            LaunchConfiguration('wheel_linear_velocity_variance'),
                            value_type=float),
                        'wheel_angular_velocity_variance': ParameterValue(
                            LaunchConfiguration('wheel_angular_velocity_variance'),
                            value_type=float),
                        'imu_yaw_rate_variance': ParameterValue(
                            LaunchConfiguration('imu_yaw_rate_variance'),
                            value_type=float),
                    }],
                    output='screen'),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node_local',
                    parameters=[sim_time, ekf_config],
                    output='screen'),
            ])
        actions.append(Node(
            package='simulation',
            executable='nodding_io',
            parameters=[sim_time],
            output='screen'))
        if stage_at_least('cloud'):
            actions.append(Node(
                package='nodding_lidar',
                executable='scan_to_cloud',
                parameters=[LaunchConfiguration('cloud_config_file')],
                output='screen'))
        # RTAB-Map scan registration uses stable returns above the grass.
        # Ground classification still sees the full cloud.
        if stage_at_least('filtered'):
            actions.append(Node(
                package='simulation',
                executable='pointcloud_height_filter',
                name='icp_height_filter',
                parameters=[sim_time, {
                    'input_topic': '/points',
                    'output_topic': '/points_icp_filtered',
                    'min_height_above_ground': ParameterValue(
                        LaunchConfiguration('icp_min_height'), value_type=float),
                    'base_link_height_above_ground':
                        _base_link_height_above_ground(context),
                }],
                output='screen'))
        if stage_at_least('assembled'):
            actions.append(_point_cloud_assembler(
                'icp_point_cloud_assembler',
                '/points_icp_filtered', '/points_assembled_icp'))
        if diagnostic_stage == 'full' and _enabled(context, 'enable_ground_segmentation'):
            # GSeg3D needs the full nodding sweep to fit ground planes. The
            # circular window updates every incoming scan after the first sweep.
            actions.append(_point_cloud_assembler(
                'ground_point_cloud_assembler',
                '/points', '/points_assembled', circular=True))
            actions.append(Node(
                package='ground_segmentation_ros2',
                executable='ground_segmentation_ros2_node',
                name='ground_segmentation',
                parameters=[LaunchConfiguration('ground_config_file'), {
                    'lidar_to_ground': -_base_link_height_above_ground(context),
                }],
                remappings=[
                    ('/ground_segmentation/input_pointcloud', '/points_assembled'),
                    ('/ground_segmentation/input_imu', '/imu/data_raw'),
                ],
                output='screen'))

    localization_debug = diagnostic_stage == 'localization'
    if mapping or navigation or localization_debug:
        sensor_mode = '3d' if is_3d else '2d'
        database_path = os.path.expanduser(_optional_path(
            context, 'database_path',
            os.path.join(os.path.expanduser('~'), '.ros',
                         f'mirs_gazebo_{sensor_mode}.db')))
        if (navigation or localization_debug) and not os.path.isfile(database_path):
            raise RuntimeError(
                f'RTAB-Map database not found: {database_path}. '
                f'Run slam_{sensor_mode}.launch.py first or set database_path.')
        actions.extend(_rtabmap_actions(
            is_3d, navigation or localization_debug, database_path,
            _enabled(context, 'enable_rtabmap_viz'),
            _value(context, 'rtabmap_initial_pose')))

    if navigation:
        nav2_params = _optional_path(
            context, 'nav2_params_file',
            os.path.join(share, 'config',
                         'nodding_nav2.yaml' if is_3d else 'nav2_params.yaml'))
        nav2_share = get_package_share_directory('nav2_bringup')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params,
            }.items()))

    if _enabled(context, 'enable_rviz'):
        rviz_config = _optional_path(
            context, 'rviz_config_file',
            os.path.join(share, 'rviz',
                         'nodding.rviz' if is_3d else 'mapping.rviz'))
        actions.append(Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time], output='screen'))
    if _enabled(context, 'enable_teleop'):
        actions.append(Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e', output='screen'))

    return actions


def generate_launch_description():
    share = get_package_share_directory('simulation')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value=os.path.join(share, 'urdf', 'mirs_gazebo.urdf.xacro')),
        DeclareLaunchArgument(
            'bridge_config_file',
            default_value=os.path.join(share, 'config', 'nodding_bridge.yaml')),
        DeclareLaunchArgument(
            'cloud_config_file',
            default_value=os.path.join(share, 'config', 'nodding_cloud.yaml')),
        DeclareLaunchArgument('cloud_assembling_time', default_value='2.0'),
        DeclareLaunchArgument(
            'icp_min_height', default_value='0.5',
            description='Minimum ICP point height above the simulated ground (m)'),
        DeclareLaunchArgument('wheel_linear_velocity_variance', default_value='0.04'),
        DeclareLaunchArgument('wheel_angular_velocity_variance', default_value='0.09'),
        DeclareLaunchArgument('imu_yaw_rate_variance', default_value='0.01'),
        DeclareLaunchArgument(
            'ground_config_file',
            default_value=os.path.join(share, 'config', 'nodding_ground.yaml')),
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value=os.path.join(share, 'config', 'nodding_ekf.yaml')),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0665'),
        DeclareLaunchArgument('wheel_track', default_value='0.465'),
        DeclareLaunchArgument('wheel_mu_longitudinal', default_value='1.0'),
        DeclareLaunchArgument('wheel_mu_lateral', default_value='0.2'),
        DeclareLaunchArgument('caster_mu', default_value='0.001'),
        DeclareLaunchArgument('enable_gazebo', default_value='true'),
        DeclareLaunchArgument('enable_robot', default_value='true'),
        DeclareLaunchArgument('enable_bridge', default_value='true'),
        DeclareLaunchArgument('enable_foxglove_bridge', default_value='true'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('enable_nodding_lidar', default_value='false'),
        DeclareLaunchArgument('diagnostic_stage', default_value='full'),
        DeclareLaunchArgument('enable_ground_segmentation', default_value='false'),
        DeclareLaunchArgument('enable_slam', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='false'),
        DeclareLaunchArgument('enable_teleop', default_value='false'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('database_path', default_value=''),
        DeclareLaunchArgument('rtabmap_initial_pose', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument('rviz_config_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
