"""機器を起動せずlaunchを展開し、実機とGazeboの通信・TF・速度指令の分離を検査する。"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


_PACKAGES = ('simulation', 'mirs', 'nodding_lidar', 'coverage')


def _value(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _load(package, name):
    path = Path(get_package_share_directory(package)) / 'launch' / name
    spec = importlib.util.spec_from_file_location(package + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _expand(package, name, overrides=None):
    """自作launchだけを展開し、プロセス実行の代わりに構成と環境を返す。"""
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    context.environment['ROS_DOMAIN_ID'] = '91' if package != 'simulation' else '90'
    nodes, includes = [], []

    def visit(action):
        condition = getattr(action, "condition", None)
        if condition is not None and not condition.evaluate(context):
            return
        if isinstance(action, Node):
            nodes.append((_value(context, action.node_executable),
                          dict(context.launch_configurations), dict(context.environment), action))
            return
        if isinstance(action, IncludeLaunchDescription):
            children = action.execute(context)
            location = action.launch_description_source.location
            if any('/share/' + p + '/launch/' in location or
                   '/original/' + p + '/launch/' in location for p in _PACKAGES):
                for child in children:
                    visit(child)
            else:
                # Nav2等の外部launchは起動せず、渡された引数だけを記録する。
                args = {k: _value(context, v) for k, v in action.launch_arguments}
                includes.append((location, args, dict(context.environment)))
            return
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration,
                               SetEnvironmentVariable, OpaqueFunction, GroupAction)):
            for child in action.execute(context) or []:
                visit(child)
            return
        if isinstance(action, LaunchDescription):
            for child in action.entities:
                visit(child)
            return
        if isinstance(action, ExecuteProcess):
            return
        # GroupActionが返すPush/Popで設定と環境の復元だけを実行する。
        if type(action).__name__.startswith(('Push', 'Pop')):
            action.execute(context)

    visit(_load(package, name))
    return nodes, includes


def test_hardware_nodding_nav_has_one_laser_tf_owner_and_safety_gate():
    """首振り走行ではドライバ・laser TF・安全ゲートをそれぞれ一つにする。"""
    nodes, includes = _expand('nodding_lidar', 'nav_launch.py')
    assert all(env['ROS_DOMAIN_ID'] == '90' for _, _, env, _ in nodes)
    assert sum(exe == 'servo_tf_node' for exe, *_ in nodes) == 1
    assert sum(exe == 'scan_safety_guard' for exe, *_ in nodes) == 1
    assert not any(exe == 'static_transform_publisher' and
                   _value(LaunchContext(), node._Node__node_name) ==
                   'enable_static_base_link_to_laser_tf'
                   for exe, _, _, node in nodes)
    drivers = [(loc, args) for loc, args, _ in includes if 'sllidar_s1' in loc]
    assert len(drivers) == 1


def test_hardware_bringup_preserves_driver_flag_and_rejects_two_localizers():
    """includeの引数漏れと位置推定の二重配信を防ぐ。"""
    nodes, includes = _expand('nodding_lidar', 'bringup_launch.py', {'enable_nav2': 'true'})
    assert sum(exe == 'scan_safety_guard' for exe, *_ in nodes) == 1
    assert sum('sllidar_s1' in loc for loc, *_ in includes) == 1
    with pytest.raises(RuntimeError, match='map -> odom'):
        _expand('nodding_lidar', 'bringup_launch.py',
                {'enable_nav2': 'true', 'enable_rtabmap': 'true'})


def test_hardware_nav_preserves_coverage_plugin_and_has_no_command_cycle():
    """設定ファイルの清掃プラグインと指令経路を保つ。"""
    nodes, includes = _expand('mirs', 'mirs_launch.py', {'enable_nav2': 'true'})
    nav_args = next(args for loc, args, _ in includes if 'bringup_launch.py' in loc)
    params = yaml.safe_load(Path(nav_args['params_file']).read_text())
    bt = params['bt_navigator']['ros__parameters']
    assert bt['navigate_complete_coverage']['plugin'] == 'opennav_coverage_navigator/CoverageNavigator'
    output = params['collision_monitor']['ros__parameters']['cmd_vel_out_topic']
    assert output not in ('cmd_vel_nav', 'cmd_vel_smoothed')
    guard = next(node for exe, _, _, node in nodes if exe == 'scan_safety_guard')
    context = LaunchContext()
    # 数値等のlaunch設定は、当該ノードまでに解決された値を使う。
    context.launch_configurations.update(next(config for exe, config, _, _ in nodes
                                             if exe == 'scan_safety_guard'))
    guard_params = evaluate_parameters(context, guard._Node__parameters)[0]
    assert guard_params['input_cmd_vel_topic'] == '/' + output
    assert guard_params['output_cmd_vel_topic'] == '/cmd_vel'


@pytest.mark.parametrize('launch_file', ['coverage_2d.launch.py', 'coverage_3d.launch.py',
                                         'coverage_3d_linefit.launch.py'])
def test_simulation_coverage_never_starts_hardware_by_default(tmp_path, launch_file):
    """各清掃入口で実機用資源を起動せず、Domain 91と専用設定を使う。"""
    database = tmp_path / 'map.db'
    database.touch()  # launchのパス検査用。RTAB-Mapプロセスは起動しない。
    nodes, includes = _expand('simulation', launch_file, {'database_path': str(database)})
    assert nodes
    assert all(env['ROS_DOMAIN_ID'] == '91' for _, _, env, _ in nodes)
    assert not any(exe in ('micro_ros_agent', 'serial_angle_bridge', 'servo_node',
                           'ble_receiver_node', 'scan_safety_guard') for exe, *_ in nodes)
    assert not any('ble_receiver.launch.py' in loc or 'sllidar_s1' in loc for loc, *_ in includes)
    fox = next(args for loc, args, _ in includes if 'foxglove_bridge_launch' in loc)
    assert fox['port'] == '8766'
    nav_args = next(args for loc, args, _ in includes if 'navigation_launch.py' in loc)
    assert str(Path(get_package_share_directory('simulation')) / 'config') in nav_args['params_file']
    assert all(env['ROS_DOMAIN_ID'] == '91' for _, _, env in includes)
    if launch_file != 'coverage_2d.launch.py':
        assert sum(exe == 'ekf_node' for exe, *_ in nodes) == 1
        assert sum(exe == 'rtabmap' for exe, *_ in nodes) == 1
        assert not any('localization_launch.py' in loc for loc, *_ in includes)


def test_hardware_nav_supplies_odometry_tf_and_rejects_two_map_tf_sources():
    """実機Nav2のodom TFを用意し、独立SLAMとの競合を拒否する。"""
    nodes, _ = _expand('mirs', 'nav_launch.py')
    assert sum(exe == 'ekf_node' for exe, *_ in nodes) == 1
    with pytest.raises(RuntimeError, match='map -> odom'):
        _expand('mirs', 'mirs_launch.py',
                {'enable_nav2': 'true', 'enable_slam_toolbox': 'true'})


def test_hardware_coverage_has_imu_transform_for_ekf():
    """実機清掃の入口でIMU観測をEKFへ変換できるTFを用意する。"""
    nodes, _ = _expand('coverage', 'production.launch.py')
    assert sum(exe == 'ekf_node' for exe, *_ in nodes) == 1
    assert any(exe == 'static_transform_publisher' and
               _value(LaunchContext(), node._Node__node_name) == 'base_link_to_imu_link'
               for exe, _, _, node in nodes)
