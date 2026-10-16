#!/usr/bin/env python3
"""Line-delimited IPC; events and replies share stdout with serialized writes."""
import json
import os
from pathlib import Path
import signal
import sys
import subprocess
import threading

from profiles import Inspector, command_for, config_values
from runtime import Supervisor
from machine import NativeSession
from model_runtime import ModelRuntime
from persistence import (export_friction_candidate, export_telemetry, export_candidate_urdf, load_model_calibration_summary, list_model_calibration_records, preview_gravity_correction, restore_gravity_correction, save_gravity_correction, update_candidate_urdf_verification, preview as preview_config, save as save_config)
from profile_library import ProfileLibrary
from model_calibration_alignment import compare_calibration_urdfs

ROOT = Path(__file__).resolve().parents[3]
output_lock = threading.Lock()


def run_full_inertial_isolated(directory, destination=None, *, timeout=120, mode=None, locked_links=None):
    """Never load optional native Pinocchio extensions in the live IPC process"""
    argv = [sys.executable, str(Path(__file__).with_name('full_inertial_worker.py')),
            str(Path(directory).expanduser().resolve()), str(destination or '')]
    argv.extend([mode or 'full', json.dumps(locked_links or [])])
    env = dict(os.environ)
    # Avoid a user-site NumPy 2 overriding the ROS Humble NumPy 1 ABI
    env['PYTHONNOUSERSITE'] = '1'
    operation = {
        'legacy_gravity_recompute': '离线重力模型辨识',
        'legacy_gravity_export': '离线重力模型导出',
    }.get(mode, '高级动力学辨识')
    try:
        completed = subprocess.run(argv, capture_output=True, text=True,
                                   timeout=timeout, env=env)
    except subprocess.TimeoutExpired as error:
        raise ValueError(f'{operation}计算超时，独立计算进程已终止，工作台仍可正常使用') from error
    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip()
        numpy_abi = ('_ARRAY_API not found' in details or
                     'A module that was compiled using NumPy 1.x' in details or
                     'numpy.core.multiarray failed to import' in details)
        if numpy_abi:
            raise ValueError('Pinocchio 与 NumPy ABI 不兼容，当前候选计算已隔离终止，不影响机器人控制界面，请重新运行 ./install.sh 更新 GUI Python 环境中的 numpy<2')
        if completed.returncode < 0:
            raise ValueError(f'{operation}进程被信号 {-completed.returncode} 终止，工作台未受影响，请检查 Python 原生依赖')
        if details.startswith(('ValueError: ', 'RuntimeError: ', 'KeyError: ')):
            details = details.split(': ', 1)[1]
        raise ValueError(f'{operation}失败: ' + (details[-2000:] or f'退出码 {completed.returncode}'))
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise ValueError(f'{operation}进程没有返回有效 JSON') from error


def restore_temporary_candidates_for_disconnect(native):
    """Park and restore process-local candidates before a normal disconnect

    Keep the session running if restoration cannot be verified
    """
    overrides = native.request('candidate_overrides', {}, timeout=8)
    enabled = [kind for kind in ('dynamics', 'gravity', 'gain')
               if overrides.get(kind + '_applied')]
    if not enabled:
        return []
    status = native.request('status', {}, timeout=8)
    if status.get('robot_state') != 'INACTIVE':
        native.request('park', {}, timeout=120)
        status = native.request('status', {}, timeout=8)
    if status.get('robot_state') != 'INACTIVE':
        raise ValueError('无法确认机械臂已安全停放，已保留连接，请检查机器人状态后重试')
    for kind in enabled:
        native.request('candidate_' + kind + '_restore', {}, timeout=12)
    remaining = native.request('candidate_overrides', {}, timeout=8)
    if any(remaining.get(kind + '_applied') for kind in ('gain', 'gravity', 'dynamics')):
        raise ValueError('临时参数尚未全部撤销，已保留连接，请检查并重试')
    return enabled


# Native sessions do not trust browser button states for control admission
_READ_ONLY_MACHINE = {
    'status', 'get_admittance', 'candidate_overrides', 'calibration_status',
    'model_calibration_status', 'candidate_gain_status', 'candidate_gravity_status',
    'candidate_dynamics_status',
}
_SAFE_EXIT_MACHINE = {'deactivate'}
_TRACKING_MACHINE = {'move_to', 'move_relative'}


def verify_machine_admission(library, config, method, params, native):
    if method in _READ_ONLY_MACHINE or method in _SAFE_EXIT_MACHINE: return
    readiness = library._readiness(config)
    info = readiness.get('info') or {}
    if info.get('write_enabled') is False:
        raise ValueError('执行器写入已关闭，当前会话不可发送硬件控制命令')
    if readiness['checks'].get('profile_setup') != 'verified':
        raise ValueError('Profile 配置未完成，拒绝硬件控制')
    if method == 'park':
        # Parking moves the arm using the configured gains; safe emergency disable remains available
        if readiness['modes'].get('RIGID_HOLD') != 'verified':
            raise ValueError('启动保持增益未验收，不能执行自动停放，请确保硬件安全后失能')
        return
    if method == 'set_gravity_scale':
        # Existing Machine accepts process-local changes only while INACTIVE
        return
    if method == 'set_model_feedforward_mode':
        value = params.get('mode')
        if value == 'NONE': return
        needed = 'gravity_verification' if value == 'GRAVITY' else 'dynamics_calibration'
        if readiness['checks'].get(needed) != 'verified':
            raise ValueError(f'{needed} 未验证，不能使用该模型前馈')
        return
    if method in ('fault_compliant',):
        if readiness['checks'].get('gravity_verification') != 'verified':
            raise ValueError('重力验证未完成，不允许柔性故障恢复')
        return
    if method in ('fault_rigid', 'hold', 'clear_fault'):
        if readiness['modes'].get('RIGID_HOLD') != 'verified':
            raise ValueError('刚性保持增益尚未通过本机验收')
        return
    if method == 'activate':
        if readiness['checks'].get('bringup') != 'verified' or readiness['modes'].get('RIGID_HOLD') != 'verified':
            raise ValueError('真机检查或启动保持增益尚未通过验收，拒绝使能')
        status = native.request('status', {}, timeout=8)
        if (status.get('model_feedforward_mode') != 'NONE' and
                readiness['checks'].get('gravity_verification') != 'verified'):
            raise ValueError('重力验证未完成，使能前请先将模型前馈设置为 NONE')
        return
    if method == 'set_impedance_mode':
        mode = params.get('mode')
        if mode not in readiness['modes'] or readiness['modes'][mode] != 'verified':
            raise ValueError(f'{mode} 未通过独立模式增益验收，已拒绝切换')
        return
    if method in _TRACKING_MACHINE:
        if any(readiness['checks'].get(stage) != 'verified' for stage in
               ('mapping_calibration', 'geometry_calibration')):
            raise ValueError('关节映射或几何校准尚未通过验证，禁止运动')
        status = native.request('status', {}, timeout=8)
        mode = status.get('impedance_mode')
        if (status.get('robot_state') != 'ACTIVE' or mode not in ('RIGID_TRACKING', 'COMPLIANT_TRACKING')
                or readiness['modes'].get(mode) != 'verified'):
            raise ValueError('当前跟踪模式尚未通过验收，拒绝运动命令')
        return
    if method.startswith(('model_calibration_', 'calibration_', 'friction_', 'candidate_',
                          'set_admittance', 'gain_')):
        # Calibration paths can internally switch to drag or tracking modes
        if (readiness['checks'].get('gravity_verification') != 'verified' or
                readiness['modes'].get('RIGID_HOLD') != 'verified'):
            raise ValueError('当前标定动作需要先完成启动保持与重力验证')
        return
    if readiness['state'] != 'ready':
        raise ValueError('控制方法缺少当前 Profile 对应的后端验收，拒绝执行')


def emit(data):
    with output_lock:
        print(json.dumps(data, ensure_ascii=False), flush=True)


def main():
    inspector = Inspector(ROOT)
    lock_dir = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'serial-arm/locks'
    manager = Supervisor(emit, lock_dir)
    native = NativeSession(emit, lock_dir)
    model_runtime = ModelRuntime()
    profile_library = ProfileLibrary(ROOT, inspector)
    active_workbench_config = None
    active_workbench_fingerprint = None
    emit({'event': 'backend_ready'})
    if '--check' in sys.argv[1:]:
        model_runtime.close()
        native.close()
        manager.close()
        return
    def stop(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        for line in sys.stdin:
            request = {}
            try:
                if len(line) > 1024 * 1024: raise ValueError('request too large')
                request = json.loads(line)
                method, params = request['method'], request.get('params', {})
                if not isinstance(params, dict): raise ValueError('params must be an object')
                if method == 'status':
                    native_status = native.status()
                    session = native_status if native_status.get('state') in ('running', 'stopping') else manager.status()
                    result = {**inspector.status(), 'session': session, 'workbench': native_status}
                elif method == 'profiles': result = inspector.profiles(params)
                elif method == 'profile_library': result = profile_library.list()
                elif method == 'profile_source_inspect': result = profile_library.discover_profile_source(params.get('source', ''))
                elif method == 'profile_library_register': result = profile_library.register(params.get('profile_file', ''), params.get('profile', ''), params.get('resource_paths', ''), params.get('policy', 'error'))
                elif method == 'profile_library_remove': result = profile_library.remove(params.get('id', ''))
                elif method == 'description_inspect': result = profile_library.inspect_description(params.get('source', ''))
                elif method == 'description_create_profile': result = profile_library.create_profile(params)
                elif method == 'readiness': result = profile_library._readiness(config_values(params.get('config', params)))
                elif method == 'readiness_mark': result = profile_library.mark_stage(config_values(params.get('config', {})), params.get('stage', ''), params.get('confirmed', True) is True)
                elif method == 'readiness_record':
                    target = config_values(params.get('config', {}))
                    stage = params.get('stage', '')
                    if stage != 'model_check':
                        if not active_workbench_config or native.status().get('state') != 'running':
                            raise ValueError('真机验收需要已绑定 Profile 的工作台会话')
                        target_info = inspector.inspect(target)
                        active_info = inspector.inspect(active_workbench_config)
                        if (target_info['fingerprint'] != active_workbench_fingerprint or
                            active_info['fingerprint'] != active_workbench_fingerprint or
                            target_info['profile_file'] != active_info['profile_file'] or
                            target_info['profile'] != active_info['profile'] or
                            target_info['resources'] != active_info['resources']):
                            raise ValueError('验收 Profile 与当前真实控制会话不一致')
                    result = profile_library.record_stage(target, stage, params.get('answers', {}), native)
                elif method == 'readiness_approve_mode': result = profile_library.approve_gain_mode(config_values(params.get('config', {})), params.get('mode', ''), params.get('candidate_path', ''), native)
                elif method == 'profile_editor_load': result = profile_library.profile_editor_load(config_values(params.get('config', params)))
                elif method == 'profile_editor_save': result = profile_library.profile_editor_save(config_values(params.get('config', {})), params.get('payload', {}))
                elif method == 'inspect': result = inspector.inspect(params)
                elif method == 'model': result = model_runtime.model(inspector, params.get('config', params), params.get('positions'))
                elif method == 'model_preview': result = model_runtime.preview(inspector, params.get('config', params), params.get('positions'))
                elif method == 'workbench_start':
                    if manager.status().get('state') in ('running', 'stopping'): raise ValueError('another Launcher session is already running')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    if not info['available'].get('workbench'): raise ValueError('structured workspace is unavailable; install Standalone terminal components')
                    admission = profile_library._readiness(config)
                    if admission['checks'].get('profile_setup') != 'verified': raise ValueError('请先完成硬件与 Profile 配置')
                    if info['write_enabled'] is not False and admission['checks'].get('model_check') != 'verified':
                        raise ValueError('请先完成模型检查，再接入真实硬件')
                    if params.get('fingerprint') != info['fingerprint']: raise ValueError('configuration changed; inspect and confirm again')
                    if info['write_enabled'] is not False:
                        if params.get('confirmed') is not True: raise ValueError('real hardware confirmation is required')
                        if any(not c['ok'] for c in info['checks'] if c['name'] == 'device'):
                            raise ValueError('device is missing or not readable/writable')
                    argv = [inspector.machine_terminal(), '--machine', '--config', info['resources']['core'],
                            '--hardware-plugin', info['hardware_plugin'], '--hardware-config', info['resources']['hardware']]
                    for key, flag in [('serial_port', '--serial-port'), ('baudrate', '--baudrate'), ('bus', '--bus')]:
                        if config[key]: argv += [flag, config[key]]
                    env = dict(os.environ)
                    if config['resource_paths']:
                        env['SERIAL_ARM_RESOURCE_PATH'] = config['resource_paths'] + (os.pathsep + env['SERIAL_ARM_RESOURCE_PATH'] if env.get('SERIAL_ARM_RESOURCE_PATH') else '')
                    result = native.start(argv, env, info['devices'] if info['write_enabled'] is not False else [])
                    active_workbench_config = config
                    active_workbench_fingerprint = info['fingerprint']
                elif method == 'workbench_request':
                    native_method = params.get('method')
                    machine_params = params.get('params', {})
                    if not isinstance(machine_params, dict): raise ValueError('控制参数必须是对象')
                    if not active_workbench_config: raise ValueError('没有已绑定 Profile 的控制会话')
                    if native_method not in _READ_ONLY_MACHINE and native_method not in _SAFE_EXIT_MACHINE:
                        info = inspector.inspect(active_workbench_config)
                        if info['fingerprint'] != active_workbench_fingerprint:
                            raise ValueError('连接期间配置发生变化，请先安全断连并重新检查')
                        verify_machine_admission(profile_library, active_workbench_config, native_method, machine_params, native)
                    result = native.request(native_method, machine_params, timeout=120 if native_method == 'park' else (45 if native_method in ('model_calibration_teach_stop','model_calibration_import_trajectory') else 8))
                elif method == 'workbench_stop':
                    # Temporary overrides are process-local, but restore explicitly
                    # before disconnect so the outcome is observable and auditable
                    # When safe parking or restoration fails keep the session alive
                    force = params.get('force') is True
                    if not force and native.status().get('state') == 'running':
                        restore_temporary_candidates_for_disconnect(native)
                    result = native.stop(force)
                    active_workbench_config = None
                    active_workbench_fingerprint = None
                elif method == 'workbench_config_preview':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    runtime = native.request('get_admittance', {}, timeout=8)
                    snapshot = native.request('status', {}, timeout=8)
                    result = preview_config(info['resources']['core'], snapshot['joint_names'], runtime, snapshot['gravity_scale'])
                elif method == 'workbench_config_save':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    runtime = native.request('get_admittance', {}, timeout=8)
                    snapshot = native.request('status', {}, timeout=8)
                    result = save_config(info['resources']['core'], params.get('expected_sha', ''), snapshot['joint_names'], runtime, snapshot['gravity_scale'])
                elif method == 'workbench_export':
                    result = export_telemetry(ROOT, native.telemetry(), {'profile': params.get('profile', ''), 'core': params.get('core', '')})
                elif method == 'calibration_candidate_prepare':
                    from workflow_registry import prepare_gain_candidate, prepare_gravity_candidate, prepare_dynamics_candidate
                    kind = params.get('kind')
                    if kind == 'gain': result = prepare_gain_candidate(params.get('directory', ''))
                    elif kind == 'gravity': result = prepare_gravity_candidate(params.get('directory', ''), prefer_recomputed=params.get('prefer_recomputed') is True)
                    elif kind == 'dynamics': result = prepare_dynamics_candidate(params.get('report', ''))
                    else: raise ValueError('未知的候选类型')
                elif method == 'calibration_candidate_apply':
                    if not active_workbench_config or native.status().get('state') != 'running':
                        raise ValueError('候选只能在绑定的工作台会话内临时应用')
                    active_info = inspector.inspect(active_workbench_config)
                    if active_info['fingerprint'] != active_workbench_fingerprint:
                        raise ValueError('配置已变化，请先安全断连并重新检查')
                    verify_machine_admission(profile_library, active_workbench_config,
                                             'candidate_' + str(params.get('kind', '')) + '_apply', {}, native)
                    from workflow_registry import prepare_gain_candidate, prepare_gravity_candidate, prepare_dynamics_candidate
                    kind = params.get('kind')
                    if kind == 'gain': result_path=prepare_gain_candidate(params.get('directory', ''))['candidate_path'];native_args={'path':result_path}
                    elif kind == 'gravity': result_path=prepare_gravity_candidate(params.get('directory', ''), prefer_recomputed=params.get('prefer_recomputed') is True)['candidate_path'];native_args={'path':result_path}
                    elif kind == 'dynamics': result_path=prepare_dynamics_candidate(params.get('report', ''))['report'];native_args={'report':result_path}
                    else: raise ValueError('未知的候选类型')
                    native.request('candidate_'+kind+'_apply', native_args, timeout=12)
                    result=native.request('candidate_overrides',{},timeout=8)
                elif method == 'calibration_candidate_restore':
                    if not active_workbench_config or native.status().get('state') != 'running':
                        raise ValueError('当前没有可恢复候选的工作台会话')
                    kind = params.get('kind')
                    if kind not in ('gain','gravity','dynamics'): raise ValueError('未知的候选类型')
                    native.request('candidate_'+kind+'_restore', {}, timeout=12)
                    result=native.request('candidate_overrides',{},timeout=8)
                elif method == 'calibration_candidate_status':
                    result = native.request('candidate_overrides', {}, timeout=8)
                elif method == 'calibration_tool_records':
                    from workflow_registry import scan
                    result = scan(ROOT, params.get('kind', ''), params.get('limit', 60))
                elif method == 'calibration_tool_load':
                    from workflow_registry import load
                    result = load(params.get('directory', ''), params.get('kind', ''))
                elif method == 'calibration_tool_export':
                    from artifact_exports import export_artifact
                    result = export_artifact(ROOT, params.get('directory', ''), params.get('kind', ''),
                        report=params.get('report'), destination=params.get('destination'),
                        candidate_source=params.get('candidate_source','online'))
                elif method == 'calibration_tool_export_record':
                    from workflow_registry import export_record
                    result = export_record(ROOT, params.get('directory', ''), params.get('kind', ''))
                elif method == 'model_calibration_load':
                    result = load_model_calibration_summary(params.get('directory', ''))
                elif method == 'model_calibration_model_alignment':
                    directory = params.get('directory', '')
                    summary = load_model_calibration_summary(directory)
                    current_model = model_runtime.model(inspector, params.get('config', {}))
                    result = compare_calibration_urdfs(summary['directory'], summary['metadata'], current_model['urdf'])
                elif method == 'model_calibration_records':
                    result = list_model_calibration_records(ROOT)
                elif method == 'model_calibration_preview_save':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = preview_gravity_correction(info['resources']['core'], directory)
                elif method == 'model_calibration_save':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = save_gravity_correction(info['resources']['core'], params.get('expected_sha', ''), directory)
                elif method == 'model_calibration_restore_config':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    result = restore_gravity_correction(info['resources']['core'], params.get('directory', ''))
                elif method == 'model_calibration_export_friction':
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = export_friction_candidate(directory, params.get('destination') or None)
                elif method == 'model_calibration_export_urdf':
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    source = params.get('candidate_source', 'online')
                    if source == 'offline':
                        if native.status().get('state') in ('running', 'stopping'):
                            raise ValueError('离线候选导出前请先断开工作台')
                        result = run_full_inertial_isolated(directory, params.get('destination') or None,
                            mode='legacy_gravity_export')
                    elif source == 'online':
                        result = export_candidate_urdf(directory, params.get('destination') or None)
                    else:
                        raise ValueError('未知候选来源')
                    # Independent verification is informative for manual-review
                    # candidates and must never abort the IPC loop
                    try:
                        # Verify the exact exported URDF for both candidate sources
                        # without re-fitting, applying commands or inheriting the online pass state
                        calibrator = inspector.model_calibrator()
                        if not calibrator:
                            raise ValueError('离线校验工具未安装，候选 URDF 已导出供人工审核')
                        info = inspector.inspect(config_values(params.get('config', {})))
                        verification = subprocess.run(
                            [calibrator, '--config', info['resources']['core'],
                             '--dataset', str(Path(directory).resolve()),
                             '--verify-urdf', result['candidate_urdf']],
                            text=True, capture_output=True, timeout=60)
                        if verification.returncode != 0:
                            raise ValueError((verification.stderr or verification.stdout or
                                              '候选 URDF 离线验证失败').strip()[-1500:])
                        verified = json.loads(verification.stdout.strip().splitlines()[-1])
                        result['verification'] = update_candidate_urdf_verification(
                            Path(result['candidate_urdf']).parent, verified)
                    except (ValueError, OSError, subprocess.TimeoutExpired, json.JSONDecodeError, IndexError, KeyError) as error:
                        result['verification_warning'] = str(error)
                elif method == 'model_calibration_assess':
                    if native.status().get('state') in ('running', 'stopping'):
                        raise ValueError('请先安全停放并断开机器人工作台，再对历史标定执行离线数据分析')
                    from advanced_identification import assess_record
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = assess_record(directory)
                elif method == 'model_calibration_advanced':
                    from advanced_identification import MODES
                    if native.status().get('state') in ('running', 'stopping'):
                        raise ValueError('高级离线辨识需要先安全停放并断开机器人工作台，避免长时间计算阻塞控制请求')
                    directory = params.get('directory', '')
                    mode = params.get('mode', '')
                    if mode not in MODES:
                        raise ValueError('未知的高级辨识目标')
                    locked = params.get('locked_links', [])
                    if not isinstance(locked,list) or any(not isinstance(x,str) or not x for x in locked) or len(set(locked))!=len(locked):
                        raise ValueError('需要不重复的 Link 名称清单')
                    if not directory or not Path(directory).expanduser().is_dir():
                        raise ValueError('请选择已保存并完成标定的历史任务')
                    result = run_full_inertial_isolated(directory, params.get('destination') or None,
                        mode=mode, locked_links=locked)
                elif method == 'model_calibration_export_full_inertial':
                    raise ValueError('完整惯量候选已归入高级动力学辨识，请选择目标参数与可信 Link 后再执行离线辨识')
                elif method == 'model_calibration_recompute':
                    if native.status().get('state') in ('running', 'stopping'):
                        raise ValueError('离线重算前请先安全断开机器人工作台')
                    directory = params.get('directory', '')
                    if not directory or not Path(directory).expanduser().is_dir():
                        raise ValueError('请选择完整的标定任务目录')
                    mode = params.get('mode', 'global')
                    if mode not in ('global', 'local'):
                        raise ValueError('未知的辨识模式')
                    locked = params.get('locked_links', []) if mode == 'local' else []
                    if (not isinstance(locked, list) or len(set(locked)) != len(locked)
                            or any(not isinstance(x, str) or not x for x in locked)):
                        raise ValueError('可信 Link 列表无效')
                    result = run_full_inertial_isolated(directory, mode='legacy_gravity_recompute',
                        locked_links={'mode': mode, 'locked_links': locked})
                elif method == 'start':
                    if native.status().get('state') in ('running', 'stopping'): raise ValueError('another Launcher session is already running')
                    mode = params.get('mode')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    if not info['available'].get(mode): raise ValueError('selected run mode is unavailable; inspect installation and resources')
                    if params.get('fingerprint') != info['fingerprint']: raise ValueError('configuration changed; inspect and confirm again')
                    if params.get('confirmed') is not True: raise ValueError('real hardware confirmation is required')
                    if info['write_enabled'] is not False and profile_library._readiness(config)['state'] != 'ready':
                        raise ValueError('当前 Profile 未获得完整控制验收，请使用受限制的控制工作台完成验证')
                    if any(not c['ok'] for c in info['checks'] if c['name'] == 'device'):
                        raise ValueError('device is missing or not readable/writable')
                    config['profile_file'] = info['profile_file']
                    argv, extra = command_for(mode, config)
                    result = manager.start(argv, dict(os.environ, **extra, TERM='xterm-256color'),
                                           info['devices'], mode)
                elif method == 'input': result = manager.input(params.get('data'))
                elif method == 'resize': result = manager.resize(params.get('cols'), params.get('rows'))
                elif method == 'stop':
                    result = native.stop(params.get('force') is True) if native.status().get('state') in ('running', 'stopping') else manager.stop(params.get('force') is True)
                else: raise ValueError('unknown IPC method')
                emit({'id': request.get('id'), 'result': result})
            except Exception as error:
                emit({'id': request.get('id') if isinstance(request, dict) else None, 'error': str(error)})
    except KeyboardInterrupt: pass
    finally:
        model_runtime.close()
        native.close()
        manager.close()


if __name__ == '__main__': main()
