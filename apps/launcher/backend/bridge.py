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
from persistence import (export_telemetry, export_candidate_urdf, load_model_calibration_summary, list_model_calibration_records, preview_gravity_correction, restore_gravity_correction, save_gravity_correction, update_candidate_urdf_verification, preview as preview_config, save as save_config)
from profile_library import ProfileLibrary
from model_calibration_alignment import compare_calibration_urdfs

ROOT = Path(__file__).resolve().parents[3]
output_lock = threading.Lock()


def run_full_inertial_isolated(directory, destination=None, *, timeout=120):
    """Never load optional native Pinocchio extensions in the live IPC process"""
    argv = [sys.executable, str(Path(__file__).with_name('full_inertial_worker.py')),
            str(Path(directory).expanduser().resolve()), str(destination or '')]
    env = dict(os.environ)
    # Avoid a user-site NumPy 2 overriding the ROS Humble NumPy 1 ABI
    env['PYTHONNOUSERSITE'] = '1'
    try:
        completed = subprocess.run(argv, capture_output=True, text=True,
                                   timeout=timeout, env=env)
    except subprocess.TimeoutExpired as error:
        raise ValueError('完整惯量候选计算超时，已终止独立计算进程，Launcher 可继续使用') from error
    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip()
        numpy_abi = ('_ARRAY_API not found' in details or
                     'A module that was compiled using NumPy 1.x' in details or
                     'numpy.core.multiarray failed to import' in details)
        if numpy_abi:
            raise ValueError('Pinocchio 与 NumPy ABI 不兼容，当前候选计算已隔离终止，不影响机器人控制界面，请重新运行 ./install.sh 更新 GUI Python 环境中的 numpy<2')
        if completed.returncode < 0:
            raise ValueError(f'完整惯量计算子进程被信号 {-completed.returncode} 终止，Backend 未受影响，请检查 Python 原生依赖')
        raise ValueError('完整惯量候选导出失败: ' + (details[-2000:] or f'退出码 {completed.returncode}'))
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise ValueError('完整惯量候选进程没有返回有效 JSON') from error


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
                elif method == 'workbench_request':
                    native_method = params.get('method')
                    result = native.request(native_method, params.get('params', {}), timeout=120 if native_method == 'park' else (45 if native_method in ('model_calibration_teach_stop','model_calibration_import_trajectory') else 8))
                elif method == 'workbench_stop':
                    result = native.stop(params.get('force') is True)
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
                elif method == 'model_calibration_export_urdf':
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = export_candidate_urdf(directory, params.get('destination') or None)
                    # Independent verification is informative for manual-review
                    # candidates and must never abort the IPC loop
                    try:
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
                elif method == 'model_calibration_export_full_inertial':
                    # Offline file generation only; never apply to the running robot.
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = run_full_inertial_isolated(directory, params.get('destination') or None)
                elif method == 'model_calibration_recompute':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    calibrator = inspector.model_calibrator()
                    if not calibrator: raise ValueError('serial_arm_model_calibrator is not installed')
                    directory = str(Path(params.get('directory', '')).expanduser().resolve())
                    if not Path(directory).is_dir(): raise ValueError('model calibration task directory is missing')
                    output = str(Path(directory) / 'recomputed-candidate.json')
                    completed = subprocess.run([calibrator, '--config', info['resources']['core'], '--dataset', directory, '--output', output], text=True, capture_output=True, timeout=60)
                    if completed.returncode not in (0, 3): raise ValueError((completed.stderr or completed.stdout or 'offline recalculation failed').strip())
                    try:
                        candidate = json.loads(Path(output).read_text())
                    except (OSError, json.JSONDecodeError) as error:
                        raise ValueError(f'offline recalculation produced invalid candidate JSON: {error}')
                    result = {'path': output, 'static_pass': bool(candidate.get('static_pass')), 'candidate': candidate, 'output': completed.stdout.strip()}
                elif method == 'start':
                    if native.status().get('state') in ('running', 'stopping'): raise ValueError('another Launcher session is already running')
                    mode = params.get('mode')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    if not info['available'].get(mode): raise ValueError('selected run mode is unavailable; inspect installation and resources')
                    if params.get('fingerprint') != info['fingerprint']: raise ValueError('configuration changed; inspect and confirm again')
                    if params.get('confirmed') is not True: raise ValueError('real hardware confirmation is required')
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
