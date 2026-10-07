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
from persistence import (export_telemetry, export_candidate_urdf, load_model_calibration_summary, preview_gravity_correction, restore_gravity_correction, save_gravity_correction, update_candidate_urdf_verification, preview as preview_config, save as save_config)
from profile_library import ProfileLibrary

ROOT = Path(__file__).resolve().parents[3]
output_lock = threading.Lock()


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
                    result = native.request(native_method, params.get('params', {}), timeout=120 if native_method == 'park' else 8)
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
                    calibrator = inspector.model_calibrator()
                    if not calibrator: raise ValueError('serial_arm_model_calibrator is not installed; candidate verification cannot run')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    verification = subprocess.run([calibrator, '--config', info['resources']['core'], '--dataset', str(Path(directory).resolve()), '--verify-urdf', result['candidate_urdf']], text=True, capture_output=True, timeout=60)
                    if verification.returncode != 0:
                        raise ValueError((verification.stderr or verification.stdout or 'candidate URDF verification failed').strip())
                    try:
                        verified = json.loads(verification.stdout.strip().splitlines()[-1])
                    except (json.JSONDecodeError, IndexError):
                        raise ValueError('candidate URDF verification returned invalid JSON')
                    result['verification'] = update_candidate_urdf_verification(Path(result['candidate_urdf']).parent, verified)
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
                    result = {'path': output, 'static_pass': completed.returncode == 0, 'candidate': candidate, 'output': completed.stdout.strip()}
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
