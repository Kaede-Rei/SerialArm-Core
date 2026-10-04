#!/usr/bin/env python3
"""Line-delimited IPC; events and replies share stdout with serialized writes."""
import json
import os
from pathlib import Path
import signal
import sys
import threading

from profiles import Inspector, command_for, config_values
from runtime import Supervisor
from machine import NativeSession
from model_runtime import ModelRuntime

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
