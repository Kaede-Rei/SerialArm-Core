#!/usr/bin/env python3
"""Verify the installed native workspace through production Launcher IPC.

Run after sourcing .install/native-setup.bash, with .install/gui-venv/bin/python.
The temporary Profile disables writes; no activation or hardware command is sent.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

import yaml

from check_adaptive_gain_flow import ROOT, prepare_config


class Bridge:
    def __init__(self, directory: Path):
        self.log = tempfile.TemporaryFile(mode='w+t')
        env = dict(os.environ, XDG_CACHE_HOME=str(directory / 'cache'),
                   XDG_CONFIG_HOME=str(directory / 'preferences'))
        self.process = subprocess.Popen(
            [sys.executable, str(ROOT / 'apps/launcher/backend/bridge.py')],
            cwd=ROOT, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.log, text=True, bufsize=1)
        self.replies = queue.Queue()
        self.sequence = 0

        def read():
            for line in self.process.stdout:
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    self.replies.put({'error': 'Invalid bridge JSON: ' + line[:300]})
                    continue
                if 'id' in message:
                    self.replies.put(message)
            self.replies.put({'error': 'Bridge exited before replying'})

        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()

    def request(self, method, params=None):
        self.sequence += 1
        self.process.stdin.write(json.dumps(
            {'id': self.sequence, 'method': method, 'params': params or {}}) + '\n')
        self.process.stdin.flush()
        reply = self.replies.get(timeout=30)
        if 'error' in reply:
            raise RuntimeError(f'{method}: {reply["error"]}')
        assert reply['id'] == self.sequence, reply
        return reply['result']

    def close(self):
        if self.process.poll() is None:
            try:
                self.request('workbench_stop', {'force': True})
                self.request('close')
            finally:
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        self.reader.join(timeout=1)


def main():
    manifest = json.loads((ROOT / '.install/manifest.json').read_text())
    installed_terminal = (Path(manifest['install_prefix']) / 'bin/serial_arm_terminal').resolve()
    with tempfile.TemporaryDirectory(prefix='serialarm-workbench-startup-') as temp:
        directory = Path(temp)
        core_path, names = prepare_config(directory)
        core = json.loads(core_path.read_text())
        assert core['control']['runtime']['write_enabled'] is False
        catalogue = yaml.safe_load((ROOT / 'src/robot_supports/profiles/config/robot_profiles.yaml').read_text())
        entry = copy.deepcopy(catalogue['profiles']['dm_arm_white'])
        entry['core']['config'] = str(core_path)
        profile_path = directory / 'robot_profiles.yaml'
        profile_path.write_text(json.dumps({'profiles': {'offline_startup': entry}}))
        config = {'profile': 'offline_startup', 'profile_file': str(profile_path)}
        bridge = Bridge(directory)
        try:
            info = bridge.request('inspect', config)
            assert info['write_enabled'] is False and info['available']['workbench'], info
            session = bridge.request('workbench_start', {
                'config': config, 'fingerprint': info['fingerprint'], 'confirmed': False})
            assert session['state'] == 'running' and session['devices'] == [], session
            assert Path(session['command'][0]).resolve() == installed_terminal, session
            assert '--hardware-plugin' not in session['command'], session
            native = bridge.request('workbench_request', {'method': 'status'})
            assert native['write_enabled'] is False and native['robot_state'] == 'INACTIVE', native
            assert native['joint_names'] == names, native
            bridge.request('workbench_stop')
            deadline = time.monotonic() + 3
            while True:
                stopped = bridge.request('status')['workbench']
                if stopped['state'] == 'exited':
                    break
                assert time.monotonic() < deadline, stopped
                time.sleep(.02)
            assert stopped['exit_code'] == 0 and not stopped.get('exit_error'), stopped
            assert stopped['session_id'] == session['session_id'], stopped
            print(json.dumps({'status': 'passed', 'terminal': str(installed_terminal),
                              'startup': session['state'], 'robot_state': native['robot_state'],
                              'shutdown': stopped['state'], 'exit_code': stopped['exit_code'],
                              'write_enabled': False, 'scope': 'production bridge; installed native; no hardware'}))
        except BaseException:
            bridge.log.seek(0)
            print(bridge.log.read()[-8000:], file=sys.stderr)
            raise
        finally:
            bridge.close()


if __name__ == '__main__':
    main()
