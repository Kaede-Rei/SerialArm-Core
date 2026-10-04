import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[3]


class BridgeIntegrationTests(unittest.TestCase):
    def test_confirmation_fingerprint_and_interactive_process_lifecycle(self):
        with tempfile.TemporaryDirectory(prefix='bridge integration ') as temp:
            root = Path(temp)
            backend = root / 'apps/launcher/backend'
            shutil.copytree(ROOT / 'apps/launcher/backend', backend, ignore=shutil.ignore_patterns('__pycache__'))
            state = root / '.install'
            state.mkdir()
            prefix = root / 'install'
            resource = prefix / 'share/test_robot'
            resource.mkdir(parents=True)
            (resource / 'package.xml').write_text('<package><name>test_robot</name></package>')
            (resource / 'core.yaml').write_text('control:\n  runtime:\n    write_enabled: true\n')
            (resource / 'hardware.yaml').write_text('buses: {}\n')
            profile = prefix / 'share/serial_arm_robot_profiles/config/robot_profiles.yaml'
            profile.parent.mkdir(parents=True)
            profile.write_text('profiles:\n  test:\n    core: {package: test_robot, config: core.yaml}\n    hardware: {plugin: test_driver, config_package: test_robot, config: hardware.yaml}\n')
            setup = prefix / 'setup.bash'
            setup.write_text(':\n')
            (state / 'manifest.json').write_text(json.dumps({'status': 'verified', 'installed_components': ['core','terminal'], 'install_prefix': str(prefix), 'backend_setup': str(setup)}))
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            terminal = bin_dir / 'serial_arm_terminal'
            terminal.write_text('#!/bin/sh\nprintf \'PTY_READY %s\\n\' "$([ -t 0 ] && printf True || printf False)"\nIFS= read -r line\nprintf \'GOT %s\\n\' "$line"\nsleep 30\n')
            terminal.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir)+os.pathsep+os.environ['PATH'], XDG_CACHE_HOME=str(root/'cache'))
            env.pop('PYTHONPATH', None)
            env.pop('PYTHONHOME', None)
            proc = subprocess.Popen([sys.executable, '-u', str(backend/'bridge.py')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, text=True, bufsize=1)
            events = []
            def request(number, method, params=None):
                proc.stdin.write(json.dumps({'id':number,'method':method,'params':params or {}})+'\n');proc.stdin.flush()
                end = time.monotonic()+5
                while time.monotonic()<end:
                    if not select.select([proc.stdout],[],[],.1)[0]: continue
                    line=proc.stdout.readline()
                    if not line: break
                    message=json.loads(line)
                    if message.get('id')==number: return message
                    events.append(message)
                raise AssertionError(f'bridge request {method} timed out')
            try:
                config={'profile':'test'}
                info=request(1,'inspect',config)['result']
                result=request(2,'start',{'mode':'terminal','config':config,'confirmed':True,'fingerprint':'wrong'})
                self.assertIn('configuration changed',result['error'])
                result=request(3,'start',{'mode':'terminal','config':config,'confirmed':False,'fingerprint':info['fingerprint']})
                self.assertIn('confirmation',result['error'])
                result=request(4,'start',{'mode':'terminal','config':config,'confirmed':True,'fingerprint':info['fingerprint']})
                self.assertEqual(result['result']['state'],'running')
                request(5,'input',{'data':'hello\r'})
                time.sleep(.1)
                request(6,'stop')
                for n in range(7,20):
                    result=request(n,'status')['result']
                    if result['session']['state']=='exited': break
                    time.sleep(.03)
                self.assertEqual(result['session']['state'],'exited')
                self.assertIn('PTY_READY True',''.join(e.get('data','') for e in events))
                self.assertIn('GOT hello',''.join(e.get('data','') for e in events))
            finally:
                proc.stdin.close()
                try: proc.wait(timeout=4)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
                proc.stdout.close();proc.stderr.close()


if __name__ == '__main__': unittest.main()
