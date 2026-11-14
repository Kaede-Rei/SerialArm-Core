#!/usr/bin/env python3
"""Check native startup readiness and process-exit diagnostics without hardware."""
import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from machine import NativeSession


RESPONDING_NATIVE = '''
import json, sys
for line in sys.stdin:
    request = json.loads(line)
    result = {"robot_state": "INACTIVE"} if request["method"] == "status" else True
    print(json.dumps({"id": request["id"], "result": result}), flush=True)
    if request["method"] == "shutdown":
        break
'''


class NativeSessionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='serialarm-native-session-')
        self.events = []
        self.native = NativeSession(self.events.append, self.directory.name)

    def tearDown(self):
        self.native.close()
        self.directory.cleanup()

    def start(self, script):
        return self.native.start([sys.executable, '-u', '-c', script], os.environ.copy(), [])

    def test_config_failure_reports_native_stderr_and_exit_code(self):
        failure = "import sys; print(\"配置加载失败: capability.admittance.observer: missing field 'mode'\", file=sys.stderr); sys.exit(1)"
        with self.assertRaisesRegex(ValueError, "native process exited \\(code 1\\).*missing field 'mode'"):
            self.start(failure)
        self.assertEqual(self.native.status()['state'], 'exited')
        self.assertEqual(self.native.status()['exit_code'], 1)
        self.assertFalse(any(event.get('event') == 'session' and event.get('state') == 'running'
                             for event in self.events))

    def test_start_waits_for_status_before_reporting_running(self):
        self.start(RESPONDING_NATIVE)
        self.assertEqual(self.native.seq, 1)
        self.assertEqual(self.native.status()['state'], 'running')
        self.assertEqual(self.native.request('status'), {'robot_state': 'INACTIVE'})

    def test_retry_after_failed_start_discards_old_diagnostics(self):
        with self.assertRaises(ValueError):
            self.start("import sys; print('first startup failed', file=sys.stderr); sys.exit(2)")
        self.start(RESPONDING_NATIVE)
        self.assertEqual(list(self.native.stderr_tail), [])
        self.assertNotIn('exit_error', self.native.status())

    def test_runtime_exit_reports_the_failure_to_pending_request(self):
        native = '''
import json, sys
for line in sys.stdin:
    request = json.loads(line)
    if request["method"] != "status":
        print("native runtime failed", file=sys.stderr)
        sys.exit(3)
    print(json.dumps({"id": request["id"], "result": {"robot_state": "INACTIVE"}}), flush=True)
'''
        self.start(native)
        with self.assertRaisesRegex(ValueError, r'native process exited \(code 3\): native runtime failed'):
            self.native.request('trigger_failure')
        self.assertEqual(self.native.status()['state'], 'exited')

    def test_startup_timeout_releases_process_and_device_for_retry(self):
        request = self.native.request
        self.native.request = lambda method: request(method, timeout=0.05)
        device = str(Path(self.directory.name) / 'mock-device')
        with self.assertRaises(TimeoutError):
            self.native.start([sys.executable, '-u', '-c', 'import sys; sys.stdin.read()'],
                              os.environ.copy(), [device])
        self.assertIsNotNone(self.native.proc.poll())
        self.assertEqual(self.native.status()['state'], 'exited')
        self.assertEqual(self.native.leases, [])
        self.native.request = request
        self.native.start([sys.executable, '-u', '-c', RESPONDING_NATIVE], os.environ.copy(), [device])
        self.assertEqual(self.native.status()['state'], 'running')


if __name__ == '__main__':
    unittest.main()
