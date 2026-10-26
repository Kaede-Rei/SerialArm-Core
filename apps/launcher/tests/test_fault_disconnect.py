from pathlib import Path
import json
import os
import subprocess
import shutil
import sys
import tempfile
import time
import threading
import unittest
from unittest import mock
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/launcher/backend'))
from machine import NativeSession

class FaultDisconnect(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'),'Node is required for renderer checks')
    def test_fault_exit_controls(self):
        subprocess.run(['node',str(ROOT/'apps/launcher/scripts/test-disconnect.cjs')],
                       capture_output=True,text=True,check=True,timeout=10)

    @unittest.skipUnless(shutil.which('node'),'Node is required for renderer checks')
    def test_teaching_recovery_controls(self):
        subprocess.run(['node',str(ROOT/'apps/launcher/scripts/test-teaching-recovery.cjs')],
                       capture_output=True,text=True,check=True,timeout=10)

    def test_rejected_shutdown_preserves_session_and_force_exit_releases_lease(self):
        with tempfile.TemporaryDirectory() as temporary:
            events=[];native=NativeSession(events.append,Path(temporary)/'locks')
            program='''import json,sys
for line in sys.stdin:
 m=json.loads(line)
 print(json.dumps({'id':m['id'],'error':'deactivate: DISABLE_FAILED'}),flush=True)
'''
            try:
                native.start([sys.executable,'-u','-c',program],dict(os.environ),[str(Path(temporary)/'fake-tty')])
                with self.assertRaisesRegex(ValueError,'DISABLE_FAILED'):native.stop()
                status=native.status()
                self.assertEqual(status['state'],'running')
                self.assertTrue(status['disconnect_blocked'])
                self.assertFalse(status.get('recovery_required'))
                self.assertTrue(native.leases)
                native.stop(force=True)
                native.proc.wait(timeout=3);native.reader.join(3)
                self.assertEqual(native.status()['state'],'exited')
                self.assertTrue(native.status()['force_stop_unverified'])
                self.assertFalse(native.leases)
            finally:
                if native.proc and native.proc.poll() is None:
                    native.stop(force=True);native.proc.wait(timeout=3)
                native.close()
    def test_telemetry_carries_its_native_session_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            ready=threading.Event();events=[]
            def emit(message):
                events.append(message)
                if message.get('event')=='telemetry':ready.set()
            native=NativeSession(emit,Path(temporary)/'locks')
            program="""import json,sys
print(json.dumps({'event':'telemetry','data':{'robot_state':'INACTIVE'}}),flush=True)
for line in sys.stdin:
 m=json.loads(line)
 print(json.dumps({'id':m['id'],'result':True}),flush=True)
 if m['method']=='shutdown':break
"""
            try:
                current=native.start([sys.executable,'-u','-c',program],dict(os.environ),[])
                self.assertTrue(ready.wait(3))
                telemetry=next(e for e in events if e.get('event')=='telemetry')
                self.assertEqual(telemetry['session_id'],current['session_id'])
                self.assertEqual(telemetry['session_pid'],current['pid'])
                native.stop()
            finally:
                if native.proc and native.proc.poll() is None:
                    native.stop(force=True);native.proc.wait(timeout=3)
                native.close()

    def test_disconnect_failure_does_not_change_known_robot_state(self):
        events=[];native=NativeSession(events.append,'/unused')
        native.state={'state':'running','mode':'workbench'}
        native.note_disconnect_failure(ValueError('DISABLE_FAILED'))
        self.assertTrue(events[-1]['disconnect_blocked'])
        self.assertNotIn('robot_state',events[-1])
        self.assertNotIn('hardware_disable_verified',events[-1])

if __name__=='__main__':unittest.main()
