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
from itertools import product
from unittest import mock
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/launcher/backend'))
from machine import NativeSession
import bridge

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

    def test_cancelled_park_is_not_sent_after_a_stop_request(self):
        with tempfile.TemporaryDirectory() as temporary:
            native=NativeSession(lambda message:None,Path(temporary)/'locks')
            program="""import json,sys
for line in sys.stdin:
 m=json.loads(line)
 print(json.dumps({'id':m['id'],'result':m['method']}),flush=True)
 if m['method']=='shutdown':break
"""
            try:
                native.start([sys.executable,'-u','-c',program],dict(os.environ),[])
                cancelled=threading.Event();cancelled.set()
                with self.assertRaisesRegex(ValueError,'park interrupted by operator'):
                    native.request('park',cancel_event=cancelled)
                self.assertEqual(native.request('hold'),'hold')
                self.assertEqual(native.seq,1)
                native.stop()
            finally:native.close()

    def test_bridge_dispatches_stop_commands_while_parking_is_pending(self):
        for action,job in product(('hold','deactivate','force'),('park','workbench_stop','stop')):
            with self.subTest(action=action,job=job):
                started=threading.Event();interrupted=threading.Event();events=[];calls=[]
                class Inspector:
                    def __init__(self,*args):pass
                    def inspect(self,*args):return dict(available={'workbench':True},fingerprint='f',write_enabled=False,resources={'core':'/unused','hardware':'/unused'},hardware_plugin='mock',devices=[])
                    def machine_terminal(self):return '/unused'
                class Manager:
                    def __init__(self,*args):pass
                    def status(self):return {'state':'idle'}
                    def close(self):pass
                class Native:
                    def __init__(self,*args):self.state='idle'
                    def status(self):return {'state':self.state,'mode':'workbench'}
                    def start(self,*args):self.state='running';return self.status()
                    def request(self,method,params=None,timeout=8,cancel_event=None):
                        calls.append(method)
                        if method in ('park','shutdown'):
                            if cancel_event is not None and cancel_event.is_set():raise ValueError('park interrupted by operator')
                            started.set()
                            if not interrupted.wait(2):calls.append('park_timeout');raise AssertionError('stop command was blocked behind park')
                            raise ValueError('park interrupted by operator')
                        if method in ('hold','deactivate'):interrupted.set()
                        if method=='status':return {'robot_state':'ACTIVE'}
                        if method=='candidate_overrides':return {}
                        return True
                    def stop(self,force=False,cancel_event=None):
                        if not force:return self.request('shutdown',cancel_event=cancel_event)
                        self.assert_force=force
                        calls.append('force');interrupted.set();self.state='exited';return self.status()
                    def close(self):pass
                class Model:
                    def close(self):pass
                class Library:
                    def __init__(self,*args):pass
                    def _readiness(self,*args):return {'checks':{'profile_setup':'verified'},'control_available':True,'modes':{'RIGID_TRACKING':'configured','COMPLIANT_TRACKING':'configured'}}
                def lines():
                    yield json.dumps({'id':1,'method':'workbench_start','params':{'config':{'profile':'mock'},'fingerprint':'f'}})+'\n'
                    command={'method':'workbench_request','params':{'method':'park'}} if job=='park' else {'method':job,'params':{}}
                    yield json.dumps({'id':2,**command})+'\n'
                    self.assertTrue(started.wait(2))
                    command={'method':'workbench_stop','params':{'force':True}} if action=='force' else {'method':'workbench_request','params':{'method':action}}
                    yield json.dumps({'id':3,**command})+'\n'
                with mock.patch.multiple(bridge,Inspector=Inspector,Supervisor=Manager,NativeSession=Native,ModelRuntime=Model,ProfileLibrary=Library,TcpCalibration=lambda *args:None,profile_home=lambda *args:Path('/unused'),verify_machine_admission=lambda *args:None,emit=events.append),mock.patch.object(sys,'stdin',lines()),mock.patch.object(sys,'stderr'):
                    bridge.main()
                self.assertTrue(interrupted.is_set())
                self.assertNotIn('park_timeout',calls)
                self.assertIn('force' if action=='force' else action,calls)
                self.assertTrue(any(message.get('id')==3 and message.get('result') for message in events),events)

if __name__=='__main__':unittest.main()
