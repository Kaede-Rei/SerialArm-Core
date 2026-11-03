"""User workflows: editable drafts, historical selection, and independent TCP adoption."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import selectors
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from profiles import Inspector
from profile_library import ProfileLibrary
from workflow_registry import candidates, load, scan_page
from persistence import select_gravity_candidate, preview_gravity_correction, save_gravity_correction, sha256_file
from tcp_calibration import TcpCalibration
from model_calibration_source import recorded_file_fingerprint
from interaction_response import analyze_interaction_response


class WorkflowUsability(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / 'source'; self.source.mkdir()
        self.urdf = self.source / 'model.urdf'
        self.urdf.write_text('<robot name="test"><link name="base"/><link name="tip"><inertial><mass value="1"/><origin xyz="0.1 0 0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link><joint name="axis" type="revolute"><parent link="base"/><child link="tip"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint></robot>')
        self.library = ProfileLibrary(ROOT, Inspector(ROOT))
        self.library.cache = self.root / 'cache'
        self.library.library_file = self.root / 'library.yaml'
        self.options = dict(source=str(self.urdf), profile='test', destination=str(self.root / 'created'),
                            joint_names=['axis'], base_frame='base', tool_frame='tip', hardware_plugin='')
        self.created = self.library.create_profile(self.options)
        self.config = self.created['config']

    def test_nested_destination_rejected_before_any_directory_creation(self):
        options = {**self.options, 'destination':str(self.source / 'nested')}
        with self.assertRaisesRegex(ValueError, 'destination.*源 Description'):
            self.library.create_profile(options)
        self.assertFalse((self.source / 'nested').exists())
        self.assertTrue(self.urdf.is_file())

    def test_null_required_scalar_repaired_through_real_editor_save(self):
        editor = self.library.profile_editor_load(self.config)
        path = Path(editor['core_path'])
        core = yaml.safe_load(path.read_text())
        core['control']['runtime']['ctrl_frequency_hz'] = None
        path.write_text(yaml.safe_dump(core))
        editor = self.library.profile_editor_load(self.config)
        editor['core']['control']['runtime']['ctrl_frequency_hz'] = 100.
        saved = self.library.profile_editor_save(self.config, editor)
        self.assertEqual(saved['core']['control']['runtime']['ctrl_frequency_hz'], 100.)
        original = path.read_bytes()
        saved['core']['control']['runtime']['ctrl_frequency_hz'] = -2
        with self.assertRaises(ValueError): self.library.profile_editor_save(self.config, saved)
        self.assertEqual(path.read_bytes(), original)

    def test_readiness_distinguishes_safe_defaults_without_enabling_write(self):
        state = self.library._readiness(self.config)['connection_readiness']
        conditions = {item['id']:item for item in state['basic_calibration_prerequisites']}
        self.assertFalse(state['write_enabled'])
        self.assertFalse(conditions['write_enabled']['ready'])
        self.assertFalse(conditions['drag_damping']['ready'])
        self.assertTrue(conditions['gravity']['ready'])
        self.assertFalse(state['basic_calibration_ready'])
        self.assertEqual(conditions['write_enabled']['action']['field'], 'core.control.runtime.write_enabled')
        self.assertFalse(self.library.profile_editor_load(self.config)['core']['control']['runtime']['write_enabled'])

    def test_tcp_preview_export_and_adoption_preserve_source_and_open_independently(self):
        (self.source / 'mesh.stl').write_text('solid test\nendsolid test\n')
        source_model = Path(self.library.profile_editor_load(self.config)['urdf_path'])
        (source_model.parent / 'mesh.stl').write_bytes((self.source / 'mesh.stl').read_bytes())
        source_model.write_text(source_model.read_text().replace('<link name="base"/>', '<link name="base"><visual><geometry><mesh filename="mesh.stl"/></geometry></visual></link>'))
        geometry = TcpCalibration(self.library)
        geometry.begin(self.config, 'tip', 'tcp')
        geometry.session['candidate'] = {'tcp_xyz_m':[.01,.02,.03]}
        geometry.session['report'] = {'passed':True, 'rms_mm':.1, 'max_mm':.2}
        preview = geometry.preview_export(self.config)
        inputs = [Path(preview[key]) for key in ('source_urdf','source_core','source_profile')]
        before = {path:path.read_bytes() for path in inputs}
        exported = geometry.export(self.config)
        model = ET.parse(exported['candidate_urdf']).getroot()
        mesh = model.find('.//mesh').get('filename')
        self.assertTrue((Path(exported['candidate_urdf']).parent / mesh).is_file())
        fixed = model.find("joint[@name='test_tcp_fixed']")
        self.assertEqual(fixed.find('parent').get('link'), 'tip')
        self.assertEqual(fixed.find('origin').get('xyz'), '0.01 0.02 0.03')
        info = self.library.inspector.inspect(exported['config'])
        self.assertTrue(all(check['ok'] for check in info['checks'] if check['name'] in ('core','description','hardware')))
        self.assertEqual(yaml.safe_load(Path(exported['candidate_core']).read_text())['model']['tool_frame'], 'test_tcp')
        with self.assertRaisesRegex(ValueError,'断开'): geometry.apply(self.config,native_running=True,expected_hashes=preview['source_hashes'])
        with self.assertRaisesRegex(ValueError,'重新预览'): geometry.apply(self.config,native_running=False,expected_hashes={})
        adopted = geometry.apply(self.config,native_running=False,expected_hashes=preview['source_hashes'])
        self.assertTrue(adopted['adopted'])
        self.assertTrue(adopted['library_entry']['id'].startswith('user:'))
        self.assertEqual({path:path.read_bytes() for path in inputs},before)

    def history_task(self, name, *, profile='test', kind='gain', time=1):
        folder = self.root / '.serialarm/model-calibration' / name
        folder.mkdir(parents=True)
        (folder/'metadata.json').write_text(json.dumps({'task_id':name,'joint_names':['axis'],'profile':profile}))
        if kind=='gain': (folder/'gain-candidate.json').write_text('{}')
        else: (folder/'accepted_static.csv').write_text('header\n')
        os.utime(folder,(time,time))
        return folder

    def test_history_filters_profile_and_tool_before_pagination(self):
        expected=[]
        for i in range(8): expected.append(self.history_task('model-calibration-target-'+str(i),kind='gravity',time=i+1))
        for i in range(85): self.history_task('model-calibration-new-'+str(i),profile='other' if i%2 else 'test',time=100+i)
        page=scan_page(self.root,'gravity',3,offset=3,profile_name='test')
        self.assertEqual(page['total'],8)
        self.assertTrue(page['has_more'])
        self.assertEqual([item['directory'] for item in page['records']],[str(path) for path in expected[::-1][3:6]])
        self.assertEqual(len(scan_page(self.root,'gravity',3,offset=6,profile_name='test')['records']),2)
        self.assertEqual(scan_page(self.root,'gravity',3,query='target-2',profile_name='test')['total'],1)

    def test_multiple_rounds_load_selected_gravity_and_idim_report(self):
        folder=self.history_task('model-calibration-rounds',kind='gravity')
        for key in ('old','new'):
            target=folder/'offline_candidates'/key/'recomputed-candidate.json';target.parent.mkdir(parents=True)
            target.write_text(json.dumps({'task_id':folder.name,'candidate_run_id':key,'archived_report':str(target.relative_to(folder)),'first_moments':[{'value':[1 if key=='old' else 2]}]}))
        latest=folder/'offline_candidates/new/recomputed-candidate.json'
        (folder/'recomputed-candidate.json').write_bytes(latest.read_bytes())
        listed=candidates(folder,'gravity')['candidates']
        self.assertEqual(len(listed),2)
        old=folder/'offline_candidates/old/recomputed-candidate.json'
        selected=load(folder,'gravity',candidate_path=str(old))
        self.assertEqual(selected['recomputed_candidate']['candidate_run_id'],'old')
        self.assertEqual(selected['selected_candidate']['path'],str(old))
        with self.assertRaisesRegex(ValueError,'明确选择'): select_gravity_candidate(folder)
        self.assertEqual(select_gravity_candidate(folder,'offline',run_id='old')[1],old)
        with self.assertRaises(ValueError): select_gravity_candidate(folder,'offline',candidate_path=self.urdf)
        report=folder/'idim_candidates/one/idim-report.json';report.parent.mkdir(parents=True)
        report.write_text(json.dumps({'task_id':folder.name,'run_id':'one','schema':'serial_arm_idim_candidate_v1'}))
        value=load(folder,'dynamics',report=str(report))
        self.assertEqual(value['idim_candidate']['run_id'],'one')
        self.assertEqual(value['dynamic_candidates'],[])

    def test_offline_save_keeps_selected_mass_and_first_moments(self):
        # Reuse the semantic source fixture rather than mocking persistence
        import test_gravity_save_target
        fixture=test_gravity_save_target.GravitySaveTargetTests()
        fixture.setUp();self.addCleanup(fixture.doCleanups)
        folder=fixture.task
        online=json.loads((folder/'result.json').read_text())['gravity_result']
        for key,mass in [('old',2.),('new',3.)]:
            report={**online,'task_id':fixture.metadata['task_id'],'core_fingerprint':fixture.metadata['core_fingerprint'],
                'urdf_fingerprint':fixture.metadata['urdf_fingerprint'],'candidate_run_id':key,'candidate_masses':{'tip':mass},
                'solver':'static_centered_svd_ridge_pose_cv','identification_mode':'global','fitted_links':['tip'],'constraints_ok':True,
                'frames_sha256':hashlib.sha256((folder/'frames.csv').read_bytes()).hexdigest(),
                'source_sha256':hashlib.sha256(fixture.model.read_bytes()).hexdigest(),'accepted_static_sha256':None}
            target=folder/'offline_candidates'/key/'recomputed-candidate.json';target.parent.mkdir(parents=True)
            target.write_text(json.dumps(report))
        (folder/'recomputed-candidate.json').write_bytes((folder/'offline_candidates/new/recomputed-candidate.json').read_bytes())
        old=folder/'offline_candidates/old/recomputed-candidate.json'
        with self.assertRaisesRegex(ValueError,'明确选择'): preview_gravity_correction(fixture.core,folder)
        preview=preview_gravity_correction(fixture.core,folder,candidate_source='offline',candidate_path=old)
        saved=save_gravity_correction(fixture.core,preview['sha256'],folder,candidate_source='offline',candidate_path=old)
        payload=yaml.safe_load(Path(saved['gravity_correction_path']).read_text())
        self.assertEqual(payload['candidate_masses'],{'tip':2.})
        self.assertEqual(payload['first_moments'],{'tip':[.12,0.,0.]})
        self.assertEqual(saved['candidate_run_id'],'old')
        import gravity_legacy
        with patch.object(gravity_legacy,'_prepare_inputs',return_value=(folder,fixture.metadata,{},fixture.model,{})):
            exported=gravity_legacy.export_recomputed(folder,str(self.root/'exported-old'),candidate_path=old)
        model=ET.parse(exported['candidate_urdf']).getroot()
        self.assertEqual(float(model.find('.//mass').get('value')),2.)
        self.assertEqual(model.find('.//inertial/origin').get('xyz'),'0.06 0 0')
        self.assertEqual(exported['candidate_run_id'],'old')

    def test_changed_closed_loop_conditions_do_not_produce_joint_model(self):
        samples=[{'time_s':i*.01,'accepted':[0.1 if i%20 else 0.], 'pos':[0.], 'kp':[20.], 'kd':[2.], 'nominal':[0.]} for i in range(150)]
        for key,value in [('kp',[30.]),('kd',[3.]),('gain_purpose','SERVO_TRACKING'),('reference_behavior','MODEL_ADMITTANCE')]:
            changed=deepcopy(samples)
            changed[-1][key]=value
            result=analyze_interaction_response({'joint_names':['axis'],'samples':changed})
            self.assertFalse(result['conditions_consistent'])
            self.assertFalse(result['joints'][0]['identified'])
            self.assertIn(key,result['joints'][0]['reason'])

    def test_connected_export_does_not_block_status_or_control_loop(self):
        import bridge
        messages=[]
        release=threading.Event()
        finished=threading.Event()
        def emit(message):
            messages.append(message)
            if message.get('id') == 1: finished.set()
        def export(*args):
            self.assertTrue(release.wait(3))
            return {'path':'fixture.zip'}
        class Stub:
            def __init__(self,*args): pass
            def status(self): return {'state':'running'}
            def close(self): pass
        def lines():
            yield json.dumps({'id':1,'method':'calibration_tool_export_record','params':{}})+'\n'
            yield json.dumps({'id':2,'method':'status','params':{}})+'\n'
            self.assertTrue(any(message.get('id')==2 for message in messages))
            self.assertFalse(finished.is_set())
            release.set()
            self.assertTrue(finished.wait(3))
        with patch.multiple(bridge,Inspector=Stub,Supervisor=Stub,NativeSession=Stub,ModelRuntime=Stub,ProfileLibrary=Stub,TcpCalibration=Stub,emit=emit,export_offline_artifact=export), patch.object(sys,'stdin',lines()):
            bridge.main()
        self.assertEqual(next(message for message in messages if message.get('id')==1)['result']['path'],'fixture.zip')

    def test_tcp_export_real_ipc_keeps_hold_and_disable_responsive_and_uses_snapshot(self):
        editor = self.library.profile_editor_load(self.config)
        model = Path(editor['urdf_path'])
        model.write_text(model.read_text().replace('<link name="base"/>', '<link name="base"><visual><geometry><mesh filename="mesh.stl"/></geometry></visual></link>'))
        (model.parent / 'mesh.stl').write_text('solid fixture\nendsolid fixture\n')
        geometry = TcpCalibration(self.library)
        geometry.begin(self.config, 'tip', 'tcp')
        geometry.session['candidate'] = {'tcp_xyz_m':[.01,.02,.03]}
        geometry.session['report'] = {'passed':True}
        geometry._save()
        started, release = self.root / 'copy-started', self.root / 'copy-release'
        script = self.root / 'ipc_fixture.py'
        script.write_text(f'''
import sys,time
from pathlib import Path
sys.path.insert(0,{str(ROOT / 'apps/launcher/backend')!r})
import bridge,tcp_calibration
original_inspect = bridge.Inspector.inspect
def inspect(self,config):
    result=original_inspect(self,config)
    result['available']['workbench']=True
    result['write_enabled']=False
    return result
bridge.Inspector.inspect=inspect
bridge.Inspector.machine_terminal=lambda self:'/never-launch-native'
bridge.ProfileLibrary._readiness=lambda *args:{{'checks':{{'profile_setup':'verified'}}}}
class Idle:
    def __init__(self,*args):self.state='idle'
    def status(self):return {{'state':self.state,'mode':'workbench','session_id':'fixture'}}
    def close(self):pass
class Native(Idle):
    def start(self,*args):self.state='running';return self.status()
    def request(self,method,params,**kwargs):return {{'method':method,'robot_state':'INACTIVE' if method=='deactivate' else 'ACTIVE'}}
bridge.Supervisor=Idle
bridge.NativeSession=Native
copy=tcp_calibration.shutil.copy2
def slow_copy(source,target,*args,**kwargs):
    if Path(source).name=='mesh.stl':
        Path({str(started)!r}).touch()
        until=time.monotonic()+10
        while not Path({str(release)!r}).exists():
            if time.monotonic()>until:raise RuntimeError('fixture copy release timed out')
            time.sleep(.01)
    return copy(source,target,*args,**kwargs)
tcp_calibration.shutil.copy2=slow_copy
bridge.main()
''')
        error_log = self.root / 'ipc-stderr.log'
        with error_log.open('wb') as errors:
            process = subprocess.Popen([sys.executable,str(script)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=errors,bufsize=0)
            self.addCleanup(lambda: process.kill() if process.poll() is None else None)
            selector = selectors.DefaultSelector()
            selector.register(process.stdout,selectors.EVENT_READ)
            self.addCleanup(selector.close)
            def read():
                self.assertTrue(selector.select(3),error_log.read_text())
                line=process.stdout.readline()
                self.assertTrue(line,error_log.read_text())
                return json.loads(line)
            def send(key,method,params):
                process.stdin.write((json.dumps({'id':key,'method':method,'params':params})+'\n').encode())
                process.stdin.flush()
            self.assertEqual(read()['event'],'backend_ready')
            info=self.library.inspector.inspect(self.config)
            send(1,'workbench_start',{'config':self.config,'fingerprint':info['fingerprint']})
            self.assertEqual(read()['id'],1)
            send(2,'geometry_fixed_state',{'config':self.config})
            self.assertTrue(read()['result']['active'])
            send(3,'geometry_fixed_export_tcp',{'config':self.config,'destination':str(self.root)})
            until=time.monotonic()+3
            while not started.exists() and time.monotonic()<until:time.sleep(.01)
            self.assertTrue(started.exists(),error_log.read_text())
            for key,method,params in [(4,'status',{}),(5,'workbench_request',{'method':'hold'}),(6,'workbench_request',{'method':'deactivate'})]:
                send(key,method,params)
                reply=read()
                self.assertEqual(reply.get('id'),key,reply)
                self.assertIn('result',reply)
                if key>4:self.assertEqual(reply['result']['method'],params['method'])
            send(7,'geometry_fixed_export_tcp',{'config':self.config})
            self.assertIn('error',read())
            # Mutate the live experiment while its detached export is copying
            send(8,'geometry_fixed_discard',{'config':self.config})
            self.assertEqual(read()['result'],{'active':False})
            # Another adoption may replace the legacy latest-report pointer;
            # the paused export must keep its own candidate A report
            latest_report = Path(geometry.session['directory']) / 'tcp-candidate.yaml'
            latest_report.write_text(yaml.safe_dump({'tcp_xyz_m':[.9,.8,.7], 'candidate':'B'}))
            release.touch()
            exported=read()
            self.assertEqual(exported.get('id'),3,exported)
            self.assertTrue(Path(exported['result']['candidate_urdf']).is_file())
            independent_report = Path(exported['result']['path'])
            self.assertNotEqual(independent_report,latest_report)
            self.assertEqual(yaml.safe_load(independent_report.read_text())['tcp_xyz_m'],[.01,.02,.03])
            self.assertEqual(yaml.safe_load(latest_report.read_text())['candidate'],'B')
            process.stdin.close()
            self.assertEqual(process.wait(timeout=3),0,error_log.read_text())
            process.stdout.close()


if __name__=='__main__': unittest.main()
