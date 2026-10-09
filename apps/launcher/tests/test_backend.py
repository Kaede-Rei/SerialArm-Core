import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from profiles import Inspector, command_for
from runtime import Supervisor
from machine import NativeSession
from model_runtime import ModelRuntime
from profile_library import ProfileLibrary


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.inspector = Inspector(ROOT)

    def test_builtin_profiles_are_discoverable_without_install(self):
        result = self.inspector.profiles({})
        self.assertEqual({p['name'] for p in result['profiles']}, {'dm_arm_gray', 'dm_arm_white'})
        self.assertTrue(result['profile_file'].endswith('robot_profiles.yaml'))

    def test_inspection_is_readonly_and_exposes_hardware_state(self):
        result = self.inspector.inspect({'profile': 'dm_arm_gray'})
        self.assertTrue(result['write_enabled'])
        self.assertEqual(result['devices'], ['/dev/ttyACM0'])
        self.assertTrue(Path(result['resources']['core']).is_file())
        self.assertFalse(result['available']['terminal'])

    def test_missing_profile_is_actionable(self):
        with self.assertRaises(ValueError):
            self.inspector.inspect({'profile': 'unknown'})

    def test_argument_building_keeps_shell_text_literal(self):
        config = {'profile': 'robot; touch /tmp/unsafe', 'profile_file': '/tmp/a b.yaml',
                  'serial_port': '/dev/ttyUSB0', 'resource_paths': '/tmp/one:/tmp/two'}
        args, env = command_for('terminal', config)
        self.assertIn('robot; touch /tmp/unsafe', args)
        self.assertEqual(env['SERIAL_ARM_RESOURCE_PATH'], '/tmp/one:/tmp/two')
        with self.assertRaisesRegex(ValueError, 'unknown run mode'):
            command_for('model', config)

    def test_each_ros_mode_targets_an_installed_launch_source(self):
        launch_dir = ROOT / 'src/serial_arm/bringup/ros2_control/launch'
        for mode, name in [('hardware', 'hardware.launch.py'), ('moveit', 'moveit.launch.py')]:
            args, _ = command_for(mode, {'profile': 'dm_arm_gray'})
            self.assertEqual(args[3], name)
            self.assertTrue((launch_dir / args[3]).is_file())

    def test_invalid_mode_and_baudrate_rejected(self):
        with self.assertRaises(ValueError): command_for('shell', {'profile': 'x'})
        with self.assertRaises(ValueError): command_for('terminal', {'profile': 'x', 'baudrate': 'bad'})



class ModelInspectorTests(unittest.TestCase):
    def test_model_is_readonly_and_does_not_require_hardware_resources(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            profiles = root / 'src/robot_supports/profiles/config/robot_profiles.yaml'
            profiles.parent.mkdir(parents=True)
            package = root / 'src/test_robot'
            package.mkdir(parents=True)
            (package / 'package.xml').write_text('<package><name>test_robot</name></package>')
            urdf = package / 'robot.urdf'
            urdf.write_text('''<robot name="fixture">
<link name="base"/>
<link name="link1"><inertial><origin xyz="0.1 0 0" rpy="0 0 0"/><mass value="2"/><inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/></inertial><visual><geometry><box size="0.2 0.1 0.1"/></geometry></visual></link>
<joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint>
</robot>''')
            core = package / 'core.yaml'
            core.write_text("""model:
  joint_names: [joint1]
  urdf_path: robot.urdf
  base_frame: base
  tool_frame: link1
  gravity: [0, 0, -9.81]
  gravity_scale: [1]
""")
            profiles.write_text("""profiles:
  test:
    core: {package: test_robot, config: core.yaml}
    hardware: {plugin: missing, config_package: test_robot, config: missing.yaml}
""")
            probe = root / 'serial_arm_model_probe'
            payload = {
                'ok': True,
                'config': {'urdf_path': str(urdf), 'base_frame': 'base', 'tool_frame': 'link1', 'gravity': [0, 0, -9.81], 'gravity_scale': [1], 'joint_names': ['joint1']},
                'info': {'joints_count': 1, 'nq': 1, 'nv': 1, 'total_mass': 2, 'reduced_total_mass': 2, 'effective_moving_mass': 2, 'frame_names': ['base', 'link1'], 'effective_inertias': [{'joint_name': 'joint1', 'mass': 2, 'center_of_mass': [0.1, 0, 0], 'inertia': [[0.2, 0, 0], [0, 0.3, 0], [0, 0, 0.4]]}]},
                'state': {'positions': [0], 'gravity': [0], 'gravity_compensation': [0], 'mass_matrix': [[1]], 'center_of_mass': [0.1, 0, 0], 'frames': [{'name': 'base', 'position': [0, 0, 0], 'quaternion': [0, 0, 0, 1]}, {'name': 'link1', 'position': [0, 0, 0], 'quaternion': [0, 0, 0, 1]}]},
            }
            probe.write_text("#!/usr/bin/env python3\nimport json\nprint(" + repr(json.dumps(payload)) + ")\n")
            probe.chmod(0o755)
            inspector = Inspector(root)
            inspector.model_probe = lambda: str(probe)
            info = inspector.inspect({'profile': 'test'})
            self.assertTrue(info['available']['model'])
            self.assertFalse(any(c['ok'] for c in info['checks'] if c['name'] == 'hardware'))
            model = inspector.model({'profile': 'test'})
            self.assertTrue(model['offline'])
            self.assertEqual(model['model']['joints'][0]['axis'], [0.0, 1.0, 0.0])
            self.assertEqual(model['model']['links'][1]['inertial']['mass'], 2.0)
            self.assertEqual(model['native']['info']['effective_inertias'][0]['joint_name'], 'joint1')




class ModelRuntimeTests(unittest.TestCase):
    def test_preview_reuses_persistent_probe_process(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            core = root / 'core.yaml'
            core.write_text('model: {}\n')
            probe = root / 'probe.py'
            probe.write_text("""#!/usr/bin/env python3
import json,os,sys
for line in sys.stdin:
    if line.rstrip('\\n') == 'quit': break
    values=[] if not line.strip() else [float(x) for x in line.strip().split(',')]
    print(json.dumps({'ok':True,'pid':os.getpid(),'state':{'positions':values}}),flush=True)
""")
            probe.chmod(0o755)

            class FixtureInspector:
                def inspect(self, config):
                    return {'resources': {'core': str(core)}, 'profile_file': str(root / 'profiles.yaml')}
                def model_probe(self):
                    return str(probe)

            runtime = ModelRuntime()
            self.addCleanup(runtime.close)
            inspector = FixtureInspector()
            first = runtime.preview(inspector, {'profile': 'fixture'}, [0.1, 0.2])
            second = runtime.preview(inspector, {'profile': 'fixture'}, [0.3, 0.4])
            self.assertEqual(first['pid'], second['pid'])
            self.assertEqual(second['state']['positions'], [0.3, 0.4])

class NativeSessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.events = []
        self.native = NativeSession(self.events.append, Path(self.tmp.name) / 'locks')
        self.addCleanup(self.native.close)
        self.server = Path(self.tmp.name) / 'server.py'
        self.server.write_text('''import json,sys
print(json.dumps({"event":"telemetry","time_ms":1,"data":{"robot_state":"INACTIVE"}}),flush=True)
for line in sys.stdin:
    r=json.loads(line); m=r.get("method"); i=r.get("id")
    if m=="shutdown": print(json.dumps({"id":i,"result":True}),flush=True); break
    if m=="fail": print(json.dumps({"id":i,"error":"rejected"}),flush=True); continue
    print(json.dumps({"id":i,"result":{"method":m,"params":r.get("params",{})}}),flush=True)
''')

    def wait(self, test):
        deadline = time.time() + 4
        while time.time() < deadline:
            if test(): return
            time.sleep(.02)
        self.fail('native session timeout')

    def test_structured_requests_events_shutdown_and_device_lease(self):
        self.native.start([sys.executable, '-u', str(self.server)], dict(os.environ), ['/dev/native-test'])
        result = self.native.request('hold', {'x': 1})
        self.assertEqual(result['method'], 'hold')
        self.assertEqual(result['params'], {'x': 1})
        self.wait(lambda: any(e.get('event') == 'telemetry' for e in self.events))
        second = Supervisor(lambda e: None, Path(self.tmp.name) / 'locks')
        self.addCleanup(second.close)
        with self.assertRaises(ValueError):
            second.start([sys.executable, '-c', 'pass'], dict(os.environ), ['/dev/native-test'], 'hardware')
        with self.assertRaisesRegex(ValueError, 'rejected'):
            self.native.request('fail')
        self.native.stop()
        self.wait(lambda: self.native.status()['state'] == 'exited')
        self.assertEqual(self.native.status()['exit_code'], 0)
        second.start([sys.executable, '-c', 'pass'], dict(os.environ), ['/dev/native-test'], 'hardware')


class ProfileLibraryTests(unittest.TestCase):
    def make_description(self, root):
        package = root / 'sample_description'
        package.mkdir(parents=True)
        (package / 'package.xml').write_text('<package><name>sample_description</name></package>')
        (package / 'robot.urdf').write_text("""<robot name="sample">
<link name="base"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual><inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
<link name="link1"><visual><geometry><box size="0.1 0.1 0.2"/></geometry></visual><inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
<link name="tool0"/>
<joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint>
<joint name="tool0_joint" type="fixed"><parent link="link1"/><child link="tool0"/><origin xyz="0 0 0.2"/></joint>
</robot>""")
        return package

    def test_description_to_profile_library_lifecycle(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            source = self.make_description(temp / 'source')
            root = ROOT
            inspector = Inspector(root)
            old = os.environ.get('SERIAL_ARM_PROFILE_LIBRARY')
            os.environ['SERIAL_ARM_PROFILE_LIBRARY'] = str(temp / 'profile-library.yaml')
            try:
                library = ProfileLibrary(root, inspector)
                description = library.inspect_description(str(source))
                self.assertEqual(description['roots'], ['base'])
                self.assertEqual(description['preview']['native']['config']['tool_frame'], 'tool0')
                created = library.create_profile({
                    'source': str(source), 'profile': 'sample', 'destination': str(temp / 'profiles'),
                    'base_frame': 'base', 'tool_frame': 'tool0', 'joint_names': ['joint1'],
                    'hardware_plugin': 'serial_arm_hardware_damiao', 'bus': 'main_can',
                    'device': '/dev/ttyACM0', 'baudrate': '921600',
                    'actuators': [{'name': 'actuator1', 'motor_id': 1, 'master_id': 0, 'motor_type': 'DM4310'}],
                })
                self.assertTrue(Path(created['profile_file']).is_file())
                info = inspector.inspect(created['config'])
                self.assertFalse(info['write_enabled'])
                registered = library.register(created['profile_file'], 'sample', created['resource_paths'], 'suffix')
                self.assertEqual(registered['id'], 'user:sample')
                listing = library.list()
                external = next(x for x in listing['profiles'] if x['id'] == 'user:sample')
                self.assertEqual(external['readiness']['state'], 'pending_validation')
                updated = library.mark_stage(created['config'], 'model_check', True)
                self.assertEqual(updated['checks']['model_check'], 'verified')
                library.remove('user:sample')
                self.assertFalse(any(x['id'] == 'user:sample' for x in library.list()['profiles']))
            finally:
                if old is None: os.environ.pop('SERIAL_ARM_PROFILE_LIBRARY', None)
                else: os.environ['SERIAL_ARM_PROFILE_LIBRARY'] = old


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.events = []
        self.manager = Supervisor(self.events.append, Path(self.tmp.name))
        self.addCleanup(self.manager.close)

    def wait(self, test):
        deadline = time.time() + 4
        while time.time() < deadline:
            if test(): return
            time.sleep(.02)
        self.fail('runtime timeout')

    def test_pty_is_interactive_and_input_reaches_process(self):
        self.manager.start([sys.executable, '-u', '-c',
                            'import sys; print("READY",sys.stdin.isatty()); print(input())'],
                           dict(os.environ), [], 'terminal')
        self.wait(lambda: 'READY True' in ''.join(e.get('data', '') for e in self.events))
        self.manager.input('hello\r')
        self.wait(lambda: self.manager.status()['state'] == 'exited')
        self.assertIn('hello', ''.join(e.get('data', '') for e in self.events))

    def test_device_lock_prevents_second_launcher_and_is_released(self):
        second = Supervisor(lambda e: None, Path(self.tmp.name))
        self.addCleanup(second.close)
        self.manager.start([sys.executable, '-c', 'import time; time.sleep(30)'],
                           dict(os.environ), ['/dev/test-device'], 'hardware')
        with self.assertRaises(ValueError):
            second.start([sys.executable, '-c', 'pass'], dict(os.environ), ['/dev/test-device'], 'hardware')
        self.manager.stop()
        self.wait(lambda: self.manager.status()['state'] == 'exited')
        second.start([sys.executable, '-c', 'pass'], dict(os.environ), ['/dev/test-device'], 'hardware')

    def test_only_one_live_session_and_start_failure_releases_locks(self):
        with self.assertRaises(OSError):
            self.manager.start(['/does/not/exist'], dict(os.environ), ['/dev/test'], 'hardware')
        self.manager.start([sys.executable, '-c', 'import time; time.sleep(30)'], dict(os.environ), ['/dev/test'], 'hardware')
        with self.assertRaises(ValueError):
            self.manager.start([sys.executable, '-c', 'pass'], dict(os.environ), [], 'terminal')


if __name__ == '__main__': unittest.main()

class PersistenceTests(unittest.TestCase):
    def test_narrow_persistence_preserves_unrelated_text_and_detects_conflict(self):
        import tempfile
        from persistence import preview, save
        joints = ['joint1', 'joint2']
        text = '''# keep me\nmodel:\n  gravity_scale: {joint1: 1.0, joint2: 1.0} # keep comment\ncapability:\n  admittance:\n    observer:\n      mode: FULL_ID\n      momentum_gain: {joint1: 10, joint2: 10}\n    calibration:\n      torque_bias: {joint1: 0, joint2: 0}\n      torque_threshold: {joint1: 0.1, joint2: 0.1}\n      friction:\n        enabled: false\n        velocity_transition: 0.03\n        positive_coulomb: {joint1: 0, joint2: 0}\n        positive_viscous: {joint1: 0, joint2: 0}\n        negative_coulomb: {joint1: 0, joint2: 0}\n        negative_viscous: {joint1: 0, joint2: 0}\n    controller:\n      mass: {joint1: 1, joint2: 1}\n      damping: {joint1: 2, joint2: 2}\n      stiffness: {joint1: 3, joint2: 3}\n      max_delta_q: {joint1: 0.5, joint2: 0.5}\n      max_delta_q_dot: {joint1: 1, joint2: 1}\nunrelated:\n  answer: 42\n'''
        runtime = {'observer_mode':'MOMENTUM','momentum_gain':[20,21],'mass':[1.1,1.2],'damping':[2.1,2.2],
                   'stiffness':[3.1,3.2],'max_delta_q':[0.4,0.4],'max_delta_q_dot':[0.9,0.9],
                   'torque_bias':[0.01,0.02],'torque_threshold':[0.2,0.3],
                   'friction':{'enabled':True,'velocity_transition':0.03,'positive_coulomb':[1,2],
                               'positive_viscous':[3,4],'negative_coulomb':[5,6],'negative_viscous':[7,8]}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'core.yaml'; path.write_text(text)
            diff = preview(path, joints, runtime, [1.01, 0.99])
            self.assertTrue(diff['changed']); self.assertGreater(len(diff['changes']), 5)
            result = save(path, diff['sha256'], joints, runtime, [1.01, 0.99])
            self.assertTrue(result['changed']); self.assertTrue(Path(result['backup']).is_file())
            updated = path.read_text(); self.assertIn('# keep me', updated); self.assertIn('answer: 42', updated); self.assertIn('# keep comment', updated)
            path.write_text(updated + '# external\n')
            with self.assertRaises(ValueError): save(path, result['sha256'], joints, runtime, [1.0, 1.0])

class ModelCalibrationPersistenceTests(unittest.TestCase):
    def make_task(self, root):
        from persistence import sha256_file
        root = Path(root)
        task = root / 'task'; task.mkdir()
        urdf = root / 'robot.urdf'
        urdf.write_text('<robot name="r"><link name="base"/><link name="link1"><inertial><origin xyz="0.1 0 0" rpy="0 0 0"/><mass value="2"/><inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/></inertial></link><joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint></robot>')
        core = root / 'core.yaml'
        core.write_text(f'''model:\n  joint_names: [joint1]\n  urdf_path: {urdf}\n  base_frame: base\n  tool_frame: link1\n  gravity: [0, 0, -9.81]\n  gravity_scale: [1]\n''')
        metadata = {'task_id':'task-1','core_config_path':str(core),'core_fingerprint':sha256_file(core),'urdf_path':str(urdf),'urdf_fingerprint':sha256_file(urdf),'joint_names':['joint1'],'original_gravity_scale':[1.0],'resource_paths':[],'units':{'position':'rad','velocity':'rad/s','torque':'Nm'},'mapping':{},'load_description':'fixture','calibration_options':{'max_com_offset_m':0.05,'regularization':0.01,'svd_relative_threshold':0.0001}}
        gravity = {'static_pass':True,'constraints_ok':True,'numerical_rank':1,'first_moments':[{'link_name':'link1','value':[0.24,0.0,0.0]}],'torque_bias':[0.01],'validation_candidate':{'rms':[0.1],'p99':[0.15],'maximum':[0.2],'bias':[0.0]},'validation_original':{'rms':[0.3],'p99':[0.4],'maximum':[0.5],'bias':[0.0]},'validation_scaled':{'rms':[0.25],'p99':[0.35],'maximum':[0.45],'bias':[0.0]},'training_original':{'rms':[0.3]},'training_candidate':{'rms':[0.1]},'noise_rms':[0.02]}
        result = {'task_id':'task-1','phase':'complete','core_fingerprint':metadata['core_fingerprint'],'urdf_fingerprint':metadata['urdf_fingerprint'],'joint_names':['joint1'],'original_gravity_scale':[1.0],'gravity_result':gravity,'friction_pass':False,'friction':{}}
        (task/'metadata.json').write_text(json.dumps(metadata)); (task/'result.json').write_text(json.dumps(result)); (task/'frames.csv').write_text('header\n'); (task/'trajectory.csv').write_text('sample_dt,0.01\nindex,joint1\n0,0\n')
        return task, core, urdf

    def test_candidate_save_load_and_restore_are_traceable(self):
        from persistence import load_model_calibration_summary, preview_gravity_correction, save_gravity_correction, restore_gravity_correction, sha256_file
        with tempfile.TemporaryDirectory() as temp:
            task, core, _ = self.make_task(temp)
            loaded = load_model_calibration_summary(task)
            self.assertEqual(loaded['task_id'], 'task-1')
            preview = preview_gravity_correction(core, task)
            self.assertTrue(preview['changed'])
            saved = save_gravity_correction(core, preview['sha256'], task)
            self.assertTrue(Path(saved['gravity_correction_path']).is_file())
            self.assertIn('gravity_correction_path:', core.read_text())
            restored = restore_gravity_correction(core, task)
            self.assertTrue(Path(restored['core_path']).is_file())
            self.assertNotIn('gravity_correction_path:', core.read_text())
            self.assertEqual(sha256_file(core), restored['core_sha256'])

    def test_unbounded_candidate_exports_when_holdout_fails_and_com_is_far(self):
        from persistence import export_candidate_urdf, preview_gravity_correction
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as temp:
            task, core, _ = self.make_task(temp)
            saved = json.loads((task / 'result.json').read_text())
            saved['phase'] = 'failed'
            saved['gravity_result']['static_pass'] = False
            saved['gravity_result']['constraints_ok'] = False  # legacy rejected model
            saved['gravity_result']['failure_reason'] = 'center_of_mass_constraint_conflict'
            saved['gravity_result']['first_moments'][0]['value'] = [3.0, 0.0, 0.0]
            (task / 'result.json').write_text(json.dumps(saved))
            # This unreviewed candidate must never enter the live applied correction.
            with self.assertRaises(ValueError):
                preview_gravity_correction(core, task)
            out = export_candidate_urdf(task)
            self.assertTrue(out['review_required'])
            self.assertFalse(out['holdout_pass'])
            link = ET.parse(out['candidate_urdf']).getroot().find("link[@name='link1']")
            self.assertEqual(link.find('inertial/origin').get('xyz'), '1.5 0 0')
            self.assertEqual(link.find('inertial/mass').get('value'), '2')

    def test_offline_recompute_overrides_rejected_historical_first_moments(self):
        from persistence import export_candidate_urdf
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as temp:
            task, _, _ = self.make_task(temp)
            new_candidate = {'static_pass': False, 'solver': 'robust_centered_svd_ridge_unbounded',
                'first_moments': [{'link_name': 'link1', 'value': [2.8, 0, 0]}],
                'torque_bias': [0.0], 'failure_reason': 'holdout_validation_not_improved',
                'validation_candidate': {'p99': [0.3]}, 'noise_rms': [0.01]}
            (task / 'recomputed-candidate.json').write_text(json.dumps(new_candidate))
            output = export_candidate_urdf(task)
            self.assertTrue(output['review_required'])
            self.assertEqual(output['candidate_solver'], 'robust_centered_svd_ridge_unbounded')
            link = ET.parse(output['candidate_urdf']).getroot().find("link[@name='link1']")
            self.assertEqual(link.find('inertial/origin').get('xyz'), '1.4 0 0')

    def test_candidate_urdf_changes_only_calibrated_inertial_origin(self):
        from persistence import export_candidate_urdf
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as temp:
            task, _, _ = self.make_task(temp)
            exported = export_candidate_urdf(task)
            root = ET.parse(exported['candidate_urdf']).getroot()
            link = next(node for node in root.findall('link') if node.get('name') == 'link1')
            inertial = link.find('inertial')
            self.assertEqual(inertial.find('mass').get('value'), '2')
            self.assertEqual(inertial.find('inertia').get('ixx'), '0.2')
            self.assertEqual(inertial.find('origin').get('xyz'), '0.12 0 0')
            self.assertFalse(exported['joint_geometry_modified'])


    def test_candidate_urdf_preserves_namespace_and_copies_relative_meshes(self):
        from persistence import export_candidate_urdf, sha256_file
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as temp:
            task, _, urdf = self.make_task(temp)
            mesh = urdf.parent / 'meshes' / 'link1.stl'
            mesh.parent.mkdir()
            mesh.write_text('solid fixture\nendsolid fixture\n')
            urdf.write_text('<robot xmlns="urn:serial-arm:test" name="r"><link name="base"/><link name="link1"><inertial><origin xyz="0.1 0 0" rpy="0 0 0"/><mass value="2"/><inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/></inertial><visual><geometry><mesh filename="meshes/link1.stl"/></geometry></visual></link><joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="5" velocity="2"/></joint></robot>')
            metadata_path = task / 'metadata.json'
            result_path = task / 'result.json'
            metadata = json.loads(metadata_path.read_text())
            result = json.loads(result_path.read_text())
            metadata['urdf_fingerprint'] = sha256_file(urdf)
            result['urdf_fingerprint'] = metadata['urdf_fingerprint']
            metadata_path.write_text(json.dumps(metadata))
            result_path.write_text(json.dumps(result))

            exported = export_candidate_urdf(task)
            candidate = Path(exported['candidate_urdf'])
            root = ET.parse(candidate).getroot()
            self.assertTrue(root.tag.startswith('{urn:serial-arm:test}'))
            link = next(node for node in root if node.tag.endswith('link') and node.get('name') == 'link1')
            inertial = next(node for node in link if node.tag.endswith('inertial'))
            origin = next(node for node in inertial if node.tag.endswith('origin'))
            self.assertEqual(origin.get('xyz'), '0.12 0 0')
            mesh_node = next(node for node in root.iter() if node.tag.endswith('mesh'))
            self.assertTrue(mesh_node.get('filename').startswith('resources/'))
            self.assertTrue((candidate.parent / mesh_node.get('filename')).is_file())
