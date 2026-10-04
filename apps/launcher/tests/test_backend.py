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
