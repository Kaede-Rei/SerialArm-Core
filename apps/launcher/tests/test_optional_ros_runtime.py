"""Native workbench boot and release selection stay independent of ROS."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
sys.path.insert(0, str(ROOT / 'tools/install'))
from profiles import Inspector
from plan import parse_plan
from runner import Runner
spec = importlib.util.spec_from_file_location('linux_stage', ROOT / 'packaging/linux/stage.py')
stage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stage)


class OptionalRosRuntime(unittest.TestCase):
    def test_packaged_components_are_actual_selection(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {
            'SERIAL_ARM_BUNDLE_PREFIX': folder, 'SERIAL_ARM_BUNDLE_COMPONENTS': '["core", "terminal", "python"]'}, clear=True):
            inspector = Inspector(ROOT)
            manifest = inspector.manifest()
            self.assertFalse(manifest['ros2'])
            self.assertFalse(manifest['moveit'])
            self.assertIn('未安装 ROS2', inspector.adapter_problem('hardware'))
            with patch('profiles.subprocess.run') as run:
                self.assertEqual(inspector.launch_environment('terminal'), dict(os.environ))
                run.assert_not_called()
                with self.assertRaisesRegex(ValueError, '未安装 ROS2'):
                    inspector.launch_environment('hardware')
                run.assert_not_called()

    def test_host_ros_is_not_sourced_in_native_setup(self):
        plan = parse_plan(['--preset', 'ros2', '--jobs', '1'], ROOT)
        runner = Runner(plan)
        self.assertNotIn('/opt/ros', runner.setup_text())
        self.assertNotIn('local_setup', runner.setup_text())
        self.assertIn('/opt/ros/humble/setup.bash', runner.ros_setup_text())
        self.assertIn('ros_workspace_setup', runner.entry_text('ros_workspace_setup'))

    def test_gui_refresh_preserves_native_and_explicit_ros_entries(self):
        manifest_file = ROOT / '.install/manifest.json'
        if not manifest_file.is_file(): self.skipTest('No local installation')
        with tempfile.TemporaryDirectory() as temporary:
            runner = Runner(parse_plan(['--preset', 'custom', '--gui-only'], ROOT))
            runner.state = Path(temporary)
            manifest = json.loads(manifest_file.read_text())
            runner.refresh_runtime_entries(manifest)
            (runner.state / 'manifest.json').write_text(json.dumps(manifest))
            self.assertNotIn('/opt/ros', Path(manifest['workspace_setup']).read_text())
            self.assertIn('ros2-setup.bash', (runner.state / 'run').read_text())
            result = subprocess.run([str(runner.state / 'run'), 'python3', '-c',
                'import serial_arm; from pathlib import Path; assert "/opt/ros/" not in Path("/proc/self/maps").read_text()'],
                env={'PATH': '/usr/bin:/bin', 'PYTHONDONTWRITEBYTECODE': '1'}, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_deb_has_no_mandatory_ros_dependency(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            (folder / 'project').mkdir()
            stage.make_manifest(ROOT, folder, folder / 'payload')
            package = json.loads((folder / 'project/package.json').read_text())
            self.assertFalse(any(name.startswith('ros-') for name in package['build']['deb']['depends']))
            self.assertIn('python3', package['build']['deb']['depends'])

    def test_release_manifest_accepts_standalone_without_ros_setup(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            prefix = root / 'install/native'
            names = ['serial_arm_core', 'serial_arm_robot_profiles', 'serial_arm_protocol_damiao_usb2can',
                     'serial_arm_hardware_damiao', 'dm_arm_description']
            for name in names: (prefix / 'share' / name).mkdir(parents=True)
            for name in stage.REQUIRED_BINARIES:
                binary = prefix / 'bin' / name
                binary.parent.mkdir(exist_ok=True)
                binary.write_text('#!/bin/sh\nexit 0\n')
                binary.chmod(0o755)
            for item in ('lib/libserial_arm_core.so', 'share/serial_arm_robot_profiles/config/robot_profiles.yaml'):
                target = prefix / item
                target.parent.mkdir(parents=True, exist_ok=True)
                target.touch()
            for item in ('.install/gui-venv/bin/python', 'apps/launcher/node_modules/electron/dist/electron',
                         'apps/launcher/renderer/vendor/three/build/three.module.js'):
                target = root / item
                target.parent.mkdir(parents=True, exist_ok=True)
                target.touch()
            manifest = {'status': 'verified', 'python': True, 'terminal': True, 'gui': True, 'ros2': False,
                        'moveit': False, 'robot': 'dm_arm', 'gui_status': 'ready', 'packages': names,
                        'installed_components': ['core', 'terminal', 'python'], 'install_prefix': str(prefix),
                        'native_receipt': {'source_sha256': 'native', 'installed_sha256': 'installed'},
                        'gui_source_sha256': 'gui', 'gui_assets_sha256': 'assets'}
            (root / '.install/manifest.json').write_text(json.dumps(manifest))
            with patch.object(stage, 'native_inputs', return_value={'source_sha256': 'native'}), \
                 patch.object(stage, 'installed_content', return_value='installed'), \
                 patch.object(stage, 'gui_inputs', return_value='gui'), patch.object(stage, 'gui_assets', return_value='assets'):
                self.assertEqual(stage.read_manifest(root), prefix)
                with self.assertRaisesRegex(SystemExit, 'selection differs'):
                    stage.read_manifest(root, True)

    def test_environment_cleaning_removes_ros_paths(self):
        with patch.dict(os.environ, {'PATH': '/opt/ros/humble/bin:/usr/bin', 'AMENT_PREFIX_PATH': '/tmp/overlay',
                                    'LD_LIBRARY_PATH': '/tmp/overlay/lib:/opt/ros/humble/lib:/opt/openrobots/lib',
                                    'ROS_DISTRO': 'humble'}, clear=True):
            env = stage.clean_runtime_environment()
            self.assertNotIn('ROS_DISTRO', env)
            self.assertNotIn('AMENT_PREFIX_PATH', env)
            self.assertEqual(env['LD_LIBRARY_PATH'], '/opt/openrobots/lib')


if __name__ == '__main__': unittest.main()
