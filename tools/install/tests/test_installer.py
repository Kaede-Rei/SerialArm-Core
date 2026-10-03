import importlib.util
import json
import contextlib
import io
from unittest.mock import patch
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools/install'))
from plan import parse_plan


class PlanTests(unittest.TestCase):
    def plan(self, *args):
        return parse_plan(list(args), ROOT)

    def test_core_excludes_optional_components(self):
        p = self.plan('--preset', 'core')
        self.assertFalse(p.terminal or p.python or p.ros2 or p.moveit or p.gui)
        self.assertEqual(p.robot, 'none')

    def test_presets_and_alias(self):
        for name in ('core', 'standalone', 'standalone-dm', 'python', 'ros2', 'dev'):
            self.assertEqual(self.plan('--preset', name).preset,
                             'standalone' if name == 'standalone-dm' else name)
        self.assertTrue(self.plan('--preset', 'dev').tests)

    def test_ros2_requires_python_binding_for_profile_resolution(self):
        self.assertTrue(self.plan('--preset', 'ros2').python)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.plan('--preset', 'ros2', '--without-python')

    def test_gui_only_does_not_select_core_build_packages(self):
        p = self.plan('--preset', 'custom', '--gui-only')
        self.assertTrue(p.gui)
        self.assertEqual(p.packages, [])
        self.assertFalse(p.ros2 or p.python or p.terminal)

    def test_dependency_normalization(self):
        p = self.plan('--preset', 'custom', '--with-moveit', '--robot', 'none')
        self.assertTrue(p.ros2 and p.moveit)
        self.assertEqual(p.robot, 'dm_arm')

    def test_invalid_combinations(self):
        for args in [('--preset', 'core', '--jobs', '0'),
                     ('--preset', 'bad'),
                     ('--preset', 'custom', '--with-moveit', '--without-ros2'),
                     ('--preset', 'core', '--ros-distro', '../bad')]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.plan(*args)

    def test_external_support_keeps_generic_adapter(self):
        p = self.plan('--preset', 'ros2', '--robot', 'none', '--without-moveit')
        self.assertIn('serial_arm_ros2_control', p.packages)
        self.assertNotIn('dm_arm_description', p.packages)
        self.assertNotIn('serial_arm_hardware_damiao', p.packages)

    def test_yes_does_not_enable_source_build(self):
        self.assertFalse(self.plan('--preset', 'standalone', '--yes').allow_source_build)

    def test_dry_run_no_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = subprocess.run([sys.executable, str(ROOT / 'tools/install/main.py'),
                                   '--preset', 'dev', '--dry-run'], cwd=tmp,
                                  text=True, capture_output=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            data = json.loads(proc.stdout)
            self.assertEqual(data['gui_status'], 'selected')
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_wizard_feeds_same_plan(self):
        from main import wizard
        with patch('builtins.input', side_effect=['2', 'y']) as prompt, contextlib.redirect_stdout(io.StringIO()):
            args = wizard([])
        self.assertRegex(prompt.call_args_list[0].args[0], r'\[默认 [24]\]')
        self.assertIn('默认 y', prompt.call_args_list[1].args[0])
        p = parse_plan(args, ROOT)
        self.assertEqual(p.preset, 'standalone')
        self.assertEqual(p.robot, 'dm_arm')
        self.assertTrue(p.terminal)

    def test_no_tty_requires_preset(self):
        proc = subprocess.run(['bash', str(ROOT / 'install.sh')], input='',
                              text=True, capture_output=True)
        self.assertEqual(proc.returncode, 2)
        self.assertIn('--preset', proc.stderr)


if __name__ == '__main__':
    unittest.main()
