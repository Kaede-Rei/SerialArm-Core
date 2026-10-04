import contextlib
import ctypes
import fcntl
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from plan import parse_plan
from runner import InstallError, Runner, install, standalone_env


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='serial-arm test ')
        self.root = Path(self.tmp.name)
        self.addCleanup(self.tmp.cleanup)

    def plan(self, *args):
        return parse_plan(list(args), self.root)

    def artifacts(self, r, bad_lib=False):
        r.prefix.mkdir(parents=True, exist_ok=True)
        r.logs.mkdir(parents=True, exist_ok=True)
        r.setup.write_text('export SERIAL_ARM_TEST=ready\n')
        lib = r.prefix / 'lib/libserial_arm_core.so'
        lib.parent.mkdir()
        if bad_lib:
            lib.write_text('broken shared library')
        else:
            subprocess.run(['cc', '-shared', '-fPIC', '-x', 'c', '-o', str(lib), '-'],
                           input='int installer_test(void) { return 1; }', text=True, check=True)
        binary = r.prefix / 'bin/serial_arm_terminal'
        binary.parent.mkdir()
        binary.write_text('#!/bin/sh\n[ "$1" = "--help" ]\n')
        binary.chmod(0o755)
        model_probe = r.prefix / 'bin/serial_arm_model_probe'
        model_probe.write_text('#!/bin/sh\n[ "$1" = "--help" ]\n')
        model_probe.chmod(0o755)

    def test_verify_ros_terminal_matches_existing_bin_layout(self):
        r = Runner(self.plan('--preset', 'ros2', '--without-moveit', '--robot', 'none'))
        self.artifacts(r)
        with patch.object(r, 'run') as command:
            r.verify()
            terminal = [c for c in command.call_args_list if c.args[0] == 'verify-terminal'][0]
            self.assertEqual(terminal.args[1][0], r.prefix / 'bin/serial_arm_terminal')
            model_probe = [c for c in command.call_args_list if c.args[0] == 'verify-model-probe'][0]
            self.assertEqual(model_probe.args[1][0], r.prefix / 'bin/serial_arm_model_probe')

    def test_success_records_only_verified_components_and_sources_spaces(self):
        p = self.plan('--preset', 'core', '--skip-system-deps')
        def build(r):
            self.artifacts(r)
        with patch.object(Runner, 'preflight'), patch.object(Runner, 'native', build), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(install(p), 0)
        manifest = json.loads((self.root / '.install/manifest.json').read_text())
        self.assertEqual(manifest['installed_components'], ['core'])
        self.assertFalse(manifest['gui'])
        proc = subprocess.run(['bash', '-c', 'source "$1" && test "$SERIAL_ARM_TEST" = ready',
                               'test', str(self.root / '.install/setup.bash')])
        self.assertEqual(proc.returncode, 0)
        proc = subprocess.run([str(self.root / '.install/run'), sys.executable, '-c',
                               'import sys; print(sys.argv[1])', 'a b; $(echo unexpected)'],
                              text=True, capture_output=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(proc.stdout.strip(), 'a b; $(echo unexpected)')

    def test_failed_verification_preserves_successful_manifest(self):
        state = self.root / '.install'
        state.mkdir()
        previous = '{"previous": true}\n'
        (state / 'manifest.json').write_text(previous)
        p = self.plan('--preset', 'core')
        def build(r):
            self.artifacts(r, bad_lib=True)
        with patch.object(Runner, 'preflight'), patch.object(Runner, 'native', build), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(install(p), 1)
        self.assertEqual((state / 'manifest.json').read_text(), previous)
        attempt = json.loads((state / 'last_attempt.json').read_text())
        self.assertEqual(attempt['status'], 'failed')
        self.assertEqual(attempt['stage'], 'verify-core')

    def test_installation_lock(self):
        state = self.root / '.install'
        state.mkdir()
        with (state / 'install.lock').open('w') as lock, contextlib.redirect_stderr(io.StringIO()):
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(install(self.plan('--preset', 'core')), 3)
            self.assertFalse((state / 'manifest.json').exists())

    def test_missing_ros_fails_before_system_package_changes(self):
        r = Runner(self.plan('--preset', 'ros2'))
        r.ros_setup = self.root / 'missing/ros/setup.bash'
        with patch.object(r, 'apt') as apt, self.assertRaises(InstallError):
            r.preflight()
        apt.assert_not_called()

    def test_ros_build_scopes_packages_and_optional_cmake_flags(self):
        r = Runner(self.plan('--preset', 'ros2', '--without-moveit', '--robot', 'none', '--skip-system-deps'))
        r.plan.package_paths = lambda: [str(self.root / 'core'), str(self.root / 'adapter')]
        with patch.object(r, 'run') as command:
            r.ros()
        call = [c for c in command.call_args_list if c.args[0] == 'ros2-build'][0]
        args = list(map(str, call.args[1]))
        self.assertIn('--base-paths', args)
        self.assertNotIn('--packages-up-to', args)
        self.assertIn('-DSERIAL_ARM_BUILD_PYTHON=ON', args)
        self.assertIn('-DBUILD_TESTING=OFF', args)
        self.assertFalse(any(c.args[0].startswith('apt-') or c.args[0].startswith('rosdep-')
                             for c in command.call_args_list))

    def test_rosdep_dependency_types_use_repeated_options_and_tests_use_merge(self):
        r = Runner(self.plan('--preset', 'dev'))
        r.plan.package_paths = lambda: ['/tmp/core']
        with patch.object(r, 'apt'), patch.object(r, 'run') as command:
            r.ros()
        calls = {c.args[0]: list(map(str, c.args[1])) for c in command.call_args_list}
        args = calls['rosdep-install']
        self.assertEqual(args.count('--dependency-types'), 6)
        self.assertIn('--merge-install', calls['ros2-test'])

    def test_filtered_env_preserves_non_ros_system_prefix(self):
        env = standalone_env({'CMAKE_PREFIX_PATH': '/opt/ros/humble;/some/overlay;/opt/openrobots',
                              'AMENT_PREFIX_PATH': '/some/overlay', 'ROS_DISTRO': 'humble'})
        self.assertEqual(env['CMAKE_PREFIX_PATH'], '/opt/openrobots')
        self.assertNotIn('ROS_DISTRO', env)

    def test_conan_bootstrap_is_lazy_for_system_dependencies(self):
        r = Runner(self.plan('--preset', 'core', '--skip-system-deps'))
        with patch('runner.shutil.which', side_effect=lambda c: None if c == 'conan' else '/usr/bin/' + c), \
             patch.object(r, 'run') as command:
            r.native()
        self.assertEqual([c.args[0] for c in command.call_args_list], ['standalone-build'])

    def test_bootstrap_real_script_honors_terminal_off_and_build_root(self):
        import importlib.util
        root = Path(__file__).resolve().parents[3]
        spec = importlib.util.spec_from_file_location('bootstrap_test', root / 'src/serial_arm/core/tests/bootstrap_standalone_cli_test.py')
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
        fake, log = old.make_fake_tools(self.root, system_deps=True, binary_available=False)
        env = dict(os.environ, PATH=str(fake) + ':' + os.environ['PATH'],
                   SERIALARM_TEST_LOG=str(log), BOOTSTRAP_BUILD_ROOT=str(self.root / 'build'),
                   CORE_BUILD_DIR=str(self.root / 'build/core'), INSTALL_PREFIX=str(self.root / 'install'),
                   SERIAL_ARM_ALLOW_SOURCE_BUILD='0')
        proc = subprocess.run(['bash', str(root / 'tools/bootstrap_standalone.sh'),
                               '--robot', 'dm_arm', '--without-terminal'], env=env, text=True, capture_output=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        calls = log.read_text()
        self.assertIn('-DSERIAL_ARM_BUILD_TERMINAL=OFF', calls)
        self.assertIn(str(self.root / 'build/serial_arm_hardware_damiao'), calls)
        self.assertNotIn('conan ', calls)

    def test_missing_conan_provisions_private_venv_then_retries(self):
        r = Runner(self.plan('--preset', 'core', '--skip-system-deps'))
        def execute(stage, args, **kwargs):
            if stage == 'standalone-build':
                raise InstallError('error: required command not found: conan')
        with patch('runner.shutil.which', return_value='/usr/bin/fake'), patch.object(r, 'run', side_effect=execute) as command:
            r.native()
        self.assertEqual([c.args[0] for c in command.call_args_list],
                         ['standalone-build', 'conan-venv', 'conan-install', 'standalone-build-retry'])
        self.assertIn('--without-terminal', command.call_args_list[-1].args[1])
        self.assertTrue(r.env['PATH'].startswith(str(r.state / 'tools-venv/bin')))

    def test_gui_only_preserves_core_manifest_without_native_build(self):
        state = self.root / '.install'
        state.mkdir()
        previous = {'schema_version': 1, 'status': 'verified', 'installed_components': ['core'],
                    'workspace_setup': '/test/core/setup.bash', 'install_prefix': '/test/core'}
        (state / 'manifest.json').write_text(json.dumps(previous))
        p = self.plan('--preset', 'custom', '--gui-only')
        with patch.object(Runner, 'preflight'), patch('gui.install_gui', return_value={'gui': True, 'gui_status': 'ready'}), \
             patch.object(Runner, 'native') as native, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(install(p), 0)
        native.assert_not_called()
        manifest = json.loads((state / 'manifest.json').read_text())
        self.assertEqual(manifest['install_prefix'], previous['install_prefix'])
        self.assertEqual(manifest['installed_components'], ['core'])
        self.assertTrue(manifest['gui'])

    def test_gui_install_failure_does_not_commit_ready_state(self):
        state = self.root / '.install'
        state.mkdir()
        previous = '{"status": "verified", "gui": false}'
        (state / 'manifest.json').write_text(previous)
        p = self.plan('--preset', 'custom', '--gui-only')
        with patch.object(Runner, 'preflight'), patch('gui.install_gui', side_effect=InstallError('download failed')), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(install(p), 1)
        self.assertEqual((state / 'manifest.json').read_text(), previous)

    def test_source_optin_never_inherited_from_environment(self):
        with patch.dict(os.environ, {'SERIAL_ARM_ALLOW_SOURCE_BUILD': '1'}):
            r = Runner(self.plan('--preset', 'core', '--yes'))
        self.assertEqual(r.env['SERIAL_ARM_ALLOW_SOURCE_BUILD'], '0')

    def test_build_variants_do_not_share_install_prefix(self):
        core = Runner(self.plan('--preset', 'core'))
        terminal = Runner(self.plan('--preset', 'core', '--with-terminal'))
        self.assertNotEqual(core.prefix, terminal.prefix)


if __name__ == '__main__':
    unittest.main()
