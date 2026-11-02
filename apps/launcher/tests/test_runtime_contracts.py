import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from profiles import Inspector
from profile_library import ProfileLibrary, _safe_extract
from gravity_mass_com import observable_gravity_basis
sys.path.insert(0, str(ROOT / 'tools/install'))
from provenance import environment_problem
from plan import parse_plan
from runner import Runner
sys.path.insert(0, str(ROOT / 'packaging/linux'))
from stage import python_gui_modules, validate_python_payload

URDF = '<robot name="test"><link name="base"/><link name="tip"/><joint name="axis" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>'

class RuntimeContracts(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name)
        self.inspector = Inspector(ROOT)
        self.library = ProfileLibrary(ROOT, self.inspector)
        self.library.cache = self.path / 'cache'
        self.library.library_file = self.path / 'library.yaml'
    def tearDown(self):
        self.temporary.cleanup()
    def test_explicit_model_is_not_replaced_by_sibling(self):
        first = self.path / 'first.urdf'; second = self.path / 'second.urdf'
        first.write_text(URDF); second.write_text(URDF.replace('axis', 'other'))
        self.assertEqual(self.library.inspect_description(str(second))['joints'][0]['name'], 'other')
        choices = self.library.inspect_description(str(self.path))['model_choices']
        self.assertEqual(choices, ['first.urdf', 'second.urdf'])
        selected = self.library.inspect_description(str(self.path), 'second.urdf')
        self.assertEqual(selected['joints'][0]['name'], 'other')
        with self.assertRaises(ValueError): self.library.inspect_description(str(self.path), '../escape.urdf')
    def test_xacro_expands_and_keeps_package_resources(self):
        source = self.path / 'arm'; source.mkdir()
        (source / 'package.xml').write_text('<package><name>test_arm</name></package>')
        (source / 'model.xacro').write_text(URDF)
        completed = mock.Mock(returncode=0, stdout=URDF, stderr='')
        with mock.patch('profile_library.subprocess.run', return_value=completed) as run:
            result = self.library.inspect_description(str(source / 'model.xacro'))
        self.assertTrue(Path(result['urdf']).is_file())
        self.assertTrue(result['urdf'].endswith('.expanded.urdf'))
        self.assertEqual(result['package_name'], 'test_arm')
        self.assertIn('xacro', run.call_args.args[0][2])
        self.assertFalse((source / 'model.xacro.expanded.urdf').exists())
    def test_zip_cannot_escape_destination(self):
        archive = self.path / 'model.zip'
        with zipfile.ZipFile(archive, 'w') as output: output.writestr('../escape', 'bad')
        with self.assertRaises(ValueError): _safe_extract(archive, self.path / 'out')
        self.assertFalse((self.path / 'escape').exists())
    def test_inertia_requires_physical_principal_moments(self):
        self.assertFalse(self.inspector._inertia_valid([[1,0,0],[0,1,0],[0,0,3]], 1)[0])
        self.assertTrue(self.inspector._inertia_valid([[1,0,0],[0,1,0],[0,0,1]], 1)[0])
    def test_builtin_without_evidence_is_not_ready(self):
        with mock.patch.dict(os.environ, {'XDG_DATA_HOME':str(self.path),'SERIAL_ARM_PROFILE_LIBRARY':str(self.path/'library.yaml')}):
            result = self.library._readiness({'profile':'dm_arm_gray'})
        self.assertEqual(result['state'], 'pending_validation')
        self.assertTrue(result['control_available'])
    def test_gravity_basis_preserves_observable_subspace(self):
        import numpy as np
        matrix=np.array([[0.,0.,1.],[1.,0.,1.],[2.,0.,1.],[3.,0.,1.]])
        basis, singular, rank=observable_gravity_basis(matrix,1,np.ones(1))
        self.assertEqual(rank,1)
        self.assertTrue(np.allclose(basis@basis.T,np.diag([1,0,0])))
    def test_moved_checkout_reports_complete_rebuild_command(self):
        state = self.path / '.install'; state.mkdir()
        (state / 'manifest.json').write_text(json.dumps({
            'root': str(self.path / 'previous-location'), 'preset': 'ros2',
            'robot': 'dm_arm', 'ros_distro': 'humble', 'terminal': True,
            'python': True, 'ros2': True, 'moveit': True, 'gui': True, 'tests': True}))
        problem = environment_problem(self.path)
        self.assertIn('源码目录已迁移', problem)
        self.assertIn(str(self.path / 'install.sh'), problem)
        self.assertIn('--clean-rebuild', problem)
        self.assertIn('--with-python', problem)
        self.assertIn('--with-tests', problem)
    def test_environment_detects_broken_native_install_link(self):
        state = self.path / '.install'; state.mkdir()
        setup = state / 'environment.bash'; setup.write_text(':\n')
        prefix = self.path / 'install'; prefix.mkdir()
        (prefix / 'terminal').symlink_to(self.path / 'deleted-build/terminal')
        (state / 'manifest.json').write_text(json.dumps({
            'root': str(self.path), 'workspace_setup': str(setup), 'install_prefix': str(prefix)}))
        self.assertIn('安装链接已失效', environment_problem(self.path))
    def test_clean_rebuild_only_removes_selected_native_configuration(self):
        plan = parse_plan(['--preset', 'core', '--clean-rebuild'], self.path)
        runner = Runner(plan)
        for directory in (runner.build, runner.prefix, self.path / 'install/unified/other'):
            directory.mkdir(parents=True)
            (directory / 'marker').write_text('generated')
        state = self.path / '.install'; state.mkdir()
        (state / 'manifest.json').write_text('{}')
        runner.clean_configuration()
        self.assertFalse(runner.build.exists())
        self.assertFalse(runner.prefix.exists())
        self.assertTrue((self.path / 'install/unified/other/marker').is_file())
        self.assertTrue((state / 'manifest.json').is_file())
    def test_payload_cannot_borrow_solver_from_host_pythonpath(self):
        with mock.patch.dict(os.environ, {'PYTHONPATH': str(ROOT / '.install/gui-venv/lib/python3.10/site-packages')}):
            with self.assertRaisesRegex(ValueError, 'Isolated packaged Python solver check failed'):
                validate_python_payload(self.path)
    @unittest.skipUnless((ROOT / '.install/gui-venv/bin/python').is_file(), 'GUI environment not provisioned')
    def test_packaged_dependency_closure_solves_without_host_packages(self):
        python_gui_modules(ROOT, self.path)
        self.assertTrue((self.path / 'python/cvxpy').is_dir())
        self.assertTrue((self.path / 'python/clarabel').is_dir())
        self.assertTrue(any((self.path / 'python').glob('_cffi_backend*.so')))

if __name__ == '__main__': unittest.main()
