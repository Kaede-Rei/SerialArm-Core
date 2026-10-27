"""A changed exported URDF must not inherit another model's recovery trust.

The native worker and task lookup are mocked; model comparison, production
YAML patching, and artifact copying run on temporary files. For an isolated
staging checkout, SERIALARM_TEST_DEPENDENCIES may name the unchanged production
backend containing persistence.py; an ordinary repository checkout needs no env.
"""
from contextlib import ExitStack
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import yaml

BACKEND = Path(__file__).resolve().parents[1] / 'backend'
SUPPORT = Path(os.environ.get('SERIALARM_TEST_DEPENDENCIES', BACKEND))
source_dependencies = ModuleType('model_calibration_source')
for name in ('matches_recorded_fingerprint', 'verified_source_urdf', 'resolve_relative_mesh'):
    setattr(source_dependencies, name, mock.Mock())
registry = ModuleType('workflow_registry')
registry.load = mock.Mock()
spec = importlib.util.spec_from_file_location('model_trust_persistence', SUPPORT / 'persistence.py')
persistence = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'model_calibration_source': source_dependencies}):
    spec.loader.exec_module(persistence)
spec = importlib.util.spec_from_file_location('model_trust_exports', BACKEND / 'artifact_exports.py')
exports = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'model_calibration_source': source_dependencies,
                                  'persistence': persistence, 'workflow_registry': registry}):
    spec.loader.exec_module(exports)


MODEL = '''<robot name="fixture">
  <link name="base"/>
  <link name="tip">
    <inertial>
      <origin xyz="0.123456789012345 0 0.05" rpy="0 0 0"/>
      <mass value="1.23456789012345"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/>
    </inertial>
    <visual><geometry><mesh filename="original/fixture.stl"/></geometry></visual>
  </link>
  <joint name="axis" type="revolute">
    <parent link="base"/><child link="tip"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>'''

CORE = '''model:
  urdf_path: source.urdf
  gravity_scale: {axis: 1.0}
capability:
  admittance:
    calibration:
      torque_bias: {axis: 9.0}
      torque_threshold: {axis: 8.0}
      friction:
        enabled: true
        velocity_transition: 0.2
        positive_coulomb: {axis: 1.0}
        positive_viscous: {axis: 2.0}
        negative_coulomb: {axis: 3.0}
        negative_viscous: {axis: 4.0}
control:
  retained_setting: keep-me
'''


def core_bytes(validated, allow, scale=1.0):
    text = CORE.replace('gravity_scale: {axis: 1.0}', f'gravity_scale: {{axis: {scale}}}')
    if validated != 'no_policy':
        text += ('safety_policy:\n  fault_recovery:\n'
                 f'    allow_compliant_recovery: {str(allow).lower()}\n'
                 '    require_operator_request: true\n')
        if validated is not None:
            text += f'    gravity_model_validated: {str(validated).lower()} # model-specific trust\n'
    return text.encode()


def candidate_bytes(change=None):
    root = ET.fromstring(MODEL)
    # Exercise canonical XML/numeric rewrite and relocated visual resources.
    root.set('name', 'exported')
    root.find('.//mesh').set('filename', 'resources/000-fixture.stl')
    root.find('.//mass').set('value', '1.23456789012')
    root.find('.//inertial/origin').set('xyz', '0.123456789012 0.0 5e-2')
    if change == 'mass':
        root.find('.//mass').set('value', '1.24')
    elif change == 'com':
        root.find('.//inertial/origin').set('xyz', '0.123457789012 0 0.05')
    elif change == 'inertia':
        root.find('.//inertia').set('ixx', '0.011')
    elif change == 'joint_origin':
        root.find('./joint/origin').set('xyz', '0 0 0.201')
    elif change == 'joint_axis':
        root.find('./joint/axis').set('xyz', '1 0 0')
    elif change == 'joint_parent':
        root.find('./joint/parent').set('link', 'elsewhere')
    elif change == 'joint_name':
        root.find('./joint').set('name', 'other_axis')
    elif change == 'link_added':
        ET.SubElement(root, 'link', name='new-link')
    elif change == 'invalid_mass':
        root.find('.//mass').set('value', 'nan')
    elif change is not None:
        raise AssertionError(change)
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)


class GravityExportModelTrustTests(unittest.TestCase):
    def run_export(self, *, change=None, validated=True, allow=True,
                   static_pass=False, source='online', current_validated='unused',
                   scale=1.0, current_scale=None):
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            root = Path(temporary)
            folder = root / 'task'
            folder.mkdir()
            dest = root / 'export'
            dest.mkdir()
            model = folder / 'source.urdf'
            model.write_text(MODEL)
            core = folder / 'source-core.yaml'
            core.write_bytes(core_bytes(validated, allow, scale))
            candidate = folder / 'candidate-fixture.urdf'
            candidate.write_bytes(candidate_bytes(change))
            current = folder / 'current-core.yaml'
            current.write_bytes(core_bytes(current_validated if current_validated != 'unused' else validated,
                                           allow, scale if current_scale is None else current_scale))
            originals = {path: path.read_bytes() for path in folder.iterdir()}
            selected_core = core if current_validated == 'unused' else current
            selected_cfg = yaml.safe_load(selected_core.read_bytes())
            metadata = {'joint_names': ['axis'], 'task_id': 'fixture'}
            report_bytes = json.dumps({'static_pass': static_pass, 'candidate_run_id': 'fixture'}).encode()
            payload = {
                'candidate_source': 'recomputed-candidate.json' if source == 'offline' else 'online',
                'candidate_report_sha256': hashlib.sha256(report_bytes).hexdigest(),
                'static_pass': static_pass, 'friction_pass': static_pass,
                'friction_matches_gravity': static_pass,
                'residual': {'torque_bias': [.3], 'torque_threshold': [.06], 'friction': {
                    'velocity_transition': .03,
                    'positive_coulomb': [0.], 'positive_viscous': [0.],
                    'negative_coulomb': [0.], 'negative_viscous': [0.],
                }},
            }

            def write_candidate(staging):
                target = Path(staging) / 'candidate.urdf'
                target.write_bytes(candidate.read_bytes())
                return {'candidate_urdf': str(target)}

            def online_export(directory, staging):
                self.assertEqual(directory, folder)
                return write_candidate(staging)

            def worker(argv, **kwargs):
                details = write_candidate(argv[3])
                parent = Path(details['candidate_urdf']).parent
                (parent / 'recomputed-candidate.json').write_bytes(report_bytes)
                verification = {**details,
                    'candidate_report_sha256': hashlib.sha256(report_bytes).hexdigest(),
                    'candidate_urdf_sha256': hashlib.sha256(candidate.read_bytes()).hexdigest()}
                (parent / 'candidate-urdf-verification.json').write_text(json.dumps(verification))
                return SimpleNamespace(returncode=0, stdout=json.dumps(details), stderr='')

            stack.enter_context(mock.patch.object(exports, '_snapshot', return_value=(
                folder, metadata, model, core, yaml.safe_load(core.read_bytes()))))
            stack.enter_context(mock.patch.object(exports, '_gravity_merge_source', return_value=(
                selected_core, 'current_core' if selected_core == current else 'task_snapshot')))
            stack.enter_context(mock.patch.object(exports, 'build_gravity_correction_payload',
                return_value=(folder, metadata, {}, payload)))
            stack.enter_context(mock.patch.object(exports, 'export_candidate_urdf', side_effect=online_export))
            stack.enter_context(mock.patch('subprocess.run', side_effect=worker))
            exports._gravity_export({'directory': str(folder)}, dest, source)
            result = yaml.safe_load((dest / 'core.yaml').read_bytes())
            # Every input, including the selected live Core and candidate, is immutable.
            self.assertEqual({path: path.read_bytes() for path in folder.iterdir()}, originals)
            self.assertEqual((dest / 'robot.urdf').read_bytes(), candidate.read_bytes())
            self.assertEqual(result['control'], selected_cfg['control'])
            expected_policy = selected_cfg.get('safety_policy')
            if expected_policy is None:
                self.assertNotIn('safety_policy', result)
            else:
                expected_policy = json.loads(json.dumps(expected_policy))
                changed = change is not None or selected_cfg['model']['gravity_scale']['axis'] != 1.0
                if changed and expected_policy['fault_recovery'].get('gravity_model_validated') is True:
                    expected_policy['fault_recovery']['gravity_model_validated'] = False
                self.assertEqual(result['safety_policy'], expected_policy)
            return result

    def test_semantically_unchanged_model_keeps_existing_trust(self):
        for source in ('online', 'offline'):
            for allow in (True, False):
                with self.subTest(source=source, allow=allow):
                    self.run_export(source=source, allow=allow)

    def test_changed_model_revokes_trust_even_after_static_pass(self):
        for source in ('online', 'offline'):
            for static_pass in (True, False):
                for allow in (True, False):
                    with self.subTest(source=source, static_pass=static_pass, allow=allow):
                        self.run_export(change='mass', source=source, static_pass=static_pass, allow=allow)

    def test_false_validation_is_never_promoted(self):
        for change in (None, 'com'):
            for source in ('online', 'offline'):
                with self.subTest(change=change, source=source):
                    self.run_export(change=change, validated=False, static_pass=True, source=source)

    def test_missing_validation_or_policy_stays_absent(self):
        for validated in (None, 'no_policy'):
            for change in (None, 'com'):
                with self.subTest(validated=validated, change=change):
                    self.run_export(change=change, validated=validated, static_pass=True)

    def test_inertia_geometry_or_joint_mapping_changes_revoke_trust(self):
        for change in ('com', 'inertia', 'joint_origin', 'joint_axis', 'joint_parent',
                       'joint_name', 'link_added', 'invalid_mass'):
            with self.subTest(change=change):
                self.run_export(change=change)

    def test_selected_current_core_is_checked_instead_of_stale_snapshot(self):
        for snapshot, current in ((True, False), (False, True), (None, True), (True, None)):
            for change in (None, 'mass'):
                with self.subTest(snapshot=snapshot, current=current, change=change):
                    self.run_export(change=change, validated=snapshot, current_validated=current)

    def test_changed_gravity_scale_revokes_trust_even_with_same_urdf(self):
        for source in ('online', 'offline'):
            for allow in (True, False):
                for validated in (True, False, None):
                    with self.subTest(source=source, allow=allow, validated=validated):
                        self.run_export(scale=1.2, source=source, allow=allow,
                                        validated=validated, static_pass=True)

    def test_scale_comparison_uses_selected_current_core(self):
        for snapshot, current in ((1.2, 1.0), (1.0, 1.2)):
            with self.subTest(snapshot_scale=snapshot, current_scale=current):
                self.run_export(scale=snapshot, current_scale=current, current_validated=True)


if __name__ == '__main__':
    unittest.main()
