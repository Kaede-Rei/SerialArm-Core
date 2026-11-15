#!/usr/bin/env python3
"""Exercise purpose-specific configuration and selected saves without hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from bridge import parameter_save_selection, parking_configured, verify_machine_admission
from control_parameters import canonical_core, interaction_configuration
from gravity_recovery_validation import gravity_validation_context, gravity_validation_changes, gravity_trusted
from persistence import _runtime_text, preview, save, sha256_file
from profile_library import ProfileLibrary


def fixture():
    source = ROOT / 'src/robot_supports/robots/dm_arm/description/config/core/white.yaml'
    core = yaml.safe_load(source.read_text())
    core['model']['urdf_path'] = str((source.parent / core['model']['urdf_path']).resolve())
    core['model']['gravity_model_validated'] = False
    core['safety_policy']['fault_recovery'] = {'default_mode': 'rigid_hold', 'allow_drag_recovery': True,
        'require_operator_request': True, 'recovery_timeout_s': 30.}
    core['control']['controller'] = {'allow_full_cmd': False, 'rigid': core['control']['controller']['rigid']}
    return core


def runtime_values(core):
    names = core['model']['joint_names']
    admittance = core['capability']['admittance']
    result = {}
    for section in ('observer', 'calibration', 'controller'):
        for key, value in admittance.get(section, {}).items():
            result[key] = ([value[name] for name in names] if isinstance(value, dict) and set(value) == set(names)
                else {term: [column[name] for name in names] if isinstance(column, dict) else column
                      for term, column in value.items()} if isinstance(value, dict) else value)
    if 'joint_enabled' in admittance:
        result['joint_enabled'] = [admittance['joint_enabled'][name] for name in names]
    return result


class BackendParameters(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='serialarm-backend-parameters-')
        self.home = Path(self.directory.name)
        self.core_path = self.home / 'core.yaml'
        self.hardware_path = self.home / 'hardware.yaml'
        self.profile_path = self.home / 'robot_profiles.yaml'
        self.core = fixture()
        self.names = self.core['model']['joint_names']
        self.core_path.write_text(yaml.safe_dump(self.core, sort_keys=False))
        self.hardware_path.write_text('{}\n')
        self.entry = {'hardware': {'plugin': 'test_backend'}}
        self.profile_path.write_text(yaml.safe_dump({'profiles': {'review': self.entry}}))
        self.info = {'profile': 'review', 'profile_file': str(self.profile_path), 'hardware_plugin': 'test_backend',
            'resources': {'core': str(self.core_path), 'hardware': str(self.hardware_path),
                          'description': self.core['model']['urdf_path']},
            'checks': [{'name': key, 'ok': True} for key in ('core', 'hardware', 'description')],
            'fingerprint': 'current', 'write_enabled': True, 'devices': []}
        info = self.info
        class Inspector:
            def inspect(self, config): return info
            def validate_model_config(self, *args): return {'valid': True}
            def profiles(self, config): return {'profile_file': str(ROOT / 'src/robot_supports/profiles/config/robot_profiles.yaml')}
        self.inspector = Inspector()
        self.library = ProfileLibrary(self.home, self.inspector)

    def tearDown(self):
        self.directory.cleanup()

    def validate(self, core):
        self.library._validate_core_profile(core, {}, self.entry, self.core_path, self.core)

    def write(self, core):
        self.core_path.write_text(yaml.safe_dump(core, sort_keys=False))

    def test_basic_mit_does_not_invent_observation_or_other_gain_purposes(self):
        core = deepcopy(self.core); core.pop('capability', None)
        self.validate(core)
        self.assertNotIn('capability', canonical_core(core))
        self.write(core)
        ready = self.library._readiness({})
        self.assertTrue(ready['operation_readiness']['basic_control']['ready'])
        self.assertFalse(ready['operation_readiness']['drag']['ready'])
        self.assertFalse(ready['operation_readiness']['observation']['ready'])
        self.assertFalse(ready['operation_readiness']['admittance']['ready'])
        self.assertEqual(ready['verification_state'], 'pending')

    def test_observation_only_and_disabled_friction_need_no_mdk_or_coefficients(self):
        core = deepcopy(self.core)
        admittance = core['capability']['admittance']
        admittance.pop('controller'); admittance.pop('joint_enabled')
        admittance['calibration']['friction'] = {'enabled': False}
        self.validate(core)
        self.assertEqual(interaction_configuration(core), {'observation_configured': True, 'admittance_configured': False})
        core['capability']['admittance']['calibration']['friction']['enabled'] = True
        with self.assertRaisesRegex(ValueError, 'friction'):
            self.validate(core)

    def test_csp_rigid_first_run_needs_no_mit_groups_or_mdk(self):
        core = deepcopy(self.core)
        core['control']['runtime']['control_path'] = 'CSP'
        core['control'].pop('controller')
        core.pop('capability', None)
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.validate(core); self.write(core)
        ready = self.library._readiness({})
        self.assertTrue(ready['control_available'])
        self.assertEqual(ready['modes']['RIGID'], 'configured')
        self.assertEqual(ready['modes']['COMPLIANT'], 'unconfigured')
        self.assertEqual(ready['modes']['DRAG'], 'unconfigured')
        core['capability'] = deepcopy(self.core['capability'])
        core['capability']['admittance'].pop('controller')
        core['capability']['admittance'].pop('joint_enabled')
        self.write(core)
        self.assertTrue(self.library._readiness({})['operation_readiness']['gravity_verification']['ready'])

    def test_gravity_scope_follows_model_context_not_pd(self):
        before = self.library._dependencies(self.info, self.core, {})['gravity_verification']
        core = deepcopy(self.core)
        core['control']['controller']['rigid']['kp'][self.names[0]] += 1
        core['model']['gravity_model_validated'] = True
        self.assertEqual(before, self.library._dependencies(self.info, core, {})['gravity_verification'])
        core['model']['gravity_scale'][self.names[0]] = .9
        self.assertNotEqual(before, self.library._dependencies(self.info, core, {})['gravity_verification'])
        self.assertEqual(gravity_validation_changes(gravity_validation_context(self.core, self.core_path),
            gravity_validation_context(core, self.core_path)), ['gravity_scale'])

    def test_editor_uses_one_gravity_confirmation_and_preserves_it_across_pd_edits(self):
        editor = self.library.profile_editor_load({})
        editor['core']['model']['gravity_model_validated'] = True
        with self.assertRaisesRegex(ValueError, '确认重力'):
            self.library.profile_editor_save({}, editor)
        editor['gravity_confirmed'] = True
        editor = self.library.profile_editor_save({}, editor)
        self.assertTrue(gravity_trusted(editor['core']))
        editor['core']['control']['controller']['rigid']['kp'][self.names[0]] += 1
        editor = self.library.profile_editor_save({}, editor)
        self.assertTrue(gravity_trusted(editor['core']))
        self.assertFalse(editor['gravity_validation_reset'])
        editor['core']['model']['gravity_scale'][self.names[0]] = .9
        editor = self.library.profile_editor_save({}, editor)
        self.assertFalse(gravity_trusted(editor['core']))
        self.assertTrue(editor['gravity_validation_reset'])

    def test_editor_can_remove_unused_groups_for_csp_rigid(self):
        editor = self.library.profile_editor_load({})
        editor['core']['control']['runtime']['control_path'] = 'CSP'
        editor['core']['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        editor['core']['control'].pop('controller')
        editor['core'].pop('capability', None)
        saved = self.library.profile_editor_save({}, editor)
        self.assertNotIn('controller', saved['core']['control'])
        self.assertNotIn('capability', saved['core'])

    def test_only_bound_completed_gravity_task_and_final_acknowledgment_persist_trust(self):
        answers = dict.fromkeys(('distinct_poses', 'support', 'restored'), True)
        note = self.library.record_stage({}, 'gravity_verification', answers)
        self.assertFalse(gravity_trusted(yaml.safe_load(self.core_path.read_text())))
        self.assertFalse(note['requires_reconnect'])
        sample = {'robot_state': 'INACTIVE', 'gravity_verification': {'active': False, 'phase': 'completed', 'sample_count': 3},
            'joint_names': self.names, 'gravity_scale': [self.core['model']['gravity_scale'][name] for name in self.names],
            'model_feedforward_mode': self.core['control']['runtime']['model_feedforward_mode'], 'fault': {}, 'control_path': 'CSP'}
        class Native:
            def request(self, *args, **kwargs): return deepcopy(sample)
        self.library.record_stage({}, 'gravity_verification', answers, Native(), 'other-profile')
        self.assertFalse(gravity_trusted(yaml.safe_load(self.core_path.read_text())))
        self.library.record_stage({}, 'gravity_verification', {**answers, 'support': False}, Native(), self.info['fingerprint'])
        self.assertFalse(gravity_trusted(yaml.safe_load(self.core_path.read_text())))
        result = self.library.record_stage({}, 'gravity_verification', answers, Native(), self.info['fingerprint'])
        self.assertTrue(result['gravity_validation_saved']); self.assertTrue(result['requires_reconnect'])
        expected = deepcopy(self.core); expected['model']['gravity_model_validated'] = True
        self.assertEqual(yaml.safe_load(self.core_path.read_text()), expected)
        # Rigid CSP samples verify the model, not an unexecuted DRAG mode.
        metadata = yaml.safe_load(self.library._readiness_path(self.profile_path, 'review').read_text())
        self.assertNotIn('DRAG', metadata.get('modes', {}))

    def test_selected_save_never_copies_gain_gravity_or_other_axis_trials(self):
        runtime = runtime_values(self.core)
        runtime['mass'][0] += .5
        runtime['mass'][1] += 8
        runtime['execution_gains'] = {'rigid': {'kp': [999.] * len(self.names)}}
        runtime['hold_compensation'] = {'enabled': True}
        field = 'controller.mass.' + self.names[0]
        before = self.core_path.read_text()
        result = yaml.safe_load(_runtime_text(before, self.names, runtime, [0.] * len(self.names), fields=[field]))
        expected = deepcopy(self.core)
        expected['capability']['admittance']['controller']['mass'][self.names[0]] += .5
        self.assertEqual(result, expected)
        with self.assertRaisesRegex(ValueError, '选择'):
            _runtime_text(before, self.names, runtime)
        for forbidden in ('execution_gains', 'hold_compensation', 'model.gravity_scale', 'enabled'):
            with self.assertRaisesRegex(ValueError, '不可保存'):
                _runtime_text(before, self.names, runtime, fields=[forbidden])
        reviewed = preview(self.core_path, self.names, runtime, fields=[field])
        self.assertTrue(reviewed['changed'])
        saved = save(self.core_path, reviewed['sha256'], self.names, runtime, fields=[field])
        self.assertTrue(saved['changed'])
        self.assertEqual(yaml.safe_load(self.core_path.read_text()), expected)
        with self.assertRaisesRegex(ValueError, 'changed outside'):
            save(self.core_path, reviewed['sha256'], self.names, runtime, fields=[field])

    def test_new_mdk_save_requires_selected_complete_pair(self):
        runtime = runtime_values(self.core)
        core = deepcopy(self.core)
        core['capability']['admittance'].pop('controller')
        core['capability']['admittance'].pop('joint_enabled')
        text = yaml.safe_dump(core)
        with self.assertRaisesRegex(ValueError, 'joint_enabled|字段不完整'):
            _runtime_text(text, self.names, runtime, fields=['controller.mass'])
        result = yaml.safe_load(_runtime_text(text, self.names, runtime, fields=['controller', 'joint_enabled']))
        self.assertTrue(interaction_configuration(result)['admittance_configured'])

    def test_unselected_incomplete_trials_do_not_block_observation_or_single_axis_saves(self):
        runtime = runtime_values(self.core)
        runtime['mass'] = [None] * len(self.names)
        runtime['joint_enabled'] = 'unfinished draft'
        runtime['friction'] = None
        runtime['torque_bias'][0] += .1
        text = self.core_path.read_text()
        result = yaml.safe_load(_runtime_text(text, self.names, runtime,
            fields=['calibration.torque_bias.' + self.names[0]]))
        self.assertEqual(result['capability']['admittance']['calibration']['torque_bias'][self.names[0]], runtime['torque_bias'][0])
        runtime['mass'] = [2.] + [None] * (len(self.names) - 1)
        result = yaml.safe_load(_runtime_text(text, self.names, runtime, fields=['controller.mass.' + self.names[0]]))
        self.assertEqual(result['capability']['admittance']['controller']['mass'][self.names[0]], 2.)

    def test_draft_selection_is_offline_and_actual_requires_bound_session(self):
        class Native:
            def status(self): return {'state': 'idle'}
            def request(self, *args, **kwargs): raise AssertionError('offline draft touched native')
        draft = runtime_values(self.core)
        _, names, values, selection = parameter_save_selection(self.inspector, Native(), {}, None, None,
            {'source': 'draft', 'draft': draft, 'fields': ['controller.mass']})
        self.assertEqual(names, self.names); self.assertIs(values, draft)
        self.assertFalse(selection['runtime_applied']); self.assertFalse(selection['requires_reconnect'])
        with self.assertRaisesRegex(ValueError, '连接'):
            parameter_save_selection(self.inspector, Native(), {}, None, None, {'source': 'actual'})

    def test_actual_wire_boolean_flags_are_normalized_at_snapshot_boundary(self):
        info, names, values = self.info, self.names, runtime_values(self.core)
        values['joint_enabled'] = [1, 0] + [1] * (len(names) - 2)
        class Native:
            def status(self): return {'state': 'running'}
            def request(self, method, *args, **kwargs):
                return values if method == 'get_admittance' else {'joint_names': names}
        _, _, selected, metadata = parameter_save_selection(self.inspector, Native(), {}, {'profile': 'review'}, info['fingerprint'], {'source': 'actual'})
        self.assertIs(selected['joint_enabled'][0], True)
        self.assertIs(selected['joint_enabled'][1], False)
        self.assertTrue(metadata['runtime_applied'])

    def test_csp_bootstrap_positioning_is_limited_without_opening_other_actions(self):
        library = self.library
        class Admission:
            def _readiness(self, config): return {'info': {'write_enabled': True}, 'control_available': True,
                'modes_by_path': {'CSP': {'RIGID': 'configured', 'DRAG': 'configured'}}}
        status = {'robot_state': 'ACTIVE', 'control_path': 'CSP', 'impedance_mode': 'RIGID',
            'valid': True, 'feedback_age_ms': 0., 'sequence': 1, 'fault': {}, 'joint': {'pos': [0.]},
            'gravity_verification': {'active': True, 'phase': 'sampling'}}
        class Native:
            def request(self, method, *args, **kwargs): return deepcopy(status)
        params = {'positions': [.1], 'speed_scale': .8}
        verify_machine_admission(Admission(), {}, 'move_to', params, Native())
        self.assertEqual(params['speed_scale'], .1)
        with self.assertRaisesRegex(ValueError, '重力验证期间'):
            verify_machine_admission(Admission(), {}, 'set_impedance_mode', {'mode': 'DRAG'}, Native())
        status['gravity_verification'] = {'active': False}
        status['gravity_model_validated'] = False
        with self.assertRaisesRegex(ValueError, '重力补偿'):
            verify_machine_admission(Admission(), {}, 'set_impedance_mode', {'mode': 'DRAG'}, Native())

    def test_csp_parking_needs_only_rigid_while_mit_retains_compliant_requirement(self):
        readiness = {'info': {'write_enabled': True}, 'control_available': True,
            'modes_by_path': {'CSP': {'RIGID': 'configured'}, 'MIT': {'RIGID': 'configured'}}}
        status = {'robot_state': 'ACTIVE', 'control_path': 'CSP', 'valid': True,
            'feedback_age_ms': 0., 'sequence': 1, 'fault': {}}
        class Admission:
            def _readiness(self, config): return readiness
        class Native:
            def request(self, method, *args, **kwargs): return deepcopy(status)
        verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertTrue(parking_configured(readiness, 'CSP'))
        status['control_path'] = 'MIT'
        with self.assertRaisesRegex(ValueError, '自动停放'):
            verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertFalse(parking_configured(readiness, 'MIT'))
        readiness['modes_by_path']['MIT']['COMPLIANT'] = 'configured'
        verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertTrue(parking_configured(readiness, 'MIT'))
        status['control_path'] = 'CSP'
        readiness['modes_by_path']['CSP'].clear()
        with self.assertRaisesRegex(ValueError, '自动停放'):
            verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertFalse(parking_configured(readiness, 'CSP'))


if __name__ == '__main__':
    unittest.main()
