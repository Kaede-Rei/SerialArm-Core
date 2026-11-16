#!/usr/bin/env python3
"""Exercise purpose-specific configuration and selected saves without hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
import json

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from bridge import parameter_save_selection, parking_configured, verify_machine_admission
from control_parameters import canonical_core, interaction_configuration
from commissioning import FOUNDATION, PATH_STEPS, REVIEWS, stage_key, review_requirements
from gravity_recovery_validation import gravity_validation_context, gravity_validation_changes, gravity_trusted
from persistence import _runtime_text, preview, save, sha256_file
from profile_library import ProfileLibrary


def fixture():
    source = ROOT / 'src/robot_supports/robots/dm_arm/description/config/core/white.yaml'
    core = yaml.safe_load(source.read_text())
    core['model']['urdf_path'] = str((source.parent / core['model']['urdf_path']).resolve())
    core['model']['gravity_model_validated'] = False
    core['safety_policy']['fault_recovery'] = {'default_mode': 'rigid_hold', 'allow_drag_recovery': True,
        'require_operator_request': True, 'recovery_timeout_s': 30.,
        'velocity_tolerance': dict.fromkeys(core['model']['joint_names'], .05)}
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

    def test_numeric_contract_rejects_saved_but_unloadable_relationships(self):
        core = deepcopy(self.core)
        core['safety_policy'].update(cmd_vel_scale=.8, state_vel_scale=.2)
        with self.assertRaisesRegex(ValueError, 'state_vel_scale'):
            self.validate(core)
        core['safety_policy']['state_vel_scale'] = 1.2
        self.validate(core)
        core['safety_policy']['max_dt_s'] = .0001
        with self.assertRaisesRegex(ValueError, 'max_dt_s'):
            self.validate(core)
        core['control']['runtime']['ctrl_frequency_hz'] = .5
        core['safety_policy']['max_dt_s'] = 2.
        self.validate(core)

    def test_editor_constraints_match_save_ranges_and_discrete_direction(self):
        rules = self.library.profile_editor_load({})['field_constraints']
        self.assertEqual(rules['core.calibration.joints.joint1.direction']['choices'], [-1, 1])
        self.assertTrue(rules['core.safety_policy.cmd_vel_scale']['exclusive_min'])
        self.assertNotIn('max', rules['core.safety_policy.state_vel_scale'])
        core = deepcopy(self.core)
        core['calibration']['joints'][self.names[0]]['direction'] = 1.5
        with self.assertRaisesRegex(ValueError, 'direction'): self.validate(core)

    def test_driver_startup_settings_match_native_ranges(self):
        from parameter_contract import field_constraints, constraint_errors
        hardware = {'damiao': {'activation_retries': 1, 'startup_read_cycles': 1}}
        self.assertFalse(list(constraint_errors(self.core, hardware, self.entry)))
        rules = field_constraints(self.core, hardware, self.entry)
        self.assertNotIn('hardware.damiao.stop_kp', rules)
        for key, value in (('activation_retries', 0), ('startup_read_cycles', 0), ('startup_read_cycles', 1.5)):
            candidate = deepcopy(hardware)
            candidate['damiao'][key] = value
            self.assertIn('hardware.damiao.' + key,
                          [path for path, _ in constraint_errors(self.core, candidate, self.entry)])

    def test_parking_permission_defaults_and_independent_paths(self):
        editor = self.library.profile_editor_load({})
        plan = editor['core']['shutdown']['paths']['MIT']
        plan.update(enabled=False, park_pos={})
        saved = self.library.profile_editor_save({}, editor)
        self.assertFalse(self.library._readiness({})['operation_readiness']['parking']['allowed'])
        saved['core']['shutdown']['paths']['MIT']['enabled'] = True
        saved = self.library.profile_editor_save({}, saved)
        self.assertEqual(saved['core']['shutdown']['paths']['MIT']['park_pos'], dict.fromkeys(self.names, 0.))
        self.assertTrue(self.library._readiness({})['operation_readiness']['parking']['allowed'])
        plan = saved['core']['shutdown']['paths']['MIT']
        plan['position_tolerance'] = {}
        with self.assertRaisesRegex(ValueError, '停放需要完整'): self.library.profile_editor_save({}, saved)

    def test_disabled_parking_does_not_require_plan_or_valid_zero(self):
        core = deepcopy(self.core)
        core['shutdown']['paths']['MIT'] = {'enabled': False, 'park_pos': dict.fromkeys(self.names, 100.)}
        self.validate(core)
        core['shutdown']['paths']['MIT']['enabled'] = True
        with self.assertRaises(ValueError): self.validate(core)

    def test_parking_receipt_scopes_are_path_specific_and_ignore_switches(self):
        scopes = self.library._dependencies(self.info, self.core, {})
        self.assertNotEqual(scopes['MIT:parking_validation'], scopes['CSP:parking_validation'])
        core = deepcopy(self.core)
        plan = core['shutdown']['paths']['MIT']
        plan['enabled'] = not plan['enabled']
        self.assertEqual(scopes['MIT:parking_validation'], self.library._dependencies(self.info, core, {})['MIT:parking_validation'])
        plan['park_pos'][self.names[0]] += .1
        edited = self.library._dependencies(self.info, core, {})
        self.assertNotEqual(scopes['MIT:parking_validation'], edited['MIT:parking_validation'])
        self.assertEqual(scopes['CSP:parking_validation'], edited['CSP:parking_validation'])
        plan['park_pos'][self.names[0]] -= .1
        plan['strategy'] = 'COMPLIANT_THEN_RIGID'
        self.assertNotEqual(scopes['MIT:parking_validation'], self.library._dependencies(self.info, core, {})['MIT:parking_validation'])

    def test_response_tolerance_is_independent_of_parking(self):
        core, window, native = self.response_fixture()
        from interaction_response import validate_admittance_response
        before = validate_admittance_response(window, core)
        core['shutdown']['paths']['MIT']['velocity_tolerance'] = dict.fromkeys(self.names, 100.)
        self.assertEqual(before, validate_admittance_response(window, core))
        core['capability']['admittance']['validation']['velocity_tolerance'][self.names[0]] = 0
        self.assertFalse(validate_admittance_response(window, core)['validated'])

    def test_permission_is_single_source_and_receipts_remain_independent(self):
        reviewed = self.library.record_stage({}, 'parking_validation',
            dict.fromkeys((key for key, _ in REVIEWS['parking_validation'][2]), True))
        self.assertNotIn('exit_parking', reviewed['operation_readiness'])
        self.assertTrue(reviewed['operation_readiness']['parking']['allowed'])
        plans = reviewed['configuration_summary']['parking_by_path']
        self.assertTrue(plans['MIT']['validated'])
        self.assertFalse(plans['CSP']['validated'])
        self.assertNotIn('park_on_exit', plans['MIT'])
        editor = self.library.profile_editor_load({})
        editor['core']['shutdown']['paths']['MIT']['enabled'] = False
        self.library.profile_editor_save({}, editor)
        reviewed = self.library._readiness({})
        self.assertFalse(reviewed['operation_readiness']['parking']['allowed'])
        self.assertTrue(reviewed['configuration_summary']['parking_by_path']['MIT']['validated'])
        editor = self.library.profile_editor_load({})
        plan = editor['core']['shutdown']['paths']['MIT']
        plan['enabled'] = True
        plan['park_pos'][self.names[0]] += .1
        self.library.profile_editor_save({}, editor)
        reviewed = self.library._readiness({})
        self.assertTrue(reviewed['operation_readiness']['parking']['allowed'])
        self.assertFalse(reviewed['configuration_summary']['parking_by_path']['MIT']['validated'])
        editor = self.library.profile_editor_load({})
        editor['core']['shutdown']['paths']['MIT']['park_on_exit'] = True
        with self.assertRaisesRegex(ValueError, '未知停放字段'):
            self.library.profile_editor_save({}, editor)

    def test_csp_flexible_parking_requires_quality_and_rigid_does_not(self):
        core = deepcopy(self.core)
        core['control']['runtime']['control_path'] = 'CSP'
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        plan = core['shutdown']['paths']['CSP']
        plan.update(enabled=True, strategy='RIGID')
        self.write(core)
        self.assertTrue(self.library._readiness({})['operation_readiness']['parking']['allowed'])
        plan['strategy'] = 'COMPLIANT_THEN_RIGID'
        self.write(core)
        ready = self.library._readiness({})
        self.assertFalse(ready['operation_readiness']['parking']['allowed'])
        self.assertIn('验收', ready['operation_readiness']['parking']['reason'])
        core['capability']['admittance']['validation'].update(observation_validated=True, response_validated=True)
        self.write(core)
        self.assertTrue(self.library._readiness({})['operation_readiness']['parking']['allowed'])

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
        with self.assertRaisesRegex(ValueError, '独立外力'):
            verify_machine_admission(Admission(), {}, 'set_impedance_mode', {'mode': 'DRAG'}, Native())

    def test_both_parking_paths_use_rigid_without_compliant_prerequisite(self):
        readiness = {'info': {'write_enabled': True}, 'control_available': True,
            'configuration_summary': {'parking_by_path': {p: {'enabled': True, 'configured': True, 'strategy': 'RIGID'} for p in ('MIT', 'CSP')}},
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
        verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertTrue(parking_configured(readiness, 'MIT'))
        readiness['modes_by_path']['MIT']['COMPLIANT'] = 'configured'
        verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertTrue(parking_configured(readiness, 'MIT'))
        status['control_path'] = 'CSP'
        readiness['modes_by_path']['CSP'].clear()
        readiness['configuration_summary']['parking_by_path']['CSP']['configured'] = False
        with self.assertRaisesRegex(ValueError, '停放目标'):
            verify_machine_admission(Admission(), {}, 'park', {}, Native())
        self.assertFalse(parking_configured(readiness, 'CSP'))

    def test_saved_configuration_summary_survives_new_library_without_editor_or_session(self):
        ready = ProfileLibrary(self.home, self.inspector)._readiness({})
        self.assertTrue(ready['configuration_summary']['hold'])
        self.assertTrue(ready['configuration_summary']['observation'])
        self.assertTrue(ready['configuration_summary']['response'])
        self.assertEqual(ready['configuration_summary']['source'], 'saved_profile')
        self.assertEqual([group['id'] for group in ready['commissioning']['groups']], ['foundation', 'path', 'optional'])
        self.assertEqual([group['label'] for group in ready['commissioning']['groups']], ['共同基础', '路径专项', '可选提升'])
        self.assertEqual(set(ready['commissioning']['paths']), {'MIT', 'CSP'})
        self.assertEqual(len(ready['commissioning']['groups'][0]['steps']), 4)
        self.assertTrue(all(len(path['steps']) == 3 for path in ready['commissioning']['paths'].values()))
        self.assertFalse(ready['commissioning']['completed'])
        self.assertTrue(all(step['source']['live']['status'] == 'session_required'
                            for group in ready['commissioning']['groups'] for step in group['steps']))

    def task(self, task_id, ready=None, path=None):
        ready = ready or self.library._readiness({})
        groups = ready['commissioning']['groups']
        items = ready['commissioning']['paths'][path]['steps'] if path else [item for group in groups for item in group['steps']]
        return next(item for item in items if item['id'] == task_id)

    def task_answers(self, task_id):
        ready = self.library._readiness({})
        return dict.fromkeys((key for key, _ in ready['review_requirements'][task_id]['keys']), True)

    def test_manual_task_saves_partial_then_completes_with_one_form_without_measurement(self):
        answers = self.task_answers('model_connection')
        first = next(iter(answers))
        partial = self.library.record_stage({}, 'model_connection', {first: True, 'note': '已核对模型，尚未连接'})
        self.assertFalse(self.task('model_connection', partial)['completed'])
        self.assertEqual(partial['inspection_result']['status'], 'partial')
        self.assertFalse(partial['requires_reconnect'])
        completed = self.library.record_stage({}, 'model_connection', answers)
        task = self.task('model_connection', completed)
        self.assertTrue(task['completed'])
        self.assertFalse(any(item['kind'] == 'measured' for item in task['requirements']))
        self.assertTrue(completed['inspection_result']['completed'])
        restarted = ProfileLibrary(self.home, self.inspector)._readiness({})
        self.assertTrue(self.task('model_connection', restarted)['completed'])
        self.assertEqual(self.task('model_connection', restarted)['source']['live']['status'], 'session_required')
        saved = yaml.safe_load(self.library._readiness_path(self.profile_path, 'review').read_text())
        self.assertIn('model_check', saved['stages']); self.assertIn('bringup', saved['stages'])

    def test_startup_checks_are_saved_per_axis_and_legacy_all_axis_check_is_retained(self):
        answers = self.task_answers('startup_hold')
        first = 'joint:' + self.names[0]
        partial = self.library.record_stage({}, 'startup_hold', {'supported': True, first: True})
        task = self.task('startup_hold', partial)
        axes = [item for item in task['requirements'] if item.get('axis')]
        self.assertEqual(len(axes), len(self.names))
        self.assertEqual(axes[0]['status'], 'recorded')
        self.assertTrue(all(item['status'] == 'pending' for item in axes[1:]))
        self.assertIn(self.names[1], ' '.join(task['blockers']))
        self.assertFalse(task['completed'])
        complete = self.library.record_stage({}, 'startup_hold', answers)
        self.assertTrue(self.task('startup_hold', complete)['completed'])
        # A prior explicit all-axis inspection is stronger than an empty new form.
        self.library.record_stage({}, 'startup_hold', dict.fromkeys((key for key, _ in REVIEWS['startup_hold'][2]), True))
        saved_path = self.library._readiness_path(self.profile_path, 'review')
        saved = yaml.safe_load(saved_path.read_text())
        old = saved['stages']['MIT:startup_hold']['observations']
        for key in list(old):
            if key.startswith('joint:'): old.pop(key)
        saved_path.write_text(yaml.safe_dump(saved))
        reloaded = ProfileLibrary(self.home, self.inspector)._readiness({})
        self.assertTrue(self.task('startup_hold', reloaded)['completed'])

    def test_manual_task_rechecks_only_changed_dependencies(self):
        for task_id in ('model_connection', 'mapping_safety', 'startup_hold'):
            self.library.record_stage({}, task_id, self.task_answers(task_id))
        editor = self.library.profile_editor_load({})
        editor['core']['control']['controller']['rigid']['kp'][self.names[0]] += 1
        self.library.profile_editor_save({}, editor)
        ready = self.library._readiness({})
        self.assertTrue(self.task('model_connection', ready)['completed'])
        self.assertFalse(self.task('startup_hold', ready)['completed'])
        self.assertEqual(self.task('startup_hold', ready)['evidence_status'], 'stale')
        self.assertIn('待核对关节', ' '.join(self.task('startup_hold', ready)['blockers']))

    def test_missing_mit_gains_still_allows_disabled_hardware_observation(self):
        core = deepcopy(self.core)
        core['control'].pop('controller')
        core.pop('capability', None)
        self.validate(core); self.write(core)
        ready = self.library._readiness({})
        self.assertTrue(ready['connection_readiness']['hardware_connectable'])
        self.assertFalse(ready['configuration_summary']['hold'])
        self.assertFalse(ready['operation_readiness']['basic_control']['ready'])

    def test_startup_note_preserves_gain_receipt_and_is_visible_independently(self):
        scopes = self.library._dependencies(self.info, self.core, {})
        metadata = {'profile': 'review', 'modes': {'RIGID': {'kind': 'measured_gain', 'result': 'passed',
                                                            'scope': scopes['startup_hold']}}}
        path = self.library._readiness_path(self.profile_path, 'review')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(metadata))
        answers = dict.fromkeys((key for key, _ in REVIEWS['startup_hold'][2]), True)
        ready = self.library.record_stage({}, 'startup_hold', answers)
        stored = yaml.safe_load(path.read_text())
        self.assertEqual(stored['modes'], metadata['modes'])
        self.assertEqual(ready['checks']['startup_hold'], 'recorded')
        step = next(step for group in ready['commissioning']['groups'] for step in group['steps'] if step['id'] == 'startup_hold')
        self.assertEqual(step['source']['historical']['status'], 'recorded')
        self.assertEqual(step['source']['measurement']['status'], 'pending')

    def test_required_selected_path_can_complete_without_optional_calibration(self):
        core = deepcopy(self.core)
        core['model']['gravity_model_validated'] = True
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        core['control']['controller']['compliant'] = {'kp': dict.fromkeys(self.names, 1.), 'kd': dict.fromkeys(self.names, .1)}
        core['control']['controller']['drag'] = {'kp': dict.fromkeys(self.names, 0.), 'kd': dict.fromkeys(self.names, .1)}
        self.write(core)
        for stage in (*FOUNDATION, *PATH_STEPS['MIT']):
            self.library.record_stage({}, stage, dict.fromkeys((key for key, _ in review_requirements('MIT', self.names)[stage]['keys']), True))
        ready = ProfileLibrary(self.home, self.inspector)._readiness({})
        self.assertTrue(ready['commissioning']['completed'])
        self.assertEqual(ready['verification_state'], 'verified')
        self.assertFalse(ready['commissioning']['groups'][2]['completed'])
        self.assertTrue(ready['commissioning']['paths']['MIT']['completed'])
        self.assertFalse(ready['commissioning']['paths']['CSP']['completed'])
        core['control']['runtime']['control_path'] = 'CSP'
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.write(core)
        csp = self.library._readiness({})
        self.assertFalse(csp['commissioning']['completed'])
        self.assertEqual(csp['checks']['model_check'], 'recorded')
        self.assertEqual(csp['checks']['startup_hold'], 'pending')
        self.assertTrue(csp['commissioning']['paths']['MIT']['completed'])
        self.assertTrue(all(step['view_only'] for step in csp['commissioning']['paths']['MIT']['steps']))
        self.assertEqual(csp['commissioning']['selected_path'], 'CSP')
        stored = yaml.safe_load(self.library._readiness_path(self.profile_path, 'review').read_text())
        self.assertIn('MIT:startup_hold', stored['stages'])

    def test_path_views_use_their_own_configuration_without_switching_saved_path(self):
        self.library.record_stage({}, 'upper_layer_validation',
            dict.fromkeys((key for key, _ in REVIEWS['upper_layer_validation'][2]), True))
        mit = self.library._readiness({})
        mit_steps = {step['id']: step for step in mit['commissioning']['paths']['MIT']['steps']}
        csp_steps = {step['id']: step for step in mit['commissioning']['paths']['CSP']['steps']}
        self.assertTrue(mit_steps['mit_tracking_validation']['configuration_ready'])
        self.assertFalse(csp_steps['observation_validation']['configuration_ready'])
        self.assertFalse(csp_steps['admittance_response']['configuration_ready'])
        self.assertEqual(mit_steps['mit_interaction']['source']['historical']['records'][-1]['status'], 'recorded')
        self.assertEqual(csp_steps['csp_interaction']['source']['historical']['status'], 'pending')
        core = deepcopy(self.core)
        core['control']['runtime']['control_path'] = 'CSP'
        core['control'].pop('controller')
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.write(core)
        saved = self.core_path.read_bytes()
        csp = self.library._readiness({})
        mit_steps = {step['id']: step for step in csp['commissioning']['paths']['MIT']['steps']}
        csp_steps = {step['id']: step for step in csp['commissioning']['paths']['CSP']['steps']}
        self.assertFalse(mit_steps['mit_tracking_validation']['configuration_ready'])
        self.assertFalse(mit_steps['mit_interaction']['configuration_ready'])
        self.assertTrue(csp_steps['observation_validation']['configuration_ready'])
        self.assertTrue(csp_steps['admittance_response']['configuration_ready'])
        self.assertTrue(mit_steps['mit_tracking_validation']['view_only'])
        self.assertFalse(csp_steps['observation_validation']['view_only'])
        self.assertEqual(csp['commissioning']['groups'][1]['steps'], csp['commissioning']['paths']['CSP']['steps'])
        self.assertEqual(csp['commissioning']['selected_path'], 'CSP')
        self.assertEqual(self.core_path.read_bytes(), saved)

    def test_dependency_changes_invalidate_only_relevant_receipts(self):
        before = self.library._dependencies(self.info, self.core, {})
        core = deepcopy(self.core)
        core['capability']['admittance']['controller']['mass'][self.names[0]] += .1
        after = self.library._dependencies(self.info, core, {})
        for stage in ('model_check', 'mapping_calibration', 'CSP:observation_validation', 'MIT:startup_hold'):
            self.assertEqual(before[stage], after[stage])
        self.assertNotEqual(before['CSP:admittance_response'], after['CSP:admittance_response'])
        core['capability']['admittance']['validation'].update(observation_validated=True, response_validated=True)
        self.assertEqual(after['CSP:admittance_response'], self.library._dependencies(self.info, core, {})['CSP:admittance_response'])

    def test_manual_notes_cannot_grant_observation_quality(self):
        ready = self.library.record_stage({}, 'observation_validation', dict.fromkeys((key for key, _ in REVIEWS['observation_validation'][2]), True))
        step = next(step for step in ready['commissioning']['paths']['CSP']['steps'] if step['id'] == 'observation_validation')
        self.assertEqual(ready['checks']['observation_validation'], 'recorded')
        self.assertFalse(ready['operation_readiness']['observation']['validated'])
        self.assertFalse(yaml.safe_load(self.core_path.read_text())['capability']['admittance']['validation'].get('observation_validated'))
        self.assertFalse(step['completed'])
        editor = self.library.profile_editor_load({})
        editor['core']['capability']['admittance']['validation'].update(observation_validated=True)
        with self.assertRaisesRegex(ValueError, '独立测量'):
            self.library.profile_editor_save({}, editor)

    def test_runtime_parameter_save_revokes_only_affected_quality(self):
        core = deepcopy(self.core)
        core['capability']['admittance']['validation'].update(observation_validated=True, response_validated=True)
        runtime = runtime_values(core)
        runtime['mass'][0] += .1
        updated = yaml.safe_load(_runtime_text(yaml.safe_dump(core), self.names, runtime, fields=['controller.mass']))
        self.assertEqual(updated['capability']['admittance']['validation'], {**core['capability']['admittance']['validation'], 'response_validated': False})
        runtime['momentum_gain'][0] += 1
        updated = yaml.safe_load(_runtime_text(yaml.safe_dump(core), self.names, runtime, fields=['observer.momentum_gain']))
        self.assertEqual(updated['capability']['admittance']['validation'], {**core['capability']['admittance']['validation'], 'observation_validated': False, 'response_validated': False})

    def observation_fixture(self):
        core = deepcopy(self.core)
        core['control']['runtime']['control_path'] = 'CSP'
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.write(core)
        validation = {'independent': True, 'static_window_s': 15., 'pose_count': 5, 'friction_required': False,
                      'validated': True, 'pass': [1] * len(self.names), 'scope_config': runtime_values(core)}
        status = {'hardware_connected': True, 'control_path': 'CSP', 'joint_names': self.names,
            'gravity_scale': [core['model']['gravity_scale'][name] for name in self.names],
            'calibration': {'kind': 'none', 'completed_kind': 'validation', 'phase': 'complete', 'validation': validation}}
        interval = core['safety_policy']['state_timeout_s'] / 2
        raw = {'static_window_s': 15., 'poses': [{'samples': [dict(time_s=frame * interval,
            position=[float(index)] * len(self.names), gravity=[0.] * len(self.names), measured_torque=[0.] * len(self.names))
            for frame in range(int(15. / interval) + 1)]} for index in range(5)]}
        class Native:
            def request(self, method, *args, **kwargs): return deepcopy(raw if method == 'calibration_samples' else status)
        return Native(), status

    def test_observation_native_receipt_needs_confirmation_and_can_be_completed_after_restart(self):
        native, _ = self.observation_fixture()
        measured = self.library.record_observation_measurement({}, native, self.info['fingerprint'])
        self.assertTrue(measured['measurement']['summary']['validated'])
        self.assertFalse(measured['observation_validated'])
        self.assertFalse(measured['requires_reconnect'])
        self.assertFalse(self.task('observation_validation', measured)['completed'])
        self.library = ProfileLibrary(self.home, self.inspector)
        answers = self.task_answers('observation_validation')
        incomplete = self.library.record_stage({}, 'observation_validation', {**answers, 'observation_validation__direction': False})
        self.assertFalse(self.task('observation_validation', incomplete)['completed'])
        self.assertFalse(incomplete['operation_readiness']['observation']['validated'])
        complete = self.library.record_stage({}, 'observation_validation', answers)
        self.assertTrue(self.task('observation_validation', complete)['completed'])
        self.assertTrue(complete['requires_reconnect'])
        self.assertEqual(complete['inspection_result']['permission_updates'], {'observation_validated': True})
        self.assertTrue(complete['operation_readiness']['observation']['validated'])
        # Removing a required human confirmation revokes both runtime permissions.
        core = yaml.safe_load(self.core_path.read_text())
        core['capability']['admittance']['validation']['response_validated'] = True
        self.write(core)
        revoked = self.library.record_stage({}, 'observation_validation', {**answers, 'observation_validation__contact_free': False})
        self.assertFalse(self.task('observation_validation', revoked)['completed'])
        self.assertTrue(revoked['requires_reconnect'])
        self.assertEqual(revoked['inspection_result']['permission_updates'], {'observation_validated': False, 'response_validated': False})

    def test_observation_failure_explains_affected_axis_and_checkboxes_cannot_override_it(self):
        native, status = self.observation_fixture()
        status['calibration']['validation']['pass'][2] = 0
        status['calibration']['validation']['validated'] = False
        self.library.record_observation_measurement({}, native, self.info['fingerprint'])
        ready = self.library.record_stage({}, 'observation_validation', self.task_answers('observation_validation'))
        task = self.task('observation_validation', ready)
        measured = next(item for item in task['requirements'] if item['kind'] == 'measured')
        self.assertFalse(task['completed'])
        self.assertEqual(measured['failed_axes'], [self.names[2]])
        self.assertIn(self.names[2], measured['reason'])
        self.assertFalse(ready['operation_readiness']['observation']['validated'])

    def test_measured_response_cannot_be_completed_by_manual_checklist(self):
        self.response_fixture()
        ready = self.library.record_stage({}, 'admittance_response', dict.fromkeys((key for key, _ in REVIEWS['admittance_response'][2]), True))
        task = self.task('admittance_response', ready)
        self.assertFalse(task['completed'])
        self.assertIsNone(task['record_stage'])
        self.assertEqual(sum(item['kind'] == 'measured' for item in task['requirements']), 1)
        self.assertEqual(task['completion_kind'], 'measured')

    def test_independent_observation_receipt_survives_restart_and_rejects_fit_only(self):
        self.core['control']['runtime']['control_path'] = 'CSP'
        self.core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.write(self.core)
        values = runtime_values(self.core)
        validation = {'independent': True, 'static_window_s': 15., 'pose_count': 5,
                      'friction_required': False, 'validated': True, 'pass': [1] * len(self.names), 'scope_config': values}
        status = {'hardware_connected': True, 'control_path': 'CSP', 'joint_names': self.names,
            'gravity_scale': [self.core['model']['gravity_scale'][name] for name in self.names],
            'calibration': {'kind': 'none', 'completed_kind': 'validation', 'phase': 'complete', 'validation': validation}}
        sample_interval = self.core['safety_policy']['state_timeout_s'] / 2
        raw = {'static_window_s': 15., 'poses': [{'samples': [dict(time_s=frame * sample_interval, position=[float(i)] * len(self.names),
                    gravity=[0.] * len(self.names), measured_torque=[0.] * len(self.names))
                    for frame in range(int(15. / sample_interval) + 1)]} for i in range(5)]}
        class Native:
            def request(self, method, *args, **kwargs): return deepcopy(raw if method == 'calibration_samples' else status)
        self.library.record_stage({}, 'observation_validation', dict.fromkeys((key for key, _ in REVIEWS['observation_validation'][2]), True))
        ready = self.library.record_observation_measurement({}, Native(), self.info['fingerprint'])
        self.assertTrue(ready['measurement']['summary']['validated'])
        self.assertTrue(yaml.safe_load(self.core_path.read_text())['capability']['admittance']['validation']['observation_validated'])
        reloaded = ProfileLibrary(self.home, self.inspector)._readiness({})
        measurement = next(step for group in reloaded['commissioning']['groups'] for step in group['steps'] if step['id'] == 'observation_validation')
        self.assertEqual(measurement['source']['measurement']['status'], 'verified')
        self.assertEqual(measurement['source']['live']['status'], 'session_required')
        saved = yaml.safe_load(self.library._readiness_path(self.profile_path, 'review').read_text())
        self.assertTrue(Path(saved['measurements']['CSP:observation_validation']['artifact_path']).is_file())
        # Duration and a native pass alone cannot qualify a window containing
        # long holes; each adjacency must satisfy the saved feedback timeout.
        for pose in raw['poses']:
            pose['samples'] = [pose['samples'][index] for index in (0, len(pose['samples']) // 2, len(pose['samples']) - 1)]
        gapped = self.library.record_observation_measurement({}, Native(), self.info['fingerprint'])
        self.assertFalse(gapped['measurement']['summary']['validated'])
        self.assertFalse(yaml.safe_load(self.core_path.read_text())['capability']['admittance'].get('validation', {}).get('observation_validated', False))
        status['calibration']['kind'] = 'static'
        status['calibration']['completed_kind'] = 'static'
        with self.assertRaisesRegex(ValueError, '独立静态验证'):
            self.library.record_observation_measurement({}, Native(), self.info['fingerprint'])

    def test_short_raw_observation_window_never_grants_quality(self):
        self.core['control']['runtime']['control_path'] = 'CSP'
        self.core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        self.write(self.core)
        status = {'hardware_connected': True, 'control_path': 'CSP', 'calibration': {'kind': 'validation', 'phase': 'complete',
            'validation': {'independent': True, 'static_window_s': 15., 'pose_count': 5, 'validated': True,
                           'pass': [1] * len(self.names), 'scope_config': runtime_values(self.core)}}}
        class Native:
            def request(self, method, *args, **kwargs): return {'poses': []} if method == 'calibration_samples' else deepcopy(status)
        ready = self.library.record_observation_measurement({}, Native(), self.info['fingerprint'])
        self.assertFalse(ready['measurement']['summary']['validated'])
        self.assertFalse(yaml.safe_load(self.core_path.read_text())['capability']['admittance'].get('validation', {}).get('observation_validated', False))

    def response_fixture(self, moved_axis=0, active_axes=(0,), *, core=None):
        core = deepcopy(core or self.core)
        core['control']['runtime']['control_path'] = 'CSP'
        core['safety_policy']['max_tracking_error'] = dict.fromkeys(self.names, .2)
        admittance = core['capability']['admittance']
        admittance['validation'].update(observation_validated=True, response_validated=False)
        admittance['joint_enabled'] = {name: i in active_axes for i, name in enumerate(self.names)}
        self.write(core)
        count = len(self.names)
        samples = []
        for index in range(41):
            time_s = index / 10
            pos = [0.] * count
            if .2 <= time_s <= 1.2: pos[moved_axis] = .05
            samples.append(dict(time_s=time_s, sequence=index + 1, pos=pos, vel=[0.] * count,
                nominal=[0.] * count, accepted=list(pos), observer_valid=True, new_measurement=True,
                reference_limit_flags=[], torque_limit_flags=[], reference_limited=False, torque_limited=False))
        window = dict(profile='review', core=str(self.core_path), behavior='CSP', impedance_mode='COMPLIANT',
            source='passive_native_cycle_snapshots', recording=False, session_id='test-session',
            gravity_scale=[core['model']['gravity_scale'][name] for name in self.names],
            joint_names=self.names, joint_units=['rad'] * count, parameters=runtime_values(core),
            samples=samples, release_time_s=1., return_tolerance=[.01] * count)
        events = [dict(event='telemetry', data=dict(sequence=sample['sequence'], monotonic_ns=sample['time_s'] * 1e9,
            valid=True, robot_state='ACTIVE', write_enabled=True, hardware_connected=True,
            gravity_scale=window['gravity_scale'],
            control_path='CSP', impedance_mode='COMPLIANT', observer_valid=True, new_measurement=True,
            reference_limited=[], torque_limited=[], joint=dict(pos=sample['pos'], vel=sample['vel'],
                nominal_ref_pos=sample['nominal'], accepted_ref_pos=sample['accepted']))) for sample in deepcopy(samples)]
        class Native:
            def telemetry(self): return deepcopy(events)
            def status(self): return {'state': 'running', 'session_id': 'test-session'}
            def request(self, method, *args, **kwargs): return {}
        return core, window, Native()

    def record_response(self, window, native, name='response.jsonl'):
        artifact = self.home / name
        artifact.write_text(json.dumps(window))
        return self.library.record_response_measurement({}, window, artifact, native=native,
                                                        native_fingerprint=self.info['fingerprint'])

    def test_finite_stiffness_response_requires_real_motion_and_survives_restart(self):
        core, window, native = self.response_fixture()
        result = self.record_response(window, native)
        self.assertTrue(result['response_validated'])
        self.assertEqual(result['receipt']['summary']['passed_axes'], self.names[:1])
        saved = yaml.safe_load(self.core_path.read_text())
        self.assertTrue(saved['capability']['admittance']['validation']['response_validated'])
        reloaded = ProfileLibrary(self.home, self.inspector)._readiness({})
        self.assertEqual(len(reloaded['response_records']), 1)
        self.assertEqual(reloaded['response_records'][0]['status'], 'verified')
        response = next(step for group in reloaded['commissioning']['groups'] for step in group['steps'] if step['id'] == 'admittance_response')
        self.assertTrue(response['completed'])
        self.assertEqual(response['source']['measurement']['status'], 'verified')
        saved = yaml.safe_load(self.library._readiness_path(self.profile_path, 'review').read_text())
        self.assertIn('CSP:admittance_response', saved['measurements'])
        self.assertNotIn('CSP:admittance_release', saved['measurements'])
        self.assertEqual(sum(item['kind'] == 'measured' for item in response['requirements']), 1)

    def test_no_motion_or_reference_reanchoring_never_qualifies_response(self):
        _, window, native = self.response_fixture(moved_axis=1)
        self.assertFalse(self.record_response(window, native)['response_validated'])
        _, window, native = self.response_fixture()
        # Binding succeeds for a real reference jump too; analysis must reject
        # the jump rather than accepting a small final error around a new target.
        for sample in window['samples'][20:]: sample['nominal'][0] = .1
        result = self.record_response(window, native, 'reanchored.jsonl')
        self.assertFalse(result['response_validated'])
        self.assertIn('不匹配', result['receipt']['summary']['binding_error'])
        from interaction_response import validate_admittance_response
        measured = validate_admittance_response(window, yaml.safe_load(self.core_path.read_text()))
        self.assertFalse(measured['eligible'])
        self.assertFalse(measured['return_verification']['target_unchanged'])

    def test_response_coverage_accumulates_only_across_same_parameter_scope(self):
        _, window, native = self.response_fixture(active_axes=(0, 1))
        first = self.record_response(window, native, 'axis-one.jsonl')
        self.assertFalse(first['response_validated'])
        self.assertEqual(first['receipt']['result'], 'recorded')
        _, window, native = self.response_fixture(moved_axis=1, active_axes=(0, 1))
        second = self.record_response(window, native, 'axis-two.jsonl')
        self.assertTrue(second['response_validated'])
        self.assertEqual(second['receipt']['summary']['passed_axes'], sorted(self.names[:2]))
        core = yaml.safe_load(self.core_path.read_text())
        core['capability']['admittance']['controller']['mass'][self.names[0]] += .1
        _, window, native = self.response_fixture(moved_axis=1, active_axes=(0, 1), core=core)
        changed = self.record_response(window, native, 'new-parameters.jsonl')
        self.assertFalse(changed['response_validated'])
        self.assertEqual(changed['receipt']['summary']['passed_axes'], self.names[1:2])

    def test_renderer_claims_cannot_grant_response_quality(self):
        _, window, native = self.response_fixture()
        window['samples'][5]['pos'][0] = .08
        result = self.record_response(window, native)
        self.assertFalse(result['response_validated'])
        self.assertIn('原生', result['receipt']['summary']['binding_error'])
        self.assertFalse(yaml.safe_load(self.core_path.read_text())['capability']['admittance']['validation']['response_validated'])

    def test_validation_exit_and_disabled_observation_remain_available(self):
        class Admission:
            def _readiness(self, config): raise AssertionError('safe exit must not depend on current configuration')
        class Native:
            def request(self, *args, **kwargs): raise AssertionError('read-only admission must not issue commands')
        for method, params in (('connect', {}), ('observe', {}), ('calibration_samples', {}), ('admittance_validation', {'active': False})):
            verify_machine_admission(Admission(), {}, method, params, Native())


if __name__ == '__main__':
    unittest.main()
