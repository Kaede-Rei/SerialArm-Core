from copy import deepcopy
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from control_parameters import canonical_core, canonical_control_text, add_mapping_value, deployment_gain_groups, complete_mapping_values
from persistence import _runtime_text, patch_text
from profile_library import ProfileLibrary
from bridge import runtime_parameter_snapshot, verify_machine_admission


class ControlParameters(unittest.TestCase):
    def test_shared_yaml_defaults_cannot_change_unselected_paths(self):
        source = 'model: {joint_names: [a]}\ncontrol:\n  hold_compensation: &hold {enabled: false}\nmetadata: {original_defaults: *hold}\n'
        canonical = canonical_core(yaml.safe_load(source))
        self.assertEqual(canonical['metadata']['original_defaults'], {'enabled': False})
        self.assertIn('integral_gain', canonical['control']['hold_compensation'])
        with self.assertRaisesRegex(ValueError, '共享别名'):
            complete_mapping_values(source, ('control',), {'hold_compensation': {'enabled': False, 'max_rate': {'a': .05}}})
        with self.assertRaisesRegex(ValueError, '超出指定范围'):
            patch_text(source, {('control', 'hold_compensation', 'enabled'): 'true'})

    def test_tracking_success_does_not_grant_admittance_or_hold_qualification(self):
        value = dict(category='servo_tracking', validated=True, native_validation={'validated': True},
                     admittance_validation={'validated': False})
        with self.assertRaisesRegex(ValueError, '联合'): deployment_gain_groups(value)
        value['admittance_validation']['validated'] = True
        self.assertEqual(set(deployment_gain_groups(value)), {'servo_tracking'})
        value = dict(category='rigid_tracking', validated=True, hold_verification={'validated': True})
        with self.assertRaisesRegex(ValueError, '独立保持'): deployment_gain_groups(value)

    def test_canonical_alias_preserves_independent_hold_and_tracking(self):
        original = {'control': {'controller': {'rigid_hold': {'kp': {'a': 2}, 'kd': {'a': 1}},
                     'rigid_tracking': {'kp': {'a': 5}, 'kd': {'a': 2}},
                     'identification_tracking': {'kp': {'a': 8}, 'kd': {'a': 3}}}}}
        result = canonical_core(original)
        self.assertEqual(result['control']['controller']['servo_tracking']['kp'], {'a': 8})
        self.assertEqual(result['control']['controller']['rigid_hold']['kp'], {'a': 2})
        self.assertEqual(result['control']['controller']['rigid_tracking']['kp'], {'a': 5})
        self.assertIn('identification_tracking', original['control']['controller'])
        original['control']['controller']['servo_tracking'] = {'kp': {'a': 9}, 'kd': {'a': 3}}
        with self.assertRaisesRegex(ValueError, '冲突'): canonical_core(original)

    def test_source_additions_and_alias_migration_support_flow_and_block_yaml(self):
        for flow in (False, True):
            for duplicate in (False, True):
                gain = dict(kp={'a': 8}, kd={'a': 2})
                value = {'control': {'controller': {'allow_full_cmd': False, 'identification_tracking': gain}},
                         'capability': {'admittance': {'enabled': False}}, 'unrelated': {'keep': 42}}
                if duplicate: value['control']['controller']['servo_tracking'] = deepcopy(gain)
                text = '# preserve header\n' + yaml.safe_dump(value, sort_keys=False, default_flow_style=flow)
                updated = canonical_control_text(text)
                updated = add_mapping_value(updated, ('capability', 'admittance'), 'reference_behavior', 'NATIVE_IMPEDANCE')
                parsed = yaml.safe_load(updated)
                self.assertNotIn('identification_tracking', parsed['control']['controller'])
                self.assertEqual(parsed['control']['controller']['servo_tracking'], gain)
                self.assertEqual(parsed['unrelated'], {'keep': 42})
                self.assertTrue(updated.startswith('# preserve header\n'))

    def test_runtime_save_includes_strategy_axes_and_all_execution_gains(self):
        names = ['a', 'b']
        zero, one = [0., 0.], [1., 1.]
        runtime = dict(enabled=True, reference_behavior='MODEL_ADMITTANCE', joint_enabled=[True, False],
                       observer_mode='MOMENTUM', momentum_gain=[20., 20.], mass=one, damping=[4., 4.],
                       stiffness=[10., 10.], max_delta_q=one, max_delta_q_dot=one,
                       torque_bias=zero, torque_threshold=zero,
                       execution_gains={'servo_tracking': {'kp': [11., 12.], 'kd': [1., 2.]}})
        runtime['hold_compensation'] = dict(enabled=True, integral_gain=[.15, .2],
            max_torque=[.2, .25], max_rate=[.05, .06], contact_threshold=[.3, .4],
            velocity_threshold=.02, error_threshold=.03)
        friction = dict(enabled=False, velocity_transition=.03,
                        **{key: zero for key in ('positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous')})
        runtime['friction'] = friction
        core = {'model': {'joint_names': names, 'gravity_scale': dict(zip(names, one))},
                'control': {'controller': {'identification_tracking': {'kp': dict(zip(names, one)), 'kd': dict(zip(names, one))}}},
                'capability': {'admittance': {'enabled': False, 'joint_enabled': dict(zip(names, [True, True])),
                'observer': {'mode': 'FULL_ID', 'momentum_gain': dict(zip(names, one))},
                'calibration': {'torque_bias': dict(zip(names, zero)), 'torque_threshold': dict(zip(names, zero)),
                 'friction': {key: dict(zip(names, val)) if isinstance(val, list) else val for key, val in friction.items()}},
                'controller': {key: dict(zip(names, one)) for key in ('mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot')}}}}
        updated = yaml.safe_load(_runtime_text(yaml.safe_dump(core), names, runtime, one))
        self.assertEqual(updated['capability']['admittance']['reference_behavior'], 'MODEL_ADMITTANCE')
        self.assertIs(updated['capability']['admittance']['enabled'], True)
        self.assertIs(updated['capability']['admittance']['joint_enabled']['b'], False)
        self.assertEqual(updated['control']['controller']['servo_tracking']['kp']['a'], 11.)
        self.assertNotIn('identification_tracking', updated['control']['controller'])
        self.assertEqual(updated['control']['hold_compensation']['max_torque'], {'a': .2, 'b': .25})
        self.assertIs(updated['control']['hold_compensation']['enabled'], True)
        self.assertEqual(_runtime_text(yaml.safe_dump(updated), names, runtime, one), yaml.safe_dump(updated))
        for flow in (False, True):
            short = deepcopy(core)
            short['control']['hold_compensation'] = {'enabled': False}
            source = '# keep abbreviated config\n' + yaml.safe_dump(short, default_flow_style=flow)
            text = _runtime_text(source, names, runtime, one)
            self.assertTrue(text.startswith('# keep abbreviated config\n'))
            self.assertEqual(yaml.safe_load(text)['control']['hold_compensation'], updated['control']['hold_compensation'])
        for capability in (None, {'unrelated': {'keep': 42}}):
            old = deepcopy(core)
            if capability is None: del old['capability']
            else: old['capability'] = capability
            migrated = yaml.safe_load(_runtime_text('# keep old comment\n' + yaml.safe_dump(old), names, runtime, one))
            self.assertEqual(migrated['capability']['admittance']['controller']['damping'], {'a': 4., 'b': 4.})
            if capability: self.assertEqual(migrated['capability']['unrelated'], {'keep': 42})
        for bad in (float('nan'), -1., True):
            invalid = deepcopy(runtime)
            invalid['hold_compensation']['max_torque'][0] = bad
            with self.assertRaisesRegex(ValueError, '补偿'):
                _runtime_text(yaml.safe_dump(core), names, invalid, one)

    def test_valid_profile_can_enable_observation_without_implicit_strategy_change(self):
        root = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source = folder / 'source'; source.mkdir()
            urdf = source / 'model.urdf'
            urdf.write_text('<robot name="t"><link name="base"/><link name="tip"/><joint name="axis" type="revolute"><parent link="base"/><child link="tip"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint></robot>')
            from profiles import Inspector
            library = ProfileLibrary(root, Inspector(root)); library.cache = folder / 'cache'; library.library_file = folder / 'library.yaml'
            created = library.create_profile(dict(source=str(urdf), profile='test', destination=str(folder / 'created'),
                joint_names=['axis'], base_frame='base', tool_frame='tip', hardware_plugin=''))
            entry = created.get('entry') or created
            config = created.get('config') or {key: entry.get(key, '') for key in ('profile', 'profile_file', 'resource_paths')}
            if not config.get('profile_file'):
                config = {'profile': 'test', 'profile_file': str(folder / 'created/test/robot_profiles.yaml'),
                          'resource_paths': str(folder / 'created/test')}
            draft = library.profile_editor_load(config)
            draft['core']['capability']['admittance']['enabled'] = True
            saved = library.profile_editor_save(config, draft)
            self.assertTrue(saved['core']['capability']['admittance']['enabled'])
            self.assertEqual(saved['core']['capability']['admittance']['reference_behavior'], 'NATIVE_IMPEDANCE')
            text = Path(saved['core_path']).read_bytes()
            library.profile_editor_save(config, saved)
            self.assertEqual(Path(saved['core_path']).read_bytes(), text)
            for enabled in (False, True):
                old = yaml.safe_load(text)
                old['capability']['admittance']['enabled'] = enabled
                old['capability']['admittance'].pop('reference_behavior', None)
                old['control']['hold_compensation'] = {'enabled': False}
                path = Path(saved['core_path'])
                original = '# retain untouched legacy source\n' + yaml.safe_dump(old)
                path.write_text(original)
                loaded = library.profile_editor_load(config)
                self.assertEqual(loaded['core']['capability']['admittance']['reference_behavior'],
                                 'MODEL_ADMITTANCE' if enabled else 'NATIVE_IMPEDANCE')
                self.assertEqual(loaded['core']['control']['hold_compensation']['integral_gain'], {'axis': .15})
                library.profile_editor_save(config, loaded)
                self.assertEqual(path.read_text(), original)

    def test_persistence_is_bound_to_live_configuration_and_excludes_trials(self):
        info = dict(fingerprint='same', profile='arm', resources={'core': '/tmp/arm.yaml'})
        inspector = Mock(); inspector.inspect.return_value = info
        native = Mock(); native.status.return_value = {'state': 'running'}
        native.request.side_effect = lambda method, *args, **kwargs: {'model_calibration': {'phase': 'searching'}} if method == 'status' else {}
        with self.assertRaisesRegex(ValueError, '试验'): runtime_parameter_snapshot(inspector, native, {}, {'profile': 'arm'}, 'same')
        with self.assertRaisesRegex(ValueError, '不一致'): runtime_parameter_snapshot(inspector, native, {}, {'profile': 'arm'}, 'changed')


if __name__ == '__main__':
    unittest.main()
