from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import zipfile

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from control_parameters import canonical_core
from profiles import Inspector
from profile_library import ProfileLibrary
from model_calibration_source import recorded_file_fingerprint
from workflow_registry import export_record
from artifact_exports import export_artifact
from user_errors import describe


class ReadinessEvidence(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name)
        original = Inspector(ROOT).inspect({'profile': 'dm_arm_gray'})
        self.core_path = self.path / 'core.yaml'
        self.hardware_path = self.path / 'hardware.yaml'
        self.profile_path = self.path / 'robot_profiles.yaml'
        self.core = canonical_core(yaml.safe_load(Path(original['resources']['core']).read_text()))
        self.core['model']['urdf_path'] = str((Path(original['resources']['core']).parent / self.core['model']['urdf_path']).resolve())
        self.names = self.core['model']['joint_names']
        named = lambda value: {name: value for name in self.names}
        for name, kp, kd in [('rigid_hold', 30, 6), ('rigid_tracking', 20, 4),
                ('compliant_hold', 10, 3), ('compliant_tracking', 8, 2),
                ('compliant_drag', 0, .2), ('servo_tracking', 25, 5)]:
            self.core['control']['controller'][name] = {'kp': named(kp), 'kd': named(kd)}
        self.core['control']['hold_compensation'] = {'enabled': False}
        self.core['safety_policy']['max_kp_override'] = named(100)
        self.core['safety_policy']['max_kd_override'] = named(50)
        self.hardware_path.write_text(Path(original['resources']['hardware']).read_text())
        original_entry = yaml.safe_load(Path(original['profile_file']).read_text())['profiles']['dm_arm_gray']
        self.profile_path.write_text(yaml.safe_dump({'profiles': {'test': original_entry}}))
        self.write_core()
        info = deepcopy(original)
        info.update(profile='test', profile_file=str(self.profile_path), fingerprint='fixture')
        info['resources'].update(core=str(self.core_path), hardware=str(self.hardware_path))
        inspector = Mock()
        inspector.inspect.return_value = info
        inspector.profiles.return_value = {'profile_file': '/builtin', 'profiles': []}
        self.library = ProfileLibrary(ROOT, inspector)
        self.library.library_file = self.path / 'library.yaml'
        self.config = {'profile': 'test', 'profile_file': str(self.profile_path)}
        self.task = self.path / 'task'; self.task.mkdir()
        (self.task / 'source').mkdir()
        (self.task / 'source/model.urdf').write_bytes(Path(self.core['model']['urdf_path']).read_bytes())
        (self.task / 'source/core.yaml').write_bytes(self.core_path.read_bytes())
        (self.task / 'source/hardware.yaml').write_bytes(self.hardware_path.read_bytes())
        self.metadata = {'task_id': 'task', 'profile': 'test', 'joint_names': self.names,
            'urdf_fingerprint': recorded_file_fingerprint(self.task / 'source/model.urdf'),
            'core_fingerprint': recorded_file_fingerprint(self.task / 'source/core.yaml'),
            'urdf_path': self.core['model']['urdf_path'],
            'hardware_config_path': str(self.hardware_path),
            'hardware_fingerprint': recorded_file_fingerprint(self.task / 'source/hardware.yaml'),
            'hardware_overrides': {}, 'runtime_provenance': {'hardware_plugin_file_path': original['hardware_plugin']}}
        (self.task / 'metadata.json').write_text(json.dumps(self.metadata))

    def write_core(self):
        self.core_path.write_text(yaml.safe_dump(self.core))

    def candidate(self):
        result = {'schema': 'serial_arm_gain_candidate', **self.metadata, 'category': 'all', 'validated': True, 'targets': {}}
        vector = lambda value: [value] * len(self.names)
        for key, purpose in [('rigid_tracking', 'RIGID'), ('compliant_tracking', 'COMPLIANT'), ('compliant_drag', 'DRAG'), ('servo_tracking', 'SERVO_TRACKING')]:
            cfg = self.core['control']['controller'][key]
            fields = {term: [cfg[term][name] for name in self.names] for term in ('kp', 'kd')}
            fields['validated'] = True
            if purpose in ('RIGID', 'COMPLIANT'):
                fields['hold_gains'] = self.core['control']['controller'][key.replace('_tracking', '_hold')]
                fields['hold_gains'] = {term: [fields['hold_gains'][term][name] for name in self.names] for term in ('kp', 'kd')}
                fields['hold_verification'] = {'validated': True}
            if purpose == 'RIGID': fields['hold_compensation'] = {'enabled': False}
            if purpose == 'COMPLIANT': fields.update(compliant_separation_validated=True,
                tested_speed_fraction=.2, rigid_reference_kp=vector(20), rigid_reference_kd=vector(4))
            if purpose == 'DRAG': fields['drag_validation'] = {'validated': True}
            if purpose == 'SERVO_TRACKING':
                fields.update(native_validation={'validated': True}, admittance_validation={'validated': True})
            result['targets'][key] = fields
        (self.task / 'gain-candidate.json').write_text(json.dumps(result))
        return result

    def test_usable_configuration_does_not_require_research_evidence(self):
        self.core.pop('capability', None)
        self.core['control']['controller']['identification_tracking'] = self.core['control']['controller'].pop('servo_tracking')
        self.write_core()
        before = self.core_path.read_bytes()
        result = self.library._readiness(self.config)
        self.assertTrue(result['control_available'], result['reason'])
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['verification_state'], 'pending')
        self.assertEqual(set(result['gain_purposes']), {'RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING'})
        self.assertEqual(self.core_path.read_bytes(), before)
        canonical = canonical_core(self.core)
        info = self.library.inspector.inspect.return_value
        hardware = yaml.safe_load(self.hardware_path.read_text())
        self.assertEqual(self.library._dependencies(info, self.core, hardware), self.library._dependencies(info, canonical, hardware))

    def test_self_checks_and_legacy_gain_records_do_not_become_joint_validation(self):
        info = self.library.inspector.inspect.return_value
        scopes = self.library._dependencies(info, self.core, yaml.safe_load(self.hardware_path.read_text()))
        path = self.library._readiness_path(str(self.profile_path), 'test')
        path.parent.mkdir(parents=True)
        path.write_text(yaml.safe_dump({'stages': {key: {'result': 'recorded', 'kind': 'operator_note',
            'scope': scopes[key], 'observations': {'checked': True}} for key in self.library._STAGES},
            'modes': {'RIGID_TRACKING': {'result': 'passed', 'scope': scopes['modes']['RIGID_TRACKING']}}}))
        result = self.library._readiness(self.config)
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['verification_state'], 'partial')
        self.assertEqual(result['gain_purposes']['RIGID']['evidence_status'], 'historical')
        self.assertEqual(result['gain_purposes']['RIGID']['checks'][1]['status'], 'pending')

    def test_import_all_matches_hold_compensation_and_both_servo_trials(self):
        candidate = self.candidate()
        for wrong in (self.task / 'metadata.json', self.task / 'missing.json'):
            with self.assertRaisesRegex(ValueError, 'gain-candidate.json'):
                self.library.approve_gain_mode(self.config, 'all', str(wrong))
        result = self.library.approve_gain_mode(self.config, 'all', str(self.task))
        self.assertTrue(all(item['evidence_status'] == 'verified' for item in result['gain_purposes'].values()))
        self.core['control']['controller']['rigid_hold']['kp'][self.names[0]] += 1
        self.write_core()
        self.assertEqual(self.library._readiness(self.config)['gain_purposes']['RIGID']['evidence_status'], 'stale')
        with self.assertRaisesRegex(ValueError, 'rigid_hold'):
            self.library.approve_gain_mode(self.config, 'RIGID_TRACKING', str(self.task / 'gain-candidate.json'))
        self.core['control']['controller']['rigid_hold']['kp'][self.names[0]] -= 1
        self.core['control']['hold_compensation'] = {'enabled': True}
        self.write_core()
        with self.assertRaisesRegex(ValueError, 'hold_compensation'):
            self.library.approve_gain_mode(self.config, 'RIGID', str(self.task))
        candidate['targets']['servo_tracking']['admittance_validation']['validated'] = False
        (self.task / 'gain-candidate.json').write_text(json.dumps(candidate))
        with self.assertRaisesRegex(ValueError, '联合验收'):
            self.library.approve_gain_mode(self.config, 'SERVO_TRACKING', str(self.task))

    def test_import_enabled_compensation_keeps_all_tested_limits_bound(self):
        candidate = self.candidate()
        rigid = candidate['targets']['rigid_tracking']
        named = lambda value: {name: value for name in self.names}
        vector = lambda value: [value] * len(self.names)
        compensation = dict(enabled=True, integral_gain=vector(.1), max_torque=vector(.12),
            max_rate=vector(.02), contact_threshold=vector(.2), velocity_threshold=.01, error_threshold=.02)
        rigid['hold_compensation'] = compensation
        rigid['hold_verification']['validated'] = False
        rigid['hold_compensation_verification'] = {'validated': True}
        self.core['control']['hold_compensation'] = {key: dict(zip(self.names, value)) if isinstance(value, list) else value for key, value in compensation.items()}
        self.write_core()
        (self.task / 'gain-candidate.json').write_text(json.dumps(candidate))
        approved = self.library.approve_gain_mode(self.config, 'RIGID', str(self.task))
        self.assertEqual(approved['gain_purposes']['RIGID']['evidence_status'], 'verified')
        self.core['control']['hold_compensation']['max_rate'][self.names[0]] = .03
        self.write_core()
        self.assertEqual(self.library._readiness(self.config)['gain_purposes']['RIGID']['evidence_status'], 'stale')
        with self.assertRaisesRegex(ValueError, 'max_rate'):
            self.library.approve_gain_mode(self.config, 'RIGID', str(self.task))

    def test_same_urdf_does_not_validate_another_robot_mapping_or_hardware(self):
        self.candidate()
        original_metadata = deepcopy(self.metadata)
        original_core = deepcopy(self.core)
        original_hardware = yaml.safe_load(self.hardware_path.read_text())
        info = self.library.inspector.inspect.return_value
        cases = ('profile', 'mapping', 'gravity', 'hardware', 'missing_context')
        for case in cases:
            with self.subTest(case=case):
                self.core = deepcopy(original_core)
                metadata = deepcopy(original_metadata)
                hardware = deepcopy(original_hardware)
                if case == 'profile': metadata['profile'] = 'another_arm'
                if case == 'mapping': self.core['calibration']['joints'][self.names[0]]['direction'] *= -1
                if case == 'gravity': self.core['model']['gravity_scale'][self.names[0]] = .8
                if case == 'hardware': hardware['damiao']['actuators'][self.names[0]]['motor_id'] += 40
                if case == 'missing_context': metadata.pop('hardware_overrides')
                self.write_core()
                self.hardware_path.write_text(yaml.safe_dump(hardware))
                (self.task / 'metadata.json').write_text(json.dumps(metadata))
                with self.assertRaises(ValueError): self.library.approve_gain_mode(self.config, 'all', str(self.task))
                self.assertFalse(self.library._readiness_path(str(self.profile_path), 'test').exists())
        self.core = original_core; self.write_core()
        self.hardware_path.write_text(yaml.safe_dump(original_hardware))
        (self.task / 'metadata.json').write_text(json.dumps(original_metadata))
        self.library.approve_gain_mode(self.config, 'all', str(self.task))
        # Mutating the recorded context after import revokes the displayed proof
        (self.task / 'metadata.json').write_text(json.dumps({**original_metadata, 'profile': 'another_arm'}))
        self.assertTrue(all(item['evidence_status'] == 'stale' for item in self.library._readiness(self.config)['gain_purposes'].values()))

    def test_effective_hardware_snapshot_and_connection_overrides_are_bound(self):
        from control_parameters import effective_hardware
        self.candidate()
        metadata = deepcopy(self.metadata)
        hardware = yaml.safe_load(self.hardware_path.read_text())
        metadata['hardware_overrides'] = {'serial_port': '/dev/fixture', 'baudrate': 921600}
        effective = effective_hardware(hardware, metadata['hardware_overrides'])
        path = self.task / 'source/hardware-effective.yaml'
        path.write_text(yaml.safe_dump(effective))
        metadata['hardware_effective_fingerprint'] = recorded_file_fingerprint(path)
        (self.task / 'metadata.json').write_text(json.dumps(metadata))
        self.library.inspector.inspect.return_value['connection_overrides'] = metadata['hardware_overrides']
        result = self.library.approve_gain_mode(self.config, 'all', str(self.task))
        self.assertTrue(all(item['evidence_status'] == 'verified' for item in result['gain_purposes'].values()))
        self.library.inspector.inspect.return_value['connection_overrides'] = {'serial_port': '/dev/another'}
        with self.assertRaisesRegex(ValueError, '硬件配置'):
            self.library.approve_gain_mode(self.config, 'all', str(self.task))
        self.library.inspector.inspect.return_value['connection_overrides'] = metadata['hardware_overrides']
        path.write_text(yaml.safe_dump(hardware))
        with self.assertRaisesRegex(ValueError, '快照已变化'):
            self.library.approve_gain_mode(self.config, 'all', str(self.task))

    def test_missing_mapping_and_safety_provide_exact_edit_actions(self):
        joint = self.names[0]
        self.core['calibration']['joints'][joint]['direction'] = 0
        self.core['safety_policy']['state_timeout_s'] = -1
        self.write_core()
        result = self.library._readiness(self.config)
        issues = {item['path']: item for item in result['setup_issues']}
        for path, group in [(f'core.calibration.joints.{joint}.direction', 'mapping'), ('core.safety_policy.state_timeout_s', 'safety')]:
            self.assertEqual(issues[path]['action']['group'], group)
            self.assertEqual(issues[path]['action']['field'], path)
        self.assertFalse(result['control_available'])

    def test_gain_zips_keep_probe_samples_with_the_candidate(self):
        self.candidate()
        probe = self.task / 'servo-probe-native-0.csv'
        probe.write_text('time_s,target,actual\n0,0,0\n')
        (self.task / 'servo-probe-link.csv').symlink_to(self.core_path)
        exported = export_record(self.path, str(self.task), 'gain')
        with zipfile.ZipFile(exported['path']) as archive:
            self.assertIn(probe.name, archive.namelist())
            self.assertNotIn('servo-probe-link.csv', archive.namelist())
        output = self.path / 'output'; output.mkdir()
        artifact = export_artifact(self.path, str(self.task), 'gain', destination=str(output))
        with zipfile.ZipFile(artifact['path']) as archive:
            self.assertIn('evidence/' + probe.name, archive.namelist())
            self.assertIn('evidence/gain-candidate.json', archive.namelist())
            self.assertIn('core.yaml', archive.namelist())

    def test_machine_error_shows_reason_and_next_step_separate_from_details(self):
        result = describe(RuntimeError('SAFETY_FAILED / STATE_TIMEOUT / value=0.0553 / limit=0.0500'), 'workbench_request', {'method': 'activate'})
        self.assertEqual(result['code'], 'FEEDBACK_TIMEOUT')
        self.assertIn('55.3 ms', result['message'])
        self.assertIn('反馈恢复', result['advice'])
        self.assertNotIn('RuntimeError', result['message'])
        self.assertIn('STATE_TIMEOUT', result['technical'])
        self.assertEqual(result['action']['page'], 'diagnostics')
        result = describe(RuntimeError('MOTOR_BUS_ACTIVATE_FAILED / DISABLE_FAILED'), 'workbench_request', {'method': 'activate'})
        self.assertIn('失能未得到确认', result['message'])
        self.assertIn('硬件急停', result['advice'])
        offline = describe(TimeoutError('timeout'), 'model_calibration_recompute')
        self.assertNotIn('指令执行结果', offline['message'])
        command = describe(RuntimeError('CMD_TIMEOUT'), 'workbench_request')
        self.assertEqual(command['action']['tab'], 'monitor')
        limited = describe(RuntimeError('REFERENCE_GOVERNOR_FAILED / INFEASIBLE'), 'workbench_request')
        self.assertEqual(limited['severity'], 'error')
        self.assertEqual(limited['category'], 'safety')
        invalid = describe(ValueError('control.hold_compensation.max_rate.joint1 不能小于 0'), 'profile_editor_save')
        self.assertEqual(invalid['action']['field'], 'core.control.hold_compensation.max_rate.joint1')
        self.assertEqual(invalid['action']['group'], 'motion')
        try:
            yaml.safe_load('control: [\n')
        except yaml.YAMLError as error:
            invalid_yaml = describe(error, 'profile_editor_load')
            exported_yaml = describe(error, 'calibration_tool_export')
        self.assertIn('第', invalid_yaml['message'])
        self.assertIn('缩进', invalid_yaml['advice'])
        self.assertIn('导出逻辑', exported_yaml['advice'])


if __name__ == '__main__':
    unittest.main()
