"""Offline launcher behavior checks; no robot or user Profile is touched."""
from copy import deepcopy
import json
import os
from pathlib import Path
import re
import sys
import tempfile
import unittest
import zipfile
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/launcher/backend'))
from gain_initialization import model_gain_limits, initialize_profile_gains, protocol_gain_limits, DAMIAO_MODEL_LIMITS
from artifact_exports import gain_candidate_targets
from control_parameters import deployment_gain_groups
from workflow_registry import export_record
from persistence import load_model_calibration_summary
from control_parameters import canonical_core, four_mode_control_text
from gain_context import gain_hardware_context, verify_gain_context
from model_calibration_source import recorded_file_fingerprint


def core_for(names):
    return {'model': {'joint_names': names},
            'calibration': {'joints': {name: {'pos_ratio': 1., 'tor_ratio': 1., 'direction': 1,
                'joint_zero_offset': 0., 'actuator_zero_offset': 0.} for name in names}},
            'control': {'controller': {name: {'kp': {j: 20 for j in names}, 'kd': {j: .5 for j in names}}
                for name in ('rigid_hold','rigid_tracking','servo_tracking','compliant_hold','compliant_tracking','compliant_drag')}},
            'safety_policy': {'position_margin': .01, 'cmd_vel_scale': .25,
                'max_acc': {name: 2 for name in names}, 'max_kp_override': {name: 50 for name in names},
                'max_kd_override': {name: 2 for name in names},
                'fault_recovery': {'compliant_recovery': {'kp': {}, 'kd': {}}}},
            'shutdown': {'park_pos': {name: 0 for name in names}}}


def urdf(names):
    return '<robot name="test">'+''.join(f'<joint name="{name}" type="revolute"><limit lower="-1" upper="1" velocity="100" effort="1000"/></joint>' for name in names)+'</robot>'


class GainInitializationTests(unittest.TestCase):
    def test_candidate_hardware_alias_and_override_compatibility(self):
        plugin='serial_arm_hardware_damiao'
        with tempfile.TemporaryDirectory(prefix='serialarm-gain-hardware-') as tmp:
            alias=Path(tmp)/'serial-alias';alias.symlink_to('/dev/null')
            saved={'buses':{'main_can':{'type':'can','backend':'damiao_usb2can','device':'/dev/null','baudrate':921600}},
                   'damiao':{'bus':'main_can','feedback_timeout_s':.05,'actuators':{'j':{'motor_id':1,'master_id':0,'motor_type':'DM4310'}}}}
            recorded=deepcopy(saved)
            recorded['buses']['main_can']['device']=str(alias)
            recorded['damiao'].update(serial_port=str(alias),baudrate=921600)
            unchanged=deepcopy(recorded)
            self.assertEqual(gain_hardware_context(saved,plugin),gain_hardware_context(recorded,plugin))
            self.assertEqual(recorded,unchanged)
            self.assertNotEqual(gain_hardware_context(saved,'other_driver'),gain_hardware_context(recorded,'other_driver'))
            overridden=gain_hardware_context(saved,plugin,{'serial_port':str(alias),'baudrate':921600,'bus':'main_can'})
            self.assertEqual(overridden,gain_hardware_context(recorded,plugin))
            for change in ('port','baud','motor','backend','timing'):
                changed=deepcopy(recorded)
                if change=='port':changed['damiao']['serial_port']='/dev/zero'
                elif change=='baud':changed['damiao']['baudrate']=115200
                elif change=='motor':changed['damiao']['actuators']['j']['motor_type']='DM4340'
                elif change=='backend':changed['buses']['main_can']['backend']='other'
                else:changed['damiao']['feedback_timeout_s']=.1
                self.assertNotEqual(gain_hardware_context(saved,plugin),gain_hardware_context(changed,plugin),change)
            # No Loader overrides means authored flat settings retain the
            # driver's precedence rather than being overwritten by the bus.
            authored=deepcopy(saved);authored['damiao'].update(serial_port='/dev/zero',baudrate=115200)
            resolved=gain_hardware_context(authored,plugin,{})
            self.assertEqual(resolved['damiao']['serial_port'],'/dev/zero')
            self.assertEqual(resolved['damiao']['baudrate'],115200)
            missing=Path(tmp)/'missing';missing.symlink_to(Path(tmp)/'absent')
            authored['damiao']['serial_port']=str(missing)
            self.assertEqual(gain_hardware_context(authored,plugin)['damiao']['serial_port'],str(missing))

    def test_candidate_context_comparison_retains_snapshot_and_hardware_checks(self):
        with tempfile.TemporaryDirectory(prefix='serialarm-gain-context-') as tmp:
            root=Path(tmp);source=root/'source';source.mkdir()
            alias=root/'serial-alias';alias.symlink_to('/dev/null')
            cfg=canonical_core(core_for(['j']))
            model=root/'robot.urdf';model.write_text(urdf(['j']))
            cfg['model'].update(urdf_path=str(model))
            cfg.setdefault('control',{})['runtime']={'write_enabled':True,'model_feedforward_mode':'GRAVITY','ctrl_frequency_hz':200}
            (source/'core.yaml').write_text(yaml.safe_dump(cfg))
            hardware={'buses':{'main_can':{'type':'can','backend':'damiao_usb2can','device':'/dev/null','baudrate':921600}},
                'damiao':{'bus':'main_can','actuators':{'j':{'motor_id':1,'master_id':0,'motor_type':'DM4310'}}}}
            overrides={'serial_port':str(alias),'baudrate':921600,'bus':'main_can'}
            from control_parameters import effective_hardware
            effective=effective_hardware(hardware,overrides)
            (source/'hardware.yaml').write_text(yaml.safe_dump(hardware))
            (source/'hardware-effective.yaml').write_text(yaml.safe_dump(effective))
            metadata={'profile':'fixture','joint_names':['j'],'core_fingerprint':recorded_file_fingerprint(source/'core.yaml'),
                'urdf_fingerprint':recorded_file_fingerprint(model),'hardware_fingerprint':recorded_file_fingerprint(source/'hardware.yaml'),
                'hardware_effective_fingerprint':recorded_file_fingerprint(source/'hardware-effective.yaml'),
                'hardware_overrides':overrides,'runtime_provenance':{'hardware_plugin_file_path':'/lib/libserial_arm_hardware_damiao.so'}}
            (root/'metadata.json').write_text(json.dumps(metadata))
            info={'profile':'fixture','resources':{'core':str(source/'core.yaml')},'hardware_plugin':'serial_arm_hardware_damiao'}
            files=verify_gain_context(root,info,cfg,hardware,['DRAG'])
            self.assertIn(source/'hardware-effective.yaml',files)
            changed=deepcopy(hardware);changed['damiao']['actuators']['j']['motor_type']='DM4340'
            with self.assertRaisesRegex(ValueError,'硬件配置与增益标定时不同'):
                verify_gain_context(root,info,cfg,changed,['DRAG'])
            with self.assertRaisesRegex(ValueError,'硬件驱动'):
                verify_gain_context(root,{**info,'hardware_plugin':'different_driver'},cfg,hardware,['DRAG'])
            # A config-only native session does not carry a Profile name.
            # It can still bind to this object through all trusted snapshots
            # and its complete measured control context, without editing the
            # recorded metadata to invent a name.
            metadata['profile']=''
            metadata_bytes=json.dumps(metadata).encode()
            (root/'metadata.json').write_bytes(metadata_bytes)
            (source/'model.urdf').write_bytes(model.read_bytes())
            measured={'runtime':{**cfg['control']['runtime'],'joint_acc_filter_alpha':.2},'admittance':{'enabled':False}}
            (root/'gain-candidate.json').write_text(json.dumps({'control_context':measured}))
            files=verify_gain_context(root,info,cfg,hardware,['DRAG'])
            self.assertIn(source/'model.urdf',files)
            self.assertEqual((root/'metadata.json').read_bytes(),metadata_bytes)
            for name in ('core.yaml','model.urdf','hardware.yaml','hardware-effective.yaml'):
                snapshot=source/name;original=snapshot.read_bytes()
                for operation in ('missing','changed'):
                    if operation=='missing':snapshot.unlink()
                    else:snapshot.write_bytes(original+b'\n# altered source evidence\n')
                    with self.assertRaisesRegex(ValueError,'快照',msg=name+' '+operation):
                        verify_gain_context(root,info,cfg,hardware,['DRAG'])
                    snapshot.write_bytes(original)
            for invalid in (None,{'runtime':{},'admittance':{'enabled':False}},
                            {'runtime':measured['runtime'],'admittance':{}},
                            {'runtime':{**measured['runtime'],'ctrl_frequency_hz':float('nan')},'admittance':{'enabled':False}}):
                (root/'gain-candidate.json').write_text(json.dumps({'control_context':invalid}))
                with self.assertRaisesRegex(ValueError,'实测控制上下文'):
                    verify_gain_context(root,info,cfg,hardware,['DRAG'])
            (root/'gain-candidate.json').write_text(json.dumps({'control_context':measured}))
            changed=deepcopy(cfg);changed['calibration']['joints']['j']['pos_ratio']=2
            with self.assertRaisesRegex(ValueError,'当前映射'):
                verify_gain_context(root,info,changed,hardware,['DRAG'])
            changed=deepcopy(cfg);changed['control']['runtime']['ctrl_frequency_hz']=201
            with self.assertRaisesRegex(ValueError,'ctrl_frequency_hz'):
                verify_gain_context(root,info,changed,hardware,['DRAG'])
            changed=deepcopy(hardware);changed['damiao']['actuators']['j']['motor_type']='DM4340'
            with self.assertRaisesRegex(ValueError,'硬件配置与增益标定时不同'):
                verify_gain_context(root,info,cfg,changed,['DRAG'])
            metadata['profile']='another_robot'
            (root/'metadata.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'属于其他机器人'):
                verify_gain_context(root,info,cfg,hardware,['DRAG'])
            metadata['profile']='fixture'
            (root/'metadata.json').write_text(json.dumps(metadata))
            # Equivalent YAML is still not permission to edit recorded source
            # evidence; its original fingerprint remains mandatory.
            (source/'hardware-effective.yaml').write_text(yaml.safe_dump(hardware))
            with self.assertRaisesRegex(ValueError,'实际硬件配置快照已变化'):
                verify_gain_context(root,info,cfg,hardware,['DRAG'])

    def test_retired_pd_fields_are_ignored_removed_and_not_export_limits(self):
        cfg=core_for(['j']);modern=deepcopy(cfg)
        modern['safety_policy'].pop('max_kp_override');modern['safety_policy'].pop('max_kd_override')
        cfg['safety_policy']['max_kp_override']='legacy malformed value is ignored'
        cfg['safety_policy']['max_kd_override']={'unmapped_legacy_joint':-4}
        self.assertEqual(canonical_core(cfg),canonical_core(modern))
        source='# source comment\n'+yaml.safe_dump(cfg,sort_keys=False)+'# trailing comment\n'
        saved=four_mode_control_text(source)
        self.assertNotIn('max_kp_override',saved);self.assertNotIn('max_kd_override',saved)
        self.assertTrue(saved.startswith('# source comment\n'));self.assertTrue(saved.endswith('# trailing comment\n'))
        self.assertEqual(canonical_core(yaml.safe_load(saved)),canonical_core(modern))
        row={'samples':200,'rms_error_rad':.01,'peak_error_rad':.02,'velocity_rms_rad_s':.01,'tail_error_rad':.01,'settling_s':.3}
        candidate={'category':'servo_tracking','joint_names':['j'],'validation_scope':'independent_demonstration_trajectory',
            'kp':[90.],'kd':[3.],'available':True,'validated':False,
            'native_validation':{'independent':True,'duration_s':1,'metrics':[row]},
            'disturbance_verification':{'available':True,'completed':True,'record_path':'disturbance.csv'},
            'admittance_required':False,'admittance_validation':{'required':False,'status':'not_requested'}}
        hardware={'damiao':{'actuators':{'j':{'motor_type':'DM4310'}}}}
        self.assertTrue(gain_candidate_targets(candidate,cfg,['j'],hardware=hardware)[0])
        candidate['kp']=[501.]
        with self.assertRaisesRegex(ValueError,'硬件协议范围'):
            gain_candidate_targets(candidate,cfg,['j'],hardware=hardware)

    def test_protocol_grid_mapping_and_unknown_driver_preview(self):
        cfg=core_for(['j']);cfg['calibration']['joints']['j'].update(pos_ratio=2,tor_ratio=3)
        protocol=protocol_gain_limits(cfg,{'damiao':{'actuators':{'j':{'motor_type':'DM4340'}}}})['j']
        self.assertEqual(protocol['kp_max'],3000);self.assertEqual(protocol['kd_max'],30)
        self.assertAlmostEqual(protocol['kp_quantum'],3000/4095)
        self.assertFalse(protocol_gain_limits(cfg,{}))
        self.assertFalse(protocol_gain_limits(cfg,{'damiao':{}},'custom_hardware'))
        self.assertFalse(protocol_gain_limits(cfg,{'damiao':{}},''))
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'model.urdf';model.write_text(urdf(['j']))
            preview,_,_=model_gain_limits(cfg,model,{})
            self.assertEqual(preview[0]['gain_domain_source'],'model_effort_preview')
            self.assertIsNone(preview[0]['protocol_kp_max'])
            pending,_,complete=model_gain_limits(cfg,model,{},'serial_arm_hardware_damiao')
            self.assertFalse(complete);self.assertEqual(pending[0]['protocol_kp_max'],3000)
            self.assertEqual(pending[0]['gain_domain_source'],'damiao_mit_driver_pending_motor')

    def test_source_independent_of_old_gains_and_hardware_mapping(self):
        names=['wrist','shoulder'];a=core_for(names);b=deepcopy(a)
        b['control']['controller']['servo_tracking']={'kp':{'wrist':1,'shoulder':49},'kd':{'wrist':.01,'shoulder':1.9}}
        b['control']['controller']['rigid_tracking']=deepcopy(b['control']['controller']['servo_tracking'])
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'model.urdf';model.write_text(urdf(names))
            hw={'damiao':{'actuators':{name:{'motor_type':'DMH3510'} for name in names}}}
            self.assertEqual(model_gain_limits(a,model,hw),model_gain_limits(b,model,hw))
            limits,positions,complete=model_gain_limits(a,model,hw)
            self.assertTrue(complete);self.assertEqual([row['joint'] for row in limits],names)
            self.assertAlmostEqual(limits[0]['effort_budget'],.2)
            self.assertAlmostEqual(limits[0]['max_velocity'],7)
            a['calibration']['joints']['wrist'].update(pos_ratio=2,tor_ratio=3,direction=-1)
            mapped,_,_=model_gain_limits(a,model,hw)
            self.assertAlmostEqual(mapped[0]['max_velocity'],3.5)
            self.assertAlmostEqual(mapped[0]['effort_budget'],.6)
            self.assertAlmostEqual(mapped[0]['protocol_kp_max'],3000)
            self.assertAlmostEqual(mapped[0]['protocol_kd_quantum'],30/4095)
            self.assertGreater(min(positions[0]+1,1-positions[0]),0)

    def test_invalid_model_or_mapping_is_not_initialized(self):
        cfg=core_for(['j'])
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'model.urdf';model.write_text(urdf(['j']))
            cfg['calibration']['joints']['j']['pos_ratio']=0
            with self.assertRaisesRegex(ValueError,'映射无效'):model_gain_limits(cfg,model)
            cfg['calibration']['joints']['j']['pos_ratio']=1
            model.write_text(urdf(['j']).replace('revolute','prismatic'))
            with self.assertRaisesRegex(ValueError,'回转关节'):model_gain_limits(cfg,model)

    def test_declared_motor_ranges_match_native_driver(self):
        text=(ROOT/'src/robot_supports/hardware/damiao/third_lib/damiao/include/dm_hw/damiao.hpp').read_text()
        body=text.split('static LimitParam limit_param')[1].split('};',1)[0]
        rows=re.findall(r'\{\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\}',body)
        self.assertEqual([tuple(map(float,row)) for row in rows],list(DAMIAO_MODEL_LIMITS.values()))

    def test_full_cycle_and_hold_evidence_survive_raw_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);record=root/'task';record.mkdir()
            (record/'metadata.json').write_text(json.dumps({'task_id':'test','joint_names':['j']}))
            (record/'gain-candidate.json').write_text(json.dumps({'category':'servo_tracking'}))
            for name in ('gain-hold-baseline.csv','servo-probe-independent.csv','gain-transition-native.csv','frames.csv'):
                (record/name).write_text('time,accepted_q,actual_q,integrator_freeze\n0,0,0,velocity\n')
            summary=load_model_calibration_summary(record)
            self.assertTrue(summary['files']['gain-hold-baseline.csv'])
            self.assertTrue(summary['files']['gain-transition-native.csv'])
            exported=export_record(root,record,'gain')
            with zipfile.ZipFile(exported['path']) as archive:
                self.assertTrue({'gain-hold-baseline.csv','servo-probe-independent.csv','gain-transition-native.csv','frames.csv'} <= set(archive.namelist()))
                self.assertIn('integrator_freeze',archive.read('gain-hold-baseline.csv').decode())
                self.assertEqual(archive.read('gain-transition-native.csv'),(record/'gain-transition-native.csv').read_bytes())

    def test_fine_result_export_only_selected_use(self):
        row={'samples':200,'rms_error_rad':.01,'peak_error_rad':.02,'velocity_rms_rad_s':.01,'tail_error_rad':.01,'settling_s':.3}
        candidate={'category':'servo_tracking','joint_names':['j'],'validation_scope':'independent_demonstration_trajectory',
            'kp':[3.],'kd':[.2],'available':True,'validated':False,
            'native_validation':{'independent':True,'duration_s':1,'metrics':[row]},
            'disturbance_verification':{'available':True,'completed':True,'record_path':'disturbance.csv'},
            'admittance_required':False,'admittance_validation':{'required':False,'status':'not_requested'}}
        cfg=core_for(['j'])
        self.assertEqual(list(deployment_gain_groups(candidate)),['servo_tracking'])
        leaves,_=gain_candidate_targets(candidate,cfg,['j'])
        self.assertTrue(leaves)
        self.assertTrue(all(path[2]=='servo_tracking' for path in leaves))
        candidate['native_validation']['completed']=False
        with self.assertRaises(ValueError):deployment_gain_groups(candidate)

    def test_drag_quiet_scope_remains_available_without_release_evidence(self):
        axis={'samples':100,'quiet_stability_measured':True,'quiet_stability_passed':True,
              'quiet_position_span_rad':.0001,'quiet_speed_rad_s':.0002,
              'release_decay_measured':False,'release_decay_passed':False,
              'initial_velocity_rad_s':None,'decay_s':None}
        candidate={'category':'compliant_drag','joint_names':['j'],
                   'validation_scope':'independent_demonstration_trajectory',
                   'kp':[0.],'kd':[.2],'available':True,'validated':True,
                   'drag_validation':{'available':True,'completed':True,'samples':100,'axes':[axis]}}
        self.assertEqual(list(deployment_gain_groups(candidate)),['compliant_drag'])
        candidate['validated']=False;axis['quiet_stability_passed']=False
        self.assertEqual(list(deployment_gain_groups(candidate)),['compliant_drag'])
        axis['quiet_stability_measured']=False
        with self.assertRaises(ValueError):deployment_gain_groups(candidate)

    def test_complete_parameters_remain_available_with_missing_purpose_evidence(self):
        row={'samples':200,'rms_error_rad':.01,'peak_error_rad':.02,'velocity_rms_rad_s':.01,'tail_error_rad':.01,'settling_s':.3}
        path={'independent':True,'completed':True,'duration_s':1,'metrics':[row]}
        hold={'completed':True,'available':True,'samples':100,'worst_error_rad':.01,'worst_velocity_rad_s':.01}
        disturbance={'completed':True,'available':True,'record_path':'disturbance.csv'}
        fields={'kp':[3.],'kd':[.2],'available':True,'validated':False,'qualification_complete':True,
                'native_validation':path,'hold_gains':{'kp':[3.],'kd':[.2]},'hold_verification':hold,
                'disturbance_verification':disturbance,'hold_disturbance_verification':disturbance}
        groups={purpose:deepcopy(fields) for purpose in ('rigid_tracking','compliant_tracking','compliant_drag','servo_tracking')}
        groups['compliant_tracking'].update(tested_speed_fraction=.65,rigid_reference_kp=[3.],rigid_reference_kd=[.2])
        groups['servo_tracking'].update(admittance_required=False,admittance_validation={'required':False,'status':'not_requested'})
        groups['compliant_drag'].update(kp=[0.],drag_validation={'available':True,'completed':True,'samples':100,
            'axes':[{'samples':100,'initial_velocity_rad_s':.02,'peak_velocity_rad_s':.03,'final_velocity_rad_s':.001,'decay_s':.8}]})
        candidate={'category':'all','joint_names':['j'],'validation_scope':'independent_demonstration_trajectory','targets':groups}
        self.assertEqual(len(deployment_gain_groups(candidate)),4)
        groups['compliant_drag']['drag_validation']['available']=False
        self.assertEqual(list(deployment_gain_groups(candidate,['servo_tracking'])),['servo_tracking'])
        self.assertEqual(len(deployment_gain_groups(candidate)),3)
        with self.assertRaises(ValueError):deployment_gain_groups(candidate,['compliant_drag'])
        groups['compliant_drag']['drag_validation']['available']=True
        groups['rigid_tracking']['hold_disturbance_verification']={'available':False}
        candidate.update(review_ready=False,measurement_complete=False,qualification_complete=False)
        groups['rigid_tracking']['qualification_complete']=False
        self.assertEqual(list(deployment_gain_groups(candidate,['rigid_tracking'])),['rigid_tracking'])
        self.assertFalse(groups['rigid_tracking']['validated'])
        groups['rigid_tracking']['hold_verification']['completed']=False
        with self.assertRaises(ValueError):deployment_gain_groups(candidate,['rigid_tracking'])


@unittest.skipUnless(os.environ.get('SERIAL_ARM_GAIN_TEST_PROBE'), 'set SERIAL_ARM_GAIN_TEST_PROBE for compiled model integration')
class NativeProfileInitializationTests(unittest.TestCase):
    def test_actual_profile_creation_mapping_inertia_and_old_gain_independence(self):
        from profiles import Inspector
        from profile_library import ProfileLibrary
        class ProbeInspector(Inspector):
            def model_probe(self):return os.environ['SERIAL_ARM_GAIN_TEST_PROBE']
        with tempfile.TemporaryDirectory(prefix='serialarm-gain-profile-') as tmp:
            tmp=Path(tmp);source=tmp/'fixture';source.mkdir()
            (source/'package.xml').write_text('<package><name>fixture</name></package>')
            links='<link name="base"/>'
            for index in (1,2):
                links+=f'<link name="l{index}"><inertial><origin xyz=".2 0 0"/><mass value="{index}"/><inertia ixx="{.01*index}" iyy="{.01*index}" izz="{.01*index}" ixy="0" ixz="0" iyz="0"/></inertial></link>'
            joints=''
            for index,parent in ((1,'base'),(2,'l1')):
                joints+=f'<joint name="j{index}" type="revolute"><parent link="{parent}"/><child link="l{index}"/><origin xyz=".4 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="3" effort="20"/></joint>'
            (source/'model.urdf').write_text('<robot name="fixture">'+links+joints+'</robot>')
            library=ProfileLibrary(ROOT,ProbeInspector(ROOT))
            options={'source':str(source),'profile':'fresh','destination':str(tmp/'out'),
                'model_relative':'model.urdf','base_frame':'base','tool_frame':'l2','joint_names':['j2','j1'],'hardware_plugin':''}
            created=library.create_profile(options);initial=created['gain_initialization']
            self.assertEqual([axis['joint'] for axis in initial['axes']],['j2','j1'])
            self.assertFalse(initial['measured']);self.assertFalse(initial['validated'])
            self.assertTrue(initial['reference_only'])
            self.assertEqual(initial['value_role'],'unmeasured_model_reference')
            self.assertFalse(initial['accuracy_thresholds_enabled'])
            self.assertTrue(all(axis['reference_only'] for axis in initial['axes']))
            self.assertTrue(initial['configuration_applied'])
            self.assertTrue(initial['configuration_initialized'])
            self.assertNotEqual(initial['axes'][0]['inertia_max'],initial['axes'][1]['inertia_max'])
            self.assertNotEqual(initial['axes'][0]['unconstrained_kp'],initial['axes'][1]['unconstrained_kp'])
            self.assertEqual(initial['design_frequency_hz'],2)
            self.assertEqual(initial['damping_ratio'],.7)
            self.assertEqual(initial['positions'],[0.,0.])
            self.assertEqual(initial['damping_design_method'],'double_diagonalization_diagonal_projection')
            self.assertIn('double_diagonalization_diagonal_projection',initial['source'])
            self.assertEqual(initial['damping_interface_projection'],'retain_diagonal_for_independent_mit_derivative_channels')
            self.assertIn('diagnostic',initial['full_damping_matrix_role'])
            self.assertFalse(initial['modal_diagnostics']['measured'])
            self.assertEqual(len(initial['full_damping_matrix']),2)
            for index,axis in enumerate(initial['axes']):
                self.assertAlmostEqual(axis['unconstrained_kd'],initial['full_damping_matrix'][index][index])
                self.assertGreater(axis['unconstrained_kd'],0)
                self.assertAlmostEqual(initial['full_damping_matrix'][0][index],initial['full_damping_matrix'][index][0])
            single=library.create_profile({**options,'profile':'single','base_frame':'l1','joint_names':['j2']})
            self.assertEqual(len(single['gain_initialization']['axes']),1)
            self.assertAlmostEqual(single['gain_initialization']['axes'][0]['inertia_max'],initial['axes'][0]['inertia_max'])
            config=Path(created['package'])/'fixture/config/serialarm/core.yaml'
            self.assertIn('双对角化模型，仅供参考',config.read_text())
            from control_parameters import canonical_core
            core=canonical_core(yaml.safe_load(config.read_text()))
            self.assertNotIn('max_kp_override',core['safety_policy'])
            self.assertEqual(core['control']['controller']['servo_tracking']['kp'],{axis['joint']:axis['kp'] for axis in initial['axes']})
            self.assertTrue(all(value==0 for value in core['control']['controller']['compliant_drag']['kp'].values()))
            purposes=initial['purpose_suggestions']
            self.assertEqual(purposes['compliant_hold']['design_frequency_hz'],1)
            self.assertEqual(purposes['servo_tracking']['design_frequency_hz'],2)
            for name in ['j2','j1']:
                self.assertAlmostEqual(core['control']['controller']['compliant_tracking']['kp'][name],
                    core['control']['controller']['servo_tracking']['kp'][name]/4)
                self.assertAlmostEqual(core['control']['controller']['compliant_tracking']['kd'][name],
                    core['control']['controller']['servo_tracking']['kd'][name]/2)
                self.assertEqual(core['control']['controller']['compliant_drag']['kd'][name],
                    core['control']['controller']['servo_tracking']['kd'][name])
            self.assertTrue(all(not value['measured'] and not value['validated'] for value in purposes.values()))
            self.assertTrue(all(value['reference_only'] and not value['accuracy_thresholds_enabled']
                and all(axis['reference_only'] for axis in value['axes']) for value in purposes.values()))
            for purpose in purposes.values():
                self.assertEqual(purpose['damping_design_method'],initial['damping_design_method'])
                self.assertEqual(purpose['damping_stiffness_basis'],
                    initial['unconstrained_kp'] if purpose['design_frequency_hz']==2 else purposes['compliant_hold']['unconstrained_kp'])
                for index,axis in enumerate(purpose['axes']):
                    self.assertAlmostEqual(axis['unconstrained_kd'],purpose['full_damping_matrix'][index][index])
            original=deepcopy(core['control']['controller'])
            for role,parameters in original.items():
                if isinstance(parameters,dict) and 'kp' in parameters:
                    core['control']['controller'][role]['kp']={name:0 if role=='compliant_drag' else 1 for name in ['j2','j1']}
                    core['control']['controller'][role]['kd']={name:.01 for name in ['j2','j1']}
            config.write_text(yaml.safe_dump(core))
            rerun=initialize_profile_gains(ProbeInspector(ROOT),config,core,source/'model.urdf',{})
            self.assertEqual(initial['axes'],rerun['axes'])
            # Artificial limits, park pose and old gains cannot change the raw
            # geometry/inertia design. This invokes the real native model.
            changed=deepcopy(core)
            changed['shutdown']['park_pos']={'j2':1.8,'j1':-1.8}
            changed['safety_policy'].update(position_margin=.5,cmd_vel_scale=.01,max_acc={'j2':.02,'j1':.03})
            alternate=source/'artificial-limits.urdf'
            alternate.write_text((source/'model.urdf').read_text().replace('lower="-2" upper="2" velocity="3" effort="20"','lower="-9" upper="9" velocity=".01" effort=".02"'))
            limit_preview=initialize_profile_gains(ProbeInspector(ROOT),config,changed,alternate,{})
            self.assertEqual(initial['unconstrained_kp'],limit_preview['unconstrained_kp'])
            self.assertEqual(initial['unconstrained_kd'],limit_preview['unconstrained_kd'])
            self.assertEqual(initial['mass_matrix_basis'],limit_preview['mass_matrix_basis'])
            self.assertEqual(initial['purpose_nominal_gain_designs'],limit_preview['purpose_nominal_gain_designs'])
            # Choosing a protocol makes the suggestion encodable, but does not
            # turn an unmeasured seed into every runtime/recovery baseline.
            core['control']['controller']=deepcopy(original)
            config.write_text(yaml.safe_dump(core))
            editor=library.profile_editor_load(created['config'])
            editor['profile_entry']['hardware']['plugin']='serial_arm_hardware_damiao'
            editor['hardware']={'buses':{'main':{'type':'can','backend':'damiao_usb2can','device':'/not-connected','baudrate':921600}},
                'damiao':{'bus':'main','refresh_state_in_read':False,'feedback_timeout_s':.05,'activation_retries':3,'startup_read_cycles':5,
                'stop_kp':10.,'stop_kd':.15,'stop_cycles':5,'actuators':{name:{'name':'actuator'+str(index+1),'motor_id':index+1,'master_id':0,'motor_type':'DM4310'} for index,name in enumerate(['j2','j1'])}}}
            editor['hardware_imported']=True
            saved_editor=library.profile_editor_save(created['config'],editor)
            self.assertTrue(all(value>0 for value in saved_editor['core']['control']['controller']['servo_tracking']['kp'].values()))
            receipt=json.loads((config.parent/'gain-initialization.json').read_text())
            self.assertTrue(receipt['configuration_applied']);self.assertFalse(receipt['measured'])
            self.assertTrue(receipt['reference_only']);self.assertFalse(receipt['accuracy_thresholds_enabled'])
            self.assertIn('双对角化模型，仅供参考',config.read_text())
            self.assertTrue(receipt['suggestion_applicable']);self.assertFalse(receipt['runtime_baseline_unchanged'])
            self.assertEqual(receipt['source'],initial['source'])
            self.assertEqual(receipt['unconstrained_kp'],initial['unconstrained_kp'])
            self.assertEqual(receipt['mass_matrix_basis'],initial['mass_matrix_basis'])
            self.assertEqual(receipt['task_design_targets']['purposes'],initial['task_design_targets']['purposes'])
            for role,purpose in receipt['purpose_suggestions'].items():
                for axis in purpose['axes']:
                    self.assertEqual(saved_editor['core']['control']['controller'][role]['kp'][axis['joint']],axis['kp'])
                    self.assertEqual(saved_editor['core']['control']['controller'][role]['kd'][axis['joint']],axis['kd'])
                    self.assertAlmostEqual(axis['kd']/(5/4095),round(axis['kd']/(5/4095)),places=4)
            self.assertAlmostEqual(receipt['protocol_gain']['j2']['kp_quantum'],500/4095)
            for axis in receipt['axes']:
                self.assertAlmostEqual(axis['kp']/(500/4095),round(axis['kp']/(500/4095)),places=4)
            # A source-authored profile preserves its parameters while receiving
            # suggestions separately; only disposable copied output is modified.
            authored=source/'config/core/authored.yaml';authored.parent.mkdir(parents=True)
            core['control']['controller']=original
            core['control']['controller']['servo_tracking']['kp']={'j2':7,'j1':8}
            authored.write_text(yaml.safe_dump(core));before=authored.read_bytes()
            saved=library.create_profile({**options,'profile':'authored'})
            copied=Path(saved['package'])/'fixture/config/core/authored.yaml'
            self.assertEqual(canonical_core(yaml.safe_load(copied.read_text()))['control']['controller']['servo_tracking']['kp'],{'j2':7,'j1':8})
            self.assertEqual(authored.read_bytes(),before)
            self.assertTrue(saved['gain_initialization']['existing_configuration_preserved'])
            self.assertTrue(saved['gain_initialization']['reference_only'])
            self.assertNotIn('双对角化模型，仅供参考',copied.read_text())
            for driver,hardware in [('',{}),('serial_arm_hardware_damiao',editor['hardware'])]:
                untouched=deepcopy(core)
                initialize_profile_gains(ProbeInspector(ROOT),config,core,source/'model.urdf',hardware,driver)
                self.assertEqual(core,untouched,'model inspection must preserve all operational and fault recovery parameters')


if __name__=='__main__':unittest.main()
