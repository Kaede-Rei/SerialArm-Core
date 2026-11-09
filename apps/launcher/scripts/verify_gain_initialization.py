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
            for name in ('gain-hold-baseline.csv','servo-probe-independent.csv','frames.csv'):
                (record/name).write_text('time,accepted_q,actual_q,integrator_freeze\n0,0,0,velocity\n')
            summary=load_model_calibration_summary(record)
            self.assertTrue(summary['files']['gain-hold-baseline.csv'])
            exported=export_record(root,record,'gain')
            with zipfile.ZipFile(exported['path']) as archive:
                self.assertTrue({'gain-hold-baseline.csv','servo-probe-independent.csv','frames.csv'} <= set(archive.namelist()))
                self.assertIn('integrator_freeze',archive.read('gain-hold-baseline.csv').decode())

    def test_fine_result_export_only_selected_use(self):
        row={'samples':200,'rms_error_rad':.01,'peak_error_rad':.02,'velocity_rms_rad_s':.01,'tail_error_rad':.01,'settling_s':.3}
        candidate={'category':'servo_tracking','joint_names':['j'],'validation_scope':'independent_demonstration_trajectory',
            'kp':[3.],'kd':[.2],'available':True,'validated':False,
            'native_validation':{'independent':True,'duration_s':1,'metrics':[row]},
            'admittance_required':False,'admittance_validation':{'required':False,'status':'not_requested'}}
        cfg=core_for(['j'])
        self.assertEqual(list(deployment_gain_groups(candidate)),['servo_tracking'])
        leaves,_=gain_candidate_targets(candidate,cfg,['j'])
        self.assertTrue(leaves)
        self.assertTrue(all(path[2]=='servo_tracking' for path in leaves))
        candidate['native_validation']['completed']=False
        with self.assertRaises(ValueError):deployment_gain_groups(candidate)


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
                'base_frame':'base','tool_frame':'l2','joint_names':['j2','j1'],'hardware_plugin':''}
            created=library.create_profile(options);initial=created['gain_initialization']
            self.assertEqual([axis['joint'] for axis in initial['axes']],['j2','j1'])
            self.assertFalse(initial['measured']);self.assertFalse(initial['validated'])
            self.assertFalse(initial['configuration_applied'])
            self.assertNotEqual(initial['axes'][0]['inertia_max'],initial['axes'][1]['inertia_max'])
            self.assertNotEqual(initial['axes'][0]['unconstrained_kp'],initial['axes'][1]['unconstrained_kp'])
            self.assertTrue(all(axis['kp']<=axis['working_kp_max'] for axis in initial['axes']))
            single=library.create_profile({**options,'profile':'single','base_frame':'l1','joint_names':['j2']})
            self.assertEqual(len(single['gain_initialization']['axes']),1)
            self.assertAlmostEqual(single['gain_initialization']['axes'][0]['inertia_max'],initial['axes'][0]['inertia_max'])
            config=Path(created['package'])/'fixture/config/serialarm/core.yaml'
            from control_parameters import canonical_core
            core=canonical_core(yaml.safe_load(config.read_text()))
            self.assertNotIn('max_kp_override',core['safety_policy'])
            self.assertTrue(all(value==0 for value in core['control']['controller']['servo_tracking']['kp'].values()))
            original=deepcopy(core['control']['controller'])
            for role,parameters in original.items():
                if isinstance(parameters,dict) and 'kp' in parameters:
                    core['control']['controller'][role]['kp']={name:0 if role=='compliant_drag' else 1 for name in ['j2','j1']}
                    core['control']['controller'][role]['kd']={name:.01 for name in ['j2','j1']}
            config.write_text(yaml.safe_dump(core))
            rerun=initialize_profile_gains(ProbeInspector(ROOT),config,core,source/'model.urdf',{})
            self.assertEqual(initial['axes'],rerun['axes'])
            # Choosing a protocol makes the suggestion applicable, but cannot
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
            self.assertTrue(all(value==0 for value in saved_editor['core']['control']['controller']['servo_tracking']['kp'].values()))
            receipt=json.loads((config.parent/'gain-initialization.json').read_text())
            self.assertFalse(receipt['configuration_applied']);self.assertFalse(receipt['measured'])
            self.assertTrue(receipt['suggestion_applicable']);self.assertTrue(receipt['runtime_baseline_unchanged'])
            self.assertEqual(receipt['source'],'urdf_task_response_and_protocol_grid')
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
            for driver,hardware in [('',{}),('serial_arm_hardware_damiao',editor['hardware'])]:
                untouched=deepcopy(core)
                initialize_profile_gains(ProbeInspector(ROOT),config,core,source/'model.urdf',hardware,driver)
                self.assertEqual(core,untouched,'model inspection must preserve all operational and fault recovery parameters')


if __name__=='__main__':unittest.main()
