#!/usr/bin/env python3
"""Check native demonstration-bound probes and explicit margins using only Mock.

Temporary configurations disable writes and never load a hardware plugin. The
synthetic path puts joints 2/3 at demonstrated endpoints while other joints
have two-sided room. No physical stop or successful identification is inferred.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def config_case(directory, margin, widen):
    directory.mkdir()
    config, names = prepare_config(directory)
    document = yaml.safe_load(config.read_text())
    original = Path(document['model']['urdf_path'])
    model = ET.parse(original)
    joint2 = next(joint for joint in model.getroot().findall('joint') if joint.get('name') == names[1])
    original_lower = float(joint2.find('limit').get('lower'))
    if widen:
        for joint in model.getroot().findall('joint'):
            if joint.get('name') not in names:
                continue
            limit = joint.find('limit')
            limit.set('lower',str(float(limit.get('lower'))-1))
            limit.set('upper',str(float(limit.get('upper'))+1))
        copied = directory / 'wider-limits.urdf'
        model.write(copied,encoding='utf-8',xml_declaration=True)
        document['model']['urdf_path'] = str(copied)
    document['safety_policy']['position_margin'] = margin
    config.write_text(json.dumps(document))
    return config,names,original_lower


def source_path(native, directory, names, joint2_min):
    synthetic_record(native,directory,names,25)
    bounds = [[math.inf,-math.inf] for _ in names]
    with (directory/'trajectory.csv').open('w') as stream:
        stream.write('sample_dt,0.05\nindex,'+','.join(names)+'\n')
        for index in range(501):
            u=index/500
            q=[.03*math.sin(2*math.pi*u) for _ in names]
            q[1]=joint2_min*math.sin(math.pi*u)**2
            # Fresh Mock position is 0, legitimately 1 mrad outside the old
            # demonstrated endpoint; the worker must include that fresh point
            q[2]=.001+.03*math.sin(math.pi*u)**2
            for axis,value in enumerate(q):
                bounds[axis][0]=min(bounds[axis][0],value)
                bounds[axis][1]=max(bounds[axis][1],value)
            stream.write(str(index)+','+','.join(format(v,'.17g') for v in q)+'\n')
    return bounds


def wait_saved_stop(native, task):
    native.request('model_calibration_cancel',{'task_id':task['task_id']})
    deadline=time.monotonic()+10
    folder=Path(task['directory'])
    while time.monotonic()<deadline:
        status=native.request('model_calibration_status')
        if status['phase'] in ('complete','cancelled','failed'):
            try:
                saved=json.loads((folder/'result.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):
                saved={}
            if saved.get('phase')==status['phase'] and saved.get('termination')==status.get('termination'):
                candidate=json.loads((folder/'gain-candidate.json').read_text())
                assert candidate['termination']==status['termination'],status
                assert status['termination']['kind']!='execution_fault',status
                assert native.request('status')['robot_state']=='ACTIVE'
                return candidate,status
        time.sleep(.03)
    raise AssertionError('Native worker did not finish recording after stop')


def checked_bounds(path,names):
    with path.open() as stream:
        rows=csv.reader(stream)
        next(rows)
        assert next(rows)==['index',*names]
        bounds=[[math.inf,-math.inf] for _ in names]
        for row in rows:
            for axis,value in enumerate(map(float,row[1:])):
                bounds[axis][0]=min(bounds[axis][0],value)
                bounds[axis][1]=max(bounds[axis][1],value)
    return bounds


def check_endpoint_transition(folder,candidate,names):
    experiment=next(item for item in candidate['experiments']
                    if item.get('kind')=='feedback_bounded_endpoint_transition' and item.get('completed'))
    assert experiment['initial_reference_source']=='fresh_measured_feedback',experiment
    assert experiment['counted_in_budget'] is True,experiment
    with (folder/experiment['record_path']).open() as stream:rows=list(csv.DictReader(stream))
    measured={}
    for axis,joint in enumerate(names):
        rr=[row for row in rows if row['joint']==joint]
        assert len(rr)>=10 and all(None not in row and None not in row.values() for row in rr),joint
        origin=experiment['initial_reference_rad'][axis]
        assert abs(float(rr[0]['reference'])-origin)<1e-8,(joint,rr[0],origin)
        assert abs(float(rr[-1]['reference'])-experiment['target_rad'][axis])<1e-7,joint
        max_velocity=experiment['max_velocity_rad_s'][axis]
        max_acceleration=experiment['max_acceleration_rad_s2'][axis]
        assert all(int(row['limited'])==0 for row in rr),joint
        assert max(abs(float(row['reference_velocity'])) for row in rr)<=max_velocity+1e-6,joint
        assert max(abs(float(row['reference'])-float(row['actual'])) for row in rr)<=experiment['reference_error_cap_rad'][axis]+1e-6,joint
        observed_acceleration=0
        for before,after in zip(rr,rr[1:]):
            dt=float(after['time_s'])-float(before['time_s'])
            assert dt>0,joint
            # The planner clamps its own update interval to at most .02 s;
            # CSV stamps describe completed writes, not that planner clock
            dq=abs(float(after['reference'])-float(before['reference']))
            dv=abs(float(after['reference_velocity'])-float(before['reference_velocity']))
            assert dq<=max_velocity*.02+1e-6,joint
            assert dv<=max_acceleration*.02+1e-6,joint
            observed_acceleration=max(observed_acceleration,dv/dt)
        measured[joint]={'samples':len(rr),'initial_reference_rad':origin,'target_rad':experiment['target_rad'][axis],
                         'first_reference_rad':float(rr[0]['reference']),'peak_reference_velocity_rad_s':max(abs(float(row['reference_velocity'])) for row in rr),
                         'observed_command_write_velocity_derivative_rad_s2':observed_acceleration,
                         'step_bound_scope':'planner_maximum_update_interval_not_equal_to_CSV_write_clock'}
    # A completed demonstrated-path return can already have reached the full
    # path endpoint. Do not require the subsequent entrance to move it again.
    return measured


def check_measurement_routes(folder,candidate,names,demonstrated):
    records=[item for item in candidate['experiments']
             if item.get('kind')=='demonstrated_measurement_center_entry' and item.get('completed')]
    entering=next(item for item in records if item['direction']=='toward_measurement_region')
    assert max(abs(value) for value in entering['fresh_origin_rad'])<1e-9,entering
    measured=[]
    for item in records:
        assert item['counted_in_budget'] is True,item
        assert item['validation_scope']=='positioning_only_not_identification_or_purpose_qualification',item
        assert item['kd']==item['initial_kd'],item
        assert all(step['kd']==item['initial_kd'] for step in item['gain_steps']),item
        with (folder/item['record_path']).open() as stream:rows=list(csv.DictReader(stream))
        assert rows and all(None not in row and None not in row.values() for row in rows),item
        counts=[];pending=[]
        for axis,joint in enumerate(names):
            rr=[row for row in rows if row['joint']==joint]
            assert len(rr)>=10,joint
            assert abs(float(rr[0]['reference'])-item['fresh_origin_rad'][axis])<1e-8,(joint,item)
            # Both the initial fresh-origin command and final target must
            # complete a real cycle before the recorder advances or freezes.
            assert item['reference_rad'][axis]==item['selected_center_rad'][axis],item
            final_pending=abs(float(rr[-1]['reference'])-item['selected_center_rad'][axis])
            assert final_pending<1e-7,(joint,final_pending,item)
            pending.append(final_pending)
            lo=min(item['fresh_origin_rad'][axis],demonstrated[axis][0])
            hi=max(item['fresh_origin_rad'][axis],demonstrated[axis][1])
            assert all(lo-1e-8<=float(row['reference'])<=hi+1e-8 for row in rr),joint
            assert all(int(row['limited'])==0 and int(row['actual_gains_known'])==1 for row in rr),joint
            assert max(abs(float(row['reference_velocity'])) for row in rr)<=item['max_velocity_rad_s'][axis]+1e-6,joint
            assert max(abs(float(row['reference'])-float(row['actual'])) for row in rr)<=item['feedback_lead_rad'][axis]+item['position_uncertainty_rad'][axis]+1e-6,joint
            counts.append(len(rr))
        measured.append({'direction':item['direction'],'record_path':item['record_path'],
                         'fresh_origin_rad':item['fresh_origin_rad'],'selected_center_rad':item['selected_center_rad'],
                         'elapsed_s':item['elapsed_s'],'samples_per_axis':counts,'gain_steps':len(item['gain_steps']),
                         'pending_final_reference_step_rad':pending})
    assert (folder/'gain-native-measurement-center-noise.csv').is_file(),records
    return entering,measured


def probe_case(terminal,directory,widen):
    config,names,lower=config_case(directory,0,widen)
    storage=directory/'storage';storage.mkdir()
    native=Native(terminal,config,storage)
    try:
        native.request('activate');native.ready()
        raw_bounds=source_path(native,directory/'source',names,lower+.01)
        native.ready()
        task=native.request('model_calibration_import_trajectory',{'directory':str(directory/'source'),'pose_budget':5})
        demonstrated=checked_bounds(Path(task['directory'])/'trajectory.csv',names)
        native.ready()
        native.request('model_calibration_start',{'task_id':task['task_id'],'workflow':'gain','gain_mode':'rigid',
            'gain_tuning_mode':'fine','released':True,'gain_trajectory_confirmed':True,'gain_motion_range_confirmed':True})
        folder=Path(task['directory']);disturbance='servo-probe-native-established-disturbance.csv'
        deadline=time.monotonic()+90
        disturbance_duration=None
        while time.monotonic()<deadline:
            status=native.request('model_calibration_status')
            if status.get('gain_probe_kind')=='native-established-disturbance':
                disturbance_duration=status.get('gain_probe_duration_s')
            target=folder/disturbance
            if target.exists():
                with target.open() as stream:rows=list(csv.DictReader(stream))
                if disturbance_duration and rows and all(None not in row and None not in row.values() for row in rows):
                    if all(len(rr:=[row for row in rows if row['joint']==joint])>=100
                           and max(float(row['time_s']) for row in rr)>=disturbance_duration-.08 for joint in names):
                        break
            assert status['phase'] not in ('complete','cancelled','failed'),status
            assert native.request('status')['robot_state']=='ACTIVE'
            time.sleep(.05)
        else:
            raise AssertionError('Native worker did not finish the mixed eligible/masked disturbance')
        if not widen:
            # Continue the genuine worker until its first full-path entrance.
            # Cancellation then persists the completed transition and partial
            # path through the same normal recording/exit path
            deadline=time.monotonic()+150
            while time.monotonic()<deadline:
                status=native.request('model_calibration_status')
                assert status['phase'] not in ('complete','cancelled','failed'),status
                assert native.request('status')['robot_state']=='ACTIVE'
                if (status.get('gain_probe_kind')=='native-frozen'
                    and status.get('gain_probe_message')=='trajectory_replay'
                    and status.get('gain_probe_elapsed_s',0)>.1):
                    break
                time.sleep(.05)
            else:
                raise AssertionError('Native worker did not reach the first feedback-bounded full-path transition')
        candidate,status=wait_saved_stop(native,task)
        parameters={item['record_path']:item for item in candidate['experiments'] if item.get('kind')=='short_experiment_parameters'}
        reference=next(item for item in parameters.values() if item['label'].startswith('native-establish-'))
        assert reference['position_domain_source']=='checked_demonstration_and_fresh_feedback_intersect_explicit_safety',reference
        assert len(reference['plans'])==len(names),reference
        entering,routes=check_measurement_routes(folder,candidate,names,demonstrated)
        assert any(abs(value)>1e-3 for value in reference['center_rad']),reference
        assert reference['center_source']=='fresh_measured_establishment_center',reference
        assert all(item['center_rad']==reference['center_rad'] for item in parameters.values()
                   if item['label'].startswith('native-establish-')),parameters
        for axis in range(len(names)):
            assert abs(reference['demonstrated_path_lower_rad'][axis]-demonstrated[axis][0])<1e-6,reference
            assert abs(reference['demonstrated_path_upper_rad'][axis]-demonstrated[axis][1])<1e-6,reference
        with (folder/reference['record_path']).open() as stream:ref_rows=list(csv.DictReader(stream))
        domains=[]
        for axis,joint in enumerate(names):
            rr=[row for row in ref_rows if row['joint']==joint]
            domain=reference['working_domain'][axis]
            lo,hi=domain['reference_lower_rad'],domain['reference_upper_rad'];domains.append([lo,hi])
            assert lo>=min(reference['center_rad'][axis],demonstrated[axis][0])-1e-6 and hi<=max(reference['center_rad'][axis],demonstrated[axis][1])+1e-6,domain
            assert min(float(row['reference']) for row in rr)>=lo-1e-8,joint
            assert max(float(row['reference']) for row in rr)<=hi+1e-8,joint
            assert min(float(row['reference']) for row in rr)<max(float(row['reference']) for row in rr),joint
            assert abs(float(rr[0]['actual'])-reference['center_rad'][axis])<.002,(joint,reference)
        applied=parameters[disturbance]
        mask=applied['disturbance_axis_eligible']
        assert len(mask)==len(names) and any(mask),mask
        with (folder/disturbance).open() as stream:drows=list(csv.DictReader(stream))
        models=next(item for item in candidate['experiments'] if item.get('kind')=='independent_torque_disturbance' and item.get('record_path')==disturbance)
        model_axes=[axis for axis in models['axes'] if 'disturbance_valid' in axis]
        for axis,joint in enumerate(names):
            rr=[row for row in drows if row['joint']==joint]
            assert len(rr)>=100 and max(float(row['time_s']) for row in rr)>=applied['duration_s']+1.5-.08,joint
            assert all(int(row['limited'])==0 for row in rr),joint
            peak=max(abs(float(row['independent_disturbance_nm'])) for row in rr)
            if mask[axis]:assert peak>0,joint
            else:
                assert peak==0,joint
                assert model_axes[axis]['reason']=='independent_disturbance_not_requested_without_bidirectional_demonstrated_room',model_axes[axis]
                assert model_axes[axis]['disturbance_valid'] is False,model_axes[axis]
        assert not candidate['validated'],candidate['validated']
        transition=check_endpoint_transition(folder,candidate,names) if not widen else None
        if not widen:
            assert any(item['direction']=='return_to_demonstrated_endpoint' for item in routes),routes
        return {'wider_urdf_limits':widen,'demonstrated_bounds_rad':demonstrated,'reference_domains_rad':domains,
                'raw_input_bounds_rad':raw_bounds,
                'measurement_center_rad':reference['center_rad'],'measurement_routes':routes,
                'bias_rad':[plan['reference_bias_rad'] for plan in reference['plans']],
                'disturbance_axis_eligible':mask,'worker_phase':status['phase'],'endpoint_transition':transition}
    except Exception:
        print(native.diagnostics(),file=sys.stderr)
        raise
    finally:native.close()


def margin_case(terminal,directory,outside_hard):
    config,names,lower=config_case(directory,.02,False)
    storage=directory/'storage';storage.mkdir();native=Native(terminal,config,storage)
    try:
        native.request('activate');native.ready()
        source_path(native,directory/'source',names,lower-.001 if outside_hard else lower+.01)
        original=(directory/'source/trajectory.csv').read_bytes()
        native.ready()
        try:
            task=native.request('model_calibration_import_trajectory',{'directory':str(directory/'source'),'pose_budget':5})
        except RuntimeError as error:
            text=str(error)
            assert 'trajectory_range_violation:joint=joint2;' in text,text
            assert (directory/'source/trajectory.csv').read_bytes()==original
            if outside_hard:
                assert ';scope=recorded_position' in text and ';margin=0;' in text,text
            else:
                assert ';scope=command_reference' in text and ';margin=0.02' in text,text
            return {'outside_hard':outside_hard,'rejection_stage':'import_planning',
                    'original_raw_bytes_unchanged':True,'diagnostic':text}
        assert not outside_hard,'Raw input outside the declared hard range was accepted'
        try:
            native.request('model_calibration_start',{'task_id':task['task_id'],'workflow':'gain','gain_mode':'rigid',
                'gain_tuning_mode':'fine','released':True,'gain_trajectory_confirmed':True,'gain_motion_range_confirmed':True})
        except RuntimeError as error:
            text=str(error)
            assert 'trajectory_range_violation:joint=joint2;' in text and ';scope=command_reference' in text,text
            assert ';margin=0.02' in text,text
            assert (directory/'source/trajectory.csv').read_bytes()==original
            return {'outside_hard':False,'rejection_stage':'start','raw_import_accepted':True,
                    'original_raw_bytes_unchanged':True,'diagnostic':text}
        raise AssertionError('Replay inside hard bounds but outside the explicit command margin was accepted')
    finally:native.close()


def teach_zero_case(terminal,directory):
    """Exercise record_stop itself at a declared hard zero, then save/load."""
    config,names,_=config_case(directory,0,False)
    document=yaml.safe_load(config.read_text())
    model=ET.parse(document['model']['urdf_path'])
    for joint in model.getroot().findall('joint'):
        if joint.get('name') in names[1:3]:
            joint.find('limit').set('lower','0')
    copied=directory/'hard-zero.urdf'
    model.write(copied,encoding='utf-8',xml_declaration=True)
    document['model']['urdf_path']=str(copied)
    config.write_text(json.dumps(document))
    storage=directory/'storage';storage.mkdir();native=Native(terminal,config,storage)
    try:
        native.request('activate');native.ready()
        started=native.request('teach_replay_record_begin')
        assert started['phase']=='recording',started
        deadline=time.monotonic()+1.2
        while time.monotonic()<deadline:
            status=native.request('status')
            assert status['robot_state']=='ACTIVE' and status['write_enabled'] is False,status
            time.sleep(.02)
        stopped=native.request('teach_replay_record_stop')
        assert stopped['phase']=='ready' and stopped['has_trajectory'] and not stopped['error'],stopped
        assert stopped['samples']>=22 and not stopped['fault_interrupted'],stopped
        native.ready()
        path=directory/'recorded-hard-zero.csv'
        native.request('teach_replay_save',{'path':str(path)})
        original=path.read_bytes()
        with path.open() as stream:
            rows=csv.reader(stream);interval=next(rows);header=next(rows);positions=list(rows)
        assert interval[0]=='sample_dt' and header==names,(interval,header)
        assert len(positions)==stopped['samples']>=22,stopped
        for axis in (1,2):
            assert all(float(row[axis])==0 for row in positions),names[axis]
        loaded=native.request('teach_replay_load',{'path':str(path)})
        assert loaded['phase']=='ready' and loaded['samples']==stopped['samples'] and not loaded['error'],loaded
        assert all(loaded['start'][axis]==loaded['end'][axis]==0 for axis in (1,2)),loaded
        assert path.read_bytes()==original
        return {'status':'passed','checks':['record_stop_accepts_fresh_zero_at_declared_hard_lower_bound',
            'at_least_twenty_actual_recorded_samples_preserve_raw_zero',
            'saved_raw_zero_loads_without_hidden_margin_and_without_rewrite'],
            'recorded_samples':len(positions),'recorded_duration_s':stopped['duration_s'],
            'joint2_joint3_lower_rad':0,'explicit_command_margin_rad':0,
            'saved_joint2_joint3_values_rad':[0,0],'robot_state':native.request('status')['robot_state'],
            'scope':'actual_native_record_stop_save_load_with_temporary_write_disabled_mock'}
    finally:native.close()


def run(terminal):
    with tempfile.TemporaryDirectory(prefix='serialarm-probe-boundaries-') as folder:
        root=Path(folder)
        normal=probe_case(terminal,root/'normal',False)
        wide=probe_case(terminal,root/'wide',True)
        assert normal['measurement_routes'][0]['selected_center_rad']==wide['measurement_routes'][0]['selected_center_rad'],(normal,wide)
        # Fresh feedback after a real transfer may differ by a control cycle.
        # The authoritative demonstration/Safety domain must remain unchanged;
        # compare each local range relative to its newly measured center.
        assert normal['demonstrated_bounds_rad']==wide['demonstrated_bounds_rad'],(normal,wide)
        for result in (normal,wide):
            for axis,(lo,hi) in enumerate(result['reference_domains_rad']):
                center=result['measurement_center_rad'][axis]
                assert lo>=min(center,result['demonstrated_bounds_rad'][axis][0])-1e-6
                assert hi<=max(center,result['demonstrated_bounds_rad'][axis][1])+1e-6
        margin=margin_case(terminal,root/'margin',False)
        hard=margin_case(terminal,root/'hard',True)
        teach=teach_zero_case(terminal,root/'teach-zero')
        return {'status':'passed','checks':['native_biased_probes_stay_inside_checked_demonstration_plus_fresh_position',
            'fresh_legal_position_near_the_old_endpoint_does_not_require_exact_alignment',
            'widening_URDF_limits_does_not_widen_demonstration_bound_probe_domain',
            'disturbance_mask_follows_the_actual_measurement_center_and_preserves_missing_evidence',
            'demonstrated_entry_remeasures_the_complete_group_and_returns_before_full_path',
            'full_path_endpoint_transition_starts_at_fresh_actual_with_bounded_planning_steps_and_PD_error',
            'raw_import_uses_hard_range_and_replay_uses_explicit_command_margin',
            'raw_outside_hard_range_is_rejected_with_axis_sample_and_scope',
            'record_stop_save_and_load_accept_actual_zero_at_declared_hard_lower_bound',
            'normal_worker_stop_does_not_promote_missing_disturbance_or_purpose_evidence_to_pass'],
            'normal':normal,'wide':wide,'explicit_margin':margin,'hard_range':hard,
            'teach_zero':teach,
            'scope':'actual_native_mock_no_hardware_no_physical_stop_or_identification_claim'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal',type=Path,default=ROOT/'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    parser.add_argument('--teach-zero-only',action='store_true',help='Only run the few-second record_stop/save/load hard-zero case')
    args=parser.parse_args();terminal=args.terminal.resolve()
    if not terminal.is_file() or not os.access(terminal,os.X_OK):parser.error('--terminal must be an executable file')
    if args.teach_zero_only:
        with tempfile.TemporaryDirectory(prefix='serialarm-teach-hard-zero-') as folder:
            report=teach_zero_case(terminal,Path(folder)/'case')
    else:
        report=run(terminal)
    print(json.dumps(report,indent=2))
