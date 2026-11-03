"""Exercise native teaching fault cleanup through the actual JSON process API"""
from pathlib import Path
import json
import math
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
import yaml

TERMINAL,PLUGIN,REPO=sys.argv[1:4]

class Terminal:
    def __init__(self,config,hardware,storage,overrides=()):
        env=dict(os.environ,SERIAL_ARM_PROFILE_STORAGE=str(storage))
        self.proc=subprocess.Popen([TERMINAL,'--machine','--config',str(config),
            '--hardware-plugin',PLUGIN,'--hardware-config',str(hardware),*overrides],env=env,
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        self.messages=queue.Queue();self.logs=[];self.number=0;self.replies={}
        def stdout():
            for line in self.proc.stdout:
                try:self.messages.put(json.loads(line))
                except json.JSONDecodeError:self.logs.append(line)
        def stderr():
            for line in self.proc.stderr:self.logs.append(line)
        self.reader=threading.Thread(target=stdout,daemon=True);self.reader.start()
        self.errors=threading.Thread(target=stderr,daemon=True);self.errors.start()
    def send(self,method,params=None):
        self.number+=1;number=self.number
        self.proc.stdin.write(json.dumps({'id':number,'method':method,'params':params or {}})+'\n');self.proc.stdin.flush()
        return number
    def reply(self,number,method='request',expected_error=None):
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:
            try:item=self.replies.pop(number,None) or self.messages.get(timeout=.2)
            except queue.Empty:
                if self.proc.poll() is not None:raise AssertionError('Terminal exited: '+''.join(self.logs)[-2000:])
                continue
            if item.get('id')!=number:
                if 'id' in item:self.replies[item['id']]=item
                continue
            if expected_error:
                assert expected_error in item.get('error',''),item
                return item['error']
            if item.get('error'):raise AssertionError(f'{method}: {item["error"]}')
            return item.get('result')
        raise AssertionError(f'{method} timed out: '+''.join(self.logs)[-2000:])
    def request(self,method,params=None):
        return self.reply(self.send(method,params),method)
    def wait(self,predicate):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            state=self.request('status')
            if predicate(state):return state
            time.sleep(.02)
        raise AssertionError('expected state not reached: '+json.dumps(state))
    def close(self):
        if self.proc.poll() is None:self.proc.kill()
        self.proc.wait(timeout=3);self.reader.join(1);self.errors.join(1)
        for stream in (self.proc.stdin,self.proc.stdout,self.proc.stderr):stream.close()

with tempfile.TemporaryDirectory(prefix='serialarm-teaching-fault-') as temporary:
    folder=Path(temporary);source=Path(REPO)/'src/robot_supports/robots/dm_arm/description/config/core/gray.yaml'
    data=yaml.safe_load(source.read_text());names=data['model']['joint_names']
    data['model']['urdf_path']=str((source.parent/data['model']['urdf_path']).resolve())
    data['control']['runtime']['write_enabled']=True
    data['control']['runtime']['model_feedforward_mode']='GRAVITY'
    data['capability']['admittance']['enabled']=False
    data['shutdown']['park_before_disable']=False
    # Give CI scheduling the existing watchdog margin while testing feedback freshness
    data['safety_policy']['max_dt_s']=.2
    config=folder/'core.yaml';config.write_text(yaml.safe_dump(data,sort_keys=False))
    positions=[]
    for name in names:
        m=data['calibration']['joints'][name]
        positions.append(m['actuator_zero_offset']+m['direction']*m['pos_ratio']*(data['shutdown']['park_pos'][name]-m['joint_zero_offset']))
    trigger=folder/'stale-feedback'
    bus_config=yaml.safe_load((Path(REPO)/'src/robot_supports/robots/dm_arm/description/config/hardware.yaml').read_text())
    bus_config['buses']['main_can']['device']=str(folder/'unused-mock-device')
    bus_config.update(joints=names,positions=positions,timestamped=True,fault_trigger=str(trigger))
    hardware=folder/'hardware.yaml';hardware.write_text(yaml.safe_dump(bus_config))
    native=Terminal(config,hardware,folder/'storage')
    try:
        behavior=native.request('set_reference_behavior',{'behavior':'NATIVE_IMPEDANCE'})
        assert behavior['reference_behavior']=='NATIVE_IMPEDANCE',behavior
        native.request('activate');snapshot=native.wait(lambda s:s.get('valid') is True)
        assert snapshot['monotonic_ns']>0 and snapshot['reference_behavior']=='NATIVE_IMPEDANCE',snapshot
        assert len(snapshot['joint']['accepted_ref_pos'])==len(names),snapshot
        assert 'observer_valid' in snapshot and 'parameter_update' in snapshot,snapshot
        assert snapshot['gain_profile']=='RIGID' and snapshot['gain_source'] in ('CONFIGURED','TRANSITION'),snapshot
        assert len(snapshot['hold_target_pos'])==len(names) and len(snapshot['effective_kp'])==len(names),snapshot
        gains=native.request('get_admittance')
        assert set(gains['execution_gains'])=={'rigid_hold','rigid_tracking','compliant_hold','compliant_tracking','compliant_drag','servo_tracking'},gains
        assert gains['hold_compensation']['enabled'] is False,gains
        native.request('hold')
        target=list(snapshot['joint']['accepted_ref_pos'])
        bad=list(target);bad[0]=1e9
        native.reply(native.send('move_to',{'positions':bad,'speed_scale':.1}),expected_error='position limit')
        assert native.request('status')['impedance_mode']=='RIGID_HOLD'
        native.request('move_to',{'positions':target,'speed_scale':.1})
        assert native.request('status')['impedance_mode']=='RIGID_TRACKING'
        native.request('hold')
        native.reply(native.send('set_reference_behavior',{'behavior':'MODEL_ADMITTANCE'}),expected_error='NOT_INACTIVE')
        task=native.request('model_calibration_teach_begin')
        assert task['phase']=='teaching',task
        time.sleep(.15);trigger.touch()
        fault=native.wait(lambda s:s.get('robot_state')=='FAULT')
        assert fault['fault']['safety_code']=='STATE_TIMEOUT',fault
        assert fault['fault']['joint_name']==names[0],fault
        assert fault['fault']['feedback_age_source']=='host_transport_receive',fault
        native.wait(lambda s:s.get('model_calibration',{}).get('phase')=='failed')
        for _ in range(2):
            stopped=native.request('model_calibration_teach_stop',{'task_id':task['task_id']})
            assert stopped['fault_interrupted'] is True,stopped
            assert 'STATE_TIMEOUT' in stopped['error'] and 'RIGID_HOLD' not in stopped['error'],stopped
        assert native.request('status')['robot_state']=='FAULT'
        trigger.unlink();native.request('deactivate');native.request('activate')
        native.wait(lambda s:s.get('valid') is True)
        second=native.request('model_calibration_teach_begin')
        assert second['task_id']!=task['task_id'] and second['phase']=='teaching',second
        time.sleep(.15)
        second=native.request('model_calibration_status')
        assert second['phase']=='teaching' and second['fault_interrupted'] is False,second
        native.request('model_calibration_cancel',{'task_id':second['task_id']})
        native.request('deactivate');native.request('shutdown');native.proc.wait(timeout=3)
        assert Path(task['directory']).is_dir()
        assert (Path(task['directory'])/'result.json').is_file()
        reconnected=Terminal(config,hardware,folder/'storage')
        try:
            initial=reconnected.request('status')
            assert initial['model_calibration']['phase']=='idle',initial
            assert not initial['model_calibration'].get('error'),initial
            reconnected.request('activate');reconnected.wait(lambda s:s.get('valid') is True)
            third=reconnected.request('model_calibration_teach_begin')
            assert third['phase']=='teaching' and not third['fault_interrupted'],third
            reconnected.request('model_calibration_cancel',{'task_id':third['task_id']})
            reconnected.request('deactivate');reconnected.request('shutdown');reconnected.proc.wait(timeout=3)
        finally:reconnected.close()
        print('TERMINAL_TEACHING_FAULT_CLEANUP_AND_RESTART_PASS')
    finally:native.close()

    data['shutdown']['park_before_disable']=True
    data['shutdown']['timeout_s']=1.6
    data['shutdown']['compliant_phase_ratio']=.75
    config.write_text(yaml.safe_dump(data,sort_keys=False))
    parking=Terminal(config,hardware,folder/'park-storage')
    try:
        parking.request('activate');parking.wait(lambda s:s.get('valid') is True)
        for command,expected_state in [('hold','ACTIVE'),('deactivate','INACTIVE')]:
            if parking.request('status')['robot_state']=='INACTIVE':
                parking.request('activate');parking.wait(lambda s:s.get('valid') is True)
            pending=parking.send('park')
            parking.wait(lambda s:s.get('impedance_mode')=='COMPLIANT_TRACKING')
            started=time.monotonic();parking.request(command)
            assert time.monotonic()-started<.8,'stop waited for park timeout'
            parking.reply(pending,'park',expected_error='park interrupted by operator')
            state=parking.request('status')
            assert state['robot_state']==expected_state,state
            if command=='hold':assert state['impedance_mode']=='RIGID_HOLD',state
            time.sleep(.1)
            assert parking.request('status')['robot_state']==expected_state
        parking.request('activate');parking.wait(lambda s:s.get('valid') is True)
        pending=parking.send('shutdown')
        parking.wait(lambda s:s.get('impedance_mode')=='COMPLIANT_TRACKING')
        parking.request('hold')
        parking.reply(pending,'shutdown',expected_error='park interrupted by operator')
        assert parking.request('status')['robot_state']=='ACTIVE'
        assert parking.proc.poll() is None
        parking.request('deactivate');parking.request('shutdown');parking.proc.wait(timeout=3)
        print('TERMINAL_PARK_AND_SHUTDOWN_INTERRUPTION_PASS')
    finally:parking.close()

    disconnected=Terminal(config,hardware,folder/'eof-storage')
    try:
        disconnected.request('activate');disconnected.wait(lambda s:s.get('valid') is True)
        disconnected.send('park');disconnected.wait(lambda s:s.get('impedance_mode')=='COMPLIANT_TRACKING')
        disconnected.proc.stdin.close();disconnected.proc.wait(timeout=4)
        assert disconnected.proc.returncode==0,disconnected.logs
        print('TERMINAL_PARK_EOF_CLEANUP_PASS')
    finally:disconnected.close()

    # The actual process API keeps purpose, task and reference law independent
    # and restores temporary teaching choices after both success and a real fault
    data['shutdown']['park_before_disable']=False
    data['control']['runtime']['model_feedforward_mode']='FULL_INVERSE_DYNAMICS'
    data['capability']['admittance']['enabled']=True
    data['capability']['admittance']['reference_behavior']='NATIVE_IMPEDANCE'
    data['control']['hold_compensation']={'enabled':True,
        **{key:{name:value for name in names} for key,value in
           [('integral_gain',.15),('max_torque',.15),('max_rate',.05),('contact_threshold',.2)]},
        'velocity_threshold':.02,'error_threshold':.03}
    config.write_text(yaml.safe_dump(data,sort_keys=False))
    control=Terminal(config,hardware,folder/'purpose-storage')
    try:
        control.request('activate');control.wait(lambda s:s.get('valid') is True)
        control.request('set_impedance_mode',{'mode':'RIGID_HOLD','gain_profile':'SERVO_TRACKING'})
        servo=control.wait(lambda s:s.get('gain_profile')=='SERVO_TRACKING')
        control.request('hold')
        assert control.wait(lambda s:s.get('valid') is True)['gain_profile']=='SERVO_TRACKING'
        control.reply(control.send('set_impedance_mode',{'mode':'COMPLIANT_TRACKING','gain_profile':'SERVO_TRACKING'}),expected_error='does not match')
        assert control.request('status')['gain_profile']=='SERVO_TRACKING'
        control.request('set_impedance_mode',{'mode':'COMPLIANT_DRAG','gain_profile':'DRAG'})
        dragging=control.wait(lambda s:s.get('valid') is True)
        assert dragging['model_feedforward_mode']=='GRAVITY' and dragging['reference_behavior']=='NATIVE_IMPEDANCE',dragging
        control.request('hold')
        restored=control.wait(lambda s:s.get('valid') is True)
        assert restored['model_feedforward_mode']=='FULL_INVERSE_DYNAMICS' and restored['gain_profile']=='SERVO_TRACKING',restored
        control.request('teach_replay_record_begin');time.sleep(.55)
        stopped=control.request('teach_replay_record_stop')
        assert stopped['phase']=='ready' and stopped['has_trajectory'] is True,stopped
        restored=control.wait(lambda s:s.get('valid') is True)
        assert restored['model_feedforward_mode']=='FULL_INVERSE_DYNAMICS' and restored['gain_profile']=='SERVO_TRACKING',restored
        control.request('teach_replay_record_begin');time.sleep(.55);trigger.touch()
        fault=control.wait(lambda s:s.get('robot_state')=='FAULT')
        failed=control.request('teach_replay_record_stop')
        assert failed['phase']=='failed' and failed['fault_interrupted'] and failed['has_trajectory'],failed
        assert 'STATE_TIMEOUT' in failed['error'],failed
        assert control.request('status')['model_feedforward_mode']=='FULL_INVERSE_DYNAMICS'
        trigger.unlink();control.request('deactivate');control.request('activate')
        recovered=control.wait(lambda s:s.get('valid') is True)
        assert recovered['gain_profile']=='SERVO_TRACKING',recovered
        path=folder/'purpose-trajectory.csv';center=recovered['joint']['pos']
        rows=[]
        # Two small excursions give enough demonstrated arc length while
        # returning to the same physical endpoint for independent retry/MDK runs
        for k in range(41):
            row=list(center);row[0]+=.015*math.sin(4*math.pi*k/40);rows.append(row)
        path.write_text('sample_dt,0.02\n'+','.join(names)+'\n'+'\n'.join(','.join(map(str,row)) for row in rows)+'\n')
        control.request('teach_replay_load',{'path':str(path)})
        params={'direction':'forward','rate':.2,'cycles':1,'confirmed':True,'mode':'RIGID_TRACKING',
                'reference_behavior':'NATIVE_IMPEDANCE','gain_profile':'SERVO_TRACKING'}
        control.request('teach_replay_play',params)
        playing=control.wait(lambda s:s.get('teach_replay',{}).get('phase')=='playing' and s.get('gain_profile')=='SERVO_TRACKING')
        assert playing['teach_replay']['reference_behavior']=='NATIVE_IMPEDANCE',playing
        trigger.touch();control.wait(lambda s:s.get('robot_state')=='FAULT')
        failed=control.wait(lambda s:s.get('teach_replay',{}).get('phase')=='failed')['teach_replay']
        assert failed['has_trajectory'] and failed['fault_interrupted'] and 'STATE_TIMEOUT' in failed['error'],failed
        trigger.unlink();control.request('deactivate');control.request('activate');control.wait(lambda s:s.get('valid') is True)
        retried=control.request('teach_replay_play',params)
        assert retried['gain_profile']=='SERVO_TRACKING',retried
        restored=control.wait(lambda s:s.get('teach_replay',{}).get('phase')=='ready' and s.get('valid') is True)
        assert restored['gain_profile']=='SERVO_TRACKING' and restored['model_feedforward_mode']=='FULL_INVERSE_DYNAMICS',restored
        params['reference_behavior']='MODEL_ADMITTANCE'
        control.request('teach_replay_play',params)
        model=control.wait(lambda s:s.get('teach_replay',{}).get('phase')=='playing' and s.get('admittance_active') is True)
        assert model['gain_profile']=='SERVO_TRACKING' and model['teach_replay']['selected_reference_behavior']=='MODEL_ADMITTANCE',model
        control.request('teach_replay_stop')
        restored=control.wait(lambda s:s.get('valid') is True)
        assert restored['reference_behavior']=='NATIVE_IMPEDANCE' and restored['gain_profile']=='SERVO_TRACKING',restored
        selected=control.request('get_admittance')
        assert selected['reference_behavior']=='NATIVE_IMPEDANCE' and selected['enabled'] is True,selected
        control.request('deactivate')
        hash_value=1469598103934665603
        for byte in Path(data['model']['urdf_path']).read_bytes():hash_value=((hash_value^byte)*1099511628211)&((1<<64)-1)
        gains=selected['execution_gains']['rigid_tracking']
        candidate={'schema':'serial_arm_gain_candidate','validated':True,'category':'rigid_tracking',
            'urdf_fingerprint':format(hash_value,'016x'),'joint_names':names,**gains,
            'hold_gains':gains,'hold_verification':{'validated':True}}
        candidate_path=folder/'baseline-gain.json';candidate_path.write_text(json.dumps(candidate))
        control.request('candidate_gain_apply',{'path':str(candidate_path)})
        assert control.request('get_admittance')['hold_compensation']['enabled'] is False
        control.request('candidate_gain_restore')
        assert control.request('get_admittance')['hold_compensation']['enabled'] is True
        control.request('shutdown');control.proc.wait(timeout=3)
        print('TERMINAL_PURPOSE_REPLAY_AND_TEACHING_CONTEXT_PASS')
    finally:control.close()

    # All drag entrypoints enforce the same prerequisites before changing any
    # strategy, command source or recorded task state.
    for invalid,expected in [('gravity','nonzero gravity compensation'),('damping','positive damping')]:
        rejected=yaml.safe_load(source.read_text())
        rejected['model']['urdf_path']=str((source.parent/rejected['model']['urdf_path']).resolve())
        rejected['capability']['admittance']['reference_behavior']='NATIVE_IMPEDANCE'
        rejected['shutdown']['park_before_disable']=False
        rejected['safety_policy']['max_dt_s']=.2
        if invalid=='gravity':rejected['model']['gravity_scale']={name:0. for name in names}
        else:rejected['control']['controller']['compliant_drag']['kd']={name:0. for name in names}
        config.write_text(yaml.safe_dump(rejected,sort_keys=False))
        store=folder/('rejected-'+invalid)
        guarded=Terminal(config,hardware,store)
        try:
            guarded.request('activate');before=guarded.wait(lambda s:s.get('valid') is True)
            original_task=guarded.request('model_calibration_status')
            for method,params in [('teach_replay_record_begin',{}),('model_calibration_teach_begin',{}),
                                  ('set_impedance_mode',{'mode':'COMPLIANT_DRAG','gain_profile':'DRAG'})]:
                guarded.reply(guarded.send(method,params),expected_error=expected)
                after=guarded.request('status')
                assert after['robot_state']=='ACTIVE' and after['impedance_mode']==before['impedance_mode'],after
                assert after['reference_behavior']=='NATIVE_IMPEDANCE' and after['gain_profile']==before['gain_profile'],after
                task=guarded.request('model_calibration_status')
                assert task['phase']==original_task['phase'] and task['task_id']==original_task['task_id'],task
            assert not store.exists() or not list(store.glob('model-calibration/*'))
            guarded.request('deactivate');guarded.request('shutdown');guarded.proc.wait(timeout=3)
        finally:guarded.close()
    print('TERMINAL_DRAG_ADMISSION_ATOMICITY_PASS')

    # The human terminal's advertised replacement YAML must retain a Native
    # selection even when external-torque observation is enabled.
    printable=yaml.safe_load(source.read_text())
    printable['model']['urdf_path']=str((source.parent/printable['model']['urdf_path']).resolve())
    printable['control']['runtime']['write_enabled']=False
    printable['capability']['admittance']['enabled']=True
    printable['capability']['admittance']['reference_behavior']='NATIVE_IMPEDANCE'
    printable['shutdown']['park_before_disable']=False
    config.write_text(yaml.safe_dump(printable,sort_keys=False))
    process=subprocess.run([TERMINAL,'--config',str(config),'--hardware-plugin',PLUGIN,'--hardware-config',str(hardware)],
        input='6\n1\n3\n3\n0\n0\n0\n',text=True,capture_output=True,timeout=8)
    assert process.returncode==0,process.stderr
    block=re.search(r'(capability:\n(?: +.*\n)+)',process.stdout)
    assert block,process.stdout[-2500:]
    capability=yaml.safe_load(block.group(1))['capability']
    assert capability['admittance']['enabled'] is True and capability['admittance']['reference_behavior']=='NATIVE_IMPEDANCE'
    printable['capability']=capability;config.write_text(yaml.safe_dump(printable,sort_keys=False))
    roundtrip=Terminal(config,hardware,folder/'yaml-roundtrip')
    try:
        assert roundtrip.request('get_admittance')['reference_behavior']=='NATIVE_IMPEDANCE'
        roundtrip.request('shutdown');roundtrip.proc.wait(timeout=3)
    finally:roundtrip.close()
    print('TERMINAL_CAPABILITY_YAML_STRATEGY_ROUNDTRIP_PASS')

    # Evidence belongs to the configuration actually loaded, including connection
    # overrides. Later edits to a source file must not change that task context.
    provenance=yaml.safe_load(source.read_text())
    provenance['model']['urdf_path']=str((source.parent/provenance['model']['urdf_path']).resolve())
    provenance['capability']['admittance']['reference_behavior']='NATIVE_IMPEDANCE'
    provenance['shutdown']['park_before_disable']=False
    provenance['safety_policy']['max_dt_s']=.2
    config.write_text(yaml.safe_dump(provenance,sort_keys=False))
    original_core=config.read_bytes();original_hardware=hardware.read_bytes()
    override_port=str(folder/'override-mock-device')
    evidence=Terminal(config,hardware,folder/'evidence-storage',('--serial-port',override_port,'--baudrate','115200','--bus','main_can'))
    try:
        evidence.request('activate');evidence.wait(lambda s:s.get('valid') is True)
        config.write_text(config.read_text()+'\n# external edit after connection\n')
        hardware.write_text(hardware.read_text()+'\n# external edit after connection\n')
        task=evidence.request('model_calibration_teach_begin')
        directory=Path(task['directory']);metadata=json.loads((directory/'metadata.json').read_text())
        assert (directory/'source/core.yaml').read_bytes()==original_core
        assert (directory/'source/hardware.yaml').read_bytes()==original_hardware
        assert metadata['hardware_config_path']==str(hardware)
        assert metadata['hardware_overrides']=={'serial_port':override_port,'baudrate':115200,'bus':'main_can'}
        def fnv(content):
            value=1469598103934665603
            for byte in content:value=((value^byte)*1099511628211)&0xffffffffffffffff
            return f'{value:016x}'
        assert metadata['core_fingerprint']==fnv(original_core)
        assert metadata['hardware_fingerprint']==fnv(original_hardware)
        effective=(directory/'source/hardware-effective.yaml').read_bytes()
        assert metadata['hardware_effective_fingerprint']==fnv(effective)
        bus=yaml.safe_load(effective)['buses']['main_can']
        assert bus['device']==override_port and bus['baudrate']==115200,bus
        evidence.request('model_calibration_cancel',{'task_id':task['task_id']})
        evidence.request('deactivate');evidence.request('shutdown');evidence.proc.wait(timeout=3)
    finally:
        evidence.close();hardware.write_bytes(original_hardware)
    print('TERMINAL_LOADED_HARDWARE_CONTEXT_AND_OVERRIDES_PASS')

    data['control']['runtime']['write_enabled']=False
    data['control']['runtime']['model_feedforward_mode']='NONE'
    data['shutdown']['park_before_disable']=False
    config.write_text(yaml.safe_dump(data,sort_keys=False))
    offline=Terminal(config,hardware,folder/'offline-storage')
    try:
        initial=offline.request('status')
        assert initial['robot_state']=='INACTIVE' and initial['write_enabled'] is False,initial
        capabilities=initial['actuator_info']
        assert [item['name'] for item in capabilities]==names,capabilities
        assert all(item['min_pos']<item['max_pos'] and all(item[key]>0 for key in ('max_vel','max_effort','max_kp','max_kd')) for item in capabilities),capabilities
        offline.request('activate')
        active=offline.wait(lambda s:s.get('valid') is True)
        assert active['robot_state']=='ACTIVE' and active['write_enabled'] is False,active
        assert active['actuator_info']==capabilities,active
        assert active['actuator_info_source']=='mock_backend_after_successful_activate',active
        offline.request('deactivate');offline.request('shutdown');offline.proc.wait(timeout=3)
        print('TERMINAL_OFFLINE_MOCK_CAPABILITIES_PASS')
    finally:offline.close()
