"""Exercise native teaching fault cleanup through the actual JSON process API"""
from pathlib import Path
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import yaml

TERMINAL,PLUGIN,REPO=sys.argv[1:4]

class Terminal:
    def __init__(self,config,hardware,storage):
        env=dict(os.environ,SERIAL_ARM_PROFILE_STORAGE=str(storage))
        self.proc=subprocess.Popen([TERMINAL,'--machine','--config',str(config),
            '--hardware-plugin',PLUGIN,'--hardware-config',str(hardware)],env=env,
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
        native.request('activate');native.wait(lambda s:s.get('valid') is True)
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
