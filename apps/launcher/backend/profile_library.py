from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile

import yaml

from profiles import config_values, read_yaml, text


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    yaml.safe_load(temp.read_text())
    temp.replace(path)


def _safe_extract(source, destination):
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(source) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if destination not in target.parents and target != destination:
                raise ValueError('description archive contains an unsafe path')
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(member) as src, target.open('wb') as dst:
                    shutil.copyfileobj(src, dst)


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]


def _matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin_matrix(origin):
    r = _rpy_matrix(origin.get('rpy', [0, 0, 0])); x, y, z = origin.get('xyz', [0, 0, 0])
    return [[r[0][0],r[0][1],r[0][2],x],[r[1][0],r[1][1],r[1][2],y],[r[2][0],r[2][1],r[2][2],z],[0,0,0,1]]


def _quat_from_matrix(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = .25*s; x=(m[2][1]-m[1][2])/s; y=(m[0][2]-m[2][0])/s; z=(m[1][0]-m[0][1])/s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1+m[0][0]-m[1][1]-m[2][2])*2; w=(m[2][1]-m[1][2])/s; x=.25*s; y=(m[0][1]+m[1][0])/s; z=(m[0][2]+m[2][0])/s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1+m[1][1]-m[0][0]-m[2][2])*2; w=(m[0][2]-m[2][0])/s; x=(m[0][1]+m[1][0])/s; y=.25*s; z=(m[1][2]+m[2][1])/s
    else:
        s = math.sqrt(1+m[2][2]-m[0][0]-m[1][1])*2; w=(m[1][0]-m[0][1])/s; x=(m[0][2]+m[2][0])/s; y=(m[1][2]+m[2][1])/s; z=.25*s
    return [x,y,z,w]


class ProfileLibrary:
    def __init__(self, root, inspector):
        self.root = Path(root).resolve()
        self.inspector = inspector
        self.library_file = Path(os.environ.get('SERIAL_ARM_PROFILE_LIBRARY', Path.home() / '.config/serial-arm/profile-library.yaml')).expanduser()
        self.cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'serial-arm/description-preview'

    def _library_data(self):
        if not self.library_file.is_file(): return {'profiles': {}, 'hidden_builtin': []}
        data = yaml.safe_load(self.library_file.read_text()) or {}
        if not isinstance(data, dict): raise ValueError('Profile Library YAML must contain a mapping')
        profiles = data.get('profiles') or {}
        if not isinstance(profiles, dict): raise ValueError('Profile Library profiles must be a mapping')
        hidden = data.get('hidden_builtin') or []
        if not isinstance(hidden, list): hidden = []
        return {'profiles': profiles, 'hidden_builtin': hidden}

    def _write_library(self, data):
        _atomic_yaml(self.library_file, {'profiles': data.get('profiles', {}), 'hidden_builtin': sorted(set(data.get('hidden_builtin', [])))})

    def _readiness_path(self, profile_file, profile):
        safe=''.join(c if c.isalnum() or c in '_-' else '_' for c in str(profile)) or 'profile'
        profile_path = Path(profile_file).expanduser().resolve() if profile_file else None
        builtin_root = (self.root / 'src/robot_supports/profiles').resolve()
        if profile_path is None:
            return self.library_file.parent / 'readiness' / f'{safe}.yaml'
        try:
            profile_path.relative_to(builtin_root)
            return self.library_file.parent / 'readiness' / f'{safe}.yaml'
        except ValueError:
            return profile_path.parent / '.serialarm' / 'readiness' / f'{safe}.yaml'

    def _readiness(self, config):
        try:
            info = self.inspector.inspect(config)
        except Exception as error:
            return {'state':'configuring','next_stage':'profile_setup','checks':{},'reason':str(error),'fingerprint':''}
        checks = {item['name']: bool(item['ok']) for item in info.get('checks', [])}
        profile_setup = all(checks.get(k, False) for k in ('core','description','hardware')) and bool(info.get('hardware_plugin'))
        core = read_yaml(info['resources']['core']) if checks.get('core') else {}
        model = core.get('model') or {}; safety = core.get('safety_policy') or {}
        joints = model.get('joint_names') or []
        profile_setup = profile_setup and bool(joints) and bool(model.get('base_frame')) and bool(model.get('tool_frame'))
        gravity_valid = bool(((safety.get('fault_recovery') or {}).get('gravity_model_validated')))
        meta_path = self._readiness_path(info['profile_file'], info['profile'])
        meta = yaml.safe_load(meta_path.read_text()) if meta_path.is_file() else {}
        stages = meta.get('stages') if isinstance(meta, dict) else {}
        if not isinstance(stages, dict): stages = {}
        fingerprint = info['fingerprint']
        def evidence(name):
            item = stages.get(name) or {}
            if not isinstance(item, dict) or not item.get('confirmed'): return 'pending'
            return 'verified' if item.get('fingerprint') == fingerprint else 'stale'
        result = {
            'description': 'verified' if checks.get('description') and checks.get('core') else 'failed',
            'profile_setup': 'verified' if profile_setup else 'pending',
            'model_check': evidence('model_check'),
            'bringup': evidence('bringup'),
            'mapping_calibration': evidence('mapping_calibration'),
            'geometry_calibration': evidence('geometry_calibration'),
            'dynamics_calibration': 'verified' if gravity_valid else evidence('dynamics_calibration'),
            'validation': evidence('validation'),
        }
        required = ['description','profile_setup','model_check','bringup','mapping_calibration','geometry_calibration','dynamics_calibration','validation']
        stale = any(result[x] == 'stale' for x in required)
        missing = [x for x in required if result[x] not in ('verified',)]
        if not profile_setup: state = 'configuring'
        elif stale: state = 'needs_revalidation'
        elif not missing: state = 'ready'
        else: state = 'pending_validation'
        return {'state':state,'next_stage':missing[0] if missing else 'ready','checks':result,'reason':'','fingerprint':fingerprint,'info':info}

    def _builtin_readiness(self, config):
        readiness = self._readiness(config)
        if not readiness.get('info'):
            return readiness
        checks = {stage: 'verified' for stage in (
            'description', 'profile_setup', 'model_check', 'bringup',
            'mapping_calibration', 'geometry_calibration', 'dynamics_calibration', 'validation')}
        return {**readiness, 'state':'ready', 'next_stage':'ready', 'checks':checks, 'reason':''}

    def list(self):
        data = self._library_data(); items=[]
        builtin = self.inspector.profiles({})
        for p in builtin['profiles']:
            if p['name'] in data['hidden_builtin']: continue
            config={'profile':p['name'],'profile_file':'','resource_paths':'','serial_port':'','baudrate':'','bus':''}
            items.append({'id':'builtin:'+p['name'],'profile':p['name'],'source':'builtin','profile_file':builtin['profile_file'],'resource_paths':'','readiness':self._builtin_readiness(config)})
        for key, value in sorted(data['profiles'].items()):
            if not isinstance(value, dict): continue
            config={'profile':text(value.get('profile')),'profile_file':text(value.get('profile_file')),'resource_paths':text(value.get('resource_paths')),'serial_port':'','baudrate':'','bus':''}
            items.append({'id':'user:'+key,'profile':config['profile'],'source':'external','profile_file':config['profile_file'],'resource_paths':config['resource_paths'],'readiness':self._readiness(config)})
        return {'path':str(self.library_file.resolve()),'profiles':items}

    def register(self, profile_file, profile, resource_paths='', policy='error'):
        profile_file = str(Path(profile_file).expanduser().resolve()); profile=text(profile); resource_paths=text(resource_paths)
        self.inspector.inspect({'profile':profile,'profile_file':profile_file,'resource_paths':resource_paths})
        data=self._library_data(); key=profile
        if key in data['profiles']:
            if policy == 'replace': pass
            elif policy == 'suffix':
                i=2
                while f'{profile}_{i}' in data['profiles']: i+=1
                key=f'{profile}_{i}'
            else: raise ValueError('Profile Library already contains this profile id')
        data['profiles'][key]={'profile_file':profile_file,'profile':profile,'resource_paths':resource_paths}
        self._write_library(data)
        return {'id':'user:'+key,'profile':profile,'profile_file':profile_file,'resource_paths':resource_paths}

    def remove(self, entry_id):
        data=self._library_data(); entry_id=text(entry_id)
        if entry_id.startswith('builtin:'):
            name=entry_id.split(':',1)[1]
            data['hidden_builtin'].append(name)
        elif entry_id.startswith('user:'):
            data['profiles'].pop(entry_id.split(':',1)[1], None)
        else: raise ValueError('invalid Profile Library entry')
        self._write_library(data); return {'removed':entry_id}

    def discover_profile_source(self, source):
        path=Path(source).expanduser().resolve()
        if path.is_file(): candidates=[path]
        elif path.is_dir():
            direct=path/'robot_profiles.yaml'
            if direct.is_file():
                candidates=[direct]
            else:
                candidates=[]
                ignored={'.git','build','install','log','logs','node_modules','__pycache__','.install','.cache'}
                for root, dirs, files in os.walk(path):
                    dirs[:] = [d for d in dirs if d not in ignored and not d.startswith('.')]
                    if 'robot_profiles.yaml' in files:
                        candidates.append(Path(root)/'robot_profiles.yaml')
                        if len(candidates) >= 20: break
        else: raise ValueError('Profile source does not exist')
        if not candidates: raise ValueError('robot_profiles.yaml was not found in the selected Profile package')
        result=[]
        for candidate in candidates:
            profiles=read_yaml(candidate).get('profiles') or {}
            for name in profiles:
                result.append({'profile_file':str(candidate),'profile':name,'resource_paths':str(path if path.is_dir() else candidate.parent)})
        return {'candidates':result}

    def _description_source(self, source):
        path=Path(source).expanduser().resolve()
        if path.is_file() and path.suffix.lower()=='.zip':
            stamp=hashlib.sha256((str(path)+str(path.stat().st_mtime_ns)+str(path.stat().st_size)).encode()).hexdigest()[:20]
            root=self.cache/stamp
            if not root.is_dir(): _safe_extract(path, root)
            return root
        if path.is_dir(): return path
        if path.is_file() and (path.name.endswith('.urdf') or path.name.endswith('.xacro')): return path.parent
        raise ValueError('select a Description directory, ZIP, URDF, or Xacro')

    def inspect_description(self, source):
        root=self._description_source(source)
        packages=[]
        for package_xml in sorted(root.glob('**/package.xml'))[:30]:
            try:
                node=ET.parse(package_xml).getroot(); name=(node.findtext('name') or '').strip()
            except ET.ParseError: continue
            urdfs=[p for p in package_xml.parent.glob('**/*.urdf') if not p.name.endswith('.urdf.xacro')]
            if urdfs: packages.append((package_xml.parent,name,urdfs))
        if not packages:
            urdfs=list(root.glob('**/*.urdf'))
            if not urdfs: raise ValueError('Description contains no URDF')
            package_root=urdfs[0].parent; package_name=package_root.name; packages=[(package_root,package_name,urdfs)]
        package_root, package_name, urdfs=packages[0]
        urdf=urdfs[0]
        raw=self.inspector._parse_urdf(urdf,[str(root),str(package_root.parent),str(package_root)])
        children={j['child'] for j in raw['joints']}; roots=[l['name'] for l in raw['links'] if l['name'] not in children]
        parent_joint={j['child']:j for j in raw['joints']}; by_parent={}
        for j in raw['joints']: by_parent.setdefault(j['parent'],[]).append(j)
        matrices={name:[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] for name in roots}
        queue=list(roots)
        while queue:
            parent=queue.pop(0); base=matrices[parent]
            for j in by_parent.get(parent,[]):
                matrices[j['child']]=_matmul(base,_origin_matrix(j['origin'])); queue.append(j['child'])
        frames=[]
        for link in raw['links']:
            m=matrices.get(link['name'],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            frames.append({'name':link['name'],'position':[m[0][3],m[1][3],m[2][3]],'quaternion':_quat_from_matrix(m)})
        leaves=[l['name'] for l in raw['links'] if l['name'] not in by_parent]
        tool=next((name for name in leaves if name.lower() in ('tool0','tool','tcp')), leaves[0] if leaves else (raw['links'][-1]['name'] if raw['links'] else ''))
        xacros=sorted(package_root.glob('**/*.xacro'))
        preview={'profile':'description-preview','core_config':'','urdf':str(urdf),'description_urdf':str(urdf),'description_mismatch':False,'native':{'config':{'urdf_path':str(urdf),'base_frame':roots[0] if roots else '','tool_frame':tool,'gravity':[0,0,-9.81],'gravity_scale':[],'joint_names':[]},'info':{},'state':{'positions':[],'frames':frames,'center_of_mass':None}},'model':raw,'mesh_files':raw['mesh_files'],'offline':True}
        return {'source_root':str(root),'package_root':str(package_root),'package_name':package_name,'urdf':str(urdf),'urdf_relative':str(urdf.relative_to(package_root)),'xacro':str(xacros[0]) if xacros else '','ros2_control_xacro':next((str(p) for p in xacros if 'ros2_control' in p.name),''),'roots':roots,'leaves':leaves,'links':[x['name'] for x in raw['links']],'joints':raw['joints'],'preview':preview,'fingerprint':_sha(urdf)}

    @staticmethod
    def _joint_map(names, value): return {name:value for name in names}

    def create_profile(self, options):
        source=text(options.get('source')); analysis=self.inspect_description(source)
        profile=text(options.get('profile'))
        if not profile or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in profile): raise ValueError('Profile id may contain letters, numbers, underscore, and hyphen only')
        destination=Path(text(options.get('destination'))).expanduser().resolve()
        package_dir=destination/profile
        if package_dir.exists() and any(package_dir.iterdir()): raise ValueError('destination Profile package already exists and is not empty')
        package_dir.mkdir(parents=True,exist_ok=True)
        copied=package_dir/analysis['package_name']; shutil.copytree(analysis['package_root'],copied,dirs_exist_ok=True)
        config_dir=copied/'config'/'serialarm'; config_dir.mkdir(parents=True,exist_ok=True)
        joints=[text(x) for x in options.get('joint_names',[]) if text(x)]
        if not joints: raise ValueError('select at least one controlled joint')
        base=text(options.get('base_frame')); tool=text(options.get('tool_frame'))
        if base not in analysis['links'] or tool not in analysis['links']: raise ValueError('base/tool frame must exist in the Description')
        zeros=self._joint_map(joints,0.0); ones=self._joint_map(joints,1.0)
        kp_rigid=self._joint_map(joints,20.0); kd_rigid=self._joint_map(joints,0.5); kp_soft=self._joint_map(joints,3.0); kd_soft=self._joint_map(joints,0.1)
        core={
          'model':{'urdf_path':'../../'+analysis['urdf_relative'].replace('\\','/'),'joint_names':joints,'base_frame':base,'tool_frame':tool,'gravity':[0.0,0.0,-9.81],'gravity_scale':ones},
          'calibration':{'joints':{j:{'direction':1.0,'pos_ratio':1.0,'tor_ratio':1.0,'joint_zero_offset':0.0,'actuator_zero_offset':0.0} for j in joints}},
          'control':{'runtime':{'ctrl_frequency_hz':200.0,'joint_acc_filter_alpha':0.2,'write_enabled':False,'model_feedforward_mode':'GRAVITY','tracking_impedance_mode':'COMPLIANT_TRACKING'},'controller':{'allow_full_cmd':False,'rigid_hold':{'kp':kp_rigid,'kd':kd_rigid},'rigid_tracking':{'kp':kp_rigid,'kd':kd_rigid},'compliant_hold':{'kp':kp_soft,'kd':kd_soft},'compliant_drag':{'kp':zeros,'kd':kd_soft},'compliant_tracking':{'kp':kp_soft,'kd':kd_soft}}},
          'safety_policy':{'position_margin':0.0,'cmd_vel_scale':0.25,'state_vel_scale':1.0,'max_acc':self._joint_map(joints,2.0),'max_kp_override':self._joint_map(joints,50.0),'max_kd_override':self._joint_map(joints,2.0),'max_dt_s':0.02,'state_timeout_s':0.05,'cmd_timeout_s':0.10,'require_all_actuators_online':True,'require_all_actuators_enabled':True,'reject_motor_error':True,'require_continuous_cmd':False,'fault_recovery':{'default_mode':'rigid_hold','allow_compliant_recovery':True,'require_operator_request':True,'gravity_model_validated':False,'recovery_timeout_s':30.0,'compliant_recovery':{'kp':kp_soft,'kd':kd_soft,'max_vel':self._joint_map(joints,0.5),'effort_scale':0.5}}},
          'shutdown':{'park_before_disable':True,'park_pos':zeros,'speed_scale':0.05,'position_tolerance':0.05,'velocity_tolerance':0.05,'settle_time_s':0.25,'relaxed_tolerance_ratio':2.0,'timeout_s':10.0},
          'capability':{'admittance':{'enabled':False,'joint_enabled':{j:True for j in joints},'observer':{'mode':'MOMENTUM','momentum_gain':self._joint_map(joints,25.0)},'calibration':{'torque_bias':zeros,'torque_threshold':self._joint_map(joints,0.05),'friction':{'enabled':False,'velocity_transition':0.03,'positive_coulomb':zeros,'positive_viscous':zeros,'negative_coulomb':zeros,'negative_viscous':zeros}},'controller':{'mass':ones,'damping':self._joint_map(joints,5.5),'stiffness':self._joint_map(joints,15.0),'max_delta_q':self._joint_map(joints,0.5),'max_delta_q_dot':self._joint_map(joints,1.0)}}}
        }
        _atomic_yaml(config_dir/'core.yaml',core)
        hardware_plugin=text(options.get('hardware_plugin'))
        hardware={}
        actuators=options.get('actuators') or []
        complete=hardware_plugin=='serial_arm_hardware_damiao' and len(actuators)==len(joints) and all(text(a.get('motor_type')) and int(a.get('motor_id',0))>0 for a in actuators)
        if complete:
            bus=text(options.get('bus')) or 'main_can'; device=text(options.get('device')) or '/dev/ttyACM0'; baud=int(options.get('baudrate') or 921600)
            hardware={'buses':{bus:{'type':'can','backend':'damiao_usb2can','device':device,'baudrate':baud}},'damiao':{'bus':bus,'refresh_state_in_read':False,'feedback_timeout_s':0.05,'activation_retries':3,'startup_read_cycles':5,'stop_kp':10.0,'stop_kd':0.15,'stop_cycles':5,'actuators':{j:{'name':text(a.get('name')) or f'actuator{i+1}','motor_id':int(a.get('motor_id')),'master_id':int(a.get('master_id') or 0),'motor_type':text(a.get('motor_type'))} for i,(j,a) in enumerate(zip(joints,actuators))}}}
        _atomic_yaml(config_dir/'hardware.yaml',hardware)
        entry={'core':{'package':analysis['package_name'],'config':'config/serialarm/core.yaml'},'hardware':{'plugin':hardware_plugin if complete else '','config_package':analysis['package_name'],'config':'config/serialarm/hardware.yaml'},'description':{'package':analysis['package_name'],'urdf':analysis['urdf_relative'].replace('\\','/')}}
        if analysis['ros2_control_xacro']:
            entry['description']['ros2_control_xacro']=str(Path(analysis['ros2_control_xacro']).relative_to(analysis['package_root'])).replace('\\','/')
        profile_file=package_dir/'robot_profiles.yaml'; _atomic_yaml(profile_file,{'profiles':{profile:entry}})
        readiness=self._readiness_path(profile_file, profile); _atomic_yaml(readiness,{'profile':profile,'managed_by_launcher':True,'source_urdf_fingerprint':analysis['fingerprint'],'stages':{}})
        return {'package':str(package_dir),'profile_file':str(profile_file),'profile':profile,'resource_paths':str(package_dir),'hardware_complete':complete,'config':{'profile':profile,'profile_file':str(profile_file),'resource_paths':str(package_dir),'serial_port':'','baudrate':'','bus':''}}

    def mark_stage(self, config, stage, confirmed=True):
        allowed={'model_check','bringup','mapping_calibration','geometry_calibration','dynamics_calibration','validation'}
        if stage not in allowed: raise ValueError('unknown readiness stage')
        info=self.inspector.inspect(config); path=self._readiness_path(info['profile_file'], info['profile']); data=yaml.safe_load(path.read_text()) if path.is_file() else {}
        if not isinstance(data,dict):
            data={}
        stages=data.setdefault('stages',{})
        stages[stage]={'confirmed':bool(confirmed),'fingerprint':info['fingerprint'] if confirmed else ''}
        _atomic_yaml(path,data); return self._readiness(config)

    def profile_editor_load(self, config):
        info=self.inspector.inspect(config); core=read_yaml(info['resources']['core']); hardware=read_yaml(info['resources']['hardware']) if Path(info['resources'].get('hardware','')).is_file() else {}
        joints=(core.get('model') or {}).get('joint_names') or []; calibration=(core.get('calibration') or {}).get('joints') or {}
        damiao=hardware.get('damiao') or {}; actuators=damiao.get('actuators') or {}; bus=damiao.get('bus') or info.get('bus') or ''; bus_cfg=(hardware.get('buses') or {}).get(bus,{})
        return {'profile_file':info['profile_file'],'profile':info['profile'],'core_path':info['resources']['core'],'hardware_path':info['resources'].get('hardware',''),'core_sha':_sha(info['resources']['core']),'hardware_sha':_sha(info['resources']['hardware']) if Path(info['resources'].get('hardware','')).is_file() else '','joint_names':joints,'calibration':calibration,'write_enabled':bool(((core.get('control') or {}).get('runtime') or {}).get('write_enabled')),'park_pos':(core.get('shutdown') or {}).get('park_pos') or {},'hardware_plugin':info.get('hardware_plugin',''),'bus':bus,'device':bus_cfg.get('device',''),'baudrate':bus_cfg.get('baudrate',''),'actuators':{j:actuators.get(j,{}) for j in joints}}

    def profile_editor_save(self, config, payload):
        info=self.inspector.inspect(config); core_path=Path(info['resources']['core']); hw_path=Path(info['resources']['hardware'])
        if _sha(core_path)!=payload.get('core_sha') or (hw_path.is_file() and _sha(hw_path)!=payload.get('hardware_sha')): raise ValueError('Profile files changed outside the workspace; reload before saving')
        core=read_yaml(core_path); hardware=read_yaml(hw_path) if hw_path.is_file() else {}; joints=(core.get('model') or {}).get('joint_names') or []
        cal=(core.setdefault('calibration',{}).setdefault('joints',{})); incoming=payload.get('calibration') or {}
        for j in joints:
            item=incoming.get(j) or {}; target=cal.setdefault(j,{})
            for key in ('direction','pos_ratio','tor_ratio','joint_zero_offset','actuator_zero_offset'):
                value=float(item.get(key,target.get(key,1.0 if key in ('direction','pos_ratio','tor_ratio') else 0.0)))
                if not math.isfinite(value): raise ValueError('calibration values must be finite')
                target[key]=value
        core.setdefault('control',{}).setdefault('runtime',{})['write_enabled']=bool(payload.get('write_enabled'))
        park=payload.get('park_pos') or {}; core.setdefault('shutdown',{})['park_pos']={j:float(park.get(j,0.0)) for j in joints}
        _atomic_yaml(core_path,core)
        requested_plugin=text(payload.get('hardware_plugin')) or info.get('hardware_plugin','')
        if requested_plugin and requested_plugin != info.get('hardware_plugin',''):
            profile_file=Path(info['profile_file']); profile_data=read_yaml(profile_file); entry=(profile_data.get('profiles') or {}).get(info['profile'])
            if not isinstance(entry,dict): raise ValueError('Profile entry is missing while updating hardware plugin')
            entry.setdefault('hardware',{})['plugin']=requested_plugin
            _atomic_yaml(profile_file,profile_data)
        if requested_plugin=='serial_arm_hardware_damiao':
            damiao=hardware.setdefault('damiao',{}); bus=text(payload.get('bus')) or damiao.get('bus') or 'main_can'; damiao['bus']=bus
            for key,value in {'refresh_state_in_read':False,'feedback_timeout_s':0.05,'activation_retries':3,'startup_read_cycles':5,'stop_kp':10.0,'stop_kd':0.15,'stop_cycles':5}.items(): damiao.setdefault(key,value)
            buses=hardware.setdefault('buses',{}); b=buses.setdefault(bus,{'type':'can','backend':'damiao_usb2can'}); b['device']=text(payload.get('device')) or b.get('device','/dev/ttyACM0'); b['baudrate']=int(payload.get('baudrate') or b.get('baudrate',921600))
            target_act=damiao.setdefault('actuators',{}); incoming_act=payload.get('actuators') or {}
            for i,j in enumerate(joints):
                a=incoming_act.get(j) or {}; t=target_act.setdefault(j,{})
                t['name']=text(a.get('name')) or t.get('name') or f'actuator{i+1}'; t['motor_id']=int(a.get('motor_id') or t.get('motor_id') or i+1); t['master_id']=int(a.get('master_id') or t.get('master_id') or 0); t['motor_type']=text(a.get('motor_type')) or t.get('motor_type') or 'DM4310'
            _atomic_yaml(hw_path,hardware)
        return self.profile_editor_load(config)
