"""Read-only fixed-point TCP calibration against fresh native robot feedback

No motion commands are issued here; geometry is estimated from the live Core FK
and accepted by a separate validation set before optional Profile application
"""
from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


MIN_TRAIN = 8
MIN_VALIDATE = 5
MAX_AGE_MS = 250
MAX_JOINT_SPEED = .035
MAX_CONDITION = 250.0


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _finite_vector(value, count, name):
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} 不是有效数值数组') from exc
    if vector.shape != (count,) or not np.all(np.isfinite(vector)):
        raise ValueError(f'{name} 必须包含 {count} 个有限数值')
    return vector


def _quaternion_rotation(value):
    x, y, z, w = _finite_vector(value, 4, '法兰四元数')
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm - 1) > .05 or norm < 1e-8:
        raise ValueError('法兰四元数无效')
    x, y, z, w = np.array([x,y,z,w]) / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)


def _sample_transform(frame):
    return _quaternion_rotation(frame['quaternion']), _finite_vector(frame['position'], 3, '法兰位置')


def solve_fixed_point(samples):
    if len(samples) < MIN_TRAIN:
        raise ValueError(f'至少需要 {MIN_TRAIN} 组标定姿态')
    rows, targets = [], []
    for sample in samples:
        r, t = _sample_transform(sample['frame'])
        rows.append(np.hstack((r, -np.eye(3))))
        targets.append(-t)
    matrix, rhs = np.vstack(rows), np.concatenate(targets)
    _, singular, _ = np.linalg.svd(matrix, full_matrices=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-7))
    condition = float(singular[0]/singular[-1]) if singular[-1] > 0 else float('inf')
    if rank < 6 or condition > MAX_CONDITION:
        raise ValueError('姿态方向变化不足，TCP 无法稳定辨识，请从多个方向重新对准同一个固定点')
    solution = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
    tcp, fixed = solution[:3], solution[3:]
    errors = np.array([float(np.linalg.norm(_sample_transform(s['frame'])[0] @ tcp + _sample_transform(s['frame'])[1] - fixed) * 1000) for s in samples])
    return {'tcp_xyz_m': tcp.tolist(), 'fixed_point_m': fixed.tolist(),
            'rank': rank, 'condition': condition,
            'training_rms_mm': float(np.sqrt(np.mean(errors*errors))),
            'training_max_mm': float(max(errors)),
            'training_errors_mm': errors.tolist()}


def validate_fixed_point(candidate, samples, rms_limit_mm, max_limit_mm):
    if len(samples) < MIN_VALIDATE:
        raise ValueError(f'独立验证至少需要 {MIN_VALIDATE} 组新姿态')
    tcp = _finite_vector(candidate['tcp_xyz_m'], 3, 'TCP')
    fixed = _finite_vector(candidate['fixed_point_m'], 3, '固定点')
    errors_xyz = []
    for sample in samples:
        r, t = _sample_transform(sample['frame'])
        errors_xyz.append((r @ tcp + t - fixed) * 1000)
    components = np.asarray(errors_xyz)
    errors = np.linalg.norm(components, axis=1)
    rms, max_error = float(np.sqrt(np.mean(errors**2))), float(max(errors))
    return {'passed': bool(rms <= rms_limit_mm and max_error <= max_limit_mm),
            'rms_mm': rms, 'max_mm': max_error, 'p95_mm': float(np.percentile(errors, 95)),
            'limits_mm': {'rms': float(rms_limit_mm), 'max': float(max_limit_mm)},
            'errors_mm': errors.tolist(), 'errors_xyz_mm': components.tolist(),
            'count': len(samples), 'meaning': '基于机器人正运动学的固定点一致性，非绝对位置真值'}


class TcpCalibration:
    def __init__(self, library):
        self.library = library
        self.session = None

    def _profile(self, config):
        info = self.library.inspector.inspect(config)
        core_path = Path(info['resources']['core']).resolve()
        core = yaml.safe_load(core_path.read_text(encoding='utf-8'))
        urdf = (core_path.parent / core['model']['urdf_path']).resolve()
        return info, core, core_path, urdf

    @staticmethod
    def _kinematics_scope(core):
        # TCP FK depends on the selected chain and joint mapping, not on
        # unrelated runtime gains, gravity compensation or Safety settings
        model = core.get('model') or {}
        return {key: model.get(key) for key in ('joint_names', 'base_frame', 'tool_frame')}

    def _ensure(self, config):
        info, core, core_path, urdf = self._profile(config)
        session = self.session
        if not session or session['profile'] != info['profile'] or session['profile_file'] != info['profile_file']:
            raise ValueError('请先为当前机器人开始 TCP 标定')
        scope = session.get('kinematics_hash')
        # Earlier saved sessions did not have a scoped hash, so retain their
        # original full-file check instead of silently trusting old samples
        model_changed = (scope != self.library._digest(self._kinematics_scope(core))
                         if scope is not None else session['core_hash'] != _digest(core_path))
        if model_changed or session['urdf_hash'] != _digest(urdf):
            self.session = None
            raise ValueError('控制链或 URDF 已变化，旧 TCP 采样不再适用，请重新开始标定')
        if session['calibration_hash'] != self.library._digest(core.get('calibration')):
            self.session = None
            raise ValueError('关节映射或零位已变化，旧 TCP 采样不再适用')
        return session, info, core, core_path, urdf

    def _directory(self, info):
        readiness = self.library._readiness_path(info['profile_file'], info['profile'])
        directory = readiness.parent.parent / 'tcp-calibration' / info['profile']
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def begin(self, config, frame):
        info, core, core_path, urdf = self._profile(config)
        # Preparing a TCP session is a read-only geometry operation
        # Optional readiness notes and gain-approval records do not gate it
        # Live feedback is checked separately when capture() is called
        model = core['model']
        if not isinstance(frame, str) or not frame or len(frame) > 128:
            raise ValueError('请选择末端法兰或工具安装 Frame')
        root = ET.parse(urdf).getroot()
        links = {x.get('name') for x in root.findall('link')}
        if frame not in links or frame not in self._frame_options(core, urdf):
            raise ValueError('所选 Frame 不在末端固定工具链中')
        if frame == model['base_frame']:
            raise ValueError('TCP 标定 Frame 不能为机器人基座')
        # Native runtime must know the frame; native feedback determines its pose
        self.session = {'profile': info['profile'], 'profile_file': info['profile_file'],
                        'core_hash': _digest(core_path), 'urdf_hash': _digest(urdf),
                        'kinematics_hash': self.library._digest(self._kinematics_scope(core)),
                        'calibration_hash': self.library._digest(core.get('calibration')),
                        'parent_frame': frame, 'base_frame': model['base_frame'],
                        'tool_frame': model['tool_frame'], 'train': [], 'validation': [],
                        'candidate': None, 'report': None, 'last_sequence': -1,
                        'created_at': time.time(), 'directory': str(self._directory(info))}
        self._save()
        return self.state(config)

    def _save(self):
        if self.session is None: return
        session = {k: copy.deepcopy(v) for k, v in self.session.items() if k != 'last_sequence'}
        directory = Path(self.session['directory'])
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / 'tcp-session.yaml'
        temp = path.with_suffix('.yaml.tmp')
        temp.write_text(yaml.safe_dump(session, allow_unicode=True, sort_keys=False), encoding='utf-8')
        temp.replace(path)

    @staticmethod
    def _fixed_component(urdf, origin_frame):
        """Resolve frames rigidly attached to the active tool, in both directions

        A tip commonly sits below tool0, while a flange may sit above it
        Never traverse a revolute, prismatic, or other movable joint
        """
        root = ET.parse(urdf).getroot()
        names = {link.get('name') for link in root.findall('link')}
        if origin_frame not in names:
            raise ValueError(f'工具 Frame {origin_frame} 在 URDF 中不存在')
        edges = {name: [] for name in names}
        for joint in root.findall('joint'):
            if joint.get('type') != 'fixed':
                continue
            parent, child = joint.find('parent'), joint.find('child')
            if parent is None or child is None:
                continue
            a, b = parent.get('link'), child.get('link')
            if a not in edges or b not in edges:
                continue
            origin = joint.find('origin')
            xyz = _finite_vector((origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split(), 3, '固定关节平移')
            roll, pitch, yaw = _finite_vector((origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split(), 3, '固定关节旋转')
            cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
            rot = np.array([
                [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                [-sp, cp*sr, cp*cr]
            ])
            edges[a].append((b,rot,xyz))
            edges[b].append((a,rot.T,-rot.T@xyz))
        transforms={origin_frame:(np.eye(3),np.zeros(3))}
        queue=[origin_frame]
        while queue:
            source=queue.pop(0)
            r0,t0=transforms[source]
            for name,r,t in edges[source]:
                if name in transforms:
                    continue
                transforms[name]=(r0@r,r0@t+t0)
                queue.append(name)
        return transforms

    def _frame_options(self, core, urdf):
        tool=core['model']['tool_frame']
        base=core['model']['base_frame']
        frames=self._fixed_component(urdf,tool)
        return [tool]+sorted((name for name in frames if name not in (tool,base)),key=str.casefold)

    @staticmethod
    def _frame_from_runtime(status, selected_frame, fixed_frames, tool_frame):
        """Use the native FK pose, or reconstruct a fixed tool Link from native FK

        Some runtime reductions expose tool0 but not every fixed descendant frame
        A rigid URDF transform is sufficient; no moving-joint state is inferred
        """
        available = {f.get('name'):f for f in status.get('frames', []) if isinstance(f,dict)}
        if selected_frame in available:
            return available[selected_frame]
        if selected_frame not in fixed_frames:
            raise ValueError('所选 Link 不属于当前工具的固定关节链')
        # Always anchor the fallback at the configured tool, never an arbitrary Link
        anchor=available.get(tool_frame)
        if anchor is None:
            raise ValueError('运行时尚未提供所选 Frame 或工具 Frame 的正运动学位姿')
        r_world, t_world = _sample_transform(anchor)
        r_tool, t_tool = fixed_frames[selected_frame]
        rotation = r_world@r_tool
        # Convert a proper SO(3) rotation to a normalized xyzw quaternion
        trace=float(np.trace(rotation))
        if trace>0:
            factor=math.sqrt(1+trace)*2
            qw=.25*factor
            qx=(rotation[2,1]-rotation[1,2])/factor
            qy=(rotation[0,2]-rotation[2,0])/factor
            qz=(rotation[1,0]-rotation[0,1])/factor
        else:
            k=int(np.argmax(np.diag(rotation)))
            if k==0:
                factor=math.sqrt(max(0,1+rotation[0,0]-rotation[1,1]-rotation[2,2]))*2
                qx=.25*factor; qy=(rotation[0,1]+rotation[1,0])/factor; qz=(rotation[0,2]+rotation[2,0])/factor; qw=(rotation[2,1]-rotation[1,2])/factor
            elif k==1:
                factor=math.sqrt(max(0,1+rotation[1,1]-rotation[0,0]-rotation[2,2]))*2
                qx=(rotation[0,1]+rotation[1,0])/factor; qy=.25*factor; qz=(rotation[1,2]+rotation[2,1])/factor; qw=(rotation[0,2]-rotation[2,0])/factor
            else:
                factor=math.sqrt(max(0,1+rotation[2,2]-rotation[0,0]-rotation[1,1]))*2
                qx=(rotation[0,2]+rotation[2,0])/factor; qy=(rotation[1,2]+rotation[2,1])/factor; qz=.25*factor; qw=(rotation[1,0]-rotation[0,1])/factor
        quat = np.array([qx,qy,qz,qw],dtype=float)
        norm = np.linalg.norm(quat)
        if norm<1e-8 or not math.isfinite(norm):
            raise ValueError('URDF 固定关节坐标变换无效')
        return {'name':selected_frame,'position':(r_world@t_tool+t_world).tolist(),'quaternion':(quat/norm).tolist()}

    def state(self, config):
        info, core, core_path, urdf = self._profile(config)
        options = self._frame_options(core, urdf)
        default = {'active':False, 'base_frame':core['model']['base_frame'],
                   'tool_frame':core['model']['tool_frame'], 'frame_options':options,
                   'suggested_frame':next((f for f in ('tip', 'tcp', 'tool_tip') if f in options), core['model']['tool_frame'])}
        if self.session is not None and (self.session.get('profile') != info['profile'] or self.session.get('profile_file') != info['profile_file']):
            self.session = None
        if self.session is None:
            path = self._directory(info) / 'tcp-session.yaml'
            if path.is_file():
                try:
                    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
                    if isinstance(doc, dict) and doc.get('profile') == info['profile'] and doc.get('profile_file') == info['profile_file']:
                        self.session = doc
                        self.session['last_sequence'] = -1
                except (OSError, yaml.YAMLError):
                    pass
        try: session, info, core, core_path, urdf = self._ensure(config)
        except ValueError: return default
        return {**default, 'active':True, 'parent_frame':session['parent_frame'],
                'train':len(session['train']), 'validation':len(session['validation']),
                'candidate':session['candidate'], 'report':session['report'],
                'directory':session['directory'],
                'samples': {'train':session['train'], 'validation':session['validation']}}

    def capture(self, config, phase, status, previous=None):
        session, *_ = self._ensure(config)
        if phase not in ('train', 'validation'):
            raise ValueError('未知 TCP 采集阶段')
        if phase == 'validation' and session['candidate'] is None:
            raise ValueError('请先求解候选 TCP，再采集独立验证姿态')
        cal_phase = (status.get('model_calibration') or {}).get('phase')
        if cal_phase in ('recording','replaying','fitting','collecting','validating'):
            raise ValueError('其他动力学标定任务正在运行，不允许同时采集 TCP')
        if (status.get('robot_state') != 'ACTIVE' or status.get('valid') is not True or
                status.get('write_enabled') is not True or status.get('fault')):
            raise ValueError('TCP 采集需要有效的真机 ACTIVE 状态和可用硬件反馈')
        age = status.get('feedback_age_ms')
        if not isinstance(age, (float, int)) or not 0 <= age <= MAX_AGE_MS:
            raise ValueError('反馈过期，拒绝采样')
        seq = status.get('sequence')
        if not isinstance(seq, int) or seq < 1 or seq == session['last_sequence']:
            raise ValueError('没有新的实时控制周期反馈')
        velocities = _finite_vector((status.get('joint') or {}).get('vel'), len(status.get('joint_names', [])), '关节速度')
        if len(velocities) < 1 or max(abs(velocities)) > MAX_JOINT_SPEED:
            raise ValueError('机械臂尚未停稳，请等待关节速度降低后再采样')
        joints = _finite_vector((status.get('joint') or {}).get('pos'), len(velocities), '关节位置')
        if previous is not None:
            prior_pos = _finite_vector((previous.get('joint') or {}).get('pos'), len(joints), '稳定性检查关节位置')
            prior_vel = _finite_vector((previous.get('joint') or {}).get('vel'), len(joints), '稳定性检查关节速度')
            if (previous.get('robot_state') != 'ACTIVE' or previous.get('valid') is not True or
                    previous.get('fault') or previous.get('sequence') == seq or
                    max(abs(prior_vel)) > MAX_JOINT_SPEED or max(abs(joints - prior_pos)) > .0015):
                raise ValueError('机器人在采样窗口内未持续停稳，请保持接触姿态后重试')
        _, core, _, urdf = self._profile(config)
        fixed_frames = self._fixed_component(urdf, core['model']['tool_frame'])
        frame = self._frame_from_runtime(status, session['parent_frame'], fixed_frames, core['model']['tool_frame'])
        r, t = _sample_transform(frame)
        previous = (session['train'] if phase == 'train' else (session['train'] + session['validation']))
        if previous:
            earlier = [_sample_transform(s['frame'])[0] for s in previous]
            angles = [math.degrees(math.acos(float(np.clip((np.trace(prev.T @ r)-1)/2,-1,1)))) for prev in earlier]
            if min(angles) < (8 if phase == 'train' else 5):
                raise ValueError('与已有采样姿态过于接近，请改变末端朝向后重新接触')
        if phase == 'validation' and session['report'] is not None:
            session['report'] = None
        sample = {'frame': {'name': session['parent_frame'], 'position': t.tolist(), 'quaternion': frame['quaternion']},
                  'joints': joints.tolist(), 'sequence': seq, 'recorded_at': time.time()}
        session[phase].append(sample)
        session['last_sequence'] = seq
        # Adding new training data invalidates the frozen candidate and all validation evidence
        if phase == 'train':
            session['candidate'] = None
            session['validation'] = []
            session['report'] = None
        self._save()
        return self.state(config)

    def solve(self, config):
        session, *_ = self._ensure(config)
        session['candidate'] = solve_fixed_point(session['train'])
        session['validation'] = []
        session['report'] = None
        self._save()
        return self.state(config)

    def validate(self, config, rms_limit_mm=1.0, max_limit_mm=2.0):
        session, *_ = self._ensure(config)
        if session['candidate'] is None:
            raise ValueError('尚未求解 TCP 候选')
        for value in (rms_limit_mm, max_limit_mm):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or not .05 <= value <= 100:
                raise ValueError('验收阈值必须在 0.05～100 mm 之间')
        session['report'] = validate_fixed_point(session['candidate'], session['validation'], rms_limit_mm, max_limit_mm)
        self._save()
        return self.state(config)

    def discard(self, config):
        session, *_ = self._ensure(config)
        (Path(session['directory']) / 'tcp-session.yaml').unlink(missing_ok=True)
        self.session = None
        return {'active': False}

    def export(self, config):
        session, *_ = self._ensure(config)
        if not session['report'] or not session['report']['passed']:
            raise ValueError('必须通过独立针尖验证后才能导出正式 TCP 候选')
        directory = Path(session['directory'])
        output = directory / 'tcp-candidate.yaml'
        export = {'profile': session['profile'], 'base_frame': session['base_frame'],
                  'parent_frame': session['parent_frame'], 'tcp_xyz_m': session['candidate']['tcp_xyz_m'],
                  'tcp_rpy_rad': [0., 0., 0.], 'rotation_note': '固定点方法仅辨识平移，工具朝向沿用父 Frame',
                  'fit': session['candidate'], 'validation': session['report'],
                  'source_hashes': {'core': session['core_hash'], 'urdf': session['urdf_hash']},
                  'training_samples': session['train'], 'validation_samples': session['validation']}
        tmp = output.with_suffix('.yaml.tmp')
        tmp.write_text(yaml.safe_dump(export, allow_unicode=True, sort_keys=False), encoding='utf-8')
        tmp.replace(output)
        return {'path': str(output), 'directory': str(directory)}

    def apply(self, config, *, native_running):
        session, info, core, core_path, urdf = self._ensure(config)
        if native_running:
            raise ValueError('保存 TCP 模型前必须停放并断开真机会话')
        if not session['report'] or session['report']['passed'] is not True:
            raise ValueError('候选尚未通过独立验证，禁止应用')
        self.export(config)
        profile = Path(info['profile_file']).resolve()
        if profile.is_relative_to((self.library.root / 'src' / 'robot_supports').resolve()):
            raise ValueError('内置 Profile 不可直接改写，请导出结果并在独立 Profile 包中应用')
        if not profile.is_file(): raise ValueError('Profile 配置文件不存在')
        profile_doc = yaml.safe_load(profile.read_text(encoding='utf-8'))
        entry = (profile_doc.get('profiles') or {}).get(info['profile'])
        if not isinstance(entry, dict): raise ValueError('Profile 配置结构无效')
        description = entry.get('description') or {}
        desc_uri = description.get('urdf')
        if not isinstance(desc_uri, str) or not desc_uri.lower().endswith('.urdf'):
            raise ValueError('Profile Description 不是可直接修改的 URDF，先导出 TCP 候选')
        # This requires the model and display description to be one identical source
        desc_package = description.get('package')
        resolved_desc = self.library.inspector.share(desc_package, config.get('resource_paths','').split(':') if config.get('resource_paths') else []) / desc_uri
        if resolved_desc.resolve() != urdf:
            raise ValueError('Core URDF 与 Profile Description URDF 不同，禁止自动应用以免上层模型不一致')
        xml = ET.parse(urdf)
        root = xml.getroot()
        link_name = f"{info['profile']}_tcp"
        joint_name = f"{info['profile']}_tcp_fixed"
        existing = {x.get('name') for x in root.findall('link')} | {x.get('name') for x in root.findall('joint')}
        if link_name in existing or joint_name in existing:
            raise ValueError('目标 TCP Link 名称与原始模型冲突，请先处理旧工具配置')
        link = ET.SubElement(root, 'link', {'name':link_name})
        joint = ET.SubElement(root,'joint', {'name':joint_name,'type':'fixed'})
        ET.SubElement(joint, 'parent', {'link': session['parent_frame']})
        ET.SubElement(joint, 'child', {'link': link_name})
        xyz = ' '.join(format(float(x), '.12g') for x in session['candidate']['tcp_xyz_m'])
        ET.SubElement(joint, 'origin', {'xyz':xyz,'rpy':'0 0 0'})
        generated = urdf.with_name(urdf.stem+'-tcp.urdf')
        if generated.exists():
            raise ValueError(f'候选模型文件已存在，不自动覆盖 {generated}')
        # The new URDF shares the original directory to preserve relative mesh paths
        new_core = copy.deepcopy(core)
        new_core['model']['urdf_path'] = str(generated.relative_to(core_path.parent)) if generated.is_relative_to(core_path.parent) else str(generated)
        new_core['model']['tool_frame'] = link_name
        profile_doc['profiles'][info['profile']]['description']['urdf'] = str(Path(desc_uri).with_name(generated.name)).replace('\\','/')
        core_data = yaml.safe_dump(new_core, allow_unicode=True, sort_keys=False)
        profile_data = yaml.safe_dump(profile_doc, allow_unicode=True, sort_keys=False)
        core_backup, profile_backup = core_path.read_bytes(), profile.read_bytes()
        try:
            tmp_urdf = generated.with_suffix('.urdf.tmp')
            xml.write(tmp_urdf, encoding='utf-8', xml_declaration=True)
            ET.parse(tmp_urdf)
            tmp_urdf.replace(generated)
            c_tmp = core_path.with_suffix(core_path.suffix+'.tmp')
            p_tmp = profile.with_suffix(profile.suffix+'.tmp')
            c_tmp.write_text(core_data, encoding='utf-8')
            p_tmp.write_text(profile_data, encoding='utf-8')
            yaml.safe_load(c_tmp.read_text());yaml.safe_load(p_tmp.read_text())
            c_tmp.replace(core_path)
            p_tmp.replace(profile)
            self.library.inspector.inspect(config)
        except Exception:
            core_path.write_bytes(core_backup)
            profile.write_bytes(profile_backup)
            generated.unlink(missing_ok=True)
            raise
        (Path(session['directory']) / 'tcp-session.yaml').unlink(missing_ok=True)
        self.session = None
        return {'applied': True, 'tool_frame': link_name, 'urdf': str(generated),
                'warning':'Core 与 Profile Description 已更新，外部 ROS2 / MoveIt / LeRobot / Isaac 模型资源须自行同步，并在重新连接后完成几何验收'}
