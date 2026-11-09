"""Export deployable calibration artifacts from immutable task snapshots"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import tempfile
import time
import zipfile
import xml.etree.ElementTree as ET

import yaml

from model_calibration_source import matches_recorded_fingerprint, verified_source_urdf
from persistence import build_gravity_correction_payload, export_candidate_urdf, patch_text
from workflow_registry import load, resolve_dynamics_candidate_path
from control_parameters import (canonical_core, canonical_control_text, four_mode_control_text, complete_mapping_values,
                                deployment_gain_groups, gain_measurement_available, gain_selection_report)
from gain_context import recorded_gain_core, verify_gain_context

GAIN_FIELDS = {'rigid_tracking', 'compliant_tracking', 'compliant_drag', 'servo_tracking', 'identification_tracking', 'all'}
GAIN_DESTINATIONS = {
    'rigid_tracking': ('rigid_tracking',),
    'compliant_tracking': ('compliant_tracking',),
    'compliant_drag': ('compliant_drag',),
    'servo_tracking': ('servo_tracking',),
}


def _read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def _snapshot(record):
    folder=Path(record['directory']).resolve()
    metadata=_read_json(folder/'metadata.json')
    model=verified_source_urdf(folder,metadata)
    core=folder/'source'/'core.yaml'
    if not matches_recorded_fingerprint(core, metadata.get('core_fingerprint')):
        original=Path(metadata.get('core_config_path') or '/missing').expanduser().resolve()
        if not matches_recorded_fingerprint(original,metadata.get('core_fingerprint')):
            raise ValueError('缺少与任务指纹匹配的 Core 配置快照')
        core=original
    config=yaml.safe_load(core.read_text(encoding='utf-8'))
    if not isinstance(config,dict) or not isinstance(config.get('control'),dict):
        raise ValueError('任务 Core 配置格式不正确')
    if list(config.get('model',{}).get('joint_names') or []) != metadata.get('joint_names'):
        raise ValueError('Core 配置关节映射与任务不一致')
    return folder,metadata,model,core,config


def _replace_core(text, targets, *, hold_compensation=None):
    # Patch task-role paths in memory, then serialize the four public purposes.
    # This also upgrades old snapshots when exporting a non-servo candidate
    text = canonical_control_text(text)
    if hold_compensation is not None:
        text = complete_mapping_values(text, ('control',), {'hold_compensation': hold_compensation})
    result=patch_text(text,targets)
    return four_mode_control_text(result)


def _gain_map(joints,values,limits=None,limit_label='能力范围'):
    if len(joints)!=len(values):raise ValueError('关节增益数量与 Core 关节数量不一致')
    for i,value in enumerate(values):
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<0:
            raise ValueError(f'{joints[i]} 增益无效')
        if limits is not None and value>float(limits[joints[i]])+1e-9:
            raise ValueError(f'{joints[i]} 增益超过{limit_label}')
    return '{'+', '.join(f'{j}: {float(v):.12g}' for j,v in zip(joints,values))+'}'


def _bundle_model(snapshot, metadata, dest, name='robot.urdf'):
    from model_calibration_source import resolve_relative_mesh
    root=ET.parse(snapshot).getroot()
    copied={}
    for mesh in root.findall('.//mesh'):
        source_name=mesh.get('filename','')
        if not source_name or source_name.startswith('package://'):
            continue
        if Path(source_name).is_absolute():
            src=Path(source_name)
            if not src.is_file():raise ValueError(f'绝对路径网格资源不存在 {src}')
        else:
            src=resolve_relative_mesh(source_name,snapshot,metadata)
            if src is None:raise ValueError(f'相对路径网格资源不存在 {source_name}')
        src=src.resolve()
        if src not in copied:
            resources=dest/'resources';resources.mkdir(exist_ok=True)
            target=resources/f'{len(copied):03d}-{src.name}'
            shutil.copy2(src,target)
            copied[src]=target.relative_to(dest).as_posix()
        mesh.set('filename',copied[src])
    out=dest/name
    ET.ElementTree(root).write(out,encoding='utf-8',xml_declaration=True)
    ET.parse(out)
    return out


def gain_candidate_targets(gain, cfg, joints, gain_purposes=None, hardware=None):
    """Validate a candidate and return its exact deployment leaves"""
    category = gain.get('category')
    policy = cfg.get('safety_policy') or {}
    from gain_initialization import protocol_gain_limits
    protocol = protocol_gain_limits(cfg, hardware)
    gain_limits = {term: {joint: protocol[joint][term + '_max'] for joint in joints}
                   for term in ('kp', 'kd')} if protocol else {}
    groups = deployment_gain_groups(gain, gain_purposes)
    targets = {}
    hold_compensation = None
    for item, fields in groups.items():
        for term in ('kp', 'kd'):
            values = fields.get(term)
            if not isinstance(values, list):
                raise ValueError('增益候选参数不完整')
            gain_map = _gain_map(joints, values, gain_limits.get(term), '映射后的硬件协议范围')
            for destination in GAIN_DESTINATIONS[item]:
                targets[('control', 'controller', destination, term)] = gain_map
        if item in ('rigid_tracking', 'compliant_tracking'):
            hold = fields.get('hold_gains')
            verification = ('hold_compensation_verification'
                            if item == 'rigid_tracking' and
                            (fields.get('hold_compensation') or {}).get('enabled') is True
                            else 'hold_verification')
            measured = (gain_measurement_available(fields.get(verification), len(joints), 'hold') or
                        (fields.get(verification) or {}).get('validated') is True)
            if isinstance(hold, dict) and measured:
                for term in ('kp', 'kd'):
                    values = hold.get(term)
                    if not isinstance(values, list): raise ValueError('保持验收缺少对应增益')
                    targets[('control', 'controller', item.replace('_tracking', '_hold'), term)] = _gain_map(
                        joints, values, gain_limits.get(term), '映射后的硬件协议范围')
            compensation = fields.get('hold_compensation')
            if item == 'rigid_tracking':
                # Older baseline candidates were also tested with compensation
                # disabled; missing metadata must never revive an old trim
                if compensation is None:
                    compensation = {'enabled': False}
                if not isinstance(compensation, dict) or not isinstance(compensation.get('enabled'), bool):
                    raise ValueError('定位补偿参数不完整')
                verification = 'hold_compensation_verification' if compensation['enabled'] else 'hold_verification'
                if not (gain_measurement_available(fields.get(verification), len(joints), 'hold') or
                        (fields.get(verification) or {}).get('validated') is True):
                    raise ValueError('定位补偿状态缺少对应的保持测量记录')
                hold_compensation = {'enabled': compensation['enabled']}
                for field in ('integral_gain', 'max_torque', 'max_rate', 'contact_threshold'):
                    values = compensation.get(field)
                    if values is None and not compensation['enabled']: continue
                    if not isinstance(values, list): raise ValueError('定位补偿缺少逐轴参数: ' + field)
                    encoded = _gain_map(joints, values, policy.get('max_effort_override') if field == 'max_torque' else None)
                    hold_compensation[field] = yaml.safe_load(encoded)
                for field in ('velocity_threshold', 'error_threshold'):
                    value = compensation.get(field)
                    if value is None and not compensation['enabled']: continue
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                        raise ValueError('定位补偿阈值无效: ' + field)
                    hold_compensation[field] = value
                for key, value in hold_compensation.items():
                    targets[('control', 'hold_compensation', key)] = yaml.safe_dump(value, default_flow_style=True).strip().removesuffix('\n...')
    return targets, hold_compensation


def _gain_export(record, dest, gain_purposes=None):
    folder, metadata, model, core, cfg = _snapshot(record)
    gain = _read_json(folder / 'gain-candidate.json')
    if gain.get('schema') != 'serial_arm_gain_candidate':
        raise ValueError('无法识别增益候选格式，请重新标定或导入有效候选')
    if (gain.get('task_id') != metadata.get('task_id') or
        gain.get('urdf_fingerprint') != metadata.get('urdf_fingerprint') or
        gain.get('joint_names') != metadata.get('joint_names')):
        raise ValueError('增益候选与任务源模型不一致')
    # Selected groups are merged into the current compatible Core so an older
    # calibration cannot roll back unrelated gains adjusted since acquisition
    groups = deployment_gain_groups(gain, gain_purposes)
    core, merge_source, context_warnings = _gain_merge_source(folder, metadata, core, groups)
    cfg=yaml.safe_load(core.read_text(encoding='utf-8'))
    source_text = core.read_text(encoding='utf-8')
    if merge_source == 'task_snapshot':
        measured = recorded_gain_core(cfg, metadata, gain)
        if measured != canonical_core(cfg):
            # A runtime setting can change without saving the Profile. Export
            # the actual measured context, never the old file's defaults
            source_text = yaml.safe_dump(measured, allow_unicode=True, sort_keys=False)
            cfg = measured
    category = gain.get('category')
    if category == 'identification_tracking': category = 'servo_tracking'
    if category not in GAIN_FIELDS:
        raise ValueError('未知增益标定目标')
    joints = metadata['joint_names']
    hardware = None
    for filename in ('hardware-effective.yaml', 'hardware.yaml'):
        snapshot = folder / 'source' / filename
        if snapshot.is_file():
            hardware = yaml.safe_load(snapshot.read_text(encoding='utf-8'))
            break
    targets, hold_compensation = gain_candidate_targets(gain, cfg, joints, gain_purposes, hardware)
    result = _replace_core(source_text, targets, hold_compensation=hold_compensation)
    # Keep model.urdf_path unchanged because this artifact intentionally contains
    # only core.yaml; changing to a missing relative URDF would break deployment
    (dest / 'core.yaml').write_text(result, encoding='utf-8')
    evidence = dest / 'evidence'; evidence.mkdir(exist_ok=True)
    for source in [folder / 'metadata.json', folder / 'gain-candidate.json',
                   *sorted(folder.glob('servo-probe-*.csv')), *sorted(folder.glob('gain-trajectory-*.csv')),
                   *sorted(folder.glob('drag-decay-*.csv')), *sorted(folder.glob('gain-hold-*.csv')),
                   folder / 'frames.csv', folder / 'result.json']:
        if source.is_file() and not source.is_symlink(): shutil.copy2(source, evidence / source.name)
    snapshots = evidence / 'source'; snapshots.mkdir(exist_ok=True)
    for name in ('core.yaml', 'model.urdf', 'hardware.yaml', 'hardware-effective.yaml', 'gravity-correction.yaml'):
        source = folder / 'source' / name
        if source.is_file() and not source.is_symlink(): shutil.copy2(source, snapshots / name)
    quality = gain_selection_report(gain, groups)
    summary = ['增益候选导出', '', '配置来源：' + ('当前兼容配置' if merge_source == 'current_core' else '标定任务快照与实测控制上下文'),
               '任务：' + str(metadata.get('task_id') or ''),
               '用途：' + '、'.join(groups), '',
               'core.yaml 为独立导出文件，未修改当前机械臂配置；部署前请核对模型及资源路径']
    if context_warnings:
        summary += ['', '上下文提示：', *context_warnings,
                    '本文件保留标定时的映射、控制与硬件条件，未合并当前配置']
    if quality['warnings']:
        summary += ['', '测量质量提示：', *quality['warnings']]
    (dest / 'README.txt').write_text('\n'.join(summary) + '\n', encoding='utf-8')
    return {'target': category, 'gain_purposes':list(groups), **quality,
            'validation_scope':gain.get('validation_scope','recorded_scope'), 'merge_source':merge_source,
            'context_warnings':context_warnings, 'files_only': ['core.yaml']}


def _gain_merge_source(folder, metadata, source_core, purposes):
    """Merge only when the recorded deployment context is still unchanged"""
    current_path = metadata.get('core_config_path')
    if not current_path:
        return source_core, 'task_snapshot', []
    current = Path(current_path).expanduser().resolve()
    if not current.is_file() or current == source_core:
        return source_core, 'task_snapshot', []
    try:
        live = yaml.safe_load(current.read_text(encoding='utf-8'))
        if not isinstance(live, dict):
            raise ValueError('当前 Core 配置不是有效映射')
        runtime = ((live.get('control') or {}).get('runtime') or {})
        hardware = {}
        if runtime.get('write_enabled') is not False:
            # Task metadata records the acquired driver, not a current Profile
            # declaration. Reusing that value as the current driver would make
            # the comparison tautological after a Profile switched backends
            raise ValueError('历史记录没有当前 Profile 的驱动声明，无法确认当前硬件上下文')
        info = {'profile': metadata.get('profile'), 'resources': {'core': str(current)},
                'hardware_plugin': (metadata.get('runtime_provenance') or {}).get('hardware_plugin_file_path'),
                'connection_overrides': metadata.get('hardware_overrides')}
        verify_gain_context(folder, info, live, hardware, purposes)
    except (ValueError, OSError, yaml.YAMLError) as error:
        return source_core, 'task_snapshot', ['当前配置未合并，导出采用标定时配置：' + str(error)]
    return current, 'current_core', []


def _gravity_merge_source(folder, metadata, source_core):
    # Prefer the currently deployed Core configuration only when it still
    # describes the exact immutable source URDF and ordered joint mapping
    current_path = metadata.get('core_config_path')
    if not current_path:
        return source_core, 'task_snapshot'
    current = Path(current_path).expanduser().resolve()
    if not current.is_file() or current == source_core:
        return source_core, 'task_snapshot'
    live = yaml.safe_load(current.read_text(encoding='utf-8'))
    if not isinstance(live, dict) or not isinstance(live.get('model'), dict):
        raise ValueError('当前 Core 配置格式无效，拒绝用历史快照覆盖已调增益')
    if list(live['model'].get('joint_names') or []) != metadata.get('joint_names'):
        raise ValueError('当前 Core 的关节映射与重力候选不一致，拒绝覆盖已有增益')
    model_path = Path(str(live['model'].get('urdf_path') or ''))
    if not model_path.is_absolute():
        model_path = current.parent / model_path
    if not matches_recorded_fingerprint(model_path.resolve(), metadata.get('urdf_fingerprint')):
        raise ValueError('当前 Core 所用 URDF 已变化，请先检查模型来源，不能自动合并历史重力候选')
    if not isinstance(((live.get('control') or {}).get('controller')), dict):
        raise ValueError('当前 Core 缺少控制增益配置，拒绝自动合并')
    return current, 'current_core'


def _same_dynamics_model(source, candidate):
    """Compare model parameters, not XML layout or packaged visual resources

    Exporters serialize floating point values with 12 significant digits.  That
    roundoff must not revoke an unchanged model's operator validation, whereas
    changed inertial parameters or joint definitions must revoke it
    """
    numeric_attributes = {'value', 'xyz', 'rpy', 'ixx', 'ixy', 'ixz', 'iyy',
                          'iyz', 'izz', 'lower', 'upper', 'effort', 'velocity',
                          'damping', 'friction', 'multiplier', 'offset'}

    def same_attribute(name, left, right):
        if name not in numeric_attributes:
            return left == right
        try:
            a, b = [float(v) for v in left.split()], [float(v) for v in right.split()]
        except ValueError:
            return False
        return (len(a) == len(b) and bool(a) and
                all(math.isfinite(x) and math.isfinite(y) and
                    math.isclose(x, y, rel_tol=1e-11, abs_tol=1e-14)
                    for x, y in zip(a, b)))

    def same_element(left, right):
        if left is None or right is None:
            return left is right
        if left.tag != right.tag or left.attrib.keys() != right.attrib.keys():
            return False
        if any(not same_attribute(key, value, right.attrib[key])
               for key, value in left.attrib.items()):
            return False
        if (left.text or '').strip() != (right.text or '').strip():
            return False
        key = lambda element: (element.tag, element.get('name', ''))
        a, b = sorted(left, key=key), sorted(right, key=key)
        return len(a) == len(b) and all(same_element(x, y) for x, y in zip(a, b))

    source_root, candidate_root = ET.parse(source).getroot(), ET.parse(candidate).getroot()
    for tag in ('link', 'joint'):
        left = sorted(source_root.findall(tag), key=lambda element: element.get('name', ''))
        right = sorted(candidate_root.findall(tag), key=lambda element: element.get('name', ''))
        if len(left) != len(right):
            return False
        for a, b in zip(left, right):
            if a.get('name') != b.get('name'):
                return False
            if not same_element(a.find('inertial') if tag == 'link' else a,
                                b.find('inertial') if tag == 'link' else b):
                return False
    return True


def _gravity_export(record,dest,candidate_source=None,candidate_path=None):
    folder,metadata,model,core,cfg=_snapshot(record)
    core,core_source=_gravity_merge_source(folder,metadata,core)
    from persistence import select_gravity_candidate
    candidate_source, candidate_path = select_gravity_candidate(folder, candidate_source, candidate_path)
    offline=candidate_source=='offline'
    _,_,result,payload=build_gravity_correction_payload(folder,allow_unvalidated=True,prefer_recomputed=offline,candidate_path=candidate_path)
    if offline and payload.get('candidate_source') != 'recomputed-candidate.json':
        raise ValueError('此任务尚未生成离线重算候选，请先离线重算')
    verified = bool(payload.get('static_pass'))
    friction_verified = bool(verified and payload.get('friction_pass') and payload.get('friction_matches_gravity'))
    # Evaluation results control application, not the right to inspect a complete
    # finite physical candidate. Unsafe or structurally invalid candidates remain blocked

    # Use the existing geometry-preserving exporter, not an independently fitted model
    with tempfile.TemporaryDirectory(prefix='serialarm-gravity-') as staging:
        if offline:
            # Run the optional native dynamics stack in a separate process
            import os
            import subprocess
            import sys
            worker=Path(__file__).with_name('full_inertial_worker.py')
            env=dict(os.environ)
            env['PYTHONNOUSERSITE']='1'
            completed=subprocess.run([sys.executable,str(worker),str(folder),str(staging),
                                      'legacy_gravity_export',json.dumps({'candidate_path': str(candidate_path)})],capture_output=True,
                                      text=True,timeout=120,env=env)
            if completed.returncode:
                raise ValueError('离线重力候选导出失败 '+(completed.stderr or '')[-1400:])
            details=json.loads(completed.stdout)
        else:
            details=export_candidate_urdf(folder,staging)
        source=Path(details['candidate_urdf'])
        target=dest/'robot.urdf'
        shutil.copy2(source,target)
        resources=source.parent/'resources'
        if resources.is_dir():shutil.copytree(resources,dest/'resources')
        ET.parse(target)
        if offline:
            # Carry the exact exported candidate alongside the deployment files
            report_source = source.parent / 'recomputed-candidate.json'
            report_digest = hashlib.sha256(report_source.read_bytes()).hexdigest()
            if report_digest != payload.get('candidate_report_sha256'):
                raise ValueError('导出过程中离线候选发生变化，请重新导出')
            shutil.copy2(report_source, dest / 'identification-report.json')
            verification = _read_json(source.parent / 'candidate-urdf-verification.json')
            if (verification.get('candidate_report_sha256') != report_digest or
                    verification.get('candidate_urdf_sha256') != hashlib.sha256(target.read_bytes()).hexdigest()):
                raise ValueError('候选模型或报告与导出校验记录不一致')
            verification.update(candidate_urdf='robot.urdf', candidate_source='identification-report.json')
            (dest / 'candidate-verification.json').write_text(
                json.dumps(verification, ensure_ascii=False, indent=2), encoding='utf-8')
    joints=metadata['joint_names']
    friction=payload['residual']['friction']
    residual=payload['residual']
    targets={('model','urdf_path'):'robot.urdf',
             ('model','gravity_scale'):_gain_map(joints,[1.0]*len(joints)),
             ('capability','admittance','calibration','torque_bias'):_gain_map_signed(joints,residual['torque_bias']),
             ('capability','admittance','calibration','torque_threshold'):_gain_map(joints,residual['torque_threshold']),
             ('capability','admittance','calibration','friction','enabled'):'true' if friction_verified else 'false',
             ('capability','admittance','calibration','friction','velocity_transition'):str(float(friction['velocity_transition']))}
    for name in ('positive_coulomb','positive_viscous','negative_coulomb','negative_viscous'):
        targets[('capability','admittance','calibration','friction',name)]=_gain_map_signed(joints,friction[name])
    core_text=core.read_text(encoding='utf-8')
    export_cfg=yaml.safe_load(core_text)
    recovery=(export_cfg.get('safety_policy') or {}).get('fault_recovery') or {}
    scale_changed=any(export_cfg['model']['gravity_scale'].get(joint) != 1.0 for joint in joints)
    if (recovery.get('gravity_model_validated') is True and
            (scale_changed or not _same_dynamics_model(model,target))):
        # Static holdout scores do not validate native compliant recovery on a
        # changed URDF or gravity scale. Keep the user's allow_compliant_recovery
        # preference; only revoke inherited model-specific hardware validation
        targets[('safety_policy','fault_recovery','gravity_model_validated')]='false'
    result_core=_replace_core(core_text,targets)
    (dest/'core.yaml').write_text(result_core,encoding='utf-8')
    return {'target':'gravity_and_friction','validated':verified and friction_verified,
            'core_merge_source':core_source,
            'review_required':not verified or not friction_verified,
            'static_pass':verified,'friction_pass':friction_verified,
            'friction_enabled':friction_verified,
            'torque_bias_source':payload.get('torque_bias_source'),
            'candidate_source': candidate_source, 'candidate_path': payload.get('candidate_path'), 'candidate_run_id': payload.get('candidate_run_id'),
            'mass_scope':'joint_mass_and_com' if offline else 'original_mass_and_corrected_com'}


def _gain_map_signed(joints,values):
    if not isinstance(values,(list,tuple)) or len(joints)!=len(values):
        raise ValueError('标定参数的维度与受控关节不一致')
    if any(not isinstance(x,(float,int)) or not math.isfinite(x) for x in values):
        raise ValueError('标定参数包含无效数值')
    return '{'+', '.join(f'{j}: {float(v):.12g}' for j,v in zip(joints,values))+'}'


def _dynamics_export(record,dest,report_path=None):
    folder,metadata,model,core,cfg=_snapshot(record)
    available=list(folder.glob('advanced_candidates/*/*/identification-report.json'))
    if (folder/'identification-report.json').is_file():available.append(folder/'identification-report.json')
    available=[p.resolve() for p in available if p.is_file()]
    if report_path:
        report=Path(report_path).expanduser().resolve()
        if report not in available:raise ValueError('所选辨识候选不属于该任务')
    elif len(available) == 1:
        report=available[0]
    elif available:
        raise ValueError('当前任务包含多轮候选，请先选择具体辨识报告再导出')
    else:raise ValueError('未找到高级辨识成果，请先完成所选目标的离线辨识')
    data=_read_json(report)
    if data.get('schema')!='serial_arm_advanced_identification' or data.get('task_id')!=metadata.get('task_id'):
        raise ValueError('辨识报告与任务不匹配')
    if data.get('status')!='review_passed_offline' or data.get('full_dynamics_physical_pass') is not True:
        raise ValueError('所选高级辨识候选未通过离线物理与数据验收，不能导出为部署模型')
    candidate=resolve_dynamics_candidate_path(report, data)
    if hashlib.sha256(candidate.read_bytes()).hexdigest()!=data.get('candidate_urdf_sha256'):
        raise ValueError('候选 URDF 与审核时的模型指纹不同')
    # Candidate-local resources may have been remapped from original source paths
    _bundle_model(candidate,metadata,dest)
    return {'target':data.get('identification_mode'),'validated_offline':True,
            'validated_on_hardware':False}


def export_artifact(root,directory,kind,*,report=None,destination=None,candidate_source=None,candidate_path=None,gain_purposes=None):
    if kind not in ('teaching','gain','gravity','dynamics'):raise ValueError('未知标定功能')
    record=load(directory,kind)
    if destination:
        base=Path(destination).expanduser().resolve()
        if not base.is_dir():raise ValueError('导出目录不存在')
    else:
        base=Path(root).resolve()/'.serialarm'/'exports'
        base.mkdir(parents=True,exist_ok=True)
    folder=Path(record['directory']).resolve()
    if base==folder or base.is_relative_to(folder):
        raise ValueError('导出目录不能放在原始任务内部')
    with tempfile.TemporaryDirectory(prefix='serialarm-artifact-') as temporary:
        stage=Path(temporary)
        if kind=='teaching':
            source,metadata,model,core,cfg=_snapshot(record)
            if not (source/'trajectory.csv').is_file() or not (record.get('trajectory') or {}).get('valid'):
                raise ValueError('示教轨迹未通过原始任务数据校验')
            shutil.copy2(source/'trajectory.csv',stage/'trajectory.csv')
            shutil.copy2(source/'metadata.json',stage/'metadata.json')
            snapshot_dir=stage/'source';snapshot_dir.mkdir()
            shutil.copy2(core,snapshot_dir/'core.yaml')
            shutil.copy2(model,snapshot_dir/'model.urdf')
            for name in ('trajectory.checkpoint.csv',):
                if (source/name).is_file():shutil.copy2(source/name,stage/name)
            payload={'target':'recorded_joint_trajectory','validated':True}
        elif kind=='gain':payload=_gain_export(record,stage,gain_purposes)
        elif kind=='gravity':payload=_gravity_export(record,stage,candidate_source,candidate_path)
        else:payload=_dynamics_export(record,stage,report)
        if not list(stage.rglob('*')):raise ValueError('导出没有产生任何文件')
        # Ensure all resulting YAML and URDF files can be loaded
        for file in stage.rglob('*.yaml'):yaml.safe_load(file.read_text(encoding='utf-8'))
        for file in stage.rglob('*.urdf'):ET.parse(file)
        name=f'{folder.name}-{kind}-artifact-{int(time.time())}.zip'
        out=base/name
        if out.exists():raise ValueError('导出文件已存在，请重试')
        with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
            for file in sorted(stage.rglob('*')):
                if file.is_file() and not file.is_symlink():archive.write(file,file.relative_to(stage).as_posix())
        return {'path':str(out),'files':[p.relative_to(stage).as_posix() for p in stage.rglob('*') if p.is_file()],
                'kind':kind,'task_id':record.get('task_id'),**payload}
