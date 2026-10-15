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
from workflow_registry import load

GAIN_FIELDS = {'rigid_tracking', 'compliant_tracking', 'identification_tracking', 'all'}
GAIN_DESTINATIONS = {
    'rigid_tracking': ('rigid_hold', 'rigid_tracking'),
    'compliant_tracking': ('compliant_hold', 'compliant_tracking'),
    'identification_tracking': ('identification_tracking',),
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


def _replace_core(text, targets, *, include_identification=False):
    if include_identification:
        cfg=yaml.safe_load(text)
        controller=cfg.get('control',{}).get('controller',{})
        if 'identification_tracking' not in controller:
            lines=text.splitlines(keepends=True)
            start=None
            for i,line in enumerate(lines):
                if re.match(r'^  controller:\s*(?:#.*)?$',line):
                    start=i
                    break
            if start is None:raise ValueError('Core 配置缺少 control.controller')
            insert=len(lines)
            for j in range(start+1,len(lines)):
                if lines[j].strip() and not lines[j].lstrip().startswith('#') and re.match(r'^[^ \t]|^  [^ \t]',lines[j]):
                    insert=j
                    break
            seed=controller.get('rigid_tracking')
            if not isinstance(seed,dict):raise ValueError('Core 缺少刚性跟踪增益')
            new=['    # 辨识轨迹回放专用增益\n','    identification_tracking:\n']
            for term in ('kp','kd'):
                values=seed.get(term)
                if not isinstance(values,dict):raise ValueError('Core 刚性跟踪增益格式错误')
                new.append('      '+term+': '+ '{'+', '.join(f'{k}: {float(v):.12g}' for k,v in values.items())+'}'+'\n')
            lines[insert:insert]=new
            text=''.join(lines)
    result=patch_text(text,targets)
    yaml.safe_load(result)
    return result


def _gain_map(joints,values,limits=None):
    if len(joints)!=len(values):raise ValueError('关节增益数量与 Core 关节数量不一致')
    for i,value in enumerate(values):
        if not isinstance(value,(float,int)) or not math.isfinite(value) or value<0:
            raise ValueError(f'{joints[i]} 增益无效')
        if limits is not None and value>float(limits[joints[i]])+1e-9:
            raise ValueError(f'{joints[i]} 增益超过 Core 安全限制')
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


def _gain_export(record, dest):
    folder, metadata, model, core, cfg = _snapshot(record)
    gain = _read_json(folder / 'gain-candidate.json')
    if gain.get('schema') != 'serial_arm_gain_candidate' or gain.get('validated') is not True:
        raise ValueError('增益候选未通过独立轨迹验收，不能导出为运行配置')
    if (gain.get('task_id') != metadata.get('task_id') or
        gain.get('urdf_fingerprint') != metadata.get('urdf_fingerprint') or
        gain.get('joint_names') != metadata.get('joint_names')):
        raise ValueError('增益候选与任务源模型不一致')
    category = gain.get('category')
    if category not in GAIN_FIELDS:
        raise ValueError('未知增益标定目标')
    joints = metadata['joint_names']
    policy = cfg.get('safety_policy') or {}
    groups = (gain.get('targets') or {}) if category == 'all' else {category: gain}
    required = set(GAIN_DESTINATIONS) if category == 'all' else {category}
    if set(groups) != required:
        raise ValueError('增益候选缺少独立验证的标定目标')
    targets = {}
    for item, fields in groups.items():
        if not isinstance(fields, dict) or fields.get('validated') is not True:
            raise ValueError(f'{item} 尚未通过独立验证')
        if item == 'compliant_tracking':
            hold = fields.get('hold_verification') or {}
            if (fields.get('compliant_separation_validated') is not True or
                hold.get('validated') is not True or
                not isinstance(fields.get('tested_speed_fraction'), (float, int)) or
                not 0 < float(fields['tested_speed_fraction']) <= 1):
                raise ValueError('柔性候选缺少刚柔分离或保持稳定性验收，请重新运行柔性增益标定')
            rigid = fields.get('rigid_reference_kp')
            rigid_kd = fields.get('rigid_reference_kd')
            kp, kd = fields.get('kp'), fields.get('kd')
            expected = (groups['rigid_tracking'] if category == 'all' else
                        cfg['control']['controller']['rigid_tracking'])
            def reference_values(term):
                values = expected.get(term)
                return [values[name] for name in joints] if isinstance(values, dict) else values
            expected_kp = reference_values('kp')
            expected_kd = reference_values('kd')
            if (not all(isinstance(x, list) and len(x) == len(joints)
                        for x in (rigid, rigid_kd, kp, kd, expected_kp, expected_kd)) or
                any(not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)
                    for vals in (rigid, rigid_kd, kp, kd, expected_kp, expected_kd)) or
                any(r <= 0 or d <= 0 or
                    not math.isclose(r, ref_r, abs_tol=1e-5) or
                    not math.isclose(d, ref_d, abs_tol=1e-5) or
                    k > 0.65 * r + 1e-6 or v > 0.85 * d + 1e-6
                    for r, d, k, v, ref_r, ref_d in
                    zip(rigid, rigid_kd, kp, kd, expected_kp, expected_kd))):
                raise ValueError('柔性增益与任务源刚性参考不一致')
        for term, limit_field in [('kp', 'max_kp_override'), ('kd', 'max_kd_override')]:
            values = fields.get(term)
            if not isinstance(values, list):
                raise ValueError('增益候选参数不完整')
            gain_map = _gain_map(joints, values, policy.get(limit_field))
            for destination in GAIN_DESTINATIONS[item]:
                targets[('control', 'controller', destination, term)] = gain_map
    result = _replace_core(core.read_text(encoding='utf-8'), targets,
                           include_identification=True)
    # Keep model.urdf_path unchanged because this artifact intentionally contains
    # only core.yaml; changing to a missing relative URDF would break deployment
    (dest / 'core.yaml').write_text(result, encoding='utf-8')
    return {'target': category, 'validated': True, 'files_only': ['core.yaml']}


def _gravity_export(record,dest,candidate_source="online"):
    folder,metadata,model,core,cfg=_snapshot(record)
    if candidate_source not in ('online','offline'):
        raise ValueError('未知重力候选来源')
    offline=candidate_source=='offline'
    _,_,result,payload=build_gravity_correction_payload(folder,allow_unvalidated=True,prefer_recomputed=offline)
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
                                      'legacy_gravity_export','[]'],capture_output=True,
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
    result_core=_replace_core(core.read_text(encoding='utf-8'),targets)
    (dest/'core.yaml').write_text(result_core,encoding='utf-8')
    return {'target':'gravity_and_friction','validated':verified and friction_verified,
            'review_required':not verified or not friction_verified,
            'static_pass':verified,'friction_pass':friction_verified,
            'friction_enabled':friction_verified,
            'torque_bias_source':payload.get('torque_bias_source'),
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
    elif available:
        report=max(available,key=lambda p:p.stat().st_mtime)
    else:raise ValueError('未找到高级辨识成果，请先完成所选目标的离线辨识')
    data=_read_json(report)
    if data.get('schema')!='serial_arm_advanced_identification' or data.get('task_id')!=metadata.get('task_id'):
        raise ValueError('辨识报告与任务不匹配')
    if data.get('status')!='review_passed_offline' or data.get('full_dynamics_physical_pass') is not True:
        raise ValueError('所选高级辨识候选未通过离线物理与数据验收，不能导出为部署模型')
    candidate=Path(data.get('candidate_urdf') or '').expanduser()
    if not candidate.is_file():candidate=report.parent/f"candidate-{data.get('identification_mode')}.urdf"
    candidate=candidate.resolve()
    if candidate.parent!=report.parent or not candidate.is_file():raise ValueError('候选 URDF 路径无效')
    if hashlib.sha256(candidate.read_bytes()).hexdigest()!=data.get('candidate_urdf_sha256'):
        raise ValueError('候选 URDF 与审核时的模型指纹不同')
    # Candidate-local resources may have been remapped from original source paths
    _bundle_model(candidate,metadata,dest)
    return {'target':data.get('identification_mode'),'validated_offline':True,
            'validated_on_hardware':False}


def export_artifact(root,directory,kind,*,report=None,destination=None,candidate_source="online"):
    if kind not in ('teaching','gain','gravity','dynamics'):raise ValueError('未知标定功能')
    record=load(directory,kind)
    if destination:
        base=Path(destination).expanduser().resolve()
        if not base.is_dir():raise ValueError('导出目录不存在')
    else:
        base=Path(root).resolve()/'.install'/'exports'
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
        elif kind=='gain':payload=_gain_export(record,stage)
        elif kind=='gravity':payload=_gravity_export(record,stage,candidate_source)
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
