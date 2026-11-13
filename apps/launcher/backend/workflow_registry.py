"""Read-only cross-tool history inventory and explicit artifact provenance

Task records are immutable data sources.  The registry never authorizes hardware
motion; Core must independently check fingerprints and robot lifecycle state
"""
from __future__ import annotations
import csv
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import time
import zipfile
from persistence import load_model_calibration_summary, list_model_calibration_records

KINDS = ('teaching', 'gain', 'gravity', 'dynamics')
LABELS = {'teaching':'拖拽示教','gain':'执行器增益自动标定','gravity':'重力模型校正','dynamics':'高级动力学辨识'}


def _json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError, UnicodeError):
        return None


def classify(record):
    directory = Path(record['directory'])
    result = record.get('result') or {}
    kinds = set()
    if record.get('trajectory', {}).get('valid'):
        kinds.update(KINDS[:3])  # gain / gravity can reuse a demonstrated path
    if (directory / 'gain-candidate.json').is_file():
        kinds.add('gain')
    if (directory / 'accepted_static.csv').is_file() or (result.get('gravity_result') or {}).get('first_moments'):
        kinds.update(('gravity', 'dynamics'))
    if (directory / 'frames.csv').is_file() and (directory / 'metadata.json').is_file():
        kinds.add('dynamics')  # actual solver performs its own observability checks
    if (directory / 'identification-report.json').is_file() or list(directory.glob('candidate-*/identification-report.json')) or list(directory.glob('advanced_candidates/*/*/identification-report.json')):
        kinds.add('dynamics')
    return kinds


def record_summary(record, kind):
    directory = Path(record['directory'])
    gain = _json(directory/'gain-candidate.json')
    reports=[]
    if kind=='dynamics':
        candidates=list(directory.glob('advanced_candidates/*/*/identification-report.json'))
        if (directory/'identification-report.json').is_file():candidates.append(directory/'identification-report.json')
        for report_path in sorted(candidates,key=lambda p:p.stat().st_mtime,reverse=True)[:30]:
            report=_json(report_path)
            if report and report.get('schema')=='serial_arm_advanced_identification':
                reports.append({**report,'report':str(report_path)})
    return {'kind':kind, 'task_id':record.get('task_id'), 'directory':str(directory),
            'metadata':{key: (record.get('metadata') or {}).get(key) for key in ('profile','urdf_fingerprint','core_fingerprint','joint_names')},
            'source_task_id': (record.get('metadata') or {}).get('source_task_id'),
            'trajectory':record.get('trajectory'), 'record_kind':record.get('record_kind'),
            'gain_candidate':gain, 'dynamic_candidates':reports, 'has_result':bool(record.get('has_result')),
            'available_files':record.get('files'), 'note':record.get('note')}


def scan_page(root, kind, limit=60, *, offset=0, query='', legacy_root=None, profile_name=None):
    if kind not in KINDS: raise ValueError('未知的工具类型')
    if not isinstance(limit,int) or isinstance(limit,bool) or not 1 <= limit <= 150: raise ValueError('任务数量超限')
    if not isinstance(offset,int) or isinstance(offset,bool) or offset < 0: raise ValueError('任务起点无效')
    if not isinstance(query,str) or len(query) > 200: raise ValueError('任务搜索内容过长')
    sources = list_model_calibration_records(root, None, legacy_root=legacy_root, profile_name=profile_name)
    filtered = []
    query = query.casefold().strip()
    for record in sources:
        if kind not in classify(record): continue
        metadata = record.get('metadata') or {}
        if query and query not in json.dumps({'task_id':record.get('task_id'), 'metadata':metadata, 'note':record.get('note')}, ensure_ascii=False).casefold(): continue
        filtered.append(record)
    return {'records':[record_summary(record,kind) for record in filtered[offset:offset+limit]],
            'total':len(filtered), 'offset':offset, 'limit':limit, 'has_more':offset+limit < len(filtered)}


def scan(root, kind, limit=60, *, offset=0, query='', legacy_root=None, profile_name=None):
    return scan_page(root, kind, limit, offset=offset, query=query, legacy_root=legacy_root, profile_name=profile_name)['records']


def candidates(directory, kind):
    if kind not in ('gravity','dynamics'): raise ValueError('此工具没有多轮模型候选')
    record = load_model_calibration_summary(directory)
    folder = Path(record['directory'])
    items = []
    if kind == 'gravity':
        online = (record.get('result') or {}).get('gravity_result')
        if online:
            items.append({'id':'online', 'kind':kind, 'source':'online', 'path':str(folder/'result.json'),
                'created_at':(folder/'result.json').stat().st_mtime, 'summary':online})
        paths = list(folder.glob('offline_candidates/*/recomputed-candidate.json'))
        if (folder/'recomputed-candidate.json').is_file(): paths.append(folder/'recomputed-candidate.json')
    else:
        paths = list(folder.glob('advanced_candidates/*/*/identification-report.json')) + list(folder.glob('candidate-*/identification-report.json')) + list(folder.glob('idim_candidates/*/idim-report.json'))
        if (folder/'identification-report.json').is_file(): paths.append(folder/'identification-report.json')
    seen = set()
    for path in sorted(paths, key=lambda p:p.stat().st_mtime, reverse=True):
        report = _json(path)
        if not isinstance(report,dict) or report.get('task_id') != record['task_id']: continue
        key = report.get('candidate_run_id') or report.get('run_id') or hashlib.sha256(path.read_bytes()).hexdigest()
        if key in seen: continue
        seen.add(key)
        # Prefer the immutable archive over its mutable latest pointer
        archived = folder / str(report.get('archived_report') or '')
        if report.get('archived_report') and archived.is_file() and archived.resolve().is_relative_to(folder): path = archived
        items.append({'id':str(key), 'kind':kind, 'source':'offline', 'path':str(path.resolve()),
            'format':'idim' if report.get('schema')=='serial_arm_idim_candidate_v1' else 'advanced' if kind=='dynamics' else 'gravity',
            'created_at':path.stat().st_mtime, 'summary':report})
    return {'directory':str(folder), 'kind':kind, 'candidates':items}


def load(directory, kind, *, candidate_path=None, report=None):
    if kind not in KINDS:raise ValueError('未知的工具类型')
    record=load_model_calibration_summary(directory)
    if kind not in classify(record): raise ValueError('当前任务记录不包含此工具所需的数据')
    result = {**record_summary(record,kind), **record}
    if candidate_path or report:
        path = str(Path(candidate_path or report).expanduser().resolve())
        available = candidates(directory,kind)['candidates']
        selected = next((item for item in available if item['path']==path),None)
        if selected is None: raise ValueError('所选候选不属于当前任务')
        result['selected_candidate'] = selected
        if kind == 'gravity': result['recomputed_candidate'] = selected['summary'] if selected['source']=='offline' else None
        else:
            if selected.get('format') == 'idim':
                result['idim_candidate'] = {**selected['summary'], 'report_path':selected['path']}
                result['dynamic_candidates'] = []
            else:
                result['dynamic_candidates'] = [{**selected['summary'], 'report':selected['path']}]
    return result


def export_record(root, directory, kind):
    if kind not in KINDS: raise ValueError('未知的工具类型')
    record=load(directory,kind)
    folder=Path(record['directory'])
    root=Path(root).resolve()
    # No symbolic links in exported task tree; avoid recursive and remote paths
    if not folder.is_dir():raise ValueError('任务记录不存在')
    base=root/'.serialarm'/'exports'
    base.mkdir(parents=True,exist_ok=True)
    out=base/f'{folder.name}-{kind}-{int(time.time())}.zip'
    allowed={
        'teaching':{'metadata.json','trajectory.csv','trajectory.checkpoint.csv','source/model.urdf','source/core.yaml'},
        'gain':{'metadata.json','trajectory.csv','gain-candidate.json','frames.csv','result.json','source/model.urdf','source/core.yaml','source/hardware.yaml','source/hardware-effective.yaml','source/gravity-correction.yaml'},
        'gravity':{'metadata.json','trajectory.csv','frames.csv','accepted_static.csv','recomputed-candidate.json','result.json','candidate-offline.urdf','source/model.urdf','source/core.yaml'},
        'dynamics':{'metadata.json','trajectory.csv','frames.csv','accepted_static.csv','result.json','idim-candidate.json','source/model.urdf','source/core.yaml'}
    }[kind]
    if kind == 'gain':
        for pattern in ('servo-probe-*.csv', 'gain-trajectory-*.csv', 'drag-decay-*.csv', 'gain-hold-*.csv', 'gain-transition-*.csv'):
            allowed.update(path.name for path in folder.glob(pattern) if path.is_file())
    included=[]
    with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for name in sorted(allowed):
            path=folder/name
            if path.is_file() and not path.is_symlink():
                archive.write(path,arcname=name)
                included.append(name)
        if kind in ('dynamics', 'gravity'):
            pattern = 'advanced_candidates/**/**' if kind == 'dynamics' else 'offline_candidates/*/*.json'
            for path in sorted(folder.glob(pattern)):
                if not path.is_file() or path.is_symlink() or not path.resolve().is_relative_to(folder):
                    continue
                if path.stat().st_size > 200_000_000:
                    raise ValueError('候选文件过大，无法完整导出')
                if len(included)>=300:raise ValueError('候选文件过多，超出导出上限')
                archive.write(path,arcname=str(path.relative_to(folder)))
                included.append(str(path.relative_to(folder)))
    if not included:
        out.unlink(missing_ok=True)
        raise ValueError('记录中没有可以导出的内容')
    return {'path':str(out),'files':included,'kind':kind,'task_id':record.get('task_id')}


def prepare_gravity_candidate(directory, *, prefer_recomputed=False, candidate_path=None, candidate_source=None):
    """Stage an ephemeral validated package, without changing Core YAML"""
    import yaml
    from persistence import build_gravity_correction_payload
    from persistence import select_gravity_candidate
    source, selected = select_gravity_candidate(directory, candidate_source or ('offline' if prefer_recomputed else None), candidate_path)
    folder, metadata, _, payload=build_gravity_correction_payload(directory,prefer_recomputed=source=='offline',candidate_path=selected)
    if prefer_recomputed and payload.get('candidate_source') != 'recomputed-candidate.json':
        raise ValueError('缺少离线重力候选，请先执行离线重算')
    # Native checks actual robot URDF fingerprint again, on every apply
    folder=Path(folder)
    receipt = payload.get('candidate_report_sha256')
    target=folder/'candidate'/('gravity-runtime-preview-'+receipt[:16]+'.yaml' if receipt else 'gravity-runtime-preview.yaml')
    target.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=target.parent,delete=False) as tmp:
        yaml.safe_dump(payload,tmp,allow_unicode=True,sort_keys=False)
        name=tmp.name
    try:
        Path(name).replace(target)
    finally:
        Path(name).unlink(missing_ok=True)
    return {'kind':'gravity','candidate_path':str(target),'task_id':metadata.get('task_id'),
            'urdf_fingerprint':metadata.get('urdf_fingerprint'),'temporary':True, 'candidate_source':source, 'selected_candidate_path':str(selected) if selected else str(folder/'result.json'), 'candidate_run_id':payload.get('candidate_run_id')}

def prepare_gain_candidate(directory, gain_purposes=None):
    record=load(directory,'gain')
    path=Path(record['directory'])/'gain-candidate.json'
    candidate=_json(path)
    if not candidate or candidate.get('schema')!='serial_arm_gain_candidate':
        raise ValueError('当前任务缺少有效的增益候选文件')
    if candidate.get('task_id') != record.get('task_id'):
        raise ValueError('增益候选与任务编号不一致，请重新选择完整任务目录')
    if candidate.get('urdf_fingerprint')!=(record.get('metadata') or {}).get('urdf_fingerprint'):
        raise ValueError('增益候选与记录的 URDF 指纹不一致')
    if candidate.get('joint_names')!=(record.get('metadata') or {}).get('joint_names'):
        raise ValueError('增益候选与记录的关节映射不一致')
    from control_parameters import deployment_gain_groups, gain_selection_report
    groups=deployment_gain_groups(candidate,gain_purposes)
    return {'kind':'gain','candidate_path':str(path),'task_id':record['task_id'],'temporary':True,
            'gain_purposes':list(groups), **gain_selection_report(candidate, groups),
            'validation_scope':candidate.get('validation_scope','recorded_scope')}


def resolve_dynamics_candidate_path(report_path, report):
    """Resolve a report-local URDF, including relocated legacy absolute paths"""
    report_file = Path(report_path).expanduser().resolve(strict=True)
    value = report.get('candidate_urdf')
    if not isinstance(value, str) or not value.strip():
        raise ValueError('离线验证报告缺少模型 URDF 路径')
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        # Legacy reports stored the acquisition machine's absolute directory.
        # Bind to the copy beside this report even if the old file still exists
        candidate = report_file.parent / candidate.name
    else:
        candidate = report_file.parent / candidate
    candidate = candidate.resolve(strict=True)
    if not candidate.is_file() or candidate.parent != report_file.parent:
        raise ValueError('模型 URDF 必须与离线验证报告位于同一候选目录')
    return candidate


def prepare_dynamics_candidate(report_path):
    """Offline-reviewed dynamic URDF preview, checked again by native Core"""
    report_file=Path(report_path).expanduser().resolve(strict=True)
    report=_json(report_file)
    if report_file.name!='identification-report.json' or not report or report.get('schema')!='serial_arm_advanced_identification':
        raise ValueError('请选择有效的高级辨识候选报告')
    if report.get('status')!='review_passed_offline' or report.get('validation_scope')!='dynamic_inertia' or not report.get('full_dynamics_physical_pass'):
        raise ValueError('当前候选未通过完整动力学的离线审核和物理约束验收')
    candidate=resolve_dynamics_candidate_path(report_file, report)
    digest=hashlib.sha256(candidate.read_bytes()).hexdigest()
    if not report.get('candidate_urdf_sha256') or report['candidate_urdf_sha256']!=digest:
        raise ValueError('候选 URDF 已变化，必须重新执行离线辨识')
    return {'kind':'dynamics','report':str(report_file),'candidate_path':str(candidate),'temporary':True}


def validate_gain_trajectory(directory, response_window, purpose):
    """Assess a fresh recorded reference path with a frozen, qualified gain set

    This adds independent path evidence; it never upgrades local qualification
    into unmeasured workspace or contact certification, or changes runtime gains
    """
    import math
    from control_parameters import deployment_gain_groups, selected_gain_purposes
    record=load(directory,'gain')
    folder=Path(record['directory'])
    candidate_bytes=(folder/'gain-candidate.json').read_bytes()
    candidate=json.loads(candidate_bytes)
    keys=selected_gain_purposes([purpose])
    groups=deployment_gain_groups(candidate,keys)
    key=keys[0]
    if key=='compliant_drag': raise ValueError('拖拽使用释放稳定性验收，轨迹跟踪验收请选择刚性、柔性或伺服用途')
    metadata=record.get('metadata') or {}
    if (candidate.get('task_id')!=record.get('task_id') or candidate.get('joint_names')!=metadata.get('joint_names') or
            candidate.get('urdf_fingerprint')!=metadata.get('urdf_fingerprint')):
        raise ValueError('增益候选与原标定任务不匹配')
    if not isinstance(response_window,dict) or response_window.get('joint_names')!=candidate.get('joint_names'):
        raise ValueError('路径响应的关节顺序与候选不一致')
    from model_calibration_source import verified_source_urdf
    source=verified_source_urdf(folder,metadata)
    identity=response_window.get('model_identity') or {}
    if identity.get('urdf_sha256')!=hashlib.sha256(Path(source).read_bytes()).hexdigest():
        raise ValueError('路径响应的模型来源与候选不同或未记录，请应用当前候选并重新录制')
    if metadata.get('profile') and response_window.get('profile')!=metadata['profile']:
        raise ValueError('路径响应来自其他机械臂 Profile')
    names=candidate['joint_names'];n=len(names)
    if response_window.get('joint_units', ['rad']*n)!=['rad']*n:
        raise ValueError('当前增益候选的独立路径验收仅支持转动关节')
    samples=response_window.get('samples') or []
    if not isinstance(samples,list) or not 100<=len(samples)<=30000:
        raise ValueError('独立路径验收需要 100～30000 帧新反馈')
    def number(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    fields=groups[key]
    for row in samples:
        if not isinstance(row,dict) or not number(row.get('time_s')): raise ValueError('路径时间戳无效')
        for field in ('accepted','pos','kp','kd'):
            if not isinstance(row.get(field),list) or len(row[field])!=n or not all(number(v) for v in row[field]):
                raise ValueError('路径响应缺少有效逐轴 '+field)
        if any(not math.isclose(row[field][i],fields[field][i],rel_tol=1e-7,abs_tol=1e-5) for field in ('kp','kd') for i in range(n)):
            raise ValueError('实际执行增益与所选候选不同，请应用候选后重新录制')
        age=row.get('feedback_age_ms')
        if not number(age) or not 0<=age<=100: raise ValueError('路径响应含过期或未知年龄反馈')
        if not number(row.get('feedback_receive_ns')) or row['feedback_receive_ns']<=0:
            raise ValueError('路径响应缺少传输接收时间戳，请使用当前程序重新录制')
    times=[row['time_s'] for row in samples]
    stamps=[row['feedback_receive_ns'] for row in samples]
    if times[-1]-times[0]<2 or any(not 0<b-a<=.15 for a,b in zip(times,times[1:])):
        raise ValueError('路径响应需至少 2 秒连续记录，帧间隔不得超过 150 ms')
    if any(b<=a for a,b in zip(stamps,stamps[1:])): raise ValueError('路径响应包含重复或回退的实测反馈')
    condition_fields=('reference_behavior','gain_profile','impedance_mode','model_feedforward_mode')
    for field in condition_fields:
        observed=[row.get(field,response_window.get(field)) for row in samples]
        if any(value!=observed[0] for value in observed): raise ValueError('路径记录期间控制条件发生变化: '+field)
    expected={'rigid_tracking':'RIGID','compliant_tracking':'COMPLIANT','servo_tracking':'SERVO_TRACKING'}[key]
    profile=response_window.get('gain_profile')
    if profile is not None and profile!=expected: raise ValueError('路径记录的增益用途与候选不一致')
    requirements=candidate.get('gain_requirements') or {}
    peak=float(requirements.get('peak_error_rad',.012));rms=float(requirements.get('rms_error_rad',.004))
    if not all(math.isfinite(v) and v>0 for v in (peak,rms)): raise ValueError('候选跟踪验收阈值无效')
    saturated=any(row.get('torque_limited') or row.get('reference_limited') or any(row.get('torque_limit_flags') or []) for row in samples)
    joints=[]
    for i,name in enumerate(names):
        error=[row['pos'][i]-row['accepted'][i] for row in samples]
        references=[row['accepted'][i] for row in samples]
        minimum,maximum=min(references),max(references)
        rms_value=math.sqrt(sum(v*v for v in error)/len(error));peak_value=max(abs(v) for v in error)
        joints.append({'joint':name,'unit':'rad','rms_error':rms_value,'peak_error':peak_value,
                       'rms_limit':rms,'peak_limit':peak,'reference_min':minimum,'reference_max':maximum,
                       'excited':maximum-minimum>1e-6,'passed':rms_value<=rms and peak_value<=peak and not saturated})
    report={'schema':'serial_arm_gain_trajectory_validation','task_id':record['task_id'],'purpose':key,
            'validation_scope':'recorded_reference_path','qualification_scope_unchanged':True,
            'passed':all(item['passed'] for item in joints) and any(item['excited'] for item in joints),
            'candidate_sha256':hashlib.sha256(candidate_bytes).hexdigest(),
            'samples':len(samples),'duration_s':times[-1]-times[0],'limited':saturated,'joints':joints,
            'control_conditions':{field:response_window.get(field) for field in condition_fields},
            'recorded_at':time.time(),'trajectory':response_window.get('trajectory'),
            'source':'ordinary_replay_response_recording'}
    if (folder/'gain-candidate.json').read_bytes()!=candidate_bytes: raise ValueError('验收期间候选已变化，请重新录制并验收')
    destination=folder/'gain-trajectory-validations'/str(time.time_ns());destination.mkdir(parents=True)
    payload=destination/'response.json';payload.write_text(json.dumps(response_window,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    report['response_sha256']=hashlib.sha256(payload.read_bytes()).hexdigest()
    report['report_path']=str(destination/'validation.json')
    Path(report['report_path']).write_text(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
    return report
