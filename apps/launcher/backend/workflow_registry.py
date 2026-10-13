"""Read-only cross-tool history inventory and explicit artifact provenance.

Task records are immutable data sources.  The registry never authorizes hardware
motion; Core must independently check fingerprints and robot lifecycle state.
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
LABELS = {'teaching':'拖拽示教','gain':'自适应增益标定','gravity':'重力模型校正','dynamics':'高级动力学辨识'}


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


def scan(root, kind, limit=60):
    if kind not in KINDS: raise ValueError('未知的工具类型')
    if not isinstance(limit,int) or not 1 <= limit <= 150:raise ValueError('任务数量超限')
    sources=list_model_calibration_records(root, max(limit, 60))
    result=[]
    for record in sources:
        if kind in classify(record):
            result.append(record_summary(record,kind))
        if len(result)>=limit:break
    return result


def load(directory, kind):
    if kind not in KINDS:raise ValueError('未知的工具类型')
    record=load_model_calibration_summary(directory)
    if kind not in classify(record):
        raise ValueError('当前任务记录不包含此工具所需的数据')
    return record_summary(record,kind)


def export_record(root, directory, kind):
    if kind not in KINDS: raise ValueError('未知的工具类型')
    record=load(directory,kind)
    folder=Path(record['directory'])
    root=Path(root).resolve()
    # No symbolic links in exported task tree; avoid recursive and remote paths
    if not folder.is_dir():raise ValueError('任务记录不存在')
    base=root/'.install'/'exports'
    base.mkdir(parents=True,exist_ok=True)
    out=base/f'{folder.name}-{kind}-{int(time.time())}.zip'
    allowed={
        'teaching':{'metadata.json','trajectory.csv','trajectory.checkpoint.csv','source/model.urdf','source/core.yaml'},
        'gain':{'metadata.json','trajectory.csv','gain-candidate.json','frames.csv','result.json','source/model.urdf','source/core.yaml'},
        'gravity':{'metadata.json','trajectory.csv','frames.csv','accepted_static.csv','result.json','candidate-offline.urdf','source/model.urdf','source/core.yaml'},
        'dynamics':{'metadata.json','trajectory.csv','frames.csv','accepted_static.csv','result.json','source/model.urdf','source/core.yaml'}
    }[kind]
    included=[]
    with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for name in sorted(allowed):
            path=folder/name
            if path.is_file() and not path.is_symlink():
                archive.write(path,arcname=name)
                included.append(name)
        if kind=='dynamics':
            for path in sorted(folder.glob('advanced_candidates/**/**')):
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


def prepare_gravity_candidate(directory, *, prefer_recomputed=False):
    """Stage an ephemeral validated package, without changing Core YAML."""
    import yaml
    from persistence import build_gravity_correction_payload
    folder, metadata, _, payload=build_gravity_correction_payload(directory,prefer_recomputed=prefer_recomputed)
    # Native checks actual robot URDF fingerprint again, on every apply.
    folder=Path(folder)
    target=folder/'candidate'/'gravity-runtime-preview.yaml'
    target.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=target.parent,delete=False) as tmp:
        yaml.safe_dump(payload,tmp,allow_unicode=True,sort_keys=False)
        name=tmp.name
    try:
        Path(name).replace(target)
    finally:
        Path(name).unlink(missing_ok=True)
    return {'kind':'gravity','candidate_path':str(target),'task_id':metadata.get('task_id'),
            'urdf_fingerprint':metadata.get('urdf_fingerprint'),'temporary':True}

def prepare_gain_candidate(directory):
    record=load(directory,'gain')
    path=Path(record['directory'])/'gain-candidate.json'
    candidate=_json(path)
    if not candidate or candidate.get('schema')!='serial_arm_gain_candidate' or not candidate.get('validated'):
        raise ValueError('当前增益记录尚未通过完整控制质量验证，不能临时应用')
    if candidate.get('urdf_fingerprint')!=(record.get('metadata') or {}).get('urdf_fingerprint'):
        raise ValueError('增益候选与记录的 URDF 指纹不一致')
    if candidate.get('joint_names')!=(record.get('metadata') or {}).get('joint_names'):
        raise ValueError('增益候选与记录的关节映射不一致')
    return {'kind':'gain','candidate_path':str(path),'task_id':record['task_id'],'temporary':True}


def prepare_dynamics_candidate(report_path):
    """Offline-reviewed dynamic URDF preview, checked again by native Core."""
    report_file=Path(report_path).expanduser().resolve(strict=True)
    report=_json(report_file)
    if report_file.name!='identification-report.json' or not report or report.get('schema')!='serial_arm_advanced_identification':
        raise ValueError('请选择有效的高级辨识候选报告')
    if report.get('status')!='review_passed_offline' or report.get('validation_scope')!='dynamic_inertia' or not report.get('full_dynamics_physical_pass'):
        raise ValueError('当前候选未通过完整动力学的离线审核和物理约束验收')
    candidate=Path(report.get('candidate_urdf') or '').expanduser().resolve(strict=True)
    if not candidate.is_file() or candidate.parent!=report_file.parent:
        raise ValueError('模型 URDF 必须与离线验证报告位于同一候选目录')
    digest=hashlib.sha256(candidate.read_bytes()).hexdigest()
    if not report.get('candidate_urdf_sha256') or report['candidate_urdf_sha256']!=digest:
        raise ValueError('候选 URDF 已变化，必须重新执行离线辨识')
    return {'kind':'dynamics','report':str(report_file),'candidate_path':str(candidate),'temporary':True}
