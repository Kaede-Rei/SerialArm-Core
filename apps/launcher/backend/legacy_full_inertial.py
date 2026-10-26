"""Offline, opt-in full-inertial URDF candidate from completed model calibration.

Never modifies the source URDF or running robot. Rank-deficient fits are marked
prior-constrained, not fully identified. Raw motor torques need verified signs,
no external contacts, and accurate joint accelerations to be trusted.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def physical_inertia(mass, com, inertia, *, min_mass=1e-7):
    """Require SPD center inertia and principal-moment triangle inequalities."""
    import numpy as np
    if not np.isfinite(mass) or mass <= min_mass or not np.isfinite(com).all():
        return False
    tensor = np.asarray(inertia, dtype=float)
    if tensor.shape != (3, 3) or not np.isfinite(tensor).all():
        return False
    if np.max(np.abs(tensor - tensor.T)) > 1e-9:
        return False
    eig = np.linalg.eigvalsh(tensor)
    return bool(eig[0] > 1e-10 and eig[2] <= eig[0] + eig[1] + 1e-9)


def _prepare_inputs(directory):
    directory = Path(directory).expanduser().resolve()
    metadata = json.loads((directory / 'metadata.json').read_text())
    result = json.loads((directory / 'result.json').read_text())
    if result.get('task_id') != metadata.get('task_id'):
        raise ValueError('任务记录的任务标识不一致')
    if not (directory / 'frames.csv').is_file():
        raise ValueError('缺少 frames.csv')
    # Use a fingerprint-verified snapshot, never an unverified live CAD file
    from model_calibration_source import verified_source_urdf
    snapshot = verified_source_urdf(directory, metadata)
    source = Path(metadata['urdf_path']).expanduser().resolve()
    return directory, metadata, result, snapshot, source


def _articulated_mapping(root, model, sampled_joint_names):
    """Map only measured 1-DoF joints to their own URDF links

    A tool frame without inertial is allowed, and other articulated branches
    remain intact, with their source inertias held constant during regression
    """
    joint_entries = {j.get('name'): j for j in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    pin_ids = {str(model.names[jid]): jid for jid in range(1, model.njoints)}
    mapped = []
    for name in sampled_joint_names:
        jid = pin_ids.get(name)
        item = joint_entries.get(name)
        if jid is None or item is None or item.get('type') not in ('revolute', 'continuous', 'prismatic'):
            raise ValueError(f'采样关节 {name} 不存在或不是单自由度可动关节')
        joint = model.joints[jid]
        if int(joint.nq) not in (1, 2) or int(joint.nv) != 1:
            raise ValueError(f'采样关节 {name} 不是单自由度，无法按标定 CSV 映射')
        child = item.find('child')
        if child is None or child.get('link') not in links:
            raise ValueError(f'{name} 缺少子 Link')
        linkname = child.get('link')
        child_link = links[linkname]
        if child_link.find('inertial') is None:
            raise ValueError(f'可动 Link {linkname} 缺少 inertial，无法定义其质量参数')
        mapped.append((jid, linkname, child_link))

    names = {entry[1] for entry in mapped}
    nonwritten = [name for name, link in links.items()
                  if name not in names and link.find('inertial') is not None]
    # Fixed descendants with actual inertia are folded into a Pinocchio body
    # Their mass cannot be separated from its parent by motor torque alone
    fixed_children = {}
    for j in root.findall('joint'):
        if j.get('type') == 'fixed':
            p, c = j.find('parent'), j.find('child')
            if p is not None and c is not None:
                fixed_children.setdefault(p.get('link'), []).append(c.get('link'))
    for original in names:
        pending = list(fixed_children.get(original, []))
        seen = set()
        while pending:
            name = pending.pop()
            if name in seen:
                continue
            seen.add(name)
            child_link = links.get(name)
            if child_link is None:
                continue
            inertial = child_link.find('inertial')
            if inertial is not None:
                mass = inertial.find('mass')
                inertia = inertial.find('inertia')
                mass_value = float(mass.get('value', '0')) if mass is not None else 0.0
                tensor_values = ([float(v) for v in inertia.attrib.values()]
                                 if inertia is not None else [])
                if abs(mass_value) > 1e-12 or any(abs(v) > 1e-12 for v in tensor_values):
                    raise ValueError(f'{original} 上的固定附属 Link {name} 存在质量或惯量，无法将合并参数直接反写到单独 Link')
            pending.extend(fixed_children.get(name, []))
    return mapped, nonwritten


def _sampling_map(model, moving_joints):
    """Return Pinocchio scalar position/velocity coordinates for the CSV axes"""
    q_indices, v_indices, param_indices = [], [], []
    for jid, _name, _link in moving_joints:
        joint = model.joints[jid]
        if int(joint.nq) not in (1, 2) or int(joint.nv) != 1:
            raise ValueError(f'采样关节 {model.names[jid]} 无法映射到单自由度状态')
        q_indices.append(int(joint.idx_q))
        v_indices.append(int(joint.idx_v))
        param_indices.extend(range(10 * (jid - 1), 10 * jid))
    if len(set(q_indices)) != len(q_indices) or len(set(v_indices)) != len(v_indices):
        raise ValueError('采样关节状态索引不唯一')
    return q_indices, v_indices, param_indices



def _assign_configuration(model, target, indices, values):
    import numpy as np
    joints = {int(model.joints[j].idx_q): model.joints[j] for j in range(1, model.njoints)}
    for index, value in zip(indices, values):
        if int(joints[index].nq) == 2:
            target[index:index+2] = (np.cos(value), np.sin(value))
        else:
            target[index] = value
    return target

def _extract_rows(csvpath, joints, limit=500):
    from full_inertial import _extract_rows as validated_rows
    return validated_rows(csvpath, joints, limit)


def _fit_regressor(rows, model, pin, moving_joints, **options):
    from full_inertial import _fit_regressor as reduced_fit
    return reduced_fit(rows, model, pin, moving_joints, **options)


def _review_candidate(fit, model, pin, moving_joints=None):
    from full_inertial import _review_candidate as review
    return review(fit, model, pin, moving_joints)


def _dynamic_parameters_to_urdf_fields(parameters):
    """Represent the unconstrained 10D Pinocchio inertial vector in the link frame.

    The dynamic parameter layout is [m, m*c_x, m*c_y, m*c_z,
    Ixx_origin, Ixy_origin, Iyy_origin, Ixz_origin, Iyz_origin, Izz_origin].
    Only finite and nonzero mass is required for the COM to be mathematically
    defined; negative mass and non-SPD tensors are reported, never projected.
    """
    import numpy as np
    p=np.asarray(parameters,dtype=float).reshape(10)
    if not np.isfinite(p).all() or abs(float(p[0]))<1e-12:
        raise ValueError('数值解包含非有限或零质量，无法将一阶质量矩转换为 URDF 质心')
    mass=float(p[0])
    com=p[1:4]/mass
    inertia_origin=np.array([[p[4],p[5],p[7]],
                             [p[5],p[6],p[8]],
                             [p[7],p[8],p[9]]],dtype=float)
    inertia_com=inertia_origin-mass*((com@com)*np.eye(3)-np.outer(com,com))
    return mass, com, inertia_com


def export_full_inertial_candidate(directory, destination=None):
    import numpy as np
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise ValueError('完整惯性辨识需要 Python Pinocchio；此功能可选，不会自动安装请在 Launcher 使用的 Python 环境安装 pinocchio 后重试') from exc
    directory,metadata,result,snapshot,source=_prepare_inputs(directory)
    tree=ET.parse(snapshot);root=tree.getroot()
    from full_inertial import _apply_recorded_gravity
    model=_apply_recorded_gravity(pin.buildModelFromUrdf(str(snapshot)),metadata)
    joints=metadata.get('joint_names',[])
    if not isinstance(joints,list) or not joints or len(set(joints))!=len(joints):
        raise ValueError('任务记录中缺少有效的关节采样顺序')
    mapped,unchanged=_articulated_mapping(root,model,joints)
    if len(mapped)!=len(joints):
        raise ValueError('采样关节映射不完整')
    frames=_extract_rows(directory/'frames.csv',joints)
    fit=_fit_regressor(frames,model,pin,mapped)
    (theta,rms,alpha),baseline,review_warnings=_review_candidate(fit,model,pin,mapped)
    unmeasured=[str(model.names[j]) for j in range(1,model.njoints)
                if j not in {item[0] for item in mapped}]
    if unmeasured:
        review_warnings.append('未采样的附属关节 ' + ', '.join(unmeasured)
                               + ' 按 URDF 中立位姿处理并保持其原有惯量，若采集期间这些关节运动，拟合不具备可靠性')
    for k,(jid,linkname,link) in enumerate(mapped):
        mass_value, com, mat = _dynamic_parameters_to_urdf_fields(theta[k*10:(k+1)*10])
        inertial=link.find('inertial')
        origin=inertial.find('origin')
        if origin is None: origin=ET.SubElement(inertial,'origin')
        origin.attrib.update(xyz=' '.join(f'{v:.12g}' for v in com),rpy='0 0 0')
        mass=inertial.find('mass')
        mass.attrib['value']=f'{mass_value:.12g}'
        tensor=inertial.find('inertia')
        if tensor is None: tensor=ET.SubElement(inertial,'inertia')
        for name,value in (('ixx',mat[0,0]),('ixy',mat[0,1]),('ixz',mat[0,2]),('iyy',mat[1,1]),('iyz',mat[1,2]),('izz',mat[2,2])):
            tensor.attrib[name]=f'{value:.12g}'
    output=Path(destination).expanduser().resolve() if destination else directory/'candidate_full_inertial'
    output.mkdir(parents=True,exist_ok=True)
    # Copy relative visual/collision meshes when available alongside either
    # the task snapshot or the original model directory, even if the URDF has moved
    from model_calibration_source import resolve_relative_mesh
    for index, mesh in enumerate(root.findall('.//mesh')):
        filename=mesh.get('filename','')
        if not filename or filename.startswith('package://') or Path(filename).is_absolute():
            continue
        src=resolve_relative_mesh(filename,snapshot,metadata)
        if src is None:
            raise ValueError(f'候选资源文件缺失: {filename}')
        resources=output/'resources';resources.mkdir(parents=True,exist_ok=True)
        dest=resources/f'{index:03d}-{src.name}'
        shutil.copy2(src,dest)
        mesh.set('filename',dest.relative_to(output).as_posix())
    dst=output/'candidate-inertial.urdf';tmp=output/'candidate-inertial.urdf.tmp'
    tree.write(tmp,encoding='utf-8',xml_declaration=True)
    # Parsing and roundtrip checking are diagnostics only; keep the raw
    # unconstrained result for human review even if the dynamics engine
    # considers its physical inertias invalid.
    try:
        parsed=pin.buildModelFromUrdf(str(tmp))
        if parsed.nv!=model.nv:
            review_warnings.append('候选 URDF 回读自由度与源模型不符')
        else:
            for k,(jid, _, _) in enumerate(mapped):
                expected=theta[k*10:(k+1)*10]
                actual=np.asarray(parsed.inertias[jid].toDynamicParameters())
                if np.linalg.norm(expected-actual)>1e-7*max(1.0,np.linalg.norm(expected)):
                    review_warnings.append(f'候选惯量回读误差：{model.names[jid]}')
    except Exception as exc:
        review_warnings.append(f'候选惯量未通过 Pinocchio 重新载入检查：{exc}')
    tmp.replace(dst)
    report={
      'solver':'dynamic_reduced_svd_irls',
      'acceleration_input':'recorded_acceleration_no_reconstruction',
      'runtime_provenance':metadata.get('runtime_provenance'),
      'frames_sha256':hashlib.sha256((directory/'frames.csv').read_bytes()).hexdigest(),
      'solver_source_sha256':hashlib.sha256(Path(__file__).with_name('full_inertial.py').read_bytes()).hexdigest(),
      'candidate_urdf':str(dst),'status':'unconstrained_candidate_requires_human_review',
      'verified_joint_inertials':[name for _,name,_ in mapped],
      'unchanged_inertials':unchanged,
      'unmeasured_joints_assumed_neutral':unmeasured,
      'joint_regressor_rank':fit['rank'],'joint_regressor_columns':fit['degrees'],
      'all_parameters_identifiable':fit['rank']==fit['degrees'] and not unchanged and not unmeasured,
      'validation_original_rms_nm':baseline,'validation_candidate_rms_nm':rms,
      'prior_to_fit_step':alpha,'frames_used':len(frames),
      'inertial_parameters_raw':theta.tolist(),
      'constraints':[], 'review_warnings':review_warnings,
      'hard_com_or_mass_bounds_applied':False,
      'limitations':'Dynamic regressor with motor torque data; friction and motor drive effects are not fully isolated. Do not deploy without independent experiments and physical review.',
      'source_snapshot':str(snapshot),
    }
    (output/'inertial-identification-report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    return report
