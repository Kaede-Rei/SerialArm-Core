"""Offline, opt-in full-inertial URDF candidate from completed model calibration.

Never modifies the source URDF or running robot. Rank-deficient fits are marked
prior-constrained, not fully identified. Raw motor torques need verified signs,
no external contacts, and accurate joint accelerations to be trusted.
"""
from __future__ import annotations

import csv
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


def _articulated_mapping(root, model):
    """Only 1:1 revolute joints may be written; aggregated fixed bodies are unsafe."""
    joint_entries = {j.get('name'): j for j in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    mapped = []
    nonwritten = []
    for jid in range(1, model.njoints):
        jname = str(model.names[jid])
        item = joint_entries.get(jname)
        if item is None or item.get('type') not in ('revolute', 'continuous', 'prismatic'):
            raise ValueError(f'无法将 Pinocchio 关节 {jname} 唯一映射到 URDF 可动 Link')
        child = item.find('child')
        if child is None or child.get('link') not in links:
            raise ValueError(f'{jname} 缺少子 Link')
        linkname = child.get('link')
        child_link = links[linkname]
        if child_link.find('inertial') is None:
            raise ValueError(f'可动 Link {linkname} 缺少 inertial')
        mapped.append((jid, linkname, child_link))
    names = {entry[1] for entry in mapped}
    # Pinocchio aggregates fixed-body inertias: cannot unmix individual original
    # URDF inertia values unless they have their own independent measurements.
    for name, link in links.items():
        if name not in names and link.find('inertial') is not None:
            nonwritten.append(name)
    # Reject an articulated link with nonzero-mass fixed descendants; its
    # Pinocchio inertia is an aggregate and must not be copied into the child.
    for j in root.findall('joint'):
        if j.get('type') == 'fixed':
            p = j.find('parent')
            c = j.find('child')
            if p is not None and c is not None and p.get('link') in names:
                childlink = links.get(c.get('link'))
                if childlink is not None and childlink.find('inertial') is not None:
                    raise ValueError('有带惯量的固定附属 Link；Pinocchio 合并了刚体，不能安全地逐 Link 反写全部惯量')
    return mapped, nonwritten


def _extract_rows(csvpath, joints, limit=500):
    import numpy as np
    valid=[]
    with open(csvpath, newline='') as inp:
        reader=csv.DictReader(inp)
        required = [f'{k}:{j}' for k in ('q','dq','acc','tau') for j in joints]
        missing = [k for k in required if k not in (reader.fieldnames or [])]
        if missing:
            raise ValueError('标定帧缺少列'+', '.join(missing))
        for row in reader:
            # Drag demonstration is not a closed-loop motor model identification dataset.
            phase=row.get('phase', '')
            if phase not in ('static_reverse','friction_forward_fast') or row.get('valid') != '1':
                continue
            if float(row.get('feedback_age_ms') or 999) > 30:
                continue
            try:
                q=np.array([float(row[f'q:{j}']) for j in joints]);
                v=np.array([float(row[f'dq:{j}']) for j in joints]);
                a=np.array([float(row[f'acc:{j}']) for j in joints]);
                tau=np.array([float(row[f'tau:{j}']) for j in joints]);
            except (TypeError,ValueError):
                continue
            if not all(np.isfinite(x).all() for x in (q,v,a,tau)) or np.max(np.abs(v)) < 0.06:
                continue
            valid.append((phase,q,v,a,tau))
    if len(valid) < 120:
        raise ValueError(f'有效自动运动帧不足（{len(valid)}）；需要更多双向激励，不能从手动拖拽帧估计惯量')
    if set(item[0] for item in valid) != {'static_reverse','friction_forward_fast'}:
        raise ValueError('缺少反向或正向回放数据，无法做双向辨识')
    if len(valid) > limit:
        stride = len(valid) / limit
        valid = [valid[min(len(valid)-1,int(i*stride))] for i in range(limit)]
    return valid


def _fit_regressor(rows, model, pin, moving_joints, *, regularization=0.15):
    import numpy as np
    data=model.createData()
    nj=len(moving_joints)
    nparam=10*(model.njoints-1)
    prior=np.concatenate([np.asarray(model.inertias[j].toDynamicParameters(), dtype=float) for j in range(1,model.njoints)])
    X=[]; y=[]; labels=[]
    for phase,q,v,a,tau in rows:
        Y=np.asarray(pin.computeJointTorqueRegressor(model,data,q,v,a),dtype=float)
        if Y.shape!=(model.nv,nparam):
            raise ValueError('Pinocchio 力矩回归矩阵维度不匹配')
        # Per-axis asymmetric Coulomb + viscous friction + constant torque bias
        # as nuisance variables, estimated jointly to prevent inertia absorbing friction.
        F=np.zeros((nj,5*nj))
        for k in range(nj):
            vel=v[k]
            F[k,5*k]=1
            F[k,5*k+1]=float(vel>0.05)
            F[k,5*k+2]=max(vel,0.0)
            F[k,5*k+3]=float(vel< -0.05)
            F[k,5*k+4]=min(vel,0.0)
        X.append(np.concatenate((Y,F),axis=1));y.append(tau);labels.append(phase)
    X=np.concatenate(X);y=np.concatenate(y)
    nper=nj
    # contiguous time-block holdout within each direction
    # keeps calibration/train runs distinct in time, rather than random nearby points.
    phase_counts={p:[i for i,l in enumerate(labels) if l==p] for p in set(labels)}
    hold=set()
    for phase,indices in phase_counts.items():
        hold.update(indices[int(0.8*len(indices)):])
    train_indices=np.concatenate([np.arange(i*nj,(i+1)*nj) for i in range(len(labels)) if i not in hold])
    val_indices=np.concatenate([np.arange(i*nj,(i+1)*nj) for i in range(len(labels)) if i in hold])
    A=X[train_indices];b=y[train_indices]
    V=X[val_indices];t=y[val_indices]
    # Normalize columns before ridge (prevents SI unit scales from changing regularization).
    colnorm=np.linalg.norm(A,axis=0)
    scale=np.maximum(colnorm, 1e-10)
    # Report inertia rank AFTER removing the nuisance friction/bias subspace.
    # Otherwise friction/excitation correlations overstate identifiability.
    dynamic=A[:,:nparam]
    friction=A[:,nparam:]
    cleaned=dynamic-friction@np.linalg.lstsq(friction,dynamic,rcond=1e-9)[0]
    clean_scale=np.maximum(np.linalg.norm(cleaned,axis=0),1e-10)
    rank=int(np.linalg.matrix_rank(cleaned/clean_scale,tol=1e-3))
    nuisance=np.zeros(5*nj)
    # Rigid dynamics source prior; deviations are regularized, nuisance unregularized
    xprior=np.concatenate((prior,nuisance))
    A2=A/scale
    reg=np.ones(A.shape[1]) * regularization
    reg[nparam:]=regularization*0.1
    rhs=b-A@xprior
    # Robust reweighted ridge: torque spikes must not dominate the candidate.
    # This is a soft loss (no mass, COM or holdout acceptance bound).
    z=np.zeros(len(reg))
    for _ in range(5):
        residual=rhs-A2@z
        mad=float(np.median(np.abs(residual-np.median(residual))))
        threshold=max(0.05,1.345*1.4826*mad)
        weight=np.minimum(1.0,threshold/np.maximum(np.abs(residual),threshold))
        root=np.sqrt(weight)
        z=np.linalg.lstsq(np.vstack((A2*root[:,None],np.diag(reg))),
                          np.concatenate((rhs*root,np.zeros(len(reg)))),rcond=1e-9)[0]
    delta=z/scale
    return {
        'prior':prior,'candidate':prior+delta[:nparam], 'nuisance':delta[nparam:],
        'rank':rank,'degrees':nparam,'train':(A,b), 'validation':(V,t),
        'X':X,'full_prior':xprior,
    }


def _review_candidate(fit, model, pin):
    """Return the numerical optimum unchanged; never gate it on a CAD delta or holdout score."""
    import numpy as np
    prior=np.asarray(fit['prior'],dtype=float)
    target=np.asarray(fit['candidate'],dtype=float)
    if not np.isfinite(target).all():
        raise ValueError('惯量回归结果包含非有限数值，无法生成有意义的候选')
    V,t=fit['validation']
    nuisance=np.asarray(fit['nuisance'],dtype=float)
    baseline=float(np.sqrt(np.mean((t-V@np.concatenate((prior,nuisance)))**2)))
    candidate_rms=float(np.sqrt(np.mean((t-V@np.concatenate((target,nuisance)))**2)))
    warnings=[]
    for j in range(1,model.njoints):
        name=str(model.names[j]);old=model.inertias[j]
        sl=target[10*(j-1):10*j]
        try:
            new=pin.Inertia.FromDynamicParameters(sl)
            if not physical_inertia(float(new.mass),np.asarray(new.lever),np.asarray(new.inertia)):
                warnings.append(f'{name}: 候选质量/质心/惯性张量不满足物理一致性（仅供审核）')
            if float(old.mass)>0:
                delta=abs(float(new.mass)-float(old.mass))/float(old.mass)
                if delta>0.3:
                    warnings.append(f'{name}: 相对原模型质量变化 {delta:.1%}（仅提示，不约束）')
            com_shift=float(np.linalg.norm(np.asarray(new.lever)-np.asarray(old.lever)))
            if com_shift>0.05:
                warnings.append(f'{name}: 相对原模型质心偏移 {com_shift:.3f} m（仅提示，不约束）')
        except Exception as exc:
            warnings.append(f'{name}: 反解惯性参数异常：{exc}（仅供审核）')
    if not np.isfinite(candidate_rms) or not np.isfinite(baseline):
        raise ValueError('候选验证误差非有限数值，无法生成有意义的候选')
    if candidate_rms>=baseline:
        warnings.append(f'留出集 RMS 从 {baseline:.4f} 增加至 {candidate_rms:.4f} Nm（仅提示，不阻止导出）')
    if fit['rank']<fit['degrees']:
        warnings.append(f'惯量回归矩阵仅秩 {fit["rank"]}/{fit["degrees"]}，未辨识方向仍依赖 CAD 先验')
    return (target,candidate_rms,1.0),baseline,warnings


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
    model=pin.buildModelFromUrdf(str(snapshot))
    joints=metadata.get('joint_names',[])
    if list(map(str,model.names[1:])) != joints or model.nq!=len(joints) or model.nv!=len(joints):
        raise ValueError('URDF 约简、闭环、固定附属关节或顺序与采样配置不一致；拒绝反写惯量')
    mapped,unchanged=_articulated_mapping(root,model)
    frames=_extract_rows(directory/'frames.csv',joints)
    fit=_fit_regressor(frames,model,pin,mapped)
    (theta,rms,alpha),baseline,review_warnings=_review_candidate(fit,model,pin)
    for jid,linkname,link in mapped:
        mass_value, com, mat = _dynamic_parameters_to_urdf_fields(theta[(jid-1)*10:jid*10])
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
            for jid, _, _ in mapped:
                expected=theta[(jid-1)*10:jid*10]
                actual=np.asarray(parsed.inertias[jid].toDynamicParameters())
                if np.linalg.norm(expected-actual)>1e-7*max(1.0,np.linalg.norm(expected)):
                    review_warnings.append(f'候选惯量回读误差：{model.names[jid]}')
    except Exception as exc:
        review_warnings.append(f'候选惯量未通过 Pinocchio 重新载入检查：{exc}')
    tmp.replace(dst)
    report={
      'candidate_urdf':str(dst),'status':'unconstrained_candidate_requires_human_review',
      'verified_joint_inertials':[name for _,name,_ in mapped],
      'unchanged_inertials':unchanged,
      'joint_regressor_rank':fit['rank'],'joint_regressor_columns':fit['degrees'],
      'all_parameters_identifiable':fit['rank']==fit['degrees'] and not unchanged,
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
