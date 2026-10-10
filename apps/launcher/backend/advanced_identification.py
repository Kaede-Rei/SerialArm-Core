"""Offline supplementary identification, scoped to a fingerprint-verified task snapshot

All results are review-only. Nothing here changes robot runtime parameters
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
import numpy as np

from full_inertial import (
    _prepare_inputs, _articulated_mapping, _sampling_map, _extract_rows,
    _fit_regressor, _dynamic_parameters_to_urdf_fields, physical_inertia,
)

MODES = ('mass', 'com', 'mass_com', 'inertia', 'full')
PARAMETERS = {
    'mass': (0,), 'com': (1, 2, 3), 'mass_com': (0, 1, 2, 3),
    'inertia': (4, 5, 6, 7, 8, 9), 'full': tuple(range(10)),
}


def assess_record(directory):
    """Low-cost data inventory, no native extension and no hardware action"""
    path = Path(directory).expanduser().resolve()
    metadata = json.loads((path / 'metadata.json').read_text())
    if not (path / 'frames.csv').is_file():
        raise ValueError('标定任务缺少 frames.csv')
    phases = {}
    groups = set()
    valid_dynamic = 0
    peak_vel = 0.0
    peak_acc = 0.0
    names = metadata.get('joint_names', [])
    with (path / 'frames.csv').open(newline='') as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            phase = row.get('phase', '')
            if row.get('valid') != '1':
                continue
            phases[phase] = phases.get(phase, 0) + 1
            if phase.startswith('static_') and row.get('pose_group', '0') not in ('0', ''):
                groups.add((row['pose_group'], row.get('validation', '0')))
            if phase in ('static_reverse', 'friction_forward_fast'):
                try:
                    velocities = [abs(float(row['dq:' + name])) for name in names]
                    accels = [abs(float(row['acc:' + name])) for name in names]
                    peak_vel = max(peak_vel, *velocities)
                    peak_acc = max(peak_acc, *accels)
                    if max(velocities) >= 0.06 and float(row.get('feedback_age_ms', '999')) <= 30:
                        valid_dynamic += 1
                except (KeyError, ValueError):
                    continue
    training = sum(validation == '0' for _, validation in groups)
    holdout = sum(validation == '1' for _, validation in groups)
    has_two_phases = bool(phases.get('static_reverse')) and bool(phases.get('friction_forward_fast'))
    return {
        'directory': str(path), 'task_id': metadata.get('task_id', ''),
        'static_groups': len(groups), 'training_groups': training, 'holdout_groups': holdout,
        'valid_dynamic_samples': valid_dynamic, 'peak_joint_speed_rad_s': peak_vel,
        'peak_joint_acceleration_rad_s2': peak_acc, 'phase_samples': phases,
        'gravity_ready': training >= 4 and holdout >= 1,
        'dynamic_precheck': valid_dynamic >= 120 and has_two_phases,
        'limitations': [
            '有效动态帧数和速度幅度仅供预筛，不能替代动力学回归矩阵可辨识性检查',
            '电机反馈力矩不是独立的关节力矩真值，加速度由现有数据源估计',
            '未采样的末端可动关节默认停留在 URDF 中立位置，若实际有运动则需要补充测量',
        ],
    }


def _pose_groups(directory, joint_names):
    """Retain only stable tail samples of recorded holds, grouped by complete pose"""
    groups = {}
    with (Path(directory) / 'frames.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('phase') not in ('static_reverse', 'static_forward'):
                continue
            if row.get('pose_group') in (None, '', '0') or row.get('valid') != '1':
                continue
            try:
                if float(row.get('feedback_age_ms') or 999) > 30:
                    continue
                q = np.array([float(row[f'q:{name}']) for name in joint_names])
                dq = np.array([float(row[f'dq:{name}']) for name in joint_names])
                tau = np.array([float(row[f'tau:{name}']) for name in joint_names])
                time = int(row['monotonic_ns'])
                if not all(np.isfinite(a).all() for a in (q, dq, tau)) or np.max(np.abs(dq)) > 0.035:
                    continue
            except (ValueError, KeyError, TypeError):
                continue
            groups.setdefault((int(row['pose_group']), row.get('validation') == '1'), []).append((time, q, tau))
    poses = []
    for (group, holdout), samples in sorted(groups.items()):
        newest = max(item[0] for item in samples)
        stable = [item for item in samples if item[0] >= newest - 900_000_000]
        if len(stable) < 8:
            continue
        q = np.median([item[1] for item in stable], axis=0)
        tau = np.median([item[2] for item in stable], axis=0)
        poses.append((group, holdout, q, tau, len(stable)))
    return poses


def _fit_static(directory, model, pin, mapped, mode, locked_links):
    joints = [str(model.names[jid]) for jid, _, _ in mapped]
    poses = _pose_groups(directory, joints)
    training = [p for p in poses if not p[1]]
    validation = [p for p in poses if p[1]]
    if len(training) < 4 or not validation:
        raise ValueError('静态训练或留出姿态不足，不能独立验证质量和质心参数')
    q_idx, v_idx, all_ids = _sampling_map(model, mapped)
    all_prior = np.concatenate([np.asarray(model.inertias[j].toDynamicParameters(), dtype=float) for j in range(1, model.njoints)])
    selected = []
    per_link = []
    for k, (jid, name, _) in enumerate(mapped):
        enabled = name not in locked_links
        per_link.append(enabled)
        if enabled:
            selected.extend(all_ids[10*k+i] for i in PARAMETERS[mode])
    if not selected:
        raise ValueError('没有可辨识的未锁定 Link')
    base = np.asarray(pin.neutral(model), dtype=float)
    nv = int(model.nv)
    data = model.createData()
    values = []
    for group, holdout, q, torque, nsamples in poses:
        qq = base.copy()
        qq[q_idx] = q
        full_y = np.asarray(pin.computeJointTorqueRegressor(model, data, qq, np.zeros(nv), np.zeros(nv)))
        prediction = full_y[v_idx] @ all_prior
        design = full_y[np.ix_(v_idx, selected)]
        if mode == 'mass':
            # With fixed center of mass, h=m*c changes when mass changes
            for column, pindex in enumerate(selected):
                link_index = pindex // 10
                mass = all_prior[10 * link_index]
                if mass <= 1e-9:
                    raise ValueError('质量先验无效，无法固定该 Link 的质心')
                com = all_prior[10*link_index+1:10*link_index+4]/mass
                design[:,column] += full_y[np.ix_(v_idx, list(range(pindex+1,pindex+4)))] @ com
        values.append((holdout, design, torque - prediction, group))
    train = [v for v in values if not v[0]]
    hold = [v for v in values if v[0]]
    n = len(joints)
    # Estimate a shared per-joint bias only from training groups
    Y = np.vstack([v[1] for v in train])
    r = np.concatenate([v[2] for v in train])
    bias_design = np.tile(np.eye(n), (len(train), 1))
    # Centering prevents torque bias from being absorbed by mass / first moments
    P = np.eye(len(r)) - bias_design @ np.linalg.pinv(bias_design)
    centered = P @ Y
    scale = np.maximum(np.linalg.norm(centered, axis=0), 1e-9)
    spectrum = np.linalg.svd(centered / scale, compute_uv=False)
    rank = int(np.sum(spectrum > max(1e-3 * (spectrum[0] if len(spectrum) else 0), 1e-9)))
    if rank < 1:
        raise ValueError('当前静态姿态不包含足够的可观测信息，保留原 URDF')
    # Standardized ridge and group-wise CV; all choices use training poses only
    lambdas = (0.1, 1, 10, 100, 1000)
    def solve(blocks, regularization):
        yy = np.vstack([v[1] for v in blocks]); rr = np.concatenate([v[2] for v in blocks]);
        bb = np.tile(np.eye(n), (len(blocks), 1))
        proj = np.eye(len(rr)) - bb @ np.linalg.pinv(bb)
        a = proj @ yy / scale
        rhs = proj @ rr
        standardized = np.linalg.lstsq(np.vstack((a, np.sqrt(regularization) * np.eye(len(selected)))),
                                         np.concatenate((rhs, np.zeros(len(selected)))), rcond=1e-8)[0]
        delta = standardized / scale
        bias = np.mean([v[2] - v[1] @ delta for v in blocks], axis=0)
        return delta, bias
    def error(blocks, delta, bias):
        residual = np.concatenate([v[2] - v[1] @ delta - bias for v in blocks])
        return float(np.sqrt(np.mean(residual ** 2)))
    scores=[]
    for lam in lambdas:
        errors=[]
        for i in range(len(train)):
            subset=train[:i]+train[i+1:]
            delta, b = solve(subset, lam)
            errors.append(error([train[i]], delta, b))
        scores.append(float(np.mean(errors)))
    regularization = lambdas[int(np.argmin(scores))]
    delta, bias = solve(train, regularization)
    # Reconstruct and preserve center inertia as prior, while mass / COM is updated
    target = all_prior.copy()
    target[selected] += delta
    if mode == 'mass':
        for column,pindex in enumerate(selected):
            com = all_prior[pindex+1:pindex+4]/all_prior[pindex]
            target[pindex+1:pindex+4] += com*delta[column]
    warnings=[]
    alpha=1.0
    for _ in range(24):
        valid=True
        for jid, name, _ in mapped:
            if name in locked_links:
                continue
            p = target[10*(jid-1):10*jid] if alpha == 1 else all_prior[10*(jid-1):10*jid] + alpha*(target[10*(jid-1):10*jid]-all_prior[10*(jid-1):10*jid])
            if p[0] <= 1e-7 or not np.isfinite(p).all():
                valid=False
        if valid:
            break
        alpha*=0.5
    else:
        alpha=0
    if alpha < 1:
        warnings.append('拟合结果包含非正质量，已沿原始模型方向收缩至物理可行范围')
    delta *= alpha
    full_delta = np.zeros_like(all_prior)
    full_delta[selected] = delta
    if mode == 'mass':
        for column,pindex in enumerate(selected):
            full_delta[pindex+1:pindex+4] += (all_prior[pindex+1:pindex+4]/all_prior[pindex])*delta[column]
    original_bias = np.mean([v[2] for v in train], axis=0)
    baseline = error(hold, np.zeros_like(delta), original_bias)
    candidate = error(hold, delta, bias)
    # Favor original in absence of independent improvement
    passed = candidate < baseline - max(0.001, baseline*0.01)
    if rank < len(selected):
        warnings.append(f'静态回归仅辨识出 {rank}/{len(selected)} 个独立方向，其他参数依赖 CAD 先验')
    if not passed:
        warnings.append('留出姿态未证明质量或质心候选优于原始模型')
    return {'mode': mode, 'prior': all_prior, 'indices': selected, 'delta': delta, 'full_delta': full_delta,
            'rank': rank, 'columns': len(selected), 'regularization': regularization,
            'baseline': baseline, 'candidate_rms': candidate, 'passed': passed,
            'warnings': warnings, 'train_groups': len(train), 'holdout_groups': len(hold),
            'locked_links': sorted(locked_links), 'bias': bias.tolist()}


def _write_static_candidate(root, mapped, fit):
    # Import postponed until after a successful fit
    import xml.etree.ElementTree as ET
    updated = fit['prior'].copy()
    updated += fit['full_delta']
    for jid, name, link in mapped:
        if name in fit['locked_links']:
            continue
        p = updated[10*(jid-1):10*jid]
        m=float(p[0]); c=p[1:4]/m
        inertial=link.find('inertial')
        if inertial is None:
            raise ValueError(f'{name} 缺少 inertial')
        origin=inertial.find('origin')
        if origin is None:
            origin=ET.SubElement(inertial, 'origin')
        origin.set('xyz',' '.join(f'{x:.12g}' for x in c))
        mass=inertial.find('mass')
        mass.set('value',f'{m:.12g}')
    return root



def _physical_urdf_links(root):
    """Return nonphysical inertial bodies without changing the user's original XML

    Inertia in URDF is expressed around its own inertial origin, so the
    positive-definite and principal-moment checks apply without shifting it
    into the joint frame
    """
    invalid = []
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue  # A massless tool/frame is not an inertial body
        try:
            mass = float(inertial.find('mass').get('value'))
            inertia = inertial.find('inertia')
            c = inertial.find('origin')
            com = np.array([float(x) for x in (c.get('xyz', '0 0 0') if c is not None else '0 0 0').split()])
            tensor = np.array([
                [float(inertia.get('ixx')), float(inertia.get('ixy')), float(inertia.get('ixz'))],
                [float(inertia.get('ixy')), float(inertia.get('iyy')), float(inertia.get('iyz'))],
                [float(inertia.get('ixz')), float(inertia.get('iyz')), float(inertia.get('izz'))],
            ])
            if len(com) != 3 or not physical_inertia(mass, com, tensor):
                invalid.append(link.get('name') or '<unnamed>')
        except (AttributeError, TypeError, ValueError):
            invalid.append(link.get('name') or '<unnamed>')
    return invalid


def _physical_dynamic_candidate(prior, target, mapped, locked, active_parameters):
    """Constrain dynamic inertias while preserving every fixed source field

    The 4x4 pseudo-inertia is positive definite exactly when rigid-body
    mass, center inertia and principal-moment inequalities are physical
    SLSQP runs per movable link only; a failed fit falls back to the
    original physical prior, never a guessed nonphysical URDF
    """
    from full_inertial import _dynamic_parameters_to_urdf_fields
    candidate = prior.copy()
    details = []
    try:
        from scipy.optimize import minimize
    except ImportError:
        minimize = None
    for k, (_jid, name, _link) in enumerate(mapped):
        if name in locked:
            continue
        offset = 10 * k
        old = prior[offset:offset+10]
        proposed = target[offset:offset+10]
        active = np.array(active_parameters, dtype=int)
        if np.array_equal(proposed[active], old[active]):
            continue

        def is_physical(p):
            try:
                m, c, I = _dynamic_parameters_to_urdf_fields(p)
                return physical_inertia(m, c, I)
            except (ValueError, ZeroDivisionError, FloatingPointError):
                return False

        def pseudo_eigenvalues(values):
            p = old.copy()
            p[active] = values
            I = np.array([[p[4], p[5], p[7]],
                          [p[5], p[6], p[8]],
                          [p[7], p[8], p[9]]])
            sigma = 0.5 * np.trace(I) * np.eye(3) - I
            pseudo = np.zeros((4, 4))
            pseudo[:3, :3] = sigma
            pseudo[:3, 3] = p[1:4]
            pseudo[3, :3] = p[1:4]
            pseudo[3, 3] = p[0]
            return np.linalg.eigvalsh(pseudo)

        # Prior-centered scaling avoids treating mass and small inertial
        # cross-terms as if they had the same numeric magnitude
        weights = np.maximum(np.abs(old[active]), np.array(
            [0.02 if i == 0 else 0.003 if i < 4 else 0.0003
             for i in active]))
        trial = old.copy()
        solved = False
        if minimize is not None and is_physical(old):
            def objective(x):
                target_error = (x - proposed[active]) / weights
                prior_error = (x - old[active]) / weights
                return float(target_error @ target_error + 0.12 * (prior_error @ prior_error))
            bounds = [(1e-7, None) if i == 0 else (None, None) for i in active]
            try:
                result = minimize(
                    objective, old[active], method='SLSQP', bounds=bounds,
                    constraints=[{'type': 'ineq',
                                  'fun': lambda x: pseudo_eigenvalues(x) - 1e-9}],
                    options={'maxiter': 160, 'ftol': 1e-9, 'disp': False},
                )
                trial[active] = result.x
                solved = is_physical(trial) and np.isfinite(trial).all()
            except (ValueError, FloatingPointError):
                solved = False
        if not solved:
            # Conservative numerical fallback works without scipy or when
            # optimization is inconclusive, and keeps the prior as a valid exit
            for power in range(0, 25):
                alpha = 2.0 ** (-power)
                trial = old.copy()
                trial[active] += alpha * (proposed[active] - old[active])
                if is_physical(trial):
                    solved = True
                    break
        if not solved:
            trial = old.copy()
        candidate[offset:offset+10] = trial
        if not np.allclose(trial[active], proposed[active], rtol=1e-6, atol=1e-9):
            details.append(f'{name} 的惯性候选已受物理约束调整，需结合留出误差审核')
        if np.array_equal(trial, old):
            details.append(f'{name} 的拟合无法生成物理可行候选，已保持原始参数')
    return candidate, details


def _validate_dynamic_candidate(fit, candidate):
    """Compare source and candidate after fitting their own training-only friction"""
    A, b = fit['train']
    V, holdout = fit['validation']
    n = len(fit['prior'])
    F = A[:, n:]
    FV = V[:, n:]
    def score(parameters):
        train_residual = b - A[:, :n] @ parameters
        # Same fitting procedure for source and candidate, never reuse the
        # candidate's friction estimate as the baseline
        if F.shape[1]:
            ridge = max(1e-8, float(np.linalg.norm(F, ord='fro')) * 1e-5)
            augmented = np.vstack((F, ridge * np.eye(F.shape[1])))
            friction = np.linalg.lstsq(augmented, np.r_[train_residual, np.zeros(F.shape[1])], rcond=1e-9)[0]
        else:
            friction = np.zeros(0)
        residual = holdout - V[:, :n] @ parameters - FV @ friction
        return float(np.sqrt(np.mean(residual ** 2)))
    return score(fit['prior']), score(candidate)


def run_advanced(directory, mode, destination=None, locked_links=None):
    if mode not in MODES:
        raise ValueError('unknown advanced identification target')
    import pinocchio as pin
    import xml.etree.ElementTree as ET
    from full_inertial import _review_candidate, _extract_rows
    from model_calibration_source import resolve_relative_mesh
    import shutil
    folder, metadata, result, snapshot, _source = _prepare_inputs(directory)
    root=ET.parse(snapshot).getroot()
    model=pin.buildModelFromUrdf(str(snapshot))
    joints=metadata.get('joint_names',[])
    if not isinstance(joints, list) or not joints:
        raise ValueError('缺少采样关节名称')
    mapped, unchanged=_articulated_mapping(root,model,joints)
    names={item[1] for item in mapped}
    locked=set(locked_links or [])
    if not locked.issubset(names):
        raise ValueError('锁定列表存在不属于可辨识模型的 Link')
    if locked==names:
        raise ValueError('所有 Link 均已锁定，无参数可辨识')
    warnings=[]
    if mode in ('mass','com','mass_com'):
        fit=_fit_static(folder,model,pin,mapped,mode,locked)
        warnings.extend(fit['warnings'])
        rank=fit['rank']; columns=fit['columns']; original=fit['baseline']; candidate=fit['candidate_rms'];passed=fit['passed'] and rank==columns
        _write_static_candidate(root,mapped,fit)
        extra={'training_groups':fit['train_groups'],'holdout_groups':fit['holdout_groups'],
               'cross_validation_lambda':fit['regularization']}
    else:
        rows=_extract_rows(folder/'frames.csv', joints)
        # Explicitly solve only the chosen subspace; locked parameters stay as priors
        all_fit=_fit_regressor(rows, model, pin, mapped, active_parameters=PARAMETERS[mode], locked_links=locked)
        target=np.asarray(all_fit['candidate']);prior=np.asarray(all_fit['prior'])
        trial, physical_notes = _physical_dynamic_candidate(prior, target, mapped, locked, PARAMETERS[mode])
        warnings.extend(physical_notes)
        original, candidate = _validate_dynamic_candidate(all_fit, trial)
        rank=all_fit['rank'];columns=all_fit['degrees']
        physical_ok = True
        for k, (_,name,_) in enumerate(mapped):
            if name in locked: continue
            try:
                mass, com, tensor = _dynamic_parameters_to_urdf_fields(trial[10*k:10*k+10])
                physical_ok &= physical_inertia(mass,com,tensor)
            except ValueError:
                physical_ok = False
        changes = float(np.linalg.norm(trial-prior))
        passed=(rank >= columns and physical_ok and changes > 1e-10
                and np.isfinite(candidate) and candidate < original-max(0.001,original*0.01))
        if rank<columns: warnings.append(f'动态回归仅有 {rank}/{columns} 个可辨识方向，其余依赖原始模型先验')
        if not physical_ok: warnings.append('候选惯性参数不满足物理一致性，不允许通过审核')
        if not passed: warnings.append('动态数据可辨识性、物理约束或留出改进未通过，候选仅供审核')
        for k, (_,name, link) in enumerate(mapped):
            if name in locked: continue
            mass,com,tensor=_dynamic_parameters_to_urdf_fields(trial[10*k:10*k+10])
            iner=link.find('inertial');orig=iner.find('origin')
            if orig is not None and any(abs(float(v)) > 1e-10 for v in orig.get('rpy','0 0 0').split()):
                raise ValueError(f'{name} 的惯性坐标系存在旋转，请先统一惯性表示后再导出候选')
            if orig is None: orig=ET.SubElement(iner,'origin')
            orig.set('xyz',' '.join(f'{v:.12g}' for v in com));orig.set('rpy','0 0 0')
            iner.find('mass').set('value',f'{mass:.12g}')
            T=iner.find('inertia')
            for key,val in [('ixx',tensor[0,0]),('ixy',tensor[0,1]),('ixz',tensor[0,2]),('iyy',tensor[1,1]),('iyz',tensor[1,2]),('izz',tensor[2,2])]:
                T.set(key,f'{val:.12g}')
        extra={'dynamic_frames':len(rows),'observable_directions':rank,'unobservable_directions':columns-rank,'friction_validation':'independent_training_fit'}
    tree=ET.ElementTree(root)
    # No source replacement, always write to an isolated review directory
    from uuid import uuid4
    dest=Path(destination).expanduser().resolve() if destination else folder/'advanced_candidates'/mode/uuid4().hex[:12]
    if dest == folder or dest == snapshot.parent or dest == snapshot.parent.parent:
        raise ValueError('候选输出不能覆盖原始标定任务或模型快照目录')
    dest.mkdir(parents=True,exist_ok=True)
    for i,mesh in enumerate(root.findall('.//mesh')):
        path=mesh.get('filename','')
        if not path or path.startswith('package://') or Path(path).is_absolute(): continue
        source=resolve_relative_mesh(path,snapshot,metadata)
        if source is None: raise ValueError(f'缺少网格资源 {path}')
        assets=dest/'resources';assets.mkdir(exist_ok=True)
        relative=Path('resources')/f'{i:03d}-{source.name}'
        shutil.copy2(source,dest/relative)
        mesh.set('filename',relative.as_posix())
    # Static gravity scores do not certify the full rigid-body dynamics
    nonphysical = _physical_urdf_links(root)
    if nonphysical:
        passed = False
        warnings.append('候选模型含非物理惯量 Link：' + ', '.join(nonphysical) + '，不能按完整动力学模型验收')
    filename=dest/f'candidate-{mode}.urdf'
    tree.write(filename,encoding='utf-8',xml_declaration=True)
    report={
        'schema':'serial_arm_advanced_identification', 'task_id':metadata.get('task_id'),
        'identification_mode':mode,'source_urdf_fingerprint':metadata.get('urdf_fingerprint'),
        'candidate_urdf':str(filename), 'locked_links':sorted(locked),
        'status':'review_passed_offline' if passed else 'requires_review',
        'automatic_application_allowed':False, 'validated_on_hardware':False,
        'friction_validation_reusable':False, 'friction_status':'requires_revalidation_with_selected_model',
        'regressor_rank':rank, 'regressor_columns':columns,
        'full_dynamics_physical_pass':not bool(nonphysical),
        'nonphysical_inertial_links':nonphysical,
        'validation_scope':'static_gravity' if mode in ('mass','com','mass_com') else 'dynamic_inertia',
        'original_holdout_rms_nm':original,'candidate_holdout_rms_nm':candidate,
        'warnings':warnings,'unmodified_links':unchanged,**extra,
        'limitations':['只读候选，不修改原始模型或控制器','没有独立关节力矩真值和加速度测量，真机使用必须另行验收'],
    }
    (dest/'identification-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return report
