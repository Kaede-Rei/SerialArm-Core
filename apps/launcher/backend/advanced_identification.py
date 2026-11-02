"""Offline supplementary identification, scoped to a fingerprint-verified task snapshot

All results are review-only. Nothing here changes robot runtime parameters
"""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import numpy as np

from full_inertial import (
    _prepare_inputs, _articulated_mapping, _sampling_map, _extract_rows, _assign_configuration,
    _fit_regressor, _dynamic_parameters_to_urdf_fields, physical_inertia,
)

def _native_file_fingerprint(contents):
    """Match Core's 64-bit FNV-1a fingerprint for runtime candidate validation."""
    value=1469598103934665603
    for item in contents:
        value=((value^item)*1099511628211)&0xffffffffffffffff
    return f'{value:016x}'


def _source_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _identification_source_hashes(folder, snapshot):
    """Bind static scores to accepted samples as well as the original model."""
    folder = Path(folder)
    accepted = folder / 'accepted_static.csv'
    return {
        'frames_sha256': _source_sha256(folder / 'frames.csv'),
        'accepted_static_sha256': _source_sha256(accepted) if accepted.is_file() else None,
        'source_sha256': _source_sha256(snapshot),
    }


def _verify_identification_sources(folder, snapshot, expected):
    try:
        actual = _identification_source_hashes(folder, snapshot)
    except OSError as exc:
        raise ValueError('辨识期间原始数据或模型不可读取，拒绝保存不一致的候选') from exc
    if actual != expected:
        raise ValueError('辨识期间原始数据或模型发生变化，拒绝保存不一致的候选')


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
    try:
        static_poses = _pose_groups(path, names)
        training = sum(not pose[1] for pose in static_poses)
        holdout = sum(pose[1] for pose in static_poses)
        static_error = None
    except (ValueError, KeyError, TypeError) as error:
        static_poses = []
        training = holdout = 0
        static_error = str(error)
    try:
        from dynamic_identification import load_observations
        observed = load_observations(path)
        checked_rows = [s for s in observed['segments'] if observed['t'][s['hi']-1]-observed['t'][s['lo']] >= 3.5]
        if not checked_rows:
            raise ValueError('连续有效运动段不足 3.5 秒')
        dynamic_error = None
    except (ValueError, KeyError, TypeError) as error:
        checked_rows = []
        dynamic_error = str(error)
    return {
        'directory': str(path), 'task_id': metadata.get('task_id', ''),
        'static_groups': len(static_poses), 'static_rejection_reason': static_error, 'training_groups': training, 'holdout_groups': holdout,
        'valid_dynamic_samples': sum(x['hi']-x['lo'] for x in checked_rows),
        'position_derived_motion_blocks': len(checked_rows), 'raw_motion_sample_count': valid_dynamic, 'peak_joint_speed_rad_s': peak_vel,
        'peak_joint_acceleration_rad_s2': peak_acc, 'phase_samples': phases,
        'gravity_ready': training >= 4 and holdout >= 1,
        'dynamic_precheck': bool(checked_rows),
        'dynamic_rejection_reason': dynamic_error,
        'validated_dynamic_sample_count': sum(x['hi']-x['lo'] for x in checked_rows),
        'limitations': [
            '有效动态帧数和速度幅度仅供预筛，不能替代动力学回归矩阵可辨识性检查',
            '电机反馈力矩不是独立的关节力矩真值，加速度由现有数据源估计',
            '未采样的末端可动关节默认停留在 URDF 中立位置，若实际有运动则需要补充测量',
        ],
    }


def _pose_groups(directory, joint_names):
    # New tasks must use accepted_static.csv; do not promote rejected raw frames
    from gravity_legacy import _static_poses
    poses = _static_poses(directory, joint_names)
    return [(index+1, heldout, q, tau, None)
            for index, (heldout, q, tau) in enumerate(poses)]


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
    for k, (jid, name, _) in enumerate(mapped):
        enabled = name not in locked_links
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
        _assign_configuration(model, qq, q_idx, q)
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
        values.append((holdout, design, torque - prediction, group, prediction, torque))
    train = [v for v in values if not v[0]]
    hold = [v for v in values if v[0]]
    from gravity_mass_com import solve_static_updates
    # Keep this supplementary entrypoint's existing bias-separated objective
    # explicit; raw URDF deployment evidence is evaluated independently below.
    objective = 'bias_separated'
    solved = solve_static_updates([v[1] for v in train], [v[2] for v in train],
                                  objective=objective)
    delta, bias = solved['delta'], solved['torque_bias']
    rank, regularization = solved['rank'], solved['regularization']
    warnings=[]
    full_delta = np.zeros_like(all_prior)
    full_delta[selected] = delta
    if mode == 'mass':
        for column,pindex in enumerate(selected):
            full_delta[pindex+1:pindex+4] += (all_prior[pindex+1:pindex+4]/all_prior[pindex])*delta[column]
    from gravity_validation import evaluate_gravity_candidate
    validation = evaluate_gravity_candidate(
        [v[4] for v in train], [v[4]+v[1]@delta for v in train], [v[5] for v in train],
        [v[4] for v in hold], [v[4]+v[1]@delta for v in hold], [v[5] for v in hold],
        candidate_changed=bool(np.any(full_delta != 0)),
        prior_preferred=bool(solved['prior_preferred']), rank=rank)
    baseline = validation['raw_urdf']['validation']['baseline']['aggregate_rms_nm']
    candidate = validation['raw_urdf']['validation']['candidate']['aggregate_rms_nm']
    passed = validation['static_pass']
    if rank < len(selected):
        warnings.append(f'静态回归仅辨识出 {rank}/{len(selected)} 个独立方向，其他参数依赖 CAD 先验')
    if not passed:
        warnings.append('URDF 直接预测未通过离线改进判据: ' + validation['status'])
    warnings.append('评价仅为有限的同会话姿态留出证据，未验证跨会话泛化或真机补偿效果')
    return {'mode': mode, 'prior': all_prior, 'indices': selected, 'delta': delta, 'full_delta': full_delta,
            'rank': rank, 'columns': len(selected), 'regularization': regularization,
            'baseline': baseline, 'candidate_rms': candidate, 'passed': passed,
            'warnings': warnings, 'train_groups': len(train), 'holdout_groups': len(hold),
            'locked_links': sorted(locked_links), 'bias': bias.tolist(),
            'solver': solved.get('solver', 'static_centered_svd_ridge_pose_cv'),
            'fit_objective': objective, 'deployment_objective': 'raw_urdf',
            'gravity_validation': validation,
            'torque_bias_convention': 'model_minus_measured',
            'prior_preferred': solved['prior_preferred'],
            'singular_values': solved['singular_values']}


def _write_static_candidate(root, mapped, fit):
    # Import postponed until after a successful fit
    import xml.etree.ElementTree as ET
    updated = fit['prior'].copy()
    updated += fit['full_delta']
    for jid, name, link in mapped:
        if name in fit['locked_links']:
            continue
        p = updated[10*(jid-1):10*jid]
        m=float(p[0])
        if not np.isfinite(p).all() or m <= 1e-7:
            raise ValueError(f'{name}: 静态候选质量无效，拒绝导出且不收缩或截断数值解')
        c=p[1:4]/m
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
    """Review the solved subspace without per-link projection or post-fit masking."""
    candidate = np.asarray(target, dtype=float).copy()
    details = []
    active = set(active_parameters)
    for k, (_, name, _) in enumerate(mapped):
        before, after = prior[10*k:10*k+10], candidate[10*k:10*k+10]
        fixed = [i for i in range(10) if name in locked or i not in active]
        if not np.array_equal(before[fixed], after[fixed]):
            raise ValueError(f'{name}: 锁定参数在求解期间发生变化')
        try:
            m, c, tensor = _dynamic_parameters_to_urdf_fields(after)
            if not physical_inertia(m, c, tensor):
                details.append(f'{name}: 原始数值解未通过物理惯量检查，保留失败诊断且不投影或收缩参数')
        except ValueError as error:
            details.append(f'{name}: {error}')
    return candidate, details


def _validate_dynamic_candidate(fit, candidate):
    from full_inertial import _validate_dynamic_candidate as validation
    return validation(fit, candidate)


def run_advanced(directory, mode, destination=None, locked_links=None):
    if mode in ('full','inertia'):
        from dynamic_identification import identify
        report = identify(directory,mode='local' if locked_links else 'global',
                          locked_links=locked_links or [],destination=destination,parameter_target=mode)
        # Legacy clients may display a review report, but never activate an
        # unexported raw representative through the old temporary-apply path.
        report.update(candidate_urdf=None,validation_scope='dynamic_diagnostic',
                      full_dynamics_physical_pass=report['physical_pass'],
                      regressor_rank=report['structural_rank'],
                      regressor_columns=len(report['free_parameter_indices']),
                      original_holdout_rms_nm=report['validation_motion']['baseline']['rms_nm'],
                      candidate_holdout_rms_nm=report['validation_motion']['candidate']['rms_nm'],
                      warnings=report['blocking_reasons'])
        return report
    if mode not in MODES:
        raise ValueError('unknown advanced identification target')
    import pinocchio as pin
    import xml.etree.ElementTree as ET
    from full_inertial import _review_candidate, _extract_rows
    from model_calibration_source import resolve_relative_mesh
    import shutil
    folder, metadata, result, snapshot, _source = _prepare_inputs(directory)
    source_hashes = _identification_source_hashes(folder, snapshot)
    root=ET.parse(snapshot).getroot()
    from full_inertial import _apply_recorded_gravity
    model=_apply_recorded_gravity(pin.buildModelFromUrdf(str(snapshot)),metadata)
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
        rank=fit['rank']; columns=fit['columns']; original=fit['baseline']; candidate=fit['candidate_rms'];passed=fit['passed']
        _write_static_candidate(root,mapped,fit)
        extra={'training_groups':fit['train_groups'],'holdout_groups':fit['holdout_groups'],
               'cross_validation_lambda':fit['regularization'],
               'solver':fit['solver'], 'torque_bias_convention':fit['torque_bias_convention'],
               'prior_preferred':fit['prior_preferred'], 'singular_values':fit['singular_values'],
               'fit_objective':fit['fit_objective'], 'deployment_objective':fit['deployment_objective'],
               'gravity_validation':fit['gravity_validation'],
               'validation_conclusion':fit['gravity_validation']['status'],
               'holdout_metric':'raw_urdf_aggregate_rms_nm'}
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
        extra={'solver':'dynamic_reduced_svd_irls','acceleration_input':'recorded_acceleration_no_reconstruction','dynamic_frames':len(rows),'observable_directions':rank,'unobservable_directions':columns-rank,'friction_validation':'independent_training_fit'}
    # Check the exact files used for fitting before creating export artifacts.
    # Presence/absence changes of accepted_static.csv are included in the check.
    _verify_identification_sources(folder, snapshot, source_hashes)
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
        'runtime_provenance':metadata.get('runtime_provenance'), **source_hashes,
        'source_files_verified_unchanged':True,
        'solver_source_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('advanced_identification.py','full_inertial.py','gravity_mass_com.py','gravity_validation.py')}, 'candidate_urdf':str(filename), 'candidate_urdf_sha256':hashlib.sha256(filename.read_bytes()).hexdigest(), 'candidate_urdf_fingerprint':_native_file_fingerprint(filename.read_bytes()), 'locked_links':sorted(locked),
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
    # Asset copying and report preparation can take time; do not publish a
    # validated report if input files changed after the numerical solve.
    _verify_identification_sources(folder, snapshot, source_hashes)
    (dest/'identification-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    return report
