"""Exact historical dynamic fit for gravity mass/COM proposals

Preserve original fitted parameters without post-fit Link replacement.
Physical bounds and independent static evaluation only annotate proposals
and prevent unsafe URDF export, never alter the numerical solution.
"""
from __future__ import annotations

import csv
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from model_calibration_source import resolve_relative_mesh, verified_source_urdf
from legacy_full_inertial import (_prepare_inputs, _articulated_mapping,
                                  _extract_rows, _fit_regressor, _sampling_map)
from gravity_mass_com import fit_global_mass_com


def _digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def _rms(fit, parameters):
    import numpy as np
    X, tau = fit['validation']
    pred = X @ np.r_[parameters, fit['nuisance']]
    return float(np.sqrt(np.mean((tau - pred)**2)))


def _scope(mapped, mode, locked_links):
    if mode not in ('global', 'local'):
        raise ValueError('未知的辨识模式')
    names = {name for _jid, name, _link in mapped}
    locked = list(locked_links or []) if mode == 'local' else []
    if (any(not isinstance(x, str) or not x for x in locked)
            or len(set(locked)) != len(locked) or not set(locked) <= names):
        raise ValueError('可信 Link 列表不属于本次辨识任务')
    if set(locked) == names:
        raise ValueError('所有 Link 均已锁定')
    return locked


def _com_reason(prior, trial, max_shift, max_mass_fraction=0.30):
    """Validate a mass and COM against the unchanged URDF prior"""
    import numpy as np
    if len(trial) < 4 or not np.isfinite(trial[:4]).all():
        return '非有限质量或质心'
    m0, m = float(prior[0]), float(trial[0])
    if m0 <= 1e-7 or m <= 1e-7:
        return '质量必须为正'
    if abs(m / m0 - 1) > max_mass_fraction:
        return '质量变化超过允许范围'
    if np.linalg.norm(trial[1:4] / m - prior[1:4] / m0) > max_shift + 1e-10:
        return '质心偏移超过允许范围'
    return ''


def _static_poses(folder, joints):
    """Read recorded stationary poses, keeping validation groups independent"""
    import numpy as np
    groups = defaultdict(list)
    # Existing task archives keep their historic minimum.  New tasks use
    # metadata to opt into the stronger multi-cycle stability requirements.
    try:
        metadata = json.loads((Path(folder) / 'metadata.json').read_text(encoding='utf-8'))
        minimum_frames = 30 if metadata.get('measurement_policy', {}).get('gain_independent_of_runtime_tuning') else 15
    except (OSError, ValueError, TypeError, AttributeError):
        minimum_frames = 15
    # New data MUST have an approved, atomic static sample archive.  Never
    # silently fall back to raw frames, including if the accepted file is lost.
    accepted = Path(folder) / 'accepted_static.csv'
    new_policy = (metadata.get('measurement_policy', {}) if isinstance(metadata, dict) else {})
    if new_policy.get('static_source') == 'accepted_static.csv' and not accepted.is_file():
        raise ValueError('缺少本任务的已验收静态采样文件 accepted_static.csv')
    static_source = accepted if accepted.exists() else Path(folder) / 'frames.csv'
    if accepted.exists():
        minimum_frames = max(minimum_frames, 30)
    with static_source.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            if row.get('phase') != 'static_reverse' or row.get('valid') != '1':
                continue
            group = row.get('pose_group', '')
            if not group or group == '0' or row.get('validation') not in ('0', '1'):
                continue
            try:
                if float(row.get('feedback_age_ms') or 999) > 30:
                    continue
                q = np.array([float(row[f'q:{n}']) for n in joints])
                v = np.array([float(row[f'dq:{n}']) for n in joints])
                torque = np.array([float(row[f'tau:{n}']) for n in joints])
                if not (np.isfinite(q).all() and np.isfinite(v).all()
                        and np.isfinite(torque).all() and np.max(np.abs(v)) < 0.04):
                    continue
                groups[(group, row['validation'])].append((q, torque))
            except (KeyError, TypeError, ValueError):
                continue
    rows = []
    for (group, validation), samples in groups.items():
        if len(samples) < minimum_frames:
            continue
        # End-of-hold frames suppress any transient before the pose settles
        stable = samples[-min(60, len(samples)):]
        rows.append((validation == '1', np.median([x[0] for x in stable], axis=0),
                     np.median([x[1] for x in stable], axis=0)))
    if sum(not x[0] for x in rows) < 3 or sum(x[0] for x in rows) < 2:
        raise ValueError('缺少独立静态训练或验证姿态，不能审核离线重力模型')
    return rows


def _static_holdout(folder, model, pin, mapped, prior):
    """Independent gravity-only holdout, not the moving-trajectory training data"""
    import numpy as np
    # mapped entries are (model joint id, child Link name, XML Link)
    joint_names = [str(model.names[jid]) for jid, _link_name, _link in mapped]
    poses = _static_poses(folder, joint_names)
    q_indices, v_indices, param_indices = _sampling_map(model, mapped)
    full_prior = np.concatenate([np.asarray(model.inertias[j].toDynamicParameters(), dtype=float)
                                 for j in range(1, model.njoints)])
    q_neutral = np.asarray(pin.neutral(model), dtype=float)
    data = model.createData()
    training, heldout = [], []
    for is_val, q, tau in poses:
        state = q_neutral.copy()
        state[q_indices] = q
        Y = np.asarray(pin.computeJointTorqueRegressor(
            model, data, state, np.zeros(model.nv), np.zeros(model.nv)), dtype=float)[v_indices, :]
        moving = Y[:, param_indices]
        fixed = Y @ full_prior - moving @ prior
        (heldout if is_val else training).append((moving, fixed, tau))
    return training, heldout


def _gravity_metrics(rows, prior, candidate, bias, scale):
    """Prediction error for one group of held-out static poses"""
    import numpy as np
    diff = np.asarray([tau - (X @ candidate + fixed + bias)
                       for X, fixed, tau in rows])
    rms = np.sqrt(np.mean(diff**2, axis=0))
    norm = np.maximum(scale, 0.02)
    # Weight each joint so a large shoulder torque does not hide a wrist gain
    score = float(np.mean((rms / norm)**2))
    return {'score': score, 'rms': rms.tolist(),
            'p99': np.percentile(np.abs(diff), 99, axis=0).tolist(),
            'max': np.max(np.abs(diff), axis=0).tolist()}


def _static_validation(training, heldout, prior, candidate):
    """Validate the actual gravity-only URDF candidate without changing it

    Per-candidate constant torque biases are estimated only from training
    stationary poses and reused unchanged on held-out stationary poses.
    """
    import numpy as np
    def measurements(rows):
        X = np.asarray([entry[0] for entry in rows], dtype=float)
        tau = np.asarray([entry[2] - entry[1] for entry in rows], dtype=float)
        return X, tau
    X, tau = measurements(training)
    V, measured = measurements(heldout)
    prior_bias = np.mean(tau - np.einsum('pji,i->pj', X, prior), axis=0)
    candidate_bias = np.mean(tau - np.einsum('pji,i->pj', X, candidate), axis=0)
    original_error = tau - np.einsum('pji,i->pj', X, prior) - prior_bias
    scale = np.maximum(np.sqrt(np.mean(original_error**2, axis=0)), 0.02)
    before = _gravity_metrics(heldout, prior, prior, prior_bias, scale)
    after = _gravity_metrics(heldout, prior, candidate, candidate_bias, scale)
    return before, after, scale


def _preserve_com_inertia(prior, selected):
    """Convert selected mass/COM to dynamic parameters for the *exported* URDF

    The XML inertia tensor is about the COM and is not changed during export.
    Pinocchio uses inertia about the Link origin, so its six dynamic moment
    entries must include the updated parallel-axis term for a fair check.
    """
    import numpy as np
    out = np.asarray(selected, dtype=float).copy()
    prior = np.asarray(prior, dtype=float)
    if len(out) != len(prior) or len(out) % 10:
        raise ValueError('动力学参数数量无效')
    def parallel_axis(m, c):
        return m * ((c @ c) * np.eye(3) - np.outer(c, c))
    for pos in range(0, len(out), 10):
        old, new = prior[pos:pos+10], out[pos:pos+10]
        m0, m1 = float(old[0]), float(new[0])
        c0, c1 = old[1:4] / m0, new[1:4] / m1
        original = np.array([[old[4], old[5], old[7]],
                             [old[5], old[6], old[8]],
                             [old[7], old[8], old[9]]])
        changed = original - parallel_axis(m0, c0) + parallel_axis(m1, c1)
        out[pos+4:pos+10] = (changed[0, 0], changed[0, 1], changed[1, 1],
                              changed[0, 2], changed[1, 2], changed[2, 2])
    return out


def _raw_gravity_candidate(fit, mapped, metadata, mode, locked):
    """Expose exactly the legacy mass/first-moment output for every Link

    Never substitute online values and never clip a numerical result.
    """
    import numpy as np
    prior = np.asarray(fit['prior'], dtype=float)
    raw = np.asarray(fit['candidate'], dtype=float)
    if not np.isfinite(raw).all():
        raise ValueError('旧版动态回归产生非有限参数，已停止计算')
    max_shift = float((metadata.get('calibration_options') or {}).get('max_com_offset_m', 0.05))
    if not math.isfinite(max_shift) or not (0 < max_shift <= 0.2):
        max_shift = 0.05
    masses, moments, offsets, shifts, invalid = {}, [], [], {}, {}
    for i, (_jid, name, _link) in enumerate(mapped):
        old, new = prior[10*i:10*i+10], raw[10*i:10*i+10]
        if name in locked and not np.array_equal(old, new):
            raise ValueError(f'可信 Link 参数被修改: {name}')
        masses[name] = float(new[0])
        moments.append({'link_name': name, 'value': new[1:4].tolist()})
        reason = _com_reason(old, new, max_shift)
        if reason:
            invalid[name] = reason
        if abs(float(new[0])) > 1e-7:
            offset = (new[1:4] / new[0] - old[1:4] / old[0]).tolist()
            shifts[name] = float(np.linalg.norm(offset))
            offsets.append({'link_name': name, 'value': offset})
        else:
            shifts[name] = None
    return prior, raw, masses, moments, offsets, shifts, invalid



def recompute(directory, mode='global', locked_links=None):
    """Recompute original dynamic regression, without merging the online fit

    `result.json` always remains the online C++ result. The raw legacy fit is
    persisted separately with static validation and export safety diagnostics.
    """
    try:
        import pinocchio as pin
    except ImportError as error:
        raise ValueError('离线动力学回归需要 Python Pinocchio，请在 Launcher 的 Python 环境中安装') from error
    import numpy as np
    folder, metadata, result, snapshot, _ = _prepare_inputs(directory)
    joints = metadata.get('joint_names') or []
    if not isinstance(joints, list) or not joints or len(set(joints)) != len(joints):
        raise ValueError('任务采样关节顺序无效')
    model = pin.buildModelFromUrdf(str(snapshot))
    mapped, _ = _articulated_mapping(ET.parse(snapshot).getroot(), model, joints)
    locked = _scope(mapped, mode, locked_links)
    frames = _extract_rows(folder / 'frames.csv', joints)
    old_fit = _fit_regressor(frames, model, pin, mapped)
    selection = {}
    rejected_raw = {}
    selection_policy = None
    model_fit = None
    if mode == 'global':
        # Solve the actual exported model: mass + first moment with the
        # original COM inertia tensor held constant. Legacy full-inertia
        # estimates are kept *only* to diagnose rank loss, not for selection.
        static_training, static_heldout = _static_holdout(
            folder, model, pin, mapped, np.asarray(old_fit['prior'], float))
        options = metadata.get('calibration_options') or {}
        max_shift = float(options.get('max_com_offset_m', 0.05))
        if not math.isfinite(max_shift) or not (0 < max_shift <= 0.2):
            max_shift = 0.05
        model_fit = fit_global_mass_com(old_fit, static_training,
                                         static_heldout, max_shift=max_shift)
        fit = dict(old_fit)
        fit['candidate'] = model_fit['candidate']
        fit['nuisance'] = model_fit['nuisance']
        selected = model_fit['candidate']
        selection = {name: {'source': 'bounded_direct_mass_first_moment'}
                     for _jid, name, _link in mapped}
        selection_policy = {
            'method': 'bounded_direct_mass_first_moment',
            'inertia_about_com': 'original_urdf_unchanged',
            'mass_fraction': model_fit['constraints']['mass_fraction'],
            'max_com_offset_m': model_fit['constraints']['max_com_offset_m'],
            'regularization': model_fit['regularization'],
            'observable_gravity_rank': model_fit['static_gravity_rank'],
            'unobservable_parameters': model_fit['static_gravity_nullity'],
        }
        # Raw ten-parameter dynamic regression is *diagnostic only*.
        max_review = max_shift
        for i, (_jid, name, _link) in enumerate(mapped):
            old = np.asarray(old_fit['prior'][10*i:10*i+10])
            new = np.asarray(old_fit['candidate'][10*i:10*i+10])
            warning = _com_reason(old, new, max_review)
            if warning:
                rejected_raw[name] = warning
    else:
        # Preserve the working local workflow: legacy global fit, followed
        # by explicit Link selection without a second least-squares fit.
        fit = dict(old_fit)
        selected = np.asarray(old_fit['candidate'], dtype=float).copy()
        prior_parameters = np.asarray(old_fit['prior'], dtype=float)
        for i, (_jid, name, _link) in enumerate(mapped):
            if name in locked:
                selected[10*i:10*i+10] = prior_parameters[10*i:10*i+10]
        fit['candidate'] = selected
    prior, raw, masses, moments, offsets, shifts, invalid = _raw_gravity_candidate(
        fit, mapped, metadata, mode, set(locked))
    # The XML COM inertia tensor is never changed. Pinocchio dynamic inertia
    # about the Link origin must reflect the parallel-axis theorem for scoring
    # the *exported* URDF, not the original (possibly nonphysical) 60D fit.
    exported = None
    before = after = noise = None
    static_status = 'not_evaluated'
    static_reason = '独立静态验证尚未执行'
    try:
        if any(abs(masses[name]) <= 1e-7 for name in masses):
            raise ValueError('候选存在零质量，无法计算模型重力预测')
        exported = _preserve_com_inertia(prior, raw)
        training, heldout = _static_holdout(folder, model, pin, mapped, prior)
        before, after, noise = _static_validation(training, heldout, prior, exported)
        improving = after['score'] < before['score'] * 0.99
        static_status = 'passed' if improving else 'failed'
        static_reason = ('独立静态留出姿态的重力力矩预测有改善' if improving
                         else '独立静态留出姿态未显示足够的重力力矩改善')
    except (ValueError, KeyError, TypeError, ArithmeticError) as exc:
        static_status = 'unavailable'
        static_reason = '静态验证不可用: ' + str(exc)
    constraints_ok = not invalid
    # Static predictive evidence and physical export admissibility are distinct
    static_pass = static_status == 'passed'
    if not constraints_ok:
        reason = '; '.join(f'{name}: {issue}' for name, issue in invalid.items())
    else:
        reason = static_reason
    report = {
        'solver': (model_fit['solver'] if model_fit else 'legacy_full_inertial_exact'),
        'task_id': metadata['task_id'],
        'core_fingerprint': metadata.get('core_fingerprint'),
        'urdf_fingerprint': metadata.get('urdf_fingerprint'),
        'frames_sha256': _digest(folder / 'frames.csv'),
        'accepted_static_sha256': (_digest(folder / 'accepted_static.csv') if (folder / 'accepted_static.csv').exists() else None),
        'source_sha256': _digest(snapshot),
        'identification_mode': mode,
        'locked_links': locked,
        'fitted_links': [name for _jid, name, _link in mapped if name not in locked],
        'local_scope_method': 'global_legacy_fit_then_mask_locked_links' if mode == 'local' else None,
        'global_selection': selection if mode == 'global' else None,
        'global_selection_policy': selection_policy,
        'raw_link_review': rejected_raw,
        'first_moments': moments,
        'candidate_masses': masses,
        'com_offsets_m': offsets,
        'original_dynamic_parameters': prior.tolist(),
        'raw_dynamic_parameters': np.asarray(old_fit['candidate'], dtype=float).tolist(),
        'scoped_dynamic_parameters': raw.tolist(),
        'selected_dynamic_parameters': exported.tolist() if exported is not None else None,
        'numerical_rank': (model_fit['static_gravity_rank'] if model_fit else fit['rank']),
        'regressor_columns': (model_fit['mass_moment_columns'] if model_fit else fit['degrees']),
        'legacy_full_inertia_rank': fit['rank'],
        'legacy_full_inertia_columns': fit['degrees'],
        'global_model_fit': ({key:value for key,value in model_fit.items() if key not in ('prior','candidate','exported','nuisance')} if model_fit else None),
        'original_holdout_rms_nm': _rms(fit, prior),
        'raw_holdout_rms_nm': _rms(old_fit, old_fit['candidate']),
        'candidate_holdout_rms_nm': _rms(fit, exported) if exported is not None else None,
        'static_holdout_score_before': before['score'] if before else None,
        'static_holdout_score_after': after['score'] if after else None,
        'validation_original': {k: before[k] for k in ('rms', 'p99', 'max')} if before else {},
        'validation_candidate': {k: after[k] for k in ('rms', 'p99', 'max')} if after else {},
        'validation_scaled': {k: before[k] for k in ('rms', 'p99', 'max')} if before else {},
        'noise_rms': noise.tolist() if noise is not None else [],
        'com_shifts_m': shifts,
        'link_review': invalid,
        'review_warnings': ([f'{name}: 完整惯量诊断存在异常参数，未用于重力候选: {issue}' for name, issue in rejected_raw.items()]
                            if mode == 'global' else [])
                           + [f'{name}: {issue}' for name, issue in invalid.items()],
        'constraints_ok': constraints_ok,
        'static_validation_status': static_status,
        'static_pass': static_pass,
        'status': 'needs_review',
        'failure_reason': reason,
        'friction_status': 'not_evaluated',
        'friction_pass': None,
    }
    target = folder / 'recomputed-candidate.json'
    temp = target.with_suffix('.json.tmp')
    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(target)
    return {'path': str(target), 'candidate': report, 'static_pass': static_pass,
            'output': ('全局质量与质心约束求解已完成，原完整惯量拟合仅保留用于诊断' if mode == 'global'
                       else '旧版动态回归已完成，局部辨识保留原始数值解')}


def export_recomputed(directory, destination=None):
    """Export only the *stored* selected gravity candidate, never re-fit"""
    import numpy as np
    folder, metadata, _result, snapshot, _ = _prepare_inputs(directory)
    record = folder / 'recomputed-candidate.json'
    if not record.is_file():
        raise ValueError('该任务尚无离线候选，请先执行离线重算')
    candidate = json.loads(record.read_text(encoding='utf-8'))
    if candidate.get('solver') not in ('legacy_full_inertial_exact', 'bounded_direct_mass_first_moment'):
        raise ValueError('离线候选由旧计算器生成，请重新执行离线重算')
    if (candidate.get('task_id') != metadata.get('task_id')
            or candidate.get('urdf_fingerprint') != metadata.get('urdf_fingerprint')
            or candidate.get('core_fingerprint') != metadata.get('core_fingerprint')
            or candidate.get('frames_sha256') != _digest(folder / 'frames.csv')
            or candidate.get('accepted_static_sha256') != (
                _digest(folder / 'accepted_static.csv') if (folder / 'accepted_static.csv').exists() else None)
            or candidate.get('source_sha256') != _digest(snapshot)):
        raise ValueError('离线候选与原始任务数据不匹配，请重新计算')
    masses = candidate['candidate_masses']
    moments = {item['link_name']: item['value'] for item in candidate['first_moments']}
    locked = set(candidate.get('locked_links') or [])
    if candidate.get('identification_mode') not in ('global', 'local'):
        raise ValueError('离线候选辨识模式无效')
    # Older records with the same solver label may contain unguarded whole-arm
    # inertias; force a new computation rather than silently exporting them
    if candidate.get('identification_mode') == 'global' and (
            candidate.get('solver') != 'bounded_direct_mass_first_moment' or
            (candidate.get('global_selection_policy') or {}).get('method') != 'bounded_direct_mass_first_moment'):
        raise ValueError('全局候选不是新的约束质量与质心求解结果，请重新执行离线重算')
    tree = ET.parse(snapshot)
    root = tree.getroot()
    link_map = {link.get('name'): link for link in root.findall('link')}
    source_inertias = {name: ET.tostring(link.find('inertial')) for name, link in link_map.items()
                       if link.find('inertial') is not None}
    fitted = candidate.get('fitted_links') or []
    if (set(fitted) & locked or set(fitted) | locked != set(masses) or
            len(set(fitted)) != len(fitted) or set(masses) != set(moments) or
            not set(masses) <= set(link_map)):
        raise ValueError('离线候选的 Link 辨识范围或质量与质心参数不匹配')
    changed = []
    invalid = []
    if not candidate.get('constraints_ok'):
        raise ValueError('原始旧版候选包含不合理质量或质心，拒绝整机导出，请使用局部辨识锁定可信 Link: ' + '; '.join(
            f'{name}: {message}' for name, message in candidate.get('link_review', {}).items()))
    max_shift = float((metadata.get('calibration_options') or {}).get('max_com_offset_m', 0.05))
    if not (math.isfinite(max_shift) and 0 < max_shift <= 0.2):
        max_shift = 0.05
    for name in fitted:
        link = link_map[name]
        inertial = link.find('inertial')
        if inertial is None or inertial.find('mass') is None:
            raise ValueError(f'{name} 不含有效惯性质量')
        origin = inertial.find('origin')
        old_mass = float(inertial.find('mass').get('value'))
        old_com = np.asarray([float(x) for x in origin.get('xyz', '0 0 0').split()]
                             if origin is not None else [0., 0., 0.])
        mass = float(masses[name])
        h = np.asarray(moments[name], dtype=float)
        original = np.r_[old_mass, old_mass * old_com]
        proposed = np.r_[mass, h]
        reason = _com_reason(original, proposed, max_shift)
        if reason:
            invalid.append(f'{name}: {reason}')
            continue
        com = h / mass
        if origin is None:
            origin = ET.Element('origin')
            inertial.insert(0, origin)
        origin.set('xyz', ' '.join(f'{v:.12g}' for v in com))
        inertial.find('mass').set('value', f'{mass:.12g}')
        changed.append(name)
    if invalid:
        raise ValueError('候选参数未通过检查，拒绝导出: ' + '; '.join(invalid))
    if not changed:
        raise ValueError('候选中没有可导出的 Link')
    for name, link in link_map.items():
        inertial = link.find('inertial')
        original = source_inertias.get(name)
        if original is None:
            if inertial is not None:
                raise ValueError(f'{name} 意外新增惯性参数')
            continue
        if name in locked or name not in changed:
            if ET.tostring(inertial) != original:
                raise ValueError(f'{name} 未经允许被修改')
        if ET.tostring(ET.fromstring(original).find('inertia')) != ET.tostring(inertial.find('inertia')):
            raise ValueError(f'{name} 原始惯性张量发生改变')
    output = (Path(destination).expanduser().resolve() / 'offline' if destination
              else folder / 'candidate' / 'offline')
    if output == folder or folder == output.parent or output == snapshot.parent:
        raise ValueError('不能向原始模型目录导出候选')
    output.mkdir(parents=True, exist_ok=True)
    copied = []
    for idx, mesh in enumerate(root.findall('.//mesh')):
        name = mesh.get('filename', '')
        if not name or name.startswith('package://') or Path(name).is_absolute():
            continue
        src = resolve_relative_mesh(name, snapshot, metadata)
        if src is None:
            raise ValueError(f'缺少 URDF 引用的网格资源: {name}')
        resources = output / 'resources'
        resources.mkdir(exist_ok=True)
        dest = resources / f'{idx:03d}-{src.name}'
        shutil.copy2(src, dest)
        mesh.set('filename', str(dest.relative_to(output)))
        copied.append(str(dest))
    outfile = output / 'candidate-offline.urdf'
    temp = outfile.with_suffix('.urdf.tmp')
    tree.write(temp, encoding='utf-8', xml_declaration=True)
    ET.parse(temp)
    temp.replace(outfile)
    verification = {'candidate_urdf': str(outfile),
                    'candidate_source': 'recomputed-candidate.json',
                    'changed_links': changed, 'locked_links_preserved': sorted(locked),
                    'inertia_values_preserved': True,
                    'mass_and_inertia_values_preserved': False,
                    'joint_geometry_modified': False,
                    'candidate_solver': candidate['solver'],
                    'review_required': True, 'holdout_pass': bool(candidate.get('static_pass')),
                    'static_validation_status': candidate.get('static_validation_status', 'not_evaluated'),
                    'review_warnings': candidate.get('review_warnings', []),
                    'copied_relative_meshes': copied}
    (output / 'candidate-urdf-verification.json').write_text(
        json.dumps(verification, indent=2, ensure_ascii=False), encoding='utf-8')
    return verification
