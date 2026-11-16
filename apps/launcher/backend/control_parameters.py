"""Canonical control parameters and source-preserving YAML additions"""
from copy import deepcopy
import math

import yaml
from yaml.nodes import MappingNode, ScalarNode
from yaml.tokens import AnchorToken, AliasToken, BlockEndToken, DocumentEndToken, StreamEndToken


_CONTROLLER_ROLES = ('rigid', 'compliant', 'drag')


def _canonical_controller(controller):
    if not isinstance(controller, dict):
        raise ValueError('control.controller 必须为映射')
    if set(controller) - {'allow_full_cmd', *_CONTROLLER_ROLES}:
        raise ValueError('control.controller 含未知控制用途')
    for key in set(controller) & set(_CONTROLLER_ROLES):
        if not isinstance(controller[key], dict) or set(controller[key]) != {'kp', 'kd'}:
            raise ValueError('control.controller.' + key + ' 必须包含 kp 和 kd')
    return deepcopy(controller)


def four_mode_core(core):
    return canonical_core(core)


def canonical_core(core):
    result = deepcopy(core)
    # Historical user PD ceilings are readable but no longer participate in
    # admission, editing, provenance hashes or new-format serialization.
    policy = result.get('safety_policy')
    if isinstance(policy, dict):
        policy.pop('max_kp_override', None)
        policy.pop('max_kd_override', None)
    names = (result.get('model') or {}).get('joint_names') or []
    shutdown = result.get('shutdown') or {}
    if not isinstance(shutdown, dict) or set(shutdown) - {'paths'}:
        raise ValueError('shutdown 仅支持 paths.MIT 与 paths.CSP')
    paths = shutdown.get('paths') or {}
    if not isinstance(paths, dict) or set(paths) - {'MIT', 'CSP'}:
        raise ValueError('shutdown.paths 仅支持 MIT 与 CSP')
    for plan in paths.values():
        if isinstance(plan, dict) and plan.get('enabled') is True and ('park_pos' not in plan or plan['park_pos'] == {}):
            plan['park_pos'] = dict.fromkeys(names, 0.0)
    admittance = (result.get('capability') or {}).get('admittance')
    if isinstance(admittance, dict):
        unknown = set(admittance) - {'joint_enabled', 'observer', 'calibration', 'controller', 'validation'}
        if unknown:
            raise ValueError('capability.admittance 含已移除或未知参数: ' + ', '.join(sorted(unknown)))
        observer = admittance.get('observer')
        if isinstance(observer, dict) and set(observer) - {'momentum_gain'}:
            raise ValueError('capability.admittance.observer 仅支持 momentum_gain')
    recovery = (result.get('safety_policy') or {}).get('fault_recovery') or {}
    removed = set(recovery) & {'allow_compliant_recovery', 'gravity_model_validated', 'compliant_recovery'}
    if removed:
        raise ValueError('safety_policy.fault_recovery 含已移除参数: ' + ', '.join(sorted(removed)))
    runtime = (result.get('control') or {}).get('runtime')
    if isinstance(runtime, dict):
        unknown = set(runtime) - {'ctrl_frequency_hz', 'joint_acc_filter_alpha', 'write_enabled',
                                  'model_feedforward_mode', 'control_path'}
        if unknown:
            raise ValueError('control.runtime 含已移除或未知参数: ' + ', '.join(sorted(unknown)))
        runtime.setdefault('control_path', 'MIT')
    hold = (result.get('control') or {}).get('hold_compensation')
    if isinstance(hold, dict):
        hold = deepcopy(hold)
        result['control']['hold_compensation'] = hold
        hold.setdefault('enabled', False)
        for key, value in (('integral_gain', .15), ('max_torque', .15), ('max_rate', .05), ('contact_threshold', .2)):
            if names: hold.setdefault(key, {name: value for name in names})
        for key, value in (('velocity_threshold', .02), ('error_threshold', .03)):
            if names: hold.setdefault(key, {name: value for name in names})
    controller = (result.get('control') or {}).get('controller')
    if controller is None:
        return result
    result['control']['controller'] = _canonical_controller(controller)
    return result


def interaction_configuration(core):
    """Validate only authored observation/MDK groups, without inventing defaults."""
    names = (core.get('model') or {}).get('joint_names') or []
    capability = core.get('capability', {})
    if not isinstance(capability, dict) or set(capability) - {'admittance'}:
        raise ValueError('capability 仅支持 admittance')
    admittance = capability.get('admittance')
    if admittance is None:
        return {'observation_configured': False, 'admittance_configured': False}
    if not isinstance(admittance, dict):
        raise ValueError('capability.admittance 必须为映射')
    prefix = 'capability.admittance.'
    if set(admittance) - {'observer', 'calibration', 'controller', 'joint_enabled', 'validation'}:
        raise ValueError('capability.admittance 含未知参数')
    validation = admittance.get('validation', {})
    if (not isinstance(validation, dict) or
            set(validation) - {'observation_validated', 'response_validated', 'velocity_tolerance'} or
            any(not isinstance(value, bool) for key, value in validation.items() if key != 'velocity_tolerance')):
        raise ValueError(prefix + 'validation 仅支持质量许可与逐轴速度稳定容差')

    def mapping(value, path, keys):
        if not isinstance(value, dict) or set(value) != set(keys):
            raise ValueError(prefix + path + ' 字段不完整或包含未知字段')
        return value

    def joint_map(value, path, minimum=None, strict=False):
        mapping(value, path, names)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or
               minimum is not None and (v <= minimum if strict else v < minimum) for v in value.values()):
            raise ValueError(prefix + path + ' 必须为有效逐关节数值')

    observed = 'observer' in admittance or 'calibration' in admittance
    if observed:
        observer = mapping(admittance.get('observer'), 'observer', ('momentum_gain',))
        joint_map(observer['momentum_gain'], 'observer.momentum_gain', 0, True)
        calibration = admittance.get('calibration')
        if not isinstance(calibration, dict) or not {'torque_bias', 'torque_threshold'} <= set(calibration) or set(calibration) - {'torque_bias', 'torque_threshold', 'friction'}:
            raise ValueError(prefix + 'calibration 字段不完整或包含未知字段')
        joint_map(calibration['torque_bias'], 'calibration.torque_bias')
        joint_map(calibration['torque_threshold'], 'calibration.torque_threshold', 0)
        friction = calibration.get('friction') or {}
        if not isinstance(friction, dict) or set(friction) - {'enabled', 'velocity_transition', 'positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous'}:
            raise ValueError(prefix + 'calibration.friction 含未知字段')
        if not isinstance(friction.get('enabled', False), bool):
            raise ValueError(prefix + 'calibration.friction.enabled 必须为布尔值')
        if 'velocity_transition' in friction or friction.get('enabled'):
            joint_map(friction.get('velocity_transition'), 'calibration.friction.velocity_transition', 0, True)
        for key in ('positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous'):
            if key in friction or friction.get('enabled'):
                joint_map(friction.get(key), 'calibration.friction.' + key)
    modeled = 'controller' in admittance or 'joint_enabled' in admittance
    if modeled or 'velocity_tolerance' in validation:
        joint_map(validation.get('velocity_tolerance'), 'validation.velocity_tolerance', 0, True)
    if modeled:
        if not observed:
            raise ValueError('导纳需要先配置 capability.admittance.observer 和 calibration')
        flags = mapping(admittance.get('joint_enabled'), 'joint_enabled', names)
        if any(not isinstance(v, bool) for v in flags.values()):
            raise ValueError(prefix + 'joint_enabled 必须为逐关节布尔值')
        controller = mapping(admittance.get('controller'), 'controller',
            ('mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot'))
        for key in ('mass', 'max_delta_q', 'max_delta_q_dot'):
            joint_map(controller[key], 'controller.' + key, 0, True)
        for key in ('damping', 'stiffness'):
            joint_map(controller[key], 'controller.' + key, 0)
            if any(flags[name] and controller[key][name] <= 0 for name in names):
                raise ValueError(prefix + 'controller.' + key + ' 参与轴必须为正数')
    return {'observation_configured': observed, 'admittance_configured': modeled}


def interaction_validation_contexts(core):
    model = deepcopy(core.get('model') or {})
    model.pop('gravity_model_validated', None)
    admittance = (core.get('capability') or {}).get('admittance') or {}
    runtime = (core.get('control') or {}).get('runtime') or {}
    observation = {'model': model, 'mapping': core.get('calibration'),
        'observer': admittance.get('observer'), 'calibration': admittance.get('calibration'),
        'runtime': {key: runtime.get(key) for key in ('ctrl_frequency_hz', 'joint_acc_filter_alpha', 'control_path')}}
    safety = core.get('safety_policy') or {}
    response = {'observation': observation, 'controller': admittance.get('controller'),
        'joint_enabled': admittance.get('joint_enabled'),
        'velocity_tolerance': (admittance.get('validation') or {}).get('velocity_tolerance'),
        'safety': {key: safety.get(key) for key in ('max_tracking_error', 'max_acc', 'max_effort_override')}}
    return observation, response


def revoke_changed_interaction_validation(before, after):
    """Revoke only the affected quality flags; historical receipts stay intact."""
    previous = interaction_validation_contexts(before)
    current = interaction_validation_contexts(after)
    admittance = (after.get('capability') or {}).get('admittance') or {}
    validation = admittance.get('validation') or {}
    reset = []
    for key, changed in zip(('observation_validated', 'response_validated'),
                            (previous[0] != current[0], previous[1] != current[1])):
        if validation.get(key) is True and changed:
            validation[key] = False
            reset.append(key)
    return reset


GAIN_PURPOSE_KEYS = ('rigid', 'compliant', 'drag')

def selected_gain_purposes(gain_purposes):
    if gain_purposes is None: return None
    aliases = {key.upper(): key for key in GAIN_PURPOSE_KEYS}
    if not isinstance(gain_purposes, (list, tuple)) or not gain_purposes:
        raise ValueError('请选择至少一种已有测量结果的增益用途')
    result=[]
    for key in gain_purposes:
        canonical=aliases.get(str(key).upper())
        if canonical is None: raise ValueError('未知增益用途: '+str(key))
        if canonical not in result: result.append(canonical)
    return [key for key in GAIN_PURPOSE_KEYS if key in result]


def _gain_number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def gain_measurement_available(validation, joint_count, kind):
    """Measurement completion is separate from meeting reference metrics"""
    if not isinstance(validation, dict) or validation.get('available') is False or validation.get('completed') is False:
        return False
    if kind == 'tracking':
        rows = validation.get('metrics')
        return (validation.get('independent') is True and _gain_number(validation.get('duration_s')) and
                validation['duration_s'] > 0 and
                ('measured_duration_s' not in validation or _gain_number(validation['measured_duration_s']) and
                 validation['measured_duration_s'] >= validation['duration_s'] - .06) and isinstance(rows, list) and
                len(rows) == joint_count and all(isinstance(row, dict) and
                _gain_number(row.get('samples')) and row['samples'] > 0 and
                ('measurement_samples' not in row or _gain_number(row['measurement_samples']) and row['measurement_samples'] > 0) and
                all(_gain_number(row.get(key)) and row[key] >= 0 for key in
                    ('rms_error_rad', 'peak_error_rad', 'velocity_rms_rad_s', 'tail_error_rad', 'settling_s'))
                for row in rows))
    if not _gain_number(validation.get('samples')) or validation['samples'] <= 0:
        return False
    if kind == 'hold':
        return (validation.get('reason') not in ('not_started', 'measurement_in_progress') and
                ('fresh_samples' not in validation or isinstance(validation['fresh_samples'], list) and
                 len(validation['fresh_samples']) == joint_count and all(_gain_number(v) and v > 0 for v in validation['fresh_samples'])) and
                all(_gain_number(validation.get(key)) and validation[key] >= 0
                    for key in ('worst_error_rad', 'worst_velocity_rad_s')))
    if kind == 'drag':
        axes = validation.get('axes')
        quiet_scope = isinstance(axes, list) and any(isinstance(axis, dict) and
                      'quiet_stability_measured' in axis for axis in axes)
        return (validation.get('completed') is True and isinstance(axes, list) and
                len(axes) == joint_count and all(isinstance(axis, dict) and
                _gain_number(axis.get('samples')) and axis['samples'] > 0 and
                ((axis.get('quiet_stability_measured') is True and
                  all(_gain_number(axis.get(key)) and axis[key] >= 0 for key in
                      ('quiet_position_span_rad', 'quiet_speed_rad_s'))) if quiet_scope else
                 all(_gain_number(axis.get(key)) and axis[key] >= 0 for key in
                     ('initial_velocity_rad_s', 'peak_velocity_rad_s', 'final_velocity_rad_s', 'decay_s')))
                for axis in axes))
    return False


def gain_purpose_available(candidate, purpose, fields):
    if not isinstance(fields, dict) or fields.get('error') in ('not_started', 'measurement_in_progress'):
        return False
    if any(value in ('not_started', 'measurement_in_progress') for value in fields.get('warnings', [])):
        return False
    if fields.get('available') is False and candidate.get('validation_scope') != 'independent_demonstration_trajectory':
        return False
    names = candidate.get('joint_names')
    count = len(names) if isinstance(names, list) else 0
    def gains(value):
        return isinstance(value, dict) and all(isinstance(value.get(term), list) and
            len(value[term]) == count and all(_gain_number(v) and v >= 0 for v in value[term])
            for term in ('kp', 'kd'))
    if count == 0 or not gains(fields.get('gains')):
        return False
    # Existing qualified local-probe records retain their recorded scope.
    # Failed old records need complete independent trajectory measurements
    if candidate.get('validation_scope') != 'independent_demonstration_trajectory':
        return fields.get('validated') is True
    if purpose == 'drag':
        return all(v == 0 for v in fields['gains']['kp']) and gain_measurement_available(fields.get('drag_validation'), count, 'drag')
    if not gain_measurement_available(fields.get('native_validation'), count, 'tracking'):
        return False
    if purpose not in ('rigid', 'compliant'):
        return False
    verification = ('hold_compensation_verification' if purpose == 'rigid' and
                    (fields.get('hold_compensation') or {}).get('enabled') is True else 'hold_verification')
    if not gain_measurement_available(fields.get(verification), count, 'hold'):
        return False
    if purpose == 'compliant':
        return (_gain_number(fields.get('tested_speed_fraction')) and fields['tested_speed_fraction'] > 0 and
                all(isinstance(fields.get(key), list) and len(fields[key]) == count and
                    all(_gain_number(v) and v >= 0 for v in fields[key])
                    for key in ('rigid_reference_kp', 'rigid_reference_kd')))
    return True


def gain_review_ready(candidate):
    category = candidate.get('category')
    groups = candidate.get('targets') if category == 'all' else {category: candidate}
    if not isinstance(groups, dict) or not groups or category == 'all' and set(groups) != set(GAIN_PURPOSE_KEYS):
        return False
    if candidate.get('validation_scope') == 'independent_demonstration_trajectory':
        return any(purpose in GAIN_PURPOSE_KEYS and gain_purpose_available(candidate, purpose, fields)
                   for purpose, fields in groups.items())
    if any(candidate.get(key) is False for key in ('review_ready', 'measurement_complete', 'qualification_complete')):
        return False
    for purpose, fields in groups.items():
        if purpose not in GAIN_PURPOSE_KEYS or not gain_purpose_available(candidate, purpose, fields):
            return False
    return True


def gain_selection_report(candidate, groups):
    def reference_only(value):
        return (value.get('reference_only') is True or value.get('accuracy_thresholds_enabled') is False or
                value.get('acceptance_scope') == 'measured_response_reference_only_v1')

    def measurement(fields, key):
        recorded = fields.get(key) or {}
        shared = fields.get(recorded.get('shared_with'))
        return {'shared_source_record_path': source_key(shared), **shared, **recorded} if (
            isinstance(shared, dict) and shared is not recorded) else recorded

    def source_key(value):
        return next((value[key] for key in ('shared_source_record_path', 'shared_record_path', 'shared_source',
                     'source_record_path', 'record_path') if isinstance(value.get(key), str) and value[key]), '')

    def direct_scope(value):
        axes = value.get('axes') if isinstance(value.get('axes'), list) else []
        if not (value.get('method') == 'isolated_smooth_torque_pulse' or 'direct_measurement_complete' in value or
                any(isinstance(axis, dict) and 'direct_response_complete' in axis for axis in axes)):
            return None
        complete = value.get('direct_measurement_complete') if isinstance(value.get('direct_measurement_complete'), bool) else (
            bool(axes) and all(isinstance(axis, dict) and axis.get('input_measurement_complete') is True and
                              axis.get('direct_response_complete') is True for axis in axes))
        return complete and not any(isinstance(axis, dict) and (axis.get('input_measurement_complete') is False or
                                    axis.get('direct_response_complete') is False) for axis in axes)

    scope_codes = ('disturbance_measurement_incomplete', 'hold_disturbance_measurement_incomplete')
    warnings, seen_sources = [], set()
    for value in candidate.get('warnings', []):
        if isinstance(value, str) and value.rsplit(':', 1)[-1] not in scope_codes:
            warnings.append(value)
    for purpose, fields in groups.items():
        drag_axes = (fields.get('drag_validation') or {}).get('axes') or []
        drag_scope_known = any(isinstance(axis, dict) and ('quiet_stability_measured' in axis or
                               'release_decay_measured' in axis) for axis in drag_axes)
        quiet_measured = bool(drag_axes) and all(isinstance(axis, dict) and
                         axis.get('quiet_stability_measured') is True for axis in drag_axes)
        warnings.extend(value for value in fields.get('warnings', []) if isinstance(value, str) and
                        value.rsplit(':', 1)[-1] not in scope_codes and not
                        (purpose == 'drag' and quiet_measured and value.startswith('drag_release')))
        if not (reference_only(candidate) or reference_only(fields)) and fields.get('validated') is not True:
            warnings.append(purpose + ' 未达参考指标，保留实测结果，可应用或导出')
        scopes = [] if purpose == 'drag' else [('disturbance_verification', '独立扰动直接响应尚未测完整')]
        if purpose in ('rigid', 'compliant'):
            scopes.append(('hold_disturbance_verification', '保持扰动测量未完成'))
        for key, note in scopes:
            evidence = measurement(fields, key)
            direct_complete = direct_scope(evidence)
            complete = direct_complete if direct_complete is not None else (evidence.get('available') is True and
                       evidence.get('completed') is not False and bool(evidence.get('record_path')))
            if complete:
                continue
            source = source_key(evidence)
            if source and source in seen_sources:
                continue
            if source:
                seen_sources.add(source)
            warnings.append(purpose + ' ' + note)
        if purpose == 'drag' and drag_scope_known:
            warnings = [value for value in warnings if not value.startswith(
                ('drag_release_decay_unmeasured', 'drag:drag_release_decay_unmeasured'))]
            if not quiet_measured:
                warnings.append(purpose + ' 无外力稳定尚未测完整')
            if any(not isinstance(axis, dict) or axis.get('release_decay_measured') is not True for axis in drag_axes):
                warnings.append(purpose + ' 释放衰减未测完整，与无外力稳定分项记录')
    rigid, soft = groups.get('rigid'), groups.get('compliant')
    if soft and not (reference_only(candidate) or reference_only(soft)) and soft.get('compliant_separation_validated') is not True:
        warnings.append('柔性增益未达刚柔分离参考指标')
    if rigid and soft and any(soft.get('rigid_reference_' + term) != rigid.get('gains', {}).get(term) for term in ('kp', 'kd')):
        warnings.append('柔性测量时的刚性参考与本次采用的刚性增益不同')
    for_reference = reference_only(candidate) or any(reference_only(fields) for fields in groups.values())
    return {'available': bool(groups), 'validated': not for_reference and bool(groups) and
            all(fields.get('validated') is True for fields in groups.values()),
            **({'reference_only': True, 'accuracy_thresholds_enabled': False} if for_reference else {}),
            'warnings': list(dict.fromkeys(warnings))}


def deployment_gain_groups(candidate, gain_purposes=None):
    if not gain_review_ready(candidate):
        raise ValueError('增益标定测量未完成，尚未生成可审核结果；请补齐缺测用途后再应用或导出')
    category = candidate.get('category')
    groups = candidate.get('targets') if category == 'all' else {category: candidate}
    if not isinstance(groups, dict) or not groups:
        raise ValueError('增益候选缺少用途记录')
    selected=selected_gain_purposes(gain_purposes)
    if selected is None:
        groups = {key: fields for key, fields in groups.items() if gain_purpose_available(candidate, key, fields)}
        if not groups: raise ValueError('增益候选尚无完整测量结果')
    else:
        if any(key not in groups for key in selected): raise ValueError('候选未包含所选增益用途')
        groups={key:groups[key] for key in selected}
    for purpose, fields in groups.items():
        if purpose not in GAIN_PURPOSE_KEYS or not gain_purpose_available(candidate, purpose, fields):
            raise ValueError('增益用途尚无完整测量结果: ' + str(purpose))
        if candidate.get('validation_scope') == 'independent_demonstration_trajectory':
            continue
        if purpose == 'drag':
            if (fields.get('drag_validation') or {}).get('validated') is not True:
                raise ValueError('拖拽候选缺少独立稳定性验收')
        elif purpose == 'rigid':
            verification = ('hold_compensation_verification'
                            if (fields.get('hold_compensation') or {}).get('enabled') is True
                            else 'hold_verification')
            if (fields.get(verification) or {}).get('validated') is not True:
                raise ValueError('刚性候选缺少保持或补偿保持验收')
        elif purpose == 'compliant':
            if (fields.get('hold_verification') or {}).get('validated') is not True:
                raise ValueError('柔性候选缺少保持验收')
        else:
            raise ValueError('未知增益用途: ' + str(purpose))
    return groups


def _node_at(root, path):
    node = root
    for key in path:
        if not isinstance(node, MappingNode):
            raise ValueError('配置父节点必须为映射: ' + '.'.join(path))
        found = [value for name, value in node.value if isinstance(name, ScalarNode) and name.value == key]
        if len(found) != 1:
            raise ValueError('配置缺少或重复父节点: ' + '.'.join(path))
        node = found[0]
    if not isinstance(node, MappingNode):
        raise ValueError('配置父节点必须为映射: ' + '.'.join(path))
    return node


def add_mapping_value(text, parent, key, value):
    """Add one missing member without rewriting existing comments or values"""
    node = _node_at(yaml.compose(text), parent)
    if any(name.value == key for name, _ in node.value):
        return text
    if node.flow_style:
        encoded = yaml.safe_dump({key: value}, allow_unicode=True, default_flow_style=True,
                                 sort_keys=False, width=100000).strip()[1:-1]
        index = node.end_mark.index - 1
        return text[:index] + (', ' if node.value else '') + encoded + text[index:]
    indent = node.value[0][0].start_mark.column if node.value else node.start_mark.column
    index = node.end_mark.index
    line_start = text.rfind('\n', 0, index) + 1
    if not text[line_start:index].strip():
        index = line_start
    encoded = yaml.safe_dump({key: value}, allow_unicode=True, sort_keys=False, width=100000)
    block = ''.join(' ' * indent + line if line.strip() else line for line in encoded.splitlines(True))
    prefix = '' if index == 0 or text[index - 1] == '\n' else '\n'
    return text[:index] + prefix + block + text[index:]


def _controller_text(text, controller):
    """Replace only the controller value and verify the complete parsed document"""
    original = yaml.safe_load(text)
    root = yaml.compose(text)
    control = _node_at(root, ('control',))
    entries = [(key, value) for key, value in control.value if key.value == 'controller']
    if len(entries) != 1 or not isinstance(entries[0][1], MappingNode):
        raise ValueError('control.controller 必须为唯一映射')
    key, node = entries[0]
    visited = set()
    def unique_keys(value):
        if id(value) in visited:
            return
        visited.add(id(value))
        if isinstance(value, MappingNode):
            keys = [key.value for key, _ in value.value if isinstance(key, ScalarNode)]
            if len(keys) != len(value.value) or len(keys) != len(set(keys)):
                raise ValueError('control.controller 不支持重复键或复合键')
            for _, child in value.value:
                unique_keys(child)
        elif isinstance(value.value, list):
            for child in value.value:
                unique_keys(child)
    unique_keys(node)
    tokens = list(yaml.scan(text))
    if node.start_mark.index <= key.end_mark.index or any(
            isinstance(token, (AnchorToken, AliasToken)) and
            node.start_mark.index <= token.start_mark.index < node.end_mark.index for token in tokens):
        # Older GUI profiles shared gain dictionaries, including aliases from
        # fault_recovery into this block. Expand references before replacing
        # their definitions so the rest of the document retains its values
        return _controller_text(_expand_yaml_aliases(text), controller)
    if original['control']['controller'] == controller:
        return text
    start = node.start_mark.index
    if node.flow_style:
        end = node.end_mark.index
        encoded = yaml.safe_dump(controller, allow_unicode=True, default_flow_style=True,
                                 sort_keys=False, width=1000000).strip()
    else:
        # Mapping end marks include comments/indentation before the next section.
        # Preserve that suffix instead of consuming it with the rewritten value
        content = [token.end_mark.index for token in tokens
                   if start <= token.start_mark.index < node.end_mark.index and
                   not isinstance(token, (BlockEndToken, DocumentEndToken, StreamEndToken))]
        end = max(content, default=start)
        lines = yaml.safe_dump(controller, allow_unicode=True, default_flow_style=False,
                               sort_keys=False, width=1000000).rstrip('\n').splitlines()
        indent = ' ' * node.start_mark.column
        encoded = lines[0] + ''.join('\n' + indent + line for line in lines[1:])
    result = text[:start] + encoded + text[end:]
    expected = deepcopy(original)
    expected['control']['controller'] = deepcopy(controller)
    try:
        parsed = yaml.safe_load(result)
    except yaml.YAMLError as error:
        raise ValueError('无法保留控制配置外的 YAML 内容，请先展开共享别名后再保存') from error
    if parsed != expected:
        raise ValueError('YAML 共享别名会影响其他配置，请先拆分为独立映射后再保存')
    return result


def _expand_yaml_aliases(text):
    """Expand shared YAML values without rewriting unrelated text or comments"""
    root = yaml.compose(text)
    nodes = {}
    def collect(node):
        if id(node) in nodes:
            return
        nodes[id(node)] = node
        if isinstance(node, MappingNode):
            for key, value in node.value:
                collect(key)
                collect(value)
        elif isinstance(node, yaml.SequenceNode):
            for value in node.value:
                collect(value)
    collect(root)
    by_start = {node.start_mark.index: node for node in nodes.values()}
    anchors = {event.anchor: by_start[event.start_mark.index]
               for event in yaml.parse(text)
               if getattr(event, 'anchor', None) and not isinstance(event, yaml.AliasEvent)}

    class InlineDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True

    def string(dumper, value):
        return dumper.represent_scalar('tag:yaml.org,2002:str', value,
                                       style='"' if '\n' in value or '\r' in value else None)
    InlineDumper.add_representer(str, string)
    replacements = []
    try:
        for token in yaml.scan(text):
            if isinstance(token, AnchorToken):
                replacements.append((token.start_mark.index, token.end_mark.index, ''))
            elif isinstance(token, AliasToken):
                value = yaml.safe_load(yaml.serialize(anchors[token.value]))
                encoded = yaml.dump(value, Dumper=InlineDumper, allow_unicode=True,
                                    default_flow_style=True, sort_keys=False, width=1000000)
                encoded = encoded.removesuffix('...\n').rstrip('\n')
                replacements.append((token.start_mark.index, token.end_mark.index, encoded))
    except RecursionError as error:
        raise ValueError('控制配置不支持循环 YAML 别名，请使用独立参数映射') from error
    for start, end, value in reversed(replacements):
        text = text[:start] + value + text[end:]
    return text


def remove_deprecated_pd_limits_text(text):
    """Remove only retired policy entries, preserving surrounding source text."""
    original = yaml.safe_load(text)
    policy = (original or {}).get('safety_policy')
    retired = {'max_kp_override', 'max_kd_override'}
    if not isinstance(policy, dict) or not retired.intersection(policy):
        return text
    node = _node_at(yaml.compose(text), ('safety_policy',))
    expected = deepcopy(original)
    for key in retired: expected['safety_policy'].pop(key, None)
    tokens = list(yaml.scan(text))
    entries = [(key, value) for key, value in node.value if key.value in retired]
    if any(isinstance(token, (AnchorToken, AliasToken)) and any(
            key.start_mark.index <= token.start_mark.index < value.end_mark.index
            for key, value in entries) for token in tokens):
        return remove_deprecated_pd_limits_text(_expand_yaml_aliases(text))
    if node.flow_style:
        encoded = yaml.safe_dump(expected['safety_policy'], allow_unicode=True,
                                 default_flow_style=True, sort_keys=False, width=1000000).strip()
        result = text[:node.start_mark.index] + encoded + text[node.end_mark.index:]
    else:
        removals = []
        for key, value in entries:
            start = text.rfind('\n', 0, key.start_mark.index) + 1
            content = [token.end_mark.index for token in tokens
                       if key.start_mark.index <= token.start_mark.index < value.end_mark.index
                       and not isinstance(token, (BlockEndToken, DocumentEndToken, StreamEndToken))]
            end = text.find('\n', max(content, default=value.end_mark.index))
            removals.append((start, len(text) if end < 0 else end + 1))
        result = text
        for start, end in reversed(removals): result = result[:start] + result[end:]
    if yaml.safe_load(result) != expected:
        raise ValueError('无法保留旧 PD 上限之外的 YAML 内容，请展开共享映射后保存')
    return result


def canonical_control_text(text):
    """Validate current fields and make shared values independently editable."""
    text = remove_deprecated_pd_limits_text(text)
    original = yaml.safe_load(text)
    canonical_core(original or {})
    controller = ((original or {}).get('control') or {}).get('controller')
    if controller is not None:
        _canonical_controller(controller)
    expanded = _expand_yaml_aliases(text)
    if yaml.safe_load(expanded) != original:
        raise ValueError('展开 YAML 别名会改变配置内容，请改用独立参数映射')
    return expanded


def four_mode_control_text(text):
    return canonical_control_text(text)


def complete_mapping_values(text, parent, values):
    """Fill missing authored fields before source-preserving leaf updates"""
    def filled(data, defaults):
        result = dict(data)
        for key, value in defaults.items():
            if key not in data: result[key] = deepcopy(value)
            elif isinstance(value, dict): result[key] = filled(data[key], value)
        return result

    def expected_at(data, path):
        if not path: return filled(data, values)
        result = dict(data)
        result[path[0]] = expected_at(data[path[0]], path[1:])
        return result

    def insert(source, path, defaults):
        for key, value in defaults.items():
            source = add_mapping_value(source, path, key, value)
            if isinstance(value, dict): source = insert(source, path + (key,), value)
        return source

    expected = expected_at(yaml.safe_load(text), parent)
    result = insert(text, parent, values)
    if yaml.safe_load(result) != expected:
        raise ValueError('YAML 共享别名会同时修改其他配置，请拆分目标配置的别名后重试')
    return result


def effective_hardware(hardware, overrides=None):
    """Resolve the connection overrides applied by HardwareLoader"""
    result = deepcopy(hardware)
    overrides = overrides or {}
    connection_keys = {'serial_port', 'baudrate', 'bus'}
    maps = [value for value in result.values() if isinstance(value, dict)]
    target = result if connection_keys & result.keys() else next((value for value in maps if connection_keys & value.keys()), maps[0] if len(maps) == 1 else None)
    if target is None:
        if any(value not in (None, '') for value in overrides.values()):
            raise ValueError('硬件配置中找不到可覆盖的通信节点')
        return result
    if overrides.get('bus'): target['bus'] = str(overrides['bus'])
    buses = result.get('buses') or {}
    physical = buses.get(target.get('bus'), target) if isinstance(buses, dict) else target
    if overrides.get('serial_port'):
        key = 'device' if 'device' in physical and 'serial_port' not in physical else 'serial_port'
        physical[key] = str(overrides['serial_port'])
    if overrides.get('baudrate') not in (None, ''): physical['baudrate'] = int(overrides['baudrate'])
    if physical is not target:
        if 'serial_port' in physical or 'device' in physical: target['serial_port'] = physical.get('serial_port', physical.get('device'))
        if 'baudrate' in physical: target['baudrate'] = physical['baudrate']
    return result
