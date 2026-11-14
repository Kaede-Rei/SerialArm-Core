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
    if set(controller) != {'allow_full_cmd', *_CONTROLLER_ROLES}:
        raise ValueError('control.controller 必须包含 allow_full_cmd、rigid、compliant 和 drag')
    for key in _CONTROLLER_ROLES:
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
    admittance = (result.get('capability') or {}).get('admittance')
    if isinstance(admittance, dict):
        unknown = set(admittance) - {'joint_enabled', 'observer', 'calibration', 'controller'}
        if unknown:
            raise ValueError('capability.admittance 含已移除或未知参数: ' + ', '.join(sorted(unknown)))
        observer = admittance.get('observer')
        if isinstance(observer, dict) and set(observer) - {'momentum_gain'}:
            raise ValueError('capability.admittance.observer 仅支持 momentum_gain')
    if names and 'admittance' not in (result.get('capability') or {}):
        zeros = {name: 0.0 for name in names}
        result.setdefault('capability', {})['admittance'] = {
            'joint_enabled': {name: True for name in names},
            'observer': {'momentum_gain': {name: 25.0 for name in names}},
            'calibration': {'torque_bias': deepcopy(zeros), 'torque_threshold': {name: .05 for name in names},
                'friction': {'enabled': False, 'velocity_transition': .03,
                            **{term: deepcopy(zeros) for term in ('positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous')}}},
            'controller': {key: {name: value for name in names} for key, value in
                           (('mass', 1.0), ('damping', 5.5), ('stiffness', 15.0), ('max_delta_q', .5), ('max_delta_q_dot', 1.0))}}
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
        hold.setdefault('velocity_threshold', .02)
        hold.setdefault('error_threshold', .03)
    controller = (result.get('control') or {}).get('controller')
    if not isinstance(controller, dict):
        return result
    result['control']['controller'] = _canonical_controller(controller)
    return result


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
    _canonical_controller(((original or {}).get('control') or {}).get('controller'))
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
