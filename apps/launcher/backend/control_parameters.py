"""Canonical control parameters and source-preserving YAML additions."""
from copy import deepcopy
import math

import yaml
from yaml.nodes import MappingNode, ScalarNode
from yaml.tokens import AnchorToken, AliasToken, BlockEndToken, DocumentEndToken, StreamEndToken


_CONTROLLER_ROLES = ('rigid_hold', 'rigid_tracking', 'compliant_hold',
                     'compliant_tracking', 'compliant_drag', 'servo_tracking')
_CONTROLLER_GROUPS = ('rigid', 'compliant', 'drag')


def _canonical_controller(controller):
    """Read four user purposes while retaining the six execution roles internally."""
    if not isinstance(controller, dict):
        raise ValueError('control.controller 必须为映射')
    result = deepcopy(controller)
    modern = any(key in controller for key in _CONTROLLER_GROUPS)
    if modern:
        for key in (*_CONTROLLER_GROUPS, 'servo_tracking'):
            if key not in controller:
                raise ValueError('四用途增益配置缺少 control.controller.' + key)
        for group in ('rigid', 'compliant'):
            fields = controller[group]
            if not isinstance(fields, dict) or set(fields) != {'hold', 'tracking'}:
                raise ValueError('control.controller.' + group + ' 必须完整包含 hold 和 tracking，不支持其他分组字段')
            for task in ('hold', 'tracking'):
                key = group + '_' + task
                gains = fields[task]
                if not isinstance(gains, dict) or not {'kp', 'kd'} <= gains.keys():
                    raise ValueError('control.controller.' + group + '.' + task + ' 必须包含 kp 和 kd')
                if key in controller and controller[key] != gains:
                    raise ValueError('control.controller.' + group + '.' + task + ' 与旧 ' + key + ' 参数冲突')
                result[key] = deepcopy(gains)
            result.pop(group)
        drag = controller['drag']
        if not isinstance(drag, dict) or not {'kp', 'kd'} <= drag.keys():
            raise ValueError('control.controller.drag 必须包含 kp 和 kd')
        if 'compliant_drag' in controller and controller['compliant_drag'] != drag:
            raise ValueError('control.controller.drag 与旧 compliant_drag 参数冲突')
        result['compliant_drag'] = deepcopy(drag)
        result.pop('drag')
        servo = controller['servo_tracking']
        if not isinstance(servo, dict) or not {'kp', 'kd'} <= servo.keys():
            raise ValueError('control.controller.servo_tracking 必须包含 kp 和 kd')
    if 'identification_tracking' in result:
        legacy = result.pop('identification_tracking')
        if 'servo_tracking' in result and result['servo_tracking'] != legacy:
            raise ValueError('control.controller.servo_tracking 与旧 identification_tracking 参数冲突，请保留一份伺服跟踪配置')
        result.setdefault('servo_tracking', legacy)
    if 'servo_tracking' not in result and 'rigid_tracking' in result:
        result['servo_tracking'] = deepcopy(result['rigid_tracking'])
    return result


def four_mode_core(core):
    """Write four gain purposes without merging their separate task parameters."""
    result = canonical_core(core)
    controller = (result.get('control') or {}).get('controller')
    if not isinstance(controller, dict):
        raise ValueError('缺少 control.controller 增益配置')
    for key in _CONTROLLER_ROLES:
        if key not in controller:
            raise ValueError('四用途增益配置缺少任务参数 control.controller.' + key)
        if not isinstance(controller[key], dict) or not {'kp', 'kd'} <= controller[key].keys():
            raise ValueError('control.controller.' + key + ' 必须包含 kp 和 kd')
    grouped = {}
    if 'allow_full_cmd' in controller:
        grouped['allow_full_cmd'] = controller['allow_full_cmd']
    for group in ('rigid', 'compliant'):
        grouped[group] = {task: deepcopy(controller[group + '_' + task]) for task in ('hold', 'tracking')}
    grouped['drag'] = deepcopy(controller['compliant_drag'])
    grouped['servo_tracking'] = deepcopy(controller['servo_tracking'])
    grouped.update({key: deepcopy(value) for key, value in controller.items()
                    if key not in _CONTROLLER_ROLES and key != 'allow_full_cmd'})
    result['control']['controller'] = grouped
    return result


def canonical_core(core):
    result = deepcopy(core)
    names = (result.get('model') or {}).get('joint_names') or []
    if names and 'admittance' not in (result.get('capability') or {}):
        zeros = {name: 0.0 for name in names}
        result.setdefault('capability', {})['admittance'] = {
            'enabled': False, 'reference_behavior': 'NATIVE_IMPEDANCE',
            'joint_enabled': {name: True for name in names},
            'observer': {'mode': 'MOMENTUM', 'momentum_gain': {name: 25.0 for name in names}},
            'calibration': {'torque_bias': deepcopy(zeros), 'torque_threshold': {name: .05 for name in names},
                'friction': {'enabled': False, 'velocity_transition': .03,
                            **{term: deepcopy(zeros) for term in ('positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous')}}},
            'controller': {key: {name: value for name in names} for key, value in
                           (('mass', 1.0), ('damping', 5.5), ('stiffness', 15.0), ('max_delta_q', .5), ('max_delta_q_dot', 1.0))}}
    admittance = (result.get('capability') or {}).get('admittance')
    if isinstance(admittance, dict):
        admittance.setdefault('reference_behavior', 'MODEL_ADMITTANCE' if admittance.get('enabled') is True else 'NATIVE_IMPEDANCE')
    runtime = (result.get('control') or {}).get('runtime')
    if isinstance(runtime, dict):
        runtime.setdefault('tracking_impedance_mode', 'RIGID_TRACKING')
        runtime.setdefault('tracking_gain_profile', 'COMPLIANT' if runtime['tracking_impedance_mode'] == 'COMPLIANT_TRACKING' else 'RIGID')
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


GAIN_PURPOSE_KEYS = ('rigid_tracking', 'compliant_tracking', 'compliant_drag', 'servo_tracking')

def selected_gain_purposes(gain_purposes):
    if gain_purposes is None: return None
    aliases = {'RIGID':'rigid_tracking', 'COMPLIANT':'compliant_tracking', 'DRAG':'compliant_drag', 'SERVO':'servo_tracking', 'SERVO_TRACKING':'servo_tracking'}
    aliases.update({key.upper():key for key in GAIN_PURPOSE_KEYS})
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
    """Measurement completion is separate from meeting reference metrics."""
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
        return (validation.get('completed') is True and isinstance(axes, list) and
                len(axes) == joint_count and all(isinstance(axis, dict) and
                _gain_number(axis.get('samples')) and axis['samples'] > 0 and
                all(_gain_number(axis.get(key)) and axis[key] >= 0 for key in
                    ('initial_velocity_rad_s', 'peak_velocity_rad_s', 'final_velocity_rad_s', 'decay_s'))
                for axis in axes))
    return False


def gain_purpose_available(candidate, purpose, fields):
    if not isinstance(fields, dict) or fields.get('available') is False or fields.get('error') in ('not_started', 'measurement_in_progress'):
        return False
    names = candidate.get('joint_names')
    count = len(names) if isinstance(names, list) else 0
    def gains(value):
        return isinstance(value, dict) and all(isinstance(value.get(term), list) and
            len(value[term]) == count and all(_gain_number(v) and v >= 0 for v in value[term])
            for term in ('kp', 'kd'))
    if count == 0 or not gains(fields):
        return False
    # Existing qualified local-probe records retain their recorded scope.
    # Failed old records need complete independent trajectory measurements.
    if candidate.get('validation_scope') != 'independent_demonstration_trajectory':
        return fields.get('validated') is True
    if purpose == 'compliant_drag':
        return all(v == 0 for v in fields['kp']) and gain_measurement_available(fields.get('drag_validation'), count, 'drag')
    if not gain_measurement_available(fields.get('native_validation'), count, 'tracking'):
        return False
    if purpose == 'servo_tracking':
        admittance = fields.get('admittance_validation') or {}
        native_only = fields.get('admittance_required') is False and admittance.get('required') is False and admittance.get('status') == 'not_requested'
        return native_only or gain_measurement_available(admittance, count, 'tracking')
    if purpose not in ('rigid_tracking', 'compliant_tracking') or not gains(fields.get('hold_gains')):
        return False
    verification = ('hold_compensation_verification' if purpose == 'rigid_tracking' and
                    (fields.get('hold_compensation') or {}).get('enabled') is True else 'hold_verification')
    if not gain_measurement_available(fields.get(verification), count, 'hold'):
        return False
    if purpose == 'compliant_tracking':
        return (_gain_number(fields.get('tested_speed_fraction')) and fields['tested_speed_fraction'] > 0 and
                all(isinstance(fields.get(key), list) and len(fields[key]) == count and
                    all(_gain_number(v) and v >= 0 for v in fields[key])
                    for key in ('rigid_reference_kp', 'rigid_reference_kd')))
    return True


def gain_selection_report(candidate, groups):
    warnings = [value for value in candidate.get('warnings', []) if isinstance(value, str)]
    for purpose, fields in groups.items():
        warnings.extend(value for value in fields.get('warnings', []) if isinstance(value, str))
        if fields.get('validated') is not True:
            warnings.append(purpose + ' 未达参考指标，保留实测结果，可应用或导出')
    rigid, soft = groups.get('rigid_tracking'), groups.get('compliant_tracking')
    if soft and soft.get('compliant_separation_validated') is not True:
        warnings.append('柔性增益未达刚柔分离参考指标')
    if rigid and soft and any(soft.get('rigid_reference_' + term) != rigid.get(term) for term in ('kp', 'kd')):
        warnings.append('柔性测量时的刚性参考与本次采用的刚性增益不同')
    return {'available': bool(groups), 'validated': bool(groups) and all(fields.get('validated') is True for fields in groups.values()),
            'warnings': list(dict.fromkeys(warnings))}


def deployment_gain_groups(candidate, gain_purposes=None):
    category = candidate.get('category')
    if category == 'identification_tracking':
        raise ValueError('旧辨识增益记录未包含导纳联合验收，请重新运行伺服跟踪标定')
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
        if purpose == 'servo_tracking':
            native = fields.get('native_validation') or {}
            admittance = fields.get('admittance_validation') or {}
            native_only = (candidate.get('validation_scope') == 'independent_demonstration_trajectory'
                           and fields.get('admittance_required') is False
                           and admittance.get('required') is False
                           and admittance.get('status') == 'not_requested')
            if native.get('validated') is not True or (not native_only and admittance.get('validated') is not True):
                raise ValueError('伺服跟踪候选缺少原生跟踪验收，或未通过本次要求的导纳附加验收')
        elif purpose == 'compliant_drag':
            if (fields.get('drag_validation') or {}).get('validated') is not True:
                raise ValueError('拖拽候选缺少独立稳定性验收')
        elif purpose == 'rigid_tracking':
            verification = ('hold_compensation_verification'
                            if (fields.get('hold_compensation') or {}).get('enabled') is True
                            else 'hold_verification')
            if (fields.get(verification) or {}).get('validated') is not True:
                raise ValueError('刚性候选缺少保持或补偿保持验收')
        elif purpose == 'compliant_tracking':
            if (fields.get('hold_verification') or {}).get('validated') is not True:
                raise ValueError('柔性候选缺少保持验收')
        else:
            raise ValueError('未知增益用途: ' + str(purpose))
        if purpose in ('rigid_tracking', 'compliant_tracking') and not isinstance(fields.get('hold_gains'), dict):
            raise ValueError('保持验收缺少对应的独立保持增益')
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
    """Add one missing member without rewriting existing comments or values."""
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
    """Replace only the controller value and verify the complete parsed document."""
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
        # their definitions so the rest of the document retains its values.
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
        # Preserve that suffix instead of consuming it with the rewritten value.
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
    """Expand shared YAML values without rewriting unrelated text or comments."""
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


def canonical_control_text(text, servo=None):
    """Expand the four-purpose layout into the internal paths used by patchers."""
    original = yaml.safe_load(text)
    controller = ((original or {}).get('control') or {}).get('controller')
    normalized = _canonical_controller(controller)
    if servo is not None and 'servo_tracking' not in controller and 'identification_tracking' not in controller:
        normalized['servo_tracking'] = deepcopy(servo)
    return _controller_text(text, normalized)


def four_mode_control_text(text):
    """Write only the controller block as four purposes; keep other text intact."""
    original = yaml.safe_load(text)
    controller = four_mode_core(original)['control']['controller']
    return _controller_text(text, controller)


def complete_mapping_values(text, parent, values):
    """Fill missing authored fields before source-preserving leaf updates."""
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
    """Resolve the connection overrides applied by HardwareLoader."""
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
