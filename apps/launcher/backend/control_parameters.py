"""Canonical control parameters and source-preserving YAML additions."""
from copy import deepcopy

import yaml
from yaml.nodes import MappingNode, ScalarNode


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
    legacy = controller.pop('identification_tracking', None)
    current = controller.get('servo_tracking')
    if legacy is not None and current is not None and legacy != current:
        raise ValueError('control.controller.servo_tracking 与旧 identification_tracking 参数冲突，请保留一份伺服跟踪配置')
    if current is None:
        source = legacy if legacy is not None else controller.get('rigid_tracking')
        if source is not None:
            controller['servo_tracking'] = deepcopy(source)
    return result


def deployment_gain_groups(candidate):
    category = candidate.get('category')
    if category == 'identification_tracking':
        raise ValueError('旧辨识增益记录未包含导纳联合验收，请重新运行伺服跟踪标定')
    groups = candidate.get('targets') if category == 'all' else {category: candidate}
    if not isinstance(groups, dict) or not groups:
        raise ValueError('增益候选缺少已验证的用途')
    if category == 'all' and set(groups) != {'rigid_tracking', 'compliant_tracking', 'compliant_drag', 'servo_tracking'}:
        raise ValueError('统一增益候选必须包含四类用途的独立验收')
    for purpose, fields in groups.items():
        if not isinstance(fields, dict) or fields.get('validated') is not True:
            raise ValueError('增益用途未通过验证: ' + str(purpose))
        if purpose == 'servo_tracking':
            if any((fields.get(key) or {}).get('validated') is not True
                   for key in ('native_validation', 'admittance_validation')):
                raise ValueError('伺服跟踪候选必须通过辨识跟踪和导纳联合验收')
        elif purpose == 'compliant_drag':
            if (fields.get('drag_validation') or {}).get('validated') is not True:
                raise ValueError('拖拽候选缺少独立稳定性验收')
        elif purpose == 'rigid_tracking':
            if not any((fields.get(key) or {}).get('validated') is True
                       for key in ('hold_verification', 'hold_compensation_verification')):
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


def canonical_control_text(text, servo=None):
    original = yaml.safe_load(text)
    controller = ((original or {}).get('control') or {}).get('controller') or {}
    canonical_core(original)  # Reject conflicting aliases before changing text
    if 'identification_tracking' in controller:
        parent = _node_at(yaml.compose(text), ('control', 'controller'))
        name, value = next((name, value) for name, value in parent.value if name.value == 'identification_tracking')
        if 'servo_tracking' not in controller:
            text = text[:name.start_mark.index] + 'servo_tracking' + text[name.end_mark.index:]
        else:
            index = next(i for i, (key, _) in enumerate(parent.value) if key.value == 'identification_tracking')
            if parent.flow_style:
                if index + 1 < len(parent.value):
                    start, end = name.start_mark.index, parent.value[index + 1][0].start_mark.index
                else:
                    start, end = parent.value[index - 1][1].end_mark.index, value.end_mark.index
            else:
                start = text.rfind('\n', 0, name.start_mark.index) + 1
                end = value.end_mark.index
                line = text.rfind('\n', 0, end) + 1
                if not text[line:end].strip(): end = line
                elif text[end:end + 1] == '\n': end += 1
            text = text[:start] + text[end:]
    if servo is not None:
        text = add_mapping_value(text, ('control', 'controller'), 'servo_tracking', servo)
    return text


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
