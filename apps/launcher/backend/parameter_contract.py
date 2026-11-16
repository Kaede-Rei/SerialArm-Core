"""Numeric editor constraints shared by field help and Profile save validation."""
import math


def field_constraints(core, hardware=None, entry=None, joint_limits=None):
    names = (core.get('model') or {}).get('joint_names') or []
    result = {}

    def add(path, minimum=None, maximum=None, *, positive=False, integer=False, choices=None, hint=''):
        value = {'hint': hint}
        if minimum is not None: value['min'] = minimum
        if maximum is not None: value['max'] = maximum
        if positive: value.update(min=0, exclusive_min=True)
        if integer: value['integer'] = True
        if choices is not None: value['choices'] = choices
        result[path] = value

    def axes(path, **options):
        for name in names: add('core.' + path + '.' + name, **options)

    add('core.control.runtime.ctrl_frequency_hz', positive=True,
        hint='大于 0；周期 = 1 ÷ 频率，实际频率受总线与调度能力约束')
    add('core.control.runtime.joint_acc_filter_alpha', 0, 1,
        hint='范围 [0, 1]；0 停用有效加速度估计，1 不平滑')
    axes('model.gravity_scale', minimum=0, maximum=2,
        hint='范围 [0, 2]；0 关闭该轴重力项，1 使用原模型')
    for name in names:
        base = 'core.calibration.joints.' + name + '.'
        add(base + 'direction', choices=[-1, 1], integer=True, hint='仅允许 +1 或 −1')
        for key in ('pos_ratio', 'tor_ratio'):
            add(base + key, positive=True, hint='大于 0；方向由 direction 单独指定')
        for key in ('joint_zero_offset', 'actuator_zero_offset'):
            add(base + key, hint='任意有限数值；允许零值与负值')
    from gain_initialization import protocol_gain_limits
    try:
        protocol = protocol_gain_limits(core, hardware, ((entry or {}).get('hardware') or {}).get('plugin', ''))
    except (KeyError, TypeError, ValueError):
        protocol = {}
    for mode in ('rigid', 'compliant', 'drag'):
        for key in ('kp', 'kd'):
            for name in names:
                maximum = 0 if mode == 'drag' and key == 'kp' else protocol.get(name, {}).get(key + '_max')
                add(f'core.control.controller.{mode}.{key}.{name}', 0, maximum,
                    hint='拖拽刚度固定为 0' if mode == 'drag' and key == 'kp' else
                    '非负；MIT 刚性使能要求 Kp/Kd 为正值，上限由映射后的硬件能力确定')
                if maximum is not None and maximum > 0:
                    mapping = core['calibration']['joints'][name]
                    result[f'core.control.controller.{mode}.{key}.{name}'].update(
                        max_product_from=[f'core.calibration.joints.{name}.{term}' for term in ('pos_ratio', 'tor_ratio')],
                        max_factor=maximum / (mapping['pos_ratio'] * mapping['tor_ratio']))
    for key in ('integral_gain', 'max_torque', 'max_rate', 'contact_threshold'):
        axes('control.hold_compensation.' + key, minimum=0, hint='大于等于 0')
    for key in ('velocity_threshold', 'error_threshold'):
        axes('control.hold_compensation.' + key, positive=True, hint='大于 0；按各轴 SI 单位设置')
    axes('safety_policy.position_margin', minimum=0, hint='非负；不能压空该轴有效行程')
    for key in ('max_acc', 'max_tracking_error', 'max_effort_override'):
        axes('safety_policy.' + key, positive=True, hint='大于 0；同时受模型与硬件有效边界约束')
    add('core.safety_policy.cmd_vel_scale', positive=True, maximum=1,
        hint='范围 (0, 1]；0.5 = 有效速度上限的 50%')
    command = (core.get('safety_policy') or {}).get('cmd_vel_scale')
    add('core.safety_policy.state_vel_scale', minimum=command if _number(command) and command > 0 else 0,
        hint='不小于命令速度上限比例；只改变超速判定，不缩放反馈')
    result['core.safety_policy.state_vel_scale']['min_from'] = 'core.safety_policy.cmd_vel_scale'
    frequency = ((core.get('control') or {}).get('runtime') or {}).get('ctrl_frequency_hz')
    add('core.safety_policy.max_dt_s', minimum=1/frequency if _number(frequency) and frequency > 0 else 0,
        hint='不小于控制周期 1 ÷ 控制频率')
    result['core.safety_policy.max_dt_s']['min_inverse_from'] = 'core.control.runtime.ctrl_frequency_hz'
    for key in ('state_timeout_s', 'cmd_timeout_s'):
        add('core.safety_policy.' + key, positive=True, hint='大于 0')
    add('core.safety_policy.fault_recovery.recovery_timeout_s', positive=True, hint='大于 0')
    axes('safety_policy.fault_recovery.velocity_tolerance', positive=True,
        hint='大于 0；用于故障恢复的逐轴静止判定')
    for path in ('MIT', 'CSP'):
        base = 'shutdown.paths.' + path + '.'
        axes(base + 'position_tolerance', positive=True, hint='大于 0；每轴都满足才算到位')
        axes(base + 'velocity_tolerance', positive=True, hint='大于 0；用于停放稳定判定')
        add('core.' + base + 'speed_scale', positive=True, maximum=1,
            hint='范围 (0, 1]；相对有效命令速度上限')
        add('core.' + base + 'settle_time_s', 0, hint='大于等于 0；0 表示不要求持续稳定段')
        add('core.' + base + 'timeout_s', positive=True, hint='大于 0；覆盖整段停放与到位稳定时间')
        for name in names:
            limit = (joint_limits or {}).get(name) or {}
            margins = (core.get('safety_policy') or {}).get('position_margin') or {}
            margin = margins.get(name, 0) if isinstance(margins, dict) else None
            low, high = limit.get('lower'), limit.get('upper')
            bounded = limit.get('type') != 'continuous' and all(_number(v) for v in (low, high, margin))
            field = 'core.' + base + 'park_pos.' + name
            add(field, low + margin if bounded else None, high - margin if bounded else None,
                hint='默认零位；可修改，须在该轴有效位置范围内')
            if bounded:
                result[field].update(min_base=low, max_base=high,
                    min_add_from='core.safety_policy.position_margin.' + name,
                    max_subtract_from='core.safety_policy.position_margin.' + name)
    axes('capability.admittance.observer.momentum_gain', positive=True,
        hint='大于 0；单位 s⁻¹，决定观测收敛速度')
    axes('capability.admittance.calibration.torque_bias', hint='有符号残差零偏；允许零值与负值')
    axes('capability.admittance.calibration.torque_threshold', minimum=0, hint='非负；0 关闭连续死区')
    axes('capability.admittance.calibration.friction.velocity_transition', positive=True,
        hint='大于 0；摩擦补偿达到完整幅度的逐轴速度')
    for key in ('positive_coulomb', 'negative_coulomb', 'positive_viscous', 'negative_viscous'):
        axes('capability.admittance.calibration.friction.' + key, hint='有符号残差模型系数；允许零值与负值')
    flags = ((core.get('capability') or {}).get('admittance') or {}).get('joint_enabled') or {}
    for key in ('mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot'):
        for name in names:
            positive = key not in ('damping', 'stiffness') or flags.get(name) is True
            add(f'core.capability.admittance.controller.{key}.{name}', positive=positive,
                minimum=0, hint='参与轴要求大于 0' if key in ('damping', 'stiffness') else '大于 0')
            if key in ('damping', 'stiffness'):
                result[f'core.capability.admittance.controller.{key}.{name}']['positive_if'] = (
                    'core.capability.admittance.joint_enabled.' + name)
    axes('capability.admittance.validation.velocity_tolerance', positive=True,
        hint='大于 0；仅用于释放回位响应的稳定验收')
    damiao = (hardware or {}).get('damiao') or {}
    for key in ('activation_retries', 'startup_read_cycles'):
        add('hardware.damiao.' + key, 1, integer=True,
            hint='正整数')
    add('hardware.damiao.feedback_timeout_s', positive=True, hint='大于 0')
    for name in names:
        for key in ('motor_id', 'master_id'):
            add(f'hardware.damiao.actuators.{name}.{key}', 1 if key == 'motor_id' else 0, 0x7ff,
                integer=True, hint='整数；反馈 ID 还须满足当前驱动的唯一性约束')
    for bus in (hardware or {}).get('buses') or {}:
        add('hardware.buses.' + bus + '.baudrate', positive=True, integer=True, hint='正整数')
    return result


def _number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def parking_plan(core, path=None):
    path = path or ((core.get('control') or {}).get('runtime') or {}).get('control_path', 'MIT')
    return ((core.get('shutdown') or {}).get('paths') or {}).get(path) or {}


def parking_complete(core, path=None):
    names = (core.get('model') or {}).get('joint_names') or []
    plan = parking_plan(core, path)
    if plan.get('strategy', 'RIGID') not in ('RIGID', 'COMPLIANT_THEN_RIGID'): return False
    if not names: return False
    for key in ('park_pos', 'position_tolerance', 'velocity_tolerance'):
        values = plan.get(key)
        if not isinstance(values, dict) or set(values) != set(names): return False
        if any(not _number(v) or key != 'park_pos' and v <= 0 for v in values.values()): return False
    for key in ('speed_scale', 'settle_time_s', 'timeout_s'):
        value = plan.get(key)
        if not _number(value) or (value < 0 if key == 'settle_time_s' else value <= 0): return False
    return plan['speed_scale'] <= 1


def constraint_errors(core, hardware=None, entry=None, joint_limits=None):
    documents = {'core': core, 'hardware': hardware or {}}
    for path, rule in field_constraints(core, hardware, entry, joint_limits).items():
        value = documents
        for key in path.split('.'):
            if not isinstance(value, dict) or key not in value: break
            value = value[key]
        else:
            reason = None
            rule = dict(rule)
            parts = path.split('.')
            if parts[:3] == ['core', 'shutdown', 'paths'] and len(parts) > 4 and parts[4] == 'park_pos' and parking_plan(core, parts[3]).get('enabled') is not True:
                rule.pop('min', None)
                rule.pop('max', None)
            if not _number(value): reason = '必须是有限数值'
            elif rule.get('integer') and int(value) != value: reason = '必须是整数'
            elif 'choices' in rule and value not in rule['choices']: reason = '只能为 ' + '、'.join(map(str, rule['choices']))
            elif 'min' in rule and (value <= rule['min'] if rule.get('exclusive_min') else value < rule['min']):
                reason = ('必须大于 ' if rule.get('exclusive_min') else '不能小于 ') + str(rule['min'])
            elif 'max' in rule and value > rule['max']: reason = '不能大于 ' + str(rule['max'])
            if reason: yield path, reason
