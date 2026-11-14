"""Reference-only URDF suggestions for a new Profile, never measured qualification."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import yaml


MODEL_GAIN_REFERENCE_FIELDS = {
    'reference_only': True,
    'value_role': 'unmeasured_model_reference',
    'accuracy_thresholds_enabled': False,
}
MODEL_GAIN_REFERENCE_COMMENT = (
    '# 初次生成的 Kp/Kd 来自 URDF 几何、惯性与双对角化模型，仅供参考\n'
    '# 这些初值尚未实测；分析指标不作为精度通过标准，实际表现由示教测量与使用确认\n'
)


# Same MIT capability table used by DamiaoMotorBus::configure; these values are
# declared motor protocol ranges, not measurements of a connected actuator.
DAMIAO_MODEL_LIMITS = {
    'DM4310': (12.5, 30, 10), 'DM4310_48V': (12.5, 50, 10),
    'DM4340': (12.5, 8, 28), 'DM4340_48V': (12.5, 10, 28),
    'DM6006': (12.5, 45, 20), 'DM6248P': (12.566, 20, 120),
    'DM8006': (12.5, 45, 40), 'DM8009': (12.5, 45, 54),
    'DM10010L': (12.5, 25, 200), 'DM10010': (12.5, 20, 200),
    'DMH3510': (12.5, 28, 1), 'DMH6215': (12.5, 45, 10),
    'DMG6220': (12.5, 45, 10), 'DMJH11': (12.5, 10, 12),
}


def protocol_gain_limits(core, hardware=None, driver=None):
    """Mapped command encoding, distinct from a measured working domain."""
    damiao = (hardware or {}).get('damiao')
    if ((driver is not None and driver != 'serial_arm_hardware_damiao') or
        driver is None and not isinstance(damiao, dict)):
        return {}
    actuators = (damiao or {}).get('actuators') or {}
    result = {}
    for name in core['model']['joint_names']:
        mapping = core['calibration']['joints'][name]
        scale = float(mapping['pos_ratio']) * float(mapping['tor_ratio'])
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(f'calibration.joints.{name} 增益映射无效')
        declared = actuators.get(name, {}).get('motor_type') in DAMIAO_MODEL_LIMITS
        result[name] = {'kp_max': 500*scale, 'kd_max': 5*scale,
            'kp_quantum': 500/4095*scale, 'kd_quantum': 5/4095*scale,
            'source': 'damiao_mit_declared_motor' if declared else 'damiao_mit_driver_pending_motor',
            'motor_type': actuators.get(name, {}).get('motor_type') or None}
    return result


def model_gain_limits(core, urdf_path, hardware=None, driver=None):
    """Bound a local virtual posture using declared hardware and Joint mapping."""
    names = core['model']['joint_names']
    policy = core['safety_policy']
    joints = {joint.get('name'): joint for joint in ET.parse(urdf_path).getroot().findall('joint')}
    actuators = ((hardware or {}).get('damiao') or {}).get('actuators') or {}
    protocol = protocol_gain_limits(core, hardware, driver)
    limits, positions, hardware_complete = [], [], True
    for name in names:
        joint = joints.get(name)
        if joint is None or joint.get('type') not in ('revolute', 'continuous'):
            raise ValueError(f'model.joint_names.{name} 必须映射到有效回转关节')
        node = joint.find('limit')
        if node is None:
            raise ValueError(f'{name} 缺少 URDF limit')
        mapping = core['calibration']['joints'][name]
        ratio, torque_ratio = float(mapping['pos_ratio']), float(mapping['tor_ratio'])
        direction = float(mapping['direction'])
        if direction not in (-1, 1) or not all(math.isfinite(v) and v > 0 for v in (ratio, torque_ratio)):
            raise ValueError(f'calibration.joints.{name} 位置或力矩映射无效')
        low, high = (-math.inf, math.inf) if joint.get('type') == 'continuous' else (float(node.get('lower')), float(node.get('upper')))
        velocity, effort = float(node.get('velocity')), float(node.get('effort'))
        declared = DAMIAO_MODEL_LIMITS.get(actuators.get(name, {}).get('motor_type')) if name in protocol else None
        if declared:
            qmax, vmax, tmax = declared
            offset = float(mapping['joint_zero_offset']) - direction * float(mapping['actuator_zero_offset']) / ratio
            low, high = max(low, offset-qmax/ratio), min(high, offset+qmax/ratio)
            velocity, effort = min(velocity, vmax/ratio), min(effort, tmax*torque_ratio)
        else:
            hardware_complete = False
        margin = float(policy.get('position_margin', 0))
        low, high = low+margin, high-margin
        if low >= high:
            raise ValueError(f'{name} 模型、映射与 Safety 没有可用位置余量')
        position = float(core['shutdown']['park_pos'][name])
        if math.isfinite(low) and math.isfinite(high) and min(position-low, high-position) < .02:
            position = (low+high)/2
        available = min(position-low, high-position, .25)
        effort_budget = .2*min(effort, float(policy.get('max_effort_override', {}).get(name, effort)))
        mapped_protocol = protocol.get(name)
        if mapped_protocol:
            kp, kd = mapped_protocol['kp_max'], mapped_protocol['kd_max']
        else:
            # Preview a model-effort domain when no supported driver is selected;
            # this is not a claim about an actuator's encoding or capability.
            error = min(.008, .2*available)
            velocity_error = min(velocity*float(policy['cmd_vel_scale']),
                                 math.sqrt(2*float(policy['max_acc'][name])*error))
            kp, kd = effort_budget/(4*error), effort_budget/(4*velocity_error)
        limit = {'joint': name, 'position_margin_rad': available,
                 'max_velocity': velocity*float(policy['cmd_vel_scale']),
                 'max_acceleration': float(policy['max_acc'][name]),
                 'effort_budget': effort_budget,
                 'kp_max': kp, 'kd_max': kd}
        if any(not math.isfinite(value) or value <= 0 for key, value in limit.items() if key != 'joint'):
            raise ValueError(f'{name} 的 URDF、Safety 或硬件能力限值无效')
        limit.update({'gain_domain_source': mapped_protocol['source'] if mapped_protocol else 'model_effort_preview',
            'model_feedforward_mode': (core.get('control', {}).get('runtime') or {}).get('model_feedforward_mode', 'NONE'),
            'protocol_kp_max': mapped_protocol['kp_max'] if mapped_protocol else None,
            'protocol_kd_max': mapped_protocol['kd_max'] if mapped_protocol else None,
            'protocol_kp_quantum': mapped_protocol['kp_quantum'] if mapped_protocol else 0.,
            'protocol_kd_quantum': mapped_protocol['kd_quantum'] if mapped_protocol else 0.,
            'gain_design_peak_error_rad': math.pi/1800,
            'gain_design_settling_time_s': .7,
            'gain_design_probe_amplitude_rad': .008})
        limits.append(limit)
        positions.append(position)
    return limits, positions, hardware_complete


def _executable_gain(value, term, capability):
    """Only encoding is applied here; Safety continues to constrain execution."""
    if value == 0:
        return 0., []
    if capability is None:
        return value, []
    maximum, quantum = capability[term+'_max'], capability[term+'_quantum']
    encoded = min(maximum, max(quantum, math.floor(value/quantum+.5)*quantum))
    reasons = []
    if value > maximum: reasons.append(term+'_protocol_ceiling')
    if encoded != min(value, maximum): reasons.append(term+'_protocol_quantization')
    return encoded, reasons


def initialize_profile_gains(inspector, core_path, core, urdf_path, hardware, driver=None, *, apply_generated=False):
    """Consume the shared native URDF design; only new generated configs opt in.

    Existing authored/user parameters remain untouched by read-only inspection.
    Artificial motion/effort limits, old gains and park/taught poses never enter
    the neutral URDF design. Hardware encoding is recorded as a separate step.
    """
    probe = inspector.model_probe()
    if not probe:
        raise ValueError('模型增益初始化需要 Core Model Probe，请先编译安装模型检查组件')
    names = core['model']['joint_names']
    model = dict(core['model']);model['urdf_path'] = str(Path(urdf_path).resolve())
    # A calibrated correction is not the sole URDF geometry/inertia prior.
    model.pop('gravity_correction_path', None)
    model['gravity_scale'] = {name: 1. for name in names}
    with tempfile.TemporaryDirectory(prefix='serialarm-gain-model-') as temporary:
        model_config = Path(temporary) / 'model.yaml'
        model_config.write_text(yaml.safe_dump({'model': model}, sort_keys=False), encoding='utf-8')
        completed = subprocess.run([str(probe), '--config', str(model_config)],
            capture_output=True, text=True, timeout=12, check=False)
    try:
        payload = json.loads(completed.stdout.strip())
    except json.JSONDecodeError:
        raise ValueError('模型增益初始化未返回有效结果，请编译安装当前 Core Model Probe')
    design = payload.get('nominal_gain_design') or {}
    if completed.returncode or payload.get('ok') is not True or design.get('valid') is not True:
        raise ValueError('模型增益初始化失败，请修正 URDF 几何、惯量或受控关节映射：' +
            str(payload.get('error') or '缺少共享 URDF 名义设计结果，请编译安装当前 Core Model Probe'))
    if design.get('joint_names') != names:
        raise ValueError('URDF 名义增益设计的受控关节顺序不匹配')
    fields = ('unconstrained_kp', 'unconstrained_kd', 'inertia_basis')
    if any(not isinstance(design.get(key), list) or len(design[key]) != len(names) or
           any(isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0
               for value in design[key]) for key in fields):
        raise ValueError('URDF 名义增益设计的惯量或参数无效，请修正模型')
    matrix = design.get('mass_matrix_basis')
    if not isinstance(matrix,list) or len(matrix)!=len(names) or any(
        not isinstance(row,list) or len(row)!=len(names) or any(
            isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) for value in row)
        for row in matrix):
        raise ValueError('URDF 名义增益设计缺少有效质量矩阵，请修正模型')
    protocol = protocol_gain_limits(core, hardware, driver)
    axes = []
    for index,name in enumerate(names):
        capability = protocol.get(name)
        kp,p_reasons = _executable_gain(design['unconstrained_kp'][index], 'kp', capability)
        kd,d_reasons = _executable_gain(design['unconstrained_kd'][index], 'kd', capability)
        axes.append({**MODEL_GAIN_REFERENCE_FIELDS, 'joint':name, 'kp':kp, 'kd':kd,
            'unconstrained_kp':design['unconstrained_kp'][index], 'unconstrained_kd':design['unconstrained_kd'][index],
            'inertia_min':matrix[index][index], 'inertia_max':matrix[index][index],
            'inertia_basis':design['inertia_basis'][index],
            'frequency_hz':design.get('design_frequency_hz'),
            'frequency_role':'engineering_design_lower_bound_before_encoding_not_axis_bandwidth',
            'working_kp_max':capability['kp_max'] if capability else None,
            'working_kd_max':capability['kd_max'] if capability else None,
            'constraints':p_reasons+d_reasons})
    purpose_suggestions = {}
    raw_purposes = design.get('purpose_nominal_gain_designs') or {}
    for role in ('rigid', 'compliant', 'drag'):
        raw = raw_purposes.get(role) or {}
        if any(not isinstance(raw.get(term),list) or len(raw[term])!=len(names) or any(
            isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<0
            for value in raw[term]) for term in ('unconstrained_kp','unconstrained_kd')):
            raise ValueError('Core Model Probe 缺少有效的分用途 URDF 名义设计，请编译安装当前模型检查组件')
        purpose_axes = []
        for index,name in enumerate(names):
            kp,p_reasons = _executable_gain(raw['unconstrained_kp'][index],'kp',protocol.get(name))
            kd,d_reasons = _executable_gain(raw['unconstrained_kd'][index],'kd',protocol.get(name))
            purpose_axes.append({**MODEL_GAIN_REFERENCE_FIELDS, 'joint':name,'kp':kp,'kd':kd,
                'unconstrained_kp':raw['unconstrained_kp'][index],'unconstrained_kd':raw['unconstrained_kd'][index],
                'constraints':p_reasons+d_reasons})
        purpose_suggestions[role] = {**raw, **MODEL_GAIN_REFERENCE_FIELDS,
            'axes':purpose_axes,'measured':False,'validated':False,
            'encoded_modal_diagnostics':{'computed':False,'measured':False,
                'gain_basis':'encoded_candidate' if protocol else 'unencoded_candidate',
                'reason':'encoded_modal_diagnostics_not_computed_in_profile_import',
                'raw_modal_diagnostics_apply_only_to_unconstrained_gains':True}}
    result = {**design, **MODEL_GAIN_REFERENCE_FIELDS,
        'axes':axes, 'purpose_suggestions':purpose_suggestions, 'measured':False, 'validated':False,
        'scope':'unmeasured_urdf_neutral_nominal_design', 'demand_source':'explicit_generic_nominal_response_design',
        'hardware_constraints':'declared_protocol_encoding' if protocol else 'pending_hardware_selection',
        'configuration_applied':False, 'runtime_baseline_unchanged':True,
        'suggestion_applicable':bool(protocol), 'protocol_gain':protocol,
        'encoded_modal_diagnostics':{'computed':False,'measured':False,
            'gain_basis':'encoded_candidate' if protocol else 'unencoded_candidate',
            'reason':'encoded_modal_diagnostics_not_computed_in_profile_import',
            'raw_modal_diagnostics_apply_only_to_unconstrained_gains':True},
        'urdf_path':str(Path(urdf_path).resolve()), 'positions':[0.]*len(names),
        'model_physical_prior':{'source':'urdf_geometry_mass_com_inertia', 'scope':'neutral_mass_matrix_only',
            'actuator_response_measured':False,
            'excluded_fields':['old_pd_gains','taught_path','park_position','urdf_position_limit',
                'urdf_velocity_limit','urdf_effort_limit','policy_velocity','policy_acceleration','policy_effort']},
        'task_design_targets':{'source':'explicit_generic_response_design_not_urdf_measurement',
            'frequency_hz':design.get('design_frequency_hz'), 'damping_ratio':design.get('damping_ratio'),
            'frequency_role':'lower_bound_on_undamped_ideal_model_modes_before_protocol_encoding',
            'purposes':{role:{'frequency_hz':value.get('design_frequency_hz'),
                'damping_ratio':value.get('damping_ratio'),'scope':value.get('scope')}
                for role,value in purpose_suggestions.items()}},
        'execution_conditions':{'model_feedforward_mode':(core.get('control',{}).get('runtime') or {}).get('model_feedforward_mode','NONE')}}
    if not protocol:
        result['pending_reason'] = '模型初始增益已生成，仅供参考；选择硬件后再核对协议编码范围，当前未测量执行器响应'
    if apply_generated:
        controller = core['control']['controller']
        generated = {}
        for group in ('rigid', 'compliant', 'drag'):
            purpose_axes = purpose_suggestions[group]['axes']
            pair = {term:{axis['joint']:axis[term] for axis in purpose_axes} for term in ('kp','kd')}
            controller[group].update(pair); generated[group] = pair
        recovery = core['safety_policy']['fault_recovery']['compliant_recovery']
        recovery.update({term:dict(generated['compliant'][term]) for term in ('kp','kd')})
        result.update(configuration_applied=True, configuration_initialized=True,
            runtime_baseline_unchanged=False, generated_controller=generated)
    return result
