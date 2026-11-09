"""Read-only model suggestions for a newly created Profile, never qualification."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import yaml


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


def initialize_profile_gains(inspector, core_path, core, urdf_path, hardware, driver=None):
    probe = inspector.model_probe()
    if not probe:
        raise ValueError('模型增益初始化需要 Core Model Probe，请先编译安装模型检查组件')
    limits, positions, complete = model_gain_limits(core, urdf_path, hardware, driver)
    # Parameter edits are not on disk yet. Probe the actual edited model without
    # replacing the user's Profile or resolving resources against a temp folder.
    model = dict(core['model']);model['urdf_path'] = str(Path(urdf_path).resolve())
    if model.get('gravity_correction_path'):
        model['gravity_correction_path'] = str((Path(core_path).parent / model['gravity_correction_path']).resolve())
    with tempfile.TemporaryDirectory(prefix='serialarm-gain-model-') as temporary:
        model_config = Path(temporary) / 'model.yaml'
        model_config.write_text(yaml.safe_dump({'model': model}, sort_keys=False), encoding='utf-8')
        completed = subprocess.run([str(probe), '--config', str(model_config), '--positions',
            ','.join(format(value, '.17g') for value in positions), '--gain-limits-json',
            json.dumps(limits, separators=(',', ':'))], capture_output=True, text=True, timeout=12, check=False)
    try:
        payload = json.loads(completed.stdout.strip())
    except json.JSONDecodeError:
        raise ValueError('模型增益初始化未返回有效结果，请编译安装当前 Core Model Probe')
    result = payload.get('gain_initialization') or {}
    axes = result.get('axes')
    names = core['model']['joint_names']
    if completed.returncode or payload.get('ok') is not True or result.get('valid') is not True:
        raise ValueError('模型增益初始化失败，请修正 URDF 惯量、关节映射或能力限制：' + str(payload.get('error') or '缺少模型建议，请编译安装当前 Core'))
    if not isinstance(axes, list) or [axis.get('joint') for axis in axes] != names or any(
        not isinstance(axis.get(key), (float, int)) or not math.isfinite(axis[key]) or axis[key] < 0 or key == 'kp' and axis[key] == 0
        for axis in axes for key in ('kp', 'kd')):
        raise ValueError('模型增益初始化的关节顺序或建议参数无效')
    result = {**result, 'measured': False, 'validated': False,
              'demand_source': 'recovery_targets_without_demonstration',
              'hardware_constraints': 'declared_motor_ranges' if complete else 'pending_hardware_selection',
              'configuration_applied': False, 'runtime_baseline_unchanged': True,
              'suggestion_applicable': all(row['protocol_kp_max'] is not None for row in limits),
              'protocol_gain': protocol_gain_limits(core, hardware, driver),
              'urdf_path': str(Path(urdf_path).resolve()), 'positions': positions, 'limits': limits,
              'model_physical_prior': {'source': 'urdf_geometry_mass_com_inertia',
                  'scope': 'M_C_g_only', 'actuator_response_measured': False,
                  'excluded_fields': ['urdf_velocity_limit', 'urdf_effort_limit', 'policy_velocity', 'policy_acceleration', 'policy_effort']},
              'task_design_targets': {'source': 'recovery_targets_without_demonstration',
                  'peak_error_rad': math.pi/1800, 'settling_time_s': .7,
                  'designed_probe_amplitude_rad': .008, 'tracking_demand_measured': False},
              'user_constraints': {'source': 'declared_urdf_policy_and_protocol_constraints_not_physical_response_prior',
                  'axes': [{key: value for key, value in row.items() if not key.startswith('gain_design_') and key != 'model_feedforward_mode'} for row in limits]},
              'execution_conditions': {'model_feedforward_mode': (core.get('control', {}).get('runtime') or {}).get('model_feedforward_mode', 'NONE')}}
    if not result['suggestion_applicable']:
        result['pending_reason'] = 'select_supported_hardware_driver_before_applying_model_suggestions'
    # Model candidates are evidence for calibration preparation, not qualified
    # normal-operation or fault-recovery baselines. Leave the entire core intact.
    return result
