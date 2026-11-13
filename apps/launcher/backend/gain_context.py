"""Bind measured gains to their model, mapping and execution context"""
from __future__ import annotations

import json
import math
from copy import deepcopy
from pathlib import Path

import yaml

from control_parameters import canonical_core, effective_hardware
from model_calibration_source import matches_recorded_fingerprint


def _plugin_name(value):
    name = Path(str(value or '')).name
    if name.startswith('lib'):
        name = name[3:]
    return name.split('.so', 1)[0]


def _serial_device_identity(value):
    if not isinstance(value, str):
        return value
    path = Path(value)
    try:
        if path.is_absolute() and path.is_char_device():
            return str(path.resolve(strict=True))
    except (OSError, RuntimeError):
        pass
    return value


def gain_hardware_context(hardware, plugin, overrides=None):
    """Compare actual Damiao connection settings without rewriting snapshots"""
    # HardwareLoader rewrites a file only for an actual override. In its
    # absence, preserve authored flat fields, which override the named bus.
    overrides = {key: value for key, value in (overrides or {}).items() if value not in (None, '')}
    result = effective_hardware(hardware, overrides) if overrides else deepcopy(hardware)
    if _plugin_name(plugin) != 'serial_arm_hardware_damiao' or not isinstance(result, dict):
        return result
    target = result.get('damiao', result)
    if not isinstance(target, dict):
        return result
    buses = result.get('buses')
    bus = buses.get(target.get('bus', 'main_can')) if isinstance(buses, dict) else None
    if isinstance(bus, dict):
        if 'serial_port' not in target and ('serial_port' in bus or 'device' in bus):
            target['serial_port'] = deepcopy(bus.get('serial_port', bus.get('device')))
        if 'baudrate' not in target and 'baudrate' in bus:
            target['baudrate'] = deepcopy(bus['baudrate'])
        for key in ('serial_port', 'device'):
            if key in bus:
                bus[key] = _serial_device_identity(bus[key])
    if 'serial_port' in target:
        target['serial_port'] = _serial_device_identity(target['serial_port'])
    return result


def _purpose_name(purpose):
    return {'RIGID_TRACKING': 'RIGID', 'RIGID_HOLD': 'RIGID',
            'COMPLIANT_TRACKING': 'COMPLIANT', 'COMPLIANT_HOLD': 'COMPLIANT',
            'COMPLIANT_DRAG': 'DRAG'}.get(str(purpose).upper(), str(purpose).upper())


def _joint_context(value, names, field=''):
    """Read native ordered vectors and GUI joint maps as the same parameters"""
    if isinstance(value, list):
        if len(value) != len(names):
            raise ValueError('增益记录的逐关节上下文维度不匹配: ' + field)
        return dict(zip(names, map(bool, value) if field == 'joint_enabled' else value))
    if isinstance(value, dict):
        return {key: _joint_context(item, names, key) for key, item in value.items()}
    return value


def gain_interaction_context(core, purpose, *, compensation=False):
    purpose = _purpose_name(purpose)
    admittance = ((core.get('capability') or {}).get('admittance') or {})
    if purpose == 'SERVO_TRACKING':
        keys = ('enabled', 'joint_enabled', 'observer', 'calibration', 'controller') if admittance.get('enabled') is True else ('enabled',)
        return {key: admittance.get(key, False if key == 'enabled' else None) for key in keys}
    if purpose == 'RIGID' and compensation:
        return {key: admittance.get(key) for key in ('observer', 'calibration')}
    return None


def recorded_gain_core(core, metadata, candidate):
    """Restore measured settings which could differ from the saved source file"""
    original = canonical_core(core)
    names = metadata.get('joint_names') or (original.get('model') or {}).get('joint_names') or []
    mapping = metadata.get('mapping') or {}
    fields = ('pos_ratio', 'tor_ratio', 'direction', 'joint_zero_offset', 'actuator_zero_offset')
    if all(field in mapping for field in fields):
        columns = {field: _joint_context(mapping[field], names, field) for field in fields}
        if any(not isinstance(column, dict) or set(column) != set(names) for column in columns.values()):
            raise ValueError('增益记录的映射上下文维度不匹配')
        original['calibration'] = {'joints': {name: {field: columns[field][name] for field in fields} for name in names}}
    for field in ('base_frame', 'tool_frame', 'gravity'):
        if field in metadata:
            original['model'][field] = metadata[field]
    if 'original_gravity_scale' in metadata:
        original['model']['gravity_scale'] = _joint_context(metadata['original_gravity_scale'], names, 'gravity_scale')
    context = candidate.get('control_context')
    if context is not None:
        if not isinstance(context, dict) or not isinstance(context.get('runtime'), dict) or not isinstance(context.get('admittance'), dict):
            raise ValueError('增益候选的实测控制上下文格式无效')
        original['control']['runtime'].update(context['runtime'])
        disabled = context['admittance'].get('enabled') is not True
        def overlay(before, actual):
            result = deepcopy(before) if isinstance(before, dict) else {}
            for key, value in actual.items():
                if disabled and isinstance(value, list) and not value:
                    continue  # Native-only sessions leave unused vectors empty
                result[key] = (overlay(result.get(key), value) if isinstance(value, dict)
                               else _joint_context(value, names, key))
            return result
        original.setdefault('capability', {})['admittance'] = overlay(
            (original.get('capability') or {}).get('admittance'), context['admittance'])
    return original


def _unnamed_profile_evidence(folder, metadata, context):
    """Bind config-only recordings by complete source and measured evidence"""
    files=[]
    for name, key in (('model.urdf','urdf_fingerprint'),('hardware.yaml','hardware_fingerprint'),
                      ('hardware-effective.yaml','hardware_effective_fingerprint')):
        snapshot=folder/'source'/name
        if not matches_recorded_fingerprint(snapshot,metadata.get(key)):
            raise ValueError('未记录机器人名称的增益记录缺少可信 '+name+' 快照，无法确认当前机械臂')
        files.append(snapshot)
    runtime=context.get('runtime') if isinstance(context,dict) else None
    admittance=context.get('admittance') if isinstance(context,dict) else None
    def finite_number(value):
        return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    if (not isinstance(runtime,dict) or not isinstance(admittance,dict)
            or not isinstance(runtime.get('write_enabled'),bool)
            or not finite_number(runtime.get('ctrl_frequency_hz')) or runtime['ctrl_frequency_hz']<=0
            or not finite_number(runtime.get('joint_acc_filter_alpha')) or not 0<=runtime['joint_acc_filter_alpha']<=1
            or runtime.get('model_feedforward_mode') not in ('NONE','GRAVITY','FULL_INVERSE_DYNAMICS')
            or not isinstance(admittance.get('enabled'),bool)):
        raise ValueError('未记录机器人名称的增益记录缺少完整有效的实测控制上下文，无法确认当前机械臂')
    return files


def verify_gain_context(folder, info, core, hardware, purposes):
    """Return source evidence files, or reject a changed measurement context

    Quality metrics and gains are intentionally excluded: measured warnings
    remain usable, and selecting one purpose must not reset other gains
    """
    folder = Path(folder)
    metadata_path = folder / 'metadata.json'
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if metadata.get('profile') and metadata['profile'] != info['profile']:
        raise ValueError('增益记录属于其他机器人或未记录机器人名称，请使用当前机器人的标定记录')
    recorded_core = folder / 'source/core.yaml'
    if not matches_recorded_fingerprint(recorded_core, metadata.get('core_fingerprint')):
        raise ValueError('增益记录缺少可信 Core 快照，请恢复原始任务文件或重新标定')
    candidate_path = folder / 'gain-candidate.json'
    candidate = json.loads(candidate_path.read_text(encoding='utf-8')) if candidate_path.is_file() else {}
    context = candidate.get('control_context')
    unnamed_files=_unnamed_profile_evidence(folder,metadata,context) if not metadata.get('profile') else []
    original = recorded_gain_core(yaml.safe_load(recorded_core.read_text(encoding='utf-8')), metadata, candidate)
    core = canonical_core(core)
    names = metadata.get('joint_names') or (original.get('model') or {}).get('joint_names') or []
    if original.get('calibration') != core.get('calibration'):
        raise ValueError('core.calibration.joints 当前映射与增益标定时不同，请重新标定本机增益')
    for field in ('joint_names', 'base_frame', 'tool_frame', 'gravity', 'gravity_scale'):
        before = (original.get('model') or {}).get(field)
        current = (core.get('model') or {}).get(field)
        if field == 'gravity_scale':
            before, current = _joint_context(before, names, field), _joint_context(current, names, field)
        if before != current:
            raise ValueError('core.model.' + field + ' 与增益标定时不同，请重新标定本机增益')
    model = Path(str((core.get('model') or {}).get('urdf_path') or ''))
    if not model.is_absolute():
        model = Path(info['resources']['core']).parent / model
    if not matches_recorded_fingerprint(model.resolve(), metadata.get('urdf_fingerprint')):
        raise ValueError('当前 URDF 与增益标定时不同，请选择当前模型的标定任务')
    before_runtime = (original.get('control') or {}).get('runtime') or {}
    current_runtime = (core.get('control') or {}).get('runtime') or {}
    for field in ('write_enabled', 'model_feedforward_mode', 'ctrl_frequency_hz', 'joint_acc_filter_alpha'):
        default = .2 if field == 'joint_acc_filter_alpha' else None
        if before_runtime.get(field, default) != current_runtime.get(field, default):
            raise ValueError('core.control.runtime.' + field + ' 与增益标定时不同，请重新验证当前控制设置')
    rigid = (candidate.get('targets') or {}).get('rigid_tracking') if candidate.get('category') == 'all' else candidate
    compensation = bool(((rigid or {}).get('hold_compensation') or {}).get('enabled'))
    for purpose in purposes:
        expected = gain_interaction_context(original, purpose, compensation=compensation)
        if expected is not None:
            current = ((core.get('capability') or {}).get('admittance') or {})
            recorded = _joint_context(expected, names)
            if {key: current.get(key, False if key == 'enabled' else None) for key in expected} != recorded:
                raise ValueError('core.capability.admittance 与增益测量使用的观测或导纳参数不同，请重新验证当前设置')
    files = [metadata_path, recorded_core, *unnamed_files]
    if context is not None:
        files.append(candidate_path)
    correction = (original.get('model') or {}).get('gravity_correction_path')
    current_correction = (core.get('model') or {}).get('gravity_correction_path')
    if bool(correction) != bool(current_correction):
        raise ValueError('core.model.gravity_correction_path 与增益标定时不同，请重新标定')
    if correction:
        snapshot = folder / 'source/gravity-correction.yaml'
        expected = metadata.get('gravity_correction_fingerprint')
        current = (Path(info['resources']['core']).parent / current_correction).resolve()
        if not matches_recorded_fingerprint(snapshot, expected) or not matches_recorded_fingerprint(current, expected):
            raise ValueError('core.model.gravity_correction_path 缺少与本机一致的标定快照，请重新标定')
        files.append(snapshot)

    # Mock sessions have no hardware snapshots. They can only bind to another
    # mock context; write_enabled above prevents reuse on an actual arm
    if before_runtime.get('write_enabled') is False:
        return files
    recorded_hardware = folder / 'source/hardware.yaml'
    overrides = metadata.get('hardware_overrides')
    if not isinstance(overrides, dict) or not matches_recorded_fingerprint(recorded_hardware, metadata.get('hardware_fingerprint')):
        raise ValueError('历史增益记录未保存硬件配置与连接覆盖值，请恢复原始任务文件或重新标定')
    plugin = (metadata.get('runtime_provenance') or {}).get('hardware_plugin_file_path')
    if not plugin or _plugin_name(plugin) != _plugin_name(info.get('hardware_plugin')):
        raise ValueError('增益标定使用的硬件驱动与当前机器人不同或未记录，请重新标定')
    before_hardware = gain_hardware_context(yaml.safe_load(recorded_hardware.read_text(encoding='utf-8')), plugin, overrides)
    if before_hardware != gain_hardware_context(hardware, info.get('hardware_plugin'), info.get('connection_overrides')):
        raise ValueError('硬件配置与增益标定时不同，请检查执行器、总线及连接覆盖值后重新标定')
    if recorded_hardware not in files:
        files.append(recorded_hardware)
    effective = folder / 'source/hardware-effective.yaml'
    if effective.is_file() or metadata.get('hardware_effective_fingerprint'):
        if not matches_recorded_fingerprint(effective, metadata.get('hardware_effective_fingerprint')):
            raise ValueError('增益标定的实际硬件配置快照已变化，请恢复原始任务文件')
        if gain_hardware_context(yaml.safe_load(effective.read_text(encoding='utf-8')), plugin) != before_hardware:
            raise ValueError('实际硬件配置快照与标定连接覆盖值不一致，请检查原始任务')
        if effective not in files:
            files.append(effective)
    return files
