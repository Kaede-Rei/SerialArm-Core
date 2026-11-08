"""Stable operator-facing error contract for Python/Native Launcher requests.

No motor-control behaviour or retry policy is changed here. Full tracebacks
are sent only to the technical stderr log with a diagnostic identifier.
"""
from __future__ import annotations

import json
import traceback
import re
import uuid
import yaml

OPERATIONS = {
    'calibration_tool_export': '标定成果导出',
    'calibration_tool_export_record': '标定成果导出',
    'workbench_config_save': '工作台参数保存',
    'workbench_config_preview': '工作台参数预览',
    'profile_editor_save': '机器人配置保存',
    'profile_editor_load': '机器人配置读取',
    'geometry_fixed_export_tcp': 'TCP 标定结果导出',
    'model_calibration_export_urdf': '模型候选导出',
    'workbench_request': '机械臂操作',
    'workbench_start': '机械臂连接',
    'workbench_stop': '机械臂断开',
    'trajectory_load': '示教轨迹读取',
    'trajectory_save': '示教轨迹保存',
    'calibration_tool_records': '标定任务记录扫描',
    'calibration_tool_load': '标定任务记录加载',
    'model_calibration_records': '模型标定记录扫描',
    'model_calibration_load': '模型标定记录加载',
    'model_calibration_recompute': '历史标定数据重算',
    'model_calibration_advanced': '高级动力学辨识',
    'model_calibration_assess': '标定结果评估',
    'model': '机器人模型读取',
    'model_preview': '机器人模型预览',
    'inspect': '机器人配置检查',
    'profile_editor_save': '机器人配置保存',
    'geometry_fixed_begin': '固定点几何标定启动',
    'geometry_fixed_capture': '固定点采样',
    'geometry_fixed_validate': '正运动学一致性验证',
    'geometry_repeat_play': '定点重复到位测试',
}


def _machine_reason(technical):
    codes = set(re.findall(r'\b(?:STATE_TIMEOUT|CMD_TIMEOUT|INVALID_DT|FAULTED|SAFETY_FAILED|REFERENCE_GOVERNOR_FAILED|MOTOR_BUS_[A-Z_]+|ACTUATOR_OFFLINE|ENABLE_FAILED|DISABLE_FAILED|JOINT_[A-Z_]+)\b', technical))
    if not codes: return None
    joint = re.search(r"(?:joint|actuator)\s*[=:]\s*['\"]?([A-Za-z0-9_.-]+)", technical, re.I)
    prefix = joint.group(1) + '：' if joint else ''
    values = {key: float(value) for key, value in re.findall(r'\b(value|limit)\s*=\s*([-+0-9.eE]+)', technical) if re.fullmatch(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', value)}
    if 'STATE_TIMEOUT' in codes:
        measured = f"（{values['value'] * 1000:.1f} ms，允许 {values['limit'] * 1000:.1f} ms）" if set(values) == {'value', 'limit'} else ''
        return 'FEEDBACK_TIMEOUT', 'safety', prefix + '关节反馈已超时' + measured, '检查电机供电、USB-CAN 连接和反馈连续性；反馈恢复后清除故障，再重新使能', {'page': 'diagnostics'}
    if 'CMD_TIMEOUT' in codes:
        return 'COMMAND_TIMEOUT', 'safety', '运动命令更新超时，当前任务已中断', '检查命令发送程序及更新频率；处理后清除故障，再重新开始任务', {'page': 'workbench', 'tab': 'monitor'}
    if codes & {'MOTOR_BUS_DEACTIVATE_FAILED', 'DISABLE_FAILED'}:
        return 'MOTOR_DISABLE_UNCONFIRMED', 'safety', prefix + '电机失能未得到确认', '先支撑机械臂，检查电机供电和通信；需要时使用硬件急停，再处理连接', {'page': 'diagnostics'}
    if codes & {'MOTOR_BUS_ACTIVATE_FAILED', 'ENABLE_FAILED', 'ACTUATOR_OFFLINE'}:
        return 'MOTOR_ENABLE_FAILED', 'safety', prefix + '电机使能失败或执行器离线', '检查电机供电、急停、CAN 接线及电机 ID；反馈正常后清除故障并重新使能', {'page': 'diagnostics'}
    if 'MOTOR_BUS_CONNECT_FAILED' in codes:
        return 'MOTOR_CONNECTION_FAILED', 'communication', '无法连接执行器通信设备', '在连接诊断中核对设备路径与权限，并检查 USB-CAN 连接', {'page': 'diagnostics'}
    if 'MOTOR_BUS_READ_FAILED' in codes:
        return 'MOTOR_FEEDBACK_FAILED', 'safety', prefix + '执行器反馈读取失败', '检查通信设备、线缆与电机供电；反馈恢复后清除故障', {'page': 'diagnostics'}
    if 'MOTOR_BUS_WRITE_FAILED' in codes:
        return 'MOTOR_COMMAND_FAILED', 'safety', prefix + '执行器未接受控制命令', '先支撑机械臂，再检查通信、电机状态和供电', {'page': 'diagnostics'}
    if any(code.startswith('JOINT_POS') for code in codes):
        return 'JOINT_POSITION_LIMIT', 'safety', prefix + '关节位置超出安全范围', '核对机械位置、零位和方向；按页面恢复条件处理后再清除故障', {'page': 'profile_setup', 'group': 'mapping'}
    if any(code.startswith('JOINT_VEL') for code in codes):
        return 'JOINT_VELOCITY_LIMIT', 'safety', prefix + '关节速度超出安全上限', '检查实际运动和反馈速度，降低命令速度后再重新开始任务', {'page': 'workbench', 'tab': 'control'}
    if 'REFERENCE_GOVERNOR_FAILED' in codes:
        return 'REFERENCE_LIMIT_REJECTED', 'safety', '当前目标无法满足运动或总力矩限制', '缩小目标变化或降低速度；仍被拒绝时检查关节限位、增益和力矩预算', {'page': 'workbench', 'tab': 'control'}
    if 'INVALID_DT' in codes:
        return 'CONTROL_CYCLE_INVALID', 'safety', '控制周期超出允许范围', '检查控制循环调度和系统负载；周期恢复正常后清除故障', {'page': 'diagnostics'}
    return 'ROBOT_SAFETY_REJECTED', 'safety', '机械臂处于故障状态，当前操作被拒绝', '查看当前故障原因，处理后清除故障；重新使能需单独操作', {'page': 'workbench', 'tab': 'control'}


def describe(error, method='', params=None):
    params = params if isinstance(params, dict) else {}
    operation = method if isinstance(method, str) else ''
    label = OPERATIONS.get(operation, '当前操作')
    if operation == 'workbench_request' and isinstance(params.get('method'), str):
        label = {
            'activate': '使能机械臂', 'clear_fault': '清除故障',
            'deactivate': '失能机械臂', 'park': '停放机械臂',
            'hold': '当前位置保持', 'teach_replay_play': '示教回放',
            'model_calibration_teach_stop': '结束示教',
        }.get(params['method'], label)
    exception_name = type(error).__name__
    original_message = str(error).strip()
    # str(error) may be empty or just an opaque wrapper, so the exception
    # type and the actual traceback MUST remain reachable from the GUI.
    chain = ''.join(traceback.format_exception(type(error), error, error.__traceback__, chain=True)).strip()
    technical = f'{exception_name}: {original_message}' if original_message else exception_name
    technical_trace = chain if chain else technical
    code = 'OPERATION_FAILED'
    category = 'operation'
    reason = original_message if re.search(r'[\u3400-\u9fff]', original_message) else (
        original_message[:280] if original_message else '后台未能完成请求，未返回具体原因')
    advice = '请核对当前状态和操作条件，必要时复制诊断信息进行排查'
    retryable = False
    action = None
    details = {'exception_type': exception_name}
    if error.__traceback__ is not None:
        frames = traceback.extract_tb(error.__traceback__)
        # Prefer the project entry point to the final frame inside PyYAML or
        # another dependency. The full call stack remains in technical.
        project_frames = [frame for frame in frames if '/backend/' in frame.filename.replace('\\', '/')
                          and '/site-packages/' not in frame.filename.replace('\\', '/')]
        source_frame = project_frames[-1] if project_frames else frames[-1]
        details['source'] = f'{source_frame.filename}:{source_frame.lineno} ({source_frame.name})'
    if error.__cause__ is not None:
        details['cause_type'] = type(error.__cause__).__name__
    if operation.startswith('calibration_tool_') and operation not in OPERATIONS:
        label = '标定成果操作'
    machine = _machine_reason(technical)
    if machine:
        code, category, reason, advice, action = machine
    elif 'gain candidate control context mismatch:' in original_message:
        code, category = 'GAIN_CANDIDATE_CONTEXT_CHANGED', 'configuration'
        field = original_message.split('gain candidate control context mismatch:', 1)[1].split(';', 1)[0].strip()
        reason = '当前控制配置与增益标定时不一致，未应用候选（' + field + '）'
        advice = '使用与当前机械臂配置及会话参数一致的标定记录，或在当前设置下重新标定；历史结果仍可查看和导出'
        details['context_field'] = field
    elif isinstance(error, ValueError) and ('尚未选择机器人 Profile' in original_message or 'robot profile not found:' in original_message):
        code, category = 'PROFILE_NOT_SELECTED', 'configuration'
        reason = '尚未选择机器人 Profile，无法扫描对应机器人的历史任务'
        advice = '先选择机器人 Profile 再扫描，或点击加载任务记录直接选择历史任务目录；离线导出不需要连接机械臂'
    elif isinstance(error, ValueError) and '未找到机器人 Profile' in original_message:
        code, category = 'PROFILE_NOT_FOUND', 'configuration'
        reason = original_message
        advice = '检查当前加载的机器人 Profile 清单和路径；已经保存的历史任务可以直接加载并离线导出'
    elif isinstance(error, yaml.YAMLError):
        code, category = ('PROFILE_EXPORT_YAML_INVALID' if 'export' in operation else 'PROFILE_YAML_INVALID'), 'configuration'
        reason = '生成的 Core YAML 格式不正确，未能完成导出' if 'export' in operation else 'YAML 配置格式错误，无法读取相关参数'
        advice = ('导出程序生成了无效 YAML，请保留任务文件并复制诊断信息排查导出逻辑' if 'export' in operation
                  else '检查报错行附近的缩进、冒号和字段值，修正 YAML 后重新加载')
        mark = getattr(error, 'problem_mark', None) or getattr(error, 'context_mark', None)
        if mark:
            details['line'] = mark.line + 1
            details['column'] = mark.column + 1
            reason += f'（第 {mark.line + 1} 行，第 {mark.column + 1} 列）'
        if 'export' in operation:
            details['suspected_section'] = 'Core YAML 标定候选参数'
    elif isinstance(error, FileNotFoundError):
        path = getattr(error, 'filename', None) or ''
        code, category, reason = 'RESOURCE_NOT_FOUND', 'file', '找不到文件' + ('：' + str(path).rsplit('/', 1)[-1] if path else '')
        advice = '请确认 Profile、任务记录与资源路径仍然有效'
        details['path'] = getattr(error, 'filename', None) or ''
    elif isinstance(error, PermissionError):
        code, category, reason = 'RESOURCE_PERMISSION_DENIED', 'file', '没有读取或写入所需文件的权限'
        advice = '请检查文件所属用户和目录权限，避免直接使用管理员权限启动 GUI'
        details['path'] = getattr(error, 'filename', None) or ''
    elif isinstance(error, TimeoutError) or re.search(r'timed out|timeout|EPIPE', technical, re.I):
        code, category = 'BACKEND_TIMEOUT', 'communication'
        hardware_request = operation in ('workbench_request', 'workbench_start', 'workbench_stop', 'geometry_repeat_play')
        reason = '控制会话请求超时，指令执行结果尚未确认' if hardware_request else label + '超时'
        advice = '先确认机械臂状态，再处理连接；不要重复发送未确认的运动命令' if hardware_request else '检查任务文件与后台诊断；数据量较大时可减少计算范围后重试'
        action = {'page': 'diagnostics'} if hardware_request else None
    elif isinstance(error, ValueError):
        code, category = 'INVALID_OPERATION_INPUT', 'validation'
        if re.search(r'[\u3400-\u9fff]', technical):
            reason = original_message.strip("'\"")
        advice = '请检查页面中的参数、任务选择和当前机器人状态，修改后再操作'
        from profile_library import ProfileLibrary
        field = ProfileLibrary._field_issue(original_message)
        if field['path']:
            action = field['action']
            advice = '打开对应参数，修正后重新保存'
    elif isinstance(error, (TypeError, KeyError, IndexError, AttributeError, AssertionError)):
        code, category = 'BACKEND_INTERNAL_ERROR', 'internal'
        reason = '程序无法处理当前数据'
        advice = '程序处理当前数据时发生异常，请复制完整诊断信息以定位源码位置，无需盲目修改机器人配置'
    elif isinstance(error, OSError):
        code, category = 'IO_OPERATION_FAILED', 'file'
        reason = '访问文件或设备时发生系统错误'
        advice = '检查目标路径、设备连接和访问权限'
    diagnostic_id = uuid.uuid4().hex[:12]
    if isinstance(params.get('kind'), str):
        details['kind'] = params['kind'][:64]
    if isinstance(params.get('directory'), str):
        details['directory'] = params['directory'][:1024]
    return {
        'code': code, 'category': category, 'severity': 'critical' if category == 'safety' and code != 'REFERENCE_LIMIT_REJECTED' else 'error',
        'operation': operation, 'title': label + '失败', 'message': reason,
        'advice': advice, 'details': details, 'diagnostic_id': diagnostic_id,
        'technical': technical + ('\n\n' + technical_trace[-14000:] if technical_trace != technical else ''), 'retryable': retryable,
        'action': action,
    }
