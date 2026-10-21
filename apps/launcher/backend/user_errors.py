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
        f'请求失败（{exception_name}）：{original_message[:280]}' if original_message
        else f'请求失败（{exception_name}），后端异常未提供文字说明')
    advice = '请核对当前状态和操作条件，必要时复制诊断信息进行排查'
    retryable = False
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
    if isinstance(error, ValueError) and ('尚未选择机器人 Profile' in original_message or 'robot profile not found:' in original_message):
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
        advice = '无需重新采集标定数据 请检查 Core 配置及导出程序后重试 原始任务数据不会因本次解析失败而自动修改'
        mark = getattr(error, 'problem_mark', None) or getattr(error, 'context_mark', None)
        if mark:
            details['line'] = mark.line + 1
            details['column'] = mark.column + 1
        if 'export' in operation:
            details['suspected_section'] = 'Core YAML 标定候选参数'
    elif isinstance(error, FileNotFoundError):
        code, category, reason = 'RESOURCE_NOT_FOUND', 'file', '所需文件不存在或已被移动'
        advice = '请确认 Profile、任务记录与资源路径仍然有效'
        details['path'] = getattr(error, 'filename', None) or ''
    elif isinstance(error, PermissionError):
        code, category, reason = 'RESOURCE_PERMISSION_DENIED', 'file', '没有读取或写入所需文件的权限'
        advice = '请检查文件所属用户和目录权限，避免直接使用管理员权限启动 GUI'
        details['path'] = getattr(error, 'filename', None) or ''
    elif isinstance(error, TimeoutError) or re.search(r'timed out|timeout|EPIPE', technical, re.I):
        code, category = 'BACKEND_TIMEOUT', 'communication'
        reason = '后台请求超时或通信中断，上一条指令的执行结果可能未知'
        advice = '涉及机械臂运动时先确认当前硬件状态，不要直接重复下发运动命令'
    elif re.search(r'\b(?:FAULTED|SAFETY_FAILED|MOTOR_BUS_[A-Z_]+|JOINT_POS_LIMIT|JOINT_VEL_LIMIT|INVALID_DT)\b', technical):
        code, category = 'ROBOT_SAFETY_REJECTED', 'safety'
        reason = '机器人安全状态不允许当前操作'
        advice = '先确保机械臂安全并检查具体故障，再按控制内核的恢复流程处理，不能自动重试'
    elif isinstance(error, ValueError):
        code, category = 'INVALID_OPERATION_INPUT', 'validation'
        if re.search(r'[\u3400-\u9fff]', technical):
            reason = technical.strip("'\"")
        advice = '请检查页面中的参数、任务选择和当前机器人状态，修改后再操作'
    elif isinstance(error, (TypeError, KeyError, IndexError, AttributeError, AssertionError)):
        code, category = 'BACKEND_INTERNAL_ERROR', 'internal'
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
        'code': code, 'category': category, 'severity': 'critical' if category == 'safety' else 'error',
        'operation': operation, 'title': label + '失败', 'message': reason,
        'advice': advice, 'details': details, 'diagnostic_id': diagnostic_id,
        'technical': technical + ('\n\n' + technical_trace[-14000:] if technical_trace != technical else ''), 'retryable': retryable,
    }
