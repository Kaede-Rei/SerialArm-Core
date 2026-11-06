#!/usr/bin/env python3
"""Line-delimited IPC; events and replies share stdout with serialized writes."""
import json
import copy
import math
import os
from pathlib import Path
import signal
import sys
import subprocess
import threading
import time
import traceback

# Keep optional optimization dependencies in the dedicated GUI environment;
# ROS setup may otherwise prepend an incompatible system SciPy.
if sys.prefix != sys.base_prefix:
    _gui_site=Path(sys.prefix)/'lib'/('python'+str(sys.version_info.major)+'.'+str(sys.version_info.minor))/'site-packages'
    if _gui_site.is_dir():sys.path.insert(0,str(_gui_site))

from user_errors import describe

from profiles import Inspector, command_for, config_values
from runtime import Supervisor
from machine import NativeSession
from model_runtime import ModelRuntime
from persistence import (export_friction_candidate, export_telemetry, export_candidate_urdf, load_model_calibration_summary, list_model_calibration_records, preview_gravity_correction, restore_gravity_correction, save_gravity_correction, update_candidate_urdf_verification, preview as preview_config, save as save_config)
from profile_library import ProfileLibrary
from profile_storage import profile_home, category_directory
from model_calibration_alignment import compare_calibration_urdfs
from tcp_calibration import TcpCalibration

ROOT = Path(__file__).resolve().parents[3]
output_lock = threading.Lock()
OFFLINE_CALIBRATION_METHODS = frozenset((
    'model_calibration_assess', 'model_calibration_advanced', 'model_calibration_recompute',
    'model_calibration_idim', 'model_calibration_idim_audit',
))


def offline_profile_home(inspector, config):
    # Historical task inventory must not require hardware files, Core readiness or a live session
    selected = config_values(config)
    if not selected['profile']:
        raise ValueError('尚未选择机器人 Profile，扫描历史任务前请选择机器人，或直接加载任务目录，无需连接机械臂')
    listing = inspector.profiles(selected)
    if selected['profile'] not in {item['name'] for item in listing['profiles']}:
        raise ValueError(f'未找到机器人 Profile {selected["profile"]!r}，请检查 Profile 文件 {listing["profile_file"]}，或直接加载历史任务目录')
    return profile_home({'profile': selected['profile'], 'profile_file': listing['profile_file']}, ROOT)


def offline_task_home(directory, fallback_root):
    # Task exports are derived from immutable on-disk records, not an active Profile
    # Standard layout: <robot>/.serialarm/model-calibration/<task>
    folder = Path(directory).expanduser().resolve() if directory else None
    if folder and folder.parent.name == 'model-calibration' and folder.parent.parent.name == '.serialarm':
        return folder.parent.parent.parent
    return Path(fallback_root).resolve()


def runtime_parameter_snapshot(inspector, native, config, active_config, active_fingerprint):
    if not active_config or native.status().get('state') != 'running':
        raise ValueError('请先连接对应工作台，再保存当前会话参数')
    info, active = inspector.inspect(config), inspector.inspect(active_config)
    if (info['fingerprint'] != active_fingerprint or active['fingerprint'] != active_fingerprint or
            Path(info['resources']['core']).resolve() != Path(active['resources']['core']).resolve() or
            info.get('profile') != active.get('profile')):
        raise ValueError('保存目标与当前工作台配置不一致，请重新连接后再保存')
    snapshot = native.request('status', {}, timeout=8)
    if ((snapshot.get('gravity_verification') or {}).get('active') or
            (snapshot.get('model_calibration') or {}).get('phase', 'idle') not in
            ('idle', 'complete', 'failed', 'cancelled')):
        raise ValueError('标定或验证任务进行中不能保存临时试验参数')
    return info, native.request('get_admittance', {}, timeout=8), snapshot


from calibration_jobs import run_full_inertial_isolated, recompute_gravity_isolated


def restore_temporary_candidates_for_disconnect(native, allow_park=False):
    """Park and restore process-local candidates before a normal disconnect

    Keep the session running if restoration cannot be verified
    """
    snapshot = native.request('status', {}, timeout=8)
    if (snapshot.get('gravity_verification') or {}).get('active'):
        if snapshot.get('robot_state') != 'INACTIVE':
            raise ValueError('验证任务尚未在机械支撑下失能，请完成受控停止或紧急失能后再断开')
        native.request('gravity_verify_cancel', {}, timeout=8)
    overrides = native.request('candidate_overrides', {}, timeout=8)
    enabled = [kind for kind in ('dynamics', 'gravity', 'gain')
               if overrides.get(kind + '_applied')]
    if not enabled:
        return []
    status = native.request('status', {}, timeout=8)
    if status.get('robot_state') != 'INACTIVE':
        if not allow_park:
            raise ValueError('临时候选仍生效且自动停放未通过验收，请先机械支撑后失能，再撤销候选并断开')
        native.request('park', {}, timeout=120)
        status = native.request('status', {}, timeout=8)
    if status.get('robot_state') != 'INACTIVE':
        raise ValueError('无法确认机械臂已安全停放，已保留连接，请检查机器人状态后重试')
    for kind in enabled:
        native.request('candidate_' + kind + '_restore', {}, timeout=12)
    remaining = native.request('candidate_overrides', {}, timeout=8)
    if any(remaining.get(kind + '_applied') for kind in ('gain', 'gravity', 'dynamics')):
        raise ValueError('临时参数尚未全部撤销，已保留连接，请检查并重试')
    return enabled


# Native sessions do not trust browser button states for control admission
_READ_ONLY_MACHINE = {
    'status', 'get_admittance', 'candidate_overrides', 'calibration_status',
    'model_calibration_status', 'candidate_gain_status', 'candidate_gravity_status',
    'candidate_dynamics_status', 'teach_replay_status',
}
_SAFE_EXIT_MACHINE = {
    'deactivate', 'hold', 'gravity_verify_stop', 'gravity_verify_cancel', 'gravity_verify_finish',
    'model_calibration_cancel', 'model_calibration_teach_stop', 'model_calibration_pause',
    'candidate_gain_restore', 'candidate_gravity_restore', 'candidate_dynamics_restore',
    'teach_replay_record_stop', 'teach_replay_stop',
}
_TRACKING_MACHINE = {'move_to', 'move_relative'}
_GRAVITY_VERIFY_MACHINE = {
    'gravity_verify_begin', 'gravity_verify_drag', 'gravity_verify_target',
    'gravity_verify_sample', 'gravity_verify_stop', 'gravity_verify_finish', 'gravity_verify_cancel',
}


def verify_machine_admission(library, config, method, params, native):
    # Keep exit and read-only operations available even when the Profile is invalid
    if method in _READ_ONLY_MACHINE or method in _SAFE_EXIT_MACHINE:
        return
    readiness = library._readiness(config)
    info = readiness.get('info') or {}
    if info.get('write_enabled') is not True:
        raise ValueError('当前 Profile 的硬件写入已关闭，请在参数配置中检查 write_enabled')
    if not readiness.get('control_available'):
        raise ValueError('Core、硬件映射或 Safety 配置无效，请先修正实际加载所需参数')
    status = native.request('status', {}, timeout=8)
    robot_state = status.get('robot_state')
    modes = readiness['modes']

    def configured(mode):
        return modes.get(mode) in ('configured', 'verified', 'stale')

    def require_hold():
        if not configured('RIGID_HOLD'):
            raise ValueError('启动保持增益必须为正且不超过 Safety 上限')

    def require_feedback():
        age = status.get('feedback_age_ms')
        fault = (status.get('fault') or {}).get('code')
        if (robot_state != 'ACTIVE' or status.get('valid') is not True or
            not isinstance(age, (int, float)) or isinstance(age, bool) or
            not math.isfinite(age) or not 0 <= age <= 250 or
            not isinstance(status.get('sequence'), int) or status['sequence'] < 1 or
            fault not in (None, '', 'NONE', 'OK')):
            raise ValueError('控制命令要求有效且未超时的真实反馈及无故障 ACTIVE 状态')

    def require_gain_context(mode, *, playback=False, profile=None):
        if mode not in ('RIGID_HOLD', 'RIGID_TRACKING', 'COMPLIANT_HOLD', 'COMPLIANT_TRACKING', 'COMPLIANT_DRAG'):
            raise ValueError('无效的任务状态')
        runtime = native.request('get_admittance', {}, timeout=8)
        if playback and 'reference_behavior' in params and params['reference_behavior'] not in ('NATIVE_IMPEDANCE', 'MODEL_ADMITTANCE'):
            raise ValueError('无效的回放参考策略')
        behavior = (params.get('reference_behavior') if playback else None) or runtime.get('reference_behavior') or status.get('reference_behavior') or 'NATIVE_IMPEDANCE'
        if behavior not in ('NATIVE_IMPEDANCE', 'MODEL_ADMITTANCE'):
            raise ValueError('无效的回放参考策略')
        explicit = params.get('gain_profile') if profile is None else profile
        purpose = explicit if explicit is not None else ('DRAG' if mode == 'COMPLIANT_DRAG' else
                              'SERVO_TRACKING' if behavior == 'MODEL_ADMITTANCE' else
                              'COMPLIANT' if mode.startswith('COMPLIANT') else 'RIGID')
        if purpose not in ('RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING'):
            raise ValueError('无效的执行器增益用途')
        if playback and (purpose == 'DRAG' or mode.endswith('_HOLD')):
            raise ValueError('回放仅支持刚性、柔性或伺服跟踪用途')
        if (purpose == 'DRAG') != (mode == 'COMPLIANT_DRAG'):
            raise ValueError('拖拽用途与任务状态不一致')
        if mode != 'COMPLIANT_DRAG' and behavior == 'MODEL_ADMITTANCE':
            if purpose != 'SERVO_TRACKING' or runtime.get('enabled') is not True:
                raise ValueError('模型导纳要求启用能力并使用伺服跟踪增益')
        if 'gain_profile' in params and ((purpose in ('RIGID', 'SERVO_TRACKING') and not mode.startswith('RIGID')) or
                         (purpose == 'COMPLIANT' and not mode.startswith('COMPLIANT'))):
            raise ValueError('执行器增益用途与任务状态不一致')
        required = 'SERVO_TRACKING' if purpose == 'SERVO_TRACKING' else mode
        if not configured(required):
            raise ValueError('目标增益用途缺失或超过 Safety 上限: ' + required)

    gravity_active = (status.get('gravity_verification') or {}).get('active') is True
    if gravity_active and method not in (_GRAVITY_VERIFY_MACHINE | {'activate', 'deactivate', 'hold'}):
        raise ValueError('重力验证期间不能执行其他运动或参数调整')
    if method in _GRAVITY_VERIFY_MACHINE:
        require_hold()
        if method == 'gravity_verify_begin':
            if robot_state != 'INACTIVE' or gravity_active:
                raise ValueError('请在 INACTIVE 状态开始重力验证')
            overrides = native.request('candidate_overrides', {}, timeout=8)
            if any(overrides.get(k + '_applied') for k in ('gain', 'gravity', 'dynamics')):
                raise ValueError('请先撤销临时标定候选')
        elif method not in ('gravity_verify_finish', 'gravity_verify_cancel') and not gravity_active:
            raise ValueError('当前没有正在进行的重力验证任务')
        # Native Machine enforces drag damping, force ramp, pose sampling and safe stop
        return
    if method == 'activate':
        require_hold()
        if robot_state not in ('INACTIVE', 'FAULT'):
            raise ValueError('只允许从 INACTIVE 或可恢复的 FAULT 状态使能')
        return
    if method in ('hold', 'fault_rigid'):
        require_hold()
        return
    if method == 'clear_fault':
        # Native fault recovery is responsible for checking the robot state
        return
    if method == 'fault_compliant':
        # Native fault recovery still checks gravity_model_validated and limits
        return
    if method == 'park':
        require_feedback()
        if not all(configured(mode) for mode in ('RIGID_TRACKING', 'COMPLIANT_TRACKING')):
            raise ValueError('自动停放需要合法的刚性及柔性跟踪增益')
        return
    if method == 'set_model_feedforward_mode':
        if params.get('mode') not in ('NONE', 'GRAVITY', 'FULL_INVERSE_DYNAMICS'):
            raise ValueError('无效的模型前馈模式')
        if robot_state != 'INACTIVE':
            raise ValueError('模型前馈仅能在 INACTIVE 状态切换')
        return
    if method == 'set_gravity_scale':
        if robot_state != 'INACTIVE':
            raise ValueError('正常重力比例仅允许在 INACTIVE 状态调整')
        return
    if method == 'set_reference_behavior':
        behavior = params.get('behavior')
        if behavior not in ('NATIVE_IMPEDANCE', 'MODEL_ADMITTANCE'):
            raise ValueError('参考策略必须为 NATIVE_IMPEDANCE 或 MODEL_ADMITTANCE')
        if robot_state != 'INACTIVE':
            raise ValueError('请先停放并失能，再切换参考策略')
        if ((status.get('model_calibration') or {}).get('phase', 'idle') not in ('idle', 'complete', 'failed', 'cancelled') or
                (status.get('teach_replay') or {}).get('phase') in ('recording', 'playing')):
            raise ValueError('请先退出标定或示教回放任务，再切换参考策略')
        if behavior == 'MODEL_ADMITTANCE' and native.request('get_admittance', {}, timeout=8).get('enabled') is not True:
            raise ValueError('当前 Profile 未启用模型导纳能力')
        return
    if method == 'set_impedance_mode':
        require_feedback()
        mode = params.get('mode')
        require_gain_context(mode)
        if mode == 'COMPLIANT_DRAG':
            scales = status.get('gravity_scale') or []
            if not scales or not any(isinstance(x, (int, float)) and math.isfinite(x) and x > 0 for x in scales):
                raise ValueError('柔性拖拽需要非零的有效重力补偿')
        return
    if method in _TRACKING_MACHINE:
        require_feedback()
        mode = status.get('impedance_mode')
        tracking = {'RIGID_HOLD': 'RIGID_TRACKING', 'COMPLIANT_HOLD': 'COMPLIANT_TRACKING'}.get(mode, mode)
        if tracking not in ('RIGID_TRACKING', 'COMPLIANT_TRACKING'):
            raise ValueError('移动目标要求已配置的保持或跟踪状态，请先退出拖拽')
        require_gain_context(tracking, profile=status.get('gain_profile'))
        speed = params.get('speed_scale', 1.0)
        if (not isinstance(speed, (int, float)) or isinstance(speed, bool) or
            not math.isfinite(speed) or not 0 < speed <= 1):
            raise ValueError('运动速度比例必须在 (0,1]')
        values = params.get('delta') if method == 'move_relative' else params.get('positions')
        actual = (status.get('joint') or {}).get('pos') or []
        if (not isinstance(values, list) or len(values) != len(actual) or not values or
            not all(isinstance(x, (int, float)) and not isinstance(x, bool) and
                math.isfinite(x) for x in values + actual)):
            raise ValueError('运动目标维度无效或包含非有限数值')
        # Native Core enforces applicable URDF and Safety bounds, and any enabled continuity checks
        return
    if method == 'set_admittance':
        if robot_state != 'INACTIVE':
            require_feedback()
        if robot_state == 'ACTIVE' and status.get('impedance_mode') == 'COMPLIANT_DRAG':
            raise ValueError('柔性拖拽时不允许修改导纳运行参数')
        if status.get('reference_behavior') != 'MODEL_ADMITTANCE':
            raise ValueError('在线导纳调整要求先选择 MODEL_ADMITTANCE 参考策略')
        if (status.get('model_calibration') or {}).get('phase', 'idle') not in ('idle', 'complete', 'failed', 'cancelled'):
            raise ValueError('标定任务进行中不能修改导纳运行参数')
        return
    if method.startswith('teach_replay_'):
        if method in ('teach_replay_record_begin', 'teach_replay_play'):
            require_feedback()
        if method == 'teach_replay_record_begin':
            if not configured('COMPLIANT_DRAG'):
                raise ValueError('示教记录要求合法柔性拖拽增益')
        if method == 'teach_replay_play':
            selected_mode = params.get('mode')
            if selected_mode not in ('RIGID_TRACKING', 'COMPLIANT_TRACKING'):
                raise ValueError('回放阻抗模式只能选择刚性跟踪或柔性跟踪')
            require_gain_context(selected_mode, playback=True)
        return
    if method.startswith(('model_calibration_', 'candidate_', 'calibration_',
                          'friction_', 'gain_')):
        # Leave native calibration state machine and parameter verification intact
        return
    raise ValueError('未知的硬件控制方法')


def emit(data):
    with output_lock:
        print(json.dumps(data, ensure_ascii=False), flush=True)


ASYNC_EXPORT_METHODS = frozenset(('calibration_tool_export', 'calibration_tool_export_record', 'model_calibration_export_friction', 'model_calibration_export_urdf', 'model_calibration_idim_export', 'geometry_fixed_export_tcp'))

def export_offline_artifact(method, params, inspector, geometry_snapshot=None):
    if method == 'geometry_fixed_export_tcp':
        if geometry_snapshot is None:
            raise ValueError('缺少 TCP 候选快照，请重新打开几何实验')
        result = geometry_snapshot.export(config_values(params.get('config', {})), params.get('destination') or None)
    elif method == 'calibration_tool_export':
        from artifact_exports import export_artifact
        # Export uses the immutable task snapshot and the user-selected output directory
        # It must not consult the connected hardware or the current GUI Profile
        record_dir = params.get('directory', '')
        result = export_artifact(offline_task_home(record_dir, ROOT), record_dir, params.get('kind', ''),
            report=params.get('report'), destination=params.get('destination'),
            candidate_source=params.get('candidate_source'), candidate_path=params.get('candidate_path'), gain_purposes=params.get('gain_purposes'))
    elif method == 'calibration_tool_export_record':
        from workflow_registry import export_record
        record_dir = params.get('directory', '')
        result = export_record(offline_task_home(record_dir, ROOT), record_dir, params.get('kind', ''))
    elif method == 'model_calibration_export_friction':
        directory = params.get('directory', '')
        result = export_friction_candidate(directory, params.get('destination') or None)
    elif method == 'model_calibration_export_urdf':
        directory = params.get('directory', '')
        from persistence import select_gravity_candidate
        source, selected_candidate = select_gravity_candidate(directory, params.get('candidate_source'), params.get('candidate_path'))
        if source == 'offline':
            result = run_full_inertial_isolated(directory, params.get('destination') or None,
                mode='legacy_gravity_export', locked_links={'candidate_path':str(selected_candidate)})
        elif source == 'online':
            result = export_candidate_urdf(directory, params.get('destination') or None)
        else:
            raise ValueError('未知候选来源')
        # Independent verification is informative for manual-review
        # candidates and must never abort the IPC loop
        try:
            # Verify the exact exported URDF for both candidate sources
            # without re-fitting, applying commands or inheriting the online pass state
            calibrator = inspector.model_calibrator()
            if not calibrator:
                raise ValueError('离线校验工具未安装，候选 URDF 已导出供人工审核')
            info = inspector.inspect(config_values(params.get('config', {})))
            verify_args = [calibrator, '--config', info['resources']['core'],
                 '--dataset', str(Path(directory).resolve()),
                 '--verify-urdf', result['candidate_urdf'], '--candidate-source', source]
            if selected_candidate: verify_args.extend(['--candidate-path', str(selected_candidate)])
            verification = subprocess.run(
                verify_args,
                text=True, capture_output=True, timeout=60)
            if verification.returncode != 0:
                raise ValueError((verification.stderr or verification.stdout or
                                  '候选 URDF 离线验证失败').strip()[-1500:])
            verified = json.loads(verification.stdout.strip().splitlines()[-1])
            result['verification'] = update_candidate_urdf_verification(
                Path(result['candidate_urdf']).parent, verified)
        except (ValueError, OSError, subprocess.TimeoutExpired, json.JSONDecodeError, IndexError, KeyError) as error:
            result['verification_warning'] = str(error)
    elif method == 'model_calibration_idim_export':
        from calibration_jobs import export_dynamics_isolated
        result = export_dynamics_isolated(params.get('report_path', ''), params.get('destination') or None,review_only=params.get('review_only')is True,report_only=params.get('report_only')is True)
    else:
        raise ValueError('未知成果导出操作')
    return result


def main():
    inspector = Inspector(ROOT)
    lock_dir = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'serial-arm/locks'
    manager = Supervisor(emit, lock_dir)
    native = NativeSession(emit, lock_dir)
    model_runtime = ModelRuntime()
    profile_library = ProfileLibrary(ROOT, inspector)
    from trajectory_library import TrajectoryLibrary
    trajectories = TrajectoryLibrary(ROOT)
    geometry = TcpCalibration(profile_library)
    active_workbench_config = None
    active_workbench_fingerprint = None
    repeat_source = None
    control_job = None
    export_job = None
    control_cancel = threading.Event()

    def start_control_job(request, action):
        nonlocal control_job
        control_cancel.clear()
        def run():
            try:
                emit({'id': request.get('id'), 'result': action()})
            except Exception as error:
                report = describe(error, request.get('method', ''), request.get('params'))
                print(f'[SerialArm diagnostic {report["diagnostic_id"]}]\n{traceback.format_exc()}', file=sys.stderr, flush=True)
                emit({'id': request.get('id'), 'error': report})
        control_job = threading.Thread(target=run, name='workspace-park')
        control_job.start()
    def start_export_job(request, action=None):
        nonlocal export_job
        if export_job and export_job.is_alive():
            raise ValueError('已有成果正在导出，请等待当前导出完成；机器人控制仍可使用')
        geometry_snapshot = None
        if request['method'] == 'geometry_fixed_export_tcp':
            session, *_ = geometry._ensure(config_values(request.get('params', {}).get('config', {})))
            geometry_snapshot = TcpCalibration(profile_library)
            geometry_snapshot.session = copy.deepcopy(session)
        def run():
            try:
                if action is not None:
                    result = action()
                elif geometry_snapshot is not None:
                    result = export_offline_artifact(request['method'], request.get('params', {}), inspector, geometry_snapshot)
                else:
                    result = export_offline_artifact(request['method'], request.get('params', {}), inspector)
                emit({'id': request.get('id'), 'result': result})
            except Exception as error:
                report = describe(error, request.get('method', ''), request.get('params'))
                print(f'[SerialArm diagnostic {report["diagnostic_id"]}]\n{traceback.format_exc()}', file=sys.stderr, flush=True)
                emit({'id': request.get('id'), 'error': report})
        export_job = threading.Thread(target=run, name='calibration-artifact-export', daemon=True)
        export_job.start()
    emit({'event': 'backend_ready'})
    if '--check' in sys.argv[1:]:
        model_runtime.close()
        native.close()
        manager.close()
        return
    def stop(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        for line in sys.stdin:
            request = {}
            try:
                if len(line) > 16 * 1024 * 1024: raise ValueError('request too large')
                request = json.loads(line)
                method, params = request['method'], request.get('params', {})
                if not isinstance(params, dict): raise ValueError('params must be an object')
                if len(line) > 1024 * 1024 and not (method in ('workbench_export', 'calibration_gain_validate_trajectory') and isinstance(params.get('response_window'), dict)):
                    raise ValueError('request too large')
                if method == 'stop' and native.status().get('state') in ('running', 'stopping'):
                    method = 'workbench_stop'
                if control_job and control_job.is_alive():
                    allowed = (method == 'status' or
                        (method == 'workbench_request' and params.get('method') in (_READ_ONLY_MACHINE | {'deactivate', 'hold'})) or
                        (method in ('workbench_stop', 'stop') and params.get('force') is True))
                    if not allowed:
                        raise ValueError('请先结束当前停放操作，期间仍可保持、立即失能或强制断开')
                    if ((method == 'workbench_request' and params.get('method') in ('hold', 'deactivate')) or
                            (method in ('workbench_stop', 'stop') and params.get('force') is True)):
                        control_cancel.set()
                # All potentially lengthy export and analysis routes share the
                # same admission rule, including retained IPC entrypoints
                if method in OFFLINE_CALIBRATION_METHODS and any(
                        session.status().get('state') in ('running', 'stopping')
                        for session in (native, manager)):
                    raise ValueError('请先安全断开当前运行会话，再处理离线标定成果')
                if method in ASYNC_EXPORT_METHODS:
                    start_export_job(request)
                    continue
                if method == 'status':
                    native_status = native.status()
                    session = native_status if native_status.get('state') in ('running', 'stopping') else manager.status()
                    result = {**inspector.status(), 'session': session, 'workbench': native_status}
                elif method == 'profiles': result = inspector.profiles(params)
                elif method == 'profile_library': result = profile_library.list()
                elif method == 'profile_source_inspect': result = profile_library.discover_profile_source(params.get('source', ''))
                elif method == 'profile_library_register': result = profile_library.register(params.get('profile_file', ''), params.get('profile', ''), params.get('resource_paths', ''), params.get('policy', 'error'))
                elif method == 'profile_library_remove': result = profile_library.remove(params.get('id', ''))
                elif method == 'description_inspect': result = profile_library.inspect_description(params.get('source', ''), params.get('model_relative', ''))
                elif method == 'description_create_profile': result = profile_library.create_profile(params)
                elif method == 'readiness': result = profile_library._readiness(config_values(params.get('config', params)))
                elif method == 'readiness_mark': result = profile_library.mark_stage(config_values(params.get('config', {})), params.get('stage', ''), params.get('confirmed', True) is True)
                elif method == 'readiness_record':
                    # Operator checklists are optional notes, not live-hardware
                    # acceptance or runtime permission. Native calibration
                    # capture endpoints enforce real feedback independently.
                    target = config_values(params.get('config', {}))
                    stage = params.get('stage', '')
                    linked_native = None
                    if (stage == 'gravity_verification' and active_workbench_config and
                            native.status().get('state') == 'running'):
                        target_info = inspector.inspect(target)
                        active_info = inspector.inspect(active_workbench_config)
                        if (target_info['fingerprint'] == active_workbench_fingerprint ==
                                active_info['fingerprint'] and
                                target_info['profile_file'] == active_info['profile_file'] and
                                target_info['profile'] == active_info['profile'] and
                                target_info['resources'] == active_info['resources']):
                            linked_native = native
                    result = profile_library.record_stage(
                        target, stage, params.get('answers', {}), linked_native)
                elif method == 'readiness_approve_mode': result = profile_library.approve_gain_mode(config_values(params.get('config', {})), params.get('mode', ''), params.get('candidate_path', ''), native, params.get('gain_purposes'))
                elif method == 'profile_editor_load': result = profile_library.profile_editor_load(config_values(params.get('config', params)))
                elif method == 'profile_editor_save': result = profile_library.profile_editor_save(config_values(params.get('config', {})), params.get('payload', {}))
                elif method == 'profile_hardware_inspect': result = profile_library.hardware_inspect(config_values(params.get('config', {})), params.get('plugin'))
                elif method == 'profile_hardware_import': result = profile_library.hardware_import(config_values(params.get('config', {})), params.get('path', ''), params.get('plugin'))
                elif method == 'inspect': result = inspector.inspect(params)
                elif method == 'model': result = model_runtime.model(inspector, params.get('config', params), params.get('positions'))
                elif method == 'model_preview': result = model_runtime.preview(inspector, params.get('config', params), params.get('positions'))
                elif method.startswith('geometry_repeat_'):
                    # Dedicated GUI -> Electron IPC -> Python Bridge -> native terminal route
                    # Do not trust a saved metadata label as proof of how a live trajectory was recorded
                    op = method.removeprefix('geometry_repeat_')
                    if op == 'status' and (not active_workbench_config or native.status().get('state') != 'running'):
                        result = {'phase':'idle', 'connected':False, 'supported':False, 'contact_ready':False}
                    else:
                        if not active_workbench_config or native.status().get('state') != 'running':
                            raise ValueError('定点重复测试需要先连接当前机械臂工作台')
                        requested = inspector.inspect(config_values(params.get('config', {})))
                        connected = inspector.inspect(active_workbench_config)
                        if (requested['profile'] != connected['profile'] or
                                requested['profile_file'] != connected['profile_file'] or
                                requested['resources'] != connected['resources'] or
                                connected['fingerprint'] != active_workbench_fingerprint):
                            repeat_source = None
                            raise ValueError('当前几何实验与正在运行的机械臂 Profile 不一致，请重新连接')
                        scope = trajectories.scope(connected, ROOT)
                        identity = (connected['profile'], connected['profile_file'], connected['fingerprint'])
                        status = native.request('teach_replay_status', {}, timeout=8)
                        capabilities = status.get('capabilities') or {}
                        supported = all(capabilities.get(k) is True for k in
                                        ('contact_first_capture', 'contact_cycle', 'endpoint_dwell'))
                        if not supported and op != 'status':
                            raise ValueError('当前运行的 C++ Terminal 尚不支持反向定点循环和终点停留，请重新运行 ./install.sh 重新编译 Terminal 后再连接工作台')
                        if op == 'status':
                            result = {**status, 'connected': True, 'supported': supported,
                                      'contact_ready': supported and repeat_source == ('ready', identity) and status.get('phase') == 'ready',
                                      'reason': '' if supported else '已连接的是旧版 C++ Terminal，请重新编译安装后重新连接'}
                        elif op == 'record_begin':
                            verify_machine_admission(profile_library, active_workbench_config, 'teach_replay_record_begin', {}, native)
                            repeat_source = None
                            result = native.request('teach_replay_record_begin', {}, timeout=8)
                            if result.get('phase') == 'recording':
                                repeat_source = ('recording', identity)
                        elif op == 'record_stop':
                            if repeat_source != ('recording', identity):
                                raise ValueError('当前没有从触点 B 开始的有效示教录制')
                            result = native.request('teach_replay_record_stop', {}, timeout=12)
                            repeat_source = ('ready', identity) if result.get('phase') == 'ready' else None
                        elif op == 'load':
                            folder, metadata = trajectories.ensure(scope, params.get('id', ''))
                            if metadata.get('kind') != 'contact_first':
                                raise ValueError('该轨迹未标记为触点 B 到安全点 A 的轨迹')
                            repeat_source = None
                            trajectories.load(scope, params.get('id', ''), native)
                            result = native.request('teach_replay_status', {}, timeout=8)
                            if result.get('phase') != 'ready':
                                raise ValueError('轨迹载入后原生回放状态不正确')
                            repeat_source = ('ready', identity)
                        elif op == 'save':
                            if repeat_source != ('ready', identity) or status.get('phase') != 'ready':
                                raise ValueError('请先完成从触点 B 到安全点 A 的示教或载入专用轨迹')
                            result = trajectories.save(scope, params.get('name', ''), native, kind='contact_first')
                        elif op == 'play':
                            if repeat_source != ('ready', identity) or status.get('phase') != 'ready':
                                raise ValueError('当前没有可执行的定点重复测试轨迹')
                            kind = params.get('kind')
                            if kind not in ('cycle', 'withdraw'):
                                raise ValueError('无效的几何回放类型')
                            machine_params = {
                                'direction': 'contact_cycle' if kind == 'cycle' else 'forward',
                                'cycles': params.get('cycles', 1) if kind == 'cycle' else 1,
                                'rate': params.get('rate', 0.2),
                                'mode': params.get('mode', 'RIGID_TRACKING'),
                                'endpoint_dwell_s': params.get('endpoint_dwell_s', 0) if kind == 'cycle' else 0,
                                'confirmed': params.get('confirmed') is True,
                            }
                            for field in ('reference_behavior', 'gain_profile'):
                                if field in params:
                                    machine_params[field] = params[field]
                            verify_machine_admission(profile_library, active_workbench_config, 'teach_replay_play', machine_params, native)
                            result = native.request('teach_replay_play', machine_params, timeout=8)
                        elif op == 'stop':
                            result = native.request('teach_replay_stop', {}, timeout=12)
                        else:
                            raise ValueError('未知的定点重复测试操作')
                elif method.startswith('geometry_fixed_'):
                    config = config_values(params.get('config', {}))
                    operation = method.removeprefix('geometry_fixed_')
                    if operation == 'state':
                        result = geometry.state(config)
                    elif operation == 'begin':
                        result = geometry.begin(config, params.get('frame', ''), params.get('purpose', 'diagnostic'))
                    elif operation == 'discard':
                        result = geometry.discard(config)
                    elif operation in ('capture', 'solve', 'validate'):
                        if operation == 'capture':
                            if not active_workbench_config or native.status().get('state') != 'running':
                                raise ValueError('采集前请连接当前机械臂工作台')
                            active_info = inspector.inspect(active_workbench_config)
                            requested_info = inspector.inspect(config)
                            if (active_info['profile_file'] != requested_info['profile_file'] or
                                    active_info['profile'] != requested_info['profile'] or
                                    active_info['resources'] != requested_info['resources'] or
                                    active_info['fingerprint'] != active_workbench_fingerprint):
                                raise ValueError('几何采样会话与当前真机连接不匹配')
                            previous = native.request('status', {}, timeout=8)
                            time.sleep(0.12)
                            current = native.request('status', {}, timeout=8)
                            result = geometry.capture(config, params.get('phase'), current, previous)
                        elif operation == 'solve':
                            result = geometry.solve(config)
                        else:
                            result = geometry.validate(config, params.get('rms_mm', 2.0), params.get('max_mm', 4.0))
                    elif operation == 'report':
                        result = geometry.export_diagnostic(config)
                    elif operation == 'preview_tcp':
                        result = geometry.preview_export(config)
                    elif operation == 'apply_tcp':
                        if params.get('confirmed') is not True: raise ValueError('请先预览 TCP 配置差异，再明确确认采用')
                        result = geometry.apply(config, native_running=any(session.status().get('state') in ('running','stopping') for session in (native,manager)),
                            destination=params.get('destination') or None, expected_hashes=params.get('expected_hashes'))
                    else:
                        raise ValueError('未知几何标定操作')
                elif method in ('trajectory_list','trajectory_select','trajectory_preview','trajectory_save','trajectory_load','trajectory_rename','trajectory_remove','trajectory_export','trajectory_import'):
                    config = active_workbench_config or config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    scope = trajectories.scope(info, ROOT)
                    record_id = params.get('id', '')
                    if method == 'trajectory_list': result = {'items': trajectories.list(scope), 'directory': str(trajectories.folder(scope))}
                    elif method == 'trajectory_preview':
                        start_export_job(request, lambda scope=scope, record_id=record_id, fraction=params.get('fraction', 0): trajectories.preview(scope, record_id, fraction))
                        continue
                    elif method == 'trajectory_select':
                        result = trajectories.select(scope, record_id)
                        metadata = result
                        if params.get('required_kind') and metadata.get('kind', 'general') != params['required_kind']:
                            raise ValueError('所选轨迹用途不匹配，请选择对应示教轨迹')
                    elif method == 'trajectory_import': result = trajectories.import_file(scope, params.get('source',''))
                    elif method == 'trajectory_export': result = trajectories.export(scope,record_id,params.get('destination',''))
                    elif method == 'trajectory_rename': result = trajectories.rename(scope,record_id,params.get('name',''))
                    elif method == 'trajectory_remove':
                        if native.status().get('state') == 'running' and (native.request('teach_replay_status',{},timeout=8).get('phase') in ('recording','playing')):
                            raise ValueError('请先结束示教或回放，再删除轨迹')
                        result = trajectories.remove(scope,record_id)
                    else:
                        if not active_workbench_config or native.status().get('state') != 'running':
                            raise ValueError('请先连接当前机械臂工作台')
                        if method == 'trajectory_save':
                            if params.get('kind', 'general') != 'general':
                                raise ValueError('触点起点专用轨迹必须通过几何标定的示教工作流保存')
                            result = trajectories.save(scope, params.get('name', ''), native, kind='general')
                        else:
                            repeat_source = None  # General trajectory loading is independent of the geometry workflow
                            if params.get('required_kind'):
                                _, metadata = trajectories.ensure(scope,record_id)
                                if metadata.get('kind','general') != params['required_kind']:
                                    raise ValueError('该轨迹不是从触点向安全位置示教的轨迹，请重新示教')
                            result = trajectories.load(scope,record_id,native)
                elif method == 'workbench_start':
                    if manager.status().get('state') in ('running', 'stopping'): raise ValueError('another Launcher session is already running')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    if not info['available'].get('workbench'): raise ValueError('structured workspace is unavailable; install Standalone terminal components')
                    admission = profile_library._readiness(config)
                    offline = info['write_enabled'] is False
                    allowed = ((admission.get('connection_readiness') or {}).get('offline_connectable') is True
                               if offline else (admission.get('connection_readiness') or {}).get('hardware_connectable') is True)
                    if not allowed:
                        issues = ((admission.get('connection_readiness') or {}).get('offline_issues') if offline else admission.get('setup_issues')) or []
                        detail = '；'.join(item['message'] for item in issues[:4]) if issues else (admission.get('reason') or 'Core 或硬件配置尚不完整')
                        raise ValueError('无法连接工作台，请先在 Profile 配置中完成必需项：' + detail)
                    if params.get('fingerprint') != info['fingerprint']: raise ValueError('configuration changed; inspect and confirm again')
                    if info['write_enabled'] is not False:
                        if params.get('confirmed') is not True: raise ValueError('real hardware confirmation is required')
                        if any(not c['ok'] for c in info['checks'] if c['name'] == 'device'):
                            raise ValueError('device is missing or not readable/writable')
                    argv = [inspector.machine_terminal(), '--machine', '--config', info['resources']['core']]
                    if not offline:
                        argv += ['--hardware-plugin', info['hardware_plugin'], '--hardware-config', info['resources']['hardware']]
                    for key, flag in [('serial_port', '--serial-port'), ('baudrate', '--baudrate'), ('bus', '--bus')]:
                        if config[key]: argv += [flag, config[key]]
                    env = dict(os.environ)
                    env['SERIAL_ARM_PROFILE_STORAGE'] = str(profile_home(info, ROOT) / '.serialarm')
                    if config['resource_paths']:
                        env['SERIAL_ARM_RESOURCE_PATH'] = config['resource_paths'] + (os.pathsep + env['SERIAL_ARM_RESOURCE_PATH'] if env.get('SERIAL_ARM_RESOURCE_PATH') else '')
                    result = native.start(argv, env, info['devices'] if info['write_enabled'] is not False else [])
                    active_workbench_config = config
                    active_workbench_fingerprint = info['fingerprint']
                    repeat_source = None
                elif method == 'workbench_request':
                    native_method = params.get('method')
                    machine_params = params.get('params', {})
                    if not isinstance(machine_params, dict): raise ValueError('控制参数必须是对象')
                    if not active_workbench_config: raise ValueError('没有已绑定 Profile 的控制会话')
                    if native_method not in _READ_ONLY_MACHINE and native_method not in _SAFE_EXIT_MACHINE:
                        info = inspector.inspect(active_workbench_config)
                        if info['fingerprint'] != active_workbench_fingerprint:
                            raise ValueError('连接期间配置发生变化，请先安全断连并重新检查')
                        verify_machine_admission(profile_library, active_workbench_config, native_method, machine_params, native)
                    if native_method == 'park':
                        start_control_job(request, lambda machine_params=machine_params: native.request('park', machine_params, timeout=120, cancel_event=control_cancel))
                        continue
                    result = native.request(native_method, machine_params, timeout=45 if native_method in ('model_calibration_teach_stop','model_calibration_import_trajectory') else 8)
                    if native_method in ('teach_replay_record_begin', 'teach_replay_record_stop'):
                        repeat_source = None  # Normal teaching cannot be mistaken for a contact-first recording
                elif method == 'workbench_stop':
                    # Temporary overrides are process-local, but restore explicitly
                    # before disconnect so the outcome is observable and auditable
                    # When safe parking or restoration fails keep the session alive
                    force = params.get('force') is True
                    disconnect_config = active_workbench_config
                    def disconnect(force=force, disconnect_config=disconnect_config):
                        nonlocal active_workbench_config, active_workbench_fingerprint, repeat_source
                        try:
                            if not force and native.status().get('state') == 'running':
                                admission = profile_library._readiness(disconnect_config) if disconnect_config else {}
                                safe_park = (admission.get('control_available') is True and
                                    all(admission.get('modes', {}).get(mode) in ('configured', 'verified', 'stale')
                                        for mode in ('RIGID_TRACKING', 'COMPLIANT_TRACKING')))
                                state_before = native.request('status', {}, timeout=8)
                                if (state_before.get('teach_replay') or {}).get('phase') in ('recording','playing'):
                                    raise ValueError('请先在示教回放页面停止当前任务，再断开工作台')
                                if state_before.get('robot_state') == 'ACTIVE' and not safe_park:
                                    raise ValueError('停放轨迹尚未通过检查，请先可靠支撑机械臂并在控制工作台执行立即失能，再断开连接')
                                restore_temporary_candidates_for_disconnect(native)
                            result = native.stop(force, cancel_event=control_cancel if not force else None)
                        except Exception as error:
                            native.note_disconnect_failure(error)
                            raise
                        active_workbench_config = None
                        active_workbench_fingerprint = None
                        repeat_source = None
                        return result
                    if not force:
                        start_control_job(request, disconnect)
                        continue
                    result = disconnect()
                elif method == 'workbench_config_preview':
                    config = config_values(params.get('config', {}))
                    info, runtime, snapshot = runtime_parameter_snapshot(
                        inspector, native, config, active_workbench_config, active_workbench_fingerprint)
                    result = preview_config(info['resources']['core'], snapshot['joint_names'], runtime, snapshot['gravity_scale'])
                elif method == 'workbench_config_save':
                    config = config_values(params.get('config', {}))
                    info, runtime, snapshot = runtime_parameter_snapshot(
                        inspector, native, config, active_workbench_config, active_workbench_fingerprint)
                    result = save_config(info['resources']['core'], params.get('expected_sha', ''), snapshot['joint_names'], runtime, snapshot['gravity_scale'], profile_home(info, ROOT))
                    # The saved values came from this exact live session; refresh
                    # its disk identity without making the next command stale
                    active_workbench_fingerprint = inspector.inspect(active_workbench_config)['fingerprint']
                elif method == 'workbench_export':
                    metadata = {'profile': params.get('profile', ''), 'core': params.get('core', '')}
                    events = native.telemetry()
                    response = params.get('response_window')
                    if response is not None:
                        if (not isinstance(response, dict) or response.get('recording') is not False or
                                not isinstance(response.get('samples'), list) or not 1 <= len(response['samples']) <= 6000 or
                                not isinstance(response.get('joint_names'), list) or
                                response.get('source') != 'passive_native_cycle_snapshots'):
                            raise ValueError('响应记录必须为已结束且包含有效快照的被动记录')
                        # Preserve the operator's release annotation and source session,
                        # including when the live session has since disconnected.
                        json.dumps(response, allow_nan=False)
                        from interaction_response import analyze_interaction_response
                        response['execution_dynamics'] = analyze_interaction_response(response)
                        metadata['response_window'] = {key: value for key, value in response.items() if key != 'samples'}
                        events = [{'event': 'response_snapshot', 'data': sample} for sample in response['samples']]
                    result = export_telemetry(profile_home(inspector.inspect(active_workbench_config), ROOT) if active_workbench_config else ROOT, events, metadata)
                    if response is not None:
                        result['interaction_response'] = response['execution_dynamics']
                elif method == 'calibration_candidate_prepare':
                    from workflow_registry import prepare_gain_candidate, prepare_gravity_candidate, prepare_dynamics_candidate
                    kind = params.get('kind')
                    if kind == 'gain': result = prepare_gain_candidate(params.get('directory', ''), gain_purposes=params.get('gain_purposes'))
                    elif kind == 'gravity': result = prepare_gravity_candidate(params.get('directory', ''), prefer_recomputed=params.get('prefer_recomputed') is True, candidate_path=params.get('candidate_path'), candidate_source=params.get('candidate_source'))
                    elif kind == 'dynamics': result = prepare_dynamics_candidate(params.get('report', ''))
                    else: raise ValueError('未知的候选类型')
                elif method == 'calibration_candidate_apply':
                    if not active_workbench_config or native.status().get('state') != 'running':
                        raise ValueError('候选只能在绑定的工作台会话内临时应用')
                    active_info = inspector.inspect(active_workbench_config)
                    if active_info['fingerprint'] != active_workbench_fingerprint:
                        raise ValueError('配置已变化，请先安全断连并重新检查')
                    verify_machine_admission(profile_library, active_workbench_config,
                                             'candidate_' + str(params.get('kind', '')) + '_apply', {}, native)
                    from workflow_registry import prepare_gain_candidate, prepare_gravity_candidate, prepare_dynamics_candidate
                    kind = params.get('kind')
                    if kind == 'gain': result_path=prepare_gain_candidate(params.get('directory', ''), gain_purposes=params.get('gain_purposes'))['candidate_path'];native_args={'path':result_path, **({'gain_purposes':params['gain_purposes']} if params.get('gain_purposes') is not None else {})}
                    elif kind == 'gravity': result_path=prepare_gravity_candidate(params.get('directory', ''), prefer_recomputed=params.get('prefer_recomputed') is True, candidate_path=params.get('candidate_path'), candidate_source=params.get('candidate_source'))['candidate_path'];native_args={'path':result_path}
                    elif kind == 'dynamics': result_path=prepare_dynamics_candidate(params.get('report', ''))['report'];native_args={'report':result_path}
                    else: raise ValueError('未知的候选类型')
                    native.request('candidate_'+kind+'_apply', native_args, timeout=12)
                    result=native.request('candidate_overrides',{},timeout=8)
                elif method == 'calibration_candidate_restore':
                    if not active_workbench_config or native.status().get('state') != 'running':
                        raise ValueError('当前没有可恢复候选的工作台会话')
                    kind = params.get('kind')
                    if kind not in ('gain','gravity','dynamics'): raise ValueError('未知的候选类型')
                    native.request('candidate_'+kind+'_restore', {}, timeout=12)
                    result=native.request('candidate_overrides',{},timeout=8)
                elif method == 'calibration_candidate_status':
                    result = native.request('candidate_overrides', {}, timeout=8)
                elif method in ('calibration_tool_records','calibration_tool_records_page'):
                    from workflow_registry import scan, scan_page
                    offline_config = params.get('config') or active_workbench_config or {}
                    selected = config_values(offline_config)
                    scanner = scan_page if method.endswith('_page') else scan
                    result = scanner(offline_profile_home(inspector, selected), params.get('kind', ''), params.get('limit', 60),
                                  offset=params.get('offset',0), query=params.get('query',''), legacy_root=ROOT, profile_name=selected['profile'])
                elif method == 'calibration_candidates':
                    from workflow_registry import candidates
                    result = candidates(params.get('directory',''),params.get('kind',''))
                elif method == 'calibration_gain_validate_trajectory':
                    from workflow_registry import validate_gain_trajectory
                    start_export_job(request, lambda directory=params.get('directory',''), window=params.get('response_window'), purpose=params.get('purpose'): validate_gain_trajectory(directory, window, purpose))
                    continue
                elif method == 'calibration_tool_load':
                    from workflow_registry import load
                    result = load(params.get('directory', ''), params.get('kind', ''), candidate_path=params.get('candidate_path'), report=params.get('report'))


                elif method == 'model_calibration_load':
                    if params.get('candidate_path') or params.get('report'):
                        from workflow_registry import load
                        result = load(params.get('directory',''), params.get('kind','gravity'), candidate_path=params.get('candidate_path'), report=params.get('report'))
                    else:
                        result = load_model_calibration_summary(params.get('directory', ''))
                elif method == 'model_calibration_model_alignment':
                    directory = params.get('directory', '')
                    summary = load_model_calibration_summary(directory)
                    current_model = model_runtime.model(inspector, params.get('config', {}))
                    result = compare_calibration_urdfs(summary['directory'], summary['metadata'], current_model['urdf'])
                elif method == 'model_calibration_records':
                    offline_config = params.get('config') or active_workbench_config or {}
                    selected = config_values(offline_config)
                    result = list_model_calibration_records(offline_profile_home(inspector, selected),
                                                           legacy_root=ROOT, profile_name=selected['profile'])
                elif method == 'model_calibration_preview_save':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = preview_gravity_correction(info['resources']['core'], directory, candidate_source=params.get('candidate_source'), candidate_path=params.get('candidate_path'))
                elif method == 'model_calibration_save':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    directory = params.get('directory', '')
                    if not directory and native.status().get('state') == 'running':
                        directory = native.request('model_calibration_status', {}, timeout=8).get('directory', '')
                    result = save_gravity_correction(info['resources']['core'], params.get('expected_sha', ''), directory, profile_home(info, ROOT), candidate_source=params.get('candidate_source'), candidate_path=params.get('candidate_path'))
                elif method == 'model_calibration_restore_config':
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    result = restore_gravity_correction(info['resources']['core'], params.get('directory', ''))


                elif method == 'model_calibration_assess':
                    from advanced_identification import assess_record
                    directory = params.get('directory', '')
                    result = assess_record(directory)
                elif method == 'model_calibration_advanced':
                    from advanced_identification import MODES
                    directory = params.get('directory', '')
                    mode = params.get('mode', '')
                    if mode not in MODES:
                        raise ValueError('未知的高级辨识目标')
                    locked = params.get('locked_links', [])
                    if not isinstance(locked,list) or any(not isinstance(x,str) or not x for x in locked) or len(set(locked))!=len(locked):
                        raise ValueError('需要不重复的 Link 名称清单')
                    if not directory or not Path(directory).expanduser().is_dir():
                        raise ValueError('请选择已保存并完成标定的历史任务')
                    result = run_full_inertial_isolated(directory, params.get('destination') or None,
                        mode=mode, locked_links=locked)
                elif method == 'model_calibration_export_full_inertial':
                    raise ValueError('完整惯量候选已归入高级动力学辨识，请选择目标参数与可信 Link 后再执行离线辨识')
                elif method == 'model_calibration_idim_audit':
                    from calibration_jobs import audit_dynamics_isolated
                    result = audit_dynamics_isolated(params.get('directory', ''))
                elif method == 'model_calibration_idim':
                    from calibration_jobs import identify_dynamics_isolated
                    result = identify_dynamics_isolated(params.get('directory', ''),
                        mode=params.get('mode', 'global'), locked_links=params.get('locked_links', []),
                        validation_directory=params.get('validation_directory') or None,
                        estimator=params.get('estimator', 'WLS'), cutoff_hz=params.get('cutoff_hz', 5.),
                        fixed_joint_positions=params.get('fixed_joint_positions') or {},load_description=params.get('load_description')or None)

                elif method == 'model_calibration_recompute':
                    directory = params.get('directory', '')
                    if not directory or not Path(directory).expanduser().is_dir():
                        raise ValueError('请选择完整的标定任务目录')
                    result = recompute_gravity_isolated(directory, params.get('mode', 'global'),
                        params.get('locked_links', []), objective=params.get('objective', 'bias_separated'))
                elif method == 'start':
                    if native.status().get('state') in ('running', 'stopping'): raise ValueError('another Launcher session is already running')
                    mode = params.get('mode')
                    config = config_values(params.get('config', {}))
                    info = inspector.inspect(config)
                    if not info['available'].get(mode): raise ValueError((info.get('unavailable_reasons') or {}).get(mode) or 'selected run mode is unavailable; inspect installation and resources')
                    if params.get('fingerprint') != info['fingerprint']: raise ValueError('configuration changed; inspect and confirm again')
                    if params.get('confirmed') is not True: raise ValueError('real hardware confirmation is required')
                    if info['write_enabled'] is not False:
                        admission = profile_library._readiness(config)
                        if not admission.get('control_available'):
                            raise ValueError('真实硬件控制需要完整有效的 Core、硬件映射与 Safety 参数')
                    if any(not c['ok'] for c in info['checks'] if c['name'] == 'device'):
                        raise ValueError('device is missing or not readable/writable')
                    config['profile_file'] = info['profile_file']
                    argv, extra = command_for(mode, config)
                    launch_env = inspector.launch_environment(mode)
                    result = manager.start(argv, dict(launch_env, **extra, TERM='xterm-256color'),
                                           info['devices'], mode)
                elif method == 'input': result = manager.input(params.get('data'))
                elif method == 'resize': result = manager.resize(params.get('cols'), params.get('rows'))
                elif method == 'stop':
                    result = native.stop(params.get('force') is True) if native.status().get('state') in ('running', 'stopping') else manager.stop(params.get('force') is True)
                elif method == 'close':
                    if export_job and export_job.is_alive(): raise ValueError('成果正在导出，请完成后再关闭工作台')
                    if any(session.status().get('state') in ('running', 'stopping') for session in (native, manager)):
                        raise ValueError('请先安全停止当前运行会话，再关闭后台')
                    # Reply before leaving the loop so a rejected shutdown keeps
                    # the IPC connection available for recovery
                    native.close()
                    manager.close()
                    model_runtime.close()
                    emit({'id': request.get('id'), 'result': True})
                    return
                else: raise ValueError('unknown IPC method')
                emit({'id': request.get('id'), 'result': result})
            except Exception as error:
                report = describe(error, request.get('method', '') if isinstance(request, dict) else '',
                                  request.get('params') if isinstance(request, dict) else None)
                print(f'[SerialArm diagnostic {report["diagnostic_id"]}]\n{traceback.format_exc()}', file=sys.stderr, flush=True)
                emit({'id': request.get('id') if isinstance(request, dict) else None, 'error': report})
    except KeyboardInterrupt: pass
    finally:
        if control_job: control_job.join()
        model_runtime.close()
        native.close()
        manager.close()


if __name__ == '__main__': main()
