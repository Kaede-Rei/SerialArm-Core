"""Offline identification of accepted-reference to measured-position response."""
import math

import numpy as np


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _value(sample, key, joint):
    values = sample.get(key)
    return values[joint] if isinstance(values, list) and joint < len(values) and _number(values[joint]) else None


def _empty(name, reason):
    return dict(joint=name, identified=False, reason=reason, delay_s=None, bandwidth_hz=None,
                natural_frequency_hz=None, damping_ratio=None, validation_nrmse=None,
                excitation_range_rad=None, static_gain=None, stable=None)


def verify_return_response(window):
    result = dict(target_unchanged=None, joint_pass=None, external_tcp_pass=None,
                  maximum_error_rad=None, tcp_error_m=None, reason='',
                  reference_source='first_recorded_nominal', stable_dwell_s=0.5,
                  external_measurement_source=None)
    external = window.get('external_tcp_measurement')
    if external is not None:
        if not isinstance(external, dict) or external.get('source') != 'manual_external_measurement':
            raise ValueError('针尖回位验证必须为明确标记的外部测量，不能以模型 FK 代替')
        delta, tolerance = external.get('displacement_m'), external.get('tolerance_m')
        if (not isinstance(delta, list) or len(delta) != 3 or not all(_number(v) for v in delta) or
                not _number(tolerance) or tolerance <= 0):
            raise ValueError('外部针尖测量需提供三个有限位移和正的允许误差')
        result['tcp_error_m'] = math.sqrt(sum(v * v for v in delta))
        result['external_tcp_pass'] = result['tcp_error_m'] <= tolerance
        result['external_measurement_source'] = external['source']
    samples = window.get('samples') or []
    names = window.get('joint_names') or []
    release = window.get('release_time_s')
    tolerance = (window.get('summary') or {}).get('tolerance')
    tolerances = window.get('return_tolerance')
    if tolerances is None and _number(tolerance): tolerances = [tolerance] * len(names)
    if (not _number(release) or not isinstance(tolerances, list) or len(tolerances) != len(names) or
            any(not _number(v) or v <= 0 for v in tolerances)):
        result['reason'] = '需记录释放时刻和关节回位稳定带'
        return result
    if not samples or not names or any(not isinstance(s, dict) or not _number(s.get('time_s')) for s in samples):
        result['reason'] = '缺少有效关节响应记录'
        return result
    times = [s['time_s'] for s in samples]
    if any(b <= a for a, b in zip(times, times[1:])) or not times[0] <= release <= times[-1] - 0.5:
        result['reason'] = '需保留释放后至少 0.5 s 的连续记录'
        return result
    if any(_value(s, 'nominal', i) is None or _value(s, 'pos', i) is None
           for s in samples for i in range(len(names))):
        result['reason'] = '缺少原始名义目标或实测位置'
        return result
    reference = samples[0]['nominal']
    result['target_unchanged'] = all(abs(s['nominal'][i] - reference[i]) <= 1e-7
                                     for s in samples for i in range(len(names)))
    if not result['target_unchanged']:
        result['joint_pass'] = False
        result['reason'] = '记录期间名义目标发生变化，不能将重锚位置认定为回到原目标'
        return result
    tail = [s for s in samples if s['time_s'] >= times[-1] - 0.5]
    typical = float(np.median(np.diff(times)))
    if len(tail) < 3 or tail[0]['time_s'] > times[-1] - 0.5 + typical * 1.5 or max(np.diff(times)) > max(.1, typical * 3):
        result['reason'] = '释放后有效反馈不足以验证连续稳定回位'
        return result
    if sum(s.get('new_measurement') is False for s in tail) > len(tail) * 0.1:
        result['reason'] = '释放后记录含过多重复反馈'
        return result
    errors = [max(abs(s['pos'][i] - reference[i]) for s in tail) for i in range(len(names))]
    units = window.get('joint_units') or ['rad'] * len(names)
    if len(units) != len(names): units = ['joint_unit'] * len(names)
    result['joint_errors'] = [dict(joint=name, maximum_error=float(errors[i]), unit=units[i],
                                    tolerance=tolerances[i], passed=errors[i] <= tolerances[i])
                              for i, name in enumerate(names)]
    result['maximum_error_rad'] = float(max(errors)) if all(unit == 'rad' for unit in units) else None
    result['maximum_error_joint_units'] = float(max(errors))
    result['joint_pass'] = all(error <= allowed for error, allowed in zip(errors, tolerances))
    result['reason'] = ('关节已在原目标稳定带内连续保持 0.5 s' if result['joint_pass'] else
                        '关节释放后未回到原目标稳定带')
    return result


def _identify(samples, index, name, dt):
    result = _empty(name, '')
    if any(_value(s, 'accepted', index) is None or _value(s, 'pos', index) is None for s in samples):
        result['reason'] = '缺少 accepted reference 或实测位置'
        return result
    if any(s.get('torque_limited') or any(s.get('torque_limit_flags', [])) for s in samples):
        result['reason'] = '记录含力矩限幅，请使用未饱和的跟踪记录'
        return result
    external = [abs(_value(s, 'external', index)) for s in samples if _value(s, 'external', index) is not None]
    effort = [_value(s, 'effort_limit', index) for s in samples if _value(s, 'effort_limit', index) is not None]
    if external and max(external) > max(0.2, 0.02 * min(effort) if effort else 0.2):
        result['reason'] = '记录存在明显外力，请另录无接触跟踪段分析执行动态'
        return result
    times = np.array([s['time_s'] for s in samples], dtype=float)
    uniform = np.arange(times[0], times[-1] + dt * 0.1, dt)
    u = np.interp(uniform, times, [_value(s, 'accepted', index) for s in samples])
    y = np.interp(uniform, times, [_value(s, 'pos', index) for s in samples])
    span = float(np.ptp(u))
    result['excitation_range_rad'] = span
    if span < 0.01 or np.std(y) < 1e-4:
        result['reason'] = '参考变化不足，至少需要 0.01 rad 的多次往返跟踪'
        return result
    # An affine offset absorbs static tracking error; it must not become a pole.
    u = u - np.mean(u)
    y = y - np.mean(y)
    split = int(len(y) * 0.7)
    scale = float(np.std(y))
    best = None
    for delay in range(min(12, len(y) // 20) + 1):
        start = delay + 2
        k = np.arange(start, split)
        design = np.column_stack((y[k - 1], y[k - 2], u[k - delay],
                                  u[k - delay - 1], u[k - delay - 2], np.ones(len(k))))
        coefficients, _, rank, singular = np.linalg.lstsq(design, y[k], rcond=None)
        if rank < 6 or singular[-1] / singular[0] < 1e-9:
            continue
        a1, a2, b0, b1, b2, offset = coefficients
        poles = np.roots([1, -a1, -a2])
        if np.any(np.abs(poles) >= 1) or np.any(np.abs(poles) < 1e-8):
            continue
        predicted = y.copy()
        for j in range(split, len(y)):
            predicted[j] = (a1 * predicted[j - 1] + a2 * predicted[j - 2] +
                            b0 * u[j - delay] + b1 * u[j - delay - 1] + b2 * u[j - delay - 2] + offset)
        error = float(np.sqrt(np.mean((predicted[split:] - y[split:]) ** 2)) / scale)
        if best is None or error < best[0]:
            best = (error, delay, coefficients, poles)
    if best is None:
        result['reason'] = '无法辨识稳定的二阶执行模型，请增加不同速度的往返参考'
        return result
    error, delay, coefficients, poles = best
    result['validation_nrmse'] = error
    if error > 0.25:
        result['reason'] = '独立留出段预测误差过大，请保持控制参数不变并增加跟踪数据'
        return result
    a1, a2, b0, b1, b2, _ = coefficients
    gain = float((b0 + b1 + b2) / (1 - a1 - a2))
    frequencies = np.linspace(0, 0.5 / dt, 4097)
    z = np.exp(-2j * np.pi * frequencies * dt)
    response = np.abs((b0 + b1 * z + b2 * z ** 2) / (1 - a1 * z - a2 * z ** 2))
    crossings = np.flatnonzero(response < abs(gain) / math.sqrt(2))
    if gain <= 0 or not len(crossings) or frequencies[crossings[0]] >= 0.4 / dt:
        result['reason'] = '采样率未覆盖可解析的执行带宽，请使用更高频率的跟踪记录'
        return result
    continuous = np.log(poles.astype(complex)) / dt
    # The dominant pole pair describes the fitted second-order response only.
    dominant = continuous[np.argmax(continuous.real)]
    natural = float(abs(dominant))
    result.update(identified=True, reason='二阶执行模型，按时间顺序留出后 30% 验证',
                  delay_s=float(delay * dt), bandwidth_hz=float(frequencies[crossings[0]]),
                  natural_frequency_hz=natural / (2 * math.pi),
                  damping_ratio=float(-dominant.real / natural), static_gain=gain, stable=True)
    return result


def analyze_interaction_response(window):
    """Analyze a completed passive window without issuing a robot command."""
    samples = window.get('samples', [])
    names = window.get('joint_names', [])
    result = dict(source='passive_closed_loop_identification', sample_count=len(samples),
                  duration_s=0.0, sample_dt_s=None, joints=[],
                  return_verification=verify_return_response(window))
    reason = None
    if not samples or any(not isinstance(s, dict) or not _number(s.get('time_s')) for s in samples):
        reason = '缺少有效的控制周期时间戳'
    else:
        times = np.array([s['time_s'] for s in samples], dtype=float)
        gaps = np.diff(times)
        result['duration_s'] = float(times[-1] - times[0])
        if len(gaps) and np.any(gaps <= 0):
            reason = '控制周期时间戳必须严格递增'
        elif len(samples) < 120 or result['duration_s'] < 1:
            reason = '记录不足，需要至少 120 个快照且持续 1 s'
        else:
            dt = float(np.median(gaps))
            result['sample_dt_s'] = dt
            if np.max(gaps) > 3 * dt or np.max(np.abs(gaps - dt)) > 0.5 * dt:
                reason = '快照间隔不连续，无法可靠估计执行延迟和带宽'
            elif sum(s.get('new_measurement') is False for s in samples) > len(samples) * 0.1:
                reason = '记录中重复反馈过多，请使用持续收到新反馈的跟踪段'
            elif window.get('impedance_mode') == 'COMPLIANT_DRAG':
                reason = '原生拖拽不提供位置跟踪激励，请另录无接触跟踪段'
    result['joints'] = ([_empty(name, reason) for name in names] if reason else
                        [_identify(samples, i, name, result['sample_dt_s']) for i, name in enumerate(names)])
    return result
