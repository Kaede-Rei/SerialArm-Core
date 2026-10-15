'use strict';
// UI-only localization. Original diagnostics remain accessible in the
// "原始技术诊断" expandable section, and backend protocols remain stable.
(function (root) {
    const explanations = [
        [/model calibration stopped by safety fault/i, '示教或回放已被机械臂安全故障中断 不会自动重新使能，请先检查故障和机械臂支撑状态'],
        [/MOTOR_BUS_ACTIVATE_FAILED|ENABLE_FAILED|ACTUATOR_OFFLINE/i, '执行器使能失败或电机离线 请检查电机供电、急停、CAN 连接和电机状态；供电恢复后可清除启动故障并重新使能'],
        [/MOTOR_BUS_CONNECT_FAILED|OPEN_FAILED|NOT_CONNECTED/i, '执行器通信连接失败 请检查 USB-CAN 设备、串口权限、线缆和电机供电'],
        [/MOTOR_BUS_READ_FAILED|READ_FAILED/i, '执行器反馈读取失败 请检查通信链路和电机供电'],
        [/MOTOR_BUS_WRITE_FAILED|WRITE_FAILED/i, '执行器命令写入失败 请检查通信链路、电源和电机状态'],
        [/MOTOR_BUS_DEACTIVATE_FAILED|DISABLE_FAILED/i, '执行器失能未被全部确认 请先确保机械臂得到支撑，再检查电机供电和通信'],
        [/FAULT_RECOVERY_NOT_ALLOWED/i, '当前故障尚不满足安全恢复条件，请确认电机在线、姿态稳定，并检查故障状态'],
        [/\bFAULTED\b/i, '机器人处于故障锁存状态 请检查原因，再清除故障；未完成安全恢复前不能使能'],
        [/\bNOT_FAULTED\b/i, '当前机器人不处于故障状态，无需清除故障'],
        [/\bALREADY_ACTIVE\b/i, '机器人已经使能，无需重复使能'],
        [/\bNOT_ACTIVE\b/i, '机器人尚未使能，请先检查供电并使能'],
        [/\bNOT_INACTIVE\b/i, '此操作要求机器人先处于失能状态'],
        [/\bWRITE_DISABLED\b/i, '当前配置禁止向真实执行器下发命令'],
        [/\bCMD_TIMEOUT\b/i, '控制命令更新超时 请检查控制周期和命令续发情况，不要放宽安全超时'],
        [/\bINVALID_DT\b/i, '控制周期超出安全限制 请检查系统负载、进程调度和通信延迟'],
        [/\bJOINT_POS_LIMIT\b/i, '关节位置超出安全范围 请检查关节姿态与零位配置'],
        [/\bJOINT_VEL_LIMIT\b/i, '实测关节速度超出安全限制'],
        [/\bCMD_VEL_LIMIT\b/i, '下发的关节速度超过安全限制'],
        [/\bSAFETY_FAILED\b/i, '机器人安全检查未通过，请核对关节状态、反馈和控制限制'],
        [/calibration replay initial reference exceeds hard following limit/i, '自动回放起始参考位置与实测关节位置相差过大，已拒绝发送运动指令 请检查机械臂当前姿态和示教轨迹起点'],
        [/calibration replay hard following limit reached/i, '自动回放触及硬跟踪误差边界，已停止回放 请检查关节执行器和轨迹是否存在异常'],
        [/calibration frozen gain verification following limit exceeded/i, '冻结增益验收的实际跟踪误差超过允许的验收边界，当前候选未通过全速或指定柔性速度测试 这不是触发电机硬安全上限，需返回增益搜索检查跟踪能力'],
        [/calibration replay soft following recovery timed out/i, '自动回放已暂停等待关节追上参考位置，但在限定时间内未恢复 软阈值不是硬安全上限，请检查增益、摩擦、负载和当前是否处于冻结验收阶段'],
        [/calibration tracking error exceeded safe following limit/i, '自动回放的实际位置持续无法跟上参考轨迹，已按安全限制中止'],
        [/gain adaptation.*(without|not).*\b(verification|validated|clean|converge)/i, '增益自适应未达到全路径验证要求 候选参数已保留，但不能直接应用'],
        [/reverse static replay failed or cancelled/i, '反向自动采集失败或已取消，请查看具体关节和控制故障'],
        [/forward.*replay failed or cancelled/i, '正向自动采集失败或已取消'],
        [/cannot enter RIGID_HOLD/i, '无法切换到刚性保持模式，请检查机器人状态'],
        [/cannot.*(enter|switch).*tracking/i, '无法切换到跟踪控制模式，请检查机器人状态及安全限制'],
        [/no identification data were accepted/i, '没有通过验收的有效辨识数据，不能生成可靠的模型参数'],
        [/native session (?:request timed out|is unresponsive)/i, '原生控制会话已失去响应，上一条控制命令的结果未知，请先确保机械臂硬件安全，然后点击恢复连接并重新检查供电'],
        [/timed out|timeout/i, '操作等待超时，请检查硬件连接、系统负载或对应任务的诊断记录'],
        [/Unknown launcher request/i, '前后端接口不匹配，当前 GUI 请求未在 Electron 主进程注册'],
        [/backend.*(unavailable|exited)|native session closed/i, '后台进程已退出或未就绪，请检查安装与运行日志'],
        [/device is missing or not readable\/writable/i, '找不到设备或无读写权限，请检查 USB 连接与权限'],
        [/invalid Safety configuration/i, '机器人安全配置无效，请核对安全限额与关节映射'],
        [/failed to load|cannot load/i, '资源或任务记录读取失败，请检查文件路径和内容'],
        [/failed to save|cannot save/i, '保存失败，请检查目录权限与磁盘空间'],
        [/failed to export|export failed/i, '导出失败，请检查候选审核状态、文件权限与模型一致性'],
        [/blocked while a calibration task is active/i, '标定任务正在执行中，当前操作已被阻止 请先结束或取消标定任务'],
        [/calibration task is not initialized/i, '当前尚未创建标定任务，请先完成示教或加载已有记录'],
        [/model calibration is not waiting for replay confirmation/i, '标定任务尚未进入回放确认阶段'],
        [/model calibration teaching is not active/i, '当前示教已停止或被安全中断，无需重复结束 请查看任务状态，必要时点击清理中断示教'],
        [/cannot stop teaching while a calibration replay is active/i, '示教已经结束，当前正在执行自动回放 请使用取消任务操作而不是结束示教'],
        [/model calibration teaching stopped|teaching stopped because|teaching interrupted before normal stop/i, '示教因机械臂状态变化而中断，不会自动回放 已录制数据可在任务目录中检查'],
        [/stale or missing model calibration task_id/i, '当前操作引用了旧任务，请刷新任务状态后重试，不会重新启动机械臂'],
        [/demonstration trajectory is too short/i, '示教轨迹过短，无法进行回放规划 已关闭当前采集，可重新示教'],
        [/demonstration trajectory exceeds safe replay range/i, '示教轨迹过于接近关节限位，已拒绝回放 请重新示教更保守的路径'],
        [/explicit released confirmation is required/i, '请先确认已完全松开机械臂并检查运行区域'],
        [/operation.*cancelled|cancelled/i, '操作已取消'],
        [/invalid.*configuration|configuration.*invalid/i, '配置内容无效，请检查 Profile、硬件参数和模型'],
    ];
    const actionNames = {
        activate: '使能', deactivate: '失能', clear_fault: '清除故障',
        park: '停放', hold: '当前位置保持', fault_compliant: '柔性恢复',
        fault_rigid: '刚性恢复', set_impedance_mode: '切换阻抗模式',
        set_model_feedforward_mode: '切换模型前馈模式',
    };
    const codePattern = /\b(?:MOTOR_BUS_[A-Z_]+|ACTUATOR_[A-Z_]+|CMD_TIMEOUT|INVALID_DT|JOINT_[A-Z_]+|FAULT_RECOVERY_NOT_ALLOWED|FAULTED|NOT_FAULTED|ALREADY_ACTIVE|NOT_ACTIVE|NOT_INACTIVE|SAFETY_FAILED|WRITE_DISABLED|ENABLE_FAILED|DISABLE_FAILED|OPEN_FAILED|TIMEOUT|EPIPE)\b/g;
    function translate(raw, lang = 'zh-CN') {
        const input = String(raw ?? '').trim();
        if (lang === 'en' || !input) return input;
        const clean = input
            .replace(/^(?:Error invoking remote method ['"]launcher:request['"]:\s*)?Error:\s*/i, '')
            .replace(/^(?:操作失败|错误)[:：]\s*/, '').trim();
        // Leave fully localized backend diagnostics intact.
        if (!/[A-Za-z]{3,}/.test(clean)) return clean;
        const actionMatch = clean.match(/^([a-z_]+) failed:\s*/i);
        const action = actionMatch && actionNames[actionMatch[1]] ? `${actionNames[actionMatch[1]]}失败 ` : '';
        const matched = explanations.find(([pattern]) => pattern.test(clean));
        const numbers = [...clean.matchAll(/\b(?:value|limit|position error|speed|timeout)\s*=\s*[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?(?:\s*(?:rad(?:\/s)?|ms|s))?/gi)]
            .map(m => m[0].replace(/^value=/i, '实测值=').replace(/^limit=/i, '限制值=').replace(/^position error=/i, '位置误差=').replace(/^speed=/i, '速度=').replace(/^timeout=/i, '超时='));
        const codes = [...new Set((clean.match(codePattern) || []))];
        const replayFields = [...clean.matchAll(/\b(?:joint|actual|reference|error|soft_limit|hard_limit|kp|kd|adaptive|phase|recovery_elapsed_s|velocity|validation_pause_limit|error_ratio|validation)\s*=\s*(?:[A-Za-z_][\w-]*|[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?)/gi)].map(m => m[0]);
        const diagnostic = [...new Set([...codes, ...numbers, ...replayFields])].join('，');
        if (matched) return `${action}${matched[1]}${diagnostic ? `（${diagnostic}）` : ''}`;
        const zhPortion = clean.match(/[\u3400-\u9fff][^\n]*/);
        if (zhPortion) return `${action}${zhPortion[0]}${diagnostic ? `（${diagnostic}）` : ''}`;
        return `${action || '操作未完成 '}后端返回了尚未分类的诊断信息，请展开原始技术诊断核对具体原因${diagnostic ? `（${diagnostic}）` : ''}`;
    }
    root.serialArmErrorI18n = {translate};
})(typeof window !== 'undefined' ? window : globalThis);
