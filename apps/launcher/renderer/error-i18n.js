'use strict';
// UI-only localization. Original diagnostics remain accessible in the
// "原始技术诊断" expandable section, and backend protocols remain stable.
(function (root) {
    const explanations = [
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
        [/calibration tracking error exceeded safe following limit/i, '自动回放的实际位置持续无法跟上参考轨迹，已按安全限制中止'],
        [/gain adaptation.*(without|not).*\b(verification|validated|clean|converge)/i, '增益自适应未达到全路径验证要求 候选参数已保留，但不能直接应用'],
        [/reverse static replay failed or cancelled/i, '反向自动采集失败或已取消，请查看具体关节和控制故障'],
        [/forward.*replay failed or cancelled/i, '正向自动采集失败或已取消'],
        [/cannot enter RIGID_HOLD/i, '无法切换到刚性保持模式，请检查机器人状态'],
        [/cannot.*(enter|switch).*tracking/i, '无法切换到跟踪控制模式，请检查机器人状态及安全限制'],
        [/no identification data were accepted/i, '没有通过验收的有效辨识数据，不能生成可靠的模型参数'],
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
        const diagnostic = [...codes, ...numbers].join('，');
        if (matched) return `${action}${matched[1]}${diagnostic ? `（${diagnostic}）` : ''}`;
        const zhPortion = clean.match(/[\u3400-\u9fff][^\n]*/);
        if (zhPortion) return `${action}${zhPortion[0]}${diagnostic ? `（${diagnostic}）` : ''}`;
        return `${action || '操作失败 '}未能识别的技术错误，请展开原始技术诊断以定位原因${diagnostic ? `（${diagnostic}）` : ''}`;
    }
    root.serialArmErrorI18n = {translate};
})(typeof window !== 'undefined' ? window : globalThis);
