'use strict';
// UI-only localization. Original diagnostics remain accessible in the
// "原始技术诊断" expandable section, and backend protocols remain stable.
(function (root) {
    const explanations = [
        [/replay mode must be RIGID_TRACKING or COMPLIANT_TRACKING/i, '回放任务状态无效，请重新选择执行器增益用途'],
        [/replay requires valid ACTIVE feedback/i, '轨迹回放需要机械臂已使能并且实时反馈有效'],
        [/replay requires fresh control feedback/i, '轨迹回放的实际反馈已过期，请检查通信与控制周期'],
        [/robot is not aligned with selected trajectory endpoint/i, '当前关节姿态尚未对齐轨迹端点，请先安全返回对应起点或终点'],
        [/re-timed trajectory violates joint margins|interpolated playback would exceed URDF joint safety margins/i, '轨迹插值后的路径可能超过 URDF 关节安全范围，请重新示教并保留限位余量'],
        [/drag teaching requires GRAVITY feedforward/i, '轨迹示教需要先在失能状态启用 GRAVITY 重力前馈，再使能执行拖拽'],
        [/drag teaching requires positive damping/i, '轨迹示教需要每个关节的拖拽阻尼为正数'],
        [/drag teaching requires nonzero gravity compensation/i, '轨迹示教需要有效且非零的重力补偿比例'],
        [/load or record a trajectory before replay/i, '请先录制或从轨迹管理中载入一条轨迹'],
        [/trajectory file unavailable|trajectory file not found/i, '轨迹数据文件不存在，请检查轨迹文件夹的位置与权限'],
        [/trajectory joint names or order differ/i, '轨迹的关节名称或顺序与当前机械臂不一致，不能直接回放'],
        [/trajectory is too short or exceeds URDF joint safety margins/i, '示教轨迹不足或超出 URDF 关节安全范围，请重新录制'],
        [/finish teaching playback before calibration/i, '请先停止示教回放任务，再启动其他操作'],
        [/no replay task is active/i, '当前没有正在执行的示教回放任务'],
        [/stop active recording or replay before loading/i, '请先结束当前录制或回放，再选择其他轨迹'],
        [/workspace and released-hand confirmation required/i, '回放前请确认人员、负载、线缆与工作空间安全，并完全松开机械臂'],

        [/model calibration stopped by safety fault/i, '示教或回放已被机械臂安全故障中断 不会自动重新使能，请先检查故障和机械臂支撑状态'],
        [/MOTOR_BUS_ACTIVATE_FAILED|ENABLE_FAILED|ACTUATOR_OFFLINE/i, '执行器使能失败或电机离线，请检查供电、急停、通信连接和电机状态；恢复供电与通信后，清除启动故障再使能'],
        [/MOTOR_BUS_CONNECT_FAILED|OPEN_FAILED|NOT_CONNECTED/i, '执行器通信连接失败，请检查设备路径、访问权限、线缆和供电'],
        [/MOTOR_BUS_READ_FAILED|READ_FAILED/i, '执行器反馈读取失败 请检查通信链路和电机供电'],
        [/MOTOR_BUS_WRITE_FAILED|WRITE_FAILED/i, '执行器命令写入失败 请检查通信链路、电源和电机状态'],
        [/MOTOR_BUS_DEACTIVATE_FAILED|DISABLE_FAILED/i, '执行器失能未被全部确认 请先确保机械臂得到支撑，再检查电机供电和通信'],
        [/fault_recovery\.gravity_model_validated is false/i, '尚未确认重力补偿效果，无法进入故障柔性恢复；请先支撑机械臂并验证重力补偿，在安全策略中确认并保存后重新连接'],
        [/fault_recovery\.allow_compliant_recovery is false/i, '当前配置未开放故障柔性恢复；请在安全策略中检查，保存后重新连接生效'],
        [/fault type is not eligible for compliant recovery/i, '当前故障类型不允许柔性恢复，请先处理电机或通信故障'],
        [/FAULT_RECOVERY_NOT_ALLOWED/i, '当前故障不满足低增益柔性恢复条件 请检查重力模型可信标志、故障保持及实际故障类型'],
        [/\bFAULTED\b/i, '机械臂处于故障状态，请处理原因、清除故障后再使能'],
        [/\bNOT_FAULTED\b/i, '当前机械臂没有故障，无需清除'],
        [/\bALREADY_ACTIVE\b/i, '机械臂已经使能'],
        [/\bNOT_ACTIVE\b/i, '机械臂尚未使能，请检查供电后使能'],
        [/\bNOT_INACTIVE\b/i, '请先停放并失能机械臂，再执行此操作'],
        [/\bWRITE_DISABLED\b/i, '当前为离线后端；连接真实硬件前，请在参数配置中开启执行器写入并重新连接'],
        [/\bCMD_TIMEOUT\b/i, '控制命令更新超时，请检查命令是否持续发送，以及控制周期和通信延迟'],
        [/\bINVALID_DT\b/i, '控制周期超出安全限制 请检查系统负载、进程调度和通信延迟'],
        [/\bJOINT_POS_LIMIT\b/i, '关节位置超出安全范围 请检查关节姿态与零位配置'],
        [/\bJOINT_VEL_LIMIT\b/i, '实测关节速度超出安全限制'],
        [/\bCMD_VEL_LIMIT\b/i, '下发的关节速度超过安全限制'],
        [/\bSTATE_TIMEOUT\b/i, '机械臂反馈超时，当前任务已停止；请检查通信与控制调度，恢复后重新开始任务'],
        [/failed to (?:return|switch|enter).*RIGID_HOLD/i, '任务结束时未能恢复刚性保持，请先处理最初的机械臂故障'],
        [/\bSAFETY_FAILED\b/i, '机械臂安全检查未通过，请核对关节状态、反馈和安全限制'],
        [/\bREFERENCE_GOVERNOR_FAILED\b.*\bINFEASIBLE\b/i, '运动限制与 PD / 前馈总力矩预算内找不到可执行参考，请检查轨迹、实测位置与增益'],
        [/\bREFERENCE_GOVERNOR_FAILED\b.*\bINVALID_INPUT\b/i, '参考调节器输入无效，请检查参考运动与力矩预算参数'],
        [/\bREFERENCE_GOVERNOR_FAILED\b/i, '参考调节器无法接受当前命令，请检查参考运动和总力矩预算'],
        [/trajectory replay soft following recovery timed out/i, '关节未能在限定时间内追上回放轨迹，任务已停止；请降低回放速度或检查对应关节的跟踪增益'],
        [/trajectory replay hard following limit reached/i, '示教回放已触及硬跟踪误差保护，任务已中止 请检查控制增益、负载、摩擦和实际关节响应'],
        [/trajectory replay initial reference exceeds hard following limit/i, '当前实际姿态与轨迹起点偏差过大，已拒绝回放'],
        [/trajectory replay control feedback stalled|trajectory replay feedback is invalid|trajectory replay did not receive fresh control feedback/i, '示教回放期间反馈无效或已超时，任务已停止 请检查通信与控制周期'],
        [/trajectory replay tracking gains are unavailable/i, '示教回放的跟踪增益缺失或无效'],
        [/calibration replay initial reference exceeds hard following limit/i, '自动回放起始参考位置与实测关节位置相差过大，已拒绝发送运动指令 请检查机械臂当前姿态和示教轨迹起点'],
        [/calibration replay hard following limit reached/i, '自动回放触及硬跟踪误差边界，已停止回放 请检查关节执行器和轨迹是否存在异常'],
        [/calibration frozen gain verification following limit exceeded/i, '实际跟踪误差超过增益验收阈值，候选未通过本次测试；请检查对应关节、测试速度与跟踪增益'],
        [/calibration replay soft following recovery timed out/i, '自动回放等待关节追上参考位置超时，请检查增益、摩擦与负载'],
        [/calibration tracking error exceeded safe following limit/i, '自动回放的实际位置持续无法跟上参考轨迹，已按安全限制中止'],
        [/gain stationary noise baseline is moving/i, '静止噪声采样时关节仍在运动，未开始探测；请检查机械臂支撑和反馈'],
        [/gain motion discovery failed/i, '在已确认的局部范围内未建立可测的双向运动，请查看本窗实测幅度与判定下限'],
        [/gain dynamic search failed/i, '动态增益搜索未通过验收，请查看本窗跟踪指标、阈值与探测记录'],
        [/gain probe cannot preserve its established amplitude within safety limits/i, '当前安全限制无法容纳已测得有效的探测幅度，未继续缩小幅度验收'],
        [/gain (?:local )?motion range confirmation(?: is)? required/i, '请先确认局部探测范围并完全松开机械臂，再启动增益标定'],
        [/no observable joint response.*after bounded gain search/i, '关节在限定增益搜索后仍未出现有效跟随响应，请检查探测记录及已配置的伺服增益'],
        [/measured tracking did not qualify.*after bounded gain search/i, '关节跟踪在限定增益搜索后仍未通过验收，请检查探测记录与当前增益'],
        [/no observable joint response.*safety bounds/i, '关节未出现有效跟随响应，增益搜索已到配置限制，无法继续调整候选'],
        [/measured tracking did not qualify.*safety bounds/i, '关节跟踪未通过验收，增益搜索已到配置限制，无法继续调整候选'],
        [/servo baseline failed independent frozen verification/i, '伺服基线增益未通过独立冻结验收，后续增益用途尚未开始验收'],
        [/compliant gains failed independent frozen verification/i, '柔性增益未通过独立冻结验收'],
        [/configured servo gain baseline exceeds joint safety limits/i, '已配置的伺服基线增益超过该关节的安全限制，请检查增益及限制值'],
        [/drag excitation produced no observable release velocity/i, '拖拽测试未建立可测的释放速度，无法验收阻尼；请检查该关节跟踪响应及探测记录'],
        [/insufficient joint margin or excitation budget for an observable bounded servo probe/i, '关节剩余行程或探测预算不足，无法在当前安全限制内测得有效跟踪响应'],
        [/insufficient safe velocity and torque budget for an observable drag release/i, '当前速度与力矩预算不足，无法建立可测的拖拽释放速度'],
        [/drag damping did not qualify within its bounded search/i, '拖拽阻尼在限定搜索内未通过验收，请查看释放速度与衰减记录'],
        [/drag probe exceeded local motion envelope/i, '拖拽测试超过局部运动范围，已中止测试'],
        [/gain probe has insufficient continuous measurements/i, '关节探测缺少连续有效测量，请检查反馈后重新标定'],
        [/gain adaptation.*(without|not).*\b(verification|validated|clean|converge)/i, '增益测试未通过，请检查候选中的各用途结果，再采用已通过的用途或重新标定'],
        [/automatic actuator gain calibration currently supports rotary joints only/i, '执行器增益自动标定目前仅支持转动关节，当前控制链包含移动关节'],
        [/gain candidate failed continuous measured qualification/i, '增益候选未通过连续实测验收，请查看各用途结果及探测记录，可单独采用已通过的用途'],
        [/gain candidate is partially qualified/i, '候选中只有部分用途通过验收，请勾选要采用的已通过用途'],
        [/gain candidate URDF fingerprint mismatch|gain candidate joint mapping mismatch/i, '增益候选与当前模型或关节映射不匹配，请核对所选任务与机械臂配置'],
        [/reverse static replay failed or cancelled/i, '反向自动采集失败或已取消，请查看具体关节和控制故障'],
        [/forward.*replay failed or cancelled/i, '正向自动采集失败或已取消'],
        [/cannot enter RIGID_HOLD/i, '无法进入刚性保持，请检查机械臂状态'],
        [/cannot.*(enter|switch).*tracking/i, '无法进入跟踪控制，请检查机械臂状态与安全限制'],
        [/no identification data were accepted/i, '没有可用于辨识的有效数据，请检查采样反馈与样本接受情况'],
        [/native session (?:request timed out|is unresponsive)/i, '原生控制会话已失去响应，上一条控制命令的结果未知，请先确保机械臂硬件安全，然后点击恢复连接并重新检查供电'],
        [/timed out|timeout/i, '操作等待超时，请检查硬件连接、系统负载或对应任务的诊断记录'],
        [/Unknown launcher request/i, '界面与后台接口不匹配，请检查安装并重新启动工作台'],
        [/backend.*(unavailable|exited)|native session closed/i, '后台进程已退出或未就绪，请检查安装与运行日志'],
        [/device is missing or not readable\/writable/i, '找不到设备或没有读写权限，请检查设备路径、连接与访问权限'],
        [/invalid Safety configuration/i, '机械臂安全配置无效，请核对安全限制与关节映射'],
        [/failed to load|cannot load/i, '资源或任务记录读取失败，请检查文件路径和内容'],
        [/failed to save|cannot save/i, '保存失败，请检查目录权限与磁盘空间'],
        [/failed to export|export failed/i, '导出失败，请检查候选审核状态、文件权限与模型一致性'],
        [/blocked while a calibration task is active/i, '标定任务正在执行中，当前操作已被阻止 请先结束或取消标定任务'],
        [/calibration task is not initialized/i, '当前尚未创建标定任务，请先完成示教或加载已有记录'],
        [/model calibration is not waiting for replay confirmation/i, '标定任务尚未进入回放确认阶段'],
        [/model calibration teaching is not active/i, '当前示教已停止或被安全中断，无需重复结束 请查看任务状态，必要时点击清理中断示教'],
        [/cannot stop teaching while a calibration replay is active/i, '示教已经结束，当前正在执行自动回放 请使用取消任务操作而不是结束示教'],
        [/model calibration teaching stopped|teaching stopped because|teaching interrupted before normal stop/i, '示教因机械臂状态变化而中断，不会自动回放 已录制数据可在任务目录中检查'],
        [/stale or missing model calibration task_id/i, '当前操作引用了旧任务，请刷新任务状态后重试'],
        [/demonstration trajectory is too short/i, '示教轨迹过短，无法进行回放规划 已关闭当前采集，可重新示教'],
        [/demonstration trajectory exceeds safe replay range/i, '示教轨迹过于接近关节限位，已拒绝回放；请重新示教并留出限位余量'],
        [/explicit released confirmation is required/i, '请先确认已完全松开机械臂并检查运行区域'],
        [/operation.*cancelled|cancelled/i, '操作已取消'],
        [/invalid.*configuration|configuration.*invalid/i, '配置内容无效，请检查 Profile、硬件参数和模型'],
    ];
    const actionNames = {
        activate: '使能', deactivate: '失能', clear_fault: '清除故障',
        park: '停放', hold: '当前位置保持', fault_compliant: '柔性恢复',
        fault_rigid: '刚性恢复', set_impedance_mode: '切换控制状态',
        set_model_feedforward_mode: '切换模型前馈模式',
        set_reference_behavior: '切换控制方式', set_admittance: '调整导纳参数',
    };
    const codePattern = /\b(?:MOTOR_BUS_[A-Z_]+|ACTUATOR_[A-Z_]+|STATE_TIMEOUT|CMD_TIMEOUT|INVALID_DT|JOINT_[A-Z_]+|REFERENCE_GOVERNOR_FAILED|INFEASIBLE|INVALID_INPUT|FAULT_RECOVERY_NOT_ALLOWED|FAULTED|NOT_FAULTED|ALREADY_ACTIVE|NOT_ACTIVE|NOT_INACTIVE|SAFETY_FAILED|WRITE_DISABLED|ENABLE_FAILED|DISABLE_FAILED|OPEN_FAILED|TIMEOUT|EPIPE)\b/g;
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
        const replayFields = [...clean.matchAll(/\b(?:joint|actual|reference|error|soft_limit|hard_limit|kp|kd|adaptive|phase|tracking_mode|recovery_elapsed_s|velocity|validation_pause_limit|error_ratio|validation)\s*=\s*(?:[A-Za-z_][\w-]*|[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?)/gi)].map(m => m[0]);
        const gainJoint = clean.match(/(?:gain stationary noise baseline is moving|gain motion discovery failed|gain dynamic search failed|gain probe cannot preserve its established amplitude within safety limits|no observable joint response|measured tracking did not qualify|configured servo gain baseline exceeds joint safety limits|gain probe has insufficient continuous measurements|drag excitation produced no observable release velocity|insufficient joint margin or excitation budget for an observable bounded servo probe|insufficient safe velocity and torque budget for an observable drag release|drag damping did not qualify within its bounded search) at ([A-Za-z_][\w.-]*)/i);
        const diagnostic = [...new Set([...codes, ...numbers, ...replayFields, ...(gainJoint ? ['关节=' + gainJoint[1]] : [])])].join('，');
        if (matched) return `${action}${matched[1]}${diagnostic ? `（${diagnostic}）` : ''}`;
        const zhPortion = clean.match(/[\u3400-\u9fff][^\n]*/);
        if (zhPortion) return `${action}${zhPortion[0]}${diagnostic ? `（${diagnostic}）` : ''}`;
        return `${action || '操作未完成 '}具体原因：${clean.slice(0, 260)}${clean.length > 260 ? '…' : ''}${diagnostic ? `（${diagnostic}）` : ''}`;
    }
    root.serialArmErrorI18n = {translate};
})(typeof window !== 'undefined' ? window : globalThis);
