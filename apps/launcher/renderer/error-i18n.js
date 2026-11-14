'use strict';
// UI-only localization. Original diagnostics remain accessible in the
// "原始技术诊断" expandable section, and backend protocols remain stable
(function (root) {
    const explanations = [
        [/final_target_window_not_measured/i, '最终静止目标窗口尚未测量'],
        [/final_target_window_incomplete/i, '最终静止目标窗口未完整记录，定位测量证据不足'],
        [/final_target_nonfinite/i, '最终目标或反馈包含无效数值，无法评价到位精度'],
        [/final_target_feedback_discontinuous/i, '最终目标窗口反馈不连续，定位测量证据不足'],
        [/final_target_reference_not_stationary/i, '最终窗口参考仍在运动，尚无静止目标到位结论'],
        [/final_target_limited/i, '最终目标窗口发生限幅，定位测量证据不足'],
        [/final_target_feedback_stale/i, '最终目标窗口反馈过期，定位测量证据不足'],
        [/final_target_feedback_insufficient/i, '最终目标窗口的新鲜采样不足，定位测量证据不足'],
        [/final_target_window_qualified/i, '最终到位窗口已测量，误差仅供参考'],
        [/final_target_error_not_qualified/i, '最终到位误差已记录，仅供参考；实测参数与过程记录保留'],
        [/compliant restoring response is below executable protocol resolution/i, '柔性恢复增益低于当前协议可执行分辨率，本用途尚未完成有效测量'],
        [/mit_affine_inverse_response_gain_holdout_failed/i, '额外控制组的留出响应与模型预测不一致，本轴不采用该设计；当前实测整组参数保留'],
        [/mit_independent_(?:input_tone_below_noise|position_tone_below_noise|tone_fit_quality_insufficient|tone_fit_rank_insufficient)/i, '独立输入或位置响应尚不能在噪声中可靠分辨，本轴频响辨识未完成；已有实测结果保留'],
        [/mit_(?:independent_actual_gains_unknown_or_changing|held_other_actual_gains_not_constant|held_other_axis_gains_changed|controller_operating_condition_changed|controller_reference_changed)/i, '频响试验期间的实际增益、参考或工况不一致，本轴辨识未完成；已有实测结果保留'],
        [/mit_(?:controller_gain_variations_degenerate|controller_gain_design_ill_conditioned|internal_feedback_port_unidentifiable|controller_experiments_insufficient|controller_experiment_invalid)/i, '实际控制组变化或有效试验数量不足，无法可靠区分内部反馈通道；当前实测整组参数保留'],
        [/mit_independent_command_limited/i, '独立输入发生限幅，本次频响证据无效；当前实测整组参数保留'],
        [/mit_independent_(?:feedback_gap|feedback_insufficient|observation_incomplete|observation_too_short|clock_unknown|clock_or_feedback_age_invalid)/i, '频响测量的新鲜反馈、时长或命令反馈时序不足；当前实测整组参数保留'],
        [/frequency_identification_(?:incomplete_keep_measured_controller|warning|protocol_variations_unresolved)/i, '本轴频响辨识证据不足，保留当前实测整组参数并继续其余可执行测量'],
        [/time_budget_insufficient_for_complete_measurement/i, '当前时限不足以完成所选用途的必要测量，尚未启动；请重新选择标定用途或示教路径'],
        [/gain calibration measurements are incomplete; review is not ready/i, '增益测量未完成，尚未生成可审核结果；请补齐缺测项后再应用或导出'],
        [/replay mode must be RIGID_TRACKING or COMPLIANT_TRACKING/i, '回放任务状态无效，请重新选择执行器增益用途'],
        [/replay requires valid ACTIVE feedback/i, '轨迹回放需要机械臂已使能并且实时反馈有效'],
        [/replay requires fresh control feedback/i, '轨迹回放的实际反馈已过期，请检查通信与控制周期'],
        [/robot is not aligned with (?:the )?selected trajectory endpoint/i, '当前姿态或速度尚未满足轨迹起步条件；请查看关节目标与偏差，按现有引导安全起步'],
        [/re-timed trajectory violates joint margins|interpolated playback would exceed URDF joint safety margins|retimed (?:imported )?trajectory exceeds safe (?:joint|replay) range|interpolated (?:trajectory|replay) (?:violates|exceeds) joint (?:position margins|safety margin)/i, '重定时或插值后的轨迹未通过位置范围检查；请核对有效位置范围、用户 Safety 留量与示教路径，该诊断未注明具体关节与越界位置'],
        [/drag teaching requires GRAVITY feedforward/i, '轨迹示教需要先在失能状态启用 GRAVITY 重力前馈，再使能执行拖拽'],
        [/drag teaching requires positive damping/i, '轨迹示教需要每个关节的拖拽阻尼为正数'],
        [/drag teaching requires nonzero gravity compensation/i, '轨迹示教需要有效且非零的重力补偿比例'],
        [/load or record a trajectory before replay/i, '请先录制或从轨迹管理中载入一条轨迹'],
        [/trajectory file unavailable|trajectory file not found/i, '轨迹数据文件不存在，请检查轨迹文件夹的位置与权限'],
        [/trajectory joint names or order differ/i, '轨迹的关节名称或顺序与当前机械臂不一致，不能直接回放'],
        [/trajectory is too short or exceeds URDF joint safety margins|trajectory is too short or violates safety limits|imported trajectory is too short or exceeds joint safety margins/i, '轨迹采样不足或未通过位置范围检查；该诊断未区分具体原因，请核对采样数量、有效位置范围与用户 Safety 留量'],
        [/recorded endpoint violates joint safety margins|friction trajectory exceeds safe replay range/i, '示教端点或路径未通过位置范围检查；请核对真实机械行程、关节映射与用户 Safety 留量，该诊断未注明具体越界位置'],
        [/finish teaching playback before calibration/i, '请先停止示教回放任务，再启动其他操作'],
        [/no replay task is active/i, '当前没有正在执行的示教回放任务'],
        [/stop active recording or replay before loading/i, '请先结束当前录制或回放，再选择其他轨迹'],
        [/workspace and released-hand confirmation required/i, '回放前请确认人员、负载、线缆与工作空间安全，并完全松开机械臂'],

        [/\bCALIBRATION_RESPONSE_UNSTABLE\b/i, '参考未要求的位置持续往复触发了标定响应保护，本轮已停止；请检查增益与响应记录，再按现有流程恢复'],
        [/model calibration stopped by safety fault/i, '标定已被控制故障或响应保护中断；请查看具体故障，关闭提示不会自动重新使能或继续运动'],
        [/\bMODE_SWITCH_FAILED\b/i, '执行器控制模式切换失败，请检查所选 MIT/CSP 执行路径、硬件驱动和模式确认结果；处理后清除启动故障，再重新使能'],
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
        [/calibration frozen gain verification following limit exceeded/i, '历史测试按跟踪误差阈值结束；请查看对应关节的实测误差与增益，分析仅供参考'],
        [/gain_endpoint_transition_insufficient_response/i, '端点自动过渡响应不足，已停止本趟，保留已测结果'],
        [/gain_endpoint_transition_reference_timeout/i, '端点参考生成未在本次预留时间内完成，过渡证据保留'],
        [/gain_endpoint_transition_feedback_wait_timeout/i, '端点参考等待实际反馈跟随超时，过渡证据保留'],
        [/gain_endpoint_transition_insufficient_reserved_budget/i, '剩余预算不足以完成端点过渡并保留后续测量时间，未继续本次过渡'],
        [/gain_endpoint_transition_execution_limited/i, '端点过渡执行中发生参考或力矩限幅，本次响应不能按原计划解释'],
        [/gain_endpoint_transition_feedback_unavailable/i, '端点过渡缺少有效反馈，本次过渡未完成'],
        [/gain_endpoint_transition_feedback_stale/i, '端点过渡的实际反馈已过期，本次过渡未完成'],
        [/gain_endpoint_transition_interrupted/i, '端点过渡已中断，已有测量保留'],
        [/gain endpoint transition interrupted/i, '端点自动过渡已中断，已测结果已保留'],
        [/gain endpoint transition needs fresh measured origin/i, '端点过渡尚未取得最新实测起步位置，本次过渡已停止'],
        [/gain endpoint transition lost continuous feedback/i, '端点过渡的连续反馈中断，本次过渡已停止，已有测量保留'],
        [/gain endpoint transition feedback mapping invalid/i, '端点过渡的实际反馈与关节映射不一致，本次过渡已停止'],
        [/gain endpoint transition needs fresh per-joint feedback/i, '端点过渡未取得各轴新鲜实测反馈，本次过渡已停止'],
        [/measurement_center_(?:mapping_invalid|fresh_position_invalid|demonstrated_path_invalid|bounds_invalid)|path_progress_(?:input_invalid|mapping_invalid|feedback_or_limits_invalid|state_outside_segment)/i, '进入测量区所需的路径、关节映射、反馈或运动限制数据无效；本次进入已停止，已有测量保留'],
        [/measurement_center_fresh_position_outside_safety/i, '最新实测位置超出本次有效 Safety 范围，未开始进入测量区；请查看实际位置与本次有效范围'],
        [/measurement_center_outside_existing_endpoint_alignment/i, '当前姿态尚未满足已有引导范围，未开始进入测量区；请按现有引导安全起步'],
        [/measurement_center_no_demonstrated_safety_intersection/i, '已示教路径与本次有效 Safety 范围没有交集，无法安排进入测量区'],
        [/measurement_center_no_inward_(?:entry|route)_within_existing_alignment/i, '已有引导范围内没有沿已示教路径向内的连续入口，本次未开始进入测量区'],
        [/measurement_center_route_unavailable/i, '未取得可用的示教测量区路线，本次移动未开始'],
        [/measurement_center_entry_insufficient_reserved_budget/i, '剩余预算不足以完成路径移动并保留后续测量时间，本次未继续；已有测量保留'],
        [/measurement_center_entry_response_or_budget_insufficient/i, '本次路径移动未在预留响应时间内完成，已停止并保留已有测量'],
        [/measurement_center_entry_(?:control_mode_rejected|gain_step_rejected)/i, '测量区移动所需的控制模式或整组增益调整被拒绝，已停止本次移动；已有测量保留'],
        [/measurement_center_entry_feedback_unavailable/i, '测量区移动缺少有效连续反馈，已停止并保留已有测量'],
        [/measurement_center_entry_feedback_stale/i, '测量区移动的实际反馈已过期，已停止并保留已有测量'],
        [/measurement_center_entry_no_further_calibrated_input/i, '本次路径跟随仍不足，当前测量条件内已无可执行增益增量；未测得硬件能力上限，已有测量保留'],
        [/measurement_center_entry_execution_limited/i, '执行期间发生参考或力矩限幅，已停止本次移动，不能继续按原计划解释响应'],
        [/measurement_center_entry_gain_step_consumes_feedback_lead/i, '当前反馈误差不允许继续增加增益并收紧参考跟随距离，已停止本次移动'],
        [/measurement_center_entry_interrupted_or_insufficient_response/i, '本次路径移动已中断或响应不足，位置进入未完成；已有测量保留'],
        [/measurement_center_entry_interrupted/i, '沿示教路径的测量区移动已中断，已有测量保留'],
        [/measurement_center_evidence_write_failed/i, '本次路径移动证据写入失败，请查看原始错误；不补记为测量完成'],
        [/measurement_center_original_endpoint_route_outside_safety/i, '原示教端点至测量区的路线超出本次有效 Safety 范围，未安排该路线'],
        [/measurement_return_original_route_unavailable/i, '原示教返回路线或最新实测位置不可用，未开始返程'],
        [/measurement_return_endpoint_does_not_match_original_demonstration/i, '返回端点与原示教路径记录不一致，未开始返程'],
        [/measurement_return_original_route_invalid/i, '原示教返回路线的关节数据或采样索引无效，未开始返程'],
        [/measurement_return_outside_existing_alignment/i, '当前实测位置尚未满足已有返程引导范围，未开始返程'],
        [/measurement_return_no_nearby_original_path_projection/i, '最新实测位置无法在已有引导范围内接入原示教路径，未开始返程'],
        [/fresh_projection_then_reverse_original_demonstrated_route/i, '已规划从最新实测位置接入原示教路径，再沿原路径反向返回'],
        [/path_position_uncertainty_consumes_feedback_lead/i, '路径内部制动预留无法容纳于允许参考跟随距离，已停止本次移动'],
        [/measurement_path_feedback_reserve_input_invalid/i, '路径反馈制动预留的输入无效，不能继续本次移动'],
        [/measurement_path_position_resolution_or_noise_consumes_feedback_lead/i, '已观测位置步进或噪声所需预留占满参考跟随距离，已停止本次移动'],
        [/observed_position_step_and_stationary_noise_reserve/i, '依据已观测最小位置步进与静止噪声设置内部制动预留'],
        [/unidentified_position_step_internal_guard_buffer/i, '最小位置步进尚未知，按跟随距离比例设置内部制动预留'],
        [/path_reference_outside_feedback_lead|path_feedback_lead_(?:below_required_braking_distance|cannot_be_retained_with_acceleration_limit)/i, '沿示教路径进入测量区时，参考与实测的跟随余量无法满足本次运动限制；本次进入已停止，已有测量保留'],
        [/calibration replay soft following recovery timed out/i, '自动回放等待关节追上参考位置超时，请检查增益、摩擦与负载'],
        [/calibration tracking error exceeded safe following limit/i, '自动回放的实际位置持续无法跟上参考轨迹，已按安全限制中止'],
        [/gain stationary noise baseline is moving/i, '静止噪声采样时关节仍在运动，未开始探测；请检查机械臂支撑和反馈'],
        [/gain calibration (?:recorded-duration|300 second) wall-time budget exhausted/i, '自动标定已到本次时限，已结束后续运动；请查看已完成用途与保留的实测记录'],
        [/checked demonstration and required independent verifications do not fit (?:twelve times the original recorded duration|the 300 second (?:gain calibration )?budget)/i, '安全重定时后的轨迹与必需验收无法在原始轨迹时长的 12 倍预算内完成，请检查轨迹和所选用途'],
        [/gain calibration requires a checked demonstration trajectory/i, '请先选择并检查示教轨迹，再启动增益标定'],
        [/explicit demonstrated trajectory replay confirmation is required/i, '请先确认整条示教回放路径并完全松手，再启动增益标定'],
        [/demonstrated trajectory endpoint is stale or not aligned|gain trajectory endpoint.*(?:mismatch|aligned)|gain trajectory.*start.*(?:mismatch|aligned)/i, '当前姿态、速度或反馈未满足本趟起步条件；请查看实际位置与反馈状态，按现有引导安全起步'],
        [/demonstrated-trajectory gain replay (?:lost continuous feedback|reached the hard following limit)/i, '整轨增益测量遇到反馈中断或硬跟踪误差保护，已停止回放；请查看反馈与各轴实际响应'],
        [/rigid full-path tracking or independent hold verification did not qualify/i, '刚性跟踪或定位保持未达参考指标，实测结果已保留'],
        [/compliant full-path tracking or independent hold verification did not qualify/i, '柔性跟踪或定位保持未达参考指标，实测结果已保留'],
        [/native servo tracking or requested admittance trajectory verification did not qualify/i, '伺服跟踪或本次启用的导纳未达参考指标，实测结果已保留'],
        [/simultaneous drag release did not independently qualify every joint/i, '部分关节的拖拽释放衰减测量需查看，请结合各轴释放速度与衰减记录判断，分析仅供参考'],
        [/gain motion discovery failed/i, '在已确认的局部范围内未建立可测的双向运动，请查看本窗实测幅度与判定下限'],
        [/gain dynamic search failed/i, '动态增益搜索已结束，请查看本窗实测跟踪数据与探测记录，分析仅供参考'],
        [/gain probe cannot preserve its established amplitude within safety limits/i, '当前安全限制无法容纳已测得有效的探测幅度，未继续缩小幅度验收'],
        [/gain (?:local )?motion range confirmation(?: is)? required/i, '请先确认局部探测范围并完全松开机械臂，再启动增益标定'],
        [/no observable joint response.*after bounded gain search/i, '关节在限定增益搜索后仍未出现有效跟随响应，请检查探测记录及已配置的伺服增益'],
        [/measured tracking did not qualify.*after bounded gain search/i, '限定增益搜索后的跟踪响应已记录，请查看探测记录与当前增益，分析仅供参考'],
        [/no observable joint response.*safety bounds/i, '关节未出现有效跟随响应，增益搜索已到配置限制，无法继续调整候选'],
        [/measured tracking did not qualify.*safety bounds/i, '历史增益搜索已到当时配置限制；跟踪响应记录仅供参考'],
        [/servo baseline failed independent frozen verification/i, '历史任务在伺服独立测量后结束，后续用途未测量；实测分析仅供参考'],
        [/compliant gains failed independent frozen verification/i, '柔性增益独立测量已记录，分析仅供参考'],
        [/configured servo gain baseline exceeds joint safety limits/i, '已配置的伺服基线增益超过该关节的安全限制，请检查增益及限制值'],
        [/drag excitation produced no observable release velocity/i, '拖拽测试未建立可测的释放速度，无法验收阻尼；请检查该关节跟踪响应及探测记录'],
        [/simultaneous drag excitation produced no measured release velocity/i, '该关节未建立可测的释放速度，无法验收拖拽阻尼；请检查实际位置变化与跟踪响应'],
        [/insufficient joint margin or excitation budget for an observable bounded servo probe/i, '关节剩余行程或探测预算不足，无法在当前安全限制内测得有效跟踪响应'],
        [/insufficient safe velocity and torque budget for an observable drag release/i, '当前速度与力矩预算不足，无法建立可测的拖拽释放速度'],
        [/drag damping did not qualify within its bounded search/i, '拖拽阻尼限定搜索已结束，请查看释放速度与衰减记录，分析仅供参考'],
        [/drag probe exceeded local motion envelope/i, '拖拽测试超过局部运动范围，已中止测试'],
        [/gain probe has insufficient continuous measurements/i, '关节探测缺少连续有效测量，请检查反馈后重新标定'],
        [/gain adaptation.*(without|not).*\b(verification|validated|clean|converge)/i, '请查看各用途实测结果，分析仅供参考；可采用已完成测量的用途或重新标定'],
        [/automatic actuator gain calibration currently supports rotary joints only/i, '执行器增益自动标定目前仅支持转动关节，当前控制链包含移动关节'],
        [/gain candidate failed continuous measured qualification/i, '部分增益用途未达参考指标，完整测量的用途可直接应用或导出'],
        [/gain candidate is partially qualified/i, '部分用途尚未完成测量，请勾选已完成测量的用途'],
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
        [/demonstration trajectory exceeds safe replay range/i, '示教轨迹未通过可执行位置范围检查，已拒绝回放；请核对真实机械行程、关节映射与用户 Safety 留量，该诊断未注明具体越界位置'],
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
    const codePattern = /\b(?:CALIBRATION_RESPONSE_UNSTABLE|MOTOR_BUS_[A-Z_]+|ACTUATOR_[A-Z_]+|STATE_TIMEOUT|CMD_TIMEOUT|INVALID_DT|JOINT_[A-Z_]+|REFERENCE_GOVERNOR_FAILED|INFEASIBLE|INVALID_INPUT|FAULT_RECOVERY_NOT_ALLOWED|FAULTED|NOT_FAULTED|ALREADY_ACTIVE|NOT_ACTIVE|NOT_INACTIVE|SAFETY_FAILED|WRITE_DISABLED|MODE_SWITCH_FAILED|ENABLE_FAILED|DISABLE_FAILED|OPEN_FAILED|TIMEOUT|EPIPE)\b/g;
    function trajectoryRangeMessage(clean, lang) {
        const match=clean.match(/trajectory_range_violation:([^\r\n]*)/i);
        if(!match)return null;
        const fields=Object.fromEntries([...match[1].matchAll(/(?:^|;)\s*([a-z_]+)=([^;\s]+)/gi)].map(value=>[value[1],value[2]]));
        const numeric=['value','lower','upper','margin','sample'];
        if(!fields.joint||!['rad','m'].includes(fields.unit)||numeric.some(key=>fields[key]===undefined||!Number.isFinite(Number(fields[key])))||!Number.isInteger(Number(fields.sample))||Number(fields.sample)<0||Number(fields.margin)<0)return null;
        const number=value=>String(Number(Number(value).toPrecision(7)));
        const degrees=value=>String(Number((Number(value)*180/Math.PI).toFixed(4)))+'°';
        const position=value=>`${number(value)} ${fields.unit}${fields.unit==='rad'?`（${degrees(value)}）`:''}`;
        const interval=`[${number(fields.lower)}, ${number(fields.upper)}] ${fields.unit}${fields.unit==='rad'?`（[${degrees(fields.lower)}, ${degrees(fields.upper)}]）`:''}`;
        const recorded=fields.scope==='recorded_position',command=fields.scope==='command_reference';
        if(lang==='en'){
            const label=recorded?'Recorded position':command?'Playback reference position':'Trajectory position';
            const margin=recorded?'recording-check margin':command?'user Safety margin':'check margin';
            const source=recorded?'Recording validity uses the hard-range intersection of URDF limits and mapped driver position capability; the user Safety command margin is checked separately for playback.':`The interval is the intersection of URDF limits and mapped driver position capability, narrowed by the ${command?'user Safety':'reported check'} margin.`;
            return `${label} check failed at ${fields.joint}, sample index ${fields.sample}; position ${position(fields.value)}, effective interval ${interval}, ${margin} ${position(fields.margin)}. ${source} Check physical travel, joint mapping and the demonstrated path`;
        }
        const label=recorded?'录制位置':command?'回放参考位置':'轨迹位置',margin=recorded?'本次录制校验留量':command?'用户 Safety 留量':'本次检查留量';
        const source=recorded?'录制有效性按 URDF 与映射后驱动位置能力的有效硬范围检查；用户 Safety 命令留量另用于回放参考检查':`范围来自 URDF 与映射后驱动位置能力的交集，再按${command?'用户 Safety':'本次检查'}留量向内收缩`;
        return `${fields.joint} 的${label}检查未通过（采样索引 ${fields.sample}）；位置 ${position(fields.value)}，有效区间 ${interval}，${margin} ${position(fields.margin)}；${source}；请核对真实机械行程、关节映射与示教路径`;
    }
    function translate(raw, lang = 'zh-CN') {
        const input = String(raw ?? '').trim();
        if (!input) return input;
        const clean = input
            .replace(/^(?:Error invoking remote method ['"]launcher:request['"]:\s*)?Error:\s*/i, '')
            .replace(/^(?:操作失败|错误)[:：]\s*/, '').trim();
        const trajectoryRange=trajectoryRangeMessage(clean,lang);
        if(trajectoryRange)return trajectoryRange;
        if(clean==='reference_analysis_available')return lang==='en'?'Results saved':'结果已保存';
        const referenceMessages={
            drag_demonstrated_motion_release:['沿示教短段建立运动，随后测量零刚度释放','Preparing motion on a taught segment, then measuring zero-stiffness release'],
            drag_demonstrated_preparation:['正在沿示教短段带动关节','Moving joints along a taught segment'],
            drag_release_requires_known_constant_damping:['释放窗口的实际阻尼未保持一致','Actual damping was not constant during release'],
            drag_demonstrated_release_stopped_at_measured_domain:['释放响应已到本次示教测量范围，保留部分记录','Release response reached the taught measurement domain; partial evidence retained'],
            purpose_pulse_unresolved_at_current_ceiling_measure_opposite_direction:['本方向输入已到测量边界，改测有示教余量的反向','This input direction reached its measurement bound; measure the legal opposite direction'],
            both_legal_input_directions_measured_unresolved_small_stiffness_reduction_comparison:['两方向输入均已测到边界，安排小幅降低 Kp 的比较，Kd 保持','Both input directions reached their bounds; compare a small Kp reduction with Kd retained'],
            online_measured_static_bias_or_same_direction_lag_bounded_P_response_trial_not_identification:['依据实测静差或同向滞后进行有界 Kp 响应试探，Kd 保持；尚未辨识模型','Bounded Kp response trial from measured static error or same-direction lag; Kd retained, model not identified'],
            online_unresolved_response_trial_exhausted_continue_taught_path:['本位置的有界响应试探已结束，继续沿示教轨迹测量','Bounded response trials at this pose are complete; measurement continues along the taught path'],
            online_response_trial_has_no_existing_effort_room:['本次响应试探没有足够的既有力矩余量，保留参数继续运动','Insufficient existing effort room for this response trial; retain gains and continue motion']
        };
        if(referenceMessages[clean])return referenceMessages[clean][lang==='en'?1:0];
        if(lang==='en')return input;
        // Leave fully localized backend diagnostics intact.
        if (!/[A-Za-z]{3,}/.test(clean)) return clean;
        const actionMatch = clean.match(/^([a-z_]+) failed:\s*/i);
        const action = actionMatch && actionNames[actionMatch[1]] ? `${actionNames[actionMatch[1]]}失败 ` : '';
        const matched = explanations.find(([pattern]) => pattern.test(clean));
        const numbers = [...clean.matchAll(/\b(?:value|limit|position error|speed|timeout)\s*=\s*[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?(?:\s*(?:rad(?:\/s)?|ms|s))?/gi)]
            .map(m => m[0].replace(/^value=/i, '实测值=').replace(/^limit=/i, '限制值=').replace(/^position error=/i, '位置误差=').replace(/^speed=/i, '速度=').replace(/^timeout=/i, '超时='));
        const codes = [...new Set((clean.match(codePattern) || []))];
        const replayFields = [...clean.matchAll(/\b(?:joint|actual|reference|error|soft_limit|hard_limit|kp|kd|adaptive|phase|tracking_mode|recovery_elapsed_s|velocity|validation_pause_limit|error_ratio|validation)\s*=\s*(?:[A-Za-z_][\w-]*|[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?)/gi)].map(m => m[0]);
        const gainJoint = clean.match(/(?:gain stationary noise baseline is moving|gain motion discovery failed|gain dynamic search failed|gain probe cannot preserve its established amplitude within safety limits|no observable joint response|measured tracking did not qualify|configured servo gain baseline exceeds joint safety limits|gain probe has insufficient continuous measurements|drag excitation produced no observable release velocity|simultaneous drag excitation produced no measured release velocity|insufficient joint margin or excitation budget for an observable bounded servo probe|insufficient safe velocity and torque budget for an observable drag release|drag damping did not qualify within its bounded search) at ([A-Za-z_][\w.-]*)/i);
        const endpointJoint=clean.match(/gain_endpoint_transition_insufficient_response:([A-Za-z_][\w.-]*)/i);
        const failedUses = clean.match(/gain candidate failed continuous measured qualification:\s*([^;]+)/i);
        const purposes = {rigid_tracking:'刚性',compliant_tracking:'柔性',compliant_drag:'拖拽',servo_tracking:'伺服跟踪'};
        const failedNames = failedUses ? failedUses[1].split(',').map(key=>purposes[key.trim()]).filter(Boolean) : [];
        const diagnostic = [...new Set([...codes, ...numbers, ...replayFields, ...(gainJoint ? ['关节=' + gainJoint[1]] : []), ...(endpointJoint?['关节='+endpointJoint[1]]:[]), ...(failedNames.length ? ['相关用途='+failedNames.join('、')] : [])])].join('，');
        if (matched) return `${action}${matched[1]}${diagnostic ? `（${diagnostic}）` : ''}`;
        const zhPortion = clean.match(/[\u3400-\u9fff][^\n]*/);
        if (zhPortion) return `${action}${zhPortion[0]}${diagnostic ? `（${diagnostic}）` : ''}`;
        return `${action || '操作未完成 '}具体原因：${clean.slice(0, 260)}${clean.length > 260 ? '…' : ''}${diagnostic ? `（${diagnostic}）` : ''}`;
    }
    root.serialArmErrorI18n = {translate};
})(typeof window !== 'undefined' ? window : globalThis);
