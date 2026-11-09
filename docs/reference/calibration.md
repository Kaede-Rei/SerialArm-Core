# 增益与动力学标定参考

自动增益标定使用当前模型与短测响应选择反馈参数；静态重力辨识校正质量与一阶质量矩，完整动力学辨识进一步估计惯量和运动相关项；采集、记录管理与界面操作见 [使用指南](../guide.md)

## 数据与运行条件

- 使用源码安装的 GUI 组件与安装器生成的 Python 环境；完整动力学辨识使用 NumPy、SciPy、Pinocchio、CVXPY 和 Clarabel
- 保留完整任务目录、`metadata.json`、`frames.csv`、已验收静态数据及 `source/` 源快照；新任务静态数据来自 `accepted_static.csv`
- 数据审计与拟合前断开运行会话；记录浏览和已保存成果导出可在工作台连接期间进行
- 独立验证使用同模型、同装机和同映射的另一采集会话；同会话留出用于评价本次记录

以下命令从仓库根目录执行，路径、任务编号和连杆名替换为自己的数据

## 自动增益的模型与证据

自动增益标定保留 MIT 控制和回转轴范围；原有运行增益只用于平滑交接与任务退出，模型初值不从当前 Kp/Kd、历史候选或新 Profile 的固定正值中选择

### 模型初始化

物理先验仅来自当前 URDF 的几何、质量、质心和惯量，经受控关节映射形成 `M/C/g`；URDF 与人工配置中的速度、加速度和力矩限制不是实测机械能力，也不进入物理参数或未约束响应需求的推断

有示教时，从实际参考路径计算姿态惯量包络、速度、加速度、频率与耦合加速度需求，结合误差目标和收敛时间生成整轨性能设计；任务将其保存在 `task_model_demand`，记录 `executed=false`、`measured=false`，不将这些高性能需求值直接下发为起动增益

实际起动使用 `bootstrap_model_initialization`，从 URDF 惯量、路径姿态下的完整质量矩阵和明确的恢复设计目标产生候选；默认目标为 `peak_error=0.1°`、`settling_time=0.7 s`、虚拟幅值 `0.008 rad`，不使用整轨加速度／误差推导的高带宽作为稳定硬目标；这些仍是未实测设计输入，后续要靠短测建立响应

按本次前馈覆盖的模型项计算剩余重力梯度后，再用位置余量、执行速度／加速度限制、可用反馈力矩和硬件协议能力收窄可执行候选；联合力矩域的速度误差包络来自本次计划，不把配置的最大速度当作全程实际速度误差

模型设计采用 `Kp ≈ J ω² + |∂g_residual/∂q|`、`Kd ≈ 2 ζ √(J Kp)`，`J` 为该轴姿态惯量包络；配对值沿阻尼曲线求解 `error_bound·Kp + velocity_bound·2ζ√(J·Kp) ≤ feedback_effort_budget`，再按协议码向下选择 Kp、向上选择相应 Kd；不将 P/D 等比例缩小后误认为阻尼比保持不变

整轨性能设计中 `ω²=max((2π f_task)², coupled_acceleration_torque/(J·peak_error), (4/(ζ·settling_time))²)`，默认 `ζ=0.7`；路径频率来自离散速度／加速度的 `sqrt(sum(a²)/sum(v²))/(2π)`；实际 bootstrap 使用恢复设计需求，并保留同一路径姿态的物理惯量，不继承整轨性能需求的频率与加速度项

完整质量矩阵参与初值选择：检查正定性和各采样姿态下的名义模态阻尼，按模态参与度及“缺额改善／相对参数变化”选择需要增加 D 或降低 K 的轴，不因一个低阻尼模态统一缩小所有轴的 K；若残余重力、协议网格及力矩域内仍无法达到目标，明确结束初始化并报出原因；这项配对约束仅用于初值，后续 Kp 搜索不因初值阻尼曲线或 Kd 已到协议上限而自动冻结

输出保留三层参数：`unconstrained_kp/kd` 是物理模型与相应设计需求产生的未约束建议，`pre_modal_kp/kd` 是协议和执行力矩约束后的配对值，`kp/kd` 是模态修正后的最终模型 seed；`modal_adjustments` 记录采样姿态、模态、参与轴、刚度参与度及调整动作，便于区分模型需求和执行约束的影响

用户配置中的 PD 上限已取消；旧 `max_kp_override/max_kd_override` 和平铺 `limits.max_kp/max_kd` 仅兼容接受，完全不参与校验、初始化与搜索；硬件协议上限、位置与速度范围、力矩预算及其他 Safety 限制继续生效

协议声明了增益步进时，控制周期先拒绝非法增益，再将合法请求投影到可编码网格，之后才执行参考治理与 Safety；`requested_kp/requested_kd` 保留投影前值，接受命令和有效增益保存实际可编码值；达妙增益编码使用 nearest，避免码中心再次向下截断；位置、速度、力矩的既有编码方式不变

故障刚性保持与柔性恢复也先投影增益，再按实际可编码值检查 `|FF+Kp·(q_ref−q)+Kd·(v_ref−v)|` 是否在既有力矩预算内；不可行时沿现有停止／失能路径退出，不下发超预算保持命令

自动任务的实际 seed 来源为 `urdf_inertia_and_unmeasured_recovery_bootstrap`，记录在 `model_initialization` 与 `seed_source`，`measured=false`；同时记录当前工作域、实际误差／速度边界及完整质量矩阵下的名义模态诊断；名义模态不是实测机械模态；模型、映射或安全余量无效时给出对应错误，修正后再启动；不能计算时不换成固定 Kp/Kd 并标记初始化完成

独立离线模型建议仍使用 `source=urdf_task_response_and_protocol_grid`；无示教时记录 `scope=unmeasured_recovery_design`、`demand_source/task_demand_source=recovery_targets_without_demonstration`，提供路径时记录 `scope=provided_path_offline_design`、`task_demand_source=provided_demonstration_path`；两者均未实测、未验证，不等于自动任务已经执行过的增益

初始化凭据将 `physical_prior`、`task_design_targets`、`user_constraints` 分开；物理先验来源为 `urdf_geometry_mass_com_inertia`、范围为 `M_C_g_only`，并列出排除的速度／力矩字段；离线重算建议不覆盖既有运行和恢复增益，凭据记录 `configuration_applied=false`、`runtime_baseline_unchanged=true`；采用与保存仍使用现有入口

### 两种短测证据

| 实验 | 激励入口与实际输入 | 反馈输出 | 能支持的判断 |
| --- | --- | --- | --- |
| 参考跟踪 | 已知参考位置、MIT 参考速度，以及实际执行的模型前馈 | 新鲜实际关节位置与速度 | 跟踪不足、速度相关误差、局部响应与有效延迟 |
| 独立扰动 | 固定参考下叠加的已知独立关节力矩，受既有力矩与运动限制 | 实际关节位移 | 已测频段的柔顺度、抗扰响应与恢复 |

MIT 的参考速度项会引入响应零点；局部闭环拟合使用已知输入 `Kp·R + Kd·Vref + Ffeedforward + Dindependent`，不把参考位置到实际位置直接套入无零点二阶系统，也不将参考跟踪截止频率解释为机械臂的真实反馈带宽

实际执行的原始 FF 仍完整进入已知输入；`MitExperiment.executed_state_feedforward_gradient` 另描述该执行模型前馈对实际位置的局部梯度；由数据拟合的 `best_k` 保留在 `physical_stiffness_nm_rad`，`nominal_physical_stiffness_nm_rad=Kp+nominal_G'` 只作为先验诊断，不覆盖拟合结果

用于响应频率、阻尼比及拟合不确定族判断的有效恢复刚度为 `best_k−executed_state_feedforward_gradient`；非正时不声明已辨识；因此，随实际位置变化的重力补偿不能被算成反馈刚度，良好前馈跟踪也不能替代独立扰动验证

拟合对若干已知频率提取参考、实际位置、前馈和扰动力矩的复数响应，在 URDF 惯量先验约束下估计局部惯量、阻尼及有效延迟；保留频点验证拟合质量，拟合不足则保留缺少的证据与原因，不填默认数值为成功结果

独立扰动按当前试验增益生成输入建议，并同时受既有标定力矩余量与 Robot 扰动力矩上限约束，后者仍为各轴 `max_effort` 的 5%；不再让极小模型 seed 决定整轮扰动幅值；连续健康但缺少有效扰动响应时，只对缺证据的轴在剩余预算内补测并增加已知输入，到原上限仍不足便记录不足，不放宽 Safety

静摩擦锁止时，已知输入下位置不动只能支持这次静态／恢复记录；`static_measurement_complete` 表示这部分数据完整，`motion_observed` 表示已观测到所需位移；未获得运动响应时，刚度测量的 `measurement_complete`、`passed` 仍为 false，原因保留为 `independent_response_below_motion_floor_static_behavior_only`

扰动运动门槛合并计划、静止噪声与当前新鲜位置中观测到的编码步长；静止采样恰好只有一个位置码时，不能把未知步长当成连续位置；单编码步跳动保留为静态／量化证据，不报告实际运动或释放振荡；新恢复记录保存 `encoder_step_rad` 与 `motion_floor_rad`

保持用途的独立扰动沿实际 HOLD 路径执行：保留当前名义保持目标，只逐周期刷新有界扰动力矩；不向 HOLD 提交跟踪参考，普通 `ControlRuntime::submit_reference()` 的模式守卫不放宽；参数记录 `reference_execution=fixed_hold_target`、`center_source=preserved_nominal_hold_target`，其他短测记录 `streamed_tracking_reference`

短测命令时间取主机总线成功写入完成时间，反馈时间取逐轴传输接收时间；它们不等于设备内部执行或采样时间，有效延迟包含控制流水线和通信；重复反馈不能增加独立样本数，长反馈间隙、非有限值或限幅会影响试验有效性；跟踪验收直接比较同周期的执行参考与原始反馈，不通过平移反馈、滤去峰值或放宽限制改善分数

短测幅值与频率同时考虑实际编码器步长、静止噪声、关节余量和执行器能力；若无法产生高于分辨率与噪声的安全运动，则该轴记录信息不足；快速标定只测当前用途需要的信息，精细标定在剩余预算内补充已发现的问题

### 起动响应建立与参考中心

参考短测开始前读取各轴新鲜实测位置作为局部中心，要求各轴接收戳均有更新且反馈年龄不超过 30 ms；运动建立过程固定该实测中心，位置与局部运动余量也从同一中心计算；不再用旧 nominal HOLD 目标覆盖它；短测参数记录 `center_rad` 与 `center_source`，其值分别用 `fresh_measured_establishment_center`、`established_comparison_center`、`fresh_measured_center` 区分新鲜实测起动中心、已建立响应后的比较中心与其他新鲜实测中心

有新鲜连续反馈、没有限幅，但响应低于编码器／噪声门槛、谱拟合较差、微弱非线性或只有单侧运动时，分别记录运动不足与辨识证据不足；软件按证据安排有界修正或补测，不把这些情况归成通信缺失，也不因此生成成功辨识；缺帧、无效反馈与实际限幅不能作为加大激励的依据

谱拟合质量不足但已有明显双向运动时，优先调整激励补证据；编码器量化边界先用同组参数补测；确认位移不足时也先修正激励，再按协议网格将 Kp 分步提高，单次最多约 1.4 倍或前进一码；没有可靠阻尼证据时不随 Kp 提高 Kd，也不从 Kd 乘速度噪声形成的 PD 峰反推所需 Kp

每次提出更高 Kp 后，还比较约束前后计划可产生的反馈输入；若增益升高伴随幅值收缩，输入增长不足以从编码器／噪声中区分，就拒绝这次增益上调，保留同一完整增益组，改用更慢、允许时更大的激励补测；不把增加 Kp 本身当作增加了有效激励

已有可见运动后，分别跟踪实际 PD 命令峰、总位移跨度和较小方向行程的改善；输入明显增加但行程停留平台时，以同一完整增益组完成较慢激励确认，确认期间其他轴也不同时增加增益；多频／低频计划到边界时，预算允许便自动改用同组增益的对称单频 `motion_probe`，正式窗口至少覆盖两个完整周期，仍受原标定力矩与 Safety 约束；完整单频试验后运动仍不足，才记录受阻轴并整组经原 HOLD 路径结束，不等待其他轴继续升参；记录说明试验条件下的输入／响应不足，不给出真实静摩擦值或硬件能力耗尽结论

单频补测只建立运动证据，`motion_probe_evidence` 保持 `tracking_valid=false`、`identifiable=false`，明确缺少三频响应；建立运动后，软件自动恢复真正的三频 `spectral-confirmation`、独立扰动和所需用途验证，不能用单频运动替代频响拟合或验收；参数记录逐轴 `motion_probe` 与 `duration_s`，单频时只执行 `frequencies_hz` 的首项，其余数组项为兼容保留值

历史 note 与实时诊断保存 `current_kp/current_kd`、`next_kp/next_kd`、按 `joint_names` 索引的 `blocked_axes` 及逐轴 `input_response_progress`；进展字段分别保留输入／行程锚点、当前实测与计划输入、较慢确认条件和受阻原因；不足警告包含 `motion_establishment_incomplete;<joint>:<reason>`；这是标定测量不足退出，实际异常往复的故障锁存仍按下文监督路径处理

运动建立不再固定尝试四轮或六轮后退出；每次重算完整试验成本并预留用途验证与退出时间，在总预算与既有 Safety、局部位移、力矩、速度和增益边界内继续；剩余时间不足、可观测／能力边界无进展时，保存已执行参数及不足原因并结束，仍无完整用途测量时不提供通过结论

仅差编码器量化边界时，最多补两次同完整增益组测量，保留各轴已建立的证据及其执行条件；整组通过仍依赖本趟完整测量，不把不同趟逐轴通过拼成未经联合验证的结果

控制周期另以新鲜实测位置、已接受参考和接收时间监督参考未要求的持续往复；采用位置迟滞、多次反转及参考运动条件比较，不仅依据速度反馈尖峰；触发后调用 `Robot::stop_calibration_response()`，以 `CALIBRATION_RESPONSE_UNSTABLE` 和既有 `STOP_HOLD` 锁存故障，使用原故障恢复增益与总力矩检查退出，不继续用本趟不稳参数保持写盘，不自动清故障或重试

`response_guard_events` 保存触发轴、时间、实际 Kp/Kd、位置／参考反转次数、行程和残差峰峰值；该事件说明本次响应不适合继续试验，不将其命名为已辨识机械共振；失败短测及整轨 CSV 也保存已取得的部分全周期数据

### 候选与用途验证

候选由模型和有效短测共同生成，再作少量有界局部比较；跟踪不足或独立扰动显示刚度不足时允许提高 Kp；阻尼达到上限时保留约束原因，Kp 是否继续变化由已有响应和独立安全边界决定；动态超调与持续振荡分别处理，单次超调不等于已辨识机械共振

每组参数在运动建立后的局部比较中，快速模式最多实际比较 1 次，精细模式按既有 `Requirements.max_candidates=3` 最多比较 3 次；每次仍计入共同计划重测、独立扰动与验证的实际成本，并服从剩余总预算和退出预留；达标或缺少有依据的候选时提前结束，运动建立仍按证据和总预算安排，不套用该比较次数

每次比较固定路径方向、速度、模型前馈、执行方式、负载声明与评价窗口；最终保留一整组实际测量过的参数，不把各轮逐轴最佳值拼成未验证的新组合

整轨未达标后的局部修正使用实测幅值、方向与速度相关残差提出小步 P/D 调整，残差中的 `lag_s` 是调参诊断，不声明已辨识物理延迟或极点；候选按实测误差峰值加编码／噪声余量及实际速度误差构成联合力矩包络，约束的是增量，不能通过径向缩放把增 P 建议变成大幅降低原 P/D；整组比较优先检查全周期峰值不恶化并有改善，严格 `<0.1°` 验收仍使用原始周期数据

起动时允许为了建立运动调整激励，但这不构成候选优劣比较；正式比较需要改变幅值或频段时，先生成两组参数都能安全执行的共同计划，在同一中心重测原候选，再测新候选；不能把原来的小幅响应与后来的大幅响应直接评分后声称增益改善

独立扰动比较同样冻结输入：首次原候选实际使用的扰动幅值包含必要补测，后续原候选与新候选共同使用这组实际输入；调整增益时不同时更换独立扰动强度以制造更好的抗扰结果

原生跟踪与刚性用途共享扰动证据时，需完整增益组、中心、参考、前馈与负载条件一致，并保留来源路径；中心变化则重新测量，只可将之前实际输入作为受原上限约束的补测起点；复用不会把原不足或未达标改成通过

刚性与伺服跟踪可以共享同条件原生跟踪测量；刚性还要验证独立整定的保持参数，伺服按实际参考执行路径验证，必要时由软件切入模型导纳验证；模型导纳和普通跟踪的参考、前馈不同，记录中保留各自执行条件

柔性可从共享模型生成较柔顺候选，但比例只用于提出候选，实际跟踪与保持仍需测量；拖拽维持 `Kp=0`，利用已有惯量、噪声和摩擦信息整定阻尼，检查运动释放后是否衰减，未建立运动不能记为通过

静态到位误差、实际位置漂移、受扰恢复与动态超调分别评价；积分补偿记录实际试验参数、逐轴状态、冻结原因和补偿力矩，建立时间与最终窗口按实际收敛速度安排，全过程异常运动仍纳入检查；积分改善保持不替代伺服跟踪验收

### 时间与结果语义

设 `T` 为可信来源保存的原始有效录制时长；安全重定时、重复导入和补测不增加 `T`

| 模式 | 目标预算 | 硬上限 | T=25 秒 |
| --- | --- | --- | --- |
| 四用途快速标定 | 12T | 15T | 目标 300 秒，最多 375 秒 |
| 单用途精细标定 | 12T | 24T | 目标 300 秒，最多 600 秒 |

预算覆盖自动初始化、过渡、短测、回放、等待、保持、拖拽、验证与退出；候选安排计入实际单趟成本并预留验证与退出时间，剩余时间不足以完成的试验不启动；到限经现有保持或停止路径收尾，达标可提前结束

整轨修正按已观测的完整一趟耗时安排回程与同方向比较，成本包含端点检查、参数过渡、轨迹、尾段及记录；验证与退出各预留一次；`full_path_correction_plan` 明确保存剩余时间、所需成本、缺额、候选与约束原因，以及执行或跳过理由，不以预算不足静默省略诊断

快标与精调对同一用途采用相同验收定义，差别在补测与局部比较深度；单用途精调只更新所选用途及必要结果，其他用途保留原值；共享数据不会自动给每个用途生成通过结论

机器接口使用 `gain_tuning_mode=quick` 配合 `gain_mode=all`，或使用 `gain_tuning_mode=fine` 配合一个 `gain_mode`：`rigid`、`compliant`、`drag`、`servo_tracking`；计划与启动沿用 `model_calibration_gain_plan`、`model_calibration_start`，停止沿用原任务取消／保持入口；`servo-probe-*.csv` 保存短测输入输出，`frames.csv` 保存逐周期完整执行证据

新 `frames.csv` 追加逐轴 `requested_kp/requested_kd` 与 `cmd_kp_code/cmd_kd_code`；协议码未知时为 `-1`，不能当作 0 码；短测和整轨 CSV 同时保存 `actual_kp`、`actual_kd`、`actual_gains_known`，已记录实际增益仍变化时以 `changing_feedback_gains/stable_executed_gain_window` 说明辨识不足；原有四种历史 frames CSV 格式继续可读，缺少新证据的旧记录不自动补成通过

自动试验的静止噪声、短测、整轨和拖拽阶段正常完成后，先冻结本趟采集并切入 HOLD，以最终名义目标和本趟实际增益保持，再移出数据、写全周期明细和计算指标；持续异常往复则先走锁存故障出口，不保留不稳试验增益；离线评估与慢速磁盘写入不让已经结束的轨迹继续占用 TRACKING，也不延长普通命令断流的超时门槛

`available` 表示所需参数已有完整实测：刚性／柔性需要整轨与实际采用的保持测量，伺服需要原生整轨及实际要求的导纳路径，拖拽需要完整释放测量；`qualification_complete` 另表示所需抗扰等验证证据完整，`validated` 表示完整证据达到验收目标；独立扰动不足不再隐藏已经测出的跟踪／保持参数，仍以未达标警告提供现有应用、保存与导出操作

现代历史记录若仅因旧聚合逻辑保存 `available=false`，读取时按完整嵌套整轨、保持及实际执行需求恢复参数可用性，并保留警告；不改写原始 JSON、CSV 或通过标志；缺少必要参数测量仍不可采用，旧局部记录保持其实际范围，缺少的新证据保持未测

任务状态、结果与候选共用 `termination={kind, reason, blocked_joint_names, fault_interrupted}`；`kind` 分为 `running`、`qualified`、`measured_not_qualified`、`insufficient_measurement`、`budget_exhausted`、`cancelled`、`execution_fault`；主动停止／取消记为 `cancelled`，测量不足保留具体轴与原因，真实控制或退出故障才记为 `execution_fault`；退出或写盘后才发现的故障会刷新最终对象，不能保留过期的通过结论；未达标、未测完整、到期、取消和故障卡片均保留至用户关闭

## 增益记录的离线复核

`audit_gain_calibration.py` 使用 Python 标准库，直接读取任务目录或增益成果 ZIP；不解压覆盖原文件、不修改 Profile，不连接硬件

```bash
# 输出 JSON 报告；可按需要重定向到另一个新文件
python3 tools/audit_gain_calibration.py /path/to/task \
  --compare /path/to/gain-artifact.zip

# 单独检查成果包已有证据，包中没有的完整任务文件会明确列出
python3 tools/audit_gain_calibration.py /path/to/gain-artifact.zip

# 只看实际退出原因、用时与逐轴短测响应，保留历史结论原值
python3 tools/audit_gain_calibration.py /path/to/task --summary

# 检查审计器的目录／ZIP、逐样本峰值、方向和旧结果读取
python3 tools/audit_gain_calibration.py --self-test
```

审计保留结果与候选的原始 `termination`，列出两者不一致及旧版取消状态被候选误写为故障的情况；正式窗口同时标明是否采完，部分 CSV 不自动变成完整辨识；单频计划按实际执行的首项频率报告，保留原数组供追溯

真实机器接口的离线集成检查使用现有构建的原生命令程序，在临时目录拷贝示例 Core 并强制 `write_enabled=false`，只启用离线 Mock，不传入任何硬件插件；临时示教与候选为兼容性测试数据，不作物理验收

```bash
./.install/run .install/gui-venv/bin/python tools/check_adaptive_gain_flow.py \
  --terminal build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal \
  --full-probes
```

`--terminal` 需指向本次重新编译的程序；该检查覆盖两模式计划预算、重定时／重复导入的 T 来源、实际工作线程停止、警告候选的单用途应用与恢复；`--full-probes` 另用两组旧配置运行参考和独立扰动短测，比较模型初值并检查完整记录；省略该选项只运行较短的接口检查；算法和模拟响应检查与真实机械臂验收分别记录

报告保留历史 `available`、`validated` 与警告原值，另列初始化来源、原始录制时长和预算、每个用途的验证范围、保持与积分记录、反馈完整度及逐样本绝对跟踪峰值；`--peak-target-deg` 默认 `0.1`，评价为严格小于该值，不改写原任务采用的历史验收目标

`measurement_coverage` 并列原始可用／通过标志、`qualification_complete` 与嵌套参数测量完整度，标明旧可用性聚合冲突；审计只说明证据覆盖，不执行候选应用；`full_path_correction_plans` 原样列出已有成本与跳过原因，旧任务没有记录时不补造

每份整轨 CSV 都逐行计算峰值、RMS、限幅次数和新反馈数，绘图抽样不参与计算；若成果包没有 `frames.csv` 或 `result.json`，报告相应证据缺失，不据此生成完整周期状态或实际总耗时

`short_probe_records` 从 `servo-probe-*.csv` 分别报告无新反馈／反馈不连续、参考未激励、有新反馈但位置不响应，以及有新反馈但响应较弱；同时保留原辨识原因、参考与实际跨度、初始参考偏差、参考包络是否覆盖初始实测位置、重复反馈序号和接收时间间隔；“弱响应”是实际跨度不足参考跨度 20% 的审计提示，不是增益验收阈值，也不生成成功辨识结果

独立扰动记录另列原 CSV 的已执行关节力矩命令峰值、位移与原始辨识／恢复结果；命令峰值不等于独立标定的物理电机力矩，零位移也不自动等于已辨识出刚度；静态恢复指标与有效扰动响应证据分别保留

`execution_summary` 将任务阶段与测量结果分开，列出实际总耗时、目标／硬预算、是否耗尽、整轨趟数及未开始用途；历史 `phase=complete` 且全部 `available=false` 的任务仍是无完整用途测量；`cycle_record.gain_stages` 另列 nominal、virtual、accepted 与实际位置范围、实际命令增益、前馈、反馈年龄和限幅证据

逐周期审计另列阶段起止行、实际增益变化、逐轴接收戳连续性，以及由已记录命令和反馈重建的 PD／总命令峰值及所在行；位置反转分析只使用有效的新鲜位置，附迟滞门槛与观测周期，不推断机械模态；重建范围是已记录的接受周期，旧记录没有保存的拒绝周期输入和故障轴保持缺失

`establishment_progression` 将逐轮实际参数、激励幅频、执行力矩域、真实 PD 命令峰、双向行程与决策原因并列；响应 RMS 比只在该试验的 `0.7 s` 至 `duration−0.7 s` 新鲜测量窗口计算，不用整份 CSV 的过渡与尾段改变结论；新记录保留输入／行程进展及受阻状态，旧记录没有这些字段时保持未记录

比较两轮候选时，报告参考轨迹两端是否互换；反向轨迹会改变摩擦、重力滞后和负载条件，不能把这种差别全部归因于增益变化；相同端点只支持端点一致，完整可比性仍需阶段、速度、前馈和负载记录

`--compare` 对两个输入共有的证据文件逐字节流式核对，列出不同与缺少的文件；不读取成果包中的说明作为执行指令

### 复现一次起动响应不足的提前退出

将原任务保持只读，在自己的保存位置运行以下命令；任务编号表示本例已有记录，不是新采集命令

```bash
python3 tools/audit_gain_calibration.py /path/to/model-calibration-1794213550334 \
  --compare /path/to/model-calibration-1794195059764 --summary
```

这份历史记录的 T 为 28.22 秒，目标 338.64 秒、硬上限 423.3 秒；实际 32.6574 秒后因初始闭环响应不足结束，剩余 390.6426 秒，`budget_exhausted=false`；四次参考短测已执行，整轨趟数为 0，刚性、柔性、拖拽与伺服用途均未开始，`available=false`、`validated=false`

短测每轴参考跨度约 `0.017079 rad`；joint1 与 joint3 四趟实际跨度均为零，joint2 前三趟为零、末趟约 `0.001526 rad`，其余轴主要只有一至两个编码步；每轴每趟约 1370 个新反馈，最大接收间隔小于 14.5 ms、没有记录到限幅；该证据属于“有新反馈但起动无响应／弱响应”，不是时间到期或通信反馈缺失

前三趟各轴辨识原因均为 `response_below_encoder_noise_floor`；第四趟 joint2 有少量位移后变为 `low_spectral_fit_quality`，拟合质量 `0.46491`，其余轴仍低于分辨率／噪声门槛；该历史流程随后停止，记录中没有独立扰动、整轨、保持或拖拽验收

首趟 joint3 参考从 `0.0429153 rad` 开始，实际为 `0.0543613 rad`；joint4 参考为 `-0.00514984 rad`，实际为 `0.0173569 rad`；两轴沿用的参考中心偏移分别约 `-0.011446 rad`、`-0.022507 rad`，整个摆动参考包络都未覆盖起始实测位置；审计会直接保留这些差异，不以名义保持目标替换原始实测位置

新旧任务的 Core、URDF、硬件原始及生效快照四文件字节相同；该记录用于回归起动、中心和退出诊断，不能作为修复后真机效果的验证

### 复现一次增益上升而行程停留平台

对已有 15 趟短测的原任务执行只读回归；此开关核对该份历史数据，不重新执行机械臂运动

```bash
python3 tools/audit_gain_calibration.py /path/to/model-calibration-1794236700517 \
  --check-establishment-regression --summary
```

这份历史任务中，joint3 的 Kp 从 `16.2393` 逐步升到协议上限 `500`，Kd 一直为 `2.78877`；第 2 至 15 趟频率不变，随后执行力矩域将参考幅值从 `0.0122624` 收到 `0.00658705 rad`；最后三趟实际总跨度均为 6 个编码步，实际 PD 命令峰为 `2.72932–2.78080 Nm`，升参没有继续增加可见行程；这些命令峰不作为真实静摩擦辨识值

末趟正式窗口有 805 个新鲜样本，响应 RMS 比为 `0.204693`，已经超过 `0.2`；卡点是正向 5 步、反向 3 步仍未达到每方向 6 步的要求，且实际位置 RMS `0.000505397 rad` 不足 2 个编码步；审计分别保留行程与辨识不足，不误报为 RMS 比不足，也不降低门槛生成通过

任务在 `115.982531 s` 以 `insufficient_measurement` 结束，剩余 `269.217469 s`，整轨及四用途验证均未开始；`fault_interrupted=false`、没有任务错误或响应监督事件；原任务并未保存后续持久连接会话的故障证据，不能据此解释用户后来看到的 fault

### 复现多频平台与取消结果分类

同一只读回归入口也支持以下两份原任务；原记录保持原值，新单频补测的效果由另行离线试验或之后主动发起的实机任务测量

```bash
python3 tools/audit_gain_calibration.py /path/to/model-calibration-1794238166406 \
  --check-establishment-regression --summary
python3 tools/audit_gain_calibration.py /path/to/model-calibration-1794238336638 \
  --check-establishment-regression --summary
```

`1794238166406` 已执行新输入平台规则：joint3 最后三趟保持 `Kp=241.39194/Kd=2.78877`，基频由 `0.3825` 降到 `0.26775`、`0.2 Hz`，末趟正式窗口随之延长至 6.25 秒，覆盖 1.25 个基频周期；实际跨度 6 步、正向 6 步／反向 5 步，小于约 7.96 步的每方向要求，正式窗口响应 RMS 比 `0.151085`；当次多频条件下的计划输入增长不足，随后在 `96.120622 s` 结束，剩余 `289.079378 s`；实际 PD 峰 `2.77466 Nm`、总命令峰 `8.82448 Nm`，没有记录到限幅，协议 Kp 上限仍为 500；该退出是多频试验平台，四用途未开始，并非 Safety 硬件余量耗尽

`1794238336638` 第一趟完整，第二趟仅保存 `4.378097 s`、每轴 862 个新鲜样本，正式窗口未采完；任务 `phase=cancelled`、`fault_interrupted=false`，实际 `14.523767 s`，原候选却保存 `outcome=execution_fault`；审计保留并标记这一分类冲突，按已有取消证据解释退出，不从该字段推断未记录的后续会话故障

### 复现已测参数后保持扰动入口被拒绝

修改前的源码检查点为 `360f329`；下面复核已有任务，不重新下发运动

```bash
python3 tools/audit_gain_calibration.py /path/to/model-calibration-1794241207035 \
  --check-establishment-regression --summary
```

原任务 T=23 秒、硬预算 345 秒，实际 `220.332346 s` 后退出，仍剩 `124.667654 s`；原生整轨完整记录每轴 6347 个新鲜样本，反向、速度 1、重力前馈，峰值误差逐轴约为 `1.601°/1.044°/1.077°/1.202°/1.215°/1.907°`，均未达 0.1°；保持已完成两趟，第二趟按当次 0.004 rad 保持门槛通过，积分未启用

最后 `servo-probe-rigid_tracking-hold-disturbance.csv` 只有表头；旧入口切入 HOLD 后仍提交 TRACKING 参考，首圈被模式守卫拒绝，任务记录 `short response input rejected`、`fault_interrupted=false`；刚性／伺服嵌套参数测量完整，但旧顶层可用性因扰动不足被聚合为 false；审计分别保留两层事实，不把软件入口失败归成已触发 Safety 或预算耗尽

首份独立扰动中 joint1 只跨一个编码步，旧噪声步长为零却报告释放振荡；只读回归保留此原始误判，修正后的判据由离线重放验证；新的 P/D 建议、预算安排和 HOLD 扰动执行仍需之后主动发起的实机任务测量效果

跟踪误差属于参考到位置的响应；没有已知独立扰动输入时，不据此声称已验证实际抗扰刚度或受扰恢复；观测器残差的去偏置与去摩擦也不能代替执行器前馈实测

若保持补偿试验已有积分样本，但保存的最终补偿为关闭，报告会单独标记这一情况；旧记录缺少该试验实际参数、逐轴积分状态或冻结原因时，不能从最终关闭配置反推出失败试验所用配置

末端接触力 `±0.5 N` 的验收需要对应的接触力测量，关节角误差和处理后的外力矩估计不提供该验收证据

## 采集与数据审计

正式采集沿已经检查、重定时的示教路径，执行逆向静态、正向静态、逆向半速、正向半速、逆向计划速度、正向计划速度；运动部分约为单程的 8 倍，另加双向静态停留

采集结束后机器人保持位置、恢复会话参数并保存任务；记录浏览和候选选择可独立于硬件进行

在线静态预览用于快速诊断，最终模型来自保存后的离线分析；`prior_retained` 表示保留输入模型，本轮未支持参数更新

采集中显示双向到位位置差与组数，超过 0.01 rad 时提示重复性问题并保留原始观测；这是位置差提示，采样有效性和反馈超时按运行条件另行判断

新任务在 `source/` 保存 `model.urdf`、`core.yaml`、`hardware.yaml`、`hardware-effective.yaml`，使用重力校正时另存 `gravity-correction.yaml`；metadata 记录各文件指纹、`hardware_overrides`、程序与驱动文件指纹以及执行器量程来源，便于核对原始配置与实际采集条件

### 静态观测

`static_observations.py` 是静态重力与完整 IDIM 共用的数据读取器

- 新记录只读 `accepted_static.csv`；每方向至少 30 帧，取末端最多 60 帧的中位数；检查有效值、反馈年龄 0–30 ms、速度 <0.04 rad/s、方向和训练/验证标签
- 保留各方向的实际 q，不先平均关节角；先计算各方向回归矩阵和力矩，再按姿态组等权平均
- 同一姿态的两个方向属于同一个训练或验证组，不能把一方向当训练、另一方向当验证
- 双向半差、组内 MAD 描述迟滞不确定性；两方向实际位置分别计算，单向组保留并赋更大的不确定性
- 双向/单向评分尺度下限为 0.02/0.05 Nm，属于工程评分设置；报告另存方向、实际位置差和不确定性

### 协议量程与反馈一致性

静态窗口检查观测稳定性，反馈中仍可能包含静摩擦、恒定外力或姿态相关扰动；比较实机补偿时另做同工况对照

Damiao 连接逐轴读取 PMAX/VMAX/TMAX，用于 MIT 发送及反馈解码，并清除旧尺度反馈缓存；读取失败拒绝连接；该过程只读寄存器，同型号电机也可能配置不同量程

连接前 capability 为配置默认值，激活后刷新后端量程并保存到任务 metadata 的 `actuator_capabilities`；来源区分硬件、mock 与不可用，新任务保留硬件原始／生效配置及连接覆盖；协议量程、关节安全限制和物理力矩标定分别处理，读回范围不会自动放大用户安全上限

离线先做位置/速度一致性审计，无需 Pinocchio 或连接硬件

```bash
./.install/run .install/gui-venv/bin/python tools/audit_calibration_kinematics.py \
  /path/to/task/frames.csv \
  --output /path/to/new-kinematic-audit.json
```

以非重叠时间窗比较 `delta_q = scale * integral(dq dt) + bias * delta_t`；低激励轴返回未知；报告诊断位置与速度记录的相对一致性，物理尺度还需外部参照；输入角度需连续且已解绕，该命令筛选 `static_reverse` 和 `friction_forward_fast` 阶段的有效帧，反馈年龄须在 0–30 ms 内

工具读取原始记录，结果写到新文件或标准输出；先处理反馈尺度问题，再进行动力学拟合

## 静态重力辨识

### 模型与拟合目标

静态模型写成 `g(q; θ) = Y(q) θ + g_fixed(q)`；自由参数为每个可校正连杆的质量 `m` 和一阶质量矩 `h = m c`，其中质量单位 kg、质心单位 m、一阶矩单位 kg·m、力矩单位 N·m

局部辨识将可信连杆完整保留在基线模型中，只对其余连杆求增量；全局辨识允许所有受控连杆的静态参数更新；静态数据不辨识完整转动惯量

令 `r = τ_feedback − g_prior`、`A` 为自由参数回归矩阵；按训练列尺度 `D` 归一化，并在训练 SVD 支持的子空间 `V_r` 内求解

```
Δθ = D⁻¹ V_r z
urdf:           min_z  Σ ||A Δθ − r||² + λ ||z||²
bias_separated: min_z  Σ ||C A Δθ − C r||² + λ ||z||²
```

`C` 按关节减去训练姿态均值；后一模式的常值偏置是 `b = mean_train(g_candidate − τ_feedback)`，验证时冻结此值；前一模式拟合偏置为零，不扣掉反馈常值误差

| 目标 | 适用问题 | 限制 |
| --- | --- | --- |
| `urdf`，需显式选择的实验对照 | 单独导出 URDF 后，原始重力预测是否更接近记录反馈 | 拟合可能吸收反馈偏置，参数物理性与实机补偿另行检查 |
| `bias_separated`，默认 | 分离每关节常值反馈偏差后，姿态相关项是否可解释 | 按去偏置残差拟合，独立 URDF 按原始预测评估 |

高级动力学辨识入口的静态分支显式保留 `bias_separated` 目标，但使用相同的原始 URDF 改善验收；在线 C++ 快速预览仍为固定质量的一阶矩估计，与离线结果不等价

### 选择与评估

- 每个姿态、每个关节等权；同一姿态训练/验证标签冲突时拒绝求解
- 仅训练姿态参与 leave-one-pose-out CV，每折重新计算缩放、SVD 与偏置；外部留出姿态不选正则参数
- 正则候选为保留原模型、`0, 1e-6, 1e-4, 1e-3, .01, .1, 1, 10, 100, 1000`；不可辨识方向在归一化参数空间保留先验
- 无质量/质心经验硬边界；非有限值、不可用于质心换算的质量拒绝导出；未辨识的质心处惯量张量保持原值
- 验收比较实际导出参数的原始预测；去偏置指标单独列出，不能替代原始指标

默认离线改善要求：至少三个训练姿态、两个留出姿态、非零支持秩、参数确有更新；聚合 RMS 下降严格大于 `max(0.001 N·m, 基线 RMS 的 1%)`，任一关节 RMS 退化不超过 `0.02 N·m`；还须通过参数有效性检查

上述阈值为工程判据；状态为 `improved`、`equivalent`、`degraded`、`prior_retained` 或 `insufficient_evidence`

导出按 `c=h/m` 将一阶矩转换为质心后写入 URDF `origin xyz`；质量或质心变化也会改变质量矩阵与科氏项，动态使用需另行验证

### 偏置与外力观测

`fitted_torque_bias` 为拟合常值偏差，`urdf` 模式为零；`diagnostic_training_residual_mean` 为训练残差均值

兼容字段 `torque_bias` 保留后者，采用 `model_minus_measured` 符号；它可随人工导出的 Core 配置供 residual observer 使用，不叠加到 URDF 重力前馈，也不参与原始 RMS 判定

观测器阈值从去偏置验证残差与原训练残差尺度计算；`noise_rms` 表示记录中的评分尺度

### 静态候选操作

GUI：加载重力任务记录 → 选择全局/局部及可信 Link → 选择离线拟合目标并重算 → 选定本轮候选 → 审核原始/去偏置分数、状态和质量/质心 → 预览保存或导出

CLI 的 `--config` 为记录入口参数，拟合使用任务中经过校验的源快照

```bash
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py recompute \
  --config /path/to/core.yaml --directory /path/to/task \
  --mode local --locked-link trusted_link \
  --objective bias_separated
```

`trusted_link` 替换为模型中本次确认可信的连杆名称；多个可信连杆可重复传入 `--locked-link`；全部连杆参与拟合时使用 `--mode global` 并省略 `--locked-link`；`--objective urdf` 选择直接预测实验对照

每次重算独立归档 `offline_candidates/<run_id>/recomputed-candidate.json`；根目录同名文件仅代表最近一次结果，选择轮次后，保存与导出均使用该轮归档而不重新拟合

CLI 重算返回 `candidate_run_id` 和 `candidate_path`，后续 `preview-save`、`save`、`export-urdf` 使用同一 `--source offline --run-id ID`；也可用 `--candidate-path PATH` 替代轮次编号，两者互斥；任务存在离线候选时须明确来源，在线结果使用 `--source online`

```bash
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py preview-save \
  --config /path/to/core.yaml --directory /path/to/task \
  --source offline --run-id ID

# 检查上一步差异后保存重力校正，并更新 Core YAML 引用
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py save \
  --config /path/to/core.yaml --directory /path/to/task \
  --source offline --run-id ID --confirm

# 独立导出候选 URDF，不替换当前运行模型
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-urdf \
  --config /path/to/core.yaml --directory /path/to/task \
  --source offline --run-id ID --destination /path/to/candidate-export
```

`ID` 使用本轮返回值；需要撤回保存时使用任务记录中的备份恢复配置

```bash
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py restore-config \
  --config /path/to/core.yaml --directory /path/to/task --confirm
```

重力校正文件同时保存选定候选的质量与一阶矩，仅用于运行时重力覆盖；导出 URDF 则按 `c=h/m` 写入模型质量与质心，并校验这一轮模型

GUI 记录导出包含历史候选；成果包包含 `robot.urdf`、`core.yaml`、`identification-report.json`、`candidate-verification.json`，用模型与报告 SHA 确认对应关系；CLI `export-urdf` 导出所选候选及其校验文件，不生成完整机械臂 Profile

原始数据、已验收静态数据或模型变化后需重新计算

候选 URDF 的动力学参数或导出的重力缩放改变时，导出 Core 将撤销原模型的 `gravity_model_validated=true`；离线静态通过不继承实机故障恢复的模型验证状态，`allow_compliant_recovery` 偏好保持原值

## 完整动力学辨识

### 预处理与物理拟合

对实测 q 使用四阶零相位 Butterworth 滤波，默认 5 Hz，也可选择 2/10 Hz 比较结果；按独立连续约 10 秒块重采样并中心差分得到 v/a；无效值、阶段变化或 >25 ms 间隔切段，至少保留 3.5 秒连续数据，各块去掉 1.5 秒边缘；回归量、基线和响应统一使用 5 Hz 低通并降到约 20 Hz，在线加速度与参考导数不参与拟合

运动方程为 `tau = Y(q,v,a) theta + b + Fc sign(v) + Fv v + residual`，各轴主要运动方程要求 `|v|>0.03 rad/s`；静态组以实际姿态的 `Y(q,0,0)` 和静态观测加入同一个训练目标；静态方程只使用零速模型，静摩擦归入观测不确定性

每 Link 的质量分布矩阵为 `J = [[0.5 tr(I_O) Id - I_O, h], [h^T, m]]`；在源模型 Cholesky 同余归一化下直接约束 `J` 正定，使用 CVXPY/Clarabel 求解；约束直接作用于完整待估块，无经验质量或质心硬边界；源模型需提供严格物理参考

全局与局部共用模型和预处理；局部固定块从系统构造开始排除出优化，结果输出完整模型；导出时将连杆原点惯量转换到 COM 后写入 URDF

软正则是源质量分布同余坐标的 Frobenius 距离；系数 `{0.01,0.1,1,10,100,1000}` 及“不更新”由训练集内部三折选择；至少需要三个训练静态姿态组与三个连续时间块；每折同时留出完整时间块和整个静态姿态组，外部验证会话和本次静态留出组不参与选择

WLS 使用训练源模型残差的归一化尺度，OLS 使用统一轴尺度；IRLS 先迭代 Huber 诊断权重，再进行物理约束重拟合；WLS 的尺度来自当前数据，不是独立标定的传感器协方差

基线和候选的偏置／摩擦在同一训练静态与运动系统中估计，再冻结用于验证；纯重力评分使用原始预测，运动预测和重力补偿分别评价

方法参考：[IDIM 基准](https://doi.org/10.3390/app11094303)、[物理惯量 LMI](https://arxiv.org/abs/1701.04395)、[参数可观察性](https://arxiv.org/abs/1711.03896)

### 推荐模型与拟合候选

`recommendation=keep_source_prior` 表示推荐保留输入模型；最佳收敛的物理拟合仍作为审核对照保留，推荐结果与拟合结果分开展示

报告区分别展示质量、COM、完整惯量差值、运动误差、纯重力误差、物理性及缺少的输入条件；3D 使用完整候选的一阶矩与质量作只读质心对照；几何绑定不同或不明确时不显示不可信的对照

未采样夹爪关节应全程固定并声明实际位置；未知时按中立位诊断并保留假设；补充装机说明独立标记为操作者声明，与原始 metadata 和测量分别保存，且须与已有条件一致

### 导出选择

1. **诊断报告**：导出完整报告、SHA 记录及原始／拟合／无约束参数对照，产物为报告文件
2. **待审核 URDF**：模型有参数更新且物理有效，完整惯性参数回读和 FK 通过检查；缺少的独立会话或条件声明随报告列出，模型供离线审核
3. **通过离线检查的模型**：具有独立会话，模型／装机／映射／重力／量程一致、输入质量通过、未采样关节位置已知，运动 RMS 改善超过 2%、纯重力通过改善判据、运动回归满列秩，且训练 CV 未选择保留原模型；导出后继续进行真机验收

导出绑定选定轮次的报告并写入新目录；URDF 要求物理有效，报告与源数据哈希匹配；当前 Core、观测器参数和运行模型保持原值

默认报告归档在任务目录的 `idim_candidates/<run_id>/idim-report.json`，根目录的 `idim-candidate.json` 为最近一轮结果副本；导出使用所选轮次的报告及同目录 `receipt.json`；移动数据时须保留报告引用的训练和验证任务目录

模型导出包含 `robot.urdf`、`idim-report.json`、`receipt.json`、`verification.json`、`README.txt` 与复制的模型资源；其中不包含增益、摩擦补偿或观测器偏置，也不生成机械臂 Profile；`package://` 网格引用保留原样，目标环境需能解析对应资源包

### 完整动力学命令

```bash
# 检查阶段覆盖、量程记录与 q/dq 一致性
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py audit-dynamics --directory /path/to/task

# 使用独立会话验证完整全局模型
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py identify-dynamics \
  --directory /path/to/train --validation-directory /path/to/independent \
  --mode global --estimator WLS --cutoff-hz 5

# 局部辨识；trusted_link 替换为本次确认可信的连杆，多个连杆重复传入 --locked-link
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py identify-dynamics \
  --directory /path/to/train --mode local --locked-link trusted_link

# 若确实已知且全程固定，可追加多个 --fixed-joint actual_name=value
# 缺失装机说明可追加 --load-description '实际已知条件'；单列为操作者声明

./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-dynamics \
  --report /path/to/idim-report.json --destination /new/review-dir --review-only

./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-dynamics \
  --report /path/to/idim-report.json --destination /new/diagnostics-dir --report-only

# 报告已通过全部离线检查时，省略 --review-only 和 --report-only
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-dynamics \
  --report /path/to/idim-report.json --destination /new/validated-dir
```

## 模型对照与独立验证

离线报告分别评价运动预测和纯重力误差；静态候选为 `improved` 或完整模型通过离线检查后，补偿效果继续用独立会话与同工况对照验证

1. 保留已知可用 URDF 作对照，固定控制参数、装机、夹爪和载荷
2. 使用已知质量及安装位置做增量载荷对照，在相同姿态重复记录 q/tau；理论增量为 `Jv(q)^T * delta_m * g`，其中 `Jv` 为载荷质心位置的线速度雅可比，检查反馈力矩的符号、量级和重复性；PMAX/VMAX/TMAX 说明协议量程
3. 核对双向实际到位差、迟滞和夹爪固定位置，再采独立会话
4. 对比原模型与候选的纯重力保持、漂移和运动预测，保留退化姿态与失败结果

