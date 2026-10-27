# 离线静态重力辨识

离线重算从已保存的静态姿态提出重力预测候选；不控制机械臂，不自动替换 Profile，不将预测拟合等同于真实质量/质心测量或实机补偿改善

## 模型与两种拟合目标

静态模型写成 `g(q; θ) = Y(q) θ + g_fixed(q)`；自由参数为每个可校正连杆的质量 `m` 和一阶质量矩 `h = m c`，其中质量单位 kg、质心单位 m、一阶矩单位 kg·m、力矩单位 N·m

局部辨识将可信连杆完整保留在基线模型中，只对其余连杆求增量；全局辨识允许所有受控连杆的静态参数更新；静态数据不辨识完整转动惯量

令 `r = τ_feedback − g_prior`、`A` 为自由参数回归矩阵；按训练列尺度 `D` 归一化，并在训练 SVD 支持的子空间 `V_r` 内求解：

```
Δθ = D⁻¹ V_r z
urdf:           min_z  Σ ||A Δθ − r||² + λ ||z||²
bias_separated: min_z  Σ ||C A Δθ − C r||² + λ ||z||²
```

`C` 按关节减去训练姿态均值；后一模式的常值偏置是 `b = mean_train(g_candidate − τ_feedback)`，验证时冻结此值；前一模式拟合偏置为零，不扣掉反馈常值误差

| 目标 | 适用问题 | 限制 |
| --- | --- | --- |
| `urdf`，需显式选择的实验对照 | 单独导出 URDF 后，原始重力预测是否更接近记录反馈 | 可能把反馈偏置吸收入惯性参数，不代表参数物理真值 |
| `bias_separated`，兼容默认 | 分离每关节常值反馈偏差后，姿态相关项是否可解释 | 去偏置分数变好不保证单独 URDF 的预测变好 |

2026-10-27 复核确认直接预测目标在相同数据上未取得稳定改善，且用户反馈新 URDF 实机退化，因此撤回其默认地位；恢复兼容默认只撤销未经验证的行为变更，不代表已经恢复历史可用版效果

高级动力学辨识入口的静态分支显式保留 `bias_separated` 目标，但使用相同的原始 URDF 改善验收；在线 C++ 快速预览仍为固定质量的一阶矩估计，与离线结果不等价

## 选择、评估与导出

- 每个姿态、每个关节等权；同一姿态训练/验证标签冲突时拒绝求解
- 仅训练姿态参与 leave-one-pose-out CV，每折重新计算缩放、SVD 与偏置；外部留出姿态不选正则参数
- 正则候选为保留原模型、`0, 1e-6, 1e-4, 1e-3, .01, .1, 1, 10, 100, 1000`；不可辨识方向在归一化参数空间保留先验
- 无质量/质心经验硬边界；非有限值、不可用于质心换算的质量拒绝导出；未辨识的质心处惯量张量保持原值
- 验收比较实际导出参数的原始预测；去偏置指标单独列出，不能替代原始指标

默认离线改善要求：至少三个训练姿态、两个留出姿态、非零支持秩、参数确有更新；聚合 RMS 下降严格大于 `max(0.001 N·m, 基线 RMS 的 1%)`，任一关节 RMS 退化不超过 `0.02 N·m`；还须通过参数有效性检查

上述阈值是明确公开的工程判据，不是传感器噪声测量；两个同会话留出姿态仅满足计算门槛，不能证明跨会话泛化或实机效果；状态区分 `improved`、`equivalent`、`degraded`、`prior_retained`、`insufficient_evidence`

导出按 `c=h/m` 写入质心，绝不能把 `first_moments` 直接填入 URDF 的 `origin xyz`；改变质量/质心也会改变完整动力学模型的质量矩阵与科氏项，静态验证不能为这些动态项背书

## 偏置与 observer 契约

报告中的 `fitted_torque_bias` 是拟合目标内的 nuisance offset，`urdf` 模式为零；`diagnostic_training_residual_mean` 为训练残差均值

兼容字段 `torque_bias` 保留后者，采用 `model_minus_measured` 符号；它可随人工导出的 Core 配置供 residual observer 使用，不叠加到 URDF 重力前馈，也不参与原始 RMS 判定

observer 的阈值从去偏置验证残差与原有训练残差尺度计算，避免已减去的常值偏差又进入死区；`noise_rms` 是历史评分尺度，不应解释成传感器噪声实测值

## 使用与复现

GUI：断开硬件后加载重力任务记录 → 选择全局/局部及可信 Link → 选择离线拟合目标 → 离线重算 → 审核原始/去偏置分数、状态和质量/质心 → 导出供审核

CLI 的 `--config` 为记录入口参数，拟合使用任务中经过校验的源快照：

```bash
python3 tools/model_calibration_cli.py recompute \
  --config /path/to/core.yaml --directory /path/to/task \
  --mode local --locked-link Link1 --locked-link Link2 \
  --locked-link Link3 --locked-link Link4 --locked-link Link5 \
  --objective bias_separated
```

将 `--objective` 改为 `urdf` 可运行直接预测实验对照；名称应使用自己模型中的 Link，不能照搬示例

每次重算独立归档 `offline_candidates/<run_id>/recomputed-candidate.json`；根目录同名文件仅代表最近一次结果，导出使用最近一次已存候选而不重新拟合；记录导出包含历史候选，成果包包含 `robot.urdf`、`core.yaml`、`identification-report.json`、`candidate-verification.json`，用模型与报告 SHA 确认对应关系

若原始数据、已验收静态数据或模型发生变化，需重新计算；得到 `improved` 后仍须用独立会话数据及同工况的原模型/候选对照验证补偿效果

## 采集数据与实际协议量程

静态窗口通过仅表示观测稳定；静摩擦、恒定外力或姿态相关扰动仍可能混入反馈；不能把更低的静态 RMS 直接称为更好的实机补偿

Damiao 连接会逐轴只读 PMAX/VMAX/TMAX；全部确认后统一用于 MIT 发送和反馈解码，并清除旧尺度反馈缓存；失败会拒绝连接，不根据型号默认值继续使能，也不自动写电机寄存器；同型号的不同电机可以有不同范围

连接前的 capability 是配置默认值；成功激活后终端刷新后端 capability，并在新 calibration metadata 的 `actuator_capabilities` 中保存；这表示协议编码范围，不是关节 Safety 限速，也不代表物理 Nm 已校准；用户安全限制不会自动放大

离线先做位置/速度一致性审计（无需 Pinocchio、无需连接硬件）：

```bash
python3 tools/audit_calibration_kinematics.py /path/to/task/frames.csv \
  --output /path/to/new-kinematic-audit.json
```

以非重叠时间窗比较 `delta_q = scale * integral(dq dt) + bias * delta_t`；低激励轴返回未知，不能把未知当作通过；报告只诊断两种记录量的相对一致性，不能独立判断哪种物理尺度正确；输入角度需连续且已解绕，默认只分析有效自动回放阶段

工具不修改 CSV、参数或 URDF，不会覆盖现有输出文件；无 `--output` 时将 JSON 写到标准输出；换算异常未查清前，不应靠动态回归参数吸收传感器尺度错误

候选 URDF 的动力学参数或导出的重力缩放改变时，导出 Core 将撤销原模型的 `gravity_model_validated=true`；离线静态通过不继承实机故障恢复的模型验证状态，`allow_compliant_recovery` 偏好保持原值
