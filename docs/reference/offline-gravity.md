# 离线静态重力辨识

加载已保存的静态姿态，比较原模型与候选的重力预测，审核后导出模型；实际补偿效果在独立会话中验证

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

高级动力学辨识入口的静态分支显式保留 `bias_separated` 目标，但使用相同的原始 URDF 改善验收；在线 C++ 快速预览仍为固定质量的一阶矩估计，与离线结果不等价

## 选择、评估与导出

- 每个姿态、每个关节等权；同一姿态训练/验证标签冲突时拒绝求解
- 仅训练姿态参与 leave-one-pose-out CV，每折重新计算缩放、SVD 与偏置；外部留出姿态不选正则参数
- 正则候选为保留原模型、`0, 1e-6, 1e-4, 1e-3, .01, .1, 1, 10, 100, 1000`；不可辨识方向在归一化参数空间保留先验
- 无质量/质心经验硬边界；非有限值、不可用于质心换算的质量拒绝导出；未辨识的质心处惯量张量保持原值
- 验收比较实际导出参数的原始预测；去偏置指标单独列出，不能替代原始指标

默认离线改善要求：至少三个训练姿态、两个留出姿态、非零支持秩、参数确有更新；聚合 RMS 下降严格大于 `max(0.001 N·m, 基线 RMS 的 1%)`，任一关节 RMS 退化不超过 `0.02 N·m`；还须通过参数有效性检查

上述阈值为工程判据；同会话留出只评价本次记录，补偿效果另用独立会话验证；状态为 `improved`、`equivalent`、`degraded`、`prior_retained` 或 `insufficient_evidence`

导出按 `c=h/m` 将一阶矩转换为质心后写入 URDF `origin xyz`；质量或质心变化也会改变质量矩阵与科氏项，动态使用需另行验证

## 偏置与 observer 契约

`fitted_torque_bias` 为拟合常值偏差，`urdf` 模式为零；`diagnostic_training_residual_mean` 为训练残差均值

兼容字段 `torque_bias` 保留后者，采用 `model_minus_measured` 符号；它可随人工导出的 Core 配置供 residual observer 使用，不叠加到 URDF 重力前馈，也不参与原始 RMS 判定

观测器阈值从去偏置验证残差与原训练残差尺度计算；`noise_rms` 表示记录中的评分尺度

## 使用与复现

GUI：加载重力任务记录 → 断开运行会话后选择全局/局部及可信 Link → 选择离线拟合目标并重算 → 选定本轮候选 → 审核原始/去偏置分数、状态和质量/质心 → 预览保存或导出供审核；已保存候选的普通导出可在连接期间进行

CLI 的 `--config` 为记录入口参数，拟合使用任务中经过校验的源快照：

```bash
python3 tools/model_calibration_cli.py recompute \
  --config /path/to/core.yaml --directory /path/to/task \
  --mode local --locked-link Link1 --locked-link Link2 \
  --locked-link Link3 --locked-link Link4 --locked-link Link5 \
  --objective bias_separated
```

将 `--objective` 改为 `urdf` 可运行直接预测实验对照；名称应使用自己模型中的 Link，不能照搬示例

每次重算独立归档 `offline_candidates/<run_id>/recomputed-candidate.json`；根目录同名文件仅代表最近一次结果，选择轮次后，保存与导出均使用该轮归档而不重新拟合

CLI 重算返回 `candidate_run_id` 和 `candidate_path`，后续 `preview-save/save/export-urdf` 使用同一 `--source offline --run-id ID`；也可用 `--candidate-path PATH` 替代轮次编号，两者互斥；任务存在离线候选时不省略来源，在线结果使用 `--source online`，完整命令见 [工作台指南](../launcher.md#terminalros2-与命令行)

重力校正文件同时保存选定候选的质量与一阶矩，仅用于运行时重力覆盖；导出 URDF 则按 `c=h/m` 写入模型质量与质心，并校验这一轮模型

记录导出包含历史候选，成果包包含 `robot.urdf`、`core.yaml`、`identification-report.json`、`candidate-verification.json`，用模型与报告 SHA 确认对应关系

若原始数据、已验收静态数据或模型发生变化，需重新计算；得到 `improved` 后仍须用独立会话数据及同工况的原模型/候选对照验证补偿效果

## 采集数据与实际协议量程

静态窗口检查观测稳定性，反馈中仍可能包含静摩擦、恒定外力或姿态相关扰动；比较实机补偿时另做同工况对照

Damiao 连接逐轴读取 PMAX/VMAX/TMAX，用于 MIT 发送及反馈解码，并清除旧尺度反馈缓存；读取失败拒绝连接；该过程只读寄存器，同型号电机也可能配置不同量程

连接前 capability 为配置默认值，激活后刷新后端量程并保存到任务 metadata 的 `actuator_capabilities`；来源区分硬件、mock 与不可用，新任务保留硬件原始／生效配置及连接覆盖；协议量程、关节安全限制和物理力矩标定分别处理，读回范围不会自动放大用户安全上限

离线先做位置/速度一致性审计（无需 Pinocchio、无需连接硬件）：

```bash
python3 tools/audit_calibration_kinematics.py /path/to/task/frames.csv \
  --output /path/to/new-kinematic-audit.json
```

以非重叠时间窗比较 `delta_q = scale * integral(dq dt) + bias * delta_t`；低激励轴返回未知，不能把未知当作通过；报告只诊断两种记录量的相对一致性，不能独立判断哪种物理尺度正确；输入角度需连续且已解绕，默认只分析有效自动回放阶段

工具读取原始记录，结果写到新文件或标准输出；先处理反馈尺度问题，再进行动力学拟合

候选 URDF 的动力学参数或导出的重力缩放改变时，导出 Core 将撤销原模型的 `gravity_model_validated=true`；离线静态通过不继承实机故障恢复的模型验证状态，`allow_compliant_recovery` 偏好保持原值


## 完整动力学辨识

完整惯性参数使用 [全局/局部 IDIM 流程](dynamic-identification.md)，静态重力重算只估计质量与一阶矩

## 实验案例：拟合目标选择

2026-10-27 同数据复核中，直接预测目标未取得稳定改善，操作者报告新 URDF 的实机补偿退化，因此默认恢复为 `bias_separated`；历史效果尚需同工况对照复现

相关采集数据、模型排序与后续验证步骤见 [完整动力学辨识的实验案例](dynamic-identification.md#实验案例2026-10-29-采集复核)
