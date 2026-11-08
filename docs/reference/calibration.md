# 动力学标定参考

静态重力辨识校正质量与一阶质量矩，完整动力学辨识进一步估计惯量和运动相关项；采集、记录管理与界面操作见 [使用指南](../guide.md)

## 数据与运行条件

- 使用源码安装的 GUI 组件与安装器生成的 Python 环境；完整动力学辨识使用 NumPy、SciPy、Pinocchio、CVXPY 和 Clarabel
- 保留完整任务目录、`metadata.json`、`frames.csv`、已验收静态数据及 `source/` 源快照；新任务静态数据来自 `accepted_static.csv`
- 数据审计与拟合前断开运行会话；记录浏览和已保存成果导出可在工作台连接期间进行
- 独立验证使用同模型、同装机和同映射的另一采集会话；同会话留出用于评价本次记录

以下命令从仓库根目录执行，路径、任务编号和连杆名替换为自己的数据

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

