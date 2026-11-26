# 控制整定与模型标定参考

本页定义采集、拟合、验证与成果采用的含义；界面步骤见 [使用指南](../guide.md)，接口见 [API](api.md)

## 任务与结果

| 任务 | 输入与方法 | 产物 |
| --- | --- | --- |
| MIT 增益整定 | 模型初值、已检查轨迹、实际输入输出 | 完整实测 PD 组与用途测量 |
| 外力校准 | 无接触静止、双向运动、独立姿态 | 偏置、摩擦、残差包络与验收 |
| MDK 整定 | 已验证外力、有限刚度试验、释放窗口 | 虚拟参数与实际响应记录 |
| 重力模型校正 | 多姿态静态数据 | 质量/一阶矩候选或重力覆盖 |
| 完整动力学辨识 | 静态组、多速度运动、独立会话 | 物理惯性模型与验证报告 |

拟合、数据完整和质量通过分别记录；候选应用与持久保存是独立动作；参数或模型改变后，撤销受影响的验收

准备页按任务汇总结果：MIT 的增益与定位响应共用参数和检查记录；CSP 的外力检查、校准与独立验证在同一任务完成，有限刚度退让与释放回位使用同一次响应记录

工具保存结果后关联准备页；人工检查另存观察结论，不替代独立外力与响应实测；记录重启后保留，相关参数改变后显示需复查

## 数据与运行条件

保留任务目录、metadata、frames、accepted_static 与 source 快照；数据审计和离线拟合在断开运行会话后执行；记录浏览和已保存成果导出可在线进行

独立验证使用同模型、装机与映射的另一采集会话；同会话留出用于本次记录评价；完整动力学工具使用安装器准备的 NumPy、SciPy、Pinocchio、CVXPY 和 Clarabel

配置和报告采用 SI；力矩、力、位置与速度按模型关节类型标注

## MIT 增益整定

### 模型初始化

物理先验来自 URDF 几何、质量、质心与惯量，经映射得到 M/C/g；人工速度、力矩和 Safety 限制只约束执行，不是已识别的机械参数

名义设计使用完整质量矩阵，取 Rii=Σj|Mij|，令 Kp_i=ω²Rii；默认刚性 f=2 Hz、柔性 f=1 Hz、ζ=0.7；频率与阻尼比是设计目标

阻尼采用双对角化：KΦ=MΦΩ²、ΦᵀMΦ=I，D=2MΦ diag(ζ_iω_i)ΦᵀM；MIT 使用 diag(D)，完整矩阵及对角近似分别记录；方法见 [Albu-Schäffer 等，ICRA 2003](https://www.informatik.uni-bremen.de/agebv2/downloads/published/albuschaeffericra03.pdf)

Profile 创建使用模型零姿；轨迹任务使用时间加权的质量矩阵；下发前再投影协议网格并检查实际力矩预算；原始建议、可编码参数与实测参数分别保存

### 在线测量与更新

先完成保守 PD 保持与小运动，再选择已检查轨迹；以轨迹 1× 时钟测量已知输入、接受参考、反馈和实际增益，依据有效局部响应小步调整 P/D；证据不足时保留可执行组，预算内可进行同向短段实测比较

最终冻结一整组参数执行完整测量；不拼接不同轮次的逐轴最佳值；跟踪、抗扰、保持、柔性和拖拽释放各自保留测量覆盖

独立力矩输入不超过既有 Safety 力矩上限的 5%，并受协议码、实际完整 PD、位置和力矩余量共同限制；实际输入与响应不足时保留缺测原因

参考时序、动态峰值、RMS、局部换向超调、最终偏差、保持漂移和释放衰减用于性能分析；增益任务不设置统一精度合格阈值；持续非指令振荡由 CALIBRATION_RESPONSE_UNSTABLE 经 STOP_HOLD 锁存

以原始有效录制时长 T 为基准，目标 12T，快速上限 15T，精细上限 24T；自动过渡、等待、测量与收尾计入预算，录制与人工对齐不计入

间接自整定与闭环辨识依据见 [Åström 与 Wittenmark](https://portal.research.lu.se/en/publications/on-self-tuning-regulators/) 和 [Gilson 与 Van den Hof](https://publications.pvandenhof.nl/Paperfiles/Gilson%26vdHof_Automatica2005_final.pdf)；本项目使用自己的有限预算实现

### 离线复核

```bash
python3 tools/audit_gain_calibration.py /path/to/task --summary
python3 tools/audit_gain_calibration.py /path/to/gain-artifact.zip
python3 tools/audit_gain_calibration.py --self-test
```

复核保留原始退出原因、逐样本峰值、用途范围与历史结果，不重写历史结论

## 外力校准与验收

### 观测状态

反馈按主机接收时间对齐，各轴插值到共同时间且不外推；观测器仅在有效新测量上推进

低速量化区采用 100 ms 位置窗口的最小二乘速度，并在反馈速度的 3–4 个量化步进间平滑过渡到上报速度；同一速度用于模型、动量和摩擦标定；原始反馈另行保留

外力 residual 使用模型减实测的符号；偏置和摩擦在观测器内补偿，连续死区移除小残差；它们不加入 CSP 命令；FULL-ID 残差单列诊断

### 采集顺序

1. 在刚性保持下核对模型、实际负载、反馈力矩方向与比例，检查不同姿态的无接触残差和双向到位差
2. 在无外部接触、无连杆搭靠和未顶住机械限位的条件下，静态校准采集 8 个代表姿态，每姿态连续 15 秒；包含正反向到位及返回姿态，观察慢变与滞回
3. 用已检查轨迹进行正反向摩擦采集和交叉验证；运行与拟合使用同一低速摩擦基函数；不用摩擦补偿时明确关闭
4. 冻结校准参数，另用 5 个独立姿态再次连续采样，评价 P99、MAX 和逐轴结果；记录受控正反加载的方向检查
5. 保存独立验证结果及原始 q、重力、反馈力矩、采集时间与配置范围；准备页关联该结果，失败时显示待补测项目与原因

拟合采用每姿态等权中位数；静态阈值取姿态内噪声与留一姿态误差的保守包络；独立验证以 P99 对比已配置阈值，MAX 单帧超限单独报告

动态速度摩擦不包含静摩擦状态和回程滞回；固定偏置不能消除所有姿态/历史相关残差；估计能力的下限由实际无接触残差决定

### MDK 与释放验证

先通过外力独立验证，再启动有限 K 的受限试验；配置参与轴、M/D/K、位置/速度退让和跟踪误差；参数与响应曲线使用同一草稿

明确记录施力和释放阶段，用实际位移、持续有效反馈、无触限、释放后回位与静止窗口评价；允许位置和速度误差由本次试验显式设置并随记录保存

按“开始新记录 → 施力退让 → 标记外力已释放 → 等待回位并稳定 → 结束记录 → 分析并导出记录”操作；回位目标为记录开始时的原名义目标，试验中捕获当前位置不能代替回位

退让与释放结果在准备页合并为一个任务；每个参与轴均需覆盖，允许用同一参数范围内的多份记录补齐；部分通过时显示已通过和待测关节

`observation_validated` 表示与当前配置匹配的独立外力验证通过，且已保存必要的无接触与方向确认；`response_validated` 表示有限刚度响应验收通过；受限试验只允许有限 K 的 COMPLIANT，正常 CSP 柔性、拖拽及故障拖拽检查两项许可

K=0 时持续残差会产生持续速度；调节 M/D/K 不替代模型、摩擦和反馈一致性处理

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

候选 URDF 的动力学参数或导出的重力缩放改变时，导出 Core 将撤销原模型的 `gravity_model_validated=true`；离线静态通过不继承实机故障恢复的模型验证状态，`allow_drag_recovery` 偏好保持原值

## 完整动力学辨识

### 预处理与物理拟合

对实测 q 使用四阶零相位 Butterworth 滤波，默认 5 Hz，也可选择 2/10 Hz 比较结果；无效值、阶段变化或 >25 ms 间隔切段，再将连续阶段均衡划分为不超过约 10 秒的独立块；块内重采样并中心差分得到 v/a，至少保留 3.5 秒连续数据，各块去掉 1.5 秒边缘；回归量、基线和响应统一使用 5 Hz 低通，再插值到 20 Hz 的共同时间网格，在线加速度与参考导数不参与拟合

运动方程为 `tau = Y(q,v,a) theta + b + Fc sign(v) + Fv v + residual`，各轴主要运动方程要求 `|v|>0.03 rad/s`；静态组以实际姿态的 `Y(q,0,0)` 和静态观测加入同一个训练目标；静态方程只使用零速模型，静摩擦归入观测不确定性

每 Link 的质量分布矩阵为 `J = [[0.5 tr(I_O) Id - I_O, h], [h^T, m]]`；在源模型 Cholesky 同余归一化下直接约束 `J` 正定，使用 CVXPY/Clarabel 求解；约束直接作用于完整待估块，无经验质量或质心硬边界；源模型需提供严格物理参考

全局与局部共用模型和预处理；局部固定块从系统构造开始排除出优化，结果输出完整模型；运动拟合只使用动态阶段，静态姿态通过独立的姿态组方程参与；没有独立验证会话时，按动态阶段和各关节的有效运动方程选择完整的留出块，并在报告中列出每阶段的训练与验证覆盖；导出时将连杆原点惯量转换到 COM 后写入 URDF

软正则是源质量分布同余坐标的 Frobenius 距离，按完整模型参数数量归一化，使相同系数下全局和局部的相同参数使用相同惩罚强度；系数 `{0.01,0.1,1,10,100,1000}` 及“不更新”由各训练集内部三折选择，分别选出的系数可能不同；至少需要三个训练静态姿态组与三个连续时间块；各折按运动阶段和各关节的有效方程分配完整时间块，并留出整个静态姿态组；外部验证会话和本次静态留出组不参与选择

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

