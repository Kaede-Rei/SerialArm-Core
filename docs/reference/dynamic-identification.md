# 完整动力学辨识与实验流程

在“动力学标定 → 高级动力学辨识”选择完整惯性参数，按“数据检查 → 辨识设置 → 结果与推荐 → 导出”操作；局部范围中勾选的可信连杆从求解开始保持固定

## 采集与操作

正式采集沿已经检查、重定时的示教路径，执行逆向静态、正向静态、逆向半速、正向半速、逆向计划速度、正向计划速度；运动部分约为单程的 8 倍，另加双向静态停留

采集结束后机器人保持位置、恢复会话参数并保存任务；断开运行会话后加载记录，执行离线数据检查与辨识

在线静态预览用于快速诊断，最终模型来自保存后的离线分析；`prior_retained` 表示保留输入模型，本轮未支持参数更新

采集中显示双向到位位置差与组数，超过 0.01 rad 时提示重复性问题并保留原始观测；这是位置差提示，采样有效性和反馈超时按运行条件另行判断

新任务在 `source/` 保存 `model.urdf`、`core.yaml`、`hardware.yaml`、`hardware-effective.yaml`，使用重力校正时另存 `gravity-correction.yaml`；metadata 记录各文件指纹、`hardware_overrides`、程序与驱动文件指纹以及执行器量程来源，便于核对原始配置与实际采集条件

## 静态数据契约

`static_observations.py` 是静态重力与完整 IDIM 共用的数据读取器

- 新记录只读 `accepted_static.csv`；每方向至少 30 帧，取末端最多 60 帧的中位数；检查有效值、反馈年龄 ≤30 ms、速度 <0.04 rad/s、方向和训练/验证标签
- 保留各方向的实际 q，不先平均关节角；先计算各方向回归矩阵和力矩，再按姿态组等权平均
- 同一姿态的两个方向属于同一个训练或验证组，不能把一方向当训练、另一方向当验证
- 双向半差、组内 MAD 描述迟滞不确定性；两方向实际位置分别计算，单向组保留并赋更大的不确定性
- 双向/单向评分尺度下限为 0.02/0.05 Nm，属于工程评分设置；报告另存方向、实际位置差和不确定性

## 联合数学模型与物理拟合

对实测 q 使用四阶零相位 Butterworth 滤波，默认 5 Hz，2/10 Hz 用于敏感性比较；按独立连续约 10 秒块重采样并中心差分得到 v/a；无效值、阶段变化或 >25 ms 间隔切段，各块去掉 1.5 秒边缘；回归量与响应同样低通并降到约 20 Hz，在线加速度与参考导数不参与拟合

运动方程为 `tau = Y(q,v,a) theta + b + Fc sign(v) + Fv v + residual`，各轴主要运动方程要求 `|v|>0.03`；静态组以实际姿态的 `Y(q,0,0)` 和静态观测加入同一个训练目标；静态方程只使用零速模型，静摩擦归入观测不确定性

每 Link 的质量分布矩阵为 `J = [[0.5 tr(I_O) Id - I_O, h], [h^T, m]]`；在源模型 Cholesky 同余归一化下直接约束 `J` 正定，使用 CVXPY/Clarabel 求解；约束直接作用于完整待估块，无经验质量或质心硬边界；源模型需提供严格物理参考

全局与局部共用模型和预处理；局部固定块从系统构造开始排除出优化，结果输出完整模型；导出时将连杆原点惯量转换到 COM 后写入 URDF

软正则是源质量分布同余坐标的 Frobenius 距离；系数 `{0.01,0.1,1,10,100,1000}` 及“不更新”由训练集内部三折选择；每折同时留出完整时间块和整个静态姿态组，外部验证会话和本次静态留出组不参与选择

WLS 使用训练源模型残差的归一化尺度，OLS 使用统一轴尺度；IRLS 先迭代 Huber 诊断权重，再进行物理约束重拟合；WLS 的尺度来自当前数据，不是独立标定的传感器协方差

基线和候选的偏置/摩擦都仅在同一训练静态+运动系统估计，再冻结用于验证；纯重力评分不添加拟合偏置；动态预测改善不等于重力补偿改善

方法参考：[IDIM 基准](https://doi.org/10.3390/app11094303)、[物理惯量 LMI](https://arxiv.org/abs/1701.04395)、[参数可观察性](https://arxiv.org/abs/1711.03896)

## 推荐模型与拟合候选

`recommendation=keep_source_prior` 表示推荐保留输入模型；最佳收敛的物理拟合仍作为审核对照保留，推荐结果与拟合结果分开展示

报告区分别展示质量、COM、完整惯量差值、运动误差、纯重力误差、物理性及缺少的输入条件；3D 使用完整候选的一阶矩与质量作只读质心对照；几何绑定不同或不明确时不显示不可信的对照

未采样夹爪关节应全程固定并声明实际位置；未知时按中立位诊断并保留假设；补充装机说明独立标记为操作者声明，与原始 metadata 和测量分别保存，且须与已有条件一致

## 三种导出

1. **诊断报告**：始终可导出，含完整报告、SHA 收据、原始/拟合/无约束对照参数；不含 URDF
2. **待审核 URDF**：模型必须物理有效，校验完整惯性参数回读和 FK；可以尚缺独立会话或条件声明，但附明确缺项，不作为硬件应用或改善认证
3. **通过离线检查的模型**：另需独立会话、相同模型/装机/映射/重力/量程、输入质量通过、未采样关节位置已知、运动与纯重力改善且训练 CV 不推荐保留原模型；仍不等于真机验收

导出写入新目录，保留当前 Core、观测器参数和运行模型；URDF 要求物理有效，报告与源数据哈希匹配

```bash
# 通过安装器的环境运行，避免系统 SciPy/用户 NumPy 路径冲突
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py audit-dynamics --directory /path/to/task

./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py identify-dynamics \
  --directory /path/to/train --validation-directory /path/to/independent \
  --mode global --estimator WLS --cutoff-hz 5

# 使用本次已确认可信的连杆名称
./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py identify-dynamics \
  --directory /path/to/train --mode local --locked-link Link1 --locked-link Link2

# 若确实已知且全程固定，可追加多个 --fixed-joint actual_name=value
# 缺失装机说明可追加 --load-description '实际已知条件'；单列为操作者声明

./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-dynamics \
  --report /path/to/idim-report.json --destination /new/review-dir --review-only

./.install/run .install/gui-venv/bin/python tools/model_calibration_cli.py export-dynamics \
  --report /path/to/idim-report.json --destination /new/diagnostics-dir --report-only
```

## 实现检查

`apps/launcher/tests/test_dynamic_identification.py` 含独立解析摆臂真值、物理拟合、局部锁定、静态组划分、留出数据不进入选择、报告/源篡改拒绝、三种导出与 FK 回读测试

`apps/launcher/scripts/test-identification-ui.cjs` 使用离线模型和隔离请求桩检查 DOM、Three.js、输入保留及硬件请求边界；运行时设置 `SERIALARM_UI_FIXTURE` 与 `SERIALARM_UI_OUTPUT`

## 实验案例：2026-10-29 采集复核

任务 `model-calibration-1793103375584`，113,364 原始帧；12 静态姿态组，9 组训练/3 组留出；仅同会话证据

| 方法 | 运动 RMS（源→候选）Nm | 静态重力 RMS（源→候选）Nm |
|---|---|---|
| global | 0.21005 → 0.20646 | 0.01989 → 0.02375 |
| local | 0.21005 → 0.21015 | 0.01989 → 0.01939 |

完整全局 CV 推荐保留原模型；局部静态变化低于 0.001 Nm 工程判据；两者物理可导出供审核，但均不建议作为补偿改善模型上机

静态直接重力全局/局部留出 RMS 为 0.01149/0.01768 Nm；分离偏置全局保留源模型，局部为 0.01987 Nm；这些较小残差不能代替历史效果复现

用户认可的历史 Link6：质量 0.774252 kg，COM (0.950, −7.247, 28.037) mm；源模型 COM (−0.486, 8.353, 37.806) mm；新完整全局 (−0.676, 6.925, 37.915) mm、局部 (−0.525, 8.340, 37.805) mm；仍未复现历史分布

整份历史 URDF 在本批静态留出反馈上的 RMS 约 0.06696 Nm，源模型约 0.01989 Nm，与用户实测排序相反；需要用同工况小规模静态和已知增量载荷实验核对反馈力矩/装机/迟滞，之后再采独立会话；不按历史参数硬编码求解结果

### 该案例的后续验证

该批离线误差与实机排序不一致，后续验证按以下顺序进行

1. 保留已知可用 URDF 作对照，固定控制参数、装机、夹爪和载荷
2. 使用已知质量及安装位置做增量载荷对照，在相同姿态重复记录 q/tau；理论增量为 `Jv(q)^T * delta_m * g`，检查反馈力矩的符号、量级和重复性；PMAX/VMAX/TMAX 只说明协议量程
3. 核对双向实际到位差、迟滞和夹爪固定位置，再采独立会话
4. 对比原模型与候选的纯重力保持、漂移和运动预测，保留退化姿态与失败结果
