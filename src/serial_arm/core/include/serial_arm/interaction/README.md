# Interaction

Interaction 根据有效反馈估计外力，并为模型导纳计算关节虚拟参考

| 部分 | 职责 |
| --- | --- |
| `estimators/` | FULL_ID 或 MOMENTUM 残差、偏置、摩擦模型与死区 |
| `controllers/` | M/D/K 虚拟模型、退让限幅与参数过渡 |
| `runtime/` | 观测与控制状态交换 |

原生阻抗可只使用外力观测；模型导纳在名义参考上计算位移、速度和加速度，Robot 使用伺服跟踪增益执行，再经过总力矩预算和 Safety 检查

`InteractionState` 交换外力估计、导纳位置与速度修正及有效性；`RobotCycleOutput` 额外记录名义、虚拟、最终参考、测量时序、实际增益与限制标志

观测器只在有效新测量上推进，FULL_ID 还要求有效的实测加速度；残差偏置用于观测，不加入执行器前馈

`JointAdmittanceController` 支持 M>0、D≥0、K≥0，使用后向 Euler 积分；限制后的接受状态回写控制器，运行中更新保留状态并平滑过渡

M/D/K 决定虚拟响应，执行器带宽、延迟、模型误差和限制共同影响实际响应；接触行为按具体任务验证

任务方向、插入约束和操作流程由应用层定义；完整链路见 [开发与架构](../../../../../../docs/development.md) 与 [API](../../../../../../docs/reference/api.md)
