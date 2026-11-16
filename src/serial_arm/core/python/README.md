# SerialArm Python Binding

Python 接口复用 C++ Core；`RobotSession` 管理硬件生命周期与原生控制线程，`Dynamics` 提供独立模型计算，模型和 Backend 由 Profile 选择

## 安装

在仓库根目录执行，使用 Python 3.10+

```bash
./install.sh --preset python --robot dm_arm --yes
source .install/setup.bash
python -c 'import serial_arm; print("serial_arm import OK")'
```

## 先连接观察，再单独使能

```python
import serial_arm

arm = serial_arm.RobotSession.from_profile(
    "your_robot",
    profile_file="/absolute/path/to/robot_profiles.yaml",
    resource_paths=["/absolute/path/to/resources"],
)
arm.connect()                 # 读取实际能力，不使能
snapshot = arm.observe()      # INACTIVE 硬件快照，不启动控制线程
print(snapshot.cycle.joint_state.pos)
print(snapshot.cycle.actuator_state.enabled)
```

`observe()` 保留越界、在线与驱动错误供首次接入检查；连接观察不证明机械臂可运动；核对映射、边界和起动参数后，由调用者单独执行 `start()`，使能前 Core 检查当前位首命令及力矩预算

`runtime.write_enabled=true` 使用真实 Backend；false 使用内部 Mock，但 Python 会话仍需加载所选 Backend 的能力信息；构造和读取配置不启动控制线程，上下文管理器会自动 `start()`，首次只读检查不要使用上下文管理器

## 执行路径与模式

| 选择 | 行为 |
| --- | --- |
| `ControlPath.MIT` | 发送 q、dq、Kp、Kd 和前馈力矩；起动保持需要逐轴正的刚性 PD |
| `ControlPath.CSP` | 发送位置和速度上限，位置伺服由驱动器完成，不使用 MIT 增益 |
| `RIGID` / `SERVO_TRACKING` | 刚性位置目标，伺服跟踪暂时复用刚性行为 |
| `COMPLIANT` | MIT 使用柔性 PD，CSP 使用有限刚度 MDK |
| `DRAG` | MIT 使用零 Kp、阻尼与重力补偿，CSP 使用零刚度导纳 |

`set_control_path()` 和 `set_model_feedforward_mode()` 在线程停止且 INACTIVE 时调用；达妙 CSP 对应普通 POS_VEL 的 `0x100 + ID` 命令，`position_speed_limit` 是速度上限

CSP 柔性与拖拽需要观测和响应验证；首次有限 K 测试用 `set_admittance_validation(True)`，要求观测已验证且参与轴 K>0；测试结束调用 false 回到当前位置 RIGID，不自动标记响应验收通过；参数变动会清除受影响的验证标志

## 目标、快照与停止

`move_to(pos, speed_scale=0.2)` 提交绝对目标，速度比例在 `(0,1]`，返回运动 ID；配置与 API 使用 SI，关节顺序始终对应 `config.joint_names`

读取 `snapshot` 后检查 `last_error`、`valid` 与相应测量标志；`snapshot.motion` 区分参考结束、稳定和实际到位，`position_error` 保留真实偏差，线程运行不等于已经到位

`hold_current()` 取消目标并捕获当前实测位置，保留当前交互模式；需要刚性保持时先选择 `JointImpedanceMode.RIGID`；离开 DRAG 时，Core 捕获当前位置并丢弃旧目标

`stop()` 与上下文退出停止线程并失能，不执行停放轨迹；随附 Python Terminal 的停放采用受限 RIGID 轨迹，满足严格就位与稳定判据后调用 `stop()`，超时保持使能并报告失败

Python Terminal 的显式 `park` 不受桌面自动退出开关影响，退出前要求已失能；停放目标和位置／速度容差使用逐轴向量；未设置停放目标时仍可连接观察及进行基础控制

MIT 拖拽和故障拖拽要求当前模型与负载的重力补偿已验证；CSP 使用观测与响应质量标志，不要求 MIT 重力补偿标志；故障请求由同一 C++ 控制线程执行，清故障后不自动恢复旧目标；会话参数应用仅影响当前进程，持久化保存使用独立文件流程

完整方法、类型与错误见 [API](../../../../docs/reference/api.md)，构建见 [安装](../../../../docs/install.md)，首次接入与调试见 [使用指南](../../../../docs/guide.md)
