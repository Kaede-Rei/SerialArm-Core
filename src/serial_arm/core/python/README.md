# SerialArm Python Binding

Python 接口复用工作台的 C++ Core，提供串联机械臂模型计算、配置读取及由 C++ 工作线程执行的控制会话；机械臂模型与硬件驱动通过 Profile 选择

## 安装

在仓库根目录安装并加载环境；需要 Python 3.10+，原生依赖由统一安装器准备

```bash
./install.sh --preset python --yes
source .install/setup.bash
python -c 'import serial_arm; print("serial_arm import OK")'
```

## 从 Profile 创建会话

`RobotSession` 使用与桌面工作台相同的 Profile，管理控制周期、快照和移动目标；以下示例只构造会话，不启动控制线程

```python
import serial_arm

session = serial_arm.RobotSession.from_profile(
    "your_robot",
    profile_file="/absolute/path/to/robot_profiles.yaml",
    resource_paths=["/absolute/path/to/resources"],
)
print(session.config.joint_names)
```

将示例中的名称与路径换成自己的 Profile；`start()` 或进入上下文管理器时会激活所选后端并运行控制线程，`runtime.write_enabled=true` 时连接真实硬件

Python 会话需要可加载的 Hardware Backend 来读取执行器能力；`runtime.write_enabled=false` 时使用内部 mock 运行闭环；只需要模型且没有硬件驱动时，使用 GUI 或 C++ Terminal 的离线入口

## 控制方式与增益用途

`ReferenceBehavior` 选择原生阻抗或模型导纳，需在 `INACTIVE` 状态设置；`JointGainProfile` 选择刚性、柔性、拖拽或伺服跟踪用途，模型导纳使用伺服跟踪增益

`move_to()` 接收各关节的绝对目标，返回运动 ID；通过 `snapshot.motion` 读取到位或中断状态；该方法从保持转入对应跟踪，拖拽用途需先退出才能提交目标

`hold_current()` 取消目标，并捕获当前实测位置作为刚性保持目标；`stop()` 停止线程并失能，不执行 GUI 的停放运动，停止前应先安排好机械臂停放或支撑

仓库随附的 Python 交互终端另提供停放流程，先执行并等待配置的停放运动，再调用 `stop()`

Python 拖拽需显式配置：在失能状态选择原生阻抗、`ModelFeedforwardMode.GRAVITY` 和有效重力比例，启动会话后选择 `JointGainProfile.DRAG`；结束后由调用者停止会话并恢复所需控制方式

GUI 与 C++ Terminal 的普通示教会临时切换原生阻抗和重力补偿，并在结束后恢复原上下文；Python Session 与 Python 交互终端由调用者完成这些切换

会话参数应用只影响进程，保存配置使用单独的文件流程

见 [安装](../../../../docs/install.md)、[使用指南](../../../../docs/guide.md) 和 [API 参考](../../../../docs/reference/api.md)
