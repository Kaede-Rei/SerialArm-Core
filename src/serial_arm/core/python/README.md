# SerialArm Python Binding

Python 接口复用工作台的 C++ Core，提供串联机械臂模型计算、配置读取及由 C++ 工作线程执行的控制会话；机械臂模型与硬件驱动通过 Profile 选择

在仓库根目录安装并加载环境

```bash
./install.sh --preset python --yes
source .install/setup.bash
python -c 'import serial_arm; print("serial_arm import OK")'
```

`RobotSession` 管理周期、快照和移动目标；`ReferenceBehavior` 选择原生阻抗或模型导纳，`JointGainProfile` 选择刚性、柔性、拖拽或伺服跟踪用途

使用与桌面工作台相同的 Profile 配置

```python
import serial_arm

session = serial_arm.RobotSession.from_profile(
    "your_robot",
    profile_file="/absolute/path/to/robot_profiles.yaml",
    resource_paths=["/absolute/path/to/resources"],
)
print(session.config.joint_names)
```

该入口解析模型、驱动与配置，只构造会话；`start()` 或进入上下文管理器时才激活并运行控制线程

Python 会话配置需要可加载的 Hardware Backend 来读取执行器能力，关闭写入时运行 Bus 使用 mock；只需要模型且没有硬件驱动时，可使用 GUI 或 C++ Terminal 的离线会话

控制方式在失能时切换；模型导纳使用伺服跟踪增益；`move_to()` 从保持转入对应跟踪，`hold_current()` 捕获当前实测位置

Python 交互终端的拖拽需显式设置：停放失能 → 切换原生阻抗 → 选择 GRAVITY 和有效重力比例 → 使能 → 选择拖拽；结束后按需要停放失能，再切回原控制方式

GUI 与 C++ Terminal 的普通示教会临时切换原生阻抗和重力补偿，并在结束后恢复原上下文；Python 交互终端由用户完成这些切换

会话参数应用只影响进程，保存配置使用单独的文件流程

见 [安装](../../../../docs/install.md)、[机械臂接入](../../../../docs/tutorial.md) 和 [API 参考](../../../../docs/reference/api.md)
