# GUI

## 打开

```bash
./install.sh
./launch.sh
```

已有 Core 时补装 GUI 使用 `./install.sh --gui-only --yes`

GUI 需要 Node.js 20+、npm 和 Linux 桌面环境，启动脚本自动加载安装环境

## 选择机器人

1. 选择内置 Profile 或外部 `robot_profiles.yaml`
2. 选择机器人名称
3. 检查串口、波特率、Bus 与资源搜索路径
4. 点击检查配置后，可进入模型工作台或选择运行方式

连接参数留空使用 Profile 默认值，填写后只覆盖本次运行

多个资源目录用冒号分隔，可使用下游工作区或安装前缀

GUI 不修改 YAML，外部模型、硬件后端、Controllers 与 MoveIt 资源仍由下游项目提供

## 模型工作台

Model 是独立工作台入口，用于查看机器人结构、坐标系、关节运动与动力学参数

Model 使用 Core 实际加载的 URDF 与 Dynamics 结果，支持 GLB、STL、URDF 基本几何、Visual 与 Collision

Link Frame、Joint Axis、质心、惯性与 Labels 可独立切换，左侧模型树还可单独显示或隐藏每个 Link 与 Joint

选中 Link 或 Joint 后会高亮对应对象并降低其他模型的视觉权重，便于检查原点、运动轴、质心与惯性参数

关节滑块拖动期间持续调用常驻的原生 Model Probe 更新 Core 姿态，连续拖动时合并过期请求而不是等待松开后才刷新

原始 Link 惯性与 Core 约简后的有效惯性分开展示，物理参数无效时对应惯性图形保持禁用


## 控制工作台

Control Workspace 使用 `serial_arm_terminal --machine` 的结构化会话，并与 C++ Terminal 共用 `TerminalApp`、控制线程、Robot、Dynamics 与安全规则

工作台包含 Control、Tuning、Calibration 与 Diagnostics 四个任务页，Model 继续作为独立顶层模型工作台

Control 提供生命周期、停放、立即失能、FAULT 恢复、五种关节阻抗模式、模型前馈、绝对与相对关节目标、速度比例与当前位置保持

Tuning 提供导纳 M、D、K、位置与速度限幅、MOMENTUM gain、Observer 模式以及 ωn、Dcrit、ζ 和单位力稳态退让指标

运行时应用参数只影响当前会话，写回 Core YAML 前必须先预览差异并显式确认，保存会检查源文件指纹、建立备份并原子替换目标文件

Calibration 提供静态残差标定、独立静态验证、双向摩擦示教与回放，以及按相同顺序执行的一次性流程

标定任务状态由 C++ 后台维护，示教记录和摩擦回放可取消，冲突控制动作由后台拒绝

Diagnostics 显示 Joint、Actuator、动力学向量、质量矩阵、Frame 位姿、Observer residual 与限长实时曲线

工作台模型只显示真实遥测姿态，反馈过期时停止继续更新模型

会话遥测可导出到 `.install/exports/`，记录 Profile、Core 来源、时间戳、关节位置、速度、力矩与运行时诊断数据

## 运行

| 模式 | 入口 | 作用 |
| --- | --- | --- |
| Terminal | `serial_arm_terminal` | 保留 C++ 菜单与键盘交互 |
| Hardware | `hardware.launch.py` | 启动 ros2_control 与 Controllers |
| MoveIt | `moveit.launch.py` | 启动硬件、规划与 RViz |

缺少组件或资源时启动按钮不可用，可到诊断页查看原因

Terminal、Hardware 与 MoveIt 启动前需要确认 Profile、设备和 `write_enabled`

确认不改变 YAML 的执行器写入开关，配置改变后需重新检查

Terminal 使用 PTY，点击终端后输入，运行中不能修改连接参数或切换会话

拖选文本后用 Ctrl+Shift+C 复制，Ctrl+Shift+V 粘贴，Ctrl+Shift+A 全选

右键菜单提供复制所选、复制全部输出、粘贴和全选，工具栏也有复制全部与粘贴按钮

复制使用终端文本，去除 ANSI 控制码，范围为当前终端缓冲，最多保留约 100 万个字符

只有活动的 Terminal 会话允许粘贴，Ctrl+C 保留原程序的中断语义

停止先发送 SIGINT，等待退出后才能切换模式，强制结束会中断清理流程

关闭窗口时如有活动会话，会询问停止或强制结束

同一用户的 Launcher 按实际设备路径互斥，直接运行底层程序不受此锁约束

## 结构化工作台接口

`serial_arm_terminal --machine` 提供 JSON 行协议，和交互式 Terminal 共用同一个 `TerminalApp`、控制线程、Robot、Dynamics 与安全流程

该接口由控制工作台直接使用，不通过自动输入菜单数字驱动 Terminal

接口覆盖生命周期、FAULT 恢复、阻抗模式、模型前馈、重力比例、绝对与相对关节目标、当前位置保持、导纳调参、标定任务、状态快照与安全退出

Launcher 的结构化会话与 PTY Terminal 共用设备锁，同一实际设备不能由两种入口同时占用

## 诊断与偏好

诊断只读取文件与权限，不实例化 Hardware、不打开串口

配置语义与硬件能力仍由运行入口校验，资源检查通过不代表真机验收通过

```bash
./launch.sh --doctor
./launch.sh --doctor \
  --profile tomato_picker \
  --profile-file /absolute/path/robot_profiles.yaml \
  --resource-paths /absolute/path/workspace
```

设置支持中英文与系统、深色、浅色主题，切换不会重启进程

窗口、Profile 与连接偏好保存在 `~/.config/serial-arm/launcher.json`

终端可滚动查看和清空显示，不做长期日志归档，安装日志位于 `log/unified/`

## 界面

截图来自离线界面测试，不表示已经连接真机

![深色界面](images/launcher-dark.png)

![浅色界面](images/launcher-light.png)

验证命令见 [开发说明](development.md)

## 常见启动报错

Model 显示原生只读入口不可用时，先确认当前安装包含 Terminal 组件并重新运行对应安装预设

`Missing gripper_left` 表示 MoveIt 缺少夹爪关节状态，不等同于 CAN 或串口断线

内置白色模型包含夹爪关节，但随附 Hardware 与 ros2_control 只接入 joint1 到 joint6

需要接入真实夹爪驱动与状态发布，不能用固定零位冒充硬件反馈

运行 Hardware 或 MoveIt 后可在另一终端检查

```bash
./.install/run ros2 topic echo /joint_states --once
```

如果只有 joint1 到 joint6，没有 gripper_left，先补齐夹爪状态接口

`/recognize_objects not available` 是物体识别 action 服务未启动，不是末端电机通信错误
