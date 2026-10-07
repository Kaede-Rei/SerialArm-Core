# SerialArm-Core 架构蓝图

SerialArm-Core 将机械臂从 Description 接入到正常控制视为一条连续生命周期，而不是一组互相独立的工具页面

Launcher 负责引导、资源管理、检查与工作流组织，Core 负责真正的控制、动力学、安全与结构化任务

GUI 与非 GUI 入口复用同一套 Profile、Core 和机器接口，GUI 不是能力唯一入口

## 1 总体分层

| 层级 | 职责 | 主要位置 |
| --- | --- | --- |
| Core | 生命周期、关节控制、安全、动力学、交互、标定任务 | `src/serial_arm/core/` |
| Adapter | 将 ROS2 输入与状态映射到 Core | `src/serial_arm/bringup/ros2_control/` |
| Profile | 组合 Core、Hardware、Description、Controllers 与 MoveIt | `src/robot_supports/profiles/` 与外部 Profile 包 |
| Robot Support | 机器人模型、硬件资源、控制配置与规划资源 | `src/robot_supports/robots/` 或外部工作区 |
| Hardware Backend | 将统一执行器语义转换为厂商命令 | `src/robot_supports/hardware/` |
| Protocol / Transport | 协议编解码、总线共享与事务仲裁 | `src/robot_supports/protocol/` 与 Core transport |
| Launcher | 生命周期引导、Profile Library、模型检查、控制工作台与进程管理 | `apps/launcher/` |

## 2 Description 与 Profile 的边界

### SerialArm Description

Description 只描述机器人模型事实

典型内容包括

- URDF 或 Xacro
- Link 与 Joint 拓扑
- Visual 与 Collision
- Mesh
- Inertial
- Joint limit
- 固定 Frame
- ros2_control Xacro 等与模型直接相关的资源

Description 不负责保存某台机器人在 SerialArm-Core 中的实际使用状态

### SerialArm Profile

Profile 描述 SerialArm-Core 如何使用这台机器人

典型内容包括

- 受控串联关节与顺序
- Base Frame 与 Tool Frame
- Hardware Backend 与执行器映射
- 总线与连接配置
- direction、ratio、zero offset
- 控制与安全参数
- park pose
- 重力、残差、摩擦与导纳配置
- Controllers 与 MoveIt 资源引用

Description 可以独立存在

Profile 必须引用一个可解析的 Description

## 3 Launcher 启动入口

Launcher 第一次打开时不直接展示 Model、Control、Terminal、ROS2 或 MoveIt

主界面只询问用户当前从哪里开始，并保留 UI Settings

```text
SerialArm Workspace

准备从哪里开始

├─ 新的机械臂
│  └─ 从 Description 包、URDF 或 Xacro 创建 Profile
│
├─ 继续配置机械臂
│  └─ 打开配置中、待验证或需要重新验证的 Profile
│
└─ 使用机械臂
   └─ 打开已经完成验证的 Profile
```

UI Settings 独立于机械臂工作流，只负责语言、主题和界面偏好

## 4 Profile Library

Profile Library 是 Launcher 对多个机械臂 Profile 的统一入口

Profile Library 管理的是引用，不复制 Profile 内容，也不把所有机器人配置重新合并到一个巨型 YAML 中

用户级集成 YAML 保存

```yaml
profiles:
  tomato_picker:
    profile_file: /path/to/tomato_picker/robot_profiles.yaml
    profile: tomato_picker
    resource_paths: /path/to/tomato_picker
```

实际机器人配置仍保存在各自 Profile 包中

Profile Library 支持

- 从 Description 创建 Profile 后直接注册
- 导入外部 Profile 包目录
- 导入外部 `robot_profiles.yaml`
- 相同 Profile 名称以独立 Library 标识并存
- 更新 Library 引用
- 从 Library 移除
- 重新导入

从 Library 移除只删除注册引用，不删除磁盘上的 Profile 包

磁盘文件删除不属于普通 Profile Library 操作

内置 Profile 与外部 Profile 在 Launcher 中采用同一选择体验

## 5 新机械臂接入向导

新的 Description 入口采用下列顺序

```text
Description 导入
    ↓
模型解析
    ↓
选择 Base Frame 与 Tool Frame
    ↓
推导串联控制链
    ↓
选择受控关节
    ↓
配置 Hardware Mapping
    ↓
生成 初始 Profile 包
    ↓
选择是否加入 Profile Library
    ↓
进入 Readiness Dashboard
```

### 5.1 Description 导入

支持 Description ZIP、Description 目录、URDF 与 Xacro 入口

导入阶段只读取模型，不连接执行器

Launcher 检查

- URDF 是否可解析
- Link / Joint 树是否连通
- Mesh 是否可解析
- Inertial 格式是否合法
- Joint limit 是否存在
- 根 Link 与叶 Link
- 固定 Frame 与末端候选

Three.js 直接显示导入模型，用户不需要先创建 Profile 才能检查模型结构

### 5.2 控制链选择

用户选择 Base Frame 与 Tool Frame

Launcher 根据 URDF Tree 推导两者之间唯一的后代链

固定 Joint 保留在模型链中但不进入 Core `joint_names`

非主串联链上的夹爪、末端机构和其他 Joint 不会被自动加入主机械臂控制链

### 5.3 Hardware Mapping

向导将受控 Joint 映射到执行器

对于当前已支持的 Hardware Backend，GUI 提供对应结构化配置

DAMIÃO 映射包含

- Bus
- Device
- Baudrate
- Joint → Actuator
- Motor ID
- Master ID
- Motor Type

硬件信息不完整时仍可生成 初始 Profile，但 Profile 保持配置中状态，不能被视为可正常控制

### 5.4 初始 Profile

Launcher 生成的新 Profile 默认采用保守、安全的初始化值

- `write_enabled` 默认关闭
- 关节 direction 与 ratio 使用中性初值
- zero offset 使用零初值
- 重力模型未验证
- 摩擦补偿不自动启用
- 导纳默认关闭
- 控制增益采用保守初值
- park pose 需要后续验证

这些值只是 bring-up 初值，不表示已经完成真机标定

## 6 Readiness Dashboard

Profile 创建或导入后首先进入 Readiness Dashboard，而不是直接进入 Control

生命周期固定为

```text
Description
    ↓
Profile Setup
    ↓
Model Check
    ↓
Hardware Bring-up
    ↓
Mapping Calibration
    ↓
Geometry Calibration
    ↓
Dynamics Calibration
    ↓
Validation
    ↓
Ready
```

### 6.1 状态

Profile 使用四种顶层状态

- 配置中
- 待验证
- 需要重新验证
- 可使用

Ready 不作为一个永久布尔值写进集成 Profile YAML

Launcher 根据当前 Profile 内容、资源检查与验证证据动态计算状态

需要人工确认的阶段保存当前 Profile 指纹

Profile、Core、Hardware 或模型文件发生变化后，旧确认会自动变为需要重新验证

### 6.2 阶段职责

| 阶段 | 目标 |
| --- | --- |
| Description | 模型资源能够正确解析 |
| Profile Setup | 控制链、Frame、Hardware Mapping 与基础配置完整 |
| Model Check | 用户确认 URDF Tree、Joint Axis、limit、Visual、Collision、Frame 与 inertial |
| Hardware Bring-up | 确认通信、在线状态、反馈与设备映射 |
| Mapping Calibration | 校准 direction、ratio、zero offset 与执行器对应关系 |
| Geometry Calibration | 检查 Joint origin、Joint axis、TCP 与其他几何 Frame，需要外部修改时生成候选 Description 后重新验证 |
| Dynamics Calibration | 校准重力、COM、residual、friction 与需要的交互参数 |
| Validation | 使用独立数据或受控真机流程验证候选模型与 Profile |
| Ready | 所有必需阶段满足后进入正常控制 |

## 7 Profile Setup 工作区

Profile Setup 用于查看和修改 SerialArm-Core 自身的机器人使用配置

当前 GUI 直接提供

- 执行器写入开关
- Bus
- Device
- Baudrate
- direction
- pos_ratio
- tor_ratio
- joint_zero_offset
- actuator_zero_offset
- Motor ID
- Motor Type

执行器写入在新建 Profile 中默认关闭

用户应先完成通信、映射、方向、零位和限位检查，再明确开启执行器写入

## 8 Model Check 与候选模型

Model Workspace 继续作为完整的只读模型检查工具

它负责

- Visual / Collision
- Link Frame
- Joint Axis
- COM
- Inertia
- Joint Preview
- URDF 字段检查
- Core 约简动力学信息

候选模型采用统一 Candidate 语义

```text
Current
    ↓ 修改与校准
Candidate
    ↓ 独立验证
Adopt / Reject
```

候选模型预览不发送执行器命令

重力模型校正只覆盖已经声明和验证的重力范围，不静默替换完整动力学

## 9 Hardware Bring-up 与正常控制边界

新机械臂第一次连接真机时应先完成 Bring-up，而不是直接执行正常控制任务

Bring-up 关注

- 通信是否建立
- 执行器是否在线
- Joint 与 Motor ID 是否对应
- 反馈方向是否正确
- 反馈单位是否合理
- 零位是否一致
- 软限位与 URDF limit 是否匹配
- 低速运动是否符合模型方向

只有 Ready Profile 才默认进入正常 Control Workspace

未完成 Profile 仍可以由专业用户打开专业工具，但 Launcher 必须持续显示其准备状态

## 10 专业工具层

生命周期是默认导航

已有专业工具继续保留

- Model
- Control Workspace
- Tuning
- Calibration
- Diagnostics
- Terminal
- ROS2
- MoveIt

专业工具不再承担告诉新用户下一步做什么的职责

Readiness Dashboard 负责决定下一步，专业工具负责完成具体技术工作

## 11 GUI 与非 GUI 能力一致性

GUI 只是结构化前端，不建立第二套机器人控制逻辑

在线控制与标定继续复用 `serial_arm_terminal --machine` 和同一个 `TerminalApp`

交互式 Terminal 保留完整的控制、标定和模型校正入口

离线重算、候选保存、候选 URDF 导出与配置恢复由命令行工具复用同一持久化与校验实现

Profile 生命周期配置同样提供 `tools/profile_workspace_cli.py`，覆盖 Description 检查、初始 Profile 创建、Profile Library 导入与移除、Readiness 查询和阶段证据记录

因此

```text
GUI
   ┐
Terminal ├─> Core / TerminalApp / Dynamics / Calibration
CLI      ┘
```

GUI 可以提供额外的 Three.js 可视化，但不得成为某项核心控制或标定能力的唯一入口

## 12 资源与安全边界

Renderer 无 Node 权限

Electron 主进程只暴露明确允许的文件选择和 IPC 方法

Python 后端负责 Profile Library、只读检查、路径解析、候选文件与持久化操作

控制循环、硬件生命周期和安全状态仍由 C++ Core 负责

Launcher 不通过 Terminal 菜单数字自动化控制机器人

Profile Library 删除操作不删除外部文件

Description 导入不打开 Hardware

候选模型预览不连接执行器

需要运动的阶段必须经过明确的硬件确认和 Core 安全检查

## 13 Tomato Picker 验收路径

`tomato-picker-desc.zip` 用于验证新的 Description 入口

```text
tomato-picker-desc.zip
    ↓
导入 Description
    ↓
base_link → tool0
    ↓
自动得到 joint1 → joint6 主控制链
    ↓
配置 DAMIÃO Hardware Mapping
    ↓
生成 tomato_picker Profile 包
    ↓
加入 Profile Library
    ↓
Model Check
    ↓
Hardware Bring-up
    ↓
Mapping / Geometry / Dynamics Calibration
    ↓
Candidate Validation
    ↓
Ready
    ↓
正常控制
```

已经存在的 `tomato_picker` Profile 包用于验证第二入口

完整验证后的 `tomato_picker` Profile 用于验证第三入口

这三个真实案例分别覆盖从 Description 开始、继续完善 Profile、直接使用 Ready Profile 的完整产品路径
