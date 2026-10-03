# 架构

SerialArm-Core 将通用控制与具体机器人资源分开，Native、Python 与 ROS2 共用 Core 能力

## 职责

| 模块 | 职责 | 路径 |
| --- | --- | --- |
| Core | 生命周期、关节控制、安全、动力学与交互能力 | `src/serial_arm/core/` |
| Adapter | 将 ROS2 输入与状态映射到 Core | `src/serial_arm/bringup/ros2_control/` |
| Profile | 组合机器人模型、Core、Hardware、Controllers 与 MoveIt | `src/robot_supports/profiles/` |
| Robot Support | 具体机器人参数、标定、URDF 与规划资源 | `src/robot_supports/robots/` |
| Hardware Backend | 将统一执行器语义转换为厂商命令 | `src/robot_supports/hardware/` |
| Protocol / Transport | 协议编解码、总线共享与事务仲裁 | `src/robot_supports/protocol/` 与 Core transport |
| Launcher | Profile 选择、诊断、确认与运行进程管理 | `apps/launcher/` |

## 控制边界

上层提供关节参考，Core 计算控制命令、交互修正与模型前馈，经安全约束后交由 Backend 执行

关节与执行器映射负责方向、比例和零位转换，算法参数采用关节侧语义

Dynamics 从 URDF 建模，提供 FK、Jacobian、重力、质量矩阵与逆动力学

关节阻抗决定参考附近的刚度与阻尼，关节导纳将估计外力转换为参考偏移

轨迹规划、采摘策略与任务约束属于上层应用，具体机器人标定属于 Robot Support

## 接入选择

- 新机械臂实例优先增加或维护 Profile、模型与参数
- 新执行器语义增加 Hardware Backend
- 新通信协议增加 Protocol，复用已有 Transport
- 新上层框架通过 Adapter 调用 Core
- 通用控制能力在 Core 中维护一份实现

下游整机项目维护自己的 Profile，无需注册到 Core 内置列表

GUI 的 Electron 主进程限定 IPC 方法，Python 后端用固定 argv 启动已有入口

Renderer 无 Node 权限，连接覆盖仅作用于运行进程，Launcher 不替代 Core 控制循环
