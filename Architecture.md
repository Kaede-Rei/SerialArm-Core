# SerialArm-Core 架构设计

## 1. 文档目的

本文档定义 SerialArm-Core 的长期架构定位、职责边界、核心抽象和扩展规则

目标是保证项目在持续接入新机械臂、新执行器、新通信方式、新控制能力和新上层框架后，仍然保持轻量、统一、可移植和可测试

本文档只描述 SerialArm-Core 的长期架构，不针对具体机器人、应用任务或研究课题

---

## 2. 项目定位

> **SerialArm-Core 是一个轻量级、模型驱动的串联机械臂通用执行与控制内核，通过统一的状态、参考和命令语义，解耦机械臂硬件、控制能力与上层机器人框架**

上层负责：

```text
机器人为什么运动
机器人运动到哪里
任务如何组织
```

SerialArm-Core 负责：

```text
在给定当前状态和运动参考后
机械臂如何以统一、安全、可建模、可扩展的方式执行
```

核心关系：

```text
Measured State + Reference
            ↓
      SerialArm-Core
            ↓
Command + Runtime Feedback
```

---

## 3. 核心问题

机器人项目通常同时受到以下因素影响：

```text
机械臂型号
执行器型号
通信协议
硬件拓扑
机器人模型
控制算法
中间件
仿真平台
上层应用
```

如果这些部分直接互相依赖，不同机器人和不同上层框架之间会产生大量重复的硬件接口、状态定义、单位转换和控制逻辑

SerialArm-Core 希望通过稳定的中间语义，将 N × M 的集成关系尽量收敛为 N + M：

```text
Hardware / Robot Side
        │
        │ State / Command
        ▼
┌───────────────────────────┐
│      SerialArm-Core       │
└───────────────────────────┘
        ▲
        │ Reference / Feedback
        │
Application / Framework Side
```

因此项目最重要的目标不是功能数量，而是降低同一控制能力在不同机器人和不同软件环境之间的适配成本

---

## 4. 总体架构

### 4.1. 窄腰接口

SerialArm-Core 应保持一个稳定且尽量小的中间接口：

```text
           Upper Framework / Application
      C++ / Python / ROS 2 / LeRobot / Isaac
                        │
                     Reference
                        ▼
┌────────────────────────────────────────────┐
│              SerialArm-Core               │
│                                            │
│  Unified Semantics                         │
│  Robot Model / Dynamics                    │
│  Runtime                                   │
│  Control Capability                        │
│  Safety                                    │
└────────────────────────────────────────────┘
                        │
                      Command
                        ▼
              Hardware Abstraction
                        │
               Transport Resource
                        │
                   Real Robot
```

所有上层框架最终转换为 Core 能理解的 Reference

所有下层硬件最终转换为 Core 能理解的 State 和 Command

Core 不应因为任意一端发生变化而改变自身控制语义

### 4.2. Runtime 数据流

控制周期统一遵循：

```text
Read Hardware
      ↓
Actuator State
      ↓
Joint / Actuator Mapping
      ↓
Joint State
      ↓
State Safety Check
      ↓
Controller / Nominal Command
      ↓
Model Feedforward / Interaction Capability
      ↓
Corrected Joint Command
      ↓
Command Safety
      ↓
Joint / Actuator Mapping
      ↓
Write Hardware
```

控制能力建立在统一状态和模型之上，不直接依赖厂商 SDK 或外部框架

Interaction / Admittance 作为可选 Capability 插入 nominal command 与最终 command Safety 之间：

```text
Joint State + Dynamics + Measured Torque
                 ↓
      Interaction Observer
                 ↓
           Admittance
                 ↓
       delta_q / delta_q_dot
                 ↓
       Corrected Joint Command
                 ↓
              Safety
```

因此高级交互能力可以修改发送前的 Joint command，但不能绕过 Safety

---

## 5. 设计目标

### 5.1. 高可移植性

同一套控制能力应尽量做到：

```text
换机械臂
→ 不改算法主体

换执行器 / 驱动
→ 不改算法主体

换上层框架
→ 不改算法主体

真机切换仿真
→ 不改算法主体
```

允许变化：

```text
URDF
Robot Profile
Hardware Backend
Adapter
控制参数
```

### 5.2. 模型驱动

URDF 是串联机械臂结构和刚体模型的主要来源

SerialArm-Core 通过成熟模型库获得：

```text
Forward Kinematics
Jacobian
Mass Matrix
Gravity
Coriolis / Nonlinear Effects
Inverse Dynamics
Forward Dynamics
```

Core 不重新实现成熟刚体动力学算法

Core 的职责是把模型能力稳定接入 Runtime 和 Control Capability

### 5.3. 中间件无关

Core C++ 控制链不依赖：

```text
ROS 2 message
MoveIt message
ROS action
LeRobot data type
Isaac data type
```

所有外部框架通过 Adapter 与 Core 交互

### 5.4. 硬件无关

Hardware Backend 负责完成：

```text
Vendor Protocol / Vendor Unit
             ↓
      Actuator Semantics
             ↓
     Joint / Actuator Mapper
             ↓
        Joint Semantics
```

厂商协议字段、原始单位和特殊控制语义不能穿透 Backend 边界

### 5.5. 渐进复杂度

基础控制不应因为高级能力存在而变复杂

基础用户可以只使用：

```text
Robot
State
Reference / Command
```

高级用户再按需启用：

```text
Dynamics
Model Feedforward
Impedance
Interaction Observer
Admittance
Constraint
...
```

高级 Capability 必须可选

关闭后不应隐式改变基础控制行为

### 5.6. 可测试性

通用 Capability 应能够脱离具体上层框架独立测试

优先支持：

```text
Native C++
Terminal
Offline Replay
```

---

## 6. 核心抽象

SerialArm-Core 长期围绕以下概念组织：

```text
Robot
State
Reference
Command
Model
Dynamics
Runtime
Capability
Safety
Backend
Transport
Adapter
Profile
```

### State

描述机器人真实反馈状态

### Reference

描述当前控制周期希望机器人跟踪的目标状态

### Command

描述最终发送给 Hardware Backend 的统一控制命令

### Model / Dynamics

描述机器人结构、运动学和动力学能力

### Runtime Feedback

描述 Core 执行产生的通用控制状态和反馈

Runtime Feedback 不包含具体应用任务语义

---

## 7. 模块职责边界

### 7.1. Core Semantics

负责：

- 统一跨机器人和跨 Backend 的状态语义
- 统一关节侧和执行器侧单位
- 统一 Reference / Command / Feedback 的基础语义

原则：

> **先统一语义，再增加接口数量**

### 7.2. Model / Dynamics

负责：

```text
URDF
  ↓
Robot Model
  ↓
Kinematics / Dynamics State
```

不负责：

```text
任务规划
环境感知
应用目标生成
```

### 7.3. Runtime

Runtime 是 SerialArm-Core 的执行核心

负责：

```text
硬件状态读取
状态更新
模型更新
Capability 执行
控制计算
Safety
命令输出
生命周期与故障处理
```

Runtime 不负责：

```text
Task Executor
Behavior Tree
Perception Pipeline
Motion Planner
Application Workflow
```

### 7.4. Control Capability

Capability 是建立在统一 State、Reference 和 Model 上的可复用控制或估计能力

一个功能适合进入 Core Capability，应尽量满足：

1. 输入主要来自 State / Reference / Model / Dynamics
2. 输出主要作用于 Command 或 Runtime Feedback
3. 不依赖特定机器人厂商
4. 不依赖具体中间件
5. 不依赖具体应用任务
6. 能在 Native Runtime 中独立测试
7. 能复用到其他串联机械臂

典型 Capability：

```text
Gravity Compensation
Model Feedforward
Joint Impedance
External Interaction Observer
Collision Detection
Admittance
Task-space / Joint-space Constraint
```

Capability 可以增加，但不应为每个 Capability 创建新的 Runtime 或新的数据体系

当前关节空间导纳遵循同一边界：

```text
observer
  FULL_ID / MOMENTUM

calibration
  torque bias / threshold / friction residual

feel
  comfortable torque / follow speed / response / return / limits
```

持久化配置使用 `observer / calibration / feel` 语义，内部导纳 M / D / K 由 Core 派生；导纳只修正 nominal command 的位置和速度，并在 Safety 剩余空间内限制修正幅度；最终命令仍由 Safety 统一裁决

`COMPLIANT_DRAG` 属于阻抗控制器的直接拖拽模式，使用当前实测位置持续重建 reference，并显式旁路导纳修正；两者属于不同交互机制，不应混用同一套参数语义

### 7.5. Safety

Safety 是最终命令发送到 Hardware 前的统一约束层：

```text
Reference
   ↓
Capability / Controller
   ↓
Command
   ↓
Safety
   ↓
Hardware
```

任何 Capability 和 Adapter 都不能绕过 Safety

Safety 负责执行安全，不负责判断具体应用任务是否成功

### 7.6. Hardware Backend

Hardware Backend 负责将具体执行器和厂商协议转换为 Core 的统一硬件语义

负责：

```text
Vendor Protocol
Vendor Unit Conversion
Enable / Disable
Read / Write
Hardware Error
Torque / Current Semantics Conversion
```

不负责：

```text
Dynamics
Control Capability
Task
Planning
Application-specific Tool Logic
```

### 7.7. Transport / Shared Resource

Transport 负责物理通信资源及其所有权

多个逻辑设备可以共享同一条物理总线：

```text
Robot MotorBus ─── BusChannel ──┐
                                ├── Shared Physical Bus
Custom Tool ────── BusChannel ──┘
```

关键原则：

> **逻辑设备解耦，物理通信资源可共享**

独立工具不需要伪装成 Robot Joint

Robot 也不应为了访问独立工具而暴露通用 raw bus API

### 7.8. Adapter

Adapter 负责外部框架与 SerialArm-Core 之间的数据和生命周期适配

典型 Adapter：

```text
Native C++
Python
ROS 2 / ros2_control
LeRobot
Isaac Sim / Isaac Lab
```

Adapter 负责：

```text
数据类型转换
生命周期桥接
Framework callback 接入
状态暴露
```

Adapter 不重新实现：

```text
Dynamics
Safety
通用 Control Capability
```

### 7.9. Robot Support / Profile

Robot Support 负责具体机器人实例资源：

```text
URDF
Mesh
Core Config
Hardware Config
Adapter Config
可选外部框架配置
```

Robot Profile 负责聚合这些资源

它解决的是：

> **同一个 Core 如何启动不同机械臂**

而不是：

> **为每台机械臂复制一份 Core**

---

## 8. Reference 与 Trajectory 的边界

SerialArm-Core 接受通用 Reference，但不拥有具体上层框架或具体任务的轨迹规划语义

需要区分：

```text
Trajectory
    对未来一段运动的描述

Reference
    当前控制周期希望机器人跟踪的目标状态
```

数据关系：

```text
Trajectory / Policy / User Command
              ↓
        Reference Source
              ↓
           Reference
              ↓
        SerialArm-Core
```

因此 Core 不直接依赖：

```text
MoveIt trajectory
FollowJointTrajectory Action
应用级路径类型
学习框架 policy output type
```

如果未来需要通用轨迹采样能力，也应建立在 Core 自身语义上，而不是依赖某个上层框架

---

## 9. 架构设计原则

### 9.1. Core 不依赖具体上层框架

Core 不 include ROS、MoveIt、LeRobot、Isaac 等外部框架的数据类型

### 9.2. Core 不拥有具体任务语义

Pick、Place、Navigation、Harvest 等业务任务属于 Application

### 9.3. Core 不重新实现成熟基础算法

优先复用成熟运动学、动力学和数学基础设施

### 9.4. Backend 必须完成语义归一化

Core 中的 position、velocity、torque、gain 等数据必须具有统一单位和方向语义

### 9.5. Capability 必须跨机器人可复用

允许模型和参数变化

不允许为不同机械臂复制同一算法主体

### 9.6. Adapter 保持薄

Adapter 只负责框架桥接

通用控制算法必须留在 Core

### 9.7. Shared Bus 属于 Resource

共享通信总线服务多个逻辑设备

设备之间保持 API 和生命周期解耦

### 9.8. Runtime 是执行核心，不是全栈调度器

Runtime 负责执行控制周期

不负责 perception、planning、task orchestration 和 global trajectory generation

### 9.9. 高级能力默认可选

新增高级 Capability 不能显著提高基础用户的使用门槛

### 9.10. Safety 具有最终命令裁决权

任何 Controller、Capability 和 Adapter 都不能绕过 Safety

### 9.11. 一个能力只维护一份核心实现

C++、Python、ROS 2、LeRobot、Isaac 应尽量复用同一个 Core 实现

### 9.12. 不为假想需求提前建设大型抽象

没有真实需求时，不提前建设：

```text
大型插件系统
内部消息总线
通用任务调度器
动态 Capability DAG
第二套 Controller Manager
自定义全栈 Motion Planner
```

只有真实使用场景证明当前抽象不足时再扩展

---

## 10. 新功能归属判断

新增功能时按以下顺序判断：

```text
是否属于具体应用任务
    │
    ├─ 是 → Application
    │
    └─ 否
         ↓
是否属于外部框架桥接
    │
    ├─ 是 → Adapter
    │
    └─ 否
         ↓
是否属于具体机器人实例资源
    │
    ├─ 是 → Robot Support / Profile
    │
    └─ 否
         ↓
是否属于具体厂商硬件或协议
    │
    ├─ 是 → Hardware / Protocol
    │
    └─ 否
         ↓
是否只依赖通用 State / Reference / Model
并直接作用于 Command / Runtime Feedback
    │
    ├─ 是 → Core Capability
    │
    └─ 否 → 重新判断边界
```

目标不是让更多功能进入 Core

而是保证进入 Core 的功能具有足够强的通用性

---

## 11. 可移植性验收标准

SerialArm-Core 的可移植性不能只通过支持列表证明，需要通过适配成本验证

### 11.1. 新 Robot Variant

已有 Hardware Backend 时，主要通过：

```text
URDF
Robot Config
Profile
```

完成接入

不修改 Core Control 代码

### 11.2. 新 Hardware Backend

主要新增：

```text
Hardware implementation
Vendor protocol / transport adapter
Hardware config
```

不修改 Model、Dynamics 和通用 Capability

### 11.3. 新上层 Framework

主要新增 Adapter

不复制 Robot、Dynamics、Safety 和 Capability

### 11.4. 新 Control Capability

Capability 建立在：

```text
State
Reference
Model / Dynamics
```

之上

先在 Native Runtime 中独立验证，再由不同 Adapter 复用

### 11.5. 跨机器人算法复用

同一 Capability 复用到不同串联机械臂时，只允许主要变化：

```text
Robot Model
Hardware Backend
Control Parameters
```

不修改算法主体

这比单纯声明“支持 N-DOF serial manipulator”更能证明实际可移植性

---

## 12. 复杂度控制

SerialArm-Core 长期坚持：

> **能力可以增长，但核心概念数量尽量不增长**

新增能力优先组合现有：

```text
State
Reference
Model
Capability
Command
```

而不是增加新的 Runtime、数据体系和生命周期体系

基础使用应保持简单

高级能力在同一 Runtime 上按需增加，而不是切换到另一套框架

不为了支持列表提前维护没有真实使用场景的 Adapter

Core 的稳定性、接口一致性和适配成本优先于功能数量

---

## 13. 架构结论

SerialArm-Core 最终应保持如下稳定关系：

```text
             Upper Framework / Application
                        │
                     Reference
                        ▼
┌────────────────────────────────────────────┐
│              SerialArm-Core               │
│                                            │
│  Unified Semantics                         │
│  Robot Model / Dynamics                    │
│  Runtime                                   │
│  Control Capability                        │
│  Safety                                    │
└────────────────────────────────────────────┘
                        │
                      Command
                        ▼
              Hardware Abstraction
                        │
               Transport Resource
                        │
                   Real Robot
```

最重要的边界始终是：

> **上层决定机器人为什么运动、运动到哪里**

> **SerialArm-Core 决定给定当前状态和参考后，机械臂如何以统一、安全、可建模、可扩展的方式执行这个运动**

后续任何新功能都优先使用这一边界判断其是否属于 SerialArm-Core
