<div align="center">

# SerialArm-Core

面向自研串联机械臂的 C++17 控制、动力学、安全与硬件抽象库

提供 Robot Profile、C++ Terminal、Python Binding、ROS2 Adapter、MoveIt 与桌面 GUI

[![License](https://img.shields.io/github/license/Kaede-Rei/SerialArm-Core?style=flat-square)](https://github.com/Kaede-Rei/SerialArm-Core)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)](https://isocpp.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Optional-00A896?style=flat-square)](https://moveit.picknik.ai/)

</div>

## 安装与启动

Linux / Python 3.10+，GUI 需要 Node.js 20+、npm 和桌面环境

ROS2 方案需要预先安装 ROS2，主要面向 Ubuntu 22.04 + Humble

```bash
./install.sh
./launch.sh
```

安装器会展示功能组合和确认计划，系统依赖需要权限时在当前终端提示 sudo 密码

| 用途 | 安装内容 | 适用场景 |
| --- | --- | --- |
| `core` | Core + Dynamics | 在自己的 C++ 程序中集成控制库 |
| `standalone` | Core + Terminal + GUI，可选 DM-Arm 支持 | 不使用 ROS2，直接调试机械臂 |
| `python` | Core + Python Binding | 在独立 Python 环境中调用控制和动力学接口 |
| `ros2` | Core + Terminal + Python + ROS2 + DM-Arm + MoveIt + GUI | 使用 ros2_control 和 MoveIt |
| `dev` | ROS2 方案全部组件 + 开发测试 | 修改源码并运行测试 |
| `custom` | Core 起步，自选组件并自动补齐依赖 | 按项目需要组合安装 |

Core → Standalone → ROS2 → Developer 对应逐步增加的集成需求，Python 是独立接口选项

已有 Core 安装时单独补装 GUI

```bash
./install.sh --gui-only --yes
```

自动确认安装或预览计划

```bash
./install.sh --preset ros2 --yes
./install.sh --preset ros2 --dry-run
```

## 使用

GUI 中选择 Profile、检查配置，可进入 Model、Control Workspace、Terminal、Hardware 或 MoveIt

命令行入口自动加载对应环境

```bash
./.install/run serial_arm_terminal --robot-profile dm_arm_gray
./.install/run ros2 launch serial_arm_ros2_control display.launch.py robot_profile:=dm_arm_gray
```

运行需要对应组件已安装，Terminal、Hardware 与 MoveIt 可能连接真实执行器

外部机器人使用下游项目自己的 Profile 和资源

```bash
./.install/run serial_arm_terminal \
  --robot-profile tomato_picker \
  --profile-file /absolute/path/robot_profiles.yaml
```

## 能力与配置

Core 提供生命周期、关节控制、安全、FK、Jacobian、动力学、五种关节阻抗模式与关节导纳，桌面工作台复用同一 C++ 会话完成控制、调参、标定与诊断

Robot Profile 将 Core YAML、Hardware Backend、模型、Controllers 和 MoveIt 资源组合为一个机器人实例

内置 `dm_arm_gray` 与 `dm_arm_white`，其他机械臂由自己的 Profile 定义关节、坐标系与标定参数

## 目录

| 路径 | 职责 |
| --- | --- |
| `install.sh` / `launch.sh` | 安装与桌面启动入口 |
| `src/serial_arm/core/` | 通用控制库、Python、Terminal 与测试 |
| `src/serial_arm/bringup/ros2_control/` | ROS2 Adapter 与 launch |
| `src/robot_supports/profiles/` | 内置 Profile 注册 |
| `src/robot_supports/robots/` | 机器人模型、参数与 MoveIt 配置 |
| `src/robot_supports/hardware/` / `protocol/` | 执行器后端与通信协议 |
| `apps/launcher/` | Electron 界面、Python 后端与测试 |
| `tools/` / `conanfile.py` | 安装器、Standalone 构建与依赖声明 |
| `docs/` | 使用说明与 API 参考 |
| `docs/other/` | 私人阅读资料与原始长文，Git 忽略 |

`build/`、`install/`、`log/` 与 `.install/` 是自动生成目录，Git 忽略

## 文档

- [安装与参数](docs/install.md)
- [GUI 使用](docs/launcher.md)
- [配置与机器人接入](docs/tutorial.md)
- [架构与职责边界](docs/architecture.md)
- [API 参考](docs/reference/api.md)
- [开发与验证](docs/development.md)

MIT License，见 [LICENSE](LICENSE)
