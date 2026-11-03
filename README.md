<div align="center">

# SerialArm-Core

**面向自研串联机械臂的动力学标定与控制内核**

配置模型与驱动，标定动力学与增益，通过 C++、Python 或 ROS2 控制机械臂

<p>
  <a href="https://github.com/Kaede-Rei/SerialArm-Core/blob/main/LICENSE"><img src="https://img.shields.io/github/license/Kaede-Rei/SerialArm-Core?style=flat-square&amp;label=License" alt="许可证" /></a>
  <img src="https://img.shields.io/badge/Platform-Linux-334155?style=flat-square&amp;logo=linux&amp;logoColor=white" alt="Linux" />
  <img src="https://img.shields.io/badge/Core-C%2B%2B17-00599C?style=flat-square&amp;logo=cplusplus&amp;logoColor=white" alt="C++17" />
  <img src="https://img.shields.io/badge/Binding-Python-3776AB?style=flat-square&amp;logo=python&amp;logoColor=white" alt="Python Binding" />
  <img src="https://img.shields.io/badge/Adapter-ROS2-22314E?style=flat-square&amp;logo=ros&amp;logoColor=white" alt="ROS2" />
  <img src="https://img.shields.io/badge/Desktop-Electron-47848F?style=flat-square&amp;logo=electron&amp;logoColor=white" alt="Electron" />
</p>

[项目简介](#项目简介) · [功能与界面](#功能与界面) · [系统架构](#系统架构) · [快速开始](#快速开始) · [文档](#文档与协作) · [问题反馈](https://github.com/Kaede-Rei/SerialArm-Core/issues)

</div>

---

## 项目简介

SerialArm-Core 面向需要自行配置硬件与运动学模型的串联机械臂，将 Robot Profile、关节控制、动力学计算、模型校正和安全状态管理组织为统一的机器人运行环境

用户可以从已有 URDF 或 Description 创建机器人实例，完成硬件映射、参数配置与动力学验证，再通过 C++、Python 或 ROS2 / MoveIt 接入上层应用

图形界面、Terminal 和 Python 直接使用 C++ Core，无需宿主 ROS2 环境；ROS2 与 MoveIt 按需接入，下游机械臂在自己的 Profile 中维护模型、驱动和标定结果

<p align="center">
  <img src="docs/images/launcher-new-arm.png" alt="新机械臂导入页面展示三维模型、控制链与配置步骤" width="94%" />
</p>
<p align="center"><sub>新机械臂接入 · 从模型导入开始建立独立机器人 Profile</sub></p>

## 功能与界面

### 从接入到标定

<table>
  <tr>
    <td width="33%" align="center"><strong>工作区入口</strong></td>
    <td width="33%" align="center"><strong>机器人配置</strong></td>
    <td width="33%" align="center"><strong>动力学标定</strong></td>
  </tr>
  <tr>
    <td><img src="docs/images/launcher-start.png" alt="工作区入口" width="100%" /></td>
    <td><img src="docs/images/launcher-profile.png" alt="机器人 Profile 配置" width="100%" /></td>
    <td><img src="docs/images/launcher-dynamics.png" alt="动力学标定工作台" width="100%" /></td>
  </tr>
  <tr>
    <td>新建机械臂、继续配置或进入已有机器人</td>
    <td>组织模型、执行器映射、控制参数与准备状态</td>
    <td>示教采集、模型校正、独立验证与候选应用</td>
  </tr>
</table>

### 明暗主题与模型预览

<table>
  <tr>
    <td width="50%" align="center"><strong>浅色主题</strong></td>
    <td width="50%" align="center"><strong>深色主题</strong></td>
  </tr>
  <tr>
    <td><img src="docs/images/launcher-light.png" alt="浅色主题控制工作台" width="100%" /></td>
    <td><img src="docs/images/launcher-dark.png" alt="深色主题控制工作台" width="100%" /></td>
  </tr>
</table>

### 核心能力

| 模块 | 主要能力 |
| :--- | :--- |
| 机器人接入 | Description / URDF / Xacro 导入、Profile 创建与资源管理 |
| 模型与动力学 | 正运动学、Jacobian、质量矩阵、逆动力学与重力项计算 |
| 真机控制 | 原生阻抗、模型导纳、动力学前馈、示教与轨迹回放 |
| 标定 | 执行器增益自动标定、重力校正、几何实验与完整动力学辨识 |
| 硬件抽象 | 关节与执行器映射、Backend、CAN / Serial Transport |
| 运行安全 | 状态检查、限位、故障处理、控制准入与安全退出 |
| 上层接入 | 原生 C++、Python Binding、ROS2 ros2_control 与 MoveIt 2 |
| 桌面工作台 | Profile 配置、模型预览、控制、标定与诊断 |

控制页分别选择控制方式、执行器增益用途与运动目标；增益用途包括刚性、柔性、拖拽和伺服跟踪，模型导纳使用伺服增益跟随虚拟参考

关节滑条与数值框共用一组目标，青绿色半透明模型用于预览，确认移动后才发送命令；交互调试和示教回放使用同一控制上下文

## 系统架构

SerialArm-Core 将上层应用、机器人配置与硬件驱动解耦，控制和动力学能力由同一 C++ Core 提供

![SerialArm-Core 系统架构](docs/images/architecture.svg)

准备页面分别显示连接前配置、运行检查、基础标定、专项标定与检查记录；控制权限由当前配置、反馈和 Core 安全检查决定

![新机械臂接入工作流程](docs/images/workflow.svg)

## 快速开始

### 环境要求

- Linux，桌面发行包面向 Ubuntu x86_64
- C++17、Python 3.10 及以上
- 桌面 GUI 需要 Node.js 20 及以上、npm 与图形桌面
- ROS2 / MoveIt 是可选适配层，仅使用这些入口时需要相应环境

### 一键安装与启动

```bash
./install.sh
./launch.sh
```

安装向导默认选择 Standalone，可独立使用 Core、Terminal 与桌面 GUI；Python、ROS2 和 MoveIt 按需选择，安装计划列出所需系统依赖

### Linux 发行包构建

只在准备 Release 时生成 AppImage 与 deb，日常修改、编译验证和更新本机安装不运行发布打包

默认发行包包含 Core、Terminal、Python Binding、DM-Arm 与 GUI，启动和控制不依赖 ROS2

```bash
./build_linux.sh
# 仅复用与当前源码一致的独立安装
./build_linux.sh --reuse-install
# 显式增加 ROS2 / MoveIt 适配层
./build_linux.sh --with-ros2
```

产物为 `dist/SerialArm-Core-linux-x86_64.AppImage` 与 `dist/SerialArm-Core-linux-amd64.deb`

构建机使用 Ubuntu 22.04 x86_64、Node.js 20+；默认发行包可在未安装 ROS2 的目标机运行，包含可选适配层时也只在启动 Hardware 或 MoveIt 入口时加载宿主 ROS2 环境

详细要求见 [安装说明](docs/install.md)

### 在已有 ROS2 工作区构建

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

在包含 `serial_arm_core` 源码的工作区根目录执行即可，Python Binding 使用仓库提供的 pybind11 构建依赖，无须提前运行独立安装脚本

### 使用内置或外部 Profile

```bash
./.install/run serial_arm_terminal --robot-profile dm_arm_gray
```

下游机械臂使用独立 Profile 文件与对应模型资源

```bash
./.install/run serial_arm_terminal \
  --robot-profile your_robot \
  --profile-file /absolute/path/to/robot_profiles.yaml
```

执行真机控制前应检查关节映射、限位、模型与执行器写入配置，确保现场具备安全停机条件

## 文档与协作

| 文档 | 内容 |
| :--- | :--- |
| [安装说明](docs/install.md) | 依赖环境、组件选择、命令行安装与运行入口 |
| [GUI 使用指南](docs/launcher.md) | 机器人工作区、模型、控制、标定与诊断 |
| [机器人接入指南](docs/tutorial.md) | Profile、硬件映射、控制与安全参数 |
| [系统架构](docs/architecture.md) | Core、Adapter、Hardware、Description 与 Profile 的职责边界 |
| [API 参考](docs/reference/api.md) | C++、Python、控制接口与扩展契约 |
| [问题反馈](https://github.com/Kaede-Rei/SerialArm-Core/issues) | 问题报告与功能建议 |

---

<div align="center">

**SerialArm-Core**

Model · Calibration · Control · Integration

</div>
