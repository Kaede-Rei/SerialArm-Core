<div align="center">

# SerialArm-Core

[简体中文](README.md) · [English](README_en.md)

**串联机械臂控制内核与 SerialArm Studio 工作台**

配置模型与驱动，标定动力学与增益，通过 C++、Python 或 ROS2 控制机械臂

<p>
  <a href="LICENSE"><img src="https://img.shields.io/github/license/Kaede-Rei/SerialArm-Core?style=flat-square&amp;label=License" alt="许可证" /></a>
  <img src="https://img.shields.io/badge/Platform-Linux-334155?style=flat-square&amp;logo=linux&amp;logoColor=white" alt="Linux" />
  <img src="https://img.shields.io/badge/Core-C%2B%2B17-00599C?style=flat-square&amp;logo=cplusplus&amp;logoColor=white" alt="C++17" />
  <img src="https://img.shields.io/badge/Binding-Python-3776AB?style=flat-square&amp;logo=python&amp;logoColor=white" alt="Python Binding" />
  <img src="https://img.shields.io/badge/Adapter-ROS2-22314E?style=flat-square&amp;logo=ros&amp;logoColor=white" alt="ROS2" />
  <img src="https://img.shields.io/badge/Desktop-Electron-47848F?style=flat-square&amp;logo=electron&amp;logoColor=white" alt="Electron" />
</p>

[项目简介](#项目简介) · [功能与界面](#功能与界面) · [系统架构](#系统架构) · [快速开始](#快速开始) · [文档](#文档) · [参与贡献](#参与贡献) · [许可证](#许可证) · [问题反馈](https://github.com/Kaede-Rei/SerialArm-Core/issues)

</div>

---

## 项目简介

SerialArm-Core 将模型、硬件映射、动力学计算和控制参数组织在同一 Robot Profile 下；桌面应用 SerialArm Studio 提供模型查看、参数配置、控制、示教与标定；C++、Python 和可选的 ROS2 / MoveIt 适配层用于上层集成

达妙是仓库现有的内置硬件驱动，DM-Arm 是配套机械臂示例；其他硬件通过 `MotorBus` 驱动和 Hardware YAML 接入，驱动需提供当前控制链要求的位置、速度、力矩反馈与完整关节阻抗命令语义，接入步骤见 [机械臂接入指南](docs/guide.md)

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
| 运行检查 | 反馈与状态检查、限位、故障处理、控制准入与停放退出 |
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

### 使用桌面发行包

发行包与版本说明见 [Releases](https://github.com/Kaede-Rei/SerialArm-Core/releases)；AppImage 与 deb 面向 Ubuntu 22.04 x86_64，安装与运行步骤见 [发行包安装](docs/install.md#安装发行包)

默认发行包包含 Core、Terminal、Python Binding、DM-Arm 支持与 GUI，使用工作台无需安装 Node.js、编译工具或 ROS2；ROS2 / MoveIt 入口需要对应的可选组件与宿主环境

### 从源码安装

在仓库根目录运行；安装器需要 Linux 和 Python 3.10+，GUI 源码安装另需 Node.js 20+、npm 与图形桌面

```bash
git clone https://github.com/Kaede-Rei/SerialArm-Core.git
cd SerialArm-Core
./install.sh
./launch.sh
```

安装向导默认选择 Standalone，安装 Core、Terminal、模型检查工具与 GUI；Python Binding、ROS2 和 MoveIt 按需选择；组件、依赖与非交互命令见 [安装说明](docs/install.md)

已有 ROS2 工作区可直接使用 `colcon build`，步骤见 [ROS2 工作区构建](docs/install.md#直接在-ros2-工作区构建)；独立 C++ 与 Python 集成见 [开发说明](docs/development.md) 和 [API 参考](docs/reference/api.md)

### 先加载模型，再连接实机

首页新建机械臂 Profile，导入 URDF、Xacro 或 Description 包，选择 Base 与 Tool，检查控制链、关节限位与模型资源

保持 `control.runtime.write_enabled: false` 可使用 GUI 或 C++ Terminal 的离线会话，无需填写电机型号或安装硬件驱动

```bash
./.install/run serial_arm_terminal --config /absolute/path/to/core.yaml --machine
```

离线会话使用 mock 状态；接入实机前，选择驱动、核对映射与安全限制，开启写入并重新连接

使用内置或外部 Profile

```bash
./.install/run serial_arm_terminal --robot-profile dm_arm_gray

./.install/run serial_arm_terminal \
  --robot-profile your_robot \
  --profile-file /absolute/path/to/robot_profiles.yaml
```

这些入口需要已安装 Terminal；执行真机命令前核对设备、关节方向、零位和限位，并准备可靠支撑与现场停机措施

## 文档

| 文档 | 内容 |
| :--- | :--- |
| [安装说明](docs/install.md) | 发行包、源码安装、可选组件与图形诊断 |
| [使用指南](docs/guide.md) | 模型导入、首次连接、控制、示教、标定与导出 |
| [开发说明](docs/development.md) | 系统架构、源码目录、构建、驱动扩展与发布入口 |
| [API 参考](docs/reference/api.md) | C++、Python、控制接口与扩展契约 |
| [标定参考](docs/reference/calibration.md) | 重力与完整动力学辨识的数据、算法、验证和命令 |

## 参与贡献

通过 [Issues](https://github.com/Kaede-Rei/SerialArm-Core/issues) 报告问题或讨论功能，也可以提交 Pull Request；问题复现信息、修改范围与验证方式见 [贡献说明](CONTRIBUTING.md)

## 许可证

项目自有代码采用 [GNU GPL v3.0（GPL-3.0-only）](LICENSE)；随源码提供的第三方组件保留各自许可证，包括 [pybind11](src/serial_arm/core/third_lib/pybind11/LICENSE) 与 [tl::expected](src/serial_arm/core/third_lib/tl/COPYING)

达妙协议代码引自 Motor SDK，来源及待确认的上游许可见 [dm_hw 说明](src/robot_supports/hardware/damiao/third_lib/damiao/README.md#来源与许可)
