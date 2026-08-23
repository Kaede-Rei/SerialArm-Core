<div align="center">

# SerialArm-Core

Portable C++17 control, dynamics, safety and hardware abstraction core for custom serial manipulators

面向自研串联机械臂的 C++17 控制、动力学、安全与硬件抽象核心

[![License](https://img.shields.io/github/license/Kaede-Rei/SerialArm-Core?style=flat-square)](https://github.com/Kaede-Rei/SerialArm-Core)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)](https://isocpp.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Optional-00A896?style=flat-square)](https://moveit.picknik.ai/)

</div>

## 定位

SerialArm-Core 是面向自研串联机械臂的通用控制能力库，将机器人模型、关节与执行器映射、控制、安全、动力学、交互控制和硬件 Backend 统一在同一套 Core 接口中

仓库同时提供 Robot Profile、C++ Terminal、Python Binding、ros2_control Adapter 和 MoveIt 2 启动入口，可用于机械臂基础控制、柔顺控制、动力学实验、真机调试以及上层机器人框架接入

当前仓库内置 DM-Arm 的 Gray 和 White 两套 Robot Profile，并提供 Damiao Hardware Backend 与 Damiao USB2CAN Protocol

## 核心能力

| 能力 | 作用 | 主要入口 |
| --- | --- | --- |
| Robot Profile | 聚合 Core、Hardware、URDF、Controllers 和 MoveIt 资源 | `robot_profiles.yaml` |
| Robot Control | 生命周期、跟踪命令、保持、FAULT 与停放 | `Robot` |
| Joint / Actuator Mapping | 方向、比例、零位与执行器语义转换 | `calibration.joints` |
| Safety | 位置、速度、加速度、状态超时、命令超时与故障恢复 | `safety_policy` |
| Dynamics | FK、Jacobian、Gravity、Coriolis、Mass Matrix、Inverse Dynamics | `Dynamics` |
| Impedance | 五种关节阻抗工作模式 | `JointImpedanceMode` |
| Admittance | 无力传感器外力估计与固定 M / D / K 关节空间导纳 | `capability.admittance` |
| Hardware Abstraction | 统一 `position / velocity / torque / kp / kd` 执行器语义 | `MotorBus` |
| Shared Transport | CAN channel fan-out 与 Serial transaction arbitration | `CanChannel` / `SerialBusClient` |
| Python | Python 控制会话与 Dynamics 调用 | `RobotSession` / `Dynamics` |
| ROS 2 | ros2_control SystemInterface | `serial_arm_ros2_control` |
| MoveIt 2 | 基于 Robot Profile 启动硬件、move_group 与 RViz | `moveit.launch.py` |
| Terminal | 真机联通、运动、动力学、阻抗、导纳和故障调试 | `serial_arm_terminal` |

## 仓库结构

`serial_arm/` 是通用核心实现，日常接入新机械臂时通常不需要修改

`robot_supports/` 保存与具体机器人、执行器和通信设备相关的资源，也是使用者最常修改的部分

```text
src/
├── serial_arm/
│   ├── core/                              # Core、Python Binding、Terminal、Transport
│   └── bringup/ros2_control/              # ROS 2 / ros2_control Adapter
│
└── robot_supports/
    ├── profiles/	# 机器人汇总（统一入口）
    │   └── config/
    │       └── robot_profiles.yaml        # 机器人入口，聚合 Core、Hardware、URDF、Controller、MoveIt
    │
    ├── robots/		# 机器人具体配置
    │   └── dm_arm/
    │       ├── description/
    │       │   ├── config/
    │       │   │   ├── core/
    │       │   │   │   ├── gray.yaml     # Gray Core、Control、Safety、Admittance 配置
    │       │   │   │   └── white.yaml    # White Core、Control、Safety、Admittance 配置
    │       │   │   ├── hardware.yaml      # Joint 到执行器、ID、串口和 Backend 参数
    │       │   │   └── ros2_controllers.yaml
    │       │   └── model/
    │       │       ├── gray/
    │       │       │   ├── urdf/          # Gray URDF 与 ros2_control Xacro
    │       │       │   └── meshes/
    │       │       └── white/
    │       │           ├── urdf/          # White URDF 与 ros2_control Xacro
    │       │           └── meshes/
    │       └── moveit_config/
    │           ├── dm_arm_no_gripper/     # Gray 对应 MoveIt 配置
    │           └── dm_arm_with_gripper/   # White 对应 MoveIt 配置
    │
    ├── hardware/	# 执行器驱动
    │   └── damiao/                        # Damiao MotorBus Backend
    │       ├── include/
    │       ├── src/
    │       └── tests/
    │
    └── protocol/	# 通信协议
        └── damiao_usb2can/                # Damiao 官方 USB2CAN 私有协议适配
            ├── include/
            ├── src/
            └── tests/
```

接入新机械臂时优先在 `robot_supports/robots/` 创建 Robot Support，并在 `robot_supports/profiles/config/robot_profiles.yaml` 注册 Profile

只有执行器语义或厂商通信发生变化时才需要新增 `hardware/` 或 `protocol/`


## 使用入口

根据目标选择对应文档：

```text
第一次运行已有机械臂
    ↓
README Quick Start
    ↓
Tutorial 基础配置与 Terminal

理解控制算法与参数
    ↓
Tutorial Dynamics / Impedance / Admittance
    ↓
原理说明文档

新增机械臂或硬件适配
    ↓
Tutorial Profile / Hardware / Transport
    ↓
API Reference

接入 ROS 2 / MoveIt
    ↓
Tutorial ROS 2 Adapter
    ↓
Architecture
```

## Quick Start

### 1 获取源码与构建

```bash
git clone https://github.com/Kaede-Rei/SerialArm-Core.git
cd SerialArm-Core
```

如果使用 ROS 2、ros2_control、MoveIt 2，或者只是希望一次构建整个仓库，优先使用 colcon

只有需要独立安装 Core、Protocol、Hardware Backend 或 Robot Support 时才使用 Standalone CMake

#### 1.1 colcon 全仓构建

这是仓库最直接的整仓构建方式，同时安装 Native C++、Python Binding、C++ Terminal、Robot Support 和 ROS 2 Adapter

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --rosdistro humble

colcon build --symlink-install
source install/setup.bash
```

构建后即使不启动 ROS 2，也可以继续直接使用 Native C++、Python Binding 和 C++ Terminal

#### 1.2 只构建 Core 并运行测试

适合只检查 Core、Dynamics、Safety、Mapping 和配置系统

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=OFF

cmake --build build/serial_arm_core -j
ctest --test-dir build/serial_arm_core --output-on-failure
```

#### 1.3 Standalone CMake 按需构建

Standalone CMake 是组件级安装方式，不要求把下面所有包都构建一遍

使用已有 DM-Arm Profile 并运行 C++ Terminal 时需要 Core、Damiao Protocol、Damiao Backend、Profiles 和 DM-Arm resources

其他机械臂只构建实际需要的对应组件

```bash
# Core + C++ Terminal，Standalone 基础组件
cmake -S src/serial_arm/core -B build/serial_arm_core -DCMAKE_BUILD_TYPE=Release -DSERIAL_ARM_BUILD_PYTHON=OFF -DSERIAL_ARM_BUILD_TERMINAL=ON
cmake --build build/serial_arm_core -j && cmake --install build/serial_arm_core --prefix install/standalone

# 仅使用 Damiao 官方 USB2CAN 时需要
cmake -S src/robot_supports/protocol/damiao_usb2can -B build/serial_arm_protocol_damiao_usb2can -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/install/standalone"
cmake --build build/serial_arm_protocol_damiao_usb2can -j && cmake --install build/serial_arm_protocol_damiao_usb2can --prefix install/standalone

# 仅使用 Damiao Hardware Backend 时需要
cmake -S src/robot_supports/hardware/damiao -B build/serial_arm_hardware_damiao -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/install/standalone"
cmake --build build/serial_arm_hardware_damiao -j && cmake --install build/serial_arm_hardware_damiao --prefix install/standalone

# 使用 Robot Profile 时需要
cmake -S src/robot_supports/profiles -B build/serial_arm_robot_profiles
cmake --install build/serial_arm_robot_profiles --prefix install/standalone

# 使用仓库内置 DM-Arm resources 时需要
cmake -S src/robot_supports/robots/dm_arm/description -B build/dm_arm_description
cmake --install build/dm_arm_description --prefix install/standalone

# Standalone 安装目录的资源与动态库搜索路径
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
```

Standalone CMake 的逐组件说明和新 Robot Support 的安装方法见 [Tutorial.md](Tutorial.md)

#### 1.4 Python wheel

适合单独安装 standalone Python Binding

```bash
cd src/serial_arm/core/python
python -m pip install build
python -m build --wheel
python -m pip install --force-reinstall dist/serial_arm-*.whl
cd ../../../..
```

### 2 选择已有 Robot Profile

当前仓库可直接使用的 DM-Arm Profile 如下

| Profile | Core Config | Robot Description | MoveIt Package | 典型用途 |
| --- | --- | --- | --- | --- |
| `dm_arm_gray` | `config/core/gray.yaml` | Gray 无夹爪 | `dm_arm_no_gripper` | Native C++、Terminal、动力学与 MOMENTUM 导纳 |
| `dm_arm_white` | `config/core/white.yaml` | White 带夹爪模型 | `dm_arm_with_gripper` | Native C++、Terminal、ROS 2 与 FULL_ID 导纳 |

Profile 将下面几类资源聚合为一个可启动机器人实例

```text
Robot Profile
├── Core YAML
├── Hardware Backend
├── Hardware YAML
├── URDF / ros2_control Xacro
├── ros2_controllers.yaml
└── MoveIt config package
```

查看当前 Profile

```bash
cat src/robot_supports/profiles/config/robot_profiles.yaml
```

### 3 检查硬件连接

DM-Arm Hardware Config 默认使用 `/dev/ttyACM0`

```bash
ls /dev/ttyACM*
```

如果实际设备为 `/dev/ttyACM1`，不需要修改 Profile，可以对当前进程覆盖串口

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1
```

运行时覆盖优先级如下

```text
runtime override > hardware.yaml
```

`--serial-port`、`--baudrate` 和 `--bus` 只修改当前进程，不写回 Hardware YAML

### 4 使用 C++ Terminal

最常用启动方式

```bash
serial_arm_terminal --robot-profile dm_arm_gray
```

查看命令行参数

```bash
serial_arm_terminal --help
```

可用启动参数及作用

| 参数 | 作用 |
| --- | --- |
| `--robot-profile <name>` | 使用已有 Robot Profile |
| `--profile-file <path>` | 显式指定 Profile 文件 |
| `--config <path>` | 不使用 Profile 时直接指定 Core YAML |
| `--hardware-plugin <name>` | 不使用 Profile 时直接指定 Hardware Backend |
| `--hardware-config <path>` | 不使用 Profile 时直接指定 Hardware YAML |
| `--serial-port <path>` | 覆盖串口设备 |
| `--baudrate <n>` | 覆盖串口波特率 |
| `--bus <name>` | 覆盖 Hardware Bus 名称 |
| `--compare-config <a> <b>` | 使用指定 Backend capabilities 比较两份 Core Config |
| `--help` / `-h` | 查看用法 |

不使用 Robot Profile 时可以直接指定三项路径

```bash
serial_arm_terminal \
  --config <core.yaml> \
  --hardware-plugin <backend.so> \
  --hardware-config <hardware.yaml>
```

进入 Terminal 后的主菜单如下

| 菜单 | 作用 |
| --- | --- |
| `状态查看` | Robot 状态、Joint / Actuator 周期状态、执行器参数和配置摘要 |
| `使能 / 失能 / 故障` | activate、停放失能、立即失能、FAULT 恢复与 clear_fault |
| `模式与补偿` | 五种阻抗模式、模型前馈模式和 gravity scale |
| `运动与命令` | 绝对位置移动、相对移动、取消运动并保持当前位置 |
| `动力学与配置` | 动力学向量、Mass Matrix、Jacobian、Frame 状态和配置摘要 |
| `调参与测试` | 导纳一次性 / 分步标定、M / D / K 调参、实时状态与 Observer 诊断 |
| `安全退出` | 按 shutdown 配置回到 park pose 后失能退出 |

第一次连接机械臂时优先使用 `状态查看`、`动力学与配置` 和离线配置检查，确认方向、零位、限位和执行器型号后再允许真机写入

### 5 Python Quick Start

检查 Binding

```bash
python3 -c "import serial_arm; print(serial_arm.__file__)"
```

使用 Python Terminal 做 Profile 和配置检查

```bash
python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray \
  --check-only
```

Python 控制入口

```python
import serial_arm

arm = serial_arm.RobotSession(
    core_yaml,
    hardware_plugin,
    hardware_yaml,
)

arm.start()
arm.set_impedance_mode(serial_arm.JointImpedanceMode.RIGID_TRACKING)
arm.move_to(target, speed_scale=0.15)
arm.stop()
```

`RobotSession` 维护 C++ 控制线程，Python 上层不需要自己实现高频 `Robot::cycle()` 循环

### 6 Native C++ Quick Start

外部 CMake 项目可以直接链接已安装的 Core

```cmake
find_package(serial_arm_core CONFIG REQUIRED)

add_executable(my_robot_app main.cpp)

target_link_libraries(my_robot_app
  PRIVATE
    serial_arm::core
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
```

Native C++ 的主要执行链如下

```text
load Robot Profile
    ↓
HardwareLoader
    ↓
load_robot_cfg
    ↓
Dynamics
    ↓
Robot::configure
    ↓
Robot::activate
    ↓
set_cmd + cycle
    ↓
Robot::deactivate
```

完整 C++ 示例、模型回调和 MOMENTUM 导纳所需 `InteractionModelStateFn` 见 [Tutorial.md](Tutorial.md)

### 7 ROS 2 / ros2_control Quick Start

只显示模型，不连接 Hardware Backend

```bash
ros2 launch serial_arm_ros2_control display.launch.py \
  robot_profile:=dm_arm_gray
```

启动 ros2_control hardware

```bash
ros2 launch serial_arm_ros2_control hardware.launch.py \
  robot_profile:=dm_arm_white
```

需要覆盖串口时

```bash
ros2 launch serial_arm_ros2_control hardware.launch.py \
  robot_profile:=dm_arm_white \
  serial_port:=/dev/ttyACM1
```

检查接口和 Controller

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

当前 `SerialArmSystem` 向 ros2_control 暴露 position 和 velocity command interfaces，并暴露 position、velocity 和 effort state interfaces

### 8 MoveIt 2 Quick Start

MoveIt 通过 `serial_arm_ros2_control` Adapter 使用同一个 Robot Profile，不直接访问 MotorBus

当前链路如下

```text
MoveIt 2
  ↓
move_group
  ↓
JointTrajectoryController
  ↓
ros2_control SerialArmSystem
  ↓
Robot
  ↓
Hardware Backend
```

使用现有 `dm_arm_white` Profile 启动

```bash
ros2 launch serial_arm_ros2_control moveit.launch.py \
  robot_profile:=dm_arm_white
```

该入口会同时启动

```text
hardware.launch.py
move_group.launch.py
moveit_rviz.launch.py
```

需要覆盖串口时

```bash
ros2 launch serial_arm_ros2_control moveit.launch.py \
  robot_profile:=dm_arm_white \
  serial_port:=/dev/ttyACM1
```

`display.launch.py` 只用于检查模型，`hardware.launch.py` 用于直接检查 ros2_control 真机链路，`moveit.launch.py` 在硬件链路之上继续启动 MoveIt

当前 ros2_control Adapter 为 Robot 提供 `ModelFeedforwardFn`，但没有提供 MOMENTUM Observer 所需的 `InteractionModelStateFn`

因此通过当前 ros2_control / MoveIt Adapter 使用导纳时应使用与 Adapter 能力匹配的配置，`dm_arm_white` 当前使用 FULL_ID Observer，`dm_arm_gray` 当前使用 MOMENTUM Observer，MOMENTUM 导纳应通过 Native C++ 或 C++ Terminal 使用，或者先扩展 Adapter 的 Interaction Model State 回调

### 9 开始调试前确认 write_enabled

Core YAML 中的开关决定是否允许真实 Robot 使用 Hardware Backend 写入执行器

```yaml
control:
  runtime:
    write_enabled: false
```

新机械臂、新零位或新硬件配置第一次调试时应先保持 `false`

完成 Joint direction、zero、URDF limit、HardwareCapabilities、Safety 和 park pose 检查后再切换为真机写入

## Robot Profile 示例

`dm_arm_gray` 的 Profile 结构如下

```yaml
profiles:
  dm_arm_gray:
    core:
      package: dm_arm_description
      config: config/core/gray.yaml

    hardware:
      plugin: serial_arm_hardware_damiao
      config_package: dm_arm_description
      config: config/hardware.yaml

    description:
      package: dm_arm_description
      urdf: model/gray/urdf/dm_arm_no_gripper.urdf
      ros2_control_xacro: model/gray/urdf/dm_arm.ros2_control.xacro

    controllers:
      package: dm_arm_description
      config: config/ros2_controllers.yaml

    moveit:
      package: dm_arm_no_gripper
```

Native C++、Python、Terminal 和 ROS 2 Adapter 共用同一套 Robot Profile

## 文档

| 文档 | 适合阅读的内容 |
| --- | --- |
| [Tutorial.md](Tutorial.md) | 构建、已有 Profile 使用、YAML 配置、Terminal 调试、Dynamics、Impedance、Admittance、Adapter 和新机械臂接入 |
| [API.md](API.md) | C++ / Python 类型、函数、参数、返回值、错误码和 Transport / Hardware API |
| [Architecture.md](Architecture.md) | Core 架构、模块边界、Runtime 数据流、Transport 和 Adapter 设计 |

第一次使用仓库建议先完成本 README 的 Quick Start，再进入 Tutorial 按实际需求继续阅读

## License

许可证以仓库 [LICENSE](LICENSE) 为准
