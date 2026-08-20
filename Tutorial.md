# SerialArm-Core Tutorial

Tutorial 用于说明如何使用现有 Robot Profile、配置 Core 与 Hardware、通过 Terminal 调试机械臂、使用 Dynamics / Impedance / Admittance，以及如何通过 Python、ros2_control、MoveIt 和自定义 Adapter 接入上层应用

新增机械臂时也按照同一套 Profile、配置、能力验证和 Adapter 接入方法完成闭环

需要逐项查看类、函数、参数、返回值和错误码时阅读 [API.md](API.md)

需要理解模块边界、Runtime 数据流和 Transport 设计时阅读 [Architecture.md](Architecture.md)

## 阅读导引

第一次使用已有 Profile

```text
README Quick Start
  ↓
Tutorial 2 构建与 Profile
  ↓
Tutorial 3 配置
  ↓
Tutorial 8 Terminal
  ↓
Tutorial 4 基础控制
```

需要使用 Dynamics

```text
Tutorial 5
  ↓
API Dynamics
```

需要使用 Impedance

```text
Tutorial 6
  ↓
API JointImpedanceMode / Robot
```

需要使用 Admittance

```text
Tutorial 7
  ↓
API AdmittanceCapabilityCfg / RobotCycleOutput
```

需要使用 Python

```text
Tutorial 9 Python RobotSession
  ↓
API RobotSession
```

需要使用 ROS 2 或 MoveIt

```text
Tutorial 9 ros2_control / MoveIt
  ↓
Architecture Adapter boundary
```

需要接入新机械臂

```text
Tutorial 10
  ↓
API Config / HardwareLoader / MotorBus
```

需要开发新 Backend 或 Transport

```text
Tutorial 10 Hardware Backend / Protocol
  ↓
API MotorBus / HardwareLoader / Transport
  ↓
Architecture Hardware / Protocol / Transport
```

[toc]

---

## 1 使用模型与核心概念

### 1.1 Core 的职责

SerialArm-Core 负责把机械臂控制中与上层框架无关的能力统一在 `Robot` 周围

```text
Application
    │
    ├── Native C++
    ├── Python RobotSession
    ├── ros2_control
    └── MoveIt 2
            │
            v
         Adapter
            │
            v
          Robot
    ┌───────┼─────────┬──────────┐
    │       │         │          │
Controller Mapper   Safety   Interaction
    │       │         │          │
    └───────┴────┬────┴──────────┘
                 │
                 v
              MotorBus
                 │
                 v
         Hardware Backend
                 │
                 v
          Protocol / Transport
```

Core 不要求 ROS 2 才能运行，Native C++、Python 和 Terminal 都可以直接使用同一套 Robot Profile

### 1.2 Robot Profile

Robot Profile 是一个机器人实例的资源入口

它负责把下面内容组合起来

```text
Core YAML
Hardware Backend
Hardware YAML
Robot Description
ros2_control Xacro
Controllers Config
MoveIt Config Package
```

Profile 不是控制算法，也不是 Hardware Backend

它的作用是让不同入口使用同一套机器人资源

### 1.3 Backend

Hardware Backend 实现 `MotorBus`

Core 对执行器使用统一语义

```text
position  rad
velocity  rad/s
torque    N*m
kp
kd
```

厂商 SDK 的编码器计数、电流、RPM、私有控制帧和设备协议应留在 Backend 或 Protocol 层

### 1.4 Controller

Controller 根据当前阻抗模式和上层参考产生 nominal joint command

五种模式共用统一的 `position / velocity / torque / kp / kd` 语义

### 1.5 Dynamics

Dynamics 从 URDF 和 Joint State 计算运动学与动力学量

当前主要能力包括

```text
Forward Kinematics
Frame Pose
Jacobian
Gravity
Gravity Compensation
Coriolis
Mass Matrix
Inverse Dynamics
```

Dynamics 既可以独立使用，也可以作为 Robot 模型前馈和 Admittance Observer 的模型来源

### 1.6 Capability

`capability` 用于配置可选高级能力

当前 Core YAML 中的高级能力入口为

```yaml
capability:
  admittance:
    ...
```

基础位置控制、阻抗模式、Safety 和 Dynamics 不要求必须开启 Admittance

### 1.7 Adapter

Adapter 的职责是把上层框架的生命周期、命令和状态转换为 Core 语义

Adapter 不应该重复实现 Core 已经提供的 Dynamics、Safety、Impedance 或 Admittance 算法

当前仓库可以直接使用的上层入口包括

```text
Native C++
Python Binding / RobotSession
ros2_control SystemInterface
MoveIt 2 through ros2_control
```

### 1.8 Terminal

C++ Terminal 是工程调试入口

它适合完成

```text
Profile 和 Hardware 检查
Robot 状态检查
使能与失能
小幅运动
阻抗模式检查
Dynamics 检查
Gravity 调整
Admittance 标定与调参
FAULT 处理
安全退出
```

Terminal 不是必须嵌入最终应用的业务层接口

---

## 2 构建与使用已有 Robot Profile

### 2.1 基础依赖

Core 主要依赖

- CMake 3.20+
- C++17 compiler
- yaml-cpp
- Eigen3
- Pinocchio
- GTest 用于测试

Python Binding 还需要

- Python 3.10+
- pybind11
- NumPy
- scikit-build-core

ROS 2 Adapter 面向 ROS 2 Humble，并使用 ros2_control、controller_manager、xacro、robot_state_publisher、ament_cmake_python、ament_index_python 和 PyYAML 等组件

MoveIt 2 只在使用 MoveIt 路径时需要

### 2.2 获取仓库

```bash
git clone https://github.com/Kaede-Rei/SerialArm-Core.git
cd SerialArm-Core
```

后续命令默认从仓库根目录执行

### 2.3 只构建 Core 并运行测试

这条路径适合第一次确认 Core、Dynamics、Safety、Mapping 和配置系统

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=OFF

cmake --build build/serial_arm_core -j

ctest \
  --test-dir build/serial_arm_core \
  --output-on-failure
```

如果 Core tests 失败，应先解决依赖或源码问题，再进入 Hardware 和真机流程

### 2.4 Standalone CMake 按需构建

Standalone CMake 用于独立构建和安装仓库中的组件，不依赖 ROS 2 runtime

它不是必须把所有组件依次构建一遍的第二套整仓构建方式

如果希望一次完成整个仓库构建，直接使用 2.6 的 colcon 路径

如果只需要 Native C++ Core，执行 2.4.1 即可

如果使用仓库内置 DM-Arm + Damiao Backend + Robot Profile + C++ Terminal，再继续执行 2.4.2 到 2.4.6

接入其他机械臂时只安装对应的 Protocol、Hardware Backend、Profiles 和 Robot Support

| 组件 | 什么时候需要 |
| --- | --- |
| Core + Terminal | Standalone 使用 Core 或 C++ Terminal 时 |
| Damiao USB2CAN Protocol | 使用达妙官方 USB2CAN 时 |
| Damiao Hardware Backend | 使用 Damiao 执行器 Backend 时 |
| Robot Profiles | 通过 `--robot-profile` 加载机器人时 |
| DM-Arm resources | 使用 `dm_arm_gray` 或 `dm_arm_white` 时 |
| Resource Path | 使用 Standalone install prefix 解析 Profile 和动态库时 |

#### 2.4.1 Core 与 Terminal

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=ON

cmake --build build/serial_arm_core -j

cmake --install build/serial_arm_core \
  --prefix install/standalone
```

#### 2.4.2 Damiao USB2CAN Protocol

仅使用达妙官方 USB2CAN 时需要

```bash
cmake \
  -S src/robot_supports/protocol/damiao_usb2can \
  -B build/serial_arm_protocol_damiao_usb2can \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/standalone"

cmake --build \
  build/serial_arm_protocol_damiao_usb2can \
  -j

cmake --install \
  build/serial_arm_protocol_damiao_usb2can \
  --prefix install/standalone
```

#### 2.4.3 Damiao Hardware Backend

仅使用 Damiao Hardware Backend 时需要

```bash
cmake \
  -S src/robot_supports/hardware/damiao \
  -B build/serial_arm_hardware_damiao \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/standalone"

cmake --build build/serial_arm_hardware_damiao -j

cmake --install build/serial_arm_hardware_damiao \
  --prefix install/standalone
```

#### 2.4.4 Robot Profiles

需要通过 Profile 名称加载机器人时安装

```bash
cmake \
  -S src/robot_supports/profiles \
  -B build/serial_arm_robot_profiles

cmake --install build/serial_arm_robot_profiles \
  --prefix install/standalone
```

#### 2.4.5 DM-Arm resources

仅使用仓库内置 DM-Arm Profile 时安装

```bash
cmake \
  -S src/robot_supports/robots/dm_arm/description \
  -B build/dm_arm_description

cmake --install build/dm_arm_description \
  --prefix install/standalone
```

#### 2.4.6 资源搜索路径

Standalone install prefix 不在 ROS 2 overlay 中，因此需要显式提供资源与动态库搜索路径

```bash
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
```

安装完成后至少应能找到

```text
install/standalone/bin/serial_arm_terminal
install/standalone/lib/libserial_arm_protocol_damiao_usb2can.so
install/standalone/lib/libserial_arm_hardware_damiao.so
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
install/standalone/share/dm_arm_description/config/core/gray.yaml
install/standalone/share/dm_arm_description/config/hardware.yaml
install/standalone/share/dm_arm_description/model/...
```

### 2.5 Python wheel

Standalone Python 使用 wheel 安装

```bash
cd src/serial_arm/core/python

python -m pip install build
python -m build --wheel
python -m pip install --force-reinstall dist/serial_arm-*.whl

cd ../../../..
```

确认 Binding

```bash
python3 -c "import serial_arm; print(serial_arm.__file__)"
```

### 2.6 colcon 全仓构建

ROS 2、ros2_control 和 MoveIt 使用这条路径

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

colcon 构建后同样可以直接运行 Native C++、Python Binding 和 C++ Terminal

确认 workspace 中的 Python Binding

```bash
python3 -c "import serial_arm; print(serial_arm.__file__)"
```

输出应来自当前 workspace 的 `install/serial_arm_core/.../site-packages/serial_arm/`

### 2.7 当前已有 Robot Profile

Profile 文件位于

```text
src/robot_supports/profiles/config/robot_profiles.yaml
```

当前 DM-Arm Profile

| Profile | Core Config | MoveIt Package | Admittance Observer |
| --- | --- | --- | --- |
| `dm_arm_gray` | `config/core/gray.yaml` | `dm_arm_no_gripper` | `MOMENTUM` |
| `dm_arm_white` | `config/core/white.yaml` | `dm_arm_with_gripper` | `FULL_ID` |

以 `dm_arm_gray` 为例

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

Native C++、Python、Terminal 和 ROS 2 都可以使用这个 Profile 名称寻找对应资源

### 2.8 第一次检查硬件连接

DM-Arm Hardware Config 默认使用 `/dev/ttyACM0`

```bash
ls /dev/ttyACM*
```

如果实际设备为 `/dev/ttyACM1`，可以使用 runtime override

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1
```

还可以覆盖 baudrate 和 bus

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1 \
  --baudrate 921600 \
  --bus main_can
```

覆盖关系如下

```text
runtime override > hardware.yaml
```

运行时覆盖只影响当前进程

### 2.9 第一次配置检查

Python Terminal 提供 `--check-only`

```bash
python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray \
  --check-only
```

它适合优先检查

```text
Profile 是否可解析
Core YAML 是否存在
Hardware Backend 是否能加载
Hardware YAML 是否能解析
HardwareCapabilities 是否可读取
URDF 与 Joint 是否可解析
Safety limit 是否可建立
Dynamics 配置是否可建立
```

新机械臂第一次接入时应先保持 `write_enabled: false`

### 2.10 第一次打开 C++ Terminal

```bash
serial_arm_terminal --robot-profile dm_arm_gray
```

如果当前 Core YAML 为

```yaml
control:
  runtime:
    write_enabled: false
```

Terminal 使用离线 mock 后端，不连接串口、不使能执行器、不向真实硬件写入命令

第一次真机调试前先完成第 3 节配置检查和第 8 节 Terminal 调试流程

---

## 3 Robot Profile 与配置

### 3.1 Profile 与配置文件的关系

对于一个已有 Profile，最核心的三项是

```text
core.config
hardware.plugin
hardware.config
```

它们分别解析为

```text
Core YAML
Hardware Backend shared library
Hardware YAML
```

ROS 2 和 MoveIt 还会使用

```text
description
controllers
moveit
```

### 3.2 Core YAML 总体结构

当前 Core YAML 主要分为

```text
model
calibration
control
safety_policy
shutdown
capability
```

推荐按照这个顺序理解和调试

```text
model
  ↓
calibration
  ↓
control
  ↓
safety_policy
  ↓
shutdown
  ↓
capability
```

### 3.3 model

以 DM-Arm Gray 为例

```yaml
model:
  urdf_path: ../../model/gray/urdf/dm_arm_no_gripper.urdf
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale:
    joint1: 1.0
    joint2: 1.0
    joint3: 1.0
    joint4: 1.0
    joint5: 1.0
    joint6: 1.0
```

需要确认

- `urdf_path` 可以从当前 YAML 正确解析
- `joint_names` 与真实受控 Joint 一致
- `joint_names` 顺序就是 Core Joint Vector 顺序
- `base_frame` 和 `tool_frame` 在 URDF 中存在
- Joint axis 和 origin 正确
- position / velocity / effort limit 合理
- Dynamics 所需 inertial 参数合法

如果 inertial 仍是 placeholder，可以先检查 FK 和 Jacobian 结构，但不能把 Gravity、Mass Matrix 和 Inverse Dynamics 当作真实动力学结果

### 3.4 calibration

每个受控 Joint 都需要对应 calibration

```yaml
calibration:
  joints:
    joint1:
      direction: 1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0
```

字段含义

| 字段 | 含义 |
| --- | --- |
| `direction` | Joint 正方向与 Actuator 正方向关系，只允许 `1` 或 `-1` |
| `pos_ratio` | actuator position delta / joint position delta |
| `tor_ratio` | joint torque / actuator reported torque |
| `joint_zero_offset` | Joint 侧零位偏置 |
| `actuator_zero_offset` | Actuator 侧零位偏置 |

方向问题应先区分机械模型、Joint Mapper 和厂商协议

不要用修改 URDF axis 的方式掩盖纯执行器方向错误

不要把某台机械臂的零位补偿写死到通用 Backend

### 3.5 control.runtime

```yaml
control:
  runtime:
    ctrl_frequency_hz: 200.0
    joint_acc_filter_alpha: 0.2
    write_enabled: false
    model_feedforward_mode: NONE
    tracking_impedance_mode: RIGID_TRACKING
```

主要字段

| 字段 | 作用 |
| --- | --- |
| `ctrl_frequency_hz` | `Robot::cycle()` 目标控制频率 |
| `joint_acc_filter_alpha` | Joint acceleration 一阶滤波系数 |
| `write_enabled` | 是否允许真实 Hardware Backend 写入 |
| `model_feedforward_mode` | `NONE / GRAVITY / FULL_INVERSE_DYNAMICS` |
| `tracking_impedance_mode` | ros2_control 等跟踪入口激活后使用的跟踪阻抗模式 |

第一次接入新机械臂时建议

```yaml
write_enabled: false
model_feedforward_mode: NONE
tracking_impedance_mode: RIGID_TRACKING
```

等方向、零位、Safety 和 Dynamics 分别验证后再逐项开启

### 3.6 control.controller

Controller 为五种阻抗模式分别配置 `kp / kd`

```yaml
control:
  controller:
    allow_full_cmd: false

    rigid_hold:
      kp: {joint1: 10.0, joint2: 10.0}
      kd: {joint1: 0.1, joint2: 0.1}

    rigid_tracking:
      kp: {joint1: 10.0, joint2: 10.0}
      kd: {joint1: 0.1, joint2: 0.1}

    compliant_hold:
      kp: {joint1: 2.0, joint2: 2.0}
      kd: {joint1: 0.05, joint2: 0.05}

    compliant_drag:
      kp: {joint1: 0.0, joint2: 0.0}
      kd: {joint1: 0.05, joint2: 0.05}

    compliant_tracking:
      kp: {joint1: 2.0, joint2: 2.0}
      kd: {joint1: 0.05, joint2: 0.05}
```

示例数值只用于说明结构

新机械臂必须重新确定每个 Joint 的增益

`allow_full_cmd=false` 时，上层优先使用受约束的 `JointPosCmd`、`JointPosVelCmd` 和 `JointPosVelTorCmd`

### 3.7 safety_policy

Safety 在每个 ACTIVE 控制周期执行

```yaml
safety_policy:
  position_margin: 0.0
  cmd_vel_scale: 0.2
  state_vel_scale: 1.0

  max_acc:
    joint1: 1.0
    joint2: 1.0

  max_kp_override:
    joint1: 20.0
    joint2: 20.0

  max_kd_override:
    joint1: 0.5
    joint2: 0.5

  max_dt_s: 0.02
  state_timeout_s: 0.05
  cmd_timeout_s: 0.10
  require_all_actuators_online: true
  require_all_actuators_enabled: true
  reject_motor_error: true
  require_continuous_cmd: false
```

Safety limit 来自多个来源的交集

```text
URDF limits
    ∩
HardwareCapabilities
    ∩
Calibration Mapping
    ∩
Safety Policy
```

Safety Policy 用于继续收窄能力，不应把真实硬件能力扩大

首次真机时建议主动降低 `cmd_vel_scale` 和 `max_acc`

### 3.8 fault_recovery

FAULT 恢复配置属于 `safety_policy`

```yaml
safety_policy:
  fault_recovery:
    default_mode: rigid_hold
    allow_compliant_recovery: true
    require_operator_request: true
    gravity_model_validated: true
    recovery_timeout_s: 30.0

    compliant_recovery:
      kp: {joint1: 8.0, joint2: 20.0}
      kd: {joint1: 0.08, joint2: 0.20}
      max_vel: {joint1: 1.0, joint2: 1.0}
      effort_scale: 1.0
```

只有完成 Gravity 模型检查后才应把 `gravity_model_validated` 设为可信状态

### 3.9 shutdown

```yaml
shutdown:
  park_before_disable: true
  park_pos:
    joint1: 0.0
    joint2: 0.0
  speed_scale: 0.10
  position_tolerance: 0.05
  velocity_tolerance: 0.05
  settle_time_s: 0.25
  relaxed_tolerance_ratio: 2.0
  timeout_s: 15.0
```

`park_pos` 必须覆盖全部受控 Joint

不要默认全零位一定是安全停放姿态

### 3.10 capability.admittance

Admittance 是可选能力

如果不需要使用，可以省略该能力或设置

```yaml
capability:
  admittance:
    enabled: false
```

开启后的结构如下

```yaml
capability:
  admittance:
    enabled: true

    joint_enabled:
      joint1: true
      joint2: true

    observer:
      mode: MOMENTUM
      momentum_gain:
        joint1: 25.0
        joint2: 25.0
      filter_alpha: 0.95

    calibration:
      torque_bias:
        joint1: 0.0
        joint2: 0.0
      torque_threshold:
        joint1: 0.1
        joint2: 0.1

      friction:
        enabled: false
        velocity_transition: 0.03
        zero_velocity_adaptation_s: 0.60
        kinetic_feedforward_scale: 0.0
        positive_coulomb: {joint1: 0.0, joint2: 0.0}
        positive_viscous: {joint1: 0.0, joint2: 0.0}
        negative_coulomb: {joint1: 0.0, joint2: 0.0}
        negative_viscous: {joint1: 0.0, joint2: 0.0}

    feel:
      comfortable_torque: {joint1: 1.0, joint2: 1.0}
      follow_speed: {joint1: 0.4, joint2: 0.4}
      start_response_s: {joint1: 0.3, joint2: 0.3}
      q_elastic_start_speed: {joint1: 0.6, joint2: 0.6}
      return_time_s: {joint1: 1.0, joint2: 1.0}
      max_retreat: {joint1: 0.5, joint2: 0.5}
      max_correction_speed: {joint1: 0.8, joint2: 0.8}
      q_elastic_max_resistance_ratio: 4.0
```

`observer` 负责外力 residual 的估计方式

`calibration` 负责本机 bias、threshold 和可选 friction residual 参数

`feel` 负责用户可理解的推力、跟随速度、响应时间、回中时间和修正边界

公开 YAML 不直接填写 M / D / K

Core 根据手感参数派生

```text
D = comfortable_torque / follow_speed
M = D * start_response_s / 3
K = M * (4.74 / return_time_s)^2
```

详细工作原理和调试方法见第 7 节

### 3.11 Hardware YAML

DM-Arm 的 Hardware Config 使用 shared CAN bus 和 Damiao Backend

```yaml
buses:
  main_can:
    type: can
    backend: damiao_usb2can
    device: /dev/ttyACM0
    baudrate: 921600

damiao:
  bus: main_can
  refresh_state_in_read: false
  feedback_timeout_s: 0.05
  activation_retries: 3
  startup_read_cycles: 5
  stop_kp: 10.0
  stop_kd: 0.15
  stop_cycles: 5

  actuators:
    joint1:
      name: actuator1
      motor_id: 1
      master_id: 0
      motor_type: DM4340
```

需要确认

- `buses.main_can.device` 与实际设备一致
- bus 名称与 `damiao.bus` 一致
- actuator key 与 `model.joint_names` 一一对应
- `motor_id` 唯一且与实际电机一致
- `motor_type` 与实际电机型号一致
- HardwareCapabilities 与真实执行器能力一致

Runtime override 可以只修改当前进程的 `serial_port / baudrate / bus`

---
## 4 基础控制与生命周期

### 4.1 Robot 状态

Robot 典型状态包括

```text
UNCONFIGURED
    ↓ configure
INACTIVE
    ↓ activate
ACTIVE
    ↓ fault
FAULT
```

正常退出通常从 ACTIVE 进入停放流程，再 deactivate 回到 INACTIVE

`force_deactivate()` 用于需要立即失能的场景，不执行正常 park 流程

### 4.2 configure

Native C++ 使用 `Robot::configure()` 组装 Core Config、MotorBus 和模型回调

```cpp
Robot robot;

auto result = robot.configure(
    cfg,
    std::move(bus),
    model_feedforward,
    interaction_model_state
);
```

`model_feedforward` 在以下场景需要

```text
model_feedforward_mode != NONE
或
capability.admittance.enabled == true
```

MOMENTUM Admittance 还需要 `interaction_model_state`

### 4.3 activate

```cpp
auto result = robot.activate();
```

activate 会连接并使能 MotorBus，并使用真实状态初始化控制器和运行时状态

真机路径要求 `write_enabled=true`

### 4.4 跟踪命令

基础跟踪命令有三种常用形式

#### 只给位置

```cpp
JointPosCmd cmd;
cmd.pos = target;
robot.set_cmd(cmd);
```

#### 给位置和速度

```cpp
JointPosVelCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;
robot.set_cmd(cmd);
```

#### 给位置、速度和附加力矩

```cpp
JointPosVelTorCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;
cmd.tor = extra_torque;
robot.set_cmd(cmd);
```

如果同时启用模型前馈，附加力矩会和模型前馈一起进入 nominal command

上层已经自行加入 Gravity 时不要再次开启 Core 的 `GRAVITY`

### 4.5 Tracking 模式必须持续刷新 set_cmd

Tracking 模式受 `cmd_timeout_s` 约束

典型循环

```cpp
while(running) {
    const auto now = Robot::Clock::now();

    robot.set_cmd(
        JointPosVelCmd{target_pos, target_vel},
        now
    );

    auto output = robot.cycle(now);

    if(!output) {
        break;
    }
}
```

只发送一次命令后长期调用 `cycle()` 可能触发 command timeout

### 4.6 Hold 模式

Hold 模式不依赖持续外部目标

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_HOLD
);
```

切换 Hold 时 Core 使用当前实测位置重新建立保持参考

### 4.7 COMPLIANT_DRAG

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_DRAG
);
```

进入 `COMPLIANT_DRAG` 后不应持续发送 tracking command

```cpp
while(running) {
    auto output = robot.cycle();
    if(!output) {
        break;
    }
}
```

`COMPLIANT_DRAG` 每周期以当前实测位置作为参考，主要由低 `kp / kd` 和模型前馈形成低阻力拖拽

它和 `capability.admittance` 是两种不同机制

Robot 在 `COMPLIANT_DRAG` 下显式旁路 Admittance 位置与速度修正

### 4.8 deactivate

正常失能

```cpp
robot.deactivate();
```

最终应用应根据自身业务决定是否先执行 park trajectory

C++ Terminal 和 ros2_control Adapter 已提供基于 `shutdown` 配置的停放流程

### 4.9 FAULT

读取故障

```cpp
const auto& fault = robot.get_last_fault();
```

如果 Robot 正处于 fault hold，应继续维护

```cpp
robot.maintain_fault_hold();
```

返回 FAULT 刚性保持

```cpp
robot.return_to_fault_rigid_hold();
```

允许时进入受限柔性恢复

```cpp
robot.enter_fault_compliant_recovery();
```

满足恢复条件后

```cpp
robot.clear_fault();
```

如果目标是立即退出到失能状态

```cpp
robot.force_deactivate();
```

---

## 5 Dynamics

### 5.1 功能

Dynamics 负责从 URDF 和当前 Joint State 计算

```text
Forward Kinematics
Frame Pose
Jacobian
Gravity
Gravity Compensation
Coriolis
Mass Matrix
Inverse Dynamics
```

### 5.2 前置条件

Dynamics 依赖

```text
正确 URDF
正确 Joint 顺序
正确 Joint axis
正确 frame
合法 inertial
正确 gravity vector
```

要评价力矩精度，还需要可信的 mass、center of mass 和 inertia

### 5.3 独立使用 Dynamics

```python
import numpy as np
import serial_arm

cfg = serial_arm.load_robot_cfg(
    core_yaml,
    hardware_plugin,
    hardware_yaml,
)

dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)

n = len(cfg.joint_names)

state = serial_arm.JointState()
state.pos = np.zeros(n, dtype=np.float64)
state.vel = np.zeros(n, dtype=np.float64)
state.tor = np.zeros(n, dtype=np.float64)

acc = np.zeros(n, dtype=np.float64)
ref_acc = np.zeros(n, dtype=np.float64)

dynamics.update_state(state, acc, ref_acc)

print(dynamics.gravity)
print(dynamics.mass_matrix)
print(dynamics.tool_pose)
print(dynamics.tool_jacobian)
```

基本检查

```text
gravity 长度 = N
mass_matrix 尺寸 = N x N
tool_pose 尺寸 = 4 x 4
tool_jacobian 尺寸 = 6 x N
所有数值 finite
```

### 5.4 Model Feedforward

Robot 不直接持有具体 Dynamics 实现

上层通过 `ModelFeedforwardFn` 把模型计算注入 Robot

```cpp
ModelFeedforwardFn feedforward =
    [&dynamics, joints_count](
        ModelFeedforwardMode mode,
        const JointState& state,
        const JointVector& acc,
        const JointVector& ref_acc,
        double)
        -> tl::expected<JointVector, ModelFeedforwardErr>
    {
        auto update_result = dynamics.update(
            state,
            acc,
            ref_acc
        );

        if(!update_result) {
            return tl::make_unexpected(
                ModelFeedforwardErr::COMPUTE_FAILED
            );
        }

        switch(mode) {
            case ModelFeedforwardMode::GRAVITY:
                return dynamics.get_gravity_compensation();

            case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS:
                return dynamics.get_inverse_dynamics();

            case ModelFeedforwardMode::NONE:
                return JointVector(joints_count, 0.0);
        }

        return tl::make_unexpected(
            ModelFeedforwardErr::INVALID_MODE
        );
    };
```

### 5.5 MOMENTUM Observer 的 InteractionModelStateFn

MOMENTUM Admittance 还需要 Gravity、Coriolis 和 Mass Matrix

```cpp
InteractionModelStateFn interaction_model_state =
    [&dynamics](
        const JointState&,
        double)
        -> tl::expected<InteractionModelState, ModelFeedforwardErr>
    {
        InteractionModelState state;

        state.gravity = dynamics.get_gravity_compensation();
        state.coriolis = dynamics.get_coriolis();

        const auto& mass = dynamics.get_mass_matrix();

        state.mass_matrix.resize(
            static_cast<std::size_t>(mass.rows())
        );

        for(Eigen::Index r = 0; r < mass.rows(); ++r) {
            state.mass_matrix[static_cast<std::size_t>(r)].resize(
                static_cast<std::size_t>(mass.cols())
            );

            for(Eigen::Index c = 0; c < mass.cols(); ++c) {
                state.mass_matrix[static_cast<std::size_t>(r)]
                                 [static_cast<std::size_t>(c)] = mass(r, c);
            }
        }

        return state;
    };
```

### 5.6 Gravity Compensation

配置

```yaml
control:
  runtime:
    model_feedforward_mode: GRAVITY

model:
  gravity_scale:
    joint1: 1.0
    joint2: 1.0
```

`gravity_scale` 用于逐 Joint 缩放 Gravity Compensation

运行时可以通过 Dynamics 修改

```cpp
dynamics.set_gravity_scale({
    1.0,
    0.9,
    0.85,
    1.0,
    1.0,
    1.0,
});
```

Python RobotSession 也提供对应入口

```python
arm.set_gravity_scale(
    np.array(
        [1.0, 0.9, 0.85, 1.0, 1.0, 1.0],
        dtype=np.float64,
    )
)
```

### 5.7 Dynamics 调试顺序

推荐顺序

```text
URDF 可解析
  ↓
Joint 顺序
  ↓
FK
  ↓
Frame Pose
  ↓
Jacobian
  ↓
Gravity 方向
  ↓
逐 Joint gravity_scale
  ↓
Mass Matrix
  ↓
Coriolis
  ↓
Inverse Dynamics
```

如果 Gravity 方向错误，优先检查

- URDF joint axis
- calibration.direction
- `tor_ratio`
- 电机力矩正方向
- gravity vector
- inertial

不要先靠负 gravity scale 或大增益掩盖方向问题

### 5.8 Terminal 调试

进入

```text
动力学与配置
```

可检查

```text
完整动力学向量与末端位姿
Mass Matrix 与末端 Jacobian
指定 Frame 的缓存位姿与 Jacobian
完整配置摘要
```

进入

```text
模式与补偿
```

可以在 INACTIVE 状态切换模型前馈模式和修改 Gravity Compensation 比例

---

## 6 Impedance

### 6.1 五种模式

| 模式 | 参考位置 | 是否接受 set_cmd | 主要用途 |
| --- | --- | --- | --- |
| `RIGID_HOLD` | 切换模式时的当前实测位置 | 否 | 高刚度定点保持 |
| `RIGID_TRACKING` | 上层命令 | 是 | 刚性轨迹跟踪 |
| `COMPLIANT_HOLD` | 切换模式时的当前实测位置 | 否 | 低刚度定点保持 |
| `COMPLIANT_DRAG` | 每周期当前实测位置 | 否 | 低阻力拖拽 |
| `COMPLIANT_TRACKING` | 上层命令 | 是 | 柔性轨迹跟踪 |

五种模式不是五套独立控制器

它们主要通过参考位置生成方式和 `kp / kd` 配置形成不同语义

### 6.2 配置

```yaml
control:
  controller:
    rigid_hold:
      kp: {...}
      kd: {...}

    rigid_tracking:
      kp: {...}
      kd: {...}

    compliant_hold:
      kp: {...}
      kd: {...}

    compliant_drag:
      kp: {...}
      kd: {...}

    compliant_tracking:
      kp: {...}
      kd: {...}
```

### 6.3 切换模式

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING
);
```

切换模式会清除已有 tracking command，并重新建立相关运行时参考

正确顺序

```text
set_impedance_mode(TRACKING)
  ↓
持续 set_cmd
  ↓
持续 cycle
```

不要先发送 tracking command 再切换模式

### 6.4 COMPLIANT_DRAG 调试

初始建议使用很低的 `kp` 和适度 `kd`

```yaml
compliant_drag:
  kp:
    joint1: 0.0
  kd:
    joint1: 0.08
```

推荐顺序

```text
确认 Joint direction
  ↓
确认零位和限位
  ↓
低 kp / kd
  ↓
逐 Joint 测试
  ↓
确认 Gravity 方向
  ↓
逐 Joint 调 gravity_scale
  ↓
整臂拖拽
```

如果拖拽仍然很硬，需要同时检查机械摩擦、减速器阻力、电机内层控制器刚度和 Gravity 模型

### 6.5 COMPLIANT_TRACKING 调试

先保证 `RIGID_TRACKING` 在低速下稳定

再降低 `kp / kd` 进入 `COMPLIANT_TRACKING`

验证项目

```text
目标位置是否持续刷新
cmd_timeout 是否稳定
轨迹误差是否符合预期
速度是否满足 Safety
外部扰动后是否能继续跟踪
```

### 6.6 Terminal 调试

进入

```text
模式与补偿
  -> 切换阻抗模式
```

可直接选择

```text
RIGID_HOLD
RIGID_TRACKING
COMPLIANT_HOLD
COMPLIANT_DRAG
COMPLIANT_TRACKING
```

运动测试使用

```text
运动与命令
  -> 绝对位置移动
  -> 相对移动
  -> 取消并保持当前位置
```

首次运动建议从单关节小角度和低 `speed_scale` 开始

---

## 7 Admittance

### 7.1 功能定位

当前 Admittance 是关节空间导纳能力

它根据无力传感器的外力矩估计生成 `delta_q` 和 `delta_q_dot`，再修正 nominal joint command

控制链如下

```text
Measured Joint Torque
        ↓
Dynamics Residual
        ↓
Observer
        ↓
Bias Compensation
        ↓
Friction Compensation
        ↓
Threshold
        ↓
External Torque Estimate
        ↓
Contact Confidence
        ↓
Variable Admittance
        ↓
delta_q / delta_q_dot
        ↓
Nominal Joint Command
        ↓
Safety
        ↓
MotorBus
```

### 7.2 前置条件

启用 Admittance 前应确认

```text
Joint torque feedback 语义可信
Joint direction 正确
Torque direction 正确
URDF Dynamics 可用
Gravity 方向正确
Safety limit 正确
Observer 所需模型回调可用
```

如果基础动力学和力矩语义错误，不要直接通过 threshold 或 feel 参数补偿

### 7.3 FULL_ID Observer

FULL_ID 使用测量力矩和完整逆动力学模型力矩形成 residual

概念上可以写成

```text
tau_ext ≈ tau_measured - tau_model
```

模型误差、摩擦和 bias 会进入 residual，因此需要 calibration 和 threshold

FULL_ID 依赖 `ModelFeedforwardFn` 能够计算 `FULL_INVERSE_DYNAMICS`

### 7.4 MOMENTUM Observer

MOMENTUM Observer 使用 generalized momentum 形式估计外力

它额外需要

```text
Gravity
Coriolis
Mass Matrix
Joint Velocity
Measured Torque
```

因此 Native C++ 使用 MOMENTUM 时必须给 `Robot::configure()` 提供 `InteractionModelStateFn`

### 7.5 observer 配置

```yaml
observer:
  mode: MOMENTUM
  momentum_gain:
    joint1: 25.0
    joint2: 25.0
  filter_alpha: 0.95
```

`mode` 可使用

```text
FULL_ID
MOMENTUM
```

`momentum_gain` 只对 MOMENTUM Observer 生效

`filter_alpha` 控制 residual 一阶滤波响应

### 7.6 calibration

```yaml
calibration:
  torque_bias:
    joint1: 0.0
    joint2: 0.0

  torque_threshold:
    joint1: 0.1
    joint2: 0.1
```

`torque_bias` 用于补偿无接触 residual baseline

`torque_threshold` 用于抑制无接触噪声和小 residual

这两组参数必须针对当前机械臂本机标定

不要从其他机械臂直接复制

### 7.7 friction

可选摩擦 residual 模型

```yaml
friction:
  enabled: false
  velocity_transition: 0.03
  zero_velocity_adaptation_s: 0.60
  kinetic_feedforward_scale: 0.0
  positive_coulomb: {...}
  positive_viscous: {...}
  negative_coulomb: {...}
  negative_viscous: {...}
```

`enabled=false` 时整条摩擦补偿链旁路

`kinetic_feedforward_scale` 用于滑动摩擦主动助力，默认可以保持 `0`

### 7.8 feel

公开配置使用语义化手感参数

| 参数 | 含义 |
| --- | --- |
| `comfortable_torque` | 用户可舒适持续施加的外力矩目标 |
| `follow_speed` | 该力矩对应的期望跟随速度 |
| `start_response_s` | 起步达到约 95% 跟随速度的目标时间 |
| `q_elastic_start_speed` | 开始增加阻尼形成 Q 弹阻力的修正速度 |
| `return_time_s` | 松手后约 95% 回中的目标时间 |
| `max_retreat` | 最大导纳位置偏移 |
| `max_correction_speed` | 最大导纳修正速度 |
| `q_elastic_max_resistance_ratio` | 高速区最大阻尼倍率 |

派生关系

```text
D_follow = comfortable_torque / follow_speed
M = D_follow * start_response_s / 3
K_return = M * (4.74 / return_time_s)^2
```

接触时系统逐渐降低有效回中刚度

松手后系统逐渐恢复回中刚度和临界阻尼

因此当前行为不是固定 M / D / K 的单一二阶系统

### 7.9 参数约束

至少满足

```text
comfortable_torque > 0
follow_speed > 0
start_response_s > 0
return_time_s > 0
max_retreat > 0
max_correction_speed > 0
q_elastic_start_speed < max_correction_speed
q_elastic_max_resistance_ratio >= 1
```

### 7.10 Admittance 与 Impedance 的关系

Admittance 修正发生在 nominal command 之后、Safety 之前

```text
Impedance / Tracking
        ↓
Nominal Command
        ↓
Admittance Correction
        ↓
Safety
```

`COMPLIANT_DRAG` 是例外

进入 `COMPLIANT_DRAG` 时 Robot 显式旁路 Admittance 修正

不要用 `COMPLIANT_DRAG` 的手感判断 Admittance 是否工作

### 7.11 Runtime 配置

C++ 可以读取和更新 Admittance Config

```cpp
const auto& cfg = robot.get_admittance_cfg();
```

更新当前运行时配置

```cpp
auto candidate = robot.get_admittance_cfg();
candidate.feel.follow_speed[0] = 0.5;

robot.set_admittance_cfg(candidate);
```

临时挂起

```cpp
robot.set_admittance_suspended(true);
```

恢复

```cpp
robot.set_admittance_suspended(false);
```

检查状态

```cpp
bool suspended = robot.is_admittance_suspended();
```

Runtime 修改只作用于当前进程

是否持久化到 YAML 由上层明确处理

### 7.12 RobotCycleOutput Telemetry

Admittance 参与周期时可以读取

```text
admittance_active
full_id_residual_raw
observer_residual_raw
observer_residual_filtered
bias_compensated
friction_residual_hat
friction_compensated
tau_ext_hat
contact_confidence
delta_q
delta_q_dot
effective_damping
effective_stiffness
friction_feedforward
torque_threshold_active
delta_q_limited
delta_q_dot_limited
safety_position_margin_active
safety_velocity_margin_active
```

调试时推荐按下面顺序观察

```text
residual_raw
  ↓
residual_filtered
  ↓
bias_compensated
  ↓
friction_compensated
  ↓
tau_ext_hat
  ↓
contact_confidence
  ↓
D_eff / K_eff
  ↓
delta_q / delta_q_dot
  ↓
Safety flags
```

### 7.13 Terminal 标定

进入

```text
调参与测试
  -> 导纳控制
     -> 导纳参数标定
```

标定流程会引导完成静态 residual 和 DRAG 相关数据采集

标定完成后参数先应用到当前进程

需要写回的参数可以在诊断菜单查看

### 7.14 Terminal 手感设置

进入

```text
调参与测试
  -> 导纳控制
     -> 手感设置
```

当前菜单提供

```text
跟手阻力
起步响应
Q弹起始速度
回中速度
最大退让
高级参数
```

高级参数可以查看和修改

```text
派生 M / D / K
Observer filter_alpha
最大导纳修正速度
Q弹最大阻力倍率
摩擦主动助力比例
当前可写回参数
```

### 7.15 Terminal 状态与诊断

进入

```text
调参与测试
  -> 导纳控制
     -> 状态与诊断
```

可使用

```text
状态摘要
实时观测
标定与 Observer 详情
```

实时观测会输出外力估计、接触置信度、导纳位移、导纳速度、有效阻尼、有效刚度和限幅标志

### 7.16 Admittance 推荐调试顺序

```text
1 Joint torque direction
2 Dynamics gravity
3 FULL_ID residual
4 Observer residual
5 torque_bias
6 torque_threshold
7 contact_confidence
8 单 Joint max_retreat
9 单 Joint follow feel
10 return feel
11 q_elastic
12 多 Joint
13 Safety boundary
14 deactivate / activate
15 suspend / resume
```

如果无接触时 `tau_ext_hat` 长期不为零，先检查 bias 和模型误差

如果接触很迟钝，先检查 residual、threshold 和 filter，再调 feel

如果一推动就长期顶到 `max_retreat`，先确认无接触 residual 是否已经清零，再检查 follow feel

如果位置边界触发，应同时观察 `delta_q_limited` 和 `delta_q_dot_limited`

---

## 8 C++ Terminal 使用与调试

### 8.1 启动方式

使用 Robot Profile

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray
```

指定 Profile 文件

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --profile-file <robot_profiles.yaml>
```

不使用 Profile

```bash
serial_arm_terminal \
  --config <core.yaml> \
  --hardware-plugin <backend.so> \
  --hardware-config <hardware.yaml>
```

### 8.2 CLI 参数

| 参数 | 作用 |
| --- | --- |
| `--robot-profile <name>` | 选择 Robot Profile |
| `--profile-file <path>` | 指定 Profile 文件 |
| `--config <path>` | 指定 Core YAML |
| `--hardware-plugin <name>` | 指定 Hardware Backend |
| `--hardware-config <path>` | 指定 Hardware YAML |
| `--serial-port <path>` | 覆盖串口设备 |
| `--baudrate <n>` | 覆盖 baudrate |
| `--bus <name>` | 覆盖 Hardware bus |
| `--compare-config <a> <b>` | 在同一 HardwareCapabilities 下比较两份 Core Config |
| `--help` / `-h` | 查看帮助 |

### 8.3 主菜单

```text
1 状态查看
2 使能 / 失能 / 故障
3 模式与补偿
4 运动与命令
5 动力学与配置
6 调参与测试
0 回到停放姿态并安全退出
```

### 8.4 状态查看

可查看

```text
Robot 状态与 getter 输出
全部 Joint / Actuator 周期状态
达妙执行器静态参数
完整配置摘要
```

第一次连接硬件时优先检查这一组

### 8.5 使能 / 失能 / 故障

可执行

```text
activate
回到停放姿态并失能
立即停止并失能
clear_fault
FAULT 进入受限柔性恢复
FAULT 返回刚性保持
查看当前故障恢复模式
```

正常退出优先使用 park + deactivate 路径

立即停止并失能只用于明确需要跳过正常停放的场景

### 8.6 模式与补偿

可执行

```text
切换阻抗模式
切换模型前馈模式
设置 Gravity Compensation 比例
```

模型前馈和 Gravity Scale 修改要求 Robot 处于适合修改的状态

### 8.7 运动与命令

可执行

```text
梯形参考移动到绝对 Joint Position
梯形参考执行相对 Joint Move
取消当前输入并切换到当前位置保持
```

第一次真机运动建议采用小角度和低 speed scale

### 8.8 动力学与配置

可查看

```text
完整 Dynamics Vector
Tool Pose
Mass Matrix
Tool Jacobian
指定 Frame Pose
指定 Frame Jacobian
完整配置摘要
```

### 8.9 调参与测试

当前主要入口为 Admittance

```text
导纳参数标定
手感设置
状态与诊断
```

### 8.10 推荐 Terminal 真机顺序

```text
write_enabled=false
  ↓
Profile / Config
  ↓
状态查看
  ↓
Dynamics
  ↓
Joint direction / zero / limit
  ↓
write_enabled=true
  ↓
activate
  ↓
单 Joint 小幅运动
  ↓
RIGID_HOLD
  ↓
RIGID_TRACKING
  ↓
COMPLIANT_HOLD
  ↓
COMPLIANT_DRAG
  ↓
Gravity
  ↓
COMPLIANT_TRACKING
  ↓
Admittance
  ↓
park + deactivate
```

---
## 9 上层 Adapter 与应用接入

### 9.1 设计原则

上层框架只负责把自己的输入输出转换为 Core 语义

推荐边界如下

```text
Application
    ↓
Adapter
    ↓
Robot
    ↓
Controller / Dynamics / Interaction / Safety
    ↓
MotorBus
```

Adapter 应负责

```text
生命周期映射
命令映射
状态映射
线程与更新频率
错误传播
运行时覆盖参数
上层框架单位和 Core 单位之间的转换
```

Adapter 不应重复实现

```text
Joint / Actuator Mapping
Safety
Impedance
Admittance
Dynamics residual
FAULT hold
```

### 9.2 Native C++

Native C++ 是最完整的 Core 接口

适合需要直接控制生命周期、Dynamics 回调、Admittance Runtime Config 和完整 Telemetry 的应用

典型结构

```cpp
Robot robot;
Dynamics dynamics;
HardwareLoader loader;

// 1 load profile
// 2 load backend
// 3 load RobotCfg
// 4 configure Dynamics
// 5 create model callbacks
// 6 configure Robot
// 7 activate
// 8 set_cmd + cycle
// 9 process RobotCycleOutput
// 10 deactivate
```

外部项目 CMake

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

### 9.3 Native C++ 完整 configure 结构

```cpp
auto profile_result = load_robot_profile_core("dm_arm_gray");
if(!profile_result) {
    return 1;
}

const auto profile = profile_result.value();

HardwareLoader hardware_loader;

auto bus_result = hardware_loader.load(
    profile.hardware_plugin,
    profile.hardware_config_path
);

if(!bus_result) {
    return 1;
}

auto bus = std::move(bus_result.value());

auto cfg_result = load_robot_cfg(
    profile.core_config_path,
    bus->capabilities()
);

if(!cfg_result) {
    return 1;
}

RobotCfg cfg = cfg_result.value();

Dynamics dynamics;
if(!dynamics.configure(cfg.dynamics)) {
    return 1;
}

// 创建 ModelFeedforwardFn
// MOMENTUM Admittance 还要创建 InteractionModelStateFn

Robot robot;

auto configured = robot.configure(
    cfg,
    std::move(bus),
    model_feedforward,
    interaction_model_state
);

if(!configured) {
    return 1;
}
```

### 9.4 Python RobotSession

`RobotSession` 适合 Python 任务层、数据采集和快速实验

创建会话

```python
import serial_arm

arm = serial_arm.RobotSession(
    core_yaml,
    hardware_plugin,
    hardware_yaml,
)
```

启动

```python
arm.start()
```

切换模式

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.RIGID_TRACKING
)
```

移动

```python
arm.move_to(
    target,
    speed_scale=0.15,
)
```

保持当前位置

```python
arm.hold_current()
```

柔性拖拽

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.COMPLIANT_DRAG
)
```

读取状态

```python
snapshot = arm.snapshot

if snapshot.valid:
    print(snapshot.cycle.joint_state.pos)
```

安全退出

```python
arm.stop()
```

推荐使用上下文管理器

```python
with serial_arm.RobotSession(
    core_yaml,
    hardware_plugin,
    hardware_yaml,
) as arm:
    arm.set_impedance_mode(
        serial_arm.JointImpedanceMode.COMPLIANT_DRAG
    )

    print(arm.snapshot.cycle.joint_state.pos)
```

### 9.5 RobotSession 与 write_enabled

当 Core YAML 为

```yaml
control:
  runtime:
    write_enabled: false
```

RobotSession 使用离线 MockMotorBus 完成控制链验证

当配置为

```yaml
write_enabled: true
```

Session 才使用真实 Hardware Backend

因此 Python 调试同样应先完成离线配置检查

### 9.6 Python 当前能力边界

Core YAML 中的 Admittance Config 会参与 Robot 配置

但 Python `RobotCfg` 当前没有暴露完整 `capability.admittance` 逐项编辑接口，Python snapshot 也不覆盖全部 C++ Admittance Telemetry

需要完整 Admittance Runtime 调参、Observer 诊断和全部 Telemetry 时优先使用 Native C++ 或 C++ Terminal

### 9.7 ros2_control Adapter

`serial_arm_ros2_control` 提供 `hardware_interface::SystemInterface`

ROS 2 侧数据流

```text
JointTrajectoryController
        ↓
position / velocity command interfaces
        ↓
SerialArmSystem
        ↓
Robot::set_cmd
        ↓
Robot::cycle
        ↓
position / velocity / effort state interfaces
```

当前命令接口

```text
position
velocity
```

当前状态接口

```text
position
velocity
effort
```

### 9.8 ros2_control Profile 资源

要使用 ros2_control，Profile 需要额外提供

```yaml
description:
  package: my_arm_description
  urdf: model/default/urdf/my_arm.urdf
  ros2_control_xacro: model/default/urdf/my_arm.ros2_control.xacro

controllers:
  package: my_arm_description
  config: config/ros2_controllers.yaml
```

`SerialArmSystem` 会校验 ros2_control Joint 顺序与 `model.joint_names` 一致

每个 Joint 当前要求

```text
command interfaces
  position
  velocity

state interfaces
  position
  velocity
  effort
```

### 9.9 display.launch.py

只检查模型

```bash
ros2 launch serial_arm_ros2_control \
  display.launch.py \
  robot_profile:=dm_arm_gray
```

它适合检查

```text
URDF
Mesh
TF
Joint axis
base frame
tool frame
```

该入口不连接 Hardware Backend

### 9.10 hardware.launch.py

启动真实 ros2_control 链路

```bash
ros2 launch serial_arm_ros2_control \
  hardware.launch.py \
  robot_profile:=dm_arm_white
```

可覆盖硬件连接

```bash
ros2 launch serial_arm_ros2_control \
  hardware.launch.py \
  robot_profile:=dm_arm_white \
  serial_port:=/dev/ttyACM1 \
  baudrate:=921600 \
  bus:=main_can
```

启动后检查

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

当前 launch 会启动

```text
robot_state_publisher
ros2_control_node
joint_state_broadcaster
joint_trajectory_controller
```

### 9.11 tracking_impedance_mode

ros2_control 激活 Robot 后会读取

```yaml
control:
  runtime:
    tracking_impedance_mode: RIGID_TRACKING
```

或者

```yaml
tracking_impedance_mode: COMPLIANT_TRACKING
```

这个字段决定 ros2_control tracking command 最终使用哪套 Core `kp / kd`

如果需要严格轨迹跟踪，优先从 `RIGID_TRACKING` 开始

如果需要低刚度轨迹跟踪，再验证 `COMPLIANT_TRACKING`

### 9.12 ros2_control 与 Admittance 当前边界

当前 `SerialArmSystem` 给 `Robot::configure()` 提供 `ModelFeedforwardFn`

当前 Adapter 没有提供 MOMENTUM Observer 所需的 `InteractionModelStateFn`

因此

```text
FULL_ID Admittance
  当前 ros2_control Adapter 可以满足模型回调要求

MOMENTUM Admittance
  当前 ros2_control Adapter 缺少 InteractionModelStateFn
```

`dm_arm_white` 当前使用 FULL_ID Observer

`dm_arm_gray` 当前使用 MOMENTUM Observer

因此 `dm_arm_gray` 的 MOMENTUM Admittance 需要 Native C++ / C++ Terminal，或者扩展 ros2_control Adapter 后再通过该路径使用

这属于当前接口能力边界，不应在上层通过复制 Observer 算法绕过

### 9.13 MoveIt 2 Adapter 链

MoveIt 不直接控制 MotorBus

当前路径

```text
MoveIt
  ↓
move_group
  ↓
JointTrajectoryController
  ↓
ros2_control
  ↓
SerialArmSystem
  ↓
Robot
  ↓
MotorBus
```

Profile 需要

```yaml
moveit:
  package: my_arm_moveit_config
```

现有 `dm_arm_white` 可以作为当前 MoveIt 使用示例

```bash
ros2 launch serial_arm_ros2_control \
  moveit.launch.py \
  robot_profile:=dm_arm_white
```

当前顶层入口会包含

```text
hardware.launch.py
MoveIt move_group.launch.py
MoveIt moveit_rviz.launch.py
```

串口覆盖同样可以透传

```bash
ros2 launch serial_arm_ros2_control \
  moveit.launch.py \
  robot_profile:=dm_arm_white \
  serial_port:=/dev/ttyACM1
```

### 9.14 MoveIt 推荐使用顺序

```text
Core tests
  ↓
Profile / Config
  ↓
URDF / Dynamics
  ↓
Terminal 单 Joint
  ↓
RIGID_TRACKING
  ↓
display.launch.py
  ↓
hardware.launch.py
  ↓
joint_trajectory_controller
  ↓
moveit.launch.py
```

不要用 MoveIt 来替代 Joint direction、zero、Safety limit 和 Hardware Backend 验证

### 9.15 自定义 Adapter

自定义上层框架可以复用下面结构

```text
Framework lifecycle
    ↓
Robot configure / activate / deactivate

Framework command
    ↓
JointPosCmd / JointPosVelCmd / JointPosVelTorCmd

Framework update loop
    ↓
Robot::cycle

RobotCycleOutput
    ↓
Framework state / telemetry
```

如果上层需要 MOMENTUM Admittance，Adapter 必须同时提供

```text
ModelFeedforwardFn
InteractionModelStateFn
```

如果上层需要运行时调节 Admittance，应直接调用 Core 的

```text
get_admittance_cfg
set_admittance_cfg
set_admittance_suspended
is_admittance_suspended
```

不要在 Adapter 中建立第二套 M / D / K 状态

---

## 10 接入新的机械臂

### 10.1 总体原则

新增机械臂时优先复用 Core、Protocol 和已有 Hardware Backend

机械结构变化通常不需要修改 `serial_arm/core`

先判断变化在哪一层

| 场景 | 主要工作 |
| --- | --- |
| 同一机械臂新增结构 Variant | Robot Support + Core YAML + Profile |
| 新机械臂继续使用 Damiao | Robot Support + Hardware YAML + Profile |
| 新机械臂使用已有其他 Backend | Robot Support + 对应 Hardware YAML + Profile |
| 执行器厂商协议变化 | 新 Hardware Backend + Robot Support + Profile |
| 通信设备私有协议变化 | 新 Protocol Adapter，必要时新 Backend |
| 只增加 ROS 2 | ros2_control Xacro + Controllers + Profile ROS 字段 |
| 只增加 MoveIt | MoveIt Config + Profile `moveit` 字段 |

推荐按下面阶段完成

```text
Stage A Robot Model / Profile
Stage B Hardware / Mapping
Stage C Basic Control / Safety
Stage D Dynamics
Stage E Impedance
Stage F Admittance
Stage G Adapter / MoveIt
```

### 10.2 Stage A 确定命名与 Joint 顺序

先固定

```text
robot_name          my_arm
resource package    my_arm_description
profile name        my_arm_default
variant             default
base frame          base_link
tool frame          tool0
joint order         [joint1, joint2, ...]
hardware backend    serial_arm_hardware_damiao
```

最重要的是 `joint order`

下面内容必须围绕同一组受控 Joint

```text
URDF Joint
model.joint_names
calibration.joints
controller gains
Safety per-joint parameters
shutdown.park_pos
capability.admittance per-joint parameters
Hardware actuator mapping
ros2_control joints
controller joints
```

### 10.3 创建 Robot Support

推荐目录

```text
src/robot_supports/robots/my_arm/
└── description/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── core/
    │   │   └── default.yaml
    │   ├── hardware.yaml
    │   └── ros2_controllers.yaml
    └── model/
        └── default/
            ├── meshes/
            └── urdf/
                ├── my_arm.urdf
                └── my_arm.ros2_control.xacro
```

需要 MoveIt 时再增加独立 MoveIt Config Package

### 10.4 description package

最小 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_arm_description)

find_package(ament_cmake QUIET)

install(DIRECTORY config model
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
  ament_package()
endif()
```

这种资源包同时可以被 standalone CMake install 和 colcon 使用

### 10.5 准备 URDF

先完成机械模型，不要一开始就接 MoveIt

至少确认

```text
Joint name
parent / child
origin
axis
joint type
position limit
velocity limit
effort limit
base frame
tool frame
inertial
```

固定 Joint 不加入 `model.joint_names`

不要在 URDF 中写机器相关的绝对路径

### 10.6 建立 Core YAML

新机械臂建议复制现有 YAML 的结构，不复制具体参数值

初始结构

```yaml
model:
  urdf_path: ../../model/default/urdf/my_arm.urdf
  joint_names: [joint1, joint2]
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale:
    joint1: 0.0
    joint2: 0.0

calibration:
  joints:
    joint1:
      direction: 1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0
    joint2:
      direction: 1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0

control:
  runtime:
    ctrl_frequency_hz: 200.0
    joint_acc_filter_alpha: 0.2
    write_enabled: false
    model_feedforward_mode: NONE
    tracking_impedance_mode: RIGID_TRACKING

  controller:
    allow_full_cmd: false
    rigid_hold:
      kp: {joint1: 5.0, joint2: 5.0}
      kd: {joint1: 0.1, joint2: 0.1}
    rigid_tracking:
      kp: {joint1: 5.0, joint2: 5.0}
      kd: {joint1: 0.1, joint2: 0.1}
    compliant_hold:
      kp: {joint1: 1.0, joint2: 1.0}
      kd: {joint1: 0.05, joint2: 0.05}
    compliant_drag:
      kp: {joint1: 0.0, joint2: 0.0}
      kd: {joint1: 0.05, joint2: 0.05}
    compliant_tracking:
      kp: {joint1: 1.0, joint2: 1.0}
      kd: {joint1: 0.05, joint2: 0.05}

safety_policy:
  position_margin: 0.0
  cmd_vel_scale: 0.2
  state_vel_scale: 0.5
  max_acc: {joint1: 1.0, joint2: 1.0}
  max_kp_override: {joint1: 10.0, joint2: 10.0}
  max_kd_override: {joint1: 0.5, joint2: 0.5}
  max_dt_s: 0.02
  state_timeout_s: 0.05
  cmd_timeout_s: 0.10
  require_all_actuators_online: true
  require_all_actuators_enabled: true
  reject_motor_error: true
  require_continuous_cmd: false

shutdown:
  park_before_disable: true
  park_pos: {joint1: 0.0, joint2: 0.0}
  speed_scale: 0.10
  position_tolerance: 0.05
  velocity_tolerance: 0.05
  settle_time_s: 0.25
  relaxed_tolerance_ratio: 2.0
  timeout_s: 15.0
```

上面所有控制增益和 Safety 数值只表示接入时的保守结构示例

必须根据真实机械臂重新确定

### 10.7 Stage B 创建 Hardware YAML

如果继续使用现有 Damiao Backend，只需要创建新的 Hardware instance config

```yaml
buses:
  main_can:
    type: can
    backend: damiao_usb2can
    device: /dev/ttyACM0
    baudrate: 921600

damiao:
  bus: main_can
  refresh_state_in_read: false
  feedback_timeout_s: 0.05
  activation_retries: 3
  startup_read_cycles: 5
  stop_kp: 3.0
  stop_kd: 0.1
  stop_cycles: 5

  actuators:
    joint1:
      name: actuator1
      motor_id: 1
      master_id: 0
      motor_type: DM4340

    joint2:
      name: actuator2
      motor_id: 2
      master_id: 0
      motor_type: DM4310
```

检查

```text
actuator 数量 = joint_names 数量
actuator key 与 Joint 名称一致
motor_id 唯一
motor_type 正确
反馈单位正确
Torque sign 正确
HardwareCapabilities 正确
```

### 10.8 注册 Profile

在

```text
src/robot_supports/profiles/config/robot_profiles.yaml
```

注册最小 Profile

```yaml
profiles:
  my_arm_default:
    core:
      package: my_arm_description
      config: config/core/default.yaml

    hardware:
      plugin: serial_arm_hardware_damiao
      config_package: my_arm_description
      config: config/hardware.yaml
```

这一步完成后 Native C++、Python 和 C++ Terminal 已经可以通过 Profile 名称解析机器人

### 10.9 构建新 Robot Support

colcon 工作区

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Standalone 时按照第 2.4 节方式安装新的 description package 和 Profiles

确认资源已安装

```bash
find install/my_arm_description/share/my_arm_description \
  -maxdepth 4 \
  -type f
```

### 10.10 离线 Profile 检查

保持

```yaml
write_enabled: false
```

执行

```bash
python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile my_arm_default \
  --check-only
```

必须先处理所有 Profile、Config、URDF、Capabilities 和 Dynamics 解析问题

### 10.11 Stage B Joint / Actuator Mapping 调试

第一次连接硬件只验证状态

逐 Joint 检查

```text
实际正方向
  ↔ calibration.direction

执行器位置比例
  ↔ pos_ratio

执行器力矩比例
  ↔ tor_ratio

机械零位
  ↔ joint_zero_offset / actuator_zero_offset
```

需要记录每个 Joint 的

```text
command sign
measured position sign
measured velocity sign
measured torque sign
mechanical zero
software zero
```

### 10.12 Stage C Safety 与基础真机控制

完成硬件状态检查后才设置

```yaml
write_enabled: true
```

首次真机前确认

```text
急停可用
机械臂已固定或支撑
Joint direction 正确
Joint zero 正确
URDF limits 正确
HardwareCapabilities 正确
Safety 主动收窄
park pose 安全
```

第一次运动建议

```text
0.02 rad
0.05 rad
0.10 rad
```

按顺序测试

```text
读取状态
  ↓
单 Joint 极小幅运动
  ↓
停止
  ↓
RIGID_HOLD
  ↓
低速 RIGID_TRACKING
  ↓
park + deactivate
```

发现方向错误立即回到 calibration 和 Backend 语义检查

### 10.13 Stage D Dynamics 配置

先完成

```text
FK
Jacobian
Frame Pose
```

再验证动力学数值

```text
Gravity
Mass Matrix
Coriolis
Inverse Dynamics
```

Gravity Compensation 调试从单 Joint 和低 scale 开始

```yaml
model:
  gravity_scale:
    joint1: 0.2
    joint2: 0.2
```

确认方向后再逐步提高

### 10.14 Stage E Impedance 配置与验收

每种模式都从低增益开始

推荐顺序

```text
RIGID_HOLD
  ↓
RIGID_TRACKING
  ↓
COMPLIANT_HOLD
  ↓
COMPLIANT_DRAG
  ↓
COMPLIANT_TRACKING
```

验收项目

```text
Hold 无明显振荡
Tracking 无持续超调
Command timeout 正常
COMPLIANT_HOLD 可稳定保持
COMPLIANT_DRAG 无明显位置拉回
COMPLIANT_TRACKING 可以持续刷新轨迹
```

### 10.15 Stage F Admittance 配置

不要在基础控制和 Dynamics 未通过时开启

先添加结构

```yaml
capability:
  admittance:
    enabled: true
    joint_enabled:
      joint1: true
      joint2: true

    observer:
      mode: FULL_ID
      momentum_gain:
        joint1: 25.0
        joint2: 25.0
      filter_alpha: 0.1

    calibration:
      torque_bias:
        joint1: 0.0
        joint2: 0.0
      torque_threshold:
        joint1: 0.1
        joint2: 0.1
      friction:
        enabled: false
        velocity_transition: 0.03
        zero_velocity_adaptation_s: 0.60
        kinetic_feedforward_scale: 0.0
        positive_coulomb: {joint1: 0.0, joint2: 0.0}
        positive_viscous: {joint1: 0.0, joint2: 0.0}
        negative_coulomb: {joint1: 0.0, joint2: 0.0}
        negative_viscous: {joint1: 0.0, joint2: 0.0}

    feel:
      comfortable_torque: {joint1: 1.0, joint2: 1.0}
      follow_speed: {joint1: 0.3, joint2: 0.3}
      start_response_s: {joint1: 0.4, joint2: 0.4}
      q_elastic_start_speed: {joint1: 0.5, joint2: 0.5}
      return_time_s: {joint1: 1.2, joint2: 1.2}
      max_retreat: {joint1: 0.2, joint2: 0.2}
      max_correction_speed: {joint1: 0.7, joint2: 0.7}
      q_elastic_max_resistance_ratio: 4.0
```

上面的数值只用于提供保守结构示例

实际使用必须通过本机标定和手感调试确定

### 10.16 选择 Admittance Observer

新机械臂可以先用 FULL_ID 检查 residual

```text
measured torque
  ↓
full inverse dynamics
  ↓
full_id_residual_raw
```

如果需要 MOMENTUM，再确认 Adapter 可以提供

```text
Gravity
Coriolis
Mass Matrix
```

Native C++ 和 C++ Terminal 可以提供完整 MOMENTUM 模型链

自定义 Adapter 也必须显式实现该回调

### 10.17 Admittance 标定

使用 C++ Terminal

```bash
serial_arm_terminal \
  --robot-profile my_arm_default
```

进入

```text
调参与测试
  -> 导纳控制
     -> 导纳参数标定
```

标定完成后重点检查

```text
torque_bias
torque_threshold
Observer residual
摩擦交叉验证结果
```

不要先调 feel 再处理无接触 residual

### 10.18 Admittance 手感调试

先单 Joint

推荐顺序

```text
comfortable_torque
  ↓
follow_speed
  ↓
start_response_s
  ↓
return_time_s
  ↓
max_retreat
  ↓
q_elastic_start_speed
  ↓
max_correction_speed
  ↓
q_elastic_max_resistance_ratio
```

实时观察

```text
tau_ext_hat
contact_confidence
delta_q
delta_q_dot
D_eff
K_eff
limit flags
```

### 10.19 Stage G 增加 ros2_control

新机械臂基础 Core 能力通过后，再增加 `my_arm.ros2_control.xacro`

最小结构

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_arm">
  <xacro:arg name="config_file" default=""/>
  <xacro:arg name="hardware_plugin" default=""/>
  <xacro:arg name="hardware_config" default=""/>
  <xacro:arg name="serial_port" default=""/>
  <xacro:arg name="baudrate" default=""/>
  <xacro:arg name="bus" default=""/>

  <xacro:include filename="$(find my_arm_description)/model/default/urdf/my_arm.urdf"/>
  <xacro:include filename="$(find serial_arm_ros2_control)/urdf/serial_arm_system.ros2_control.xacro"/>

  <ros2_control name="my_arm" type="system">
    <xacro:serial_arm_ros2_control_hardware
      config_file="$(arg config_file)"
      hardware_plugin="$(arg hardware_plugin)"
      hardware_config="$(arg hardware_config)"
      serial_port="$(arg serial_port)"
      baudrate="$(arg baudrate)"
      bus="$(arg bus)"/>

    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
</robot>
```

其余 Joint 按相同接口声明

### 10.20 ros2_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    command_interfaces:
      - position
      - velocity
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
    open_loop_control: false
```

`update_rate` 建议与 `control.runtime.ctrl_frequency_hz` 保持一致

### 10.21 为 Profile 增加 ROS 字段

```yaml
profiles:
  my_arm_default:
    core:
      package: my_arm_description
      config: config/core/default.yaml

    hardware:
      plugin: serial_arm_hardware_damiao
      config_package: my_arm_description
      config: config/hardware.yaml

    description:
      package: my_arm_description
      urdf: model/default/urdf/my_arm.urdf
      ros2_control_xacro: model/default/urdf/my_arm.ros2_control.xacro

    controllers:
      package: my_arm_description
      config: config/ros2_controllers.yaml
```

重新构建后先运行

```bash
ros2 launch serial_arm_ros2_control display.launch.py \
  robot_profile:=my_arm_default
```

再运行

```bash
ros2 launch serial_arm_ros2_control hardware.launch.py \
  robot_profile:=my_arm_default
```

检查

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

### 10.22 MoveIt 最后接入

先完成

```text
Core Config
Hardware Backend
Terminal
Basic Control
Dynamics
Impedance
ros2_control
```

再创建 MoveIt Config Package

给 Profile 增加

```yaml
moveit:
  package: my_arm_moveit_config
```

启动

```bash
ros2 launch serial_arm_ros2_control moveit.launch.py \
  robot_profile:=my_arm_default
```

MoveIt 验收

```text
Planning Group 正确
Planning Scene 模型正确
Joint Limits 正确
Trajectory Controller 匹配
规划 Joint 方向与真实机械臂一致
规划后真实执行路径符合预期
```

### 10.23 新机械臂接入完成判据

基础能力

```text
[ ] description resource package 可以安装
[ ] Robot Profile 可以解析
[ ] URDF Joint 与 Core joint_names 完全一致
[ ] Hardware YAML 可以被 Backend 加载
[ ] HardwareCapabilities 与真实执行器一致
[ ] Python Terminal --check-only 通过
[ ] Joint / Actuator direction 正确
[ ] Joint zero 正确
[ ] Safety limits 正确
[ ] 单 Joint 低速真机测试通过
[ ] RIGID_HOLD 通过
[ ] RIGID_TRACKING 通过
[ ] shutdown / park 通过
```

Dynamics

```text
[ ] FK 正确
[ ] Jacobian 正确
[ ] Gravity 方向正确
[ ] gravity_scale 已验证
[ ] Mass Matrix 数值有限
[ ] Inverse Dynamics 数值有限
```

Impedance

```text
[ ] COMPLIANT_HOLD 通过
[ ] COMPLIANT_DRAG 通过
[ ] COMPLIANT_TRACKING 通过
```

Admittance

```text
[ ] Torque feedback 语义可信
[ ] Observer residual 可解释
[ ] torque_bias 已本机标定
[ ] torque_threshold 已本机标定
[ ] 无接触 tau_ext_hat 接近零
[ ] 单 Joint follow feel 通过
[ ] return feel 通过
[ ] max_retreat / max_correction_speed 通过
[ ] Safety boundary 通过
[ ] deactivate / activate 状态清理通过
[ ] suspend / resume 通过
```

ROS 2

```text
[ ] display.launch.py 正确
[ ] ros2_control interfaces 正确
[ ] joint_state_broadcaster 正常
[ ] joint_trajectory_controller 正常
```

MoveIt

```text
[ ] move_group 正常
[ ] RViz Planning Scene 正常
[ ] Planning Group 正确
[ ] 规划轨迹和真实 Joint 方向一致
```

### 10.24 新 Hardware Backend

只有执行器厂商协议或硬件控制语义发生变化时才需要新增 Backend

Backend 实现 `MotorBus`

```cpp
class MyMotorBus final : public serial_arm::MotorBus {
public:
    tl::expected<void, MotorBusErr>
    configure(const std::string& config_path) override;

    tl::expected<void, MotorBusErr>
    connect() override;

    tl::expected<ActuatorState, MotorBusErr>
    read() override;

    tl::expected<void, MotorBusErr>
    activate() override;

    tl::expected<void, MotorBusErr>
    write(const ActuatorCtrlCmd& cmd) override;

    tl::expected<void, MotorBusErr>
    stop() override;

    tl::expected<void, MotorBusErr>
    deactivate() override;

    tl::expected<void, MotorBusErr>
    recover() override;

    const HardwareCapabilities&
    capabilities() const noexcept override;

    void cleanup() noexcept override;

    std::size_t size() const noexcept override;
};
```

Backend 对 Core 输出统一状态单位

```text
position rad
velocity rad/s
torque N*m
```

Backend 接收统一控制语义

```text
position
velocity
torque
kp
kd
```

执行器厂商私有单位应在 Backend 内转换

不要让厂商单位进入 Core 或上层 Adapter

---
## 11 Safety 与 FAULT 调试

### 11.1 Safety 的位置

Safety 不是最后才执行的附加保护

它参与 ACTIVE 控制周期中的最终命令检查

```text
Upstream Command
    ↓
Controller
    ↓
Model Feedforward
    ↓
Admittance
    ↓
Safety
    ↓
Mapper
    ↓
MotorBus
```

### 11.2 首次真机建议

主动收窄

```yaml
safety_policy:
  cmd_vel_scale: 0.2
  state_vel_scale: 0.5
  max_acc:
    joint1: 1.0
    joint2: 1.0
```

不要通过 Safety Config 放大 URDF 或 HardwareCapabilities 提供的物理限制

### 11.3 Command Timeout

Tracking 运行一段时间后进入 FAULT 时优先检查

```yaml
cmd_timeout_s: 0.10
```

Tracking 模式必须持续发送 command

### 11.4 State Timeout

状态链不稳定时检查

```yaml
state_timeout_s: 0.05
```

同时检查 Hardware Backend 的 feedback timeout 和真实总线刷新频率

### 11.5 FAULT 排查

先读取完整 `RobotFault`

重点子错误

```text
MotorBusErr
JointActuatorMapErr
JointCtrllerErr
SafetyFault
ModelFeedforwardErr
InteractionControllerErr
```

故障发生后不要继续把正常 tracking command 当作恢复方法

### 11.6 clear_fault

`clear_fault()` 需要当前状态重新满足恢复条件

如果恢复条件仍然不满足，应先修复真实故障来源

如果只需要立即失能，使用 `force_deactivate()`

---

## 12 常见问题

### 12.1 Profile 找不到

检查 standalone 资源路径

```bash
echo "$SERIAL_ARM_RESOURCE_PATH"
```

确认 Profiles 已安装

```text
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
```

colcon 环境下确认已经

```bash
source install/setup.bash
```

### 12.2 Hardware Backend 找不到

Standalone 检查

```bash
ls install/standalone/lib/libserial_arm_hardware_damiao.so
```

再检查

```bash
echo "$LD_LIBRARY_PATH"
```

### 12.3 串口设备变化

```bash
ls /dev/ttyACM*
```

临时覆盖

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1
```

### 12.4 Robot::activate 返回 WRITE_DISABLED

检查

```yaml
control:
  runtime:
    write_enabled: false
```

如果仍处于新机械臂调试阶段，这是预期保护行为

只有完成真机前检查后才改为 `true`

### 12.5 set_cmd 返回 CMD_NOT_ALLOWED_IN_MODE

当前模式不是 Tracking

切换为

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING
);
```

或

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_TRACKING
);
```

### 12.6 Tracking 运行后出现 command timeout

持续刷新

```cpp
robot.set_cmd(cmd, now);
robot.cycle(now);
```

不要只发送一次目标后等待很长时间

### 12.7 Dynamics Frame 找不到

检查

```text
base_frame
tool_frame
requested frame
URDF link / joint name
```

### 12.8 Gravity 方向错误

依次检查

```text
URDF joint axis
calibration.direction
tor_ratio
执行器力矩正方向
gravity vector
inertial
```

### 12.9 COMPLIANT_DRAG 仍然很硬

检查

```text
compliant_drag.kp
compliant_drag.kd
Gravity Compensation
机械摩擦
减速器阻力
电机内层控制刚度
```

不要把 `COMPLIANT_DRAG` 和 Admittance 混在一起调试

### 12.10 Admittance 无接触仍持续运动

按顺序检查

```text
full_id_residual_raw
observer_residual_raw
observer_residual_filtered
bias_compensated
friction_compensated
tau_ext_hat
contact_confidence
```

如果 residual baseline 没有清零，先处理模型、bias 和摩擦问题

### 12.11 Admittance 推动迟钝

先检查

```text
Torque signal
Observer response
filter_alpha
torque_threshold
contact_confidence
```

确认外力检测正常后再调 `comfortable_torque / follow_speed / start_response_s`

### 12.12 Admittance 很容易打到最大退让

先确认无接触 `tau_ext_hat` 是否接近零

再检查

```text
max_retreat
follow_speed
comfortable_torque
return_time_s
```

不要只放大 `max_retreat`

### 12.13 ROS 2 launch 无法 import serial_arm

重新构建并 source

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

检查

```bash
python3 -c "import serial_arm; print(serial_arm.__file__)"
```

`serial_arm.__file__` 应来自当前 workspace install prefix

### 12.14 ros2_control Joint mismatch

确认三处顺序完全一致

```text
Core model.joint_names
ros2_control Xacro Joint 顺序
ros2_controllers.yaml Joint 顺序
```

### 12.15 MOMENTUM Profile 通过 ros2_control configure 失败

检查 Core YAML 是否启用了

```yaml
capability:
  admittance:
    enabled: true
    observer:
      mode: MOMENTUM
```

当前 ros2_control Adapter 没有提供 MOMENTUM 所需 `InteractionModelStateFn`

可以使用 Native C++ / C++ Terminal 调试 MOMENTUM，或者扩展 Adapter 提供该回调

### 12.16 MoveIt 能规划但真机不执行

按层检查

```text
hardware.launch.py 是否可以独立工作
ros2_control hardware interfaces
joint_trajectory_controller 状态
MoveIt controller config
Joint name 和顺序
Safety / Robot FAULT
```

不要直接从 MoveIt 层绕过底层错误

