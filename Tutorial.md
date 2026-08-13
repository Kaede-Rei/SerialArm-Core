# SerialArm-Core Tutorial

Tutorial 主要说明如何从一个刚克隆仓库的环境开始，逐步完成配置检查、离线验证、真机启动、阻抗控制、动力学前馈、ROS 2 接入以及新机械臂和新 Backend 的扩展

如果需要逐项查找类、函数、参数、返回值和错误码，请阅读 [API.md](API.md)

[toc]

---

## 1. SerialArm-Core 的使用层级

SerialArm-Core 的主要使用入口为 `Robot`，典型控制链如下

```text
Robot Profile
    |
    +-- Core YAML
    +-- Hardware Backend
    +-- Hardware YAML
    |
    v
HardwareLoader
    |
    v
MotorBus
    |
    +-------------------------------+
    |                               |
    v                               v
load_robot_cfg()                 Dynamics
    |                               |
    +---------------+---------------+
                    |
                    v
                  Robot
                    |
        +-----------+-----------+
        |           |           |
        v           v           v
      Safety      Mapper    JointCtrller
        |           |           |
        +-----------+-----------+
                    |
                    v
                 MotorBus
```

对于大多数应用开发，建议只直接操作以下对象

- `Robot Profile` 用于选择一套完整机器人实例
- `Robot` 用于 C++ 控制闭环
- `RobotSession` 用于 Python 控制闭环
- `Dynamics` 用于独立运动学和动力学计算
- ROS 2 Adapter 用于 ros2_control 和 MoveIt 2

只有开发新 Backend、调试 Safety 或验证关节映射时，才需要直接操作底层类

Damiao reference backend 的底层通信链路为：

```text
DamiaoMotorBus
    ↓
CanChannel
    ↓
CanBus
    ↓
DamiaoUsbCanBus
    ↓
SerialPort
```

同一进程中的机械臂和独立 CAN 外设可以通过相同 bus name 复用同一个 physical bus，但每个驱动必须持有自己的 `CanChannel`

```text
Robot
  ↓
DamiaoMotorBus
  ↓
ARM MotorControl
  ↓
ARM CanChannel ──────────┐
                         │
External actuator        ├── BusRegistry ── DamiaoUsbCanBus ── physical CAN
  ↓                      │
MotorControl             │
  ↓                      │
Tool CanChannel ─────────┘
```

`BusRegistry` 只负责同进程 physical bus 复用，`CanChannel` 只负责通用 CAN frame 过滤、fan-out 和有界 pending queue；Damiao 的 slave、RID 和参数响应语义由 `MotorControl` 处理，不放入 transport 层

下面用两个独立 `MotorControl` 演示共享 `master_id = 0` 的设备：

```cpp
#include "dm_hw/damiao.hpp"
#include "serial_arm_protocol_damiao_usb2can/bus.hpp"

using serial_arm::protocol::damiao_usb2can::Config;
using serial_arm::protocol::damiao_usb2can::acquire_channel;
using serial_arm::transport::CanFilter;

Config config;
config.serial_port = "/dev/ttyACM0";
config.baudrate = 921600;

auto arm_channel = acquire_channel(
    "main_can",
    config,
    {CanFilter{0x00, 0x7FF}});

auto tool_channel = acquire_channel(
    "main_can",
    config,
    {CanFilter{0x00, 0x7FF}});

if(!arm_channel || !tool_channel) {
    return;
}

damiao::Motor arm_joint(damiao::DM4310, 1, 0);
damiao::Motor tool_motor(damiao::DM4310, 7, 0);

damiao::MotorControl arm_control(*arm_channel);
damiao::MotorControl tool_control(*tool_channel);

arm_control.add_motor(&arm_joint);
tool_control.add_motor(&tool_motor);

bool arm_ok = arm_control.refresh_motor_status(arm_joint);
bool tool_ok = tool_control.refresh_motor_status(tool_motor);
```

两个 Channel 会各自收到匹配的 CAN ID 0 frame 副本；`MotorControl` 会继续根据 payload slave ID 找到自己的目标 motor，因此 tool motor 不需要加入 Robot joint 列表，也不需要新增 `Robot::send_can_frame()` 之类接口

`CanChannel` 默认采用有界 pending queue，达到上限时丢弃最旧帧并累计 `dropped_frames`；这只解决长期运行的内存上限问题，transaction correctness 仍来自 `MotorControl` 的 targeted receive

外部 CAN Driver 只拥有自己的 `CanChannel`

Driver 销毁时释放 channel 和 shared pointer 即可，不应关闭仍被其他 Driver 使用的 physical bus

串行扩展 Driver 应先通过 `acquire_serial_bus_client()` 获取 transaction-only client，再在 `SerialBusClient::transaction()` 中完成完整协议事务：

```cpp
auto client = serial_arm::transport::acquire_serial_bus_client("tool_serial", config);
if(!client) {
    return;
}

(*client)->transaction([&](serial_arm::transport::SerialTransaction& transaction) {
    transaction.write(request.data(), request.size());
    transaction.read_exact(response.data(), response.size());
});
```

request 和 response 不要拆成两个 transaction

transaction callback 只获得受限 `SerialTransaction&`，不能 open/close 串口、修改持久配置或获取 raw fd

Core 只保证同进程资源唯一所有权、CAN channel fan-out 和 Serial transaction 仲裁

不同厂家设备是否能共线仍由 bitrate、CAN ID、串口参数、电气层、地址、framing 和主动发送行为决定

### 通用 CAN Shared Bus 示例

扩展 CAN provider 通过 `acquire_can_channel()` 注册或复用 physical `CanBus`，consumer 最终只获得自己的 `CanChannel`

```cpp
auto channel_a = serial_arm::transport::acquire_can_channel(
    "main_can",
    resource_descriptor,
    [&]() -> std::shared_ptr<serial_arm::transport::CanBus> {
        auto value = std::make_shared<MyCanBus>(config);
        auto opened = value->open();
        if(!opened) return nullptr;
        return value;
    },
    {serial_arm::transport::CanFilter{0x100, 0x7FF}},
    64);

auto channel_b = serial_arm::transport::acquire_can_channel(
    "main_can",
    resource_descriptor,
    [&]() -> std::shared_ptr<serial_arm::transport::CanBus> {
        auto value = std::make_shared<MyCanBus>(config);
        auto opened = value->open();
        if(!opened) return nullptr;
        return value;
    },
    {serial_arm::transport::CanFilter{0x200, 0x7FF}},
    64);
```

相同 logical bus、physical resource 和物理配置只创建一个 physical `CanBus`

两个 consumer 只持有各自的 `CanChannel`

filter 独立

pending queue 独立

`channel_a->flush()` 只清理 A 的 pending queue，不影响 B

Protocol / Hardware consumer 不应通过 Registry 获取 raw `CanBus`，也不负责 open / close shared physical Bus

### 通用 Serial Shared Bus 示例

串行协议共享同一个 `SerialBus`

client 不直接接触底层 `SerialPort`

```cpp
auto client = serial_arm::transport::acquire_serial_bus_client(
    "tool_serial",
    config);

if(!client) {
    return;
}

(*client)->transaction([&](serial_arm::transport::SerialTransaction& transaction) {
    transaction.write(request.data(), request.size());
    transaction.read_exact(response.data(), response.size());
});
```

完整 request-response 放在一个 transaction

两个 client 并发调用时会自动排队

不同协议需要不同超时时可以使用 `SerialTransactionOptions`

```cpp
serial_arm::transport::SerialTransactionOptions options;
options.read_timeout = std::chrono::milliseconds(20);
options.write_timeout = std::chrono::milliseconds(100);

(*client)->transaction(options, [&](serial_arm::transport::SerialTransaction& transaction) {
    transaction.write(request.data(), request.size());
    transaction.read_exact(response.data(), response.size());
});
```

read/write timeout 属于事务策略，不属于 physical serial compatibility fingerprint

对于表现为普通 POSIX tty 且转换器自动处理收发方向的 RS485 设备可直接复用该机制；需要显式 RTS 或 `TIOCSRS485` 方向控制的场景暂不属于 v0.4.0 范围

同一个 named SerialBus 被多个 Driver 复用时，每次 `acquire_serial_bus_client()` 都会保留该 client 自己的默认 read/write timeout；这些 timeout 不会覆盖共享 Bus，也可以在单次 transaction 中通过 `SerialTransactionOptions` 临时覆盖

不要在 transaction callback 外保存 `SerialTransaction&`

---

## 2. 准备环境

### 2.1. 基础依赖

Core 使用 C++17，必需依赖包括

- CMake 3.20+
- C++17 compiler
- yaml-cpp
- Eigen3
- Pinocchio
- GTest，用于构建测试

Python Binding 额外需要

- Python 3.10+
- pybind11
- NumPy
- scikit-build-core

ROS 2 Adapter 面向 ROS 2 Humble，并额外依赖 ros2_control、controller_manager、xacro、robot_state_publisher、ament_cmake_python、ament_index_python 和 PyYAML 等组件；`serial_arm` Python binding 的 ROS 2 安装由 `ament_cmake_python` 管理

MoveIt 2 只在运行 MoveIt 路径时需要

### 2.2. 克隆仓库

```bash
git clone https://github.com/Kaede-Rei/SerialArm-Core.git
cd SerialArm-Core
```

后续命令默认都在仓库根目录执行

---

## 3. 第一步：只构建 Core 并运行测试

第一次接触仓库时不要直接连接真机

先确认 Core、配置系统、Dynamics、Safety 和映射模块能够正常编译并通过契约测试

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

预期结果

```text
100% tests passed
```

如果这一步失败，不要继续真机流程

优先解决依赖、Pinocchio、yaml-cpp、Eigen3 或编译器问题

---

## 4. 第二步：构建 standalone 安装目录

Standalone 模式不依赖 ROS 2 runtime，适合优先验证 Core、Damiao USB2CAN Protocol、Robot Profile、Hardware Backend、Dynamics 和配置文件

### 4.1. 构建 Core 与 Terminal

```bash
cmake -S src/serial_arm/core -B build/serial_arm_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DSERIAL_ARM_BUILD_PYTHON=OFF \
  -DSERIAL_ARM_BUILD_TERMINAL=ON

cmake --build build/serial_arm_core -j

cmake --install build/serial_arm_core \
  --prefix install/standalone
```

### 4.2. 构建 Damiao USB2CAN Protocol

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

### 4.3. 构建 Damiao Backend

```bash
cmake -S src/robot_supports/hardware/damiao \
  -B build/serial_arm_hardware_damiao \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/install/standalone"

cmake --build build/serial_arm_hardware_damiao -j

cmake --install build/serial_arm_hardware_damiao \
  --prefix install/standalone
```

### 4.4. 安装 Robot Profiles

```bash
cmake -S src/robot_supports/profiles \
  -B build/serial_arm_robot_profiles

cmake --install build/serial_arm_robot_profiles \
  --prefix install/standalone
```

### 4.5. 安装 DM-Arm resources

```bash
cmake -S src/robot_supports/robots/dm_arm/description \
  -B build/dm_arm_description

cmake --install build/dm_arm_description \
  --prefix install/standalone
```

### 4.6. 配置资源搜索路径

```bash
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
```

安装完成后至少应能找到以下文件

```text
install/standalone/bin/serial_arm_terminal
install/standalone/lib/libserial_arm_protocol_damiao_usb2can.so
install/standalone/lib/libserial_arm_hardware_damiao.so
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
install/standalone/share/dm_arm_description/config/core/gray.yaml
install/standalone/share/dm_arm_description/config/hardware.yaml
install/standalone/share/dm_arm_description/model/...
```

---

## 5. 第三步：理解 Robot Profile

Robot Profile 的作用是给一个完整机器人实例命名，并把 Core 配置、Hardware Backend、Hardware 配置、URDF、Controllers 和可选 MoveIt 配置关联起来

`dm_arm_gray` 的核心结构如下

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

这里最重要的是先理解前三项

```text
core.config
hardware.plugin
hardware.config
```

它们最终解析为

```text
Core YAML
Hardware Backend shared library
Hardware Backend YAML
```

Standalone C++、Python 和 ROS 2 Adapter 都复用这套 Profile

Robot Profile 不属于 ROS 2，也不要求 ROS 2 runtime

---

## 6. 第四步：检查 Core YAML

以 `dm_arm_gray` 为例，Core YAML 主要由以下部分组成

```text
model
calibration
control
safety_policy
shutdown
```

### 6.1. 检查 model

```yaml
model:
  urdf_path: ../../model/gray/urdf/dm_arm_no_gripper.urdf
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

需要确认

- `joint_names` 顺序与实际受控关节顺序一致
- URDF 中存在这些 Joint
- `base_frame` 和 `tool_frame` 在 URDF 中存在
- URDF 中的 joint axis 正确
- URDF 中的位置、速度和 effort limit 合理
- Dynamics 所需 inertial 参数存在

如果惯性参数暂时没有完成辨识，可以先使用合法的 placeholder inertial 完成软件接入

此时 FK 和 Jacobian 仍可用于软件联调，但 gravity、mass matrix、inverse dynamics 等结果不能视为真实动力学结果

### 6.2. 检查 calibration

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

参数含义

```text
direction
  Joint 与 Actuator 的方向关系，只允许 1 或 -1

pos_ratio
  actuator position delta / joint position delta

tor_ratio
  joint torque / actuator reported torque

joint_zero_offset
  Joint 侧零位偏置

actuator_zero_offset
  Actuator 侧零位偏置
```

如果电机 SDK 已经返回减速器输出端位置和力矩，`pos_ratio` 与 `tor_ratio` 通常可以保持 `1.0`

不要为了修正方向错误直接修改 URDF joint axis 和 Backend 单位转换

先明确问题属于机械模型、关节映射还是厂商协议层

### 6.3. 检查 runtime

```yaml
control:
  runtime:
    ctrl_frequency_hz: 200.0
    joint_acc_filter_alpha: 0.2
    write_enabled: false
    model_feedforward_mode: GRAVITY
    tracking_impedance_mode: COMPLIANT_TRACKING
```

第一次调试必须优先保持

```yaml
write_enabled: false
```

`Robot::activate()` 在 `write_enabled=false` 时会拒绝进入 ACTIVE，因此不会通过 `Robot` 真机闭环写执行器

在完成配置、方向、零位和限位检查前不要改成 `true`

---

## 7. 第五步：构建 Python Binding 并做配置检查

Python Binding 适合快速检查配置和进行离线工具开发

### 7.1. 构建 wheel

```bash
cd src/serial_arm/core/python

python -m pip install build

python -m build --wheel

python -m pip install --force-reinstall dist/serial_arm-*.whl

cd ../../../..
```

### 7.2. 运行 Python Terminal 的 check-only

```bash
export SERIAL_ARM_RESOURCE_PATH="$PWD/install/standalone"
export LD_LIBRARY_PATH="$PWD/install/standalone/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"

python src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray \
  --check-only
```

这一步应该用于检查

- Profile 是否可解析
- Core YAML 是否存在
- Hardware Backend 是否能加载
- Hardware YAML 是否可读取
- HardwareCapabilities 是否能参与 Safety limit 解析
- Dynamics 配置是否有效

如果 check-only 失败，优先修复配置，不要绕过错误进入真机

### 7.3. 检查串口并按需覆盖

实际连接机械臂前先查看可用的 `/dev/ttyACM*`：

```bash
ls /dev/ttyACM*
```

假设输出为：

```text
/dev/ttyACM0
/dev/ttyACM1
```

如果机械臂连接在 `/dev/ttyACM1`，启动时覆盖 Robot Profile 对应 `hardware.yaml` 中的默认串口：

```bash
serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1
```

Python Terminal 也使用同一套 runtime override：

```bash
python src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1
```

未提供 `--serial-port` 时继续使用 Robot Profile 解析到的 `hardware.yaml` 默认值

### 7.4. Standalone wheel 与 ROS 2 Python binding 的区别

Standalone Python 使用上面的 wheel / pip 安装方式；ROS 2 workspace 不需要手动 pip 安装该 workspace 源码，`serial_arm_core` 会在 ament 构建中通过 `ament_cmake_python` 安装 `serial_arm/__init__.py`，并将 pybind11 扩展 `_serial_arm*.so` 安装到同一个 Python package 目录

因此 ROS 2 下应使用：

```bash
colcon build --symlink-install
source install/setup.bash

python3 -c "import serial_arm; print(serial_arm.__file__)"
```

输出路径应来自 workspace 的 `install/serial_arm_core/.../site-packages/serial_arm/`，不要依赖源码目录或手动安装的 wheel 来掩盖 workspace 安装问题

---

## 8. 第六步：使用 Python Dynamics

这一节完全不需要进入机械臂 ACTIVE 状态

目标是验证 URDF、Joint 顺序、FK、Jacobian 和重力项

```python
from pathlib import Path

import numpy as np
import serial_arm

config_file = Path(
    "install/standalone/share/dm_arm_description/config/core/gray.yaml"
)

hardware_plugin = (
    "install/standalone/lib/libserial_arm_hardware_damiao.so"
)

hardware_config = Path(
    "install/standalone/share/dm_arm_description/config/hardware.yaml"
)

cfg = serial_arm.load_robot_cfg(
    str(config_file),
    str(hardware_plugin),
    str(hardware_config),
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

print("gravity =", dynamics.gravity)
print("gravity_compensation =", dynamics.gravity_compensation)
print("mass_matrix =")
print(dynamics.mass_matrix)
print("tool_pose =")
print(dynamics.tool_pose)
print("tool_jacobian =")
print(dynamics.tool_jacobian)
```

预期结果

- `gravity` 长度等于关节数量
- `mass_matrix` 维度为 `N x N`
- `tool_pose` 为 `4 x 4`
- `tool_jacobian` 为 `6 x N`
- 所有结果应为有限值

如果 `FRAME_NOT_FOUND`，检查 `base_frame` 和 `tool_frame`

如果 `JOINT_NOT_FOUND`，检查 `joint_names`

如果结果数值明显异常，先检查 URDF joint axis、origin、mass 和 inertia

---

## 9. 第七步：理解五种阻抗模式

不要把五种模式理解成五套完全独立的控制器

它们共享统一的 MIT 语义

```text
position
velocity
torque
kp
kd
```

差异主要在于参考位置如何产生，以及使用哪套 `kp` 和 `kd`

| 模式 | 参考位置 | 是否接受 `set_cmd` | 典型用途 |
| --- | --- | --- | --- |
| `RIGID_HOLD` | 切换模式时的当前实测位置 | 否 | 高刚度定点保持 |
| `RIGID_TRACKING` | 上层命令 | 是 | 常规刚性轨迹跟踪 |
| `COMPLIANT_HOLD` | 切换模式时的当前实测位置 | 否 | 低刚度定点保持 |
| `COMPLIANT_DRAG` | 每周期当前实测位置 | 否 | 手动拖拽与柔性示教 |
| `COMPLIANT_TRACKING` | 上层命令 | 是 | 柔性轨迹跟踪 |

模式切换后，Core 会清除已有的 tracking command，并以当前实测状态重新建立参考和 Safety 命令历史

因此正确流程是

```text
set_impedance_mode(TRACKING)
        |
        v
持续发送 set_cmd(...)
        |
        v
持续执行 cycle()
```

不要先发送 `set_cmd()` 再切换 tracking mode

---

## 10. 第八步：写 C++ Robot 控制闭环

下面示例展示真正的 Core 使用路径

它不是 ROS 2 节点，也不会自动创建控制线程

调用者负责按 `ctrl_frequency_hz` 调用 `cycle()`

### 10.1. 最小 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(serial_arm_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(serial_arm_core CONFIG REQUIRED)

add_executable(robot_example main.cpp)

target_link_libraries(robot_example
  PRIVATE
    serial_arm::core
    serial_arm::config
    serial_arm::robot
    serial_arm::dynamics
)
```

### 10.2. 完整 main.cpp

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include <tl/expected.hpp>

#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/robot.hpp"

using namespace serial_arm;

int main() {
    auto profile_result = load_robot_profile_core("dm_arm_gray");

    if(!profile_result) {
        std::cerr << "load_robot_profile_core failed\n";
        return 1;
    }

    const RobotProfileCore profile = profile_result.value();

    HardwareLoader hardware_loader;

    auto bus_result = hardware_loader.load(
        profile.hardware_plugin,
        profile.hardware_config_path);

    if(!bus_result) {
        std::cerr << "HardwareLoader::load failed\n";
        return 1;
    }

    std::unique_ptr<MotorBus> bus = std::move(bus_result.value());

    auto cfg_result = load_robot_cfg(
        profile.core_config_path,
        bus->capabilities());

    if(!cfg_result) {
        std::cerr << "load_robot_cfg failed: "
                  << cfg_result.error().message
                  << "\n";
        return 1;
    }

    RobotCfg cfg = cfg_result.value();

    Dynamics dynamics;

    if(auto result = dynamics.configure(cfg.dynamics); !result) {
        std::cerr << "Dynamics::configure failed\n";
        return 1;
    }

    const std::size_t joints_count = cfg.joint_names.size();

    ModelFeedforwardFn feedforward =
        [&dynamics, joints_count](
            ModelFeedforwardMode mode,
            const JointState& state,
            const JointVector& acc,
            const JointVector& ref_acc,
            double) -> tl::expected<JointVector, ModelFeedforwardErr>
        {
            if(mode == ModelFeedforwardMode::NONE) {
                return JointVector(joints_count, 0.0);
            }

            auto update_result = dynamics.update(
                state,
                acc,
                ref_acc);

            if(!update_result) {
                return tl::make_unexpected(
                    ModelFeedforwardErr::COMPUTE_FAILED);
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
                ModelFeedforwardErr::INVALID_MODE);
        };

    Robot robot;

    if(auto result = robot.configure(
        cfg,
        std::move(bus),
        feedforward); !result)
    {
        std::cerr << "Robot::configure failed\n";
        return 1;
    }

    if(!cfg.runtime.write_enabled) {
        std::cout
            << "Configuration is valid, but write_enabled=false\n";
        return 0;
    }

    if(auto result = robot.activate(); !result) {
        std::cerr << "Robot::activate failed\n";
        return 1;
    }

    if(auto result = robot.set_impedance_mode(
        JointImpedanceMode::RIGID_TRACKING); !result)
    {
        std::cerr << "set_impedance_mode failed\n";
        robot.force_deactivate();
        return 1;
    }

    JointVector target = robot.get_joint_state().pos;

    target[0] += 0.05;

    const auto period = std::chrono::duration<double>(
        1.0 / cfg.runtime.ctrl_frequency_hz);

    for(int i = 0; i < 400; ++i) {
        const auto now = Robot::Clock::now();

        if(auto result = robot.set_cmd(
            JointPosCmd{target},
            now); !result)
        {
            std::cerr << "Robot::set_cmd failed\n";
            robot.force_deactivate();
            return 1;
        }

        auto cycle_result = robot.cycle(now);

        if(!cycle_result) {
            std::cerr << "Robot::cycle failed\n";
            robot.force_deactivate();
            return 1;
        }

        const RobotCycleOutput& output =
            cycle_result.value();

        std::cout
            << "q0=" << output.joint_state.pos[0]
            << " tau_ff0=" << output.model_feedforward[0]
            << " dt=" << output.dt
            << "\n";

        std::this_thread::sleep_for(period);
    }

    if(auto result = robot.deactivate(); !result) {
        std::cerr << "Robot::deactivate failed\n";
        return 1;
    }

    return 0;
}
```

### 10.3. 为什么 tracking 示例每周期都调用 set_cmd

`cmd_timeout_s` 是独立的 Safety 约束

只调用一次 `set_cmd()` 后长时间持续 `cycle()` 会触发命令超时

对于持续 tracking，应让上层规划器、轨迹生成器或控制任务以稳定频率刷新目标

---

## 11. 第九步：实现三种常见跟踪命令

### 11.1. 只给位置

```cpp
JointPosCmd cmd;
cmd.pos = target;

robot.set_cmd(cmd);
```

Core 最终构造

```text
pos = target
vel = 0
tor = model feedforward
kp  = 当前 tracking mode 对应 kp
kd  = 当前 tracking mode 对应 kd
```

### 11.2. 给位置和速度

```cpp
JointPosVelCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;

robot.set_cmd(cmd);
```

适合轨迹生成器已经提供关节速度参考的情况

### 11.3. 给位置、速度和附加力矩

```cpp
JointPosVelTorCmd cmd;
cmd.pos = target_pos;
cmd.vel = target_vel;
cmd.tor = extra_torque;

robot.set_cmd(cmd);
```

这里的 `tor` 是附加关节力矩

如果同时启用了模型前馈，最终关节力矩为

```text
command torque + model feedforward torque
```

不要在上层已经手动加入重力补偿后又启用 `GRAVITY`，否则会发生重复补偿

---

## 12. 第十步：实现柔性拖拽

柔性拖拽使用

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_DRAG);
```

进入该模式后不再发送 `set_cmd()`

控制循环只需要持续执行

```cpp
while(running) {
    auto result = robot.cycle();

    if(!result) {
        break;
    }
}
```

`COMPLIANT_DRAG` 每周期把当前实测位置作为新的参考位置

因此其核心效果不是追踪一个固定位置，而是避免建立明显的位置回拉误差

实际拖拽手感主要由以下参数决定

```yaml
control:
  controller:
    compliant_drag:
      kp:
        joint1: 0.0
      kd:
        joint1: 0.08
```

以及

```yaml
control:
  runtime:
    model_feedforward_mode: GRAVITY
```

建议调试顺序

1. 先验证执行器反馈方向与关节方向
2. 再启用低 `kp` 和低 `kd`
3. 再验证重力模型方向
4. 最后逐轴调整 `gravity_scale`

不要用大 `kp` 的所谓拖拽模式抵消重力模型问题

---

## 13. 第十一步：调试重力补偿

### 13.1. 开启 GRAVITY

Core YAML

```yaml
control:
  runtime:
    model_feedforward_mode: GRAVITY
```

Dynamics 配置

```yaml
model:
  gravity_scale:
    - 1.0
    - 1.0
    - 1.0
    - 1.0
    - 1.0
    - 1.0
```

### 13.2. 运行时修改 gravity_scale

C++

```cpp
Dynamics dynamics;

dynamics.configure(cfg.dynamics);

dynamics.set_gravity_scale({
    1.0,
    0.9,
    0.85,
    1.0,
    1.0,
    1.0,
});
```

Python `RobotSession`

```python
arm.set_gravity_scale(
    np.array(
        [1.0, 0.9, 0.85, 1.0, 1.0, 1.0],
        dtype=np.float64,
    )
)
```

### 13.3. 推荐验证方式

不要一开始就在整机拖拽下调整所有关节

建议逐轴进行

```text
固定其余关节
    |
    v
降低目标关节 kp
    |
    v
观察自然下坠方向
    |
    v
开启 gravity compensation
    |
    v
检查补偿方向是否正确
    |
    v
从较小 gravity_scale 向上增加
```

如果补偿方向错误，优先检查

- URDF joint axis
- `direction`
- 电机力矩正方向
- `tor_ratio`
- 重力坐标方向
- inertial 参数

不要先靠负 `gravity_scale` 修正体系错误

`gravity_scale` 的有效范围为 `[0, 1]`

---

## 14. 第十二步：使用 Python RobotSession

`RobotSession` 适合任务层、数据采集工具和快速实验

控制周期由 C++ 工作线程维护，Python 不需要自己写 200 Hz `cycle()` 循环

### 14.1. 创建会话

```python
from pathlib import Path

import numpy as np
import serial_arm

config_file = Path(
    "install/standalone/share/dm_arm_description/config/core/gray.yaml"
)

hardware_plugin = (
    "install/standalone/lib/libserial_arm_hardware_damiao.so"
)

hardware_config = Path(
    "install/standalone/share/dm_arm_description/config/hardware.yaml"
)

arm = serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
)
```

### 14.2. `write_enabled` 在 RobotSession 中的行为

`RobotSession` 会先加载真实 Backend 以获得 `HardwareCapabilities` 并完成 Core 配置解析

当 Core YAML 为

```yaml
write_enabled: false
```

Session 会把实际控制 Backend 替换成内部 `MockMotorBus`，因此 `start()` 可以用于离线控制链验证，不会使能真实执行器

当 Core YAML 为

```yaml
write_enabled: true
```

Session 才会把真实 Hardware Backend 交给 `Robot`

真机模式必须完成方向、零位、限位、Backend 和 Safety 验证后再调用

```python
arm.start()
```

### 14.3. 切换刚性跟踪并移动

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.RIGID_TRACKING
)

snapshot = arm.snapshot

if not snapshot.valid:
    raise RuntimeError(snapshot.last_error)

target = snapshot.cycle.joint_state.pos.copy()

target[0] += 0.05

arm.move_to(
    target,
    speed_scale=0.15,
)
```

### 14.4. 保持当前位置

```python
arm.hold_current()
```

### 14.5. 柔性拖拽

```python
arm.set_impedance_mode(
    serial_arm.JointImpedanceMode.COMPLIANT_DRAG
)
```

### 14.6. 安全退出

```python
arm.stop()
```

推荐使用上下文管理器

```python
with serial_arm.RobotSession(
    config_file,
    hardware_plugin,
    hardware_config,
) as arm:
    arm.set_impedance_mode(
        serial_arm.JointImpedanceMode.COMPLIANT_DRAG
    )

    snapshot = arm.snapshot

    print(snapshot.cycle.joint_state.pos)
```

---

## 15. 第十三步：真机 Bringup

在把 `write_enabled` 改成 `true` 前完成以下检查

### 15.1. 机械侧

- 机械臂安装可靠
- 无人员位于危险运动区域
- 末端负载已知
- 急停或硬件断电路径可用
- 初始姿态不会在使能瞬间发生机构干涉

### 15.2. 模型侧

- URDF joint axis 正确
- joint order 正确
- position limit 正确
- velocity limit 正确
- effort limit 正确
- base 和 tool frame 正确

### 15.3. 映射侧

- direction 正确
- pos_ratio 正确
- tor_ratio 正确
- joint zero 正确
- actuator zero 正确

### 15.4. Backend 侧

DM-Arm 示例配置

```yaml
damiao:
  serial_port: /dev/ttyACM0
  baudrate: 921600
  feedback_timeout_s: 0.05
  activation_retries: 3
  startup_read_cycles: 5
```

还要逐轴核对

```yaml
actuators:
  joint1:
    name: actuator1
    motor_id: 1
    master_id: 0
    motor_type: DM4340
```

### 15.5. 小幅单关节验证

建议先用

```text
0.02 rad
0.05 rad
0.10 rad
```

逐步验证方向、速度、跟踪误差和停止行为

---

## 16. 第十四步：ROS 2 display

当 standalone 配置和 URDF 已经通过检查后，再进入 ROS 2

第一次在新系统构建时先安装 workspace 依赖：

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

`serial_arm_ros2_control` 的 `robot_profile` launch 通过 `serial_arm` Python binding 调用 Core Profile resolver；因此 `serial_arm_core` 必须以 `SERIAL_ARM_BUILD_PYTHON=ON` 构建；默认值已经是 `ON`，普通 `colcon build` 不需要额外参数；ROS 2 下 `serial_arm` 由 `ament_cmake_python` 安装，显式设置 `-DSERIAL_ARM_BUILD_PYTHON=OFF` 时 C++ Core 仍可用，但 `robot_profile` ROS 2 launch 不可用

启动 launch 前先验证 Python overlay：

```bash
python3 - <<'PY'
from pathlib import Path

import serial_arm
from ament_index_python.packages import get_package_share_directory
from serial_arm import load_robot_profile_core

profiles_file = (
    Path(get_package_share_directory("serial_arm_robot_profiles"))
    / "config"
    / "robot_profiles.yaml"
)

print("serial_arm:", serial_arm.__file__)
profile = load_robot_profile_core("dm_arm_gray", str(profiles_file))
print("profile_file:", profile.profile_file)
print("core_config_path:", profile.core_config_path)
print("hardware_plugin:", profile.hardware_plugin)
print("serial_arm Python binding: OK")
PY
```

`serial_arm.__file__` 应指向该 workspace 的 `install/serial_arm_core/.../site-packages/serial_arm/__init__.py`；如果这里失败，先修 Python binding 安装，不要直接继续 launch

只显示模型

```bash
ros2 launch serial_arm_ros2_control \
  display.launch.py \
  robot_profile:=dm_arm_gray
```

这一阶段重点检查

- mesh 是否正确
- Joint 方向是否正确
- TF 树是否正确
- base 和 tool frame 是否正确
- 机器人模型是否与实物一致

display 正确不代表真机映射一定正确

URDF joint axis 和 actuator direction 属于不同层

---

## 17. 第十五步：ROS 2 hardware

模型验证完成后再启动 ros2_control hardware

```bash
ros2 launch serial_arm_ros2_control \
  hardware.launch.py \
  robot_profile:=dm_arm_gray
```

如果机械臂枚举为 `/dev/ttyACM1`，可以只覆盖当前 ROS 2 启动的串口：

```bash
ros2 launch serial_arm_ros2_control \
  hardware.launch.py \
  robot_profile:=dm_arm_gray \
  serial_port:=/dev/ttyACM1
```

MoveIt 顶层入口会继续透传相同参数：

```bash
ros2 launch serial_arm_ros2_control \
  moveit.launch.py \
  robot_profile:=dm_arm_gray \
  serial_port:=/dev/ttyACM1
```

该路径主要建立

```text
robot_state_publisher
controller_manager
joint_state_broadcaster
joint_trajectory_controller
SerialArm SystemInterface
```

它不依赖 MoveIt

先确认

```bash
ros2 control list_hardware_interfaces
```

再确认

```bash
ros2 control list_controllers
```

最后才发送小幅 JointTrajectoryController 目标

---

## 18. 第十六步：MoveIt 2

MoveIt 属于上层规划能力，不属于 Core 必选依赖

只有 Profile 定义了

```yaml
moveit:
  package: dm_arm_no_gripper
```

才运行

```bash
ros2 launch serial_arm_ros2_control \
  moveit.launch.py \
  robot_profile:=dm_arm_gray
```

推荐顺序始终保持

```text
Core test
    ->
Profile / config
    ->
URDF / Dynamics
    ->
Backend
    ->
小幅 Joint control
    ->
ros2_control
    ->
MoveIt
```

不要用 MoveIt 来替代底层方向和限位验证

---

## 19. 第十七步：新增一台机械臂

新增机械臂的核心原则是：**优先复用 Core、Protocol 和 Hardware Backend，只新增机器人自身的模型、配置和 Profile**

不要复制整个仓库，也不要因为机械结构变化就修改 `serial_arm/core`

### 19.1. 先判断到底需要新增什么

在开始建目录前先判断变化发生在哪一层

| 场景 | 需要新增或修改 | 不需要修改 |
| --- | --- | --- |
| 同一机械臂新增一个结构版本 | Robot Support + Core YAML + Profile | Core / Backend |
| 新机械臂继续使用现有 Damiao 电机 | Robot Support + Hardware YAML + Profile | Core / Damiao Backend |
| 新机械臂使用已有其他 Backend | Robot Support + 对应 Hardware YAML + Profile | Core |
| 执行器厂商协议发生变化 | 新 Hardware Backend + Robot Support + Profile | Core |
| 通信设备私有协议发生变化 | 新 Protocol Adapter，通常还需要对应 Backend | Core |
| 只增加 ROS 2 接入 | `ros2_control_xacro` + controllers + Profile ROS 字段 | Core / Backend |
| 只增加 MoveIt | MoveIt config + Profile `moveit` 字段 | Core / Backend |

例如新增一台仍然使用达妙电机和达妙官方 USB2CAN 的机械臂时，正常情况下只需要：

```text
新 URDF / mesh
    ↓
新 Core YAML
    ↓
新 hardware.yaml
    ↓
在 robot_profiles.yaml 注册
```

不应该新建第二份 `DamiaoMotorBus`

### 19.2. 先确定命名和 Joint 顺序

建议先固定以下名称，再开始写文件

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

其中最重要的是 `joint order`

SerialArm-Core 中以下内容都必须围绕同一组受控 Joint：

```text
URDF Joint
Core model.joint_names
calibration.joints
controller gains
Safety per-joint parameters
shutdown.park_pos
Hardware actuator mapping
ros2_control joints                 # 仅 ROS 2 使用时
controller joints                   # 仅 ROS 2 使用时
```

`model.joint_names` 的顺序就是 Core 的 Joint Vector 顺序

后续不要为了适配 Backend 随意改变这组顺序

### 19.3. 创建 Robot Support 目录

推荐直接参考 `dm_arm` 的资源组织方式

```text
src/robot_supports/robots/my_arm/
└── description/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── core/
    │   │   └── default.yaml
    │   ├── hardware.yaml
    │   └── ros2_controllers.yaml        # 仅 ROS 2 需要
    └── model/
        └── default/
            ├── meshes/
            │   └── ...
            └── urdf/
                ├── my_arm.urdf
                └── my_arm.ros2_control.xacro  # 仅 ROS 2 需要
```

如果同一台机械臂有多个版本，可以继续：

```text
model/
├── default/
├── long_reach/
└── with_gripper/

config/core/
├── default.yaml
├── long_reach.yaml
└── with_gripper.yaml
```

不要为每个 variant 再创建一份 Backend

### 19.4. 创建 description resource package

Robot Profile 的 resource resolver 会从安装目录或源码目录寻找包含 `package.xml` 的 resource package

因此 `description/` 不只是 ROS package，也承担 framework-neutral resource package 的作用

最小 `CMakeLists.txt` 可以直接使用：

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

这种写法同时支持：

```text
standalone CMake install
colcon / ament workspace
```

最小 `package.xml`：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>my_arm_description</name>
    <version>0.1.0</version>
    <description>Robot model and configuration resources for my_arm</description>
    <maintainer email="you@example.com">your_name</maintainer>
    <license>MIT</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <exec_depend>xacro</exec_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

如果完全不使用 ROS 2 Xacro，可以根据实际情况去掉不需要的 `xacro` runtime dependency

### 19.5. 准备 URDF

首先只关注机械模型本身，不要一开始就做 MoveIt

例如：

```text
model/default/urdf/my_arm.urdf
```

至少确认以下内容

#### 19.5.1. 受控 Joint

每个受控 Joint 必须：

```text
名称唯一
parent / child 正确
origin 正确
axis 正确
joint type 正确
limit 正确
```

Core YAML 中：

```yaml
model:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
```

这些名称必须在 URDF 中真实存在

固定连接 `fixed joint` 不需要放入 `model.joint_names`

#### 19.5.2. Joint limit

对于普通 revolute joint，至少确认：

```xml
<limit lower="..."
       upper="..."
       velocity="..."
       effort="..."/>
```

这些 limit 会参与 SerialArm Safety limit 解析

不要为了“先跑起来”随意给一个特别大的 limit

对于真正的 continuous joint，应使用符合实际模型的 continuous joint 语义，不要伪造位置上下限

#### 19.5.3. Dynamics 参数

所有进入动力学链路的 Link 应具有合法 inertial：

```xml
<inertial>
    <origin .../>
    <mass value="..."/>
    <inertia .../>
</inertial>
```

如果质量和惯量仍在建模阶段，可以先使用合法 placeholder 完成软件链路验证

但是此时：

```text
FK / Jacobian
```

可以用于结构检查，而：

```text
gravity
mass matrix
inverse dynamics
```

不能被视为真实机械臂动力学结果

#### 19.5.4. Frame

提前确定：

```text
base_frame
tool_frame
```

例如：

```yaml
model:
  base_frame: base_link
  tool_frame: tool0
```

两个 Frame 必须在 URDF 中存在

#### 19.5.5. Mesh 和路径

模型安装后仍然需要能解析 mesh 和 URDF 依赖

不要在 URDF 中写机器相关的绝对路径，例如：

```text
/home/user/project/...
```

### 19.6. 创建 Core YAML

建议复制现有 Core YAML 的**结构**，不要复制原机械臂的具体数值

文件：

```text
config/core/default.yaml
```

完整配置分为：

```text
model
calibration
control
safety_policy
shutdown
```

#### 19.6.1. model

```yaml
model:
  urdf_path: ../../model/default/urdf/my_arm.urdf
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  base_frame: base_link
  tool_frame: tool0
  gravity: [0.0, 0.0, -9.81]
  gravity_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

检查：

```text
urdf_path 相对该 YAML 可解析
joint_names 与 URDF 一致
base_frame / tool_frame 存在
gravity_scale 长度等于 Joint 数量
```

#### 19.6.2. calibration

每一个 `model.joint_names` 中的 Joint 都必须有 calibration

```yaml
calibration:
  joints:
    joint1:
      direction: 1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0

    joint2:
      direction: -1.0
      pos_ratio: 1.0
      tor_ratio: 1.0
      joint_zero_offset: 0.0
      actuator_zero_offset: 0.0
```

其中：

```text
direction
    Joint 正方向与执行器正方向关系

pos_ratio
    actuator position delta / joint position delta

tor_ratio
    joint torque / actuator reported torque

joint_zero_offset
    Joint 侧零位

actuator_zero_offset
    Actuator 侧零位
```

不要用修改 URDF joint axis 的方式补偿纯执行器方向错误

也不要在 Backend 内偷偷加入只针对某台机械臂的零位修正

机械模型、Mapper calibration 和厂商协议层要分开

#### 19.6.3. control

第一次接入新机械臂时必须：

```yaml
control:
  runtime:
    ctrl_frequency_hz: 200.0
    joint_acc_filter_alpha: 0.2
    write_enabled: false
    model_feedforward_mode: NONE
    tracking_impedance_mode: RIGID_TRACKING
```

重点：

```text
write_enabled: false
```

在模型、方向、零位、HardwareCapabilities 和 Safety 没有确认前不要打开写入

初始阶段也建议先使用：

```text
model_feedforward_mode: NONE
```

等 Dynamics 参数验证后再切换：

```text
GRAVITY
FULL_INVERSE_DYNAMICS
```

五套阻抗增益必须针对新机械臂重新设定

不要直接复制 DM-Arm 的 kp / kd

至少需要：

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

上面的数值只表示配置格式，不是新机械臂推荐参数

实际文件必须为所有受控 Joint 提供对应项

#### 19.6.4. safety_policy

新机械臂必须重新检查：

```text
position margin
velocity scale
max acceleration
kp / kd override
state timeout
command timeout
fault recovery
```

最重要的原则是：

> Core Safety 可以在 URDF 和 HardwareCapabilities 基础上进一步收窄限制，但不应该依靠配置把真实硬件能力扩大

例如：

```yaml
safety_policy:
  position_margin: 0.0
  cmd_vel_scale: 0.2
  state_vel_scale: 1.0

  max_acc:
    joint1: 1.0
    joint2: 1.0

  max_dt_s: 0.02
  state_timeout_s: 0.05
  cmd_timeout_s: 0.10
  require_all_actuators_online: true
  require_all_actuators_enabled: true
  reject_motor_error: true
  require_continuous_cmd: false
```

第一次运动测试可以主动把速度和加速度限制收窄

#### 19.6.5. shutdown

为新机械臂单独确定安全停放姿态

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

`park_pos` 必须包含全部受控 Joint

不要默认认为全零位就是机械结构上的安全停放姿态

### 19.7. 创建 Hardware YAML

如果新机械臂继续使用现有 Backend，只需要为这台机器人提供新的 Hardware instance config

以 Damiao Backend 为例：

```yaml
damiao:
  bus: main_can
  serial_port: /dev/ttyACM0
  baudrate: 921600
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

Damiao Hardware YAML 中：

```text
actuators 的 YAML key
    → joint_name

name
    → 执行器名称

motor_id
    → 达妙电机 ID，必须非 0 且不能重复

master_id
    → 主站 ID

motor_type
    → Backend 能识别的达妙电机型号名称
```

配置应保证：

```text
actuator 数量 == model.joint_names 数量
actuator key 与 Joint 名称一一对应
actuator 配置顺序与 model.joint_names 顺序保持一致
motor_id 不重复
motor_type 与真实电机一致
```

不要把机械臂专属的 Joint direction、减速比和零位写入 Damiao 通用 Backend 源码

这些机器人相关关系应优先放在 Core calibration 中

如果新机械臂根本不使用 Damiao 电机，不要强行修改这个 YAML 适配

此时转到第 20 节实现新的 Hardware Backend

### 19.8. 注册 framework-neutral Robot Profile

打开：

```text
src/robot_supports/profiles/config/robot_profiles.yaml
```

对于 Native C++、Python、Terminal，Core resolver 真正必须的字段只有：

```text
core
hardware
```

最小 Profile：

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

这时已经可以通过：

```text
C++
Python
serial_arm_terminal
```

使用该机械臂

Robot Profile 本身不是 ROS 2 专属配置

### 19.9. 使用 colcon 构建并验证新 Robot Support

如果仓库本身使用 colcon 管理，直接：

```bash
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash
```

新增 resource package 后，先确认资源已经安装：

```bash
find install/my_arm_description/share/my_arm_description \
  -maxdepth 4 \
  -type f
```

应至少能看到：

```text
package.xml
config/core/default.yaml
config/hardware.yaml
model/...
```

再确认 Profile 已安装：

```bash
grep -n "my_arm_default" \
  install/serial_arm_robot_profiles/share/serial_arm_robot_profiles/config/robot_profiles.yaml
```

### 19.10. 先做不连接真机的 Profile / Config 检查

不要新增 Profile 后直接 `write_enabled=true`

先保持：

```yaml
write_enabled: false
```

然后使用 Python Terminal：

```bash
python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile my_arm_default \
  --check-only
```

`--check-only` 应至少验证：

```text
Profile 可以找到
Core YAML 可以解析
Hardware Backend 可以加载
Hardware YAML 可以解析
HardwareCapabilities 可以读取
URDF / Joint 可以解析
Safety limit 可以解析
Dynamics 配置可以建立
```

这一步不会连接或写入真实硬件

如果失败，先修资源和配置，不要绕过错误进入真机

### 19.11. 检查 Dynamics 和 Joint Mapping

Profile 可以解析后，再进行：

```text
FK
Jacobian
gravity
Joint / Actuator mapping
```

至少确认：

```text
FK 姿态方向合理
Jacobian 维度 = 6 x N
gravity 长度 = N
所有结果 finite
Joint 正方向与真实机械臂一致
零位与机械结构一致
```

如果动力学 inertial 仍是 placeholder，只验证软件结构，不评价力矩数值精度

### 19.12. 第一次打开 C++ Terminal

配置检查通过后可以：

```bash
serial_arm_terminal \
  --robot-profile my_arm_default
```

此时仍建议保持：

```yaml
write_enabled: false
```

先确认 Terminal 能正确解析：

```text
Profile
Core config
Backend plugin
Hardware config
Joint 数量
```

然后再进入真机联调阶段

### 19.13. 第一次真机测试

只有在以下内容全部确认后才把：

```yaml
write_enabled: true
```

打开

检查表：

```text
[ ] 急停可用
[ ] 电源和通信稳定
[ ] motor_id 正确且唯一
[ ] Joint 顺序正确
[ ] Joint direction 正确
[ ] 零位正确
[ ] URDF limit 正确
[ ] HardwareCapabilities 正确
[ ] Safety 已主动收窄速度和加速度
[ ] park pose 不会碰撞
```

第一次运动不要直接测试完整轨迹

建议顺序：

```text
1. 读取全部 Joint state
2. 单关节极小幅运动
3. 检查正负方向
4. 检查停止行为
5. RIGID_HOLD
6. 低速 RIGID_TRACKING
7. 再逐步恢复正常 Safety limit
8. 最后才测试 compliant mode 和动力学前馈
```

发现方向错误时先回到 `calibration.direction` 检查

发现位置比例错误时检查 `pos_ratio`

发现力矩比例错误时检查 `tor_ratio`

不要通过随意修改 URDF axis 来掩盖执行器映射问题

### 19.14. 如果需要 ROS 2，再增加 ros2_control 资源

ROS 2 不是新增机械臂进入 Core 的前置条件

只有需要 `serial_arm_ros2_control` 时才继续增加以下内容

#### 19.14.1. ros2_control Xacro

例如：

```text
model/default/urdf/my_arm.ros2_control.xacro
```

最小结构参考 DM-Arm：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_arm">
    <xacro:arg name="config_file" default=""/>
    <xacro:arg name="hardware_plugin" default=""/>
    <xacro:arg name="hardware_config" default=""/>

    <xacro:include filename="$(find my_arm_description)/model/default/urdf/my_arm.urdf"/>
    <xacro:include filename="$(find serial_arm_ros2_control)/urdf/serial_arm_system.ros2_control.xacro"/>

    <ros2_control name="my_arm" type="system">
        <xacro:serial_arm_ros2_control_hardware
            config_file="$(arg config_file)"
            hardware_plugin="$(arg hardware_plugin)"
            hardware_config="$(arg hardware_config)"/>

        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <!-- 其余受控 Joint 按相同方式声明 -->
    </ros2_control>
</robot>
```

所有 ROS 2 Joint 名称仍然必须和 `model.joint_names` 一致

#### 19.14.2. ros2_controllers.yaml

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

`update_rate` 建议与 Core 的：

```yaml
control.runtime.ctrl_frequency_hz
```

保持一致

#### 19.14.3. 为 Profile 增加 ROS 字段

在已经能被 Core 使用的最小 Profile 基础上增加：

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

然后重新：

```bash
colcon build
source install/setup.bash
```

先只看模型：

```bash
ros2 launch serial_arm_ros2_control display.launch.py \
  robot_profile:=my_arm_default
```

确认：

```text
URDF
TF
Joint axis
Joint limits
Mesh
```

全部正常后才启动：

```bash
ros2 launch serial_arm_ros2_control hardware.launch.py \
  robot_profile:=my_arm_default
```

并检查：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

### 19.15. MoveIt 最后再接

MoveIt 不属于 Core 接入前置条件

先完成：

```text
Core Config
Hardware Backend
Terminal
真机基础控制
ros2_control                    # 如果需要 ROS 2
```

再创建或整理 MoveIt config package

最后给 Profile 增加：

```yaml
moveit:
  package: my_arm_moveit_config
```

然后：

```bash
ros2 launch serial_arm_ros2_control moveit.launch.py \
  robot_profile:=my_arm_default
```

不要使用 MoveIt 来替代：

```text
Joint direction 验证
零位验证
Safety limit 验证
Hardware Backend 验证
```

### 19.16. 新机械臂接入完成判据

新增一台机械臂至少应完成以下闭环

```text
[ ] description resource package 可以安装
[ ] Robot Profile 可以解析
[ ] URDF Joint 和 Core joint_names 完全一致
[ ] Core YAML 可以通过配置检查
[ ] Hardware YAML 可以被现有 Backend 加载
[ ] HardwareCapabilities 与实际执行器一致
[ ] Python Terminal --check-only 通过
[ ] Dynamics FK / Jacobian 软件检查通过
[ ] Joint / Actuator direction 与 zero 校准完成
[ ] write_enabled=false 下可以完成离线检查
[ ] 单关节低速真机测试通过
[ ] RIGID_HOLD / RIGID_TRACKING 工作正常
[ ] shutdown / park 行为验证完成
```

如果使用 ROS 2，再增加：

```text
[ ] display.launch.py 模型正确
[ ] ros2_control interfaces 正确
[ ] joint_state_broadcaster 正常
[ ] joint_trajectory_controller 正常
```

如果使用 MoveIt，再增加：

```text
[ ] MoveIt planning group 正确
[ ] planning scene 模型正确
[ ] 规划轨迹与真实 Joint 方向一致
```

到这里才算完成一个新的 Robot Support

---

## 20. 第十八步：新增 Hardware Backend

只有厂商协议发生变化时才应该新增 Backend

Core 不应该感知 Damiao、RealMan 或其他厂商协议

新 Backend 必须实现 `MotorBus`

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

Backend 对 Core 的状态输出必须统一为

```text
position -> rad
velocity -> rad/s
torque   -> N*m
```

Backend 从 Core 接收完整控制语义

```text
position
velocity
torque
kp
kd
```

如果厂商 SDK 使用编码器计数、电流、转速 RPM 或其他单位，应在 Backend 内转换

不要让这些厂商单位进入 Core

---

## 21. 第十九步：调试 Safety

Safety 不是最后才加的保护层

它参与每个 ACTIVE 控制周期

重点参数

```yaml
safety_policy:
  position_margin: 0.0
  cmd_vel_scale: 1.0
  state_vel_scale: 1.0
  max_dt_s: 0.02
  state_timeout_s: 0.05
  cmd_timeout_s: 0.10
  require_all_actuators_online: true
  require_all_actuators_enabled: true
  reject_motor_error: true
  require_continuous_cmd: false
```

建议首次真机时主动收窄，而不是放宽限制

例如

```yaml
cmd_vel_scale: 0.2
state_vel_scale: 0.5
```

前提是这些值满足配置校验规则并且不会试图放宽底层物理限制

Safety 最终限制来自多个来源的交集

```text
URDF limits
    ∩
HardwareCapabilities
    ∩
Calibration mapping
    ∩
Safety policy
```

Safety policy 只能收窄限制，不能扩张物理能力

---

## 22. 第二十步：处理 FAULT

Robot 进入 `FAULT` 后不要继续发送正常 tracking command

先读取

```cpp
const auto& fault = robot.get_last_fault();
```

确认故障来源

```text
RobotErr
MotorBusErr
JointActuatorMapErr
JointCtrllerErr
SafetyFault
ModelFeedforwardErr
```

如果 Robot 正在执行 fault hold，应持续调用

```cpp
robot.maintain_fault_hold();
```

恢复流程应按故障性质选择

### 22.1. 返回故障刚性保持

```cpp
robot.return_to_fault_rigid_hold();
```

### 22.2. 人工进入受限柔性恢复

只有配置允许、重力模型已验证并且当前故障类型允许时才能使用

```cpp
robot.enter_fault_compliant_recovery();
```

### 22.3. 清除故障

```cpp
robot.clear_fault();
```

`clear_fault()` 不是无条件复位

它要求当前状态重新可信，关节速度足够低，并且已经积累连续合法的恢复周期

如果条件不满足，可能返回 `FAULT_RECOVERY_NOT_ALLOWED`

如果目标只是无条件退出到失能状态，使用

```cpp
robot.force_deactivate();
```

---

## 23. 推荐的完整开发顺序

新机械臂或新 Backend 建议严格按以下顺序推进

```text
1. Core 编译
2. Core tests
3. URDF
4. Robot Profile
5. Core YAML
6. Hardware YAML
7. HardwareCapabilities
8. config check
9. Dynamics FK / Jacobian
10. Safety limit
11. Joint / Actuator mapping
12. write_enabled=true
13. 单关节小幅运动
14. RIGID_HOLD
15. RIGID_TRACKING
16. COMPLIANT_HOLD
17. COMPLIANT_DRAG
18. gravity compensation
19. COMPLIANT_TRACKING
20. ros2_control
21. MoveIt
22. 上层任务
```

这个顺序的目的不是增加流程，而是让每个错误都能在最小范围内定位

---

## 24. 常见问题定位

### Profile 找不到

检查

```bash
echo "$SERIAL_ARM_RESOURCE_PATH"
```

确认

```text
install/standalone/share/serial_arm_robot_profiles/config/robot_profiles.yaml
```

存在

如果显式传入 `--profile-file`，resolver 只使用该文件，不会自动 fallback 到其他 Profile 文件

### Hardware Backend 找不到

检查

```bash
ls install/standalone/lib/libserial_arm_hardware_damiao.so
```

再检查

```bash
echo "$LD_LIBRARY_PATH"
```

### ROS 2 launch 报 `failed to import serial_arm Python binding`

先确认已经重新构建并 source 该 workspace：

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

检查 Python binding：

```bash
python3 -c "import serial_arm; print(serial_arm.__file__); from serial_arm import load_robot_profile_core; print('OK')"
```

再检查实际安装文件：

```bash
find install/serial_arm_core \
  \( -name '__init__.py' -o -name '_serial_arm*.so' \) \
  -print
```

`__init__.py` 与 `_serial_arm*.so` 应位于同一个 `serial_arm` Python package 中，并由 `source install/setup.bash` 加入 Python 搜索路径

如果错误内容是 `No module named numpy`，通过 rosdep 或系统包补齐 `python3-numpy`；如果显式以 `-DSERIAL_ARM_BUILD_PYTHON=OFF` 构建，则基于 `robot_profile` 的 ROS 2 launch 本身不可用

### `Robot::activate()` 返回 `WRITE_DISABLED`

这是预期保护行为

修改 Core YAML

```yaml
write_enabled: true
```

前提是已经完成真机前检查

### `set_cmd()` 返回 `CMD_NOT_ALLOWED_IN_MODE`

当前模式不是 tracking mode

切换为

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::RIGID_TRACKING);
```

或者

```cpp
robot.set_impedance_mode(
    JointImpedanceMode::COMPLIANT_TRACKING);
```

### tracking 运行一段时间后进入 FAULT

优先检查 `cmd_timeout_s`

持续 tracking 必须持续刷新 `set_cmd()`

### `Dynamics::get_frame_pose()` 返回 `FRAME_NOT_FOUND`

检查请求 frame 是否存在于 URDF

并检查 `base_frame` 是否正确

### 重力补偿方向错误

不要直接放大增益

依次检查

```text
URDF joint axis
direction
tor_ratio
电机力矩正方向
gravity vector
inertial
```

### 拖拽仍然很硬

检查

```yaml
compliant_drag.kp
compliant_drag.kd
```

再检查是否存在机械摩擦、减速器阻力或电机自身控制器刚度

不要把所有问题都归因于 Core 阻抗参数

---

## 25. 下一步应该读什么

需要写 C++ 控制程序时，直接阅读 [API.md](API.md) 中的 `Robot`、`JointCmd` 和错误处理章节

需要写 Python 实验脚本时，直接阅读 [API.md](API.md) 中的 `RobotSession` 章节

需要开发运动学或控制算法时，直接阅读 [API.md](API.md) 中的 `Dynamics` 章节

需要开发新电机驱动时，直接阅读 [API.md](API.md) 中的 `MotorBus` 与 `HardwareLoader` 章节

需要接 ROS 2 时，先完成本教程的 standalone 与真机小幅验证，再进入 `serial_arm_ros2_control`
