# 配置与机器人接入

## Profile

Profile 组合 Core、Hardware、模型、Controllers 与 MoveIt 资源

内置配置位于 `src/robot_supports/profiles/config/robot_profiles.yaml`

| Profile | Core YAML | MoveIt Package |
| --- | --- | --- |
| `dm_arm_gray` | `config/core/gray.yaml` | `dm_arm_no_gripper` |
| `dm_arm_white` | `config/core/white.yaml` | `dm_arm_with_gripper` |

资源通过 package 名和相对路径解析，机器人身份与标定参数由自己的 Profile 定义

下游机器人在自己的项目中维护 `robot_profiles.yaml`，不用合并到 Core 内置列表

```bash
./install.sh --preset ros2 --robot none --without-moveit --yes
export SERIAL_ARM_RESOURCE_PATH=/absolute/path/downstream/workspace
./.install/run serial_arm_terminal \
  --robot-profile tomato_picker \
  --profile-file /absolute/path/downstream/robot_profiles.yaml
```

以上安装只提供通用组件，下游 Hardware Backend 和资源包仍需自行构建并加载

## 配置位置

| 内容 | 字段或文件 | 需要核对 |
| --- | --- | --- |
| 模型 | Core YAML 的 `model` | URDF、受控关节及顺序、base_frame、tool_frame |
| 标定 | `calibration.joints` | 方向、位置和力矩比例、关节和执行器零位 |
| 运行 | `control.runtime` | 频率、执行器写入、模型前馈与跟踪模式 |
| 阻抗 | `control.controller` | 刚性、柔性、拖拽模式的 kp/kd |
| 安全 | `safety_policy` | 状态和命令超时、限位、恢复策略 |
| 停机 | `shutdown` | 停放目标与超时 |
| 导纳 | `capability.admittance` | 观测器、标定、M/D/K 与限制 |
| 硬件 | Hardware YAML | plugin、Bus、设备、执行器 ID 与能力 |
| ROS2 | Profile 的 `controllers` | 控制器配置与关节匹配 |
| MoveIt | Profile 的 `moveit` | SRDF、规划关节与控制器匹配 |

`tool_frame` 使用实际末端坐标系，不按自由度数量猜测最后一个 link 名

Gray 与 White 的标定值只适用于对应实例，新机械臂需要自己的限位、标定和增益

## 检查与启动

先检查资源和权限，不连接执行器

```bash
./launch.sh --doctor --profile dm_arm_gray
```

Python Terminal 还提供模型、配置与 Hardware capability 检查，不连接或写入真机

```bash
./.install/run python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray --check-only
```

Core YAML 中的写入开关

```yaml
control:
  runtime:
    write_enabled: false
```

Terminal 在 false 时使用离线 mock 后端，在 true 时使用实际 Hardware Backend

内置配置可能已经开启写入，启动前查看实际值，GUI 确认不会改变这个开关

串口、波特率与 Bus 可以仅覆盖当前运行

```bash
./.install/run serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1 \
  --baudrate 921600 \
  --bus main_can
```

## 控制与调参

Tracking 模式需要持续刷新命令，保持与拖拽使用对应的阻抗模式

关节阻抗参数 kp/kd 是关节侧刚度与阻尼，Backend 再转换为执行器命令

关节导纳使用估计外力矩生成参考偏移，与内层阻抗增益分别配置

调试顺序为模型与映射 → 安全约束 → 基础保持与跟踪 → 重力标定 → 阻抗 → 导纳

正常结束等待程序完成停放与清理，FAULT 或通信中断时软件不能保证保持力矩

类型、模式、错误码和调用示例见 [API 参考](reference/api.md)
