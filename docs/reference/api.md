# SerialArm-Core API Reference

应用优先使用 C++ `Robot` 或 Python `RobotSession`；离线模型计算使用 `Dynamics`；驱动扩展从 `MotorBus` 与 Transport 接口开始

配置与操作流程见 [使用指南](../guide.md)，构建与安装见 [安装说明](../install.md)，模块职责见 [开发指南](../development.md)

## 按任务查找

| 任务 | 入口 |
| --- | --- |
| 建立第一个 C++ 应用 | [C++ 配置示例](#c-配置示例) |
| 从 Python 完成一次移动 | [Python 运动示例](#python-运动示例) |
| 查单位、向量顺序与命令结构 | [基础类型](#基础类型) |
| 读取 Profile 与四用途增益 | [配置](#配置) |
| 管理使能、周期、目标和快照 | [Robot 与 ControlRuntime](#robot-与-controlruntime) |
| 计算位姿、Jacobian、重力和惯性 | [Dynamics](#dynamics) |
| 使用模型导纳与外力观测 | [导纳](#导纳) |
| 处理限位、超时和故障恢复 | [Safety 与 FAULT](#safety-与-fault) |
| 接入新执行器或共享总线 | [驱动与 Transport](#驱动与-transport) |
| 查完整签名与错误枚举 | [公共接口索引](#公共接口索引) |

## 快速示例

### C++ 配置示例

下面从已安装的 `dm_arm_gray` Profile 加载 Backend、配置和模型，构造 `Robot`；示例不调用 `activate()`，可用于核对资源与接口连接

`Dynamics` 在 `Robot` 之前构造，确保回调引用在 Robot 生命周期内有效；配置字段与路径由所选 Profile 提供

```cpp
#include <iostream>
#include <memory>

#include "serial_arm/config/config.hpp"
#include "serial_arm/config/robot_profile.hpp"
#include "serial_arm/dynamics/dynamics.hpp"
#include "serial_arm/hardware/hardware_loader.hpp"
#include "serial_arm/robot.hpp"

using namespace serial_arm;

int main() {
    auto profile = load_robot_profile_core("dm_arm_gray");
    if(!profile) {
        std::cerr << profile.error().message << '\n';
        return 1;
    }

    HardwareLoader loader;
    auto bus = loader.load(profile->hardware_plugin, profile->hardware_config_path);
    if(!bus) return 1;

    auto config = load_robot_cfg(profile->core_config_path, (*bus)->capabilities());
    if(!config) {
        std::cerr << config.error().message << '\n';
        return 1;
    }

    Dynamics dynamics;
    if(!dynamics.configure(config->dynamics)) return 1;
    const auto n = config->joint_names.size();

    ModelFeedforwardFn feedforward = [&dynamics, n](
        ModelFeedforwardMode mode, const JointState& measured,
        const JointVector& acc, const JointVector& ref_acc, double)
        -> tl::expected<JointVector, ModelFeedforwardErr> {
        if(!dynamics.update(measured, acc, ref_acc)) {
            return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
        }
        switch(mode) {
        case ModelFeedforwardMode::NONE:
            return JointVector(n, 0.0);
        case ModelFeedforwardMode::GRAVITY:
            return dynamics.get_gravity_compensation();
        case ModelFeedforwardMode::FULL_INVERSE_DYNAMICS:
            return dynamics.get_inverse_dynamics();
        }
        return tl::make_unexpected(ModelFeedforwardErr::INVALID_MODE);
    };

    InteractionModelStateFn model_state = [&dynamics, n](
        const JointState& measured, double)
        -> tl::expected<InteractionModelState, ModelFeedforwardErr> {
        const JointVector zero(n, 0.0);
        if(!dynamics.update(measured, zero, zero)) {
            return tl::make_unexpected(ModelFeedforwardErr::COMPUTE_FAILED);
        }
        InteractionModelState value;
        value.gravity = dynamics.get_gravity_compensation();
        value.coriolis = dynamics.get_coriolis();
        value.mass_matrix.assign(n, JointVector(n, 0.0));
        const auto& mass = dynamics.get_mass_matrix();
        for(std::size_t r = 0; r < n; ++r) {
            for(std::size_t c = 0; c < n; ++c) {
                value.mass_matrix[r][c] = mass(r, c);
            }
        }
        return value;
    };

    Robot robot;
    auto configured = robot.configure(
        *config, std::move(*bus), feedforward, model_state);
    if(!configured) return 1;
    std::cout << "Robot configured\n";
}
```

应用通过已安装的 CMake package 链接公共 Target

```cmake
find_package(serial_arm_core CONFIG REQUIRED)
add_executable(my_robot_app main.cpp)
target_link_libraries(my_robot_app PRIVATE
  serial_arm::robot
  serial_arm::dynamics
  serial_arm::config
  serial_arm::core
)
```

只做离线动力学时可仅链接 `serial_arm::dynamics` 与 `serial_arm::config`

### Python 运动示例

使用安装环境的 Python 运行；所选 Profile 的 `write_enabled` 决定真实 Backend 或离线 mock，实机运行前完成 [首次连接检查](../guide.md)

以下移动内置 `dm_arm_gray` 的旋转首轴 `0.05 rad`；其他机械臂按实际关节类型、限位与环境设置目标

```python
import time

import numpy as np
import serial_arm

with serial_arm.RobotSession.from_profile("dm_arm_gray") as arm:
    deadline = time.monotonic() + 2.0
    while True:
        snapshot = arm.snapshot
        if snapshot.last_error:
            raise RuntimeError(snapshot.last_error)
        if snapshot.valid:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError("No valid joint feedback within 2 s")
        time.sleep(0.02)

    target = np.array(snapshot.cycle.joint_state.pos, dtype=np.float64, copy=True)
    target[0] += 0.05
    motion_id = arm.move_to(target, speed_scale=0.15)

    deadline = time.monotonic() + 10.0
    while True:
        snapshot = arm.snapshot
        motion = snapshot.motion
        if snapshot.last_error:
            raise RuntimeError(snapshot.last_error)
        if motion.id != motion_id:
            raise RuntimeError("Motion was replaced by another request")
        if motion.state == "completed":
            print("目标已到达" if motion.target_reached else "动作已结束，存在到位偏差", motion.position_error)
            break
        if motion.state in ("interrupted", "failed"):
            raise RuntimeError(motion.reason)
        if time.monotonic() >= deadline:
            arm.hold_current()
            raise TimeoutError("Motion did not settle within 10 s")
        time.sleep(0.02)

    # 真机应用在此完成自己的停放流程与支撑确认，再退出上下文并失能
```

`move_to()` 返回运动编号表示请求已接收；同一编号的 `completed` 表示参考已完成，且新鲜位置反馈的运动窗口已经稳定；`target_reached` 另行说明是否在原关节类型位置容差内，`position_error` 保留实际位置减目标的位置偏差；稳态静差不会让请求永久处于 `running`，也不会被写成目标已到达；稳定速度采用250毫秒新鲜位置窗口估计并检查行程范围，保留150毫秒持续稳定确认，不依赖固定 `sleep()`；原始速度、控制命令、位置容差与 Safety 均不因此改变

`stop()` 和上下文退出停止线程并失能，不执行停放轨迹；机械臂失能前应有可靠支撑

## 基础类型

<a id="2-基础类型与命令"></a>

声明见 [types.hpp](../../src/serial_arm/core/include/serial_arm/core/types.hpp)；所有关节向量长度与顺序均对应 `RobotCfg::joint_names`，数值必须有限

| 数据 | 内容 |
| --- | --- |
| `JointVector` / `ActuatorVector` | `std::vector<double>`，分别表示关节侧与执行器侧数据 |
| `JointState` | `pos`、`vel`、`tor` |
| `ActuatorState` | `pos`、`vel`、`tor`、`online`、`enabled`、`err_code` 与可选逐轴反馈时序 |
| `JointPosCmd` | 位置参考 `pos` |
| `JointPosVelCmd` | 位置与速度参考 `pos`、`vel` |
| `JointPosVelTorCmd` | 位置、速度与附加力矩参考 `pos`、`vel`、`tor` |
| `JointCmd` | 上述三类参考命令的 `std::variant` |
| `JointCtrlCmd` | 关节侧完整 `pos`、`vel`、`tor`、`kp`、`kd` |
| `ActuatorCtrlCmd` | 映射后的执行器侧完整命令，Backend 将其转换为厂商协议 |

| 物理量 | 旋转／连续关节 | 直线关节 | 执行器侧 |
| --- | --- | --- | --- |
| 位置 | rad | m | rad |
| 速度 | rad/s | m/s | rad/s |
| 力矩／力 | N·m | N | N·m |
| Kp | N·m/rad | N/m | N·m/rad |
| Kd | N·m·s/rad | N·s/m | N·m·s/rad |

GUI 的 °／mm 显示不改变配置与 API 的 SI 单位；Backend 须先转换编码器计数、RPM、电流等厂商量，再交给 Core

`feedback_received_ns` 为逐轴 `steady_clock` 接收时间戳，空数组表示未知；`feedback_timing_valid` 为可选的时序连续性标志，失效数据不能用于速度微分

## 配置

<a id="3-robot-profile-与配置"></a>

### 加载与校验

| 接口 | 用途与契约 |
| --- | --- |
| `load_robot_profile_core(name, options)` | 解析 Profile，返回 Core YAML、Hardware YAML 与插件路径，不使能硬件 |
| `RobotProfileLoadOptions` | `profile_file` 显式指定索引，`resource_paths` 提供额外资源根目录 |
| `robot_profile_search_paths(options)` | 返回实际资源搜索路径，供路径错误诊断 |
| `load_robot_cfg(path, capabilities)` | 加载并校验 Core YAML，结合执行器能力解析安全范围 |
| `validate_robot_core_cfg(cfg)` | 校验无需硬件能力的配置部分 |
| `validate_robot_cfg(cfg)` | 校验完整配置，成功无返回内容 |
| `compare_robot_cfg(a, b, capabilities)` | 返回解析后的配置差异，空数组表示一致 |
| `ModelLoader::load(urdf, joint_names)` | 读取请求顺序的关节类型与 URDF 限位 |
| `LimitResolver::resolve(...)` | 合并 URDF、映射、硬件能力与用户策略，生成最终限位 |

精确参数见 [robot_profile.hpp](../../src/serial_arm/core/include/serial_arm/config/robot_profile.hpp)、[config.hpp](../../src/serial_arm/core/include/serial_arm/config/config.hpp)、[limit_resolver.hpp](../../src/serial_arm/core/include/serial_arm/config/limit_resolver.hpp) 与 [model_loader.hpp](../../src/serial_arm/core/include/serial_arm/model/model_loader.hpp)

`RobotCfg` 包含 `runtime`、`shutdown`、`dynamics`、`ctrller`、`mapper`、`safety`、`capability` 与 `hold_compensation`；手工组装后仍需校验，不以成功解析 YAML 代替完整配置检查

PD 配置不提供用户上限；旧 `safety_policy.max_kp_override`、`max_kd_override` 与平铺格式 `limits.max_kp`、`max_kd` 接受但完全忽略，包括字段值与维度校验；运行时 `safety.limits.max_kp/max_kd` 仍保留，表示硬件协议能力经映射后的边界，加载、配置及连接能力刷新都从当前硬件重新生成

`ActuatorCapability.kp_resolution/kd_resolution` 表示协议增益步进，0 表示连续或未声明；Joint 侧 `JointLimitCfg` 的同名数组按 `pos_ratio × tor_ratio` 映射；达妙 MIT 的执行器侧上限为 `500/5`，步进为 `500/4095`、`5/4095`；位置、速度、加速度、力矩与故障检查继续使用既有 Safety 规则

`ActuatorCapability.effort_resolution` 与 Python `ActuatorInfo.effort_resolution` 保存执行器侧力矩命令步长，0 表示未声明；达妙由已确认协议范围计算 `2 × TMAX / 4095`，关节命令步长按 `tor_ratio` 映射；它用于独立输入可分辨性规划，不等同于电机实际力矩测量精度，也不修改用户 Safety 限制

普通周期与故障保持共用增益投影；故障保持在投影后检查实际 PD 与前馈总力矩，超出既有力矩边界时返回 `REFERENCE_GOVERNOR_FAILED/INFEASIBLE`，经现有停止／失能路径处理

### 四用途与任务

公开增益用途为 `JointGainProfile::{RIGID, COMPLIANT, DRAG, SERVO_TRACKING}`；刚性和柔性各保存保持、跟踪两组增益，拖拽与伺服跟踪各保存一组

| Core YAML 路径 | C++ 配置字段 |
| --- | --- |
| `control.controller.rigid.hold` | `ctrller.rigid_hold_gains` |
| `control.controller.rigid.tracking` | `ctrller.rigid_tracking_gains` |
| `control.controller.compliant.hold` | `ctrller.compliant_hold_gains` |
| `control.controller.compliant.tracking` | `ctrller.compliant_tracking_gains` |
| `control.controller.drag` | `ctrller.compliant_drag_gains` |
| `control.controller.servo_tracking` | `ctrller.servo_tracking_gains` |

保持与跟踪是任务阶段，四用途是参数组织方式；兼容枚举 `JointImpedanceMode` 保留以下五种任务

| 任务 | 接收 `set_cmd()` | 原生方式默认增益 |
| --- | --- | --- |
| `RIGID_HOLD` | 否 | 刚性保持 |
| `RIGID_TRACKING` | 是 | 刚性跟踪 |
| `COMPLIANT_HOLD` | 否 | 柔性保持 |
| `COMPLIANT_TRACKING` | 是 | 柔性跟踪 |
| `COMPLIANT_DRAG` | 否 | 拖拽，每周期以实测位置为位置参考 |

`ReferenceBehavior::NATIVE_IMPEDANCE` 直接跟踪名义参考；`MODEL_ADMITTANCE` 使用外力估计修正参考并由伺服增益执行，拖拽仍使用拖拽增益

`JointImpedanceGains` 的 `kp`、`kd` 必须逐轴、有限、非负且不超过最终 Safety 上限；Safety 上限是允许范围，不是推荐标定目标

### 运行、前馈与限制

- `runtime.ctrl_frequency_hz` 控制原生周期频率，`write_enabled` 控制硬件写入授权
- `ModelFeedforwardMode` 为 `NONE`、`GRAVITY` 或 `FULL_INVERSE_DYNAMICS`，非 `NONE` 需要有效模型回调
- `DynamicsCfg` 定义 URDF、关节顺序、基座／工具 Frame、重力向量、逐轴 `gravity_scale` 与可选重力校正包，`gravity_scale` 范围为 `[0, 2]`
- `ShutdownCfg` 描述上层停放策略，配置它不会让 `Robot::deactivate()` 或 `RobotSession.stop()` 自动执行轨迹
- `SafetyPolicyCfg` 的 override 与比例只收窄 URDF 和硬件能力的交集，不放宽物理限制
- continuous 关节可以没有绝对位置限位，不能人为按 `[-pi, pi]` 截断
- Backend 提供完整且唯一的 `joint_name` 时，Core 按关节名对齐能力、反馈和命令；部分缺名、重名或不匹配会被拒绝，所有轴均无名的旧 Backend 则必须已按 Core 顺序排列

## Robot 与 ControlRuntime

<a id="4-robot-控制闭环"></a>
<a id="robot"></a>
<a id="controlruntime"></a>

声明见 [robot.hpp](../../src/serial_arm/core/include/serial_arm/robot.hpp) 与 [control_runtime.hpp](../../src/serial_arm/core/include/serial_arm/runtime/control_runtime.hpp)

### 生命周期与周期

| 操作 | 条件与结果 |
| --- | --- |
| `configure(cfg, bus, feedforward, model_state)` | 接管 `std::unique_ptr<MotorBus>`，成功后 `UNCONFIGURED → INACTIVE`，拒绝重复配置 |
| `activate()` | 连接并使能 Backend，以真实反馈初始化控制器，成功后进入 `ACTIVE` |
| `set_cmd(cmd, now)` | ACTIVE 跟踪任务提交参考，时间戳须单调，保持／拖拽拒绝该命令 |
| `set_full_cmd(cmd, now)` | 原生方式且 `allow_full_cmd=true` 才可直接覆盖完整关节命令，仍经过 Safety |
| `cycle(now)` | 执行一次完整读状态、控制、检查、映射与写命令，返回 `RobotCycleOutput` 或 `RobotFault` |
| `deactivate()` | 正常停止并失能，成功后回到 `INACTIVE` |
| `force_deactivate()` | ACTIVE 或 FAULT 中停止并失能，失败仍返回错误 |

Robot 析构尽力失能并释放资源；应用应显式处理停止结果，不能把析构当作停放成功确认

同一 Robot 同时只允许一个周期所有者；直接使用 Robot 的应用负责按配置频率调度，并在同一串行上下文更新命令、模式与模型缓存

`ControlRuntime::run()` 管理固定频率周期；`step()` 执行单步并按 FAULT 状态维护故障保持；`submit_reference()` 提交带接收时间和 `MotionSource` 的参考；`snapshot()` 返回复制后的线程安全快照

`RuntimeHooks` 的 `before_cycle`、`after_step` 与 Robot 周期在相同串行上下文执行；使用 Terminal、Python 或 ROS2 适配层时，复用它们的原生周期，不另开第二个 Robot 控制循环

同一运动源的待处理参考由最新请求覆盖，不同 `MotionSource` 同时占用时返回 `MOTION_SOURCE_BUSY`；切换任务或 `clear_commands()` 释放来源，FAULT 也会清除旧参考

`ControlRuntime` 对非法调度参数抛 `std::invalid_argument`，对重复周期所有者抛 `std::logic_error`，调用方在启动时处理这些配置／所有权错误

### 模式与参数更新

| 接口 | 主要行为 |
| --- | --- |
| `set_impedance_mode(mode, now, target_policy)` | ACTIVE 中切换任务，清除旧跟踪命令、用途覆盖与保持积分，平滑交接增益和命令力矩 |
| `set_execution_gain_profile(profile)` / `clear_execution_gain_profile()` | 选择或清除原生用途覆盖，模型导纳与拖拽的用途优先 |
| `set_servo_tracking_gains(gains)` | 校验并更新伺服增益，ACTIVE 平滑交接 |
| `set_hold_compensation_cfg(cfg)` | 更新保持补偿，受尺寸、有限值及安全上限检查 |
| `set_reference_behavior(behavior)` | INACTIVE 时选择原生阻抗或模型导纳 |
| `set_model_feedforward_mode(mode)` | INACTIVE 时选择前馈策略 |
| `set_admittance_cfg(cfg)` | INACTIVE 配置，ACTIVE 仅允许同策略调参并平滑交接 |
| `set_admittance_suspended(bool)` | 临时旁路导纳修正并重锚到已接受参考，不修改持久配置 |

伺服增益和保持补偿在 FAULT 中只允许恢复合法配置，不恢复执行；模式切换清除用途覆盖，需要特定用途时应在切换后选择

普通保持默认使用 `HoldTargetPolicy::PRESERVE_NOMINAL`，显式捕获当前位置使用 `CAPTURE_MEASURED`，两者不能混用

```cpp
auto held = robot.set_impedance_mode(
    serial_arm::JointImpedanceMode::RIGID_HOLD,
    serial_arm::Robot::Clock::now(),
    serial_arm::HoldTargetPolicy::CAPTURE_MEASURED);
if(!held) return 1;
```

标定使用 `set_calibration_reference_behavior()`、`set_calibration_model_feedforward_mode()` 及临时保持／跟踪增益覆盖；这些试验上下文不改持久配置，调用方需在成功、取消与异常路径清除覆盖并恢复原会话

`set_calibration_disturbance(torque, now)` 为独立扰动短测提供有界关节力矩入口，`clear_calibration_disturbance()` 清除该输入；扰动仍经过原有力矩、参考与 Safety 约束，调用方需按周期刷新并在所有退出路径清除；周期输出的 `calibration_disturbance` 记录本周期实际叠加的扰动，不以请求值代替执行值

HOLD 扰动保留 `get_hold_target()` 返回的名义目标，只刷新独立扰动；`ControlRuntime::submit_reference()` 在 HOLD 中仍返回 `CMD_NOT_ALLOWED_IN_MODE`；新候选的 `available` 表示参数测量完整，`qualification_complete` 保留证据范围完整度；`validated` 为旧接口兼容字段，新任务固定为 false，不能把仅供参考的指标提升为精度通过，见 [时间与结果语义](calibration.md#时间与结果语义)

自动增益的模型初始化、MIT 闭环短测、候选与预算定义见 [标定参考](calibration.md#自动增益的模型与证据)；旧 `identify()` 的无零点响应假设不作为 MIT 参考速度与前馈试验的物理带宽解释

`model_calibration_gain_plan` 与新任务候选的 `target_budget_s/time_budget_s` 使用同一可信原始 `budget_source_duration_s`：四用途 `gain_mode=all/gain_tuning_mode=quick` 为目标 18T、硬上限 24T，单用途 `gain_tuning_mode=fine` 为目标 12T、硬上限 15T；重定时、导入、补测与局部比较不增加 T，历史候选按原有预算字段读取，不进行倍率迁移

`gain_stage_budget_reservation` 事件保存 `purpose/future_measurements_reserved_s/scope`，说明进入当前用途后仍保留给后续学习、用途比较、固定组测量和退出的成本；预留不等于已经执行或已经测全，实际阶段仍以对应 CSV 和测量结果为准

新增益任务保存 `acceptance_scope=measured_response_reference_only_v1`、`reference_only=true` 和 `accuracy_thresholds_enabled=false`，取消精度／误差验收阈值；过程峰值、RMS、滞后和 `endpoint_*` 最终目标窗数据均为参考指标；旧 `gain_requirements` 误差目标参数兼容读取但不再用于新任务的流程门槛或通过评分；`available` 继续表示实际完整参数测量，新任务 `validated` 与 `endpoint_passed` 等兼容字段固定为 false；历史标志与范围按原记录保留，不将参考分析自动补成通过；新 Profile 初始化凭据保存 `reference_only=true`、`value_role=unmeasured_model_reference` 和 `accuracy_thresholds_enabled=false`，双对角化初值算法不变

运动中整定阶段为 `trajectory_adapting`；`online_trajectory_PD_update` 保存 `time_s/source_record_path`、`previous_kp/kd`、`proposed_kp/kd`、`candidate_generated/admission_allowed/installed`、逐轴模型与原因；`installed` 只表示目标组已交给平滑切换，实时有效增益仍读取 `RobotCycleOutput.joint_cmd.kp/kd`；`native_validation.online_adaptation=false` 才是最终冻结整组测量，变增益学习记录不会提前提升为 `available`；整轨 CSV 新增 `planned_independent_disturbance_nm/independent_input_planned_known`，用以核对已知独立输入是否被实时约束削减，缺少这些列的旧数据按旧格式读取

在线模型另外保存 `fresh_samples/command_samples/resampled_samples`、`feedback_interval_min_s/max_s`、`observed_encoder_step_rad` 与 `fit_attempts/rank_rejections/plausibility_rejections/prediction_rejections`；分别区分原始输出、真实命令和等间隔建模点，以及信息秩、作用方向／动态系数与留出预测问题；尚未执行的训练／留出预测 RMS 为 `null`，不以默认零误差填补缺测；旧事件缺少这些字段时界面显示未知

途中用途测量阶段为 `trajectory_purpose_pass`；`taught_path_purpose_dwell` 保存原路径 `source_index/reverse/reference_rad`、`source_duration_s/source_time_s`、实际 `elapsed_s/pause_begin_s/pause_end_s/measurement_dwell_s`、`resumed/completed` 与主记录路径；`arrival_error_gate=false/exact_anchor_arrival_required=false` 明确新鲜反馈不受精确到位门槛阻断；经过该内点时平滑暂停源时钟，测完后继续原方向，最终固定组测量另行保存

内点记录增加 `nearby_response_motion_score/nearby_reference_span_rad/axis_response_weights`；权重来自本次轨迹实测误差与噪声尺度，行程来自候选附近原始示教参考，不把预测能力写成实际反馈运动

用途回调结束后的 `gain-transition-*-passing-resume.csv` 保留接受参考到原示教内点的实际交接；提交跟踪参考前先恢复对应跟踪模式、用途参数和参考行为，允许回调以保持或拖拽模式返回；固定组最终测量仍以独立完成的全程记录为准

用途测量期间中断时 `completed=false/resumed=false`，`pause_end_s` 与尚未提交到源时钟的 `measurement_dwell_s` 可为0；已有有效 `pause_begin_s` 时，以 `elapsed_s-pause_begin_s` 展示中断前已停留用时，保留部分记录，不显示为停留0秒或已经恢复轨迹

`gain-trajectory-{native|compliant}-passing-purpose.csv` 的 `trajectory_source_time_s` 仅表示示教源进度；`time_s/command_time_s/reference_sample_time_s/feedback_time_s` 保留真实墙钟，单独用途测量期间主轨迹 CSV 保留实际采集间隙，不能跨该间隙假装持续运动；用途输入与保持仍使用各自原始 CSV；新快标计划 `max_path_passes=6`、单用途精调为4，单独拖拽继续沿原路径访问内点而不产生整轨测量

`structure/parameter_count/training_span_rad/validation_error_limit_rad` 保存所选局部模型与训练窗预测尺度；用途成本区分跟踪和柔性；`taught_measurement_anchor_visit` 记录原路径源索引、计划和实际成本、引用角及 CSV，`interior_purpose_measurement_skipped` 说明内点或预算缺项且不默认回端点扰动；`rigid_purpose_local_comparison` 与 `compliant_purpose_local_comparison` 保留整组比较，`kept` 不代表性能认证；共享扰动用 `shared_disturbance_evidence` 引用实际源记录

`taught_local_correction_plan` 保存同向短示教段的 `source_index_from/to/reverse/playback_rate/duration_s/complete_comparison_cost_s` 与选择依据记录；`taught_local_complete_group_comparison` 保存实际两组完整 P/D、`changed_joint/coordinate`、共同前馈与负载声明、两条原始 CSV、逐轴前后指标及 `kept`；`reason` 表示提出候选的依据，`comparison_reason` 单独说明实测采用或拒绝的原因，旧记录缺少该字段时不从提案原因推测拒绝结论；初态不一致或未改轴入口偏置时，`taught_local_comparison_repeat` 保存原两份记录、`bounded_repeats=1/raw_metrics_preserved=true`；预算允许才补测完整两组，保留原始误差，不扣除静差后宣布改善；两类事件均保持 `model_identified=false`，不把直接实测比较伪装成局部模型成立；原始 CSV 继续使用 `gain-trajectory-local-*.csv`，沿现有轨迹导出入口保存；局部行程计入任务预算但不计整轨趟数，最终冻结整轨另测

用途短测使用逐轴隔离的已知输入，固定完整 P/D、参考中心和前馈条件；逐周期记录保留被激励轴及其他轴的反馈，输入基线、加载与释放分别解释；直接位移／恢复测量的完整性与参数模型有效性分开，模型未知不删除已完成的直接测量，未发生可分辨运动也不补成刚度、频响或释放测量

用途参数事件沿用 `short_experiment_parameters`，以 `waveform=isolated_smooth_pulse`、`isolation=sequential_single_axis` 和 `pulse_axes` 描述已执行的逐轴窗口；每项保存 `axis_index/attempt`，同轴补测独立成项，`start_s/release_s/end_s` 与同一记录时钟对应；未测轴在逐轴直接测量结果中保留缺项原因；`independent_torque_disturbance` 的 `method=isolated_smooth_torque_pulse`、`model_identified=false` 与直接输入／响应字段分别保存；用途结果的 `disturbance_verification.direct_measurement_complete` 要求各轴直接窗口完整且用途响应 `measurement_complete`；仅有完整输入、响应低于分辨能力时该汇总及相应用途 `qualification_complete` 保持 false，完整参数组仍可 `available`；直接响应测全不等同参数模型成立

用途脉冲参数的 `center_source=completed_taught_anchor_nominal_reference_not_actual_rezero`、`center_rad` 与 `entry_actual_rad` 区分名义参考和实际入口；桌面展示两者以及目标减入口的偏差，旧记录没有这些来源字段时不自动声称使用了示教名义目标

`purpose_input_prior` 事件通过 `record_path` 关联本次用途脉冲，以 `input_priors` 逐轴保存 `valid/independent_samples/low_speed_pd_samples/rejected_samples`、`independent_peak_nm/low_speed_pd_level_nm/seed_nm/low_speed_cutoff_rad_s`、`source/reason/interpretation`，另存实际计划 `planned_input_nm/current_domain_input_ceiling_nm`；`input_allowance_source` 明示既有 Robot 独立注入边界来自用户 Safety 力矩限制的 5%，计划、补测和固定输入复测使用同一边界；原始已知输入与低速主机 PD 尺度分列，来源指向实际轨迹 CSV；无有效先验时保留不足原因，界面不显示默认零值为已测成功

`purpose_input_supplement_decision` 保存本轴 `attempt/previous_input_nm/next_input_nm/reason/next_window_planned`；拟安排不代表已经执行，实际输入与起止时刻仍以 `purpose_pulse_window_parameters`、汇总 `pulse_axes` 和逐周期 CSV 为准；各次补测在同一原 T 预算内，不因新增窗口延长任务上限

`gain-purpose-*.csv` 保存同一固定完整组下的所有轴控制周期，`gain-anchor-*.csv` 保存内点访问／返回；两者与 `drag-decay-*.csv`、`drag-taught-release-*.csv`、`gain-trajectory-*-passing-purpose.csv` 一并进入桌面记录文件清单、任务记录 ZIP 和候选成果 ZIP，导出保留原始 CSV 字节，不修改源任务

拖拽安静窗口必须来自实际输出已完成 Kp＝0／Kd 交接且独立输入为零的周期；静置基本参数结果与可选释放证据分别保存，释放未测不清除已完成的安静结果；新的用途测量仍沿用原 T 预算、用途选择、完整参数组应用与恢复语义

`drag_damping_comparison_conditions` 保存比较姿态的 `comparable/reason/relative_M_change/max_gravity_change_nm`，以完整惯量和已执行重力项判断工况是否可比；`arrival_error_gate=false` 不要求回到同一个编码值；工况未知或变化过大则保留原阻尼，预算不足时保存 `drag_damping_comparison_skipped`

拖拽逐轴 `drag_input_plan`、`drag_input_window` 和 `drag_input_supplement_decision` 分别保存计划、实际已执行窗口和补测决定，包含 `joint/reason/scheduled/input_nm/next_input_nm/input_ceiling_nm/torque_quantum_nm/prior_valid/prior_input_nm/prior_source/duration_s`；未安排输入必须保留实际缺项原因，安静结果不覆盖该原因；`scheduled` 与正幅值均不等于已测得物理释放运动，是否可分辨释放仍读取 `release_decay_measured`

`drag_demonstrated_motion_release` 保存附近示教运动释放的 `source_index_from/source_index_release/preparation_duration_s/actual_zero_stiffness_s/elapsed_s/domain_stopped` 和逐轴测量原因；`actual_zero_stiffness_s` 是实际完整 P0／Kd 交接完成时刻，未知时为 `null`；`drag-taught-release-*.csv` 分别保留正刚度准备与零刚度释放周期；拖拽逐轴 `release_record_path` 指向实际采用的释放证据，`demonstrated_motion_preparation` 区别示教运动准备与独立力矩脉冲，不覆盖原始脉冲记录或安静窗口

保持结果的 `hold_evaluation_scope` 区分内点固定目标分析与整轨终点定位；`target_source=completed_taught_anchor_nominal_reference_not_actual_rezero` 表示保留已发送的示教内点，`target_pos` 与 `entry_actual_pos` 分开保存；`hold_parameters` 同样保存这些入口证据，不以到位误差决定能否测量；增加 `controls_known/reference_fixed/independent_input_zero`，保持 CSV 的 `kp/kd` 为实际周期值并附 `actual_gains_known`；参数过渡与积分建立期继续保留全过程异常运动记录，最终固定参数窗口单独检查；积分尝试的误差窗口按已观测保持静差设置并记录，持续冻结且没有补偿输出时可提前结束该尝试，以 `hold_integral_blocked:<原因>` 保留实际参数和冻结记录，不删除已有基本保持结果

机器接口的增益任务状态与候选使用同一 `termination` 对象，包含 `kind`、`reason`、`blocked_joint_names`、`fault_interrupted`；新任务基本参数测全为 `measured_reference_complete`，缺项为 `insufficient_measurement`，正常无故障测量阶段为 COMPLETE；终止分类及取消语义见 [时间与结果语义](calibration.md#时间与结果语义)；`motion_establishment_probe` 是单频运动证据，逐轴计划 `motion_probe=true` 时只执行频率数组首项，不能作为三频拟合或用途通过依据

### 周期输出与读取

`get_joint_state()`、`get_actuator_state()`、`get_joint_acc()`、`get_joint_ref_acc()`、`get_model_feedforward()`、`get_interaction_state()` 读取最近周期缓存；引用会随后续周期更新，跨线程读取应使用适配层快照

| `RobotCycleOutput` 字段 | 用途 |
| --- | --- |
| `joint_state` / `actuator_state` | 本周期关节与执行器测量 |
| `dt` / `feedback_age_ms` / `feedback_skew_ms` | 周期和反馈时序 |
| `joint_acc_valid` / `new_measurement` / `measurement_dt` | 判断测量、微分与观测数据是否可用 |
| `gain_profile` / `gain_source` / `effective_kp` / `effective_kd` | 实际执行用途、来源与协议可编码增益 |
| `requested_kp` / `requested_kd` | 当前周期投影到协议网格前的请求增益；`joint_cmd`、`actuator_cmd` 保留实际可编码值 |
| `hold_target_pos` / `hold_integral_torque` / `hold_integrator_frozen` | 保持目标和积分补偿状态 |
| `reference_behavior` / `parameter_update_progress` | 当前参考策略与参数交接进度 |
| `nominal_ref_*` / `virtual_ref_*` / `accepted_ref_*` | 名义、导纳虚拟与最终接受的参考 |
| `model_feedforward` / `predicted_total_torque` | 模型前馈与预测总力矩 |
| `joint_cmd` / `actuator_cmd` | Safety 后关节命令与实际发送的执行器命令 |
| `reference_limited` / `torque_limited` | 本周期参考与力矩限幅标志 |
| `observer_valid` / `admittance_active` / `interaction_state` | 观测器与导纳实际参与状态 |
| `residual_raw` / `full_id_residual_raw` / `bias_compensated` / `friction_compensated` / `tau_ext_hat` | 外力矩估计各阶段及对照值 |
| `delta_q` / `delta_q_dot` / `delta_q_ddot` 与逐轴限幅标志 | 导纳修正和剩余安全空间 |

`residual_filtered` 为兼容字段，当前与 `residual_raw` 相同；读取数值前检查相应有效标志，不将缓存默认值视为本周期测量

### 独立 JointCtrller

<a id="6-impedance-与-jointctrller"></a>

[JointCtrller](../../src/serial_arm/core/include/serial_arm/core/joints_ctrller.hpp) 单独使用时遵循 `UNCONFIGURED → configure() → CONFIGURED → initialize(state) → INITIALIZED`，随后调用 `update()` 生成关节命令

`reset()` 清理初始化状态和命令并回到 CONFIGURED，不重新读取配置；再次计算前调用 `initialize()`；底层保持任务直接捕获传入实测位置，上层 Robot 另按 `HoldTargetPolicy` 管理名义保持目标

独立控制器不承担硬件连接、映射和完整 Robot 安全闭环，模块集成方仍需处理这些环节

## Dynamics

<a id="5-dynamics"></a>

声明见 [dynamics.hpp](../../src/serial_arm/core/include/serial_arm/dynamics/dynamics.hpp)；支持按指定顺序约简的单自由度关节，实例使用 `configure()` 后才能计算

| 接口组 | 用途 |
| --- | --- |
| `configure(cfg)` / `cleanup()` | 创建模型或恢复未配置状态，重复配置会报错 |
| `update(state, acc, ref_acc)` | 更新完整运动学与动力学缓存 |
| `update_state(state, acc)` / `update_reference(ref_acc)` | 分开更新实测模型和参考逆动力学 |
| `is_configured()` / `is_updated()` | 检查配置与首次成功更新 |
| `set_gravity_scale(values)` / `get_gravity_scale()` | 设置或读取逐轴重力缩放 |
| `get_info()` / `get_state()` | 模型信息与最近缓存 |
| `get_frame_pose(name)` / `get_frame_jacobian(name)` | 指定 Frame 的位姿或 `6×N` Jacobian 副本 |
| `get_gravity()` / `get_gravity_compensation()` | 原始重力与当前有效重力补偿 |
| `get_mass_matrix()` / `get_coriolis()` / `get_nonlinear()` | 质量矩阵、科氏／离心项与非线性项 |
| `get_inverse_dynamics()` / `get_forward_dynamics()` | 参考加速度对应逆动力学与反馈力矩对应正向动力学 |
| `get_tool_pose()` / `get_tool_jacobian()` / `get_state().center_of_mass` | 工具位姿、Jacobian 与整体质心 |
| 重力回归与候选接口 | 查询 Link 参数、计算候选重力或应用临时覆盖，详见公共头文件与 [标定参考](calibration.md) |

位姿和质心相对于 `base_frame`；工具 Jacobian 使用 Pinocchio `LOCAL_WORLD_ALIGNED`，与其他坐标系数据相乘或比较前保持约定一致

`gravity` 为原始 URDF 未缩放重力，`candidate_gravity` 为候选重力，`effective_gravity` 为当前实际模型重力，`gravity_compensation` 与有效重力一致

缓存 Getter 返回只读引用，调用前检查 `is_updated()`；下一次更新或清理会改变缓存，独立读取时应复制所需结果

Python 离线计算示例，不创建运动会话

```python
import numpy as np
import serial_arm

profile = serial_arm.load_robot_profile_core("dm_arm_gray")
cfg = serial_arm.load_robot_cfg(
    profile.core_config_path, profile.hardware_plugin, profile.hardware_config_path
)
dynamics = serial_arm.Dynamics()
dynamics.configure(cfg.dynamics)
zero = np.zeros(len(cfg.joint_names), dtype=np.float64)
dynamics.update(zero, zero, zero, zero, zero)
print(dynamics.gravity_compensation)
print(dynamics.mass_matrix)
print(dynamics.tool_pose)
```

## 导纳

<a id="7-admittance"></a>

模型导纳按 `M * delta_q_ddot + D * delta_q_dot + K * delta_q = tau_ext_hat` 修正关节参考；M、D、K 为逐轴固定参数，不是执行器 Kp／Kd

| 配置／接口 | 契约 |
| --- | --- |
| `AdmittanceCapabilityCfg` | 总开关、参考方式、逐轴开关、观测器、标定与控制器参数 |
| `AdmittanceObserverCfg` | `FULL_ID` 使用实测加速度计算 residual，`MOMENTUM` 使用广义动量观测，不显式依赖实测加速度 |
| `momentum_gain` | MOMENTUM 一阶增益，单位 s⁻¹，逐轴且大于零 |
| `AdmittanceCalibrationCfg` | 固定 `torque_bias`、速度相关摩擦 residual 模型与 `torque_threshold` |
| `AdmittanceControllerCfg` | `mass`、`damping`、`stiffness`、`max_delta_q` 与 `max_delta_q_dot` |
| `JointAdmittanceController` | 独立关节导纳计算，`configure()` 后更新，接受状态后再用于下一周期 |
| `compute_admittance_damping_metrics()` | 计算二阶系统频率、阻尼比等诊断指标，不替代离散周期和执行器验证 |
| `InteractionModelStateFn` | 实测状态对应有效重力、科氏项与质量矩阵，供 MOMENTUM 观测器使用 |

正式外力估计经过 bias、摩擦补偿和 deadband；`full_id_residual_raw` 是对照诊断，不能当作正式输出替代 `tau_ext_hat`

启用模型导纳需要合法模型回调和配置；挂起导纳会旁路修正并清空观测与 delta 状态，恢复时从已接受参考重锚，正常模式切换不能把受扰退让位置误当作新的名义目标

运行时配置过渡保留已接受虚拟状态；Safety 限制对最终参考仍然有效，观测有效性和导纳是否实际激活通过周期输出判定

完整定义见 [导纳配置](../../src/serial_arm/core/include/serial_arm/config/config.hpp)、[控制器](../../src/serial_arm/core/include/serial_arm/interaction/controllers/joint_admittance_controller.hpp)、[运行组合](../../src/serial_arm/core/include/serial_arm/interaction/runtime/interaction_controller.hpp) 与 [标定工具](../../src/serial_arm/core/include/serial_arm/interaction/admittance_calibration.hpp)

## Safety 与 FAULT

<a id="8-safety-与-fault"></a>

声明见 [safety.hpp](../../src/serial_arm/core/include/serial_arm/core/safety.hpp)；安全检查分为实测状态、命令年龄和输出命令检查

| 接口 | 检查或行为 |
| --- | --- |
| `Safety::configure(cfg)` | 校验并保存最终安全策略 |
| `check_state(...)` | 尺寸、有限性、dt、反馈年龄、硬限位、实测速度、在线／使能与电机故障 |
| `check_cmd_age(...)` | 在要求连续命令时检查命令时效 |
| `check_joint_cmd(...)` | 输出软限位、速度、力矩、Kp／Kd 与相邻命令位置／速度跳变，返回可发送命令或故障 |
| `reset_cmd_history(...)` / `clear_cmd_history()` | 初始化或清理相邻命令检查历史 |
| `action_for(error)` | 将错误映射为保持或失能动作 |
| `clamp_count()` | 查看发生轻微限幅的累计次数 |

`SafetyFault` 包含 `code`、`index`、`value` 和 `limit`；无对应轴时 `index` 为 `std::numeric_limits<std::size_t>::max()`，不能直接作为数组下标

`SafetyAction::STOP_HOLD` 用于状态可信时拒绝错误命令并尝试保持，`DISABLE` 用于状态或硬件不可信时失能；限幅不能代替对反复触限原因的处理

自动标定逐周期监督器发现参考未要求的持续位置往复时，调用 `Robot::stop_calibration_response()` 锁存 `RobotErr::CALIBRATION_RESPONSE_UNSTABLE` 并执行既有 `STOP_HOLD`；故障保持改用原恢复参数并继续检查实际可编码增益与 PD 加前馈总力矩，不继续刷新本趟不稳增益；不会自动清故障、恢复旧运动或重试标定

### 故障恢复

| Robot 接口 | 条件与结果 |
| --- | --- |
| `get_last_fault()` | 返回最近锁存故障，按顶层码选择有意义的子错误 |
| `is_fault_holding()` / `get_fault_hold_mode()` | 检查故障保持是否有效及其刚性／柔性恢复状态 |
| `maintain_fault_hold()` | FAULT 且保持有效时刷新故障保持，不执行普通目标 |
| `enter_fault_compliant_recovery()` | 人工请求受限柔性恢复，需配置允许且当前故障可恢复 |
| `return_to_fault_rigid_hold()` | 退出柔性恢复并返回故障刚性保持 |
| `clear_fault()` | 满足恢复条件后以当前实测位置进入 `ACTIVE + RIGID_HOLD` |
| `reset_fault()` | 兼容入口，执行与 `clear_fault()` 相同的恢复流程 |

自管周期在进入 FAULT 后转入故障保持分支；`ControlRuntime` 与 `RobotSession` 已处理这一分支，调用方仍需决定恢复或失能

清故障不是无条件清错；通信、反馈、安全状态与恢复配置必须满足条件，成功后重新提交新的运动请求，旧目标不会自动恢复

## 关节与执行器映射

<a id="9-joint--actuator-mapping"></a>

声明见 [joint_actuator_mapper.hpp](../../src/serial_arm/core/include/serial_arm/core/joint_actuator_mapper.hpp)；转换前调用 `configure(cfg.mapper)`，`to_joint_state()` 和 `to_actuator_cmd()` 返回 `tl::expected`

`pos_ratio` 定义为执行器位置变化量／关节位置变化量，`tor_ratio` 为关节力矩／执行器报告力矩，`direction` 为 `1` 或 `-1`

```text
q_joint   = joint_zero + direction * (q_actuator - actuator_zero) / pos_ratio
v_joint   = direction * v_actuator / pos_ratio
tau_joint = direction * tor_ratio * tau_actuator

q_actuator   = actuator_zero + direction * pos_ratio * (q_joint - joint_zero)
v_actuator   = direction * pos_ratio * v_joint
tau_actuator = direction * tau_joint / tor_ratio
kp_actuator  = kp_joint / (pos_ratio * tor_ratio)
kd_actuator  = kd_joint / (pos_ratio * tor_ratio)
```

若厂商 SDK 已报告减速器输出端位置与力矩，两个比例通常均为 `1.0`；改变映射后需重新核对配置、限位与标定上下文，不能沿用旧比例下的测量结论

## Python API

<a id="10-python-api"></a>

安装方式见 [安装说明](../install.md)；导出接口以 [bindings.cpp](../../src/serial_arm/core/python/bindings.cpp) 为准，会话实现在 [robot_session.hpp](../../src/serial_arm/core/python/robot_session.hpp)、[Python 封装](../../src/serial_arm/core/python/serial_arm/__init__.py) 与 [robot_session.cpp](../../src/serial_arm/core/python/robot_session.cpp)

### RobotSession

Python 线程提交请求，独立 C++ 工作线程串行维护控制周期；Python 轮询或 UI 刷新不改变控制频率

| 接口／属性 | 用途与条件 |
| --- | --- |
| `RobotSession.from_profile(name, ...)` | 从 Profile 创建会话 |
| `RobotSession(core_yaml, plugin, hardware_yaml)` | 显式指定配置与驱动 |
| `start()` | 激活 Robot 并启动原生工作线程 |
| `stop()` | 停止线程，ACTIVE 调用 `deactivate()`，FAULT 调用 `force_deactivate()`，不执行停放 |
| `move_to(pos, speed_scale)` | 绝对位置目标，`speed_scale` 在 `(0, 1]`，返回运动编号 |
| `hold_current()` | 取消位置目标并捕获当前实测位置保持 |
| `set_impedance_mode(mode)` | 工作线程串行应用任务切换，清除旧目标与用途覆盖 |
| `set_execution_gain_profile(profile)` / `clear_execution_gain_profile()` | 设置或清除用途覆盖 |
| `set_reference_behavior(behavior)` / `set_model_feedforward_mode(mode)` | 在已配置、未启动且 INACTIVE 时选择策略 |
| `set_servo_tracking_gains(gains)` / `set_hold_compensation_cfg(cfg)` / `set_admittance_cfg(cfg)` | 提交合法参数，运行中等待工作线程应用结果，导纳仅允许同策略更新 |
| `set_gravity_scale(values)` | 运行时由工作线程应用逐轴重力比例 |
| `clear_fault()` / `reset_fault()` / `enter_fault_compliant_recovery()` / `return_to_fault_rigid_hold()` | 提交给同一工作线程处理的故障恢复请求 |
| `snapshot` | 线程安全复制的周期、运动、故障与动力学快照 |
| `state` / `fault_hold_mode` / `configured` / `running` | 会话状态 |
| `config` / `dynamics_info` / `actuator_info` | 静态配置、模型与执行器能力副本 |

从保持提交合法位置目标时自动转入相应跟踪任务；拖拽需先退出再移动，无效目标不改变当前任务

读取 `snapshot` 后先检查 `last_error`、`valid`，再检查具体周期有效标志；`running` 只表示线程状态，不能证明反馈新鲜或机械臂到位

可恢复 FAULT 时工作线程继续维护故障保持；进入 FAULT 会丢弃旧目标、旧任务与尚未应用的重力比例请求，清故障成功后从刚性保持重新开始

### 计算与错误

Python 同时绑定配置加载、`Dynamics`、`JointCtrller`、`JointActuatorMapper`、`Safety` 与相应配置结构；NumPy 数组按同一关节顺序使用 SI 单位

`Dynamics.update(q, dq, ddq, tau, ddq_ref)` 与 C++ 完整更新对应，`update_state()`、`set_gravity_scale()`、`frame_pose()`、`frame_jacobian()` 提供分组计算，结果以只读属性读取

C++ `tl::expected` 错误转换为 Python `serial_arm.SerialArmError`；调用失败需读取异常，线程内错误从 `snapshot.last_error` 查看

```python
try:
    arm.hold_current()
except serial_arm.SerialArmError as exc:
    print(exc)
```

## ROS2 适配层

<a id="11-ros-2--ros2_control-adapter"></a>

[SerialArmSystem](../../src/serial_arm/bringup/ros2_control/include/serial_arm_ros2_control/serial_arm_system.hpp) 是可选的 `hardware_interface::SystemInterface` 插件，由 `controller_manager` 管理生命周期；Core、Terminal、Python 和桌面应用不依赖 ROS2

| 生命周期／接口 | 职责 |
| --- | --- |
| `on_init()` / `on_configure()` | 读取参数、加载配置和 Backend，建立状态与命令缓存 |
| `on_activate()` | 显式授权后连接、使能并启动原生控制线程 |
| `on_deactivate()` | 停止后台线程并请求保持后失能 |
| `export_state_interfaces()` / `export_command_interfaces()` | 向 ros2_control 提供关节状态和参考接口 |
| `read()` / `write()` | 复制后台快照和上层参考，不在 controller_manager 线程另做完整串口／CAN 控制闭环 |
| `CommandFrame` / `StateFrame` | 适配层与后台工作线程之间的命令／状态缓存 |

适配层复用 `ControlRuntime`；MoveIt 启动与配置见 [使用指南](../guide.md)

## 驱动与 Transport

<a id="12-hardware-backend-与-transport"></a>
<a id="motorbus"></a>

### MotorBus 与动态加载

公共契约见 [motor_bus.hpp](../../src/serial_arm/core/include/serial_arm/hardware/motor_bus.hpp)、[hardware_capability.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_capability.hpp) 与 [hardware_loader.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_loader.hpp)

| Backend 方法 | 实现要求 |
| --- | --- |
| `configure(path)` | 解析硬件配置并建立能力，不使能或驱动执行器运动 |
| `connect()` | 打开总线并允许读取状态，成功不代表已使能 |
| `read()` | 返回执行器侧状态、在线／使能／故障信息及可用的逐轴反馈时序 |
| `activate()` | 使能并切换到契约所需的可写控制模式 |
| `write(cmd)` | 将完整执行器命令转换为厂商协议与单位 |
| `stop()` / `deactivate()` | 请求停止／安全保持，或退出激活并失能 |
| `recover()` | 恢复硬件故障，不能替代 Robot 层 `clear_fault()` |
| `capabilities()` / `size()` | 提供真实型号或明确保守的物理能力与轴数 |
| `cleanup()` | `noexcept`、可重复调用，释放 Backend 自己持有的逻辑资源 |

`HardwareLoader::load(plugin, path[, overrides])` 返回绑定 Backend 与 DSO 显式引用生命周期的 `unique_ptr<MotorBus>`；析构时先销毁 Backend、释放设备与总线连接，再释放加载句柄；DSO 代码通过 `RTLD_NODELETE` 保留至进程退出，供共享总线注册表中较晚释放的弱引用控制块安全析构，调用方不手动 `dlclose()`

`HardwareConfigOverrides` 只覆盖本次加载的 `serial_port`、`baudrate` 与 `bus`，不回写硬件 YAML

自定义驱动必须导出三个 C 入口，直接使用当前 SDK 头文件的 ABI 标识；替换 SDK 后重新构建驱动，并保持工具链、标准库和链接配置匹配

```cpp
extern "C" std::uint64_t serial_arm_motor_bus_abi() {
    return serial_arm::motor_bus_abi_fingerprint;
}

extern "C" serial_arm::MotorBus* create_motor_bus() {
    return new MyMotorBus;
}

extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus) {
    delete bus;
}
```

`MyMotorBus` 需完整实现上表，构建方式可参考 [内置驱动](../../src/robot_supports/hardware/damiao/CMakeLists.txt)；Backend 返回完整唯一的关节身份后，`capabilities_for_joint_order()` 与 `align_motor_bus()` 支持统一排序

### 共享 CAN

<a id="transport-api"></a>

[bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/bus.hpp) 提供 `acquire_can_channel()`、`CanChannel`、`CanBus`、资源描述和冲突错误；[can.hpp](../../src/serial_arm/core/include/serial_arm/transport/can.hpp) 定义帧和过滤器

- Driver 通过 `acquire_can_channel()` 获取自己的逻辑通道，不持有或关闭 raw physical Bus
- logical name、physical resource、provider identity 与影响共存的物理配置必须一致，冲突以 `BusRegistryErr` 返回
- `BusResourceDescriptor::ownership_key` 标识真正唯一的底层资源，不同 Provider 访问同一 tty 时应提供相同 ownership key
- `CanChannel::send()` 的 TX 和 `receive()` 的物理接收／分发分别串行化，每个 Driver 只消费自己的 channel
- pending 队列默认最多 `256` 帧，达到上限丢弃最旧帧；`diagnostics()` 提供 pending、received、dropped 统计，协议层仍需持续消费并匹配响应 payload
- 最后一个逻辑引用释放后，由共享资源管理关闭总线；协调范围为同一进程，不提供跨进程 CAN broker

达妙官方 USB2CAN 可通过 [protocol helper](../../src/robot_supports/protocol/damiao_usb2can/include/serial_arm_protocol_damiao_usb2can/bus.hpp) 获取 `CanChannel`，它适配该设备私有串口协议，不是通用 USB2CAN 协议

### 共享 Serial

[serial_bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/serial_bus.hpp) 的 `acquire_serial_bus_client()` 返回 transaction-only client；内部 `SerialBus` 唯一持有 [SerialPort](../../src/serial_arm/core/include/serial_arm/transport/serial_port.hpp)

```cpp
auto client = serial_arm::transport::acquire_serial_bus_client("tool_serial", config);
if(!client) return;
(*client)->transaction([&](serial_arm::transport::SerialTransaction& transaction) {
    transaction.write(request.data(), request.size());
    const auto received = transaction.read_exact(response.data(), response.size());
    // 按协议检查 received、帧头、地址和校验码后再使用 response
});
```

- 同一次 request-response 放在同一 transaction 内，避免其他 client 在中间插入事务
- client 与 transaction 不暴露物理资源的 open／close／set_config，Driver 释放自己的 client，不接管串口生命周期
- baud rate、data bits、parity、stop bits、flow control 属于物理兼容条件，读写 timeout 是 client／transaction 策略，可用 `SerialTransactionOptions` 临时覆盖
- `read()` 超时可返回零字节，`read_exact()` 与 `write()` 仍可能少于请求长度，协议实现检查返回长度
- 同一物理 Bus 的 transaction 串行执行，callback 异常原样抛出，`failed_transaction_count` 只统计 callback 异常
- Driver 仍需确认电气层、半双工、寻址、framing 与主动发送行为可共线，Core 不替任意协议保证共存
- 自动处理收发方向的普通 POSIX tty／RS485 可使用此接口，需要显式 RTS 或 `TIOCSRS485` 方向控制的场景不在当前支持范围

## 错误处理

<a id="14-错误处理与错误码"></a>

主要 C++ 模块以 `tl::expected<T, E>` 返回正常错误；成功后才能取结果，失败记录模块、顶层码和对应子错误，不只输出一个整数

```cpp
auto output = robot.cycle();
if(!output) {
    const auto& fault = output.error();
    std::cerr << "RobotErr=" << static_cast<int>(fault.code) << '\n';
    if(fault.code == serial_arm::RobotErr::SAFETY_FAILED) {
        const auto& safety = fault.safety_fault;
        std::cerr << "SafetyErr=" << static_cast<int>(safety.code)
                  << " index=" << safety.index
                  << " value=" << safety.value
                  << " limit=" << safety.limit << '\n';
    }
    // 由周期所有者继续故障保持或显式失能
}
```

| 错误类型 | 排查入口 |
| --- | --- |
| `RobotProfileErr` / `ConfigErr` | Profile 资源、YAML 语法、必填项、数组长度和字段值 |
| `ModelErr` / `DynamicsErr` | URDF、关节与 Frame、模型维度、输入有限性和缓存更新状态 |
| `LimitResolverErr` / `JointActuatorMapErr` | 能力交集、方向、比例、零位及转换结果 |
| `JointCtrllerErr` | 初始化、任务是否接受命令、完整命令授权与增益 |
| `SafetyErr` / `SafetyFault` | dt、命令／反馈时效、在线／使能、限位和数值 |
| `MotorBusErr` / `HardwareLoaderErr` | 设备通信、硬件状态、配置、共享库符号及 ABI |
| `ModelFeedforwardErr` / `ReferenceGovernorErr` / `InteractionControllerErr` | 模型回调、参考限幅和导纳配置／运行状态 |
| `RobotErr` / `RobotFault` | 顶层生命周期与上述模块错误的组合 |
| `BusRegistryErr` / `CanErr` / `SerialPort` 异常 | 物理资源冲突、帧通信、串口 I/O 与事务 |

故障恢复条件见 [Safety 与 FAULT](#safety-与-fault)，图形预览错误与机器人连接、运动错误分别处理，不能用重新加载模型代替硬件故障恢复

## 公共接口索引

完整成员、重载、默认值与错误枚举以当前公共头文件和 Python 绑定为准，以下链接直接定位实现契约

| 分组 | 声明／绑定 |
| --- | --- |
| 基础类型与命令 | [types.hpp](../../src/serial_arm/core/include/serial_arm/core/types.hpp) |
| Profile 与完整配置 | [robot_profile.hpp](../../src/serial_arm/core/include/serial_arm/config/robot_profile.hpp) · [config.hpp](../../src/serial_arm/core/include/serial_arm/config/config.hpp) |
| 模型与限位解析 | [model_loader.hpp](../../src/serial_arm/core/include/serial_arm/model/model_loader.hpp) · [limit_resolver.hpp](../../src/serial_arm/core/include/serial_arm/config/limit_resolver.hpp) |
| Robot 与共享周期 | [robot.hpp](../../src/serial_arm/core/include/serial_arm/robot.hpp) · [control_runtime.hpp](../../src/serial_arm/core/include/serial_arm/runtime/control_runtime.hpp) · [joint_motion.hpp](../../src/serial_arm/core/include/serial_arm/runtime/joint_motion.hpp) |
| 底层控制器与参考约束 | [joints_ctrller.hpp](../../src/serial_arm/core/include/serial_arm/core/joints_ctrller.hpp) · [reference_governor.hpp](../../src/serial_arm/core/include/serial_arm/core/reference_governor.hpp) |
| 关节映射与 Safety | [joint_actuator_mapper.hpp](../../src/serial_arm/core/include/serial_arm/core/joint_actuator_mapper.hpp) · [safety.hpp](../../src/serial_arm/core/include/serial_arm/core/safety.hpp) |
| 动力学与重力校正 | [dynamics.hpp](../../src/serial_arm/core/include/serial_arm/dynamics/dynamics.hpp) · [gravity_correction.hpp](../../src/serial_arm/core/include/serial_arm/dynamics/gravity_correction.hpp) |
| 导纳控制与指标 | [joint_admittance_controller.hpp](../../src/serial_arm/core/include/serial_arm/interaction/controllers/joint_admittance_controller.hpp) |
| FULL_ID 与 MOMENTUM | [torque_residual_observer.hpp](../../src/serial_arm/core/include/serial_arm/interaction/estimators/torque_residual_observer.hpp) · [generalized_momentum_observer.hpp](../../src/serial_arm/core/include/serial_arm/interaction/estimators/generalized_momentum_observer.hpp) |
| 外力估计与运行组合 | [external_torque_observer.hpp](../../src/serial_arm/core/include/serial_arm/interaction/estimators/external_torque_observer.hpp) · [interaction_controller.hpp](../../src/serial_arm/core/include/serial_arm/interaction/runtime/interaction_controller.hpp) |
| 导纳标定 | [admittance_calibration.hpp](../../src/serial_arm/core/include/serial_arm/interaction/admittance_calibration.hpp) |
| Backend 与能力 | [motor_bus.hpp](../../src/serial_arm/core/include/serial_arm/hardware/motor_bus.hpp) · [hardware_capability.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_capability.hpp) · [hardware_loader.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_loader.hpp) |
| Transport | [bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/bus.hpp) · [can.hpp](../../src/serial_arm/core/include/serial_arm/transport/can.hpp) · [serial_bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/serial_bus.hpp) · [serial_port.hpp](../../src/serial_arm/core/include/serial_arm/transport/serial_port.hpp) |
| ROS2 适配 | [serial_arm_system.hpp](../../src/serial_arm/bringup/ros2_control/include/serial_arm_ros2_control/serial_arm_system.hpp) |
| Python | [bindings.cpp](../../src/serial_arm/core/python/bindings.cpp) · [robot_session.hpp](../../src/serial_arm/core/python/robot_session.hpp) |
