# SerialArm-Core API Reference

应用优先使用 C++ `Robot` 或 Python `RobotSession`；离线模型计算使用 `Dynamics`；驱动扩展从 `MotorBus` 与 Transport 接口开始

配置与操作流程见 [使用指南](../guide.md)，构建与安装见 [安装说明](../install.md)，模块职责见 [开发指南](../development.md)

## 按任务查找

| 任务 | 入口 |
| --- | --- |
| 建立第一个 C++ 应用 | [C++ 配置示例](#c-配置示例) |
| 从 Python 完成一次移动 | [Python 运动示例](#python-运动示例) |
| 查单位、向量顺序与命令结构 | [基础类型](#基础类型) |
| 读取 Profile、执行路径与交互参数 | [配置](#配置) |
| 管理使能、周期、目标和快照 | [Robot 与 ControlRuntime](#robot-与-controlruntime) |
| 计算位姿、Jacobian、重力和惯性 | [Dynamics](#dynamics) |
| 使用模型导纳与外力观测 | [导纳](#导纳) |
| 处理限位、超时和故障恢复 | [Safety 与 FAULT](#safety-与-fault) |
| 接入新执行器或共享总线 | [驱动与 Transport](#驱动与-transport) |
| 查完整签名与错误枚举 | [公共接口索引](#公共接口索引) |

## 快速示例

### C++ 配置示例

下面从已安装的 `dm_arm_gray` Profile 加载 Backend、配置和模型，构造 `Robot`，只读连接并观察未使能反馈；示例不调用 `activate()` 或发送运动命令

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
    if(!robot.connect()) return 1;
    auto observed = robot.observe();
    if(!observed) return 1;
    std::cout << "Observed joints: " << observed->joint_state.pos.size() << '\n';
    // 根据真实反馈完成映射和边界核对后，由上层单独决定是否 activate()
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

配置与 API 统一使用 SI 单位；关节类型由模型决定，不通过 UI 单位开关改变；Backend 须先转换编码器计数、RPM、电流等厂商量，再交给 Core

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

### 执行路径与交互模式

`ControlPath::{MIT, CSP}` 选择执行路径；`JointImpedanceMode::{RIGID, COMPLIANT, DRAG, SERVO_TRACKING}` 选择交互模式，定点、连续参考与轨迹属于独立的运动操作

`control.controller.rigid`、`compliant`、`drag` 各包含逐关节 `kp/kd`；伺服跟踪使用刚性参数，拖拽的 MIT `kp` 为零；MIT 将位置、速度、增益与前馈送至执行器的 PD；CSP 的柔性和拖拽使用观测外力驱动虚拟 M/D/K，拖拽使用零刚度，刚性和伺服跟踪直接输出名义参考

完整的 `capability.admittance` section 声明外力观测能力，加载器由此派生内部能力状态；公开 YAML 不再接受 `capability.admittance.enabled`、`reference_behavior` 或 `observer.mode`；`joint_enabled` 只选择参与导纳的轴；CSP 的柔性与拖拽使用导纳，刚性与伺服跟踪直接执行名义参考；MIT 使用三组 PD，伺服跟踪引用刚性参数

CSP 必须提供逐关节正值 `safety_policy.max_tracking_error`；缺失或遗漏关节会在配置预检中报告具体字段与关节，MIT 可省略；Safety 检查位置、速度、加速度步长、跟踪误差和反馈力矩；`ActuatorCtrlCmd.position_speed_limit` 是正的执行器侧速度上限，独立于带符号的速度参考

达妙 Backend 将 CSP 周期位置参考映射到位置速度模式 `0x100 + ID`，周期帧包含目标位置和运行最大绝对速度；驱动器内部执行串级位置／速度控制；需在实际固件上核对驱动限矩与内部轨迹响应；TMAX 是协议映射范围，不应当作输出力矩限幅

### 运行、前馈与限制

- `runtime.ctrl_frequency_hz` 控制原生周期频率，`write_enabled` 控制硬件写入授权
- `ModelFeedforwardMode` 为 `NONE`、`GRAVITY` 或 `FULL_INVERSE_DYNAMICS`，非 `NONE` 需要有效模型回调
- `DynamicsCfg` 定义 URDF、关节顺序、基座／工具 Frame、重力向量、逐轴 `gravity_scale` 与可选重力校正包，`gravity_scale` 范围为 `[0, 2]`
- `ShutdownCfg::mit/csp` 分别保存 `ParkingPathCfg`，`for_path(ControlPath)` 返回对应配置；配置不会让 `Robot::deactivate()` 或 `RobotSession.stop()` 自动执行轨迹
- `ParkingStrategy` 支持 `RIGID` 与 `COMPLIANT_THEN_RIGID`；后者复用当前路径的交互参数，柔性接近目标并稳定后切刚性，最终满足原逐轴容差与稳定时间后失能；取消或超时会取消轨迹并捕获当前位置，报告停放未完成
- `park_pos`、`position_tolerance` 和 `velocity_tolerance` 按固定关节顺序使用 `JointVector`，YAML 使用关节名称映射；每轴分别比较本轴 SI 数值
- YAML 使用 `shutdown.paths.MIT/CSP`；`enabled` 默认 false，关闭时拒绝显式停放；开启且未设置目标时使用零位，零位超限则拒绝配置
- `enabled` 是唯一停放许可；桌面断开或关闭时，当前路径允许且参数完整则先停放，确认失能后退出；关闭许可或停放失败时先处理状态并失能再退出，不设置额外的自动停放开关
- 达妙低层 `stop/deactivate` 直接失能并确认状态，不注入独立停止 PD；使能必须提交明确首命令，停放与故障保持由上层完成
- `SafetyPolicyCfg::position_margin`、保持补偿的位置／速度门限与摩擦 `velocity_transition` 均为逐轴向量；`FaultRecoveryCfg::velocity_tolerance` 和 `AdmittanceValidationCfg::velocity_tolerance` 独立定义恢复静止与响应验收条件
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
| `connect()` | INACTIVE 中连接并刷新实际硬件能力与限制，不使能或下发运动命令 |
| `observe()` | 读取 INACTIVE 快照，返回 `RobotCycleOutput`；保留越界、离线和驱动错误供诊断 |
| `is_connected()` | 当前 Robot 是否已完成硬件连接 |
| `activate()` | 先连接并观察，验证反馈、当前位首命令及力矩预算，再使能进入 `ACTIVE` |
| `set_cmd(cmd, now)` | ACTIVE 状态提交运动参考，时间戳须单调 |
| `set_full_cmd(cmd, now)` | 原生方式且 `allow_full_cmd=true` 才可直接覆盖完整关节命令，仍经过 Safety |
| `cycle(now)` | 执行一次完整读状态、控制、检查、映射与写命令，返回 `RobotCycleOutput` 或 `RobotFault` |
| `deactivate()` | 正常停止并失能，成功后回到 `INACTIVE` |
| `force_deactivate()` | ACTIVE 或 FAULT 中停止并失能，失败仍返回错误 |

只读连接允许 MIT 增益组缺失；MIT 首次使能必须已有逐轴正的刚性 Kp/Kd，CSP 仅要求位置执行能力、跟踪误差和运动限制，不使用 MIT 增益

使能前，Core 使用未使能反馈检查在线、错误、时效与限位，构造 `q_ref=q_measured`、`dq_ref=0` 的首命令，经协议增益投影、Safety 与总力矩检查后调用 `MotorBus::activate(initial_cmd)`；失败发生在使能前时保留 INACTIVE，便于继续检查

Robot 析构仅在 ACTIVE 或 FAULT 时尝试失能；纯连接观察的析构只释放资源，Backend 的 cleanup 也须保留该只读语义；应用仍应显式处理实际停止结果

同一 Robot 同时只允许一个周期所有者；直接使用 Robot 的应用负责按配置频率调度，并在同一串行上下文更新命令、模式与模型缓存

`ControlRuntime::run()` 管理固定频率周期；`step()` 在 ACTIVE 执行控制、在已连接 INACTIVE 观察反馈、在 FAULT 维护故障反应；`RuntimeSnapshot.cycle`、`observation`、`fault_feedback` 分开保存，观察不会触发示教或标定采样；`submit_reference()` 提交带接收时间和 `MotionSource` 的参考，`snapshot()` 返回复制后的线程安全快照

`RuntimeHooks` 的 `before_cycle`、`after_step` 与 Robot 周期在相同串行上下文执行；使用 Terminal、Python 或 ROS2 适配层时，复用它们的原生周期，不另开第二个 Robot 控制循环

同一运动源的待处理参考由最新请求覆盖，不同 `MotionSource` 同时占用时返回 `MOTION_SOURCE_BUSY`；切换任务或 `clear_commands()` 释放来源，FAULT 也会清除旧参考

`ControlRuntime` 对非法调度参数抛 `std::invalid_argument`，对重复周期所有者抛 `std::logic_error`，调用方在启动时处理这些配置／所有权错误

### 模式与参数更新

`set_impedance_mode(mode, now, target_policy)` 在 ACTIVE 状态交接交互模式；`set_control_path(path)` 在 INACTIVE 状态选择 MIT 或 CSP；`set_model_feedforward_mode(mode)` 选择前馈策略，CSP 不下发力矩前馈

`set_admittance_cfg(cfg)` 更新虚拟模型参数，控制器保留虚拟状态；启动、故障保持和模式转换应锚定已接受参考或实测状态

`set_calibration_disturbance(torque, now)` 为独立扰动短测提供有界关节力矩入口，`clear_calibration_disturbance()` 清除该输入；扰动仍经过原有力矩、参考与 Safety 约束，调用方需按周期刷新并在所有退出路径清除；周期输出的 `calibration_disturbance` 记录本周期实际叠加的扰动，不以请求值代替执行值

固定目标扰动保留 `get_hold_target()` 的名义目标；独立输入必须与实际已执行的 PD、前馈、参考和新反馈一起记录，不能用请求值代替执行值

MIT 自动增益标定只适用于 MIT 执行路径；模型初值是待测起动参数，CSP 驱动器内部伺服整定与外层 M/D/K 调整均不属于该工具

增益任务记录保留四类时序与证据：源轨迹时间、参考采样时间、命令写入时间和反馈接收时间；学习中的变增益记录与最终固定整组测量分开，`joint_cmd.kp/kd` 表示实际可编码增益

| 结果／记录 | 含义 |
| --- | --- |
| `available` | 请求用途的基本参数测量完整 |
| `qualification_complete` / `measurement_complete` | 对应响应窗口和用途证据是否完整 |
| `validated` / `endpoint_passed` | 新增益任务保留为 false，不把参考精度分析当作实机验收 |
| `native_validation.online_adaptation` | 区分变增益学习与冻结参数测量 |
| `termination` | `kind`、`reason`、`blocked_joint_names`、`fault_interrupted` 保持状态与候选一致 |
| `gain-trajectory-*.csv` | 实际整轨命令、反馈、参数和墙钟记录 |
| `gain-purpose-*.csv` / `gain-anchor-*.csv` | 内点用途输入及访问记录 |
| `drag-decay-*.csv` / `drag-taught-release-*.csv` | 零刚度后安静及释放记录 |

快速三用途 `gain_mode=all/gain_tuning_mode=quick` 使用原始示教 T 的 18T 目标和 24T 硬预算；单用途精调使用 12T 和 15T，重定时和补测不增加 T；参数模型未知或响应不可分辨时保留原始数据与缺项原因

模型初始化、已知输入、整轨学习、用途比较及结果定义见 [标定参考](calibration.md)；历史任务按自身的来源和证据范围读取，不由新 UI 自动提升验证状态

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
| `control_path` / `parameter_update_progress` | 当前参考策略与参数交接进度 |
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
| `AdmittanceCapabilityCfg` | 派生能力状态、逐轴开关、动量观测器、标定、M/D/K 与验证标志 |
| `AdmittanceObserverCfg` | 使用广义动量观测，不显式依赖实测加速度 |
| `momentum_gain` | MOMENTUM 一阶增益，单位 s⁻¹，逐轴且大于零 |
| `AdmittanceCalibrationCfg` | 固定 `torque_bias`、速度相关摩擦 residual 模型与 `torque_threshold` |
| `AdmittanceControllerCfg` | `mass`、`damping`、`stiffness`、`max_delta_q` 与 `max_delta_q_dot` |
| `JointAdmittanceController` | 独立关节导纳计算，`configure()` 后更新，接受状态后再用于下一周期 |
| `compute_admittance_damping_metrics()` | 计算二阶系统频率、阻尼比等诊断指标，不替代离散周期和执行器验证 |
| `InteractionModelStateFn` | 实测状态对应有效重力、科氏项与质量矩阵，供 MOMENTUM 观测器使用 |

正式外力估计经过 bias、摩擦补偿和 deadband；`full_id_residual_raw` 是对照诊断，不能当作正式输出替代 `tau_ext_hat`

摩擦标定、交叉验证和运行补偿使用同一低速渐入函数；反馈速度分辨率从硬件 capability 按 `velocity_resolution / abs(pos_ratio)` 映射至关节侧，作为内部标定条件传入，不新增 YAML 参数；此前使用不同渐入函数得到的摩擦系数需重新标定

CSP 刚性定位不启用导纳；柔性和拖拽除了完整参数，还要求 `validation.observation_validated=true` 与 `validation.response_validated=true`；参数填写完整与验收通过是不同状态

首次有限刚度测试使用 `Robot::set_admittance_validation(true)`，条件是外力观测已验证、所有参与轴 K>0；它捕获当前位并进入 COMPLIANT，不允许零刚度拖拽；调用 `false` 返回当前位置 RIGID；该许可仅在当前运行任务中有效，不设置 `response_validated`

更改观测增益、零偏、死区或摩擦参数会清除观测和响应验证；更改 M/D/K、参与轴或退让边界会清除响应验证；在不再满足准入条件时退回当前位 RIGID，避免沿用旧验证运行

观测速度在关节侧使用短位置时间窗处理低速量化，同一处理后状态用于模型、动量与摩擦标定；这可抑制速度码造成的假动量，不能消除驱动上报力矩慢变或静摩擦滞回

运行中同策略参数过渡保留已接受虚拟状态，最终参考仍经过 Safety；零刚度拖拽对持续残差会产生持续运动，必须先完成无接触多姿态、双向到位和释放检查

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
| `is_fault_holding()` / `get_fault_hold_mode()` | 检查故障保持是否有效及其刚性／拖拽恢复状态 |
| `maintain_fault_hold()` | FAULT 中持续观察反馈；保持有效时刷新受控输出，不执行普通目标 |
| `enter_fault_drag_recovery()` | 人工请求使用已有 DRAG 参数恢复，故障须允许受控输出；MIT 要求当前模型重力已验证，CSP 要求观测与响应均已验证 |
| `return_to_fault_rigid_hold()` | 退出拖拽恢复并返回故障刚性保持 |
| `clear_fault()` | 保持型故障满足条件后以当前实测位置进入 `ACTIVE + RIGID`；停止输出型故障清到 `INACTIVE`，需重新使能 |
| `reset_fault()` | 兼容入口，执行与 `clear_fault()` 相同的恢复流程 |

自管周期在进入 FAULT 后转入故障保持分支；`ControlRuntime` 与 `RobotSession` 已处理这一分支，调用方仍需决定恢复或失能

清故障不是无条件清错；通信、反馈、安全状态与恢复配置必须满足条件，成功后重新提交新的运动请求，旧目标不会自动恢复

`FaultHoldMode::DRAG_RECOVERY` 表示 FAULT 锁存中的拖拽恢复，不是第五种交互模式；配置使用 `fault_recovery.allow_drag_recovery`，MIT 重力验证放在 `model.gravity_model_validated`，CSP 依赖导纳的两个质量标志，不要求 MIT 补偿验证标志；不提供恢复专属 Kp/Kd、速度或重力缩放

手动退出 DRAG 会捕获当前实测位置；调用端必须停止旧轨迹和目标生产者，不能在下一周期重新发送拖拽前目标；`ControlRuntime`、Terminal 和 `RobotSession` 使用同一交接语义

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
| `connect()` / `observe()` | 停止线程时只读连接并返回 INACTIVE 硬件快照 |
| `start()` | 执行首命令检查，激活 Robot 并启动原生工作线程 |
| `stop()` | 停止线程，ACTIVE 调用 `deactivate()`，FAULT 调用 `force_deactivate()`，不执行停放 |
| `move_to(pos, speed_scale)` | 绝对位置目标，`speed_scale` 在 `(0, 1]`，返回运动编号 |
| `hold_current()` | 取消位置目标并捕获当前实测位置保持 |
| `set_impedance_mode(mode)` | 工作线程串行应用任务切换，清除旧目标与用途覆盖 |
| `set_control_path(path)` / `set_model_feedforward_mode(mode)` | 在已配置、未启动且 INACTIVE 时选择策略 |
| `set_hold_compensation_cfg(cfg)` / `set_admittance_cfg(cfg)` | 提交合法参数，运行中等待工作线程应用结果，导纳仅允许同策略更新 |
| `set_admittance_validation(active)` | 在运行的 CSP 中进入有限 K 测试，或返回当前位置 RIGID |
| `set_gravity_scale(values)` | 运行时由工作线程应用逐轴重力比例 |
| `clear_fault()` / `reset_fault()` / `enter_fault_drag_recovery()` / `return_to_fault_rigid_hold()` | 提交给同一工作线程处理的故障恢复请求 |
| `snapshot` | 线程安全复制的周期、运动、故障与动力学快照 |
| `state` / `fault_hold_mode` / `configured` / `running` | 会话状态 |
| `config` / `dynamics_info` / `actuator_info` | 静态配置、模型与执行器能力副本 |

交互模式与名义目标独立；`move_to()` 设置目标而不代替模式切换，离开 DRAG 会捕获实测位置并取消旧运动请求

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
| `on_init()` | 读取参数、加载配置和 Backend |
| `on_configure()` | 建立 Robot，连接并读取未使能反馈，不发运动命令 |
| `on_activate()` | 检查当前位首命令、使能并启动原生控制线程 |
| `on_deactivate()` | 停止后台线程并失能，不执行停放轨迹 |
| `export_state_interfaces()` / `export_command_interfaces()` | 向 ros2_control 提供关节状态和参考接口 |
| `read()` / `write()` | ACTIVE 复制后台快照和上层参考，INACTIVE 可刷新只读反馈；不另建控制闭环 |
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
| `read()` | 控制周期读取执行器状态及逐轴接收时序，不伪造新时间戳 |
| `observe()` | 未使能观察，可主动请求新反馈；不使能或切换执行模式 |
| `activate(initial_cmd)` | 接收 Core 已验证的当前位首命令；逐轴使能后立即发保持命令，再等待确认或处理其他轴 |
| `activate()` | 底层生命周期入口；Robot 正常使能使用带首命令的重载 |
| `write(cmd)` | 将完整执行器命令转换为厂商协议与单位 |
| `stop()` / `deactivate()` | 请求停止／安全保持，或退出激活并失能 |
| `recover()` | 恢复硬件故障，不能替代 Robot 层 `clear_fault()` |
| `capabilities()` / `size()` | 提供真实型号或明确保守的物理能力与轴数 |
| `cleanup()` | `noexcept`、可重复调用；纯观察连接的释放不发送控制或失能命令 |

`HardwareLoader::load(plugin, path[, overrides])` 返回绑定 Backend 与 DSO 显式引用生命周期的 `unique_ptr<MotorBus>`；析构时先销毁 Backend、释放设备与总线连接，再释放加载句柄；DSO 代码通过 `RTLD_NODELETE` 保留至进程退出，供共享总线注册表中较晚释放的弱引用控制块安全析构，调用方不手动 `dlclose()`

`HardwareConfigOverrides` 只覆盖本次加载的 `serial_port`、`baudrate` 与 `bus`，不回写硬件 YAML

达妙在 MIT 与 CSP 下都以最新未使能位置建立首参考；MIT 首帧使用 Core 已检查的 Kp/Kd/前馈，CSP 使用已选速度上限；新增观察与首命令虚函数改变 Backend ABI，插件必须随当前 Core 重新构建

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
| `RobotErr` / `RobotFault` | 顶层生命周期与模块错误；`ADMITTANCE_NOT_VALIDATED` 表示当前导纳质量前提尚未满足 |
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
| 广义动量观测 | [generalized_momentum_observer.hpp](../../src/serial_arm/core/include/serial_arm/interaction/estimators/generalized_momentum_observer.hpp) |
| 外力估计与运行组合 | [external_torque_observer.hpp](../../src/serial_arm/core/include/serial_arm/interaction/estimators/external_torque_observer.hpp) · [interaction_controller.hpp](../../src/serial_arm/core/include/serial_arm/interaction/runtime/interaction_controller.hpp) |
| 导纳标定 | [admittance_calibration.hpp](../../src/serial_arm/core/include/serial_arm/interaction/admittance_calibration.hpp) |
| Backend 与能力 | [motor_bus.hpp](../../src/serial_arm/core/include/serial_arm/hardware/motor_bus.hpp) · [hardware_capability.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_capability.hpp) · [hardware_loader.hpp](../../src/serial_arm/core/include/serial_arm/hardware/hardware_loader.hpp) |
| Transport | [bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/bus.hpp) · [can.hpp](../../src/serial_arm/core/include/serial_arm/transport/can.hpp) · [serial_bus.hpp](../../src/serial_arm/core/include/serial_arm/transport/serial_bus.hpp) · [serial_port.hpp](../../src/serial_arm/core/include/serial_arm/transport/serial_port.hpp) |
| ROS2 适配 | [serial_arm_system.hpp](../../src/serial_arm/bringup/ros2_control/include/serial_arm_ros2_control/serial_arm_system.hpp) |
| Python | [bindings.cpp](../../src/serial_arm/core/python/bindings.cpp) · [robot_session.hpp](../../src/serial_arm/core/python/robot_session.hpp) |
