# 架构与开发

SerialArm-Core 由 C++ Core 执行控制、动力学、交互和安全检查；GUI、Terminal 与 Python 共用原生能力，ROS2 和 MoveIt 通过可选适配层接入

![Core、上层适配、机器人配置与硬件驱动](images/architecture.svg)

## 组件与边界

| 组件 | 职责 | 源码 |
| --- | --- | --- |
| Core | 生命周期、关节控制、动力学、安全与标定 | `src/serial_arm/core/` |
| ControlRuntime | 周期执行、命令收取、运动来源与快照 | `src/serial_arm/core/include/serial_arm/runtime/` |
| Launcher | 配置、任务、可视化与进程管理 | `apps/launcher/` |
| ROS2 Adapter | ros2_control 命令、状态与生命周期适配 | `src/serial_arm/bringup/ros2_control/` |
| Description / Profile | 模型资源与机械臂使用配置 | `src/robot_supports/` 或下游项目 |
| Hardware Backend | 将统一执行器语义转换为厂商协议 | `src/robot_supports/hardware/` |
| Protocol / Transport | 编解码、时间戳、共享总线与事务 | `src/robot_supports/protocol/` 与 Core transport |

Core 通过 `MotorBus` 接口连接硬件，达妙是现有内置驱动；新增执行器由匹配的 Backend 实现协议、反馈与控制语义

Description 保存 URDF/Xacro、网格、惯性、限位和坐标系；Profile 引用 Description，并配置受控链、Base/Tool、映射、总线、增益、安全限制、停放和可选上层资源

Base 到 Tool 之间非固定、非 mimic 的关节按链顺序进入 `joint_names`；旁支夹爪保留在模型中，由对应驱动提供状态

Renderer 无 Node 权限，Electron 主进程提供受限文件与 IPC 入口，Python 管理配置和离线任务，C++ 管理硬件生命周期与控制安全；诊断、模型导入和候选预览不打开执行器

GUI 按设备路径独占连接；直接使用底层入口的调用方负责保证设备独占

## 控制数据流

```text
上层目标
  → 名义参考及速度/加速度约束
  → 原生阻抗直接执行，或模型导纳生成虚拟参考
  → 动力学前馈、执行器增益与模式交接
  → 位置、速度、加速度与 PD+前馈总力矩约束
  → Safety 检查
  → Joint/Actuator 映射
  → Backend 写入
```

Terminal、Python Session 和 ROS2 Adapter 共享 `ControlRuntime` 的执行与快照接口；同一个 Robot 由一个控制循环驱动

| 维度 | 类型 | 作用 |
| --- | --- | --- |
| 控制方式 | `ReferenceBehavior` | 原生阻抗或模型导纳，决定参考如何生成 |
| 增益用途 | `JointGainProfile` | 刚性、柔性、拖拽、伺服跟踪，决定关节侧 Kp/Kd |
| 任务 | `JointImpedanceMode` | 保持、跟踪与拖拽，管理目标和连续命令 |

四用途配置写入 `control.controller.rigid`、`compliant`、`drag`、`servo_tracking`，`allow_full_cmd` 与它们同级；刚性与柔性各含 `hold`、`tracking` 两套参数，内部六组参数与原任务枚举保留兼容

`control.runtime.tracking_gain_profile` 支持 `RIGID`、`COMPLIANT`、`SERVO_TRACKING`；省略时从旧 `tracking_impedance_mode` 推导；旧平铺增益与 `identification_tracking` 兼容读取，重复字段冲突时拒绝，新建、保存和导出统一写四用途结构

### 原生阻抗与模型导纳

原生阻抗输出位置、速度、前馈力矩、Kp、Kd，由 Backend 实现执行器命令；内置达妙驱动使用 MIT 模式

```text
tau = Kp * (q_ref - q) + Kd * (dq_ref - dq) + tau_ff
```

前馈可选择关闭、重力或完整逆动力学；模型导纳使用外力矩估计生成相对名义目标的退让，内层伺服跟随虚拟参考

```text
M_v * delta_q_ddot + D_v * delta_q_dot + K_v * delta_q = tau_ext_hat
q_virtual = q_nominal + delta_q
```

虚拟模型使用后向 Euler 积分，将限幅后的接受状态回写积分器；M 必须为正，D、K 可为零，K 为零时无弹簧回位；运行中同策略调参按控制周期过渡，控制方式在失能时切换

导纳前馈结合实测状态下的质量矩阵、科氏项、重力、虚拟参考加速度及外力估计计算，最终参考仍受总力矩预算约束；模型导纳的保持和跟踪统一使用伺服增益，交互特性由 M/D/K 决定

### 反馈、保持与完成状态

Backend 随帧传入接收时间和连续性，观测器只在有效新测量上推进；`MOMENTUM` 使用动量观测，`FULL_ID` 使用完整逆动力学残差并要求有效实测加速度；零偏、摩擦残差和死区用于外力估计，残差零偏不加入动力学前馈

普通保持交接使用 `PRESERVE_NOMINAL` 保留原目标，显式“当前位置刚性保持”使用 `CAPTURE_MEASURED`；导纳偏移与名义目标分别保存

`control.hold_compensation` 是原生刚性保持的有界积分补偿，在低速、小误差且观测器有效时工作；接触、限幅、模式变化和故障按条件冻结或清理，模型导纳与柔性保持不使用它

点到点命令具有独立运动编号；参考结束后以新鲜反馈检查逐关节位置、速度与连续稳定时间，报告执行中、完成、中断或失败；目标草稿与命令接收不表示已经到位

## 配置与结果

| 来源 | 生命周期 |
| --- | --- |
| Profile 默认 | 显式保存后持久化 |
| 连接覆盖 | 本次连接，保留原文件 |
| 会话调参 | 当前工作台，保存前预览差异并确认 |
| 标定或停放临时参数 | 对应任务，结束或故障时按任务恢复 |

新 Profile 默认关闭执行器写入、摩擦补偿和导纳；`write_enabled=false` 的 GUI 与 Terminal 使用 Core 和 URDF 建立 mock，不加载硬件插件；实机会话检查当前资源、硬件反馈、生命周期与 Safety 限制

保存核对目标文件和来源，保留备份并原子替换；人工检查记录携带来源，配置变化后需要重新核对对应范围

历史任务和候选保留源快照，应用核对当前控制上下文，导出按兼容性选择当前配置或任务源快照；`available` 表示测量可用，`validated` 表示参考指标达标，未达指标以警告展示，未完成测量的用途不可用

重力覆盖只替换重力项；URDF 质量与质心变化会影响完整动力学，需在相应工况验证；静态和完整惯性辨识的数据、算法及导出条件见 [标定参考](reference/calibration.md)

几何 TCP 候选导出独立 URDF、Core 与 Profile；重标定已有 TCP 时更新候选自有固定关节，并将测量转换回其父坐标系，确认采用后登记新 Profile

子页切换保留模型与相机；URDF 内容、路径或受控链变化时重新加载模型和限位；记录浏览与普通导出可离线操作，耗时导出使用后台任务，保持与失能命令继续使用控制通道

## 准备开发环境

按修改范围选择组件，安装选项见 [安装](install.md)

```bash
# Core、Terminal、Python Binding 和 GUI，不依赖 ROS2
./install.sh --preset standalone --with-python --jobs 1 --yes

# 完整组件，包括 ROS2 / MoveIt；需先安装 ROS2
./install.sh --preset dev --jobs 1 --yes
```

安装器使用 CMake 3.20+、C++17、Pinocchio、Eigen3 和 yaml-cpp；Python Binding 使用 Python 3.10+ 和仓库内置 pybind11，GUI 使用 Node.js 20+ 和 npm

`.install/setup.bash` 与 `.install/run` 加载原生环境；只有显式执行 `./.install/run ros2 ...` 或加载 `.install/ros2-setup.bash` 才进入 ROS2 适配环境

## 手动构建

需要自定义安装前缀或单独构建 C++ 组件时，在仓库根目录使用独立构建脚本

```bash
# Core 库、Terminal 和模型工具，不构建 Python Binding 或 GUI
JOBS=1 ./tools/bootstrap_standalone.sh
source install/standalone/setup.bash

# 同时安装达妙驱动、DM-Arm Profile 和模型资源
JOBS=1 ./tools/bootstrap_standalone.sh --robot dm_arm

# 仅构建 Core 库，不构建 Terminal 与模型工具
JOBS=1 ./tools/bootstrap_standalone.sh --without-terminal
```

该脚本优先使用系统 CMake 包；依赖不齐时需提供 Conan 2，缺少二进制包时默认不编译依赖源码；如需允许该回退，显式设置 `SERIAL_ARM_ALLOW_SOURCE_BUILD=1`

默认构建目录为 `build/serial_arm_core`，安装目录为 `install/standalone`；可通过 `CORE_BUILD_DIR`、`INSTALL_PREFIX`、`BUILD_TYPE` 和 `JOBS` 调整

ROS2 工作区构建见 [安装](install.md#直接在-ros2-工作区构建)；GUI 由统一安装器的 `--gui-only` 入口单独准备

## C++ 下游集成

安装后通过 `find_package(serial_arm_core CONFIG REQUIRED)` 导入；按用途链接以下目标

| CMake Target | 用途 |
| --- | --- |
| `serial_arm::core` | 状态、映射、控制器和 Safety 等基础接口 |
| `serial_arm::config` | Core 配置与 Profile 读取 |
| `serial_arm::robot` | 机器人控制流程与硬件装载 |
| `serial_arm::dynamics` | 动力学模型计算与重力校正 |
| `serial_arm::interaction` | 外力观测与模型导纳 |

下游构建时将安装前缀加入 `CMAKE_PREFIX_PATH`，同时保证 Pinocchio、Eigen3 和 yaml-cpp 的 CMake 包可被找到；接口说明见 [API 参考](reference/api.md)

## 检查改动

编译所改组件后，在仓库根目录执行对应检查；以下命令不连接机械臂

```bash
# GUI 资源、图形运行库与 IPC 路由静态检查，需先安装 GUI 依赖
node apps/launcher/scripts/verify.cjs

# 真实 Electron 的 Profile／动作草稿／关闭与图形重启状态检查
DISPLAY=:1 apps/launcher/node_modules/.bin/electron --no-sandbox \
  apps/launcher/scripts/verify-session-lifecycle.cjs

# 可选：真实 Electron 渲染离线增益页，读取只读候选记录
node apps/launcher/scripts/gain-ui-fixture.cjs \
  --out /tmp/serialarm-gain-ui --record /path/to/task/gain-candidate.json
DISPLAY=:1 apps/launcher/node_modules/.bin/electron --no-sandbox \
  apps/launcher/scripts/render-gain-ui.cjs --fixture /tmp/serialarm-gain-ui

# 完整生产页面的离线渲染，默认覆盖 18 页、明暗主题及两种宽度
DISPLAY=:1 apps/launcher/node_modules/.bin/electron --no-sandbox \
  apps/launcher/scripts/render-studio-ui.cjs

# Python 数组、快照类型和 NumPy 内存布局，需已安装 Python Binding
./.install/run python tools/check_python_binding.py

# 原生命令入口
./.install/run serial_arm_terminal --help
./.install/run serial_arm_model_probe --help
./.install/run serial_arm_model_calibrator --help

# 增益历史证据审计器自测，无原生依赖
python3 tools/audit_gain_calibration.py --self-test

# 模型、辨识、候选、预算与内存 Robot 的行为检查
python3 tools/check_adaptive_gain_core.py

# 旧 PD 上限忽略、硬件能力刷新、映射和协议量化，需先重编 Core
python3 tools/check_config_pd_limits.py

# 真实原生 IPC 的 Mock 集成检查，使用本次构建的程序
./.install/run .install/gui-venv/bin/python tools/check_adaptive_gain_flow.py \
  --terminal build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal \
  --full-probes

# 用 FIFO 阻塞实际 CSV 写入，检查评估阶段已退出 TRACKING
./.install/run .install/gui-venv/bin/python tools/check_gain_assessment_hold.py \
  --terminal build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal
```

GUI 改动还需启动 `./launch.sh` 检查受影响页面、语言与主题；三维改动需验证模型实际绘制与交互，静态资源检查不能代替渲染验证

离线增益页 fixture 提取实际 renderer 函数与事件处理，不加载后端设备接口；`DISPLAY` 使用当前已有图形会话的显示值，结果写入 `/tmp/serialarm-gain-ui/screenshots/report.json`；截图按实际滚动视口分段保存，不拼接或用超大 GPU 裁剪替代；已有阴影裁切审计脚本时，可为渲染命令追加 `--clip-audit /path/to/clip-audit.js` 检查默认、悬停和展开状态

控制逻辑改动先检查配置、离线记录与数值结果，再按实际改动验证实机行为；贡献说明应区分离线验证与实机验证，记录使用的硬件、配置和未覆盖的路径

`check_adaptive_gain_flow.py` 仅在临时目录生成离线 Core、示教和候选，强制 `write_enabled=false` 且不传硬件插件；覆盖两模式预算、原始 T 在重定时与再次导入后的来源、工作线程停止及单用途候选应用／恢复；`--full-probes` 额外完成两组旧增益下的实际短测工作线程，比较模型初值并检查不响应独立力矩的 Mock 保留证据不足；其中的合成候选仅用于记录兼容与应用范围检查，不提供机械臂验收结论

Mock 的理想位置跟随不能验证执行器起动能力；编码器分辨率、静摩擦、MIT 量化、重力负载与模型耦合需由对应离线动态试验和真实机械臂测量分别覆盖；历史记录可用审计器区分“有新反馈但位置不响应”与“反馈缺失”，但离线重新解释原记录不会生成修复后的真机验收

本地安装刷新后仍通过 `./launch.sh` 启动现有桌面入口；检查使用本次构建的程序，源码检查或旧安装程序的通过不能代替当前构建与界面结果

取消用户 PD 上限后的专用检查 `check_config_pd_limits.py` 已通过：分区与平铺旧配置中的缺失、畸形 PD 上限字段均不影响加载，直接策略里的旧覆盖值不生效；连接能力刷新重建协议边界与步进，非 1 映射正确保留协议码，位置、速度与力矩收窄仍然有效；实际达妙编码函数对 Kp/Kd 共 8192 个码中心完成幂等检查，内存 Robot 周期保留投影前请求并发送可编码增益，超过协议上限的请求仍被拒绝

同一最终检查还验证初次故障刚性保持、持续保持、柔性恢复及回到刚性保持的协议网格；半码附近的阻尼请求在编码后会提高实际 PD 命令，以 1 Nm 上限构造的反例中，原浮点总力矩低于上限，而编码后为约 1.000022 Nm，初次保持没有写入超预算命令并走停止路径，持续保持同样拒绝写入并失能；错误类型沿用 `REFERENCE_GOVERNOR_FAILED/INFEASIBLE`

此前执行 `./launch.sh --doctor --profile dm_arm_gray` 时，环境、资源与模型检查正常；因未连接设备，`/dev/ttyACM0` 不存在，整体退出码为 1，设备检查未通过；本轮状态与模型修改未重新执行该设备检查

前一轮起动回归的已执行离线证据：审计器 20 项自测通过；六轴静摩擦／滑动仿真调用当时原生工作线程相同的 `motion_establishment_step`，按历史任务 T=28.22 秒的预算，在 8 次完整短测、累计 68.8945 秒仿真测量成本后建立全部六轴响应，并保留 153 秒用途验证预算；当时未改变位置、力矩与 Kp/Kd 安全上限；静摩擦 12 Nm 高于执行器 10 Nm 能力的对照未被标成建立成功，并在约束／预算内结束

同一六轴静摩擦／滑动回归的独立扰动部分经过 3 次完整测量后，腕轴获得可观测响应，joint3 在原有 1.4 Nm 扰动上限下仍缺少所需运动响应；其静态／恢复记录保留，刚度测量不标为通过

内存 Robot 回归另将两轴实测位置相对旧保持目标分别偏移 `-0.011446 rad`、`-0.022507 rad`，确认起动中心采用新鲜实测反馈，并拒绝过期反馈；这些结果支持离线起动与调用行为，真实机械臂上的运动建立、保持和四用途验收仍需操作者另行发起

本轮实际 Electron 状态检查 `verify-session-lifecycle.cjs` 通过 91 项断言：生产 renderer 函数与按钮处理、真实 preload IPC 和 BrowserWindow 共同验证 Profile 的运行／停止／启动锁、旧异步回包丢弃、目标草稿与原动作终结分离，以及原生关闭、标题栏、应用退出、图形重启的取消／放弃顺序；原生对话框使用预先指定的回答，会话桥接替换为无设备对象，未启动硬件后端；临时导纳草稿仍沿用原导航语义，关闭确认范围为现有 Profile 参数草稿

此前 `check_gain_assessment_hold.py` 在实际原生工作线程中用 FIFO 阻塞 CSV 写入约 0.35 秒，期间连续 22 次观测到 HOLD，最终排出 3,036,238 字节并正常取消；对应内存 Robot 检查同时确认 HOLD 超过 100 ms 不触发命令超时，而普通 TRACKING 命令断流仍触发 `CMD_TIMEOUT`；根因修复针对运动结束后的评估／写盘路径，未提高超时常数

模型检查区分物理先验、设计目标和执行约束：人工速度／加速度／力矩修改不改变 `unconstrained_kp/kd`，收窄力矩可改变约束后候选；完整质量矩阵的修正只调整相关模态参与轴，独立质量块不被一起缩小；真实 URDF 与历史路径的 7 类检查及新 Profile 创建／保存的 8 项检查通过，离线初始化凭据保留三层建议与原因，既有运行／恢复基线保持原值；恢复设计和不同示教路径产生不同建议，均不作为实机起动或四用途验收结论

上一版原生／Python 构建与本地安装刷新通过，使用当时安装 probe 的 Python 8 项检查、NumPy Binding 检查、后端 `--check`、PD 兼容／8192 码中心／故障保持力矩检查均通过；当时 `--full-probes` 12 项真实原生 Mock IPC 检查通过，含不同旧增益下同模型 seed、参考与独立扰动短测、预算与重复导入、停止及单用途应用兼容；T=0.2 秒合成源对应 3 秒硬预算，实际 0.515261 秒结束，`available=false`、`validated=false`

最终全应用离线界面检查加载完整生产 HTML、JS 与 CSS，以只读接口替代后端；18 页在 1280／375 宽度、明暗主题下共 72 次渲染通过，11 项交互检查和 160 次默认／悬停／展开裁切审计通过，无页面横向溢出或控制台错误；数字去尾零与真实初始化凭据另作定向检查；共享按钮沿用橙色白字，参数表按关节并列 Kp／Kd，未编辑保存保留原始数值精度

字体使用应用内独立 family 与精确本地正体名称，避免本机通用字体别名将界面替换成斜体等宽字；实际渲染字体通过浏览器字体信息核对为正体 Noto Sans CJK SC，未修改系统字体设置或新增字体依赖；界面 fixture 不启动设备或原生控制会话，三维模型与终端执行仍由各自检查覆盖

本次增益修正将未执行的整轨性能需求与实际 bootstrap 分开，使用恢复设计起动并逐步补充响应证据；逐周期位置监督触发后锁存 `CALIBRATION_RESPONSE_UNSTABLE`，在写盘前进入既有故障退出，失败试验保存部分周期数据；不提高命令超时，不自动清故障后重试

现有审计器新增实际增益、接收时序、成功周期总命令重建与位置反转证据，30 项自测通过；原始故障任务回归确认整轨入口仅有 188 周期、0.956740 秒，joint4 已出现约 50 Hz 的位置往复，而周期最大间隔仅 11.040 ms，故障为 `REFERENCE_GOVERNOR_FAILED/INFEASIBLE`；这与此前评估写盘后的 `CMD_TIMEOUT` 分开处理；最新任务 22 个文件和原参考 ZIP 读取前后 SHA256 均一致，审计不生成新硬件验收结论

本次界面针对性离线检查通过 6 组组合、36 张截图、36 次裁切检查和 232 项断言；真实机械臂的 bootstrap、在线位置监督退出及四用途效果仍由操作者另行发起验证

本次最终原生／Python 构建与本地安装通过；`--full-probes` 12 项实际原生 Mock 检查、Python Binding、后端 `--check` 与安装环境检查通过；T=0.2 秒、硬预算 3 秒的合成任务在 0.543378 秒退出，保持 `available=false/validated=false`；内存 Robot 验证了异常响应锁存、可行故障保持、不可行保持时原 `MotorBus::stop` 回退及原有 `CMD_TIMEOUT` 语义

离线六轴运动建立回归经 15 趟、114 秒测量完成并保留原验证预留；原始位置记录重放中，监督器在整轨入口后 0.905995 秒触发，早于原记录末帧 0.956740 秒，四份原短测记录没有误触发；这些是离线重放／内存控制检查，没有重新运行原始真机试验

同一最终版本的实际原生工作线程 FIFO 检查通过：阻塞写盘期间保持 HOLD 0.428643 秒、连续 21 次观测，正常排出 2,977,861 字节后取消退出；`CMD_TIMEOUT` 仍为 0.1 秒；Mock 证据不足时实际局部比较数为 0，未盲目试参；生产比较循环另按快速每组最多 1 次、精细每组最多 3 次和实际剩余预算共同约束

本次输入／行程平台修正的审计器 36 项自测与原 15 份 CSV 的 12 组只读回归通过，27 个原文件读取前后哈希不变；正式窗口重算保留末趟 `ratio=0.204693`，不将其误判为低于 0.2，也不将任务外的后续 fault 归入这份 `fault_interrupted=false` 记录

定向控制检查重放原始前 10 趟测量时，在 joint3 当前 Kp=`241.39194` 首次拒绝没有可分辨输入增长的继续升参，保留整组增益并转向较慢测量；原后 5 份 CSV 只保留为旧输入条件的回归，未用于假充新慢测结果；后续慢测成功／受阻由独立仿真验证，六轴 15 趟、114 秒的既有预算回归继续通过；本次原生构建通过，未执行新的真机试验

本轮实际原生 Mock `--full-probes` 12 项通过，新增记录断言核对 `current_kp/current_kd` 与 CSV／参数元数据一致，`blocked_axes/input_response_progress` 可解析；T=0.2 秒、硬预算 3 秒的任务在 0.506322 秒结束，`available=false/validated=false`；Launcher 检查与 31 项界面检查、4 张截图通过，两种尺寸的裁切检查均为零

本次多频平台与取消分类修正后，审计器 44 项自测、三份原任务的 12／9／8 组只读回归及旧 ZIP 兼容检查通过，66 个原始文件读取前后哈希一致；回归分别保留旧 15 趟升参平台、同组增益慢测 12 趟及取消后的部分第二趟，不把历史 CSV 视为新单频条件的实测结果；核心离线检查覆盖同组单频运动补证成功与仍不足两种情况，六轴运动建立经 14 趟、106.4 秒完成，原异常位置记录的监督触发仍为 0.905995 秒；真实机械臂效果尚待操作者主动发起验证

本次原生构建、本地安装、后端 `--check` 与安装环境检查通过；实际原生 Mock `--full-probes` 12 项及终止字段断言通过，主动停止的状态与候选共同记录 `cancelled/stop_requested/fault_interrupted=false`；T=0.2 秒、硬预算 3 秒的合成任务在 0.502098 秒结束，两处共同记录 `budget_exhausted`，`available=false/validated=false`；两份历史终态的界面检查共 218 项通过，1555／1280／375 宽度与明暗主题共 12 张终态截图、12 次零裁切检查，另核对关闭后主页面；原增益页 25 项交互与 2 次定向渲染、Launcher 及语法检查通过

## 构建发行包

准备 Release 时执行 `./build_linux.sh`，默认生成独立 Core、Python 与 GUI 发行包；使用 `--with-ros2` 才附加 ROS2 / MoveIt 适配层，构建条件和产物见 [安装](install.md#构建-linux-发行包)

日常更新按修改范围构建和检查组件，无需重新生成 AppImage 或 deb

## 文档约定

README 提供项目介绍、安装和文档入口，操作说明集中在 [使用指南](guide.md)，算法与接口细节放在 `docs/reference/`

命令应写明执行目录、所需组件与参数含义；模型、设备和资源路径使用示例值，不写入本机绝对路径、运行记录或私人笔记

修改公开行为、配置格式或命令入口时，同步更新对应说明；自然语言不使用中文句号，段末不加标点，代码、路径、URL、数学表达与许可证原文保留必要符号
