# 机械臂接入与配置

接入一台串联机械臂需要模型、硬件驱动、执行器映射、控制参数与安全限制；这些资源通过 Robot Profile 关联，达妙驱动和 DM-Arm 配置可作为内置示例

![从模型创建机械臂配置](images/launcher-new-arm.png)

## 选择资源

自己的机械臂在下游项目维护 `robot_profiles.yaml`、模型与 Hardware Backend；Controllers 和 MoveIt 资源只在使用相应适配层时提供

```bash
./install.sh --preset standalone --robot none --with-python --jobs 1 --yes
export SERIAL_ARM_RESOURCE_PATH=/absolute/path/downstream/workspace
./.install/run serial_arm_terminal \
  --robot-profile your_robot \
  --profile-file /absolute/path/downstream/robot_profiles.yaml
```

此预设安装原生 Core、Terminal、Python 与桌面工作台，下游驱动和资源包需构建并加载；需要 ROS2 时再显式选择 `--preset ros2 --robot none --without-moveit`

内置 DM-Arm 示例位于 `src/robot_supports/profiles/config/robot_profiles.yaml`

| Profile | Core YAML | MoveIt Package |
| --- | --- | --- |
| `dm_arm_gray` | `config/core/gray.yaml` | `dm_arm_no_gripper` |
| `dm_arm_white` | `config/core/white.yaml` | `dm_arm_with_gripper` |

示例零位、比例和增益对应具体机构；新的机械臂实例需填写自己的映射、标定与限位

## 创建与填写 Profile

首页使用“打开机械臂 Profile”选择已有资源目录，或用“选择机械臂 Profile 文件 · YAML”指定配置文件；新模型从“新建机械臂 Profile”导入 URDF、Xacro 或 Description 包，再选择 Base、Tool 并检查主串联链和受控关节顺序

`tool_frame` 使用实际末端坐标系，旁支夹爪不会自动加入主臂控制链

![参数配置](images/launcher-profile.png)

| 内容 | 字段或文件 | 核对内容 |
| --- | --- | --- |
| 模型 | `model` | URDF、受控关节、顺序、Base/Tool、重力方向 |
| 映射 | `calibration.joints` | 方向、位置与力矩比例、关节与执行器零位 |
| 运行 | `control.runtime` | 周期、写入开关、前馈、默认跟踪用途 |
| 执行器增益 | `control.controller` | 刚性、柔性、拖拽与伺服跟踪 Kp/Kd |
| 保持补偿 | `control.hold_compensation` | 原生刚性保持的积分范围、变化率与冻结条件 |
| 交互 | `capability.admittance` | 控制方式、观测器、M/D/K 与退让限制 |
| 安全 | `safety_policy` | 反馈与命令超时、关节限位、力矩预算、恢复条件 |
| 停放 | `shutdown` | 停放目标、路径速度、容差与超时 |
| 硬件 | Hardware YAML | 驱动、总线、设备、执行器 ID 与型号 |
| ROS2 可选适配 | Profile 的 `controllers` | 控制器、接口与关节匹配 |
| MoveIt 可选适配 | Profile 的 `moveit` | SRDF、规划关节与控制器匹配 |

参数页列出缺项并定位字段；保存后返回准备页面重新检查，分别查看离线会话、硬件连接和基础标定条件

导入 Xacro 使用工作台自带的 Python 依赖，`$(find package)` 从模型包、资源搜索路径和已安装资源中解析；无需加载 ROS2 环境

创建 Profile 会收拢所引用的网格包和外部网格资源，生成模型使用包内相对路径；缺少关节限位、模型结构不合法或资源缺失时，先修正指出的 URDF 或资源文件，再创建

### 先离线查看模型

新 Profile 默认 `control.runtime.write_enabled: false`，模型与必要 Core 参数通过检查后即可连接离线会话，硬件插件、型号和电机 ID 可留待实机接入时填写

Terminal 也支持只提供 Core 配置

```bash
./.install/run serial_arm_terminal --config /absolute/path/to/core.yaml --machine
```

离线反馈和执行器能力标记为 mock，仅用于仿真；开放写入并重新连接后读取真实硬件

### 接入硬件驱动

在“硬件与总线”选择与执行器匹配的已安装驱动，或指定共享库路径，再导入该驱动的 Hardware YAML；导入先更新参数草稿，保存后写入当前 Profile 的硬件配置

驱动将设备协议、单位和能力转换为 `MotorBus` 接口，向 Core 提供位置、速度和力矩反馈，完整执行位置、速度、前馈力矩、Kp、Kd 命令；驱动文件检查用于定位安装路径，连接时再检查实际配置

仓库内置达妙驱动；新增执行器需先提供满足上述接口的 Backend，再配置模型与硬件映射，详见 [驱动接口](reference/api.md#motorbus)

### 达妙驱动接入示例

1. 从模型创建 Profile，选择实际 Base 和 Tool，核对自动推导的关节顺序、限位、质量与惯性
2. 在“硬件与总线”选择 `serial_arm_hardware_damiao`，检查驱动文件已安装，再初始化硬件字段或导入已有 Hardware YAML
3. 填写总线、设备路径、波特率，以及每个关节的电机型号、motor ID 和 master ID；同一总线的 motor ID 必须唯一
4. 在“关节映射”核对方向、位置与力矩比例和零位；在安全、停放配置中填写适合机构的限制与停放姿态，核对模板初值
5. 核对机械支撑、急停和运动空间后手动开启写入，保存并连接；先在未使能状态检查每轴反馈，再用低速小幅动作检查方向和零位
6. 完成重力补偿验证，再进行执行器增益自动标定；按需要开展导纳调试、重力模型校正或几何标定

## 区分控制方式、增益用途与任务

| 选择 | 含义 |
| --- | --- |
| 原生阻抗 | 执行器按名义参考和 Kp/Kd 执行，前馈可关闭、仅重力或完整动力学 |
| 模型导纳 | 外力估计驱动 M/D/K 虚拟模型，伺服增益跟随退让参考 |
| 刚性、柔性、拖拽、伺服跟踪 | 四种执行器增益用途 |
| 保持、移动、拖拽 | 当前任务状态 |

刚性与柔性分别保留保持和跟踪数值，模型导纳固定使用伺服跟踪增益；GUI 与 C++ Terminal 的普通示教临时使用原生阻抗和重力补偿，结束后恢复上下文

Python 交互终端使用显式流程：停放失能 → 原生阻抗 → GRAVITY 与有效重力比例 → 使能 → 拖拽；结束后由用户在失能状态切回原控制方式

ROS2 默认用途由 `control.runtime.tracking_gain_profile` 选择 `RIGID`、`COMPLIANT` 或 `SERVO_TRACKING`；旧配置省略该字段时从 `tracking_impedance_mode` 推导刚性或柔性用途

增益标准字段为 `control.controller.servo_tracking`；旧 `identification_tracking` 兼容读取，保存时使用标准名称

`capability.admittance.enabled` 开启外力观测与交互计算，`reference_behavior` 选择默认控制方式；启用观测但选择原生阻抗时，只观测外力，不产生导纳位移

## 第一次连接

先检查模型、驱动资源与设备权限；以下以内置 `dm_arm_gray` 为例，自定义机械臂使用自己的 Profile 名称和 `--profile-file` 路径

```bash
./launch.sh --doctor --profile dm_arm_gray
./.install/run python3 src/serial_arm/core/app/serial_arm_terminal.py \
  --robot-profile dm_arm_gray --check-only
```

写入开关位于 Core YAML

```yaml
control:
  runtime:
    write_enabled: false
```

Terminal 在关闭写入时使用离线 mock；开启后使用实际 Hardware Backend，启动前核对实际值

先确认机械支撑、急停或断电手段，再核对通信、ID、反馈方向、量纲、零位与限位；以小幅低速运动检查每个关节

检查备注供操作者记录观察，连接和控制准入由当前配置、反馈与 Core 安全检查决定

串口、波特率与 Bus 可以覆盖本次连接

```bash
./.install/run serial_arm_terminal \
  --robot-profile dm_arm_gray \
  --serial-port /dev/ttyACM1 \
  --baudrate 921600 \
  --bus main_can
```

## 建立可用控制参数

先完成基础保持与低速跟踪；准备页的基础标定按“重力补偿验证 → 执行器增益自动标定”进行，再调试导纳交互

专项标定包含重力模型校正、高级动力学辨识与几何标定，按任务需要选择

增益标定在当前姿态附近进行多频探测，按用途给出候选与验证结果；伺服跟踪分别验证原生阻抗与模型导纳

先采用已通过的用途，未选用途保留现有值；柔性用途依赖相同的刚性参考，单独采用柔性时需与当前部署的刚性增益匹配

实际应用路径通过示教回放单独验证并保存响应报告，报告标记所测路径；局部探测结果的适用范围仍保留为当前姿态附近

模型导纳先设定目标退让与响应时间，再生成 M/D/K 草稿；实际响应通过交互记录检查，接触任务需按实际环境验证

`K=0` 用于松手减速而不拉回原位置；原生刚性保持补偿与导纳分别配置，补偿只在对应保持条件下工作

## 预览、应用与保存

控制页的滑条和数值框编辑同一组关节目标，青绿色虚影显示预览；点击“移动到目标”才启动运动

“当前位置保持”捕获当前位置；普通模式交接保留原名义目标，受扰退让不会自动成为新保持点

速度比例须在 `(0, 1]`，空值或越界输入会阻止移动；提交成功后使用运动编号与到位状态判断完成，不能用按钮响应成功代替完成反馈

| 操作 | 结果 |
| --- | --- |
| 编辑配置 | 修改待保存值 |
| 覆盖连接参数 | 只影响本次连接 |
| 应用会话参数或候选 | 只影响当前运行环境 |
| 预览差异并保存 | 写入选定配置文件，保留备份 |
| 导出 URDF | 按选定的候选来源与重算轮次生成模型，核对差异后独立验证 |
| 采用 TCP 候选 | 预览后导出独立 URDF、Core 和 Profile，确认后添加为新机械臂配置 |

重算产生的多轮候选可分别选择和比较；保存重力覆盖与导出 URDF 使用同一明确候选，重力覆盖仅影响重力项

正常断开等待停放与失能；立即失能前可靠支撑机械臂，FAULT 或通信中断时现场确认是否仍有保持力矩

详细操作见 [桌面工作台](launcher.md)，接口与兼容字段见 [API 参考](reference/api.md)
