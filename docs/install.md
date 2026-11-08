# 安装

SerialArm-Core 的 Core、Terminal、Python 与桌面工作台可独立运行；ROS2 和 MoveIt 是按需安装的适配层

## 选择安装方式

| 场景 | 推荐方式 |
| :--- | :--- |
| Ubuntu 22.04 x86_64，直接使用桌面工作台 | 下载 Release 中的 AppImage 或 deb，见[安装发行包](#安装发行包) |
| 从源码使用桌面工作台 | 在仓库根目录执行 `./install.sh`，完成后执行 `./launch.sh` |
| 已有 ROS2 工作区 | 在工作区根目录直接运行 `colcon build` |
| 独立 C++ 项目 | 安装 Core，按 [开发说明](development.md#c-下游集成) 链接导出的 CMake Target |
| 修改源码或准备 Release | 安装所改组件，构建与验证见 [开发说明](development.md) |

## 安装发行包

发行包面向 Ubuntu 22.04 x86_64，包含 Core、Terminal、模型工具、Python Binding、DM-Arm 支持和桌面工作台；ROS2 与 MoveIt 适配层单独选装；下载后在文件所在目录选择一种安装方式

```bash
sudo apt install ./SerialArm-Core-linux-amd64.deb
```

```bash
chmod +x SerialArm-Core-linux-x86_64.AppImage
./SerialArm-Core-linux-x86_64.AppImage
```

AppImage 需要 `libfuse2` 等系统运行库；发行包携带原生非系统依赖、Python Binding、科学计算模块与 Xacro，`$(find package)` 使用项目资源搜索路径

首次启动将内置 Profile 复制到用户配置目录，参数与标定数据不写入只读发行包；机械臂导入与连接见 [使用指南](guide.md)

## 从源码安装

源码安装器支持 Linux，使用 apt 准备系统依赖；需要 Python 3.10+，GUI 另需 Node.js 20+ 和 npm；原生构建使用 CMake 3.20+、C++17、Pinocchio、Eigen3 与 yaml-cpp，已有系统依赖时使用 `--skip-system-deps`

在仓库根目录启动交互向导

```bash
./install.sh
```

向导默认选择独立机械臂调试，按 Enter 接受提示中的默认选项

| 选项 | 内容与用途 |
| --- | --- |
| 1 / `core` | C++ Core 库，用于独立集成 |
| 2 / `standalone` | Core、Terminal、模型工具和 GUI，不依赖 ROS2，DM-Arm 支持可选 |
| 3 / `python` | Core 与 Python Binding，使用独立 Python 环境 |
| 4 / `ros2` | Core、Terminal、Python Binding、ROS2 Adapter、DM-Arm、MoveIt 和 GUI |
| 5 / `dev` | 全部组件，包含 ROS2 / MoveIt |
| 6 / `custom` | Core 起步，逐项选择组件，自动补齐依赖 |

预设按用途区分，不按编号逐级包含；`standalone` 默认不安装 Python Binding，可用 `--with-python` 添加

ROS2 预设要求宿主已安装 ROS2；ROS2 Adapter 依赖 Python Binding，仓库内置的 MoveIt 配置还依赖 DM-Arm 支持

选项 2、4、5 默认启用 DM-Arm，包含达妙驱动与机械臂示例，选项 2 可在向导中取消；其他机械臂选择 `--robot none` 后安装所需驱动和资源，按 [使用指南](guide.md) 创建 Profile

### 常用命令

```bash
# 独立工作台，按需添加 Python Binding
./install.sh --preset standalone --with-python --jobs 1 --yes

# 明确选择 ROS2 与 MoveIt 适配层
./install.sh --preset ros2 --yes

# 外部机器人，仅构建通用 ROS2 Adapter
./install.sh --preset ros2 --robot none --without-moveit --yes

# 自定义非 ROS 的 DM-Arm + Python + Terminal + GUI
./install.sh --preset custom --robot dm_arm \
  --with-terminal --with-python --with-gui --yes

# 单独补装 GUI，保留已有 Core 安装状态
./install.sh --gui-only --yes

# 只输出 JSON 计划，不安装或写文件
./install.sh --preset ros2 --dry-run
```

非交互运行需指定 `--preset`，`--gui-only` 除外；`standalone-dm` 是 `standalone` 的兼容别名

| 参数 | 作用 |
| --- | --- |
| `--with-<组件>` / `--without-<组件>` | 组件为 terminal、python、ros2、moveit、gui |
| `--robot dm_arm` / `--robot none` | 启用或省略内置机器人支持 |
| `--jobs N` | 并行数，默认 CPU 数且上限 8 |
| `--ros-distro humble` | 选择 `/opt/ros/<发行版>/setup.bash`，默认使用当前 `ROS_DISTRO` 或 humble |
| `--skip-system-deps` | 跳过 apt 与 rosdep，系统依赖自行准备 |
| `--allow-source-build` | 允许 Conan 在缺少二进制包时编译依赖源码 |
| `--clean-rebuild` | 清理所选配置的构建与安装目录后重建，用于源码目录迁移恢复 |

依赖优先使用系统 CMake 包和 Conan 二进制包，缺少 Conan 时准备本地工具 venv；源码回退需显式选择 `--allow-source-build`，`--yes` 不开启该回退；独立 Python Binding 使用安装前缀中的 venv，GUI 使用 `.install/gui-venv`

### sudo 与日志

apt 和 rosdep 的输出及密码提示显示在当前终端，密码由 sudo 直接读取；`--yes` 仅跳过安装确认，无终端运行需有可用 sudo 授权；日志位于 `log/unified/<时间>/`，失败后修正日志中的问题并重跑同一命令

### 安装后

```bash
./launch.sh
./.install/run serial_arm_model_probe --help

# 安装了 Python Binding 时检查导入
./.install/run python -c 'import serial_arm; print("serial_arm import OK")'

# 或在当前终端加载一次环境
source .install/setup.bash
```

命令需要对应组件已安装；Terminal 使用以下入口，开启控制写入的 Profile 可能连接真机，连接前先核对设备、关节映射和运行限制

```bash
./.install/run serial_arm_terminal --robot-profile dm_arm_gray
```

产物按组件组合放在 `build/unified/` 和 `install/unified/`；`.install/manifest.json` 保存最后通过检查的安装，`.install/last_attempt.json` 保存最近结果；失败可能留下部分产物，修正问题后完成重试

安装检查覆盖原生库、命令 help、Python 与 Xacro 导入及所选资源，不连接机械臂；开发验证见 [开发说明](development.md#检查改动)

源码目录迁移后，按安装器提示使用原组件选项加 `--clean-rebuild` 重建；该选项会删除对应构建和安装目录，安装器不修改 shell 启动文件、Profile 或串口权限

## ROS2 / MoveIt 适配层

仅启动 ROS2 Hardware 或 MoveIt 入口时加载宿主 ROS2；缺少适配层不影响原生 Core 和工作台；源码安装后按需加载适配环境

```bash
source .install/ros2-setup.bash

# 也可由运行入口自动选择适配环境
./.install/run ros2 pkg prefix serial_arm_ros2_control
```

可选适配运行环境按需安装

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-moveit
```

### 直接在 ROS2 工作区构建

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build
source install/setup.bash
```

Python Binding 默认参与该构建，pybind11 使用仓库内置依赖；GUI 仍需单独执行 `./install.sh --gui-only --yes`

## 图形启动与诊断

GUI 默认走正常 GPU 路径，不附加兼容后端参数；源码启动可显式选择图形模式

```bash
./launch.sh --graphics-mode=normal
# 主动接受较低的软件 WebGL 安全保证，使用兼容三维
./launch.sh --graphics-mode=compatibility
./launch.sh --graphics-mode=rescue
```

兼容模式使用 Electron 自带的 ANGLE SwiftShader 软件后端，启用 Chromium 的 `enable-unsafe-swiftshader` 会降低软件 WebGL 在 GPU 进程中的安全保证，Renderer 沙箱仍启用；应用内切换时需要确认该影响，命令行显式选择 `--graphics-mode=compatibility` 也视为主动同意

无三维救援模式用于保留配置、数值状态、记录与导出，不能替代姿态预览确认

应用内也可从设置或预览故障面板切换模式；图形后端改变后需重启，有运行会话时不会自动退出或提交停放命令

WebGL 初始化失败时在预览区域查看或导出图形诊断，核对 WebGL2、GPU、驱动、后端、启动参数、运行库与日志，再重试或选择兼容模式；详细操作见 [使用指南](guide.md#三维兼容与恢复)

图形检查需实际创建渲染器、加载模型并完成绘制，模型数据检查不能代替渲染验证

## 构建 Linux 发行包

本节供发布维护者使用，仅准备 Release 时生成 AppImage 和 deb；默认构建独立发行包，`--with-ros2` 附加 ROS2 与 MoveIt 适配层

```bash
./build_linux.sh
```

脚本复用与源码匹配的独立安装，不匹配时单任务重建；Electron 使用已安装的 `apps/launcher/node_modules/electron/dist`，保留 ANGLE 与 SwiftShader 图形运行库供正常和兼容三维使用

```bash
# 只复用与源码一致的独立安装，不自动重建
./build_linux.sh --reuse-install

# 明确重建选定组件后生成发布包
./build_linux.sh --rebuild

# 可选：额外包含 ROS2 与 MoveIt 适配组件
./build_linux.sh --with-ros2
```

构建需要 Ubuntu 22.04 x86_64、Node.js 20+、npm、系统 Python 和 `flock`（`util-linux` 包）；发布校验还需要 `dpkg-deb`（`dpkg` 包）与 `unsquashfs`（`squashfs-tools` 包），脚本会在安装或构建前检查这些命令

```bash
sudo apt install dpkg squashfs-tools util-linux
```

只有选择 `--with-ros2` 才需要构建机安装 ROS2 Humble 与 MoveIt

两个产物及校验文件输出到 `dist/`

```text
SerialArm-Core-linux-x86_64.AppImage
SerialArm-Core-linux-amd64.deb
SHA256SUMS
```

将三个文件放在同一目录后，执行 `sha256sum -c SHA256SUMS` 校验

Release 前在不含 ROS2 的 Ubuntu 22.04 环境检查 GUI、模型、Profile、Terminal 和 Python；若发布适配层，再单独检查 ROS2 与 MoveIt

手动构建与开发检查见 [开发说明](development.md)
