# 安装

SerialArm-Core 支持交互式安装和直接作为 ROS2 工作区中的 CMake 包构建，两种方式使用同一套核心库

![工作区启动界面](images/launcher-start.png)

## 构建方式一览

| 场景 | 推荐方式 |
| :--- | :--- |
| 首次使用桌面工作台 | `./install.sh` 后执行 `./launch.sh` |
| 已有 ROS2 工作区 | 在工作区根目录直接运行 `colcon build` |
| 独立 C++ 项目 | 按需安装 Core 后使用导出的 CMake Target |
| 调整安装组件 | 使用交互向导或安装脚本选项 |

### 直接在 ROS2 工作区构建

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Python Binding 默认启用，使用仓库内置的 pybind11 构建依赖，不会在配置期间联网获取该依赖

## 交互安装


在仓库根目录运行

```bash
./install.sh
```

Linux / Python 3.10+，ROS2 路径需预先安装 ROS2，GUI 需要 Node.js 20+ 和 npm

输入提示标注 `[默认 4]` 或 `[y/n，默认 y]`，直接 Enter 使用默认值

| 选项 | 内容与用途 |
| --- | --- |
| 1 / `core` | Core + Dynamics，用于 C++ 集成 |
| 2 / `standalone` | Core + Terminal + Model Probe + GUI，不依赖 ROS2，DM-Arm 支持可选 |
| 3 / `python` | Core + Python Binding，独立 Python 环境，无 Terminal、ROS2 或 GUI |
| 4 / `ros2` | Core + Terminal + Python + ROS2 Adapter + DM-Arm + MoveIt + GUI |
| 5 / `dev` | 包含选项 4，并增加测试依赖与测试执行 |
| 6 / `custom` | Core 起步，逐项选择组件，自动补齐依赖 |

选项按集成需求组织，Python 是独立接口选择，并非每个编号都包含前一项

ROS2 必须包含 Python Binding，内置 MoveIt 必须包含 ROS2 与 DM-Arm 支持

选项 2、4、5 默认启用 DM-Arm，选项 2 可以在向导中取消

## Linux AppImage 与 deb 发行包

从源码构建完整 Linux 发行包使用第 4 项对应的 `ros2` 安装预设，而非 `standalone`

```bash
./build_linux.sh
```

脚本优先复用已经通过验证的完整 ROS2 安装，未安装时自动执行完整 ROS2 / DM-Arm / MoveIt / GUI 安装，再将编译后的 ROS2 工作区、Python Binding、机器人资源和 Electron 工作台打包

Electron 构建时直接复用 `apps/launcher/node_modules/electron/dist` 已安装的运行时，不重复从网络下载 Electron

已通过完整安装验证时，可以明确要求复用现有安装，如果没有有效安装状态则直接报错

```bash
./build_linux.sh --reuse-install
```

如需强制重新编译完整组件，可以执行 `./build_linux.sh --rebuild`

构建前需要 Ubuntu 22.04 x86_64、ROS2 Humble、Node.js 20+、npm 和系统 Python，打包使用 electron-builder

两个产物及校验文件均输出到 `dist/`

```text
SerialArm-Core-linux-x86_64.AppImage
SerialArm-Core-linux-amd64.deb
SHA256SUMS
```

### 安装发行包

发行包包含 SerialArm-Core 自己编译的 C++ Core、Python Binding、DM-Arm 驱动、ROS2 Adapter、MoveIt 配置与 GUI，但不重复封装整个 ROS2 Humble 系统发行版

目标机器需要 Ubuntu 22.04 x86_64，并先配置 ROS2 Humble apt 软件源及 MoveIt 2、ros2_control 等系统依赖

安装 deb

```bash
sudo apt install ./dist/SerialArm-Core-linux-amd64.deb
```

运行 AppImage

```bash
chmod +x dist/SerialArm-Core-linux-x86_64.AppImage
./dist/SerialArm-Core-linux-x86_64.AppImage
```

AppImage 依赖目标机器的 ROS2 Humble 环境和 AppImage 所需的 libfuse2 等系统运行库

首次启动时，内置机器人 Profile 资源复制到当前用户的配置目录，用于保存参数变更和标定数据，不会写回发行包只读资源

目标机无需执行源码仓库的 `install.sh` 或重新编译 C++ 与 ROS2 Adapter

建议在干净 Ubuntu 22.04 系统上分别检查 GUI、模型显示、Profile 保存、Terminal、ROS2 控制链及 MoveIt 规划，再发布 GitHub Release

## sudo 与日志

apt 和 rosdep 在当前终端显示输出与密码提示，同时记录命令输出

密码由 sudo 直接读取，安装器不读取或记录密码

`--yes` 仅跳过安装确认，仍可能要求 sudo 密码

无终端运行需要已授权的 sudo 凭据或免密权限，缺少权限会退出并给出提示

运行日志位于 `log/unified/<时间>/`，失败后修复日志中的问题并重跑同一命令

## 常用命令

```bash
# 自动确认 ROS2 全栈安装
./install.sh --preset ros2 --yes

# 外部机器人，仅构建通用 ROS2 Adapter
./install.sh --preset ros2 --robot none --without-moveit --yes

# 自定义非 ROS 的 DM-Arm + Python + Terminal + GUI + Tests
./install.sh --preset custom --robot dm_arm \
  --with-terminal --with-python --with-gui --with-tests --yes

# 单独补装 GUI，保留已有 Core 安装状态
./install.sh --gui-only --yes

# 只输出 JSON 计划，不安装或写文件
./install.sh --preset ros2 --dry-run
```

非交互运行必须提供 `--preset`，`standalone-dm` 是 `standalone` 的兼容别名

| 参数 | 作用 |
| --- | --- |
| `--with-<组件>` / `--without-<组件>` | 组件为 terminal、python、ros2、moveit、gui、tests |
| `--robot dm_arm` / `--robot none` | 启用或省略内置机器人支持 |
| `--jobs N` | 并行数，默认 CPU 数且上限 8 |
| `--ros-distro humble` | 使用 `/opt/ros/<发行版>/setup.bash` |
| `--skip-system-deps` | 跳过 apt 与 rosdep，系统依赖自行准备 |
| `--allow-source-build` | 明确允许 Conan 缺少 binary 时编译依赖源码 |

系统依赖齐全时直接使用 CMake packages，否则优先获取 Conan binary

缺少 Conan 时准备本地工具 venv，`--yes` 不会允许大型依赖源码回退

独立 Python Binding 安装在安装前缀的 venv 中，GUI 使用 `.install/gui-venv`

## 安装后

```bash
./launch.sh
./.install/run serial_arm_terminal --robot-profile dm_arm_gray
./.install/run serial_arm_model_probe --help
./.install/run python -c 'import serial_arm; print("serial_arm import OK")'

# 或在当前终端加载一次环境
source .install/setup.bash
```

命令需要对应组件已安装，Terminal 运行可能连接真机

构建与安装产物按组件组合放在 `build/unified/` 和 `install/unified/`

`.install/manifest.json` 保存最后通过离线检查的安装状态，`.install/last_attempt.json` 保存最近结果

离线检查包括库加载、Terminal help、Model Probe help、Python import、ROS package 与资源存在性，不启动机器人

失败不会提交新 manifest，但同一安装前缀可能留下部分产物，需要完成重试

仓库搬移后重新安装以更新环境路径，安装器不修改 shell 启动文件、Profile 或串口权限

手动构建与验证命令见 [开发说明](development.md)
