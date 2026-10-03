# 安装

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
| 2 / `standalone` | Core + Terminal + GUI，不依赖 ROS2，DM-Arm 支持可选 |
| 3 / `python` | Core + Python Binding，独立 Python 环境，无 Terminal、ROS2 或 GUI |
| 4 / `ros2` | Core + Terminal + Python + ROS2 Adapter + DM-Arm + MoveIt + GUI |
| 5 / `dev` | 包含选项 4，并增加测试依赖与测试执行 |
| 6 / `custom` | Core 起步，逐项选择组件，自动补齐依赖 |

选项按集成需求组织，Python 是独立接口选择，并非每个编号都包含前一项

ROS2 必须包含 Python Binding，内置 MoveIt 必须包含 ROS2 与 DM-Arm 支持

选项 2、4、5 默认启用 DM-Arm，选项 2 可以在向导中取消

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
./.install/run python -c 'import serial_arm; print(serial_arm.__version__)'

# 或在当前终端加载一次环境
source .install/setup.bash
```

命令需要对应组件已安装，Terminal 运行可能连接真机

构建与安装产物按组件组合放在 `build/unified/` 和 `install/unified/`

`.install/manifest.json` 保存最后通过离线检查的安装状态，`.install/last_attempt.json` 保存最近结果

离线检查包括库加载、Terminal help、Python import、ROS package 与资源存在性，不启动机器人

失败不会提交新 manifest，但同一安装前缀可能留下部分产物，需要完成重试

仓库搬移后重新安装以更新环境路径，安装器不修改 shell 启动文件、Profile 或串口权限

手动构建与验证命令见 [开发说明](development.md)
