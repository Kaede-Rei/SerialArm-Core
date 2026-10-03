# 统一安装入口

在仓库根目录执行：

```bash
./install.sh
```

交互向导展示系统与构建工具，选择用途、内置机器人支持，然后展示统一安装计划
选择提示使用 `[默认 4]` 等形式；是非提示使用 `[y/n，默认 y]` 或 `[y/n，默认 n]`
直接按 Enter 使用标注的默认值
Linux / Python 3.10+；ROS2 路径以 Ubuntu 22.04 + Humble 为主要目标
首次系统依赖安装可能需要 sudo；默认只在本仓库安装 SerialArm 产物
安装器不会启动机器人、修改 Profile、修改串口权限或修改 shell 启动文件

## 预设

| 预设 | 默认安装内容 |
| --- | --- |
| `core` | C++ Core / Dynamics，无 Terminal、Python、ROS、GUI |
| `standalone` | Core + Terminal + DM-Arm 支持；GUI 不可用 |
| `ros2` | Core + Terminal + ROS2 Adapter + DM-Arm + MoveIt；GUI 不可用 |
| `python` | Core + Python Binding（独立 venv），无 ROS |
| `dev` | ROS2 全栈 + Python + Tests；GUI 不可用 |
| `custom` | Core 起步，按组件开关选择 |

`standalone-dm` 是 `standalone` 的兼容别名
`--robot none` 不安装 DM 硬件后端，可用于下游 Profile
当前内置 MoveIt 配置只有 DM-Arm，选择 MoveIt 会自动补上 ROS2 和 DM-Arm
外部机器人通常使用 `--robot none --without-moveit`，由下游维护自己的 MoveIt 配置
ROS2 测试可能额外构建 `dm_arm_description` 作为测试资源，不代表安装了 DM 硬件后端

GUI 不可用，不安装 Node/Electron；运行已安装组件请使用 `./.install/run`

## 非交互与自定义

指定 `--preset` 后不询问组件；有 TTY 时仍展示确认，`--yes` 跳过确认
无 TTY 时必须指定预设，绝不会等待菜单输入

```bash
./install.sh --preset standalone --yes
./install.sh --preset ros2 --yes
./install.sh --preset core --yes
./install.sh --preset python --yes
./install.sh --preset dev --yes

# 已由下游提供机器人资源；只构建通用 ROS2 Adapter
./install.sh --preset ros2 --robot none --without-moveit --yes

# 自定义非 ROS 的 DM + Python + Terminal
./install.sh --preset custom --robot dm_arm --with-python --with-terminal --yes

# 查看计划，不创建文件、不调用 apt/pip/Conan/colcon
./install.sh --preset standalone --dry-run
```

组件开关：`--with-terminal/python/ros2/moveit/gui/tests` 及对应 `--without-*`
其他参数：

- `--jobs N`：并行数，默认 CPU 数且上限 8，可显式覆盖
- `--ros-distro humble`：选择 `/opt/ros/<distro>/setup.bash`
- `--skip-system-deps`：跳过 apt 和 rosdep；构建依赖由用户提供必要的 Conan/Python venv 仍会调用 pip
- `--allow-source-build`：允许 Conan 缺少兼容 binary 时编译依赖源码

`--yes` **不会**允许大型源码回退；安装器固定此策略，不继承外部
`SERIAL_ARM_ALLOW_SOURCE_BUILD=1`；如果二进制不可用，请明确选择：

```bash
./install.sh --preset standalone --allow-source-build --yes
```

系统依赖完整时直接复用系统 CMake packages，不调用或安装 Conan
不完整时复用 bootstrap 的 Conan binary-first 策略；缺少 Conan 2 时在
`.install/tools-venv` 中准备它，不使用 `pip --user` 或破坏系统 Python
Python binding 在安装前缀的独立 venv 中构建 wheel，复用 bootstrap 实际选中的依赖

ROS2 路径要求 ROS 已安装；缺少时在 apt 操作前退出，给出所缺 setup 路径
安装器不改系统软件源、不自动安装整个 ROS 发行版
rosdep/colcon 只处理计划选中的包；未选 MoveIt 时不构建其配置包，未选 Python 时关闭 Binding
开启 Tests 才安装测试依赖、运行 ctest 或 colcon test，并检查测试结果

## 安装后使用

无需记住不同的 setup 路径：

```bash
# 使用环境包装入口，不需要手动 source
./.install/run serial_arm_terminal --robot-profile dm_arm_gray
./.install/run ros2 launch serial_arm_ros2_control display.launch.py robot_profile:=dm_arm_gray
./.install/run python -c 'import serial_arm; print(serial_arm.__version__)'
```

上述命令需要对应组件已安装；运行 Terminal 属于真机使用，安装过程仅执行其 `--help`
使用下游 Profile：

```bash
./.install/run serial_arm_terminal \
  --robot-profile tomato_picker \
  --profile-file /absolute/path/to/robot_profiles.yaml
```

下游资源与 hardware plugin 仍由下游项目提供；可继续通过
`SERIAL_ARM_RESOURCE_PATH` 或 ROS launch 的 `resource_paths` 指定资源

也可以在当前终端一次加载统一环境，直接使用组件命令：

```bash
source .install/setup.bash
serial_arm_terminal --help
```

安装器通过 bootstrap、colcon 和 Python wheel 构建组件
`tools/bootstrap_standalone.sh` 支持 `--without-terminal`，可用于仅安装库

## 状态、日志和重试

- `.install/manifest.json`：最后通过离线检查的安装状态，`schema_version=1`
- `.install/setup.bash` / `.install/run`：加载 manifest 选定的环境
- `.install/last_attempt.json`：最近一次安装是否成功、失败阶段与日志位置
- `build/unified/<组件组合>/` 与 `install/unified/<组件组合>/`：独立构建与安装目录
- `log/unified/<时间>/`：每个步骤的完整输出，运行时每 15 秒报告进度

验证包含共享库实际加载、Terminal `--help`、选中 ROS package 查询、Python import 和机器人资源存在性
只有全部通过后才原子提交 manifest；失败不会覆盖最后成功的 manifest
同一组件组合重试会复用构建目录，失败可能留下部分产物，因此旧 manifest 是历史验证记录，
不保证同一前缀经历失败重建后仍完整；请检查 `last_attempt.json` 并完成重试
不同组件组合使用独立前缀，防止上一组合残留的可执行程序被当作此次成果

同时只允许一个安装进程；Ctrl+C 会终止当前步骤的子进程组并记录失败
解决日志中的缺依赖或网络问题后重新执行同一安装命令即可
暂不支持修复/卸载子命令；不生成桌面图标、不更改全局 PATH
仓库搬移后请重新运行安装器，生成的新环境路径才会指向新位置

## 自动测试

自动测试覆盖计划归一化、dry-run 不写文件、非交互无挂起、组件排除、外部 Profile 组合、
ROS 命令参数、安装锁、离线动态库加载、失败不提交 manifest，以及带空格路径的环境加载
旧 bootstrap CLI 的六项 system/Conan/test 路径测试也继续通过
自动测试不访问或驱动机器人；不包含依赖下载、ROS2 全栈编译、Python wheel ABI 或真机运行验收

```bash
python3 -m unittest discover -s tools/install/tests -v
python3 src/serial_arm/core/tests/bootstrap_standalone_cli_test.py
```
