# 开发与验证

## 手动构建

常规使用优先执行 `./install.sh`，只有自定义构建流程时使用以下入口

```bash
# Standalone，无 ROS2
./tools/bootstrap_standalone.sh --robot dm_arm
source install/standalone/setup.bash

# 仅构建核心库
./tools/bootstrap_standalone.sh --without-terminal

# ROS2 全仓构建
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

在下游 ROS2 工作区中也可以直接运行 `colcon build`，Python Binding 默认参与构建

Core、Python Binding 和桌面工作台的原生构建不依赖 ROS2；准备 Pinocchio、Eigen、yaml-cpp 和 Python 开发环境，pybind11 使用仓库内置依赖；只有构建适配层才准备 ROS2

源码位于 `src/`，GUI 位于 `apps/launcher/`，安装逻辑位于 `tools/install/`

## 验证与部署

分发仓库包含可复现的控制、安全、模型与设备独占契约测试，使用 `BUILD_TESTING=ON` 构建后执行 `ctest --output-on-failure`

控制修改先完成编译、单元与集成测试，再在目标系统和机械臂上验证轨迹、力矩观测与安全停止

常规使用从仓库根目录执行 `./install.sh`；日常更新只编译受影响的目标并执行对应检查，按机器负载选择 `--jobs`，低负载构建可使用 `--jobs 1`

`.install/setup.bash` 和 `.install/run` 默认加载原生环境；显式执行 `./.install/run ros2 ...` 或加载 `.install/ros2-setup.bash` 才进入 ROS2 适配环境，GUI 按相同边界启动

Developer 预设会构建选中的工程组件并执行随源码提供的契约测试；前端与后台脚本修改可单独执行对应回归，不需要生成发行包

准备 Release 时才运行 `./build_linux.sh`，默认打包独立 Core、Python 与 GUI；使用 `--with-ros2` 才附加 ROS2 / MoveIt 适配层，详见 [安装](install.md)

## 文档约定

README 保留安装、启动、目录和文档入口，公开文档放在 `docs/`

私人笔记与原始长文放在 `docs/other/`，该目录由 `.gitignore` 排除

已有仓库若曾跟踪其中的文件，需要将它们移出 Git 索引以生效，本地文件可继续保留

```bash
git rm -r --cached --ignore-unmatch docs/other
```

自然语言句末不加标点，代码语法、路径、URL、数值与数学表达保留必要符号
