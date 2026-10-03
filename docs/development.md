# 开发与验证

## 手动构建

常规使用优先执行 `./install.sh`，只有自定义构建流程时使用以下入口

```bash
# Standalone，无 ROS2
./tools/bootstrap_standalone.sh --robot dm_arm
source install/standalone/setup.bash

# 仅库或同时运行开发测试
./tools/bootstrap_standalone.sh --without-terminal
./tools/bootstrap_standalone.sh --with-tests

# ROS2 全仓构建
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

源码位于 `src/`，GUI 位于 `apps/launcher/`，安装逻辑位于 `tools/install/`

## 离线测试

```bash
python3 -m unittest discover -s tools/install/tests -v
python3 src/serial_arm/core/tests/bootstrap_standalone_cli_test.py
python3 src/serial_arm/core/tests/standalone_dependency_contract.py
python3 -m unittest discover -s apps/launcher/tests -v
node --test apps/launcher/tests/*.test.cjs
node apps/launcher/scripts/verify.cjs
```

安装器测试用真实控制终端和假 sudo/apt 重现密码交互，验证提示可见与密码不入日志

Launcher 测试用假进程覆盖 PTY、配置确认、设备互斥与退出，不驱动机器人

安装 GUI 依赖后可执行 Electron 界面测试

```bash
SERIAL_ARM_LAUNCHER_PYTHON="$PWD/.install/gui-venv/bin/python" \
  apps/launcher/node_modules/electron/dist/electron \
  apps/launcher/tests/electron-smoke.cjs
```

界面测试使用离线状态，检查 Profile、模式、确认、主题、语言、偏好与 IPC 限制

终端测试验证系统剪贴板复制、复制全部、右键菜单、粘贴输入与 Ctrl+C，输入由测试夹具接收

以上检查不替代目标环境的 Core/ROS2 全栈编译与真机验收

## 文档约定

README 保留安装、启动、目录和文档入口，公开文档放在 `docs/`

私人笔记与原始长文放在 `docs/other/`，该目录由 `.gitignore` 排除

已有仓库若曾跟踪其中的文件，需要将它们移出 Git 索引以生效，本地文件可继续保留

```bash
git rm -r --cached --ignore-unmatch docs/other
```

自然语言句末不加标点，代码语法、路径、URL、数值与数学表达保留必要符号
