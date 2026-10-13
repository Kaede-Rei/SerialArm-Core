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

源码位于 `src/`，GUI 位于 `apps/launcher/`，安装逻辑位于 `tools/install/`

## 验证与部署

分发仓库不含开发团队的 `tests/`、`test/` 及测试数据夹具

代码修改应先在完整开发环境完成单元与集成测试，随后在目标 Ubuntu、ROS2 与达妙电机上按安全规程进行真机验收

常规使用仍从仓库根目录执行 `./install.sh`，Developer 用途会构建选中的工程组件，但在发布包中自动跳过不存在的测试源码

静态检查不能代替完整 C++ 构建、机械臂轨迹跟踪验证与实际力矩观测验证

## 文档约定

README 保留安装、启动、目录和文档入口，公开文档放在 `docs/`

私人笔记与原始长文放在 `docs/other/`，该目录由 `.gitignore` 排除

已有仓库若曾跟踪其中的文件，需要将它们移出 Git 索引以生效，本地文件可继续保留

```bash
git rm -r --cached --ignore-unmatch docs/other
```

自然语言句末不加标点，代码语法、路径、URL、数值与数学表达保留必要符号
