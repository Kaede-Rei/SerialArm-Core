#!/usr/bin/env python3
"""Installer entry point; --dry-run remains side-effect free"""
import json
from pathlib import Path
import platform
import shutil
import signal
import sys

from plan import COMPONENTS, PRESETS, parse_plan, parser

ROOT = Path(__file__).resolve().parents[2]


def ask_bool(message, default):
    while True:
        answer = input(message + (' [y/n，默认 y] ' if default else ' [y/n，默认 n] ')).strip().lower()
        if not answer:
            return default
        if answer in ('y', 'yes', 'n', 'no'):
            return answer in ('y', 'yes')
        print('请输入 y 或 n')


def wizard(argv):
    print('SerialArm-Core 安装向导')
    print(f'系统：{platform.system()} {platform.machine()} | Python {platform.python_version()}')
    print('工具：' + ' / '.join(f'{c}: {shutil.which(c) or "未检测到"}'
                               for c in ('cmake', 'c++', 'conan', 'colcon', 'rosdep')))
    names = list(PRESETS)
    labels = [
        ('基础核心库 / C++ Core', '安装 Core 与 Dynamics，用于 C++ 集成，不含调试终端、Python、ROS2 或 GUI'),
        ('独立机械臂调试 / Standalone', '安装 Core、C++ Terminal 与 GUI，不依赖 ROS2，内置 DM-Arm 支持可选'),
        ('Python 接口开发 / Python Binding', '安装 Core 与 Python Binding，使用独立 Python 环境，不含 Terminal、ROS2 或 GUI'),
        ('ROS2 控制与规划 / ROS2 + MoveIt', '安装 Core、Terminal、Python Binding、ROS2 Adapter、DM-Arm、MoveIt 与 GUI'),
        ('源码开发与测试 / Developer', '包含 ROS2 控制与规划的全部组件，并安装测试依赖、运行开发测试'),
        ('自定义组件 / Custom', '从 Core 起步逐项选择组件，自动补齐依赖，例如 ROS2 需要 Python Binding'),
    ]
    default = '4' if Path('/opt/ros/humble/setup.bash').exists() else '2'
    print('从核心库到独立调试、ROS2 集成、开发测试，集成需求依次增加')
    print('Python 接口是独立选择，选项并非按编号逐级包含')
    for i, (label, detail) in enumerate(labels, 1):
        print(f'  {i}  {label}\n     {detail}')
    while True:
        choice = input(f'选择用途 [默认 {default}] ').strip() or default
        if choice in [str(i) for i in range(1, 7)]:
            break
        print('请输入 1 到 6')
    preset = names[int(choice) - 1]
    extra = ['--preset', preset]
    if preset == 'custom':
        descriptions = {
            'terminal': 'C++ 交互终端（连接机械臂并调试控制功能）',
            'python': 'Python Binding（在 Python 中调用 Core）',
            'ros2': 'ROS2 Adapter（接入 ros2_control，同时包含 Python Binding）',
            'moveit': 'MoveIt（内置 DM-Arm 规划配置，同时包含 ROS2 与 DM-Arm 支持）',
            'gui': 'GUI（Profile、运行入口与诊断，需要 Node.js 20+ 和 npm）',
            'tests': '开发测试（额外安装测试依赖并执行测试）',
        }
        for c in COMPONENTS:
            if ask_bool('安装 ' + descriptions[c], False):
                extra.append('--with-' + c)
    if preset in ('standalone', 'ros2', 'dev', 'custom', 'python'):
        robot = ask_bool('安装内置 DM-Arm 支持（外部 Profile 可选 n，MoveIt 会自动包含 DM-Arm）',
                         preset in ('standalone', 'ros2', 'dev'))
        extra += ['--robot', 'dm_arm' if robot else 'none']
        if not robot and preset in ('ros2', 'dev'):
            extra.append('--without-moveit')
    return extra + argv


def interrupted(signum, frame):
    raise KeyboardInterrupt


def main():
    if sys.version_info < (3, 10):
        print('需要 Python 3.10+', file=sys.stderr)
        return 2
    signal.signal(signal.SIGTERM, interrupted)
    argv = sys.argv[1:]
    if '--gui-only' in argv and not any(a == '--preset' or a.startswith('--preset=') for a in argv):
        argv = ['--preset', 'custom', *argv]
    if not any(a == '--preset' or a.startswith('--preset=') for a in argv):
        if '--help' not in argv and '-h' not in argv:
            if not sys.stdin.isatty() or '--yes' in argv or '--dry-run' in argv:
                parser().error('非交互模式请提供 --preset，例如 --preset standalone --yes')
            try:
                argv = wizard(argv)
            except (EOFError, KeyboardInterrupt):
                return 130
    plan = parse_plan(argv, ROOT)
    if plan.dry_run:
        print(json.dumps(plan.describe(), ensure_ascii=False, indent=2))
        return 0
    print(json.dumps(plan.describe(), ensure_ascii=False, indent=2))
    if sys.stdin.isatty() and not plan.yes:
        try:
            if not ask_bool('按以上计划安装', True):
                return 0
        except (EOFError, KeyboardInterrupt):
            return 130
    from runner import install
    return install(plan)


if __name__ == '__main__':
    raise SystemExit(main())
