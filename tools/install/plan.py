"""Pure installation planning shared by CLI and interactive wizard"""
import argparse
import os
import re
from dataclasses import asdict, dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

PRESETS = {
    'core': (False, False, False, False, False, False, 'none'),
    'standalone': (True, False, False, False, True, False, 'dm_arm'),
    'python': (False, True, False, False, False, False, 'none'),
    'ros2': (True, False, True, True, True, False, 'dm_arm'),
    'dev': (True, True, True, True, True, True, 'dm_arm'),
    'custom': (False, False, False, False, False, False, 'none'),
}
COMPONENTS = ('terminal', 'python', 'ros2', 'moveit', 'gui', 'tests')

@dataclass
class Plan:
    preset: str
    terminal: bool
    python: bool
    ros2: bool
    moveit: bool
    gui: bool
    tests: bool
    robot: str
    ros_distro: str
    jobs: int
    yes: bool
    dry_run: bool
    skip_system_deps: bool
    allow_source_build: bool
    root: str
    gui_only: bool = False

    @property
    def install_key(self):
        mode = 'ros2-' + self.ros_distro if self.ros2 else 'standalone'
        return f'{mode}-{self.robot}-t{int(self.terminal)}-p{int(self.python)}-m{int(self.moveit)}-test{int(self.tests)}'

    @property
    def packages(self):
        if self.gui_only: return []
        names = ['serial_arm_core']
        if self.ros2:
            names += ['serial_arm_robot_profiles', 'serial_arm_ros2_control']
        if self.robot == 'dm_arm':
            names += ['serial_arm_protocol_damiao_usb2can', 'serial_arm_hardware_damiao',
                      'serial_arm_robot_profiles', 'dm_arm_description']
        if self.ros2 and self.tests:
            names += ['dm_arm_description']  # Adapter test fixture only
        if self.moveit:
            names += ['dm_arm_no_gripper', 'dm_arm_with_gripper']
        return list(dict.fromkeys(names))

    def package_paths(self):
        found = {}
        for path in (Path(self.root) / 'src').rglob('package.xml'):
            found[ET.parse(path).findtext('name')] = str(path.parent)
        return [found[name] for name in self.packages]

    def describe(self):
        data = asdict(self)
        data.update(packages=self.packages, gui_status='selected' if self.gui else 'not_requested',
                    workspace_setup='.install/setup.bash',
                    install_prefix='install/unified/' + self.install_key)
        return data


def parser():
    p = argparse.ArgumentParser(description='SerialArm-Core 统一安装器（Linux / Python 3.10+）')
    p.add_argument('--preset', choices=(*PRESETS, 'standalone-dm'))
    p.add_argument('--gui-only', action='store_true', help='只安装 GUI，复用已有 Core 安装状态')
    p.add_argument('--robot', choices=('dm_arm', 'none'))
    for name in COMPONENTS:
        group = p.add_mutually_exclusive_group()
        group.add_argument('--with-' + name, dest=name, action='store_true')
        group.add_argument('--without-' + name, dest=name, action='store_false')
        p.set_defaults(**{name: None})
    p.add_argument('--ros-distro', default=os.environ.get('ROS_DISTRO', 'humble'))
    p.add_argument('--jobs', type=int, default=min(os.cpu_count() or 1, 8))
    p.add_argument('--yes', action='store_true', help='跳过安装计划确认；不允许慢速源码回退')
    p.add_argument('--dry-run', action='store_true', help='只输出 JSON 计划，不写文件、不安装')
    p.add_argument('--skip-system-deps', action='store_true', help='不调用 apt/rosdep，依赖由用户提供')
    p.add_argument('--allow-source-build', action='store_true', help='允许 Conan 缺失二进制时编译源码')
    return p


def parse_plan(argv, root):
    p = parser()
    args = p.parse_args(argv)
    if not args.preset:
        p.error('非交互输入需要 --preset；交互向导请执行 ./install.sh')
    preset = 'standalone' if args.preset == 'standalone-dm' else args.preset
    defaults = PRESETS[preset]
    values = {name: defaults[i] if getattr(args, name) is None else getattr(args, name)
              for i, name in enumerate(COMPONENTS)}
    robot = args.robot or defaults[-1]
    if args.gui_only:
        values = {name: False for name in COMPONENTS}
        values['gui'] = True
        robot = 'none'
    if values['moveit']:
        if args.ros2 is False:
            p.error('MoveIt 依赖 ROS2，不能同时使用 --without-ros2')
        values['ros2'] = True
        # This repository only ships DM-Arm MoveIt configurations
        robot = 'dm_arm'
    if values['ros2']:
        if args.python is False:
            p.error('ROS launch 的 Profile 解析依赖 Python Binding，不能同时使用 --without-python')
        values['python'] = True
    if args.jobs < 1:
        p.error('--jobs 必须大于零')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', args.ros_distro):
        p.error('--ros-distro 格式无效')
    return Plan(preset=preset, **values, robot=robot, ros_distro=args.ros_distro,
                jobs=args.jobs, yes=args.yes, dry_run=args.dry_run,
                skip_system_deps=args.skip_system_deps,
                allow_source_build=args.allow_source_build, root=str(root), gui_only=args.gui_only)
