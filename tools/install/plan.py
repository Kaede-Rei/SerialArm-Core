"""Pure installation planning shared by CLI and interactive wizard"""
import argparse
import json
import os
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

PRESETS = {
    'core': (False, False, False, False, False, 'none'),
    'standalone': (True, False, False, False, True, 'dm_arm'),
    'python': (False, True, False, False, False, 'none'),
    'ros2': (True, False, True, True, True, 'dm_arm'),
    'dev': (True, True, True, True, True, 'dm_arm'),
    'custom': (False, False, False, False, False, 'none'),
}
COMPONENTS = ('terminal', 'python', 'ros2', 'moveit', 'gui')

@dataclass
class Plan:
    preset: str
    terminal: bool
    python: bool
    ros2: bool
    moveit: bool
    gui: bool
    robot: str
    ros_distro: str
    jobs: int
    yes: bool
    dry_run: bool
    skip_system_deps: bool
    allow_source_build: bool
    root: str
    gui_only: bool = False
    clean_rebuild: bool = False

    @property
    def install_key(self):
        mode = 'ros2-' + self.ros_distro if self.ros2 else 'standalone'
        key = f'{mode}-{self.robot}-t{int(self.terminal)}-p{int(self.python)}-m{int(self.moveit)}'
        # Existing installations encoded the removed development option in their
        # directory name. Reuse that generation when its native selection matches
        root = Path(self.root).resolve()
        try:
            manifest = json.loads((root / '.install/manifest.json').read_text())
            fields = ('terminal', 'python', 'ros2', 'moveit', 'robot', 'ros_distro')
            if (Path(manifest.get('root', '')).resolve() == root
                    and (self.gui_only or all(manifest.get(name) == getattr(self, name) for name in fields))):
                prefix = Path(manifest.get('install_prefix', ''))
                build = Path(manifest.get('build_dir', ''))
                if (prefix.parent == root / 'install/unified' and build.parent == root / 'build/unified'
                        and prefix.name == build.name
                        and (self.gui_only or prefix.name in (key, key + '-test0', key + '-test1'))):
                    return prefix.name
        except (OSError, ValueError, TypeError, AttributeError):
            pass
        return key

    @property
    def packages(self):
        if self.gui_only: return []
        names = ['serial_arm_core']
        if self.ros2:
            names += ['serial_arm_robot_profiles', 'serial_arm_ros2_control']
        if self.robot == 'dm_arm':
            names += ['serial_arm_protocol_damiao_usb2can', 'serial_arm_hardware_damiao',
                      'serial_arm_robot_profiles', 'dm_arm_description']
        if self.moveit:
            names += ['dm_arm_no_gripper', 'dm_arm_with_gripper']
        return list(dict.fromkeys(names))

    def package_paths(self):
        """Validate selected ROS manifests before apt/rosdep changes the machine"""
        root = Path(self.root) / 'src'
        if not root.is_dir():
            raise ValueError(f'缺少 ROS 源码目录：{root}')
        found = {}
        for path in sorted(root.rglob('package.xml')):
            try:
                xml = ET.parse(path).getroot()
            except ET.ParseError as exc:
                raise ValueError(f'package.xml XML 格式错误：{path}\n{exc}') from exc
            if xml.tag != 'package':
                raise ValueError(f'package.xml 根标签必须是 <package>：{path}')
            name = (xml.findtext('name') or '').strip()
            if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
                raise ValueError(f'package.xml 中的 ROS 包名无效：{path}\n当前值：{name!r}')
            if name in found:
                raise ValueError(f'发现重复 ROS 包名 {name}：\n{found[name]}\n{path.parent}')
            found[name] = path.parent
            if name in self.packages:
                version = (xml.findtext('version') or '').strip()
                if not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+', version):
                    suggested = version.removeprefix('v')
                    if not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+', suggested):
                        suggested = '0.6.0'
                    raise ValueError(
                        f'ROS package.xml 版本格式错误：{path}\n'
                        f'当前值：{version!r}，应为纯数字 X.Y.Z，例如 {suggested}\n'
                        'Git Tag 可以使用 v0.6.0，但 package.xml 的 <version> 不允许带 v'
                    )
        missing = [name for name in self.packages if name not in found]
        if missing:
            raise ValueError(f'缺少选中的 ROS 包：{", ".join(missing)}\n请核对 src/ 下的 package.xml 和安装选项')
        return [str(found[name]) for name in self.packages]

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
    p.add_argument('--clean-rebuild', action='store_true', help='清理当前配置的构建和安装目录后重建，用于源码目录迁移恢复')
    p.add_argument('--robot', choices=('dm_arm', 'none'))
    for name in COMPONENTS:
        group = p.add_mutually_exclusive_group()
        group.add_argument('--with-' + name, dest=name, action='store_true')
        group.add_argument('--without-' + name, dest=name, action='store_false')
        p.set_defaults(**{name: None})
    p.add_argument('--with-tests', dest='_legacy_tests', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--without-tests', dest='_legacy_tests', action='store_false', help=argparse.SUPPRESS)
    p.set_defaults(_legacy_tests=None)
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
    if args._legacy_tests:
        print('--with-tests 已停用，不会安装或执行内置测试', file=sys.stderr)
    if args.gui_only and args.clean_rebuild:
        p.error('--clean-rebuild 需要重建 Core，不能与 --gui-only 同时使用')
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
                allow_source_build=args.allow_source_build, root=str(root), gui_only=args.gui_only,
                clean_rebuild=args.clean_rebuild)
