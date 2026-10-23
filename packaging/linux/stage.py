#!/usr/bin/env python3
"""Prepare a relocatable installed ROS2 overlay for an Electron Linux release"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/install'))
from provenance import gui_assets, gui_inputs, installed_content, native_inputs

REQUIRED_PACKAGES = (
    'serial_arm_core',
    'serial_arm_robot_profiles',
    'serial_arm_ros2_control',
    'serial_arm_protocol_damiao_usb2can',
    'serial_arm_hardware_damiao',
    'dm_arm_description',
    'dm_arm_no_gripper',
    'dm_arm_with_gripper',
)
REQUIRED_BINARIES = (
    'serial_arm_terminal',
    'serial_arm_model_probe',
    'serial_arm_model_calibrator',
)


def fail(message):
    raise SystemExit('[SerialArm] Error: ' + str(message))


def copytree(source: Path, target: Path):
    if not source.is_dir():
        fail(f'Missing directory: {source}')
    # Colcon --symlink-install creates links back to the checkout
    # Dereference those links so a release never depends on the build host
    shutil.copytree(source, target, symlinks=False,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.pyo',
                                                  '.DS_Store', '.git', '.serialarm'))


def read_manifest(root):
    path = root / '.install/manifest.json'
    if not path.is_file():
        fail('Missing verified installation; run ./install.sh --preset ros2 --yes')
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('status') != 'verified':
        fail('Installation did not pass its verification stage')
    if not (data.get('ros2') and data.get('moveit') and data.get('python')
            and data.get('terminal') and data.get('gui')):
        fail('The active installation is not full ROS2 + MoveIt + GUI (install.sh choice 4)')
    if data.get('robot') != 'dm_arm' or data.get('ros_distro') != 'humble':
        fail('Only the DM-Arm / ROS2 Humble release target is supported')
    if data.get('gui_status') != 'ready':
        fail('GUI installation has not been verified')
    if not set(REQUIRED_PACKAGES).issubset(set(data.get('packages') or [])):
        fail('Manifest does not include all required ROS2, DM-Arm and MoveIt packages')
    if not set(('core', 'terminal', 'python', 'ros2', 'moveit')).issubset(
            set(data.get('installed_components') or [])):
        fail('Installer has not verified the required compiled components')
    prefix_raw = data.get('install_prefix')
    if not isinstance(prefix_raw, str) or not prefix_raw:
        fail('Installation prefix is not recorded')
    prefix = Path(prefix_raw).resolve()
    if not prefix.is_relative_to((root / 'install').resolve()):
        fail('Installation prefix does not belong to this repository')
    receipt = data.get('native_receipt')
    if not isinstance(receipt, dict):
        fail('Installation has no source receipt; rebuild the complete installation')
    current = native_inputs(root, data)
    if any(receipt.get(key) != value for key, value in current.items()):
        fail('Native sources or build options changed; rebuild the complete installation')
    if receipt.get('installed_sha256') != installed_content(prefix):
        fail('Installed files changed after verification; rebuild the complete installation')
    if data.get('gui_source_sha256') != gui_inputs(root):
        fail('Launcher sources changed; reinstall the GUI before packaging')
    if data.get('gui_assets_sha256') != gui_assets(root):
        fail('Launcher dependencies changed; reinstall the GUI before packaging')
    for binary in REQUIRED_BINARIES:
        exe = prefix / 'bin' / binary
        if not exe.is_file() or not os.access(exe, os.X_OK):
            fail(f'Missing compiled executable: {exe}')
    if not (prefix / 'local_setup.bash').is_file():
        fail('ROS2 local setup script not installed')
    if not any(prefix.glob('lib*/libserial_arm_core.so*')):
        fail('Compiled Core shared library is missing')
    for package in REQUIRED_PACKAGES:
        p = prefix / 'share/ament_index/resource_index/packages' / package
        if not p.is_file():
            fail(f'ROS2 package index not installed: {package}')
        share = prefix / 'share' / package
        if not share.is_dir():
            fail(f'ROS2 package resources not installed: {package}')
    if not (prefix / 'share/serial_arm_robot_profiles/config/robot_profiles.yaml').is_file():
        fail('Built-in robot_profiles.yaml was not installed')
    if not (root / '.install/gui-venv/bin/python').is_file():
        fail('GUI Python environment is not installed')
    for required in ('apps/launcher/node_modules/electron/dist/electron',
                     'apps/launcher/renderer/vendor/three/build/three.module.js'):
        if not (root / required).is_file():
            fail('Missing GUI dependency: ' + required)
    return prefix


def check_elf_dependencies(prefix):
    if not shutil.which('ldd'):
        fail('ldd is needed to validate installed native runtime libraries')
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = os.pathsep.join(
        [str(prefix / 'lib'), str(prefix / 'lib64'), env.get('LD_LIBRARY_PATH', '')])
    for name in REQUIRED_BINARIES:
        result = subprocess.run(['ldd', str(prefix / 'bin' / name)],
                                env=env, text=True, capture_output=True)
        if result.returncode or 'not found' in result.stdout:
            fail(f'Native dependency missing for {name}: ' +
                 (result.stdout + result.stderr)[-1700:])


def prepare_gui(root, project):
    app = root / 'apps/launcher'
    copytree(app / 'desktop', project / 'desktop')
    copytree(app / 'renderer', project / 'renderer')
    for name in ('electron', 'three', '@xterm/xterm', '@xterm/addon-fit'):
        copytree(app / 'node_modules' / name, project / 'node_modules' / name)
    # Make sure the main process and renderer are from the current source
    main = (project / 'desktop/main.cjs').read_text(encoding='utf-8')
    if 'function packagedBackend()' not in main or 'SERIAL_ARM_BUNDLE_PREFIX' not in main:
        fail('Electron entry does not support relocated ROS2 release runtimes')
    for asset in ('renderer/vendor/three/build/three.module.js',
                  'renderer/index.html', 'renderer/app.js',
                  'node_modules/@xterm/xterm/lib/xterm.js',
                  'node_modules/@xterm/addon-fit/lib/addon-fit.js'):
        if not (project / asset).is_file():
            fail('Missing staged GUI asset: ' + asset)


def bundle_external_libs(native):
    """Include non-system ELF deps while leaving the ROS2 distro in /opt/ros/humble"""
    libdest = native / 'lib'
    libdest.mkdir(parents=True, exist_ok=True)
    dirs = [native / 'bin', native / 'lib', native / 'lib64']
    queue = [f for directory in dirs if directory.is_dir()
             for f in directory.rglob('*') if f.is_file()
             and (directory.name == 'bin' or '.so' in f.name)]
    done = set()
    while queue:
        candidate = queue.pop()
        if candidate.resolve() in done:
            continue
        done.add(candidate.resolve())
        if not candidate.open('rb').read(4) == b'\x7fELF':
            continue
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = os.pathsep.join(
            [str(native / 'lib'), str(native / 'lib64'), env.get('LD_LIBRARY_PATH', '')])
        proc = subprocess.run(['ldd', str(candidate)], env=env, text=True, capture_output=True)
        if proc.returncode != 0:
            fail(f'Cannot inspect shared libraries: {candidate}: {proc.stderr[-700:]}')
        for line in proc.stdout.splitlines():
            if 'not found' in line:
                fail(f'Unresolved native dependency in {candidate}: {line.strip()}')
            match = re.match(r'^\s*([^\s]+)\s*=>\s*(/[^\s]+)', line)
            if not match:
                continue
            soname, filename = match.groups()
            resolved = Path(filename).resolve()
            if (resolved.is_relative_to(native.resolve()) or
                resolved.is_relative_to(Path('/lib').resolve()) or
                resolved.is_relative_to(Path('/usr/lib').resolve()) or
                resolved.is_relative_to(Path('/opt/ros/humble').resolve())):
                continue
            if not resolved.is_file() or '/' in soname:
                fail('Invalid ELF dependency: ' + line.strip())
            out = libdest / soname
            if not out.exists():
                shutil.copy2(resolved, out)
                queue.append(out)
                print('[SerialArm] External dependency bundled:', soname)


def python_gui_modules(root, payload):
    venv = root / '.install/gui-venv/bin/python'
    query = '''import json, sys, numpy, scipy, yaml
from pathlib import Path
modules = (numpy, scipy, yaml)
paths = []
for module in modules:
    source = Path(module.__file__).resolve().parent
    paths.append([module.__name__, str(source)])
    for suffix in ('libs', '.libs'):
        sibling = source.parent / (source.name + suffix)
        if sibling.is_dir(): paths.append([sibling.name, str(sibling)])
print(json.dumps({'minor': sys.version_info.minor, 'paths': paths}))'''
    probe = subprocess.run([str(venv), '-c', query], capture_output=True, text=True)
    if probe.returncode != 0:
        fail('GUI Python dependencies are unavailable: ' + probe.stderr[-1300:])
    data = json.loads(probe.stdout)
    system_minor = int(subprocess.check_output(
        ['/usr/bin/python3', '-c', 'import sys; print(sys.version_info.minor)'], text=True))
    if data['minor'] != system_minor:
        fail('GUI Python ABI does not match the system ROS2 Python')
    target = payload / 'python'
    for name, folder in data['paths']:
        copytree(Path(folder), target / name)
    env = dict(os.environ)
    env['PYTHONPATH'] = str(target) + os.pathsep + env.get('PYTHONPATH', '')
    env['PYTHONNOUSERSITE'] = '1'
    verify = subprocess.run(['/usr/bin/python3', '-c',
        'import yaml, numpy, scipy, sys; from pathlib import Path; '
        'root=Path(sys.argv[1]).resolve(); '
        'assert all(Path(mod.__file__).resolve().is_relative_to(root) '
        'for mod in (yaml,numpy,scipy))', str(target)],
        env=env, text=True, capture_output=True)
    if verify.returncode != 0:
        fail('Packaged Python dependencies failed to import: ' + verify.stderr[-1400:])
    print('[SerialArm] GUI Python modules copied: numpy, scipy, PyYAML')


def make_manifest(root, stage, payload):
    version = (ET.parse(root / 'src/serial_arm/core/package.xml')
               .getroot().findtext('version') or '').strip()
    if not re.fullmatch(r'\d+\.\d+\.\d+', version):
        fail('Invalid Core version in package.xml')
    gui_package = json.loads((root / 'apps/launcher/package.json').read_text(encoding='utf-8'))
    gui_package.update({
        'name': 'serial-arm-core',
        'version': version,
        'description': 'SerialArm-Core complete ROS2 control, MoveIt and desktop application',
        'productName': 'SerialArm Core',
        'main': 'desktop/main.cjs',
        'license': 'GPL-3.0-only',
        'author': {'name': 'AgroTech Innovation Lab', 'email': 'kaerei@foxmail.com'},
        # Linux deb packages require the application homepage in the staged app
        # metadata, not in packaging/linux/package.json (the build-tool project)
        'homepage': 'https://github.com/Kaede-Rei/SerialArm-Core',
        'repository': {'type': 'git', 'url': 'https://github.com/Kaede-Rei/SerialArm-Core.git'},
        # Match Electron's WM_CLASS with the installed desktop entry
        'desktopName': 'lab.agrotech.serialarm',
        'build': {
            'appId': 'lab.agrotech.serialarm',
            'productName': 'SerialArm Core',
            'asar': True,
            'electronVersion': gui_package['devDependencies']['electron'],
            'directories': {'output': str(stage / 'output')},
            'files': ['desktop/**/*', 'renderer/**/*', 'package.json',
                      'node_modules/@xterm/xterm/**/*',
                      'node_modules/@xterm/addon-fit/**/*',
                      'node_modules/three/**/*'],
            'extraResources': [{'from': str(payload), 'to': 'serial-arm'}],
            'linux': {
                'target': ['AppImage', 'deb'],
                'category': 'Science',
                'syncDesktopName': True,
                'maintainer': 'AgroTech Innovation Lab <kaerei@foxmail.com>',
                'vendor': 'AgroTech Innovation Lab',
                'icon': str(root / 'packaging/linux/icon.png'),
                'artifactName': 'SerialArm-Core-linux-${arch}.${ext}',
            },
            'deb': {
                'depends': [
                    'libgtk-3-0', 'libnss3', 'libxss1', 'libasound2',
                    'libgbm1', 'libstdc++6', 'libgomp1', 'libgfortran5',
                    'python3', 'ros-humble-ros2-control',
                    'ros-humble-ros2-controllers', 'ros-humble-moveit',
                    'ros-humble-robot-state-publisher',
                    'ros-humble-xacro',
                ],
            },
        },
    })
    # Fail while staging if future edits accidentally omit mandatory deb metadata
    if not gui_package.get('homepage', '').startswith('https://'):
        fail('Linux deb package requires a valid project homepage')
    author = gui_package.get('author') or {}
    if not isinstance(author, dict) or '@' not in author.get('email', ''):
        fail('Linux deb package requires an author email')
    if gui_package.get('desktopName') != gui_package['build']['appId']:
        fail('Linux desktopName must match the application ID')
    if gui_package['build']['linux'].get('syncDesktopName') is not True:
        fail('Linux desktop entry and Electron WM_CLASS must be synchronized')
    (stage / 'project/package.json').write_text(
        json.dumps(gui_package, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--stage', type=Path)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    prefix = read_manifest(root)
    if args.check_only:
        print(prefix)
        return
    if args.stage is None:
        parser.error('--stage is required unless --check-only is set')
    receipt_text = (root / '.install/manifest.json').read_text(encoding='utf-8')
    manifest = json.loads(receipt_text)
    check_elf_dependencies(prefix)
    stage = args.stage.resolve()
    project, payload = stage / 'project', stage / 'payload'
    project.mkdir(parents=True, exist_ok=True)
    payload.mkdir(parents=True, exist_ok=True)

    print('[SerialArm] Copying full ROS2 overlay, Python Binding, MoveIt and DM-Arm')
    copytree(prefix, payload / 'native')
    if installed_content(payload / 'native') != manifest['native_receipt']['installed_sha256']:
        fail('Installed files changed while copying the release payload')
    bundle_external_libs(payload / 'native')
    print('[SerialArm] Staging Electron GUI and Python bridge')
    prepare_gui(root, project)
    copytree(root / 'apps/launcher/backend', payload / 'apps/launcher/backend')
    fallback = root / 'src/robot_supports/profiles/config'
    if fallback.is_dir():
        copytree(fallback, payload / 'src/robot_supports/profiles/config')
    shutil.copy2(root / 'LICENSE', payload / 'LICENSE')
    python_gui_modules(root, payload)
    if read_manifest(root) != prefix:
        fail('Active installation changed while preparing the release payload')
    if (root / '.install/manifest.json').read_text(encoding='utf-8') != receipt_text:
        fail('Installation receipt changed while preparing the release payload')
    make_manifest(root, stage, payload)
    print('[SerialArm] Full release staged successfully')


if __name__ == '__main__':
    main()
