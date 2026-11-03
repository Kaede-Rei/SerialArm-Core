#!/usr/bin/env python3
"""Prepare a standalone native runtime with optional ROS2 adapters for Linux releases"""

import argparse
import hashlib
import platform
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


def read_manifest(root, with_ros2=False):
    path = root / '.install/manifest.json'
    if not path.is_file():
        fail('Missing verified installation; run ./install.sh --preset standalone --with-python --yes')
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('status') != 'verified':
        fail('Installation did not pass its verification stage')
    if not (data.get('python') and data.get('terminal') and data.get('gui')):
        fail('Release needs Core, Terminal, Python Binding and GUI')
    if bool(data.get('ros2')) != bool(with_ros2):
        fail('Installation adapter selection differs from the release target; use --with-ros2 for the optional ROS release')
    if data.get('robot') != 'dm_arm':
        fail('The built-in Linux release needs DM-Arm resources')
    if with_ros2 and data.get('ros_distro') != 'humble':
        fail('The optional ROS adapter release targets Humble')
    if data.get('gui_status') != 'ready':
        fail('GUI installation has not been verified')
    required = {'serial_arm_core', 'serial_arm_robot_profiles', 'serial_arm_protocol_damiao_usb2can',
                'serial_arm_hardware_damiao', 'dm_arm_description'}
    if with_ros2:
        required.add('serial_arm_ros2_control')
    if data.get('moveit'):
        required.update(('dm_arm_no_gripper', 'dm_arm_with_gripper'))
    if not required.issubset(set(data.get('packages') or [])):
        fail('Manifest does not include the selected Core, robot resources or adapters')
    components = {'core', 'terminal', 'python'} | ({'ros2'} if with_ros2 else set())
    if not components.issubset(set(data.get('installed_components') or [])):
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
    if with_ros2 and not (prefix / 'local_setup.bash').is_file():
        fail('Optional ROS2 adapter setup script is missing')
    if not any(prefix.glob('lib*/libserial_arm_core.so*')):
        fail('Compiled Core shared library is missing')
    for package in sorted(required):
        p = prefix / 'share/ament_index/resource_index/packages' / package
        if with_ros2 and not p.is_file():
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


def check_elf_dependencies(prefix, library_paths=()):
    if not shutil.which('ldd'):
        fail('ldd is needed to validate installed native runtime libraries')
    env = clean_runtime_environment()
    env['LD_LIBRARY_PATH'] = os.pathsep.join(
        [str(prefix / 'lib'), str(prefix / 'lib64'), *map(str, library_paths), env.get('LD_LIBRARY_PATH', '')])
    for name in REQUIRED_BINARIES:
        result = subprocess.run(['ldd', str(prefix / 'bin' / name)],
                                env=env, text=True, capture_output=True)
        if '/opt/ros/' in result.stdout:
            fail(f'Core executable unexpectedly depends on ROS: {name}')
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
        fail('Electron entry does not support relocated native runtimes')
    for asset in ('renderer/vendor/three/build/three.module.js',
                  'renderer/index.html', 'renderer/app.js',
                  'node_modules/@xterm/xterm/lib/xterm.js',
                  'node_modules/@xterm/addon-fit/lib/addon-fit.js'):
        if not (project / asset).is_file():
            fail('Missing staged GUI asset: ' + asset)


def bundle_external_libs(native, library_paths=()):
    """Bundle external native dependencies; optional ROS adapters use their host distro"""
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
            [str(native / 'lib'), str(native / 'lib64'), *map(str, library_paths), env.get('LD_LIBRARY_PATH', '')])
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
            # Desktop/glibc runtimes come from the target OS. Domain libraries
            # such as yaml-cpp, Boost, URDF and collision parsers must be bundled
            # even if apt installed them under /usr/lib on the build machine
            platform_libraries = ('libc.so.', 'libm.so.', 'libdl.so.', 'libpthread.so.',
                'librt.so.', 'libresolv.so.', 'libutil.so.', 'libstdc++.so.', 'libgcc_s.so.',
                'libgomp.so.', 'libgfortran.so.', 'libquadmath.so.', 'ld-linux')
            if (resolved.is_relative_to(native.resolve()) or soname.startswith(platform_libraries) or
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
    query = '''import json, sys
from pathlib import Path
from importlib import metadata
try:
    from packaging.requirements import Requirement
except ImportError:
    from pip._vendor.packaging.requirements import Requirement
# Ubuntu's Jinja2 egg metadata omits its runtime MarkupSafe dependency.
pending = [('numpy', set()), ('scipy', set()), ('PyYAML', set()),
           ('cvxpy', set()), ('clarabel', set()), ('MarkupSafe', set()), ('xacro', set())]
seen, paths = set(), {}
while pending:
    name, extras = pending.pop()
    distribution = metadata.distribution(name)
    key = (distribution.metadata['Name'].lower(), tuple(sorted(extras)))
    if key in seen: continue
    seen.add(key)
    info = Path(distribution._path).resolve()
    paths[info.name] = str(info)
    tops = {str(f).split('/')[0] for f in distribution.files or []
            if str(f).split('/')[0] not in ('..', '.')}
    tops.update((distribution.read_text('top_level.txt') or '').split())
    for top in tops:
        source = Path(distribution.locate_file(top)).resolve()
        candidates = [source] if source.exists() else list(source.parent.glob(source.name + '.*'))
        for candidate in candidates:
            if candidate.is_dir() or candidate.suffix in ('.py', '.so'):
                paths[candidate.name] = str(candidate)
    for raw in distribution.requires or []:
        requirement = Requirement(raw)
        if requirement.marker and not any(requirement.marker.evaluate({'extra': extra})
                                          for extra in extras | {''}): continue
        pending.append((requirement.name, set(requirement.extras)))
print(json.dumps({'minor': sys.version_info.minor, 'paths': sorted(paths.items())}))'''
    probe = subprocess.run([str(venv), '-I', '-c', query], capture_output=True, text=True)
    if probe.returncode != 0:
        fail('GUI Python dependencies are unavailable: ' + probe.stderr[-1300:])
    data = json.loads(probe.stdout)
    system_minor = int(subprocess.check_output(
        ['/usr/bin/python3', '-c', 'import sys; print(sys.version_info.minor)'], text=True))
    if data['minor'] != system_minor:
        fail('GUI Python ABI does not match the target system Python')
    target = payload / 'python'
    target.mkdir(parents=True, exist_ok=True)
    for name, folder in data['paths']:
        source = Path(folder)
        if source.is_dir():
            copytree(source, target / name)
        else:
            shutil.copy2(source, target / name)
    bundle_python_binding(root, payload)
    validate_python_payload(payload)
    print('[SerialArm] GUI Python dependencies, native binding and CLARABEL solve verified')


def bundle_python_binding(root, payload):
    manifest = json.loads((root / '.install/manifest.json').read_text())
    prefix = Path(manifest['install_prefix'])
    candidates = [p for folder in ('local/lib', 'lib', 'lib64', 'python-venv/lib')
                  for p in (prefix / folder).glob('python*/*-packages/serial_arm')
                  if any(p.glob('_serial_arm*.so'))]
    if len(candidates) != 1:
        fail('Expected one installed SerialArm Python binding, found: ' + str(candidates))
    copytree(candidates[0], payload / 'python/serial_arm')


def clean_runtime_environment():
    env = dict(os.environ)
    ros_prefixes = [p for name in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH')
                    for p in env.get(name, '').split(os.pathsep) if p]
    for name in ('PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'CMAKE_PREFIX_PATH'):
        env[name] = os.pathsep.join(p for p in env.get(name, '').split(os.pathsep)
            if p and not p.startswith('/opt/ros/') and not any(p == base or p.startswith(base + '/') for base in ros_prefixes))
    for name in list(env):
        if name.startswith('ROS_') or name in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONHOME', 'PYTHONPATH'):
            env.pop(name, None)
    return env


def validate_python_payload(payload):
    """Exclude build-host site packages and exercise the solver shipped to users."""
    check = r'''
import sys
from pathlib import Path
root = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root / 'python'))
import yaml, numpy as np, scipy, cvxpy as cp, clarabel, osqp, scs, serial_arm, xacro
for module in (yaml, np, scipy, cp, clarabel, osqp, scs, serial_arm, xacro):
    if not Path(module.__file__).resolve().is_relative_to(root / 'python'):
        raise ValueError('Dependency escaped payload: ' + module.__name__)
if int(np.__version__.split('.')[0]) >= 2:
    raise ValueError('Packaged NumPy violates the native binding ABI')
if 'CLARABEL' not in cp.installed_solvers():
    raise ValueError('Packaged CLARABEL solver is unavailable')
# A small joint static/dynamic regression with the same PSD constraint family
# as physical identification.
x = cp.Variable(2)
static = np.array([[1., 0.], [2., 0.]])
dynamic = np.array([[1., 1.], [1., -1.], [.5, 2.]])
truth = np.array([2., .3])
cone = cp.diag(cp.hstack([x[1], x[1], x[1], x[0]]))
problem = cp.Problem(cp.Minimize(cp.sum_squares(static @ x - static @ truth)
                               + cp.sum_squares(dynamic @ x - dynamic @ truth)),
                     [cone >> 1e-6 * np.eye(4)])
problem.solve(solver='CLARABEL', tol_gap_abs=1e-8, tol_feas=1e-8, tol_gap_rel=1e-8)
if problem.status != 'optimal' or x.value is None or not np.allclose(x.value, truth, atol=1e-5):
    raise ValueError('Packaged joint physical solve failed: ' + str(problem.status))
print('PYTHON_PAYLOAD_SOLVE_PASS')
'''
    env = clean_runtime_environment()
    env.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = os.pathsep.join(str(payload / 'native' / folder) for folder in ('lib', 'lib64'))
    result = subprocess.run(['/usr/bin/python3', '-I', '-S', '-c', check, str(payload)],
                            env=env, text=True, capture_output=True, timeout=60)
    if result.returncode != 0:
        raise ValueError('Isolated packaged Python solver check failed: ' + result.stderr[-2200:])


def release_evidence(root, payload, manifest):
    def command(argv):
        result = subprocess.run(argv, cwd=root, capture_output=True, text=True)
        if result.returncode: fail('Unable to record release provenance: ' + ' '.join(argv))
        return result.stdout.strip()
    source = {'commit': command(['git', 'rev-parse', 'HEAD']),
              'dirty': bool(command(['git', 'status', '--porcelain'])),
              'native': native_inputs(root, manifest)['source_sha256'], 'gui': gui_inputs(root)}
    if source['native'] != manifest['native_receipt']['source_sha256'] or source['gui'] != manifest['gui_source_sha256']:
        fail('Release inputs changed after installation')
    licenses = payload / 'licenses'
    licenses.mkdir(exist_ok=True)
    for name, source_file in (
        ('pybind11', root / 'src/serial_arm/core/third_lib/pybind11/LICENSE'),
        ('tl', root / 'src/serial_arm/core/third_lib/tl/COPYING'),
    ):
        shutil.copy2(source_file, licenses / (name + '.txt'))
    for source_file in sorted((root / 'packaging/linux/licenses').glob('*.txt')):
        shutil.copy2(source_file, licenses / source_file.name)
    for name in ('pinocchio', 'coal', 'eigenpy', 'boost'):
        found = []
        for base in (Path('/opt/openrobots/share'), Path('/usr/share/doc')):
            for directory in sorted(set(base.glob('*' + name + '*')) | set((base / 'doc').glob('*' + name + '*'))):
                if not directory.is_dir(): continue
                found.extend(p for p in directory.rglob('*') if p.is_file() and
                    any(word in p.name.lower() for word in ('license', 'copying', 'copyright')))
        if not found and name in ('pinocchio', 'coal'):
            fail('Missing dependency license material: ' + name)
        for index, source_file in enumerate(found):
            shutil.copy2(source_file, licenses / (name + '-' + str(index) + '-' + source_file.name))
    files = {}
    for path in sorted(payload.rglob('*')):
        if path.is_file(): files[str(path.relative_to(payload))] = hashlib.sha256(path.read_bytes()).hexdigest()
    evidence = {'source': source, 'target': platform.machine(),
                'compiler': command(['c++', '-dumpmachine']), 'files': files}
    (payload / 'build-receipt.json').write_text(json.dumps(evidence, indent=2) + '\n')


def wrap_native_entrypoints(native):
    real = native / 'libexec' / 'serial-arm'
    real.mkdir(parents=True, exist_ok=True)
    for name in REQUIRED_BINARIES:
        executable = native / 'bin' / name
        shutil.move(executable, real / name)
        executable.write_text('#!/usr/bin/env bash\nset -euo pipefail\n'
            'prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"\n'
            'export LD_LIBRARY_PATH="$prefix/lib:$prefix/lib64:${LD_LIBRARY_PATH:-}"\n'
            'exec "$prefix/libexec/serial-arm/' + name + '" "$@"\n')
        executable.chmod(0o755)


def make_manifest(root, stage, payload):
    version = (ET.parse(root / 'src/serial_arm/core/package.xml')
               .getroot().findtext('version') or '').strip()
    if not re.fullmatch(r'\d+\.\d+\.\d+', version):
        fail('Invalid Core version in package.xml')
    gui_package = json.loads((root / 'apps/launcher/package.json').read_text(encoding='utf-8'))
    gui_package.update({
        'name': 'serial-arm-core',
        'version': version,
        'description': 'SerialArm-Core native control and desktop workbench with optional ROS2 adapters',
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
            'appImage': {'executableArgs': []},
            'deb': {
                'depends': [
                    'libgtk-3-0', 'libnss3', 'libxss1', 'libasound2',
                    'libgbm1', 'libstdc++6', 'libgomp1', 'libgfortran5',
                    'python3',
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
    parser.add_argument('--with-ros2', action='store_true', help='Include explicitly selected ROS2 adapters')
    args = parser.parse_args()
    root = args.root.resolve()
    prefix = read_manifest(root, args.with_ros2)
    if args.check_only:
        print(prefix)
        return
    if args.stage is None:
        parser.error('--stage is required unless --check-only is set')
    receipt_text = (root / '.install/manifest.json').read_text(encoding='utf-8')
    manifest = json.loads(receipt_text)
    check_elf_dependencies(prefix, manifest.get('native_library_paths', []))
    stage = args.stage.resolve()
    project, payload = stage / 'project', stage / 'payload'
    project.mkdir(parents=True, exist_ok=True)
    payload.mkdir(parents=True, exist_ok=True)

    print('[SerialArm] Copying native Core, Python Binding, DM-Arm and selected adapters')
    copytree(prefix, payload / 'native')
    if installed_content(payload / 'native') != manifest['native_receipt']['installed_sha256']:
        fail('Installed files changed while copying the release payload')
    shutil.rmtree(payload / 'native/python-venv', ignore_errors=True)
    bundle_external_libs(payload / 'native', manifest.get('native_library_paths', []))
    wrap_native_entrypoints(payload / 'native')
    print('[SerialArm] Staging Electron GUI and Python bridge')
    prepare_gui(root, project)
    copytree(root / 'apps/launcher/backend', payload / 'apps/launcher/backend')
    fallback = root / 'src/robot_supports/profiles/config'
    if fallback.is_dir():
        copytree(fallback, payload / 'src/robot_supports/profiles/config')
    shutil.copy2(root / 'LICENSE', payload / 'LICENSE')
    python_gui_modules(root, payload)
    if read_manifest(root, args.with_ros2) != prefix:
        fail('Active installation changed while preparing the release payload')
    if (root / '.install/manifest.json').read_text(encoding='utf-8') != receipt_text:
        fail('Installation receipt changed while preparing the release payload')
    (payload / 'runtime.json').write_text(json.dumps({
        'installed_components': manifest['installed_components'],
        'ros_distro': manifest.get('ros_distro') if manifest.get('ros2') else '',
    }, indent=2) + '\n')
    release_evidence(root, payload, manifest)
    make_manifest(root, stage, payload)
    print('[SerialArm] Selected release runtime staged successfully')


if __name__ == '__main__':
    main()
