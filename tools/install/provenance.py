"""Content receipts shared by the installer and Linux packaging"""
import hashlib
import json
import os
from pathlib import Path
import shlex


NATIVE_INPUTS = ('src', 'install.sh', 'conanfile.py', 'tools/bootstrap_standalone.sh',
                 'tools/install/main.py', 'tools/install/plan.py',
                 'tools/install/runner.py', 'tools/install/provenance.py')
GUI_INPUTS = ('apps/launcher', 'tools/install/gui.py')
GUI_ASSETS = ('apps/launcher/node_modules/electron/dist',
              'apps/launcher/node_modules/@xterm/xterm',
              'apps/launcher/node_modules/@xterm/addon-fit',
              'apps/launcher/node_modules/three', 'apps/launcher/renderer/vendor/three')
BUILD_OPTIONS = ('terminal', 'python', 'ros2', 'moveit', 'robot', 'ros_distro', 'packages')


def recovery_command(root, manifest):
    """Rebuild the recorded component selection at the checkout's current path."""
    argv = [str(Path(root).resolve() / 'install.sh'), '--preset', manifest.get('preset', 'standalone')]
    for component in ('terminal', 'python', 'ros2', 'moveit', 'gui'):
        argv.append(('--with-' if manifest.get(component) else '--without-') + component)
    argv += ['--robot', manifest.get('robot', 'dm_arm'), '--ros-distro', manifest.get('ros_distro', 'humble'),
             '--clean-rebuild', '--yes']
    return shlex.join(argv)


def environment_problem(root):
    """Inspect installed paths without sourcing them or importing native code."""
    root = Path(root).resolve()
    path = root / '.install/manifest.json'
    if not path.is_file():
        return '未找到安装记录，请先运行 ./install.sh'
    try:
        manifest = json.loads(path.read_text())
        if not isinstance(manifest, dict):
            raise ValueError('安装记录必须为对象')
        recorded = manifest.get('root')
        problem = ''
        if recorded and Path(recorded).resolve() != root:
            problem = f'源码目录已迁移：安装位置 {recorded}，当前位置 {root}'
        else:
            for key in ('workspace_setup', 'backend_setup', 'install_prefix'):
                value = manifest.get(key)
                if value and not Path(value).exists():
                    problem = f'安装入口或组件已失效：{value}'
                    break
            prefix = manifest.get('install_prefix')
            if not problem and prefix:
                for item in Path(prefix).rglob('*'):
                    if item.is_symlink() and not item.exists():
                        problem = f'安装链接已失效：{item} -> {os.readlink(item)}'
                        break
        if problem:
            return problem + '\n请在当前目录重建所选组件：\n' + recovery_command(root, manifest)
        return ''
    except (OSError, ValueError, TypeError) as exc:
        return f'无法读取有效安装记录：{exc}\n请运行 ./install.sh 重新选择安装组件'


def fingerprint(root, paths, *, source=False):
    root = Path(root)
    digest = hashlib.sha256()

    def visit(path, ancestors):
        relative = path.relative_to(root).as_posix()
        if path.name in ('__pycache__', '.git', '.serialarm', '.DS_Store') or path.suffix in ('.pyc', '.pyo'):
            return
        if source and (path.name in ('node_modules', 'build', 'install', 'log', 'logs')
                       or relative == 'apps/launcher/renderer/vendor'
                       or '.bak-' in path.name or path.suffix == '.tmp'):
            return
        resolved = path.resolve(strict=True)
        if resolved in ancestors:
            raise OSError(f'Cyclic resource link: {path}')
        if path.is_dir():
            for child in sorted(path.iterdir()):
                visit(child, ancestors | {resolved})
            return
        if not path.is_file():
            raise OSError(f'Unsupported installation resource: {path}')
        content = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                content.update(block)
        record = (relative, path.stat().st_mode & 0o111, content.hexdigest())
        digest.update(json.dumps(record, ensure_ascii=True).encode('ascii') + b'\n')

    for name in sorted(paths):
        visit(root / name, set())
    return digest.hexdigest()


def native_inputs(root, configuration):
    return {
        'source_sha256': fingerprint(root, NATIVE_INPUTS, source=True),
        'options': {key: configuration.get(key) for key in BUILD_OPTIONS},
    }


def gui_inputs(root):
    return fingerprint(root, GUI_INPUTS, source=True)


def gui_assets(root):
    return fingerprint(root, GUI_ASSETS)


def installed_content(prefix):
    return fingerprint(prefix, ('.',))
