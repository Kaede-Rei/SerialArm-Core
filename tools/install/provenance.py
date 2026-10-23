"""Content receipts shared by the installer and Linux packaging"""
import hashlib
import json
from pathlib import Path


NATIVE_INPUTS = ('src', 'install.sh', 'conanfile.py', 'tools/bootstrap_standalone.sh',
                 'tools/install/main.py', 'tools/install/plan.py',
                 'tools/install/runner.py', 'tools/install/provenance.py')
GUI_INPUTS = ('apps/launcher', 'tools/install/gui.py')
GUI_ASSETS = ('apps/launcher/node_modules/electron/dist',
              'apps/launcher/node_modules/@xterm/xterm',
              'apps/launcher/node_modules/@xterm/addon-fit',
              'apps/launcher/node_modules/three', 'apps/launcher/renderer/vendor/three')
BUILD_OPTIONS = ('terminal', 'python', 'ros2', 'moveit', 'robot', 'ros_distro', 'packages')


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
