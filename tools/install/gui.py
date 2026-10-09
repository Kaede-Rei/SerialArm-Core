"""Provision the Electron launcher without global npm or Python packages"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
from runner import InstallError, atomic_write


def preflight_gui():
    for name in ('node', 'npm'):
        if not shutil.which(name): raise InstallError(f'GUI 需要 Node.js 20+ 和 npm；缺少 {name}；也可选择 --without-gui')
    try: major = int(subprocess.check_output(['node', '--version'], text=True).strip().lstrip('v').split('.')[0])
    except (ValueError, subprocess.CalledProcessError) as error: raise InstallError('无法检测 Node.js 版本') from error
    if major < 20: raise InstallError('GUI 需要 Node.js 20+；请安装符合要求的 Node.js，或选择 --without-gui')


def install_gui(runner):
    app = runner.root / 'apps/launcher'
    release = Path('/etc/os-release').read_text() if Path('/etc/os-release').exists() else ''
    t64 = 'ID=ubuntu' in release and any(f'VERSION_ID="{v}' in release for v in ('24', '25', '26'))
    runner.apt(['python3-venv', 'fonts-noto-cjk', 'libnss3', 'libgbm1', 'libxss1',
                'libgtk-3-0t64' if t64 else 'libgtk-3-0', 'libasound2t64' if t64 else 'libasound2'])
    python = Path('/usr/bin/python3') if Path('/usr/bin/python3').exists() else Path(sys.executable)
    venv = runner.state / 'gui-venv'
    runner.run('gui-python-venv', [python, '-m', 'venv', '--system-site-packages', venv])
    runner.run('gui-python-deps', [venv / 'bin/python', '-m', 'pip', 'install', 'PyYAML>=6,<7', 'numpy>=1.21,<2'])
    runner.run('gui-npm', ['npm', '--prefix', app, 'ci', '--ignore-scripts', '--no-audit', '--no-fund'])
    runner.run('gui-three-vendor', ['node', app / 'scripts/vendor-three.cjs'])
    electron = app / 'node_modules/electron/dist/electron'
    if not electron.is_file():
        env = dict(runner.env)
        env['ELECTRON_MIRROR'] = env.get('ELECTRON_MIRROR', 'https://npmmirror.com/mirrors/electron/')
        try: runner.run('gui-electron-mirror', ['node', app / 'node_modules/electron/install.js'], env=env)
        except InstallError:
            env.pop('ELECTRON_MIRROR', None)
            env.pop('npm_config_electron_mirror', None)
            runner.run('gui-electron-official', ['node', app / 'node_modules/electron/install.js'], env=env)
    if not os.access(electron, os.X_OK): raise InstallError('Electron 可执行文件不可用')
    runner.run('gui-static-verify', ['node', app / 'scripts/verify.cjs'])
    runner.run('gui-backend-verify', [venv / 'bin/python', '-c', 'import sys, numpy; assert int(numpy.__version__.split(".")[0]) < 2; sys.path.insert(0, sys.argv[1]); import yaml, profiles, runtime, model_runtime', app / 'backend'])
    return {'gui': True, 'gui_status': 'ready', 'gui_path': str(app), 'gui_python': str(venv / 'bin/python')}
