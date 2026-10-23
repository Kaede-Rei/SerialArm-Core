"""Installation execution. Commands are argv arrays; no user-controlled shell code"""
from datetime import datetime, timezone
import codecs
import fcntl
import json
import os
from pathlib import Path
import platform
import select
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from provenance import installed_content, native_inputs


class InstallError(RuntimeError):
    pass


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(text)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def rosdep_failure_hint(message):
    """Return actionable advice without silently bypassing unresolved dependencies"""
    lower = message.lower()
    if 'package version' in lower or 'version conventions' in lower:
        return ('先检查 package.xml 的 <version> 是否为 0.6.0 这样的 X.Y.Z 数字格式，'
                '不要写 Git Tag 格式 v0.6.0')
    if ('cannot locate rosdep definition' in lower or 'no rosdep rule' in lower
            or 'no definition of' in lower or 'no definition for' in lower):
        return ('存在无法解析的 rosdep 依赖键，检查失败日志中对应的 ROS 包与依赖名称，'
                '核对 ROS_DISTRO、rosdep 规则和 package.xml；不要盲目 --skip-keys')
    if ('rosdep database' in lower or 'cache' in lower and 'rosdep' in lower
            or 'unable to download' in lower or 'timed out' in lower
            or 'name or service not known' in lower or 'failed to fetch' in lower):
        return ('检查 ROS 源、网络和 ~/.ros/rosdep/sources.cache；网络恢复后单独运行 '
                'rosdep update --rosdistro <ROS_DISTRO>，再重试安装')
    if ('unable to locate package' in lower or 'failed to install' in lower
            or 'apt-get' in lower and 'failed' in lower):
        return ('rosdep 解析后系统包安装失败，检查失败日志里的 apt 包名与 Ubuntu/ROS2 软件源，'
                '可用 apt-cache policy <包名> 检查是否有候选版本')
    if 'permission denied' in lower or 'sudo:' in lower:
        return '检查 sudo 权限与终端交互模式，--yes 不等于 sudo 免密'
    return ('检查上面 rosdep-install.log 的原始错误；先修复 package.xml、依赖键或软件源，'
            '不要直接忽略失败依赖')


def standalone_env(env):
    env = env.copy()
    denied = set((env.get('AMENT_PREFIX_PATH', '') + ':' +
                  env.get('COLCON_PREFIX_PATH', '')).replace(';', ':').split(':'))
    env['CMAKE_PREFIX_PATH'] = ':'.join(p for p in env.get('CMAKE_PREFIX_PATH', '').replace(';', ':').split(':')
                                        if p and p not in denied and not p.startswith('/opt/ros/'))
    for name in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'ROS_DISTRO',
                 'ROS_VERSION', 'ROS_PYTHON_VERSION'):
        env.pop(name, None)
    return env


class Runner:
    def __init__(self, plan):
        self.plan = plan
        self.root = Path(plan.root)
        self.key = plan.install_key
        self.build = self.root / 'build/unified' / self.key
        self.prefix = self.root / 'install/unified' / self.key
        self.logs = self.root / 'log/unified' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        self.state = self.root / '.install'
        self.env = os.environ.copy()
        self.env['JOBS'] = str(plan.jobs)
        self.env['SERIAL_ARM_ALLOW_SOURCE_BUILD'] = '1' if plan.allow_source_build else '0'
        self.stage = 'preflight'
        self.setup = self.prefix / 'setup.bash'
        self.python = None
        self.ros_setup = Path('/opt/ros') / plan.ros_distro / 'setup.bash'

    def run(self, label, argv, *, env=None, source=None, terminal=False):
        self.stage = label
        self.logs.mkdir(parents=True, exist_ok=True)
        log = self.logs / (label + '.log')
        cmd = list(map(str, argv))
        if source:
            cmd = ['bash', '-c', 'source "$1" || exit; shift; exec "$@"', 'serial-arm-install', str(source), *cmd]
        print(f'[{label}] {shlex.join(cmd)}\n  日志：{log}', flush=True)
        # sudo reads passwords from /dev/tty, never from the log pipe
        # Keep dependency commands in the foreground terminal session and tee
        # their output so prompts remain visible even without a trailing newline
        with log.open('wb') as output:
            proc = subprocess.Popen(cmd, cwd=self.root, env=env or self.env,
                                    stdout=subprocess.PIPE if terminal else output,
                                    stderr=subprocess.STDOUT, start_new_session=not terminal)
            try:
                if terminal:
                    decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
                    report_at = time.monotonic() + 15
                    try:
                        while True:
                            ready, _, _ = select.select([proc.stdout], [], [], 1)
                            if ready:
                                chunk = os.read(proc.stdout.fileno(), 65536)
                                if not chunk:
                                    sys.stdout.write(decoder.decode(b'', final=True))
                                    sys.stdout.flush()
                                    break
                                output.write(chunk)
                                output.flush()
                                sys.stdout.write(decoder.decode(chunk))
                                sys.stdout.flush()
                            if time.monotonic() >= report_at:
                                print(f'[{label}] 正在执行（完整输出见日志）', flush=True)
                                report_at = time.monotonic() + 15
                    finally:
                        proc.stdout.close()
                while True:
                    try:
                        code = proc.wait(timeout=15)
                        break
                    except subprocess.TimeoutExpired:
                        print(f'[{label}] 正在执行…（完整输出见日志）', flush=True)
            except BaseException:
                # Foreground commands share our process group and also receive
                # terminal Ctrl+C, so never signal that group from the parent
                try:
                    if terminal:
                        if proc.poll() is None:
                            proc.terminate()
                    else:
                        os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if terminal:
                        proc.kill()
                    else:
                        os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                raise
        if code:
            tail = '\n'.join(log.read_text(errors='replace').splitlines()[-35:])
            if terminal and not sys.stdin.isatty() and 'sudo:' in tail:
                tail += '\n系统依赖需要 sudo 权限，--yes 只跳过安装确认\n请在终端运行安装器，或准备好依赖后使用 --skip-system-deps'
            raise InstallError(f'{label} 失败（退出码 {code}）\n{tail}\n完整日志：{log}')

    def apt(self, packages):
        if self.plan.skip_system_deps:
            return
        if not shutil.which('apt-get'):
            raise InstallError('自动系统依赖安装支持 apt；请自行准备依赖后使用 --skip-system-deps')
        prefix = [] if os.geteuid() == 0 else ['sudo'] + ([] if sys.stdin.isatty() else ['-n'])
        if prefix and not shutil.which('sudo'):
            raise InstallError('需要 sudo 安装系统依赖；或使用 --skip-system-deps')
        try:
            self.run('apt-update', [*prefix, 'apt-get', 'update'], terminal=True)
            self.run('apt-install', [*prefix, 'apt-get', 'install', '-y', *packages], terminal=True)
        except InstallError as error:
            if prefix and not sys.stdin.isatty() and 'sudo:' in str(error) and '--yes' not in str(error):
                raise InstallError(f'{error}\n--yes 只跳过安装确认，sudo 仍需已授权的凭据或免密权限\n请在终端运行安装器，或准备好依赖后使用 --skip-system-deps') from error
            raise

    def preflight(self):
        if platform.system() != 'Linux':
            raise InstallError('当前安装执行仅支持 Linux')
        if self.plan.ros2 and not self.ros_setup.is_file():
            raise InstallError(f'未检测到 {self.ros_setup}；请先安装 ROS2 {self.plan.ros_distro}，或选择 standalone')
        # Reject invalid package.xml metadata before apt update/install or rosdep update
        try:
            paths = self.plan.package_paths()
        except ValueError as error:
            raise InstallError(f'安装前 ROS 包检查未通过\n{error}') from error
        if self.plan.ros2:
            print(f'[preflight] ROS 包元数据检查通过（{len(paths)} 个），准备安装依赖', flush=True)
        if self.plan.gui:
            from gui import preflight_gui
            preflight_gui()

    def native(self):
        self.apt(['build-essential', 'cmake', 'python3-venv', 'python3-dev',
                  'libyaml-cpp-dev', 'libeigen3-dev'] + (['libgtest-dev'] if self.plan.tests and (self.root/'src/serial_arm/core/tests').is_dir() else []))
        for command in ('cmake', 'c++'):
            if not shutil.which(command):
                raise InstallError(f'缺少 {command}；请准备构建工具后重试')
        self.env.update(CORE_BUILD_DIR=str(self.build / 'core'),
                        CONAN_DIR=str(self.build / 'conan'),
                        BOOTSTRAP_BUILD_ROOT=str(self.build), INSTALL_PREFIX=str(self.prefix))
        argv = ['bash', self.root / 'tools/bootstrap_standalone.sh']
        if self.plan.robot == 'dm_arm':
            argv += ['--robot', 'dm_arm']
        if self.plan.tests and (self.root/'src/serial_arm/core/tests').is_dir():
            argv += ['--with-tests']
        if not self.plan.terminal:
            argv += ['--without-terminal']
        try:
            self.run('standalone-build', argv)
        except InstallError as error:
            if 'required command not found: conan' not in str(error) and 'Conan 2 is required' not in str(error):
                raise
            # System packages were incomplete. Provision Conan only now, in an
            # isolated venv; do not modify the user's global Python environment
            venv = self.state / 'tools-venv'
            self.run('conan-venv', [sys.executable, '-m', 'venv', str(venv)])
            self.run('conan-install', [venv / 'bin/python', '-m', 'pip', 'install', 'conan>=2.1,<3'])
            self.env['PATH'] = str(venv / 'bin') + os.pathsep + self.env.get('PATH', '')
            self.run('standalone-build-retry', argv)
        if self.plan.python:
            self.wheel()

    def wheel(self):
        venv = self.prefix / 'python-venv'
        self.run('python-venv', [sys.executable, '-m', 'venv', str(venv)])
        self.python = venv / 'bin/python'
        env = standalone_env(self.env)
        # Read actual bootstrap cache to distinguish system dependencies from
        # Conan. A leftover Conan toolchain must not select a stale runtime
        cache = (self.build / 'core/CMakeCache.txt').read_text()
        if 'CMAKE_TOOLCHAIN_FILE:FILEPATH=' in cache or 'CMAKE_TOOLCHAIN_FILE:UNINITIALIZED=' in cache:
            env['CMAKE_ARGS'] = shlex.join(['-DCMAKE_TOOLCHAIN_FILE=' + str(self.build / 'conan/conan_toolchain.cmake'), '-DBUILD_TESTING=OFF'])
        else:
            env['CMAKE_ARGS'] = '-DBUILD_TESTING=OFF'
        env['CMAKE_BUILD_PARALLEL_LEVEL'] = str(self.plan.jobs)
        self.run('python-build-deps', [self.python, '-m', 'pip', 'install',
                                     'scikit-build-core>=0.10', 'numpy>=1.24'], env=env)
        self.run('python-wheel', [self.python, '-m', 'pip', 'install', '--no-build-isolation',
                                 '--force-reinstall', '--no-deps', self.root / 'src/serial_arm/core/python'],
                 env=env, source=self.setup)

    def ros(self):
        self.apt(['build-essential', 'cmake', 'python3-colcon-common-extensions',
                  'python3-rosdep'] + (['python3-dev', 'python3-numpy'] if self.plan.python else []))
        paths = self.plan.package_paths()
        if not self.plan.skip_system_deps:
            if not Path('/etc/ros/rosdep/sources.list.d/20-default.list').is_file():
                prefix = [] if os.geteuid() == 0 else ['sudo'] + ([] if sys.stdin.isatty() else ['-n'])
                self.run('rosdep-init', prefix + ['rosdep', 'init'], terminal=True)
            update = ['rosdep', 'update', '--rosdistro', self.plan.ros_distro]
            if os.geteuid() == 0:
                update.append('--include-eol-distros')
            try:
                self.run('rosdep-update', update, source=self.ros_setup)
            except InstallError as error:
                raise InstallError(
                    f'{error}\n\n[rosdep 更新失败] 检查 ROS 源与网络连接，'
                    '修复后重新执行 rosdep update，不能在依赖索引不可用时跳过校验'
                ) from error
            types = [value for kind in ('build', 'buildtool', 'build_export', 'buildtool_export', 'exec')
                     for value in ('--dependency-types', kind)]
            if self.plan.tests and (self.root/'src/serial_arm/core/tests').exists():
                types += ['--dependency-types', 'test']
            cmd = ['rosdep', 'install', '--from-paths', *paths, '--ignore-src', '-y',
                   '--rosdistro', self.plan.ros_distro, *types]
            if not self.plan.python:
                cmd += ['--skip-keys', 'ament_cmake_python python3-dev python3-numpy']
            try:
                self.run('rosdep-install', cmd, source=self.ros_setup, terminal=True)
            except InstallError as error:
                raise InstallError(
                    f'{error}\n\n[rosdep 排查建议] {rosdep_failure_hint(str(error))}\n'
                    '可以单独用相同参数重试 rosdep，不需要重新执行完整安装器'
                ) from error
        installed_test_sources = (self.root/'src/serial_arm/core/tests').exists()
        run_tests = bool(self.plan.tests and installed_test_sources)
        if self.plan.tests and not run_tests:
            print('开发包不包含 tests/，将继续构建全部组件并跳过内置测试')
        colcon = ['colcon', '--log-base', str(self.logs / 'colcon')]
        build = ['build', '--base-paths', *paths, '--build-base', str(self.build),
                 '--install-base', str(self.prefix), '--merge-install', '--symlink-install',
                 '--parallel-workers', str(self.plan.jobs), '--cmake-args',
                 '-DCMAKE_BUILD_TYPE=Release', '-DSERIAL_ARM_ENABLE_ROS2=ON',
                 '-DSERIAL_ARM_BUILD_PYTHON=' + ('ON' if self.plan.python else 'OFF'),
                 '-DSERIAL_ARM_BUILD_TERMINAL=' + ('ON' if self.plan.terminal else 'OFF'),
                 '-DBUILD_TESTING=' + ('ON' if run_tests else 'OFF')]
        self.run('ros2-build', colcon + build, source=self.ros_setup)
        if run_tests:
            self.run('ros2-test', colcon + ['test', '--base-paths', *paths, '--build-base', str(self.build),
                                         '--install-base', str(self.prefix), '--merge-install', '--return-code-on-test-failure'], source=self.setup)
            self.run('ros2-test-result', ['colcon', 'test-result', '--test-result-base', str(self.build), '--verbose'])
        if self.plan.python:
            self.python = Path('/usr/bin/python3')

    def setup_text(self):
        lines = ['#!/usr/bin/env bash', '# Generated by SerialArm unified installer']
        if self.plan.ros2:
            lines.append('source ' + shlex.quote(str(self.ros_setup)) + ' || return')
        lines.append('source ' + shlex.quote(str(self.setup)) + ' || return')
        lines.append('export PATH=' + shlex.quote(str(self.prefix / 'bin')) + ':"${PATH:-}"')
        if self.plan.python and not self.plan.ros2:
            lines.append('source ' + shlex.quote(str(self.prefix / 'python-venv/bin/activate')) + ' || return')
        if self.plan.ros2:
            # Native profile resolver also benefits from the merged ROS prefix
            lines.append('export SERIAL_ARM_RESOURCE_PATH=' + shlex.quote(str(self.prefix)) +
                         '${SERIAL_ARM_RESOURCE_PATH:+:"${SERIAL_ARM_RESOURCE_PATH}"}')
        return '\n'.join(lines) + '\n'

    def verify(self):
        if not self.setup.is_file():
            raise InstallError(f'构建未生成环境入口：{self.setup}')
        lib = list(self.prefix.glob('lib*/libserial_arm_core.so*'))
        if not lib:
            raise InstallError('未找到安装后的 libserial_arm_core.so')
        candidate = self.logs / 'verify-setup.bash'
        candidate.write_text(self.setup_text())
        # dlopen catches missing runtime dependencies without opening hardware
        python = self.python or Path(sys.executable)
        self.run('verify-core', [python, '-c', 'import ctypes,sys; ctypes.CDLL(sys.argv[1])', lib[0]], source=candidate)
        if self.plan.terminal:
            binary = self.prefix / 'bin/serial_arm_terminal'
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise InstallError(f'未找到 Terminal：{binary}')
            self.run('verify-terminal', [binary, '--help'], source=candidate)
            model_probe = self.prefix / 'bin/serial_arm_model_probe'
            if not model_probe.is_file() or not os.access(model_probe, os.X_OK):
                raise InstallError(f'未找到 Model Probe：{model_probe}')
            self.run('verify-model-probe', [model_probe, '--help'], source=candidate)
            model_calibrator = self.prefix / 'bin/serial_arm_model_calibrator'
            if not model_calibrator.is_file() or not os.access(model_calibrator, os.X_OK):
                raise InstallError(f'未找到 Model Calibrator：{model_calibrator}')
            self.run('verify-model-calibrator', [model_calibrator, '--help'], source=candidate)
        if self.plan.ros2:
            for package in self.plan.packages:
                self.run('verify-' + package, ['ros2', 'pkg', 'prefix', package], source=candidate)
        if self.plan.python:
            self.run('verify-python', [self.python, '-c', 'import serial_arm; print(serial_arm.__file__)'], source=candidate)
            self.run('verify-python-binding',
                     [self.python, self.root / 'tools/check_python_binding.py'],
                     source=candidate)
        if self.plan.robot == 'dm_arm':
            for package in ('serial_arm_robot_profiles', 'dm_arm_description'):
                if not (self.prefix / 'share' / package).is_dir():
                    raise InstallError(f'缺少机器人资源：{package}')
        return candidate.read_text()

    def commit(self, setup_text):
        if native_inputs(self.root, self.plan.describe()) != self.native_inputs:
            raise InstallError('构建期间源码发生变化，请重新执行安装')
        # Generation-specific setup first, manifest last. A failed manifest write
        # never switches the stable environment entry to an uncommitted generation
        generation = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        setup_path = self.state / 'environments' / (generation + '.bash')
        atomic_write(setup_path, setup_text)
        manifest = self.plan.describe()
        manifest.update(schema_version=1, status='verified',
                        verified_at=datetime.now(timezone.utc).isoformat(),
                        installed_components=[c for c in ('core', 'terminal', 'python', 'ros2', 'moveit', 'tests')
                                              if c == 'core' or getattr(self.plan, c)],
                        gui=False, gui_requested=self.plan.gui,
                        robot_supports=[self.plan.robot] if self.plan.robot != 'none' else [],
                        ros_setup=str(self.ros_setup) if self.plan.ros2 else None,
                        workspace_setup=str(setup_path), backend_setup=str(self.setup),
                        install_prefix=str(self.prefix), build_dir=str(self.build), log_dir=str(self.logs),
                        python_executable=str(self.python) if self.python else None)
        manifest.update(getattr(self, 'gui_state', {}))
        if self.plan.ros2:
            manifest['native_receipt'] = {
                **self.native_inputs,
                'installed_sha256': installed_content(self.prefix),
            }
        # Stable entry loads the setup selected by the committed manifest
        entry = self.state / 'setup.bash'
        atomic_write(entry, self.entry_text())
        run_entry = self.state / 'run'
        atomic_write(run_entry, """#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/setup.bash"
if [[ $# -eq 0 ]]; then echo 'Usage: ./.install/run COMMAND [ARGS...]' >&2; exit 2; fi
exec "$@"
""")
        run_entry.chmod(0o755)
        atomic_write(self.state / 'manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')

    def entry_text(self):
        # Python prints one trusted path; shell quotes it as data, never evals it
        return '''#!/usr/bin/env bash
_serial_arm_state_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_serial_arm_setup="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["workspace_setup"])' "$_serial_arm_state_dir/manifest.json")" || return
source "$_serial_arm_setup"
_serial_arm_rc=$?
unset _serial_arm_state_dir _serial_arm_setup
return "$_serial_arm_rc"
'''


def install(plan):
    runner = Runner(plan)
    runner.state.mkdir(parents=True, exist_ok=True)
    with (runner.state / 'install.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('另一个安装进程正在执行，请等待其结束', file=sys.stderr)
            return 3
        attempt = {'preset': plan.preset, 'started_at': datetime.now(timezone.utc).isoformat(),
                   'status': 'running', 'log_dir': str(runner.logs)}
        atomic_write(runner.state / 'last_attempt.json', json.dumps(attempt, indent=2))
        try:
            runner.preflight()
            if plan.gui_only:
                from gui import install_gui
                runner.gui_state = install_gui(runner)
                manifest_file = runner.state / 'manifest.json'
                manifest = json.loads(manifest_file.read_text()) if manifest_file.exists() else {
                    'schema_version': 1, 'status': 'gui_ready', 'installed_components': [],
                    'workspace_setup': str(runner.state / 'gui-setup.bash')}
                if not manifest_file.exists():
                    atomic_write(runner.state / 'gui-setup.bash', '#!/usr/bin/env bash\n:\n')
                    atomic_write(runner.state / 'setup.bash', runner.entry_text())
                manifest.update(runner.gui_state)
                atomic_write(manifest_file, json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
            else:
                runner.native_inputs = native_inputs(runner.root, plan.describe())
                if plan.ros2:
                    runner.ros()
                else:
                    runner.native()
                setup_text = runner.verify()
                if plan.gui:
                    from gui import install_gui
                    runner.gui_state = install_gui(runner)
                runner.commit(setup_text)
            attempt['status'] = 'succeeded'
        except (InstallError, OSError, subprocess.SubprocessError, KeyboardInterrupt) as error:
            attempt.update(status='failed', stage=runner.stage, error=str(error))
            print(f'安装未完成：{error}\n修复以上问题后重新运行同一命令', file=sys.stderr)
            return 130 if isinstance(error, KeyboardInterrupt) else 1
        finally:
            atomic_write(runner.state / 'last_attempt.json', json.dumps(attempt, ensure_ascii=False, indent=2))
    if plan.gui_only:
        print('GUI 安装与检查完成')
    else:
        print('安装与离线检查完成；\n环境入口：source ' + shlex.quote(str(runner.state / 'setup.bash')))
    if plan.terminal and not plan.gui_only:
        command = 'serial_arm_terminal'
        print('Terminal：./.install/run ' + command + (' --robot-profile dm_arm_gray' if plan.robot == 'dm_arm' else ' --help'))
    if plan.gui:
        print('图形工作台：./launch.sh')
    return 0
