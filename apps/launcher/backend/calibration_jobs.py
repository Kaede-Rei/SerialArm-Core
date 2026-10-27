"""Shared isolated offline calibration jobs for desktop and CLI entrypoints"""
import json
import os
from pathlib import Path
import subprocess
import sys

def run_full_inertial_isolated(directory, destination=None, *, timeout=120, mode=None, locked_links=None):
    """Never load optional native Pinocchio extensions in the live IPC process"""
    argv = [sys.executable, str(Path(__file__).with_name('full_inertial_worker.py')),
            str(Path(directory).expanduser().resolve()), str(destination or '')]
    argv.extend([mode or 'full', json.dumps(locked_links or [])])
    env = dict(os.environ)
    # Avoid a user-site NumPy 2 overriding the ROS Humble NumPy 1 ABI
    env['PYTHONNOUSERSITE'] = '1'
    operation = {
        'legacy_gravity_recompute': '离线重力模型辨识',
        'legacy_gravity_export': '离线重力模型导出',
    }.get(mode, '高级动力学辨识')
    try:
        completed = subprocess.run(argv, capture_output=True, text=True,
                                   timeout=timeout, env=env)
    except subprocess.TimeoutExpired as error:
        raise ValueError(f'{operation}计算超时，独立计算进程已终止，工作台仍可正常使用') from error
    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip()
        numpy_abi = ('_ARRAY_API not found' in details or
                     'A module that was compiled using NumPy 1.x' in details or
                     'numpy.core.multiarray failed to import' in details)
        if numpy_abi:
            raise ValueError('Pinocchio 与 NumPy ABI 不兼容，当前候选计算已隔离终止，不影响机器人控制界面，请重新运行 ./install.sh 更新 GUI Python 环境中的 numpy<2')
        if completed.returncode < 0:
            raise ValueError(f'{operation}进程被信号 {-completed.returncode} 终止，工作台未受影响，请检查 Python 原生依赖')
        if details.startswith(('ValueError: ', 'RuntimeError: ', 'KeyError: ')):
            details = details.split(': ', 1)[1]
        raise ValueError(f'{operation}失败: ' + (details[-2000:] or f'退出码 {completed.returncode}'))
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise ValueError(f'{operation}进程没有返回有效 JSON') from error



def recompute_gravity_isolated(directory, mode='global', locked_links=None, *, objective='bias_separated'):
    if objective not in ('urdf', 'bias_separated'):
        raise ValueError('未知的离线拟合目标')
    if mode not in ('global','local'):
        raise ValueError('未知的辨识模式')
    locked = locked_links if locked_links is not None else []
    if (not isinstance(locked,list) or any(not isinstance(x,str) or not x for x in locked)
            or len(set(locked))!=len(locked)):
        raise ValueError('可信 Link 列表无效')
    if mode == 'global' and locked:
        raise ValueError('全局辨识不能指定锁定 Link')
    folder=Path(directory).expanduser().resolve()
    if not folder.is_dir():
        raise ValueError('请选择完整的标定任务目录')
    return run_full_inertial_isolated(folder,mode='legacy_gravity_recompute',
        locked_links={'mode':mode,'locked_links':locked,'objective':objective})
