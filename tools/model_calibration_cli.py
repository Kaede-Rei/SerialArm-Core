#!/usr/bin/env python3
"""Non-GUI commands for model-calibration records and persistence."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / 'apps' / 'launcher' / 'backend'
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from persistence import (  # noqa: E402
    export_candidate_urdf,
    load_model_calibration_summary,
    preview_gravity_correction,
    restore_gravity_correction,
    save_gravity_correction,
    sha256_file,
    update_candidate_urdf_verification,
)


def emit(value) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2))


def require_file(value: str, label: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'{label}不存在: {path}')
    return path


def require_directory(value: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.is_dir():
        raise ValueError(f'任务目录不存在: {path}')
    return path


def find_calibrator() -> str:
    candidates = [
        os.environ.get('SERIAL_ARM_MODEL_CALIBRATOR', ''),
        shutil.which('serial_arm_model_calibrator') or '',
        str(ROOT / '.install' / 'standalone' / 'bin' / 'serial_arm_model_calibrator'),
        str(ROOT / 'install' / 'standalone' / 'bin' / 'serial_arm_model_calibrator'),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file() and os.access(candidate, os.X_OK):
            return str(Path(candidate).resolve())
    for candidate in (ROOT / 'install').glob('unified/*/bin/serial_arm_model_calibrator'):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate.resolve())
    raise ValueError('未找到 serial_arm_model_calibrator，请先安装包含 Terminal 的组件')


def recompute(config: Path, directory: Path) -> dict:
    calibrator = find_calibrator()
    output = directory / 'recomputed-candidate.json'
    completed = subprocess.run(
        [calibrator, '--config', str(config), '--dataset', str(directory), '--output', str(output)],
        text=True, capture_output=True, timeout=120,
    )
    if completed.returncode not in (0, 3):
        raise ValueError((completed.stderr or completed.stdout or '离线重算失败').strip())
    try:
        candidate = json.loads(output.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'离线重算结果不可读取: {error}') from error
    return {
        'candidate_path': str(output),
        'static_validation_passed': completed.returncode == 0,
        'candidate': candidate,
    }


def export_verified(config: Path, directory: Path, destination: str | None) -> dict:
    exported = export_candidate_urdf(directory, destination)
    calibrator = find_calibrator()
    completed = subprocess.run(
        [calibrator, '--config', str(config), '--dataset', str(directory), '--verify-urdf', exported['candidate_urdf']],
        text=True, capture_output=True, timeout=120,
    )
    if completed.returncode != 0:
        raise ValueError((completed.stderr or completed.stdout or '候选 URDF 校验失败').strip())
    try:
        verified = json.loads(completed.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError) as error:
        raise ValueError('候选 URDF 校验结果不可读取') from error
    exported['verification'] = update_candidate_urdf_verification(Path(exported['candidate_urdf']).parent, verified)
    return exported


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        prog='model_calibration_cli.py',
        description='重力模型校正的非 GUI 记录、重算、保存、恢复与候选 URDF 导出入口',
    )
    commands = value.add_subparsers(dest='command', required=True)

    inspect = commands.add_parser('inspect', help='查看任务记录')
    inspect.add_argument('--directory', required=True, help='模型校正任务目录')

    recalc = commands.add_parser('recompute', help='离线重算候选模型')
    recalc.add_argument('--config', required=True, help='Core YAML 路径')
    recalc.add_argument('--directory', required=True, help='模型校正任务目录')

    preview = commands.add_parser('preview-save', help='预览持久化配置差异')
    preview.add_argument('--config', required=True, help='Core YAML 路径')
    preview.add_argument('--directory', required=True, help='模型校正任务目录')

    save = commands.add_parser('save', help='保存候选重力校正并更新 Core YAML 引用')
    save.add_argument('--config', required=True, help='Core YAML 路径')
    save.add_argument('--directory', required=True, help='模型校正任务目录')
    save.add_argument('--confirm', action='store_true', help='确认写入配置文件')

    restore = commands.add_parser('restore-config', help='恢复校正保存前的 Core YAML')
    restore.add_argument('--config', required=True, help='Core YAML 路径')
    restore.add_argument('--directory', required=True, help='模型校正任务目录')
    restore.add_argument('--confirm', action='store_true', help='确认恢复配置文件')

    export = commands.add_parser('export-urdf', help='导出并校验候选 URDF')
    export.add_argument('--config', required=True, help='Core YAML 路径')
    export.add_argument('--directory', required=True, help='模型校正任务目录')
    export.add_argument('--destination', help='候选 URDF 输出目录，默认写入任务目录 candidate')
    return value


def main() -> int:
    args = parser().parse_args()
    try:
        directory = require_directory(args.directory)
        if args.command == 'inspect':
            emit(load_model_calibration_summary(directory))
            return 0

        config = require_file(args.config, 'Core YAML')
        if args.command == 'recompute':
            emit(recompute(config, directory))
        elif args.command == 'preview-save':
            emit(preview_gravity_correction(config, directory))
        elif args.command == 'save':
            if not args.confirm:
                raise ValueError('保存会修改 Core YAML，请先执行 preview-save 检查差异，再使用 --confirm')
            emit(save_gravity_correction(config, sha256_file(config), directory))
        elif args.command == 'restore-config':
            if not args.confirm:
                raise ValueError('恢复会替换当前 Core YAML，请使用 --confirm 明确确认')
            emit(restore_gravity_correction(config, directory))
        elif args.command == 'export-urdf':
            emit(export_verified(config, directory, args.destination))
        else:
            raise ValueError('未知命令')
        return 0
    except Exception as error:
        print(f'操作失败: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
