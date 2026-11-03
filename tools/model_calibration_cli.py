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

from calibration_jobs import recompute_gravity_isolated, run_full_inertial_isolated

from persistence import (  # noqa: E402
    export_candidate_urdf,
    load_model_calibration_summary,
    preview_gravity_correction,
    restore_gravity_correction,
    save_gravity_correction,
    sha256_file,
    select_gravity_candidate,
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
    from profiles import Inspector
    result=Inspector(ROOT).model_calibrator()
    if not result:
        raise ValueError('未找到 serial_arm_model_calibrator，请先安装包含 Terminal 的组件')
    return result


def recompute(config: Path, directory: Path, mode='global', locked_links=None, *, objective='bias_separated') -> dict:
    # The archived model/config are the fit inputs, exactly as in the GUI
    result=recompute_gravity_isolated(directory,mode,locked_links,objective=objective)
    candidate=result['candidate']
    return {**result, 'candidate_path': result['path'],
            'static_validation_passed': candidate.get('static_pass') is True}


def export_verified(config: Path, directory: Path, destination: str | None, source=None, candidate_path=None) -> dict:
    source, selected = select_gravity_candidate(directory, source, candidate_path)
    exported = (run_full_inertial_isolated(directory,destination,mode='legacy_gravity_export',locked_links={'candidate_path':str(selected)})
                if source=='offline' else export_candidate_urdf(directory, destination))
    calibrator = find_calibrator()
    verify_args = [calibrator, '--config', str(config), '--dataset', str(directory), '--verify-urdf', exported['candidate_urdf'], '--candidate-source', source]
    if selected: verify_args.extend(['--candidate-path', str(selected)])
    completed = subprocess.run(
        verify_args,
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

    idim = commands.add_parser('identify-dynamics', help='滤波完整动力学辨识；可信 Link 从求解开始固定')
    idim.add_argument('--directory', required=True)
    idim.add_argument('--mode', choices=('global','local'), default='global')
    idim.add_argument('--locked-link', action='append', default=[])
    idim.add_argument('--validation-directory', help='同装机、同模型的独立采集会话')
    idim.add_argument('--estimator', choices=('OLS','WLS','IRLS'), default='WLS')
    idim.add_argument('--cutoff-hz', type=float, choices=(2.,5.,10.), default=5.)
    idim.add_argument('--destination', help='新的报告目录；默认按 run_id 归档')
    idim.add_argument('--load-description',help='补充实际已知且全程不变的装机条件，单列为操作者声明，不修改原始元数据')
    idim.add_argument('--fixed-joint', action='append', default=[], help='未采样关节的已知固定位置，例如 joint7=0；不覆盖采样轴')
    audit = commands.add_parser('audit-dynamics', help='辨识采集质量与阶段覆盖审计，不需要硬件')
    audit.add_argument('--directory', required=True)
    dexp = commands.add_parser('export-dynamics', help='导出已经通过独立检查的完整惯性模型，不重新拟合')
    dexp.add_argument('--report', required=True)
    dexp.add_argument('--destination')
    export_kind=dexp.add_mutually_exclusive_group()
    export_kind.add_argument('--review-only',action='store_true',help='导出物理有效但尚未通过完整验证的待审核模型，不授权上机')
    export_kind.add_argument('--report-only',action='store_true',help='导出诊断与参数报告，非物理解也可，不含 URDF')

    inspect = commands.add_parser('inspect', help='查看任务记录')
    inspect.add_argument('--directory', required=True, help='模型校正任务目录')

    recalc = commands.add_parser('recompute', help='离线重算候选模型')
    recalc.add_argument('--config', required=True, help='Core YAML 路径')
    recalc.add_argument('--objective', choices=('urdf','bias_separated'), default='bias_separated',
                        help='bias_separated: 分离反馈常值偏置（兼容默认）；urdf: 直接重力预测（实验对照，未验证实机改善）')
    recalc.add_argument('--mode', choices=('global','local'), default='global')
    recalc.add_argument('--locked-link', action='append', default=[], help='局部辨识固定的可信 Link，可重复指定')
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
    export.add_argument('--source', choices=('online','offline'), help='任务存在离线候选时必须明确选择来源')
    export.add_argument('--directory', required=True, help='模型校正任务目录')
    export.add_argument('--destination', help='候选 URDF 输出目录，默认写入任务目录 candidate')
    for command in (preview, save, export):
        if command is not export: command.add_argument('--source', choices=('online','offline'), help='明确选择在线结果或离线候选')
        choice = command.add_mutually_exclusive_group()
        choice.add_argument('--candidate-path', help='当前任务内归档的 recomputed-candidate.json')
        choice.add_argument('--run-id', help='离线重算返回的 candidate_run_id')
    return value


def main() -> int:
    args = parser().parse_args()
    try:
        if args.command == 'export-dynamics':
            from calibration_jobs import export_dynamics_isolated
            emit(export_dynamics_isolated(args.report,args.destination,review_only=args.review_only,report_only=args.report_only))
            return 0
        directory = require_directory(args.directory)
        if args.command == 'audit-dynamics':
            from calibration_jobs import audit_dynamics_isolated
            emit(audit_dynamics_isolated(directory))
            return 0
        if args.command == 'identify-dynamics':
            from calibration_jobs import identify_dynamics_isolated
            fixed={}
            for item in args.fixed_joint:
                name,value=item.split('=',1)
                if not name or name in fixed:raise ValueError('固定关节名称为空或重复')
                fixed[name]=float(value)
            emit(identify_dynamics_isolated(directory,mode=args.mode,locked_links=args.locked_link,
                validation_directory=args.validation_directory,estimator=args.estimator,
                cutoff_hz=args.cutoff_hz,destination=args.destination,fixed_joint_positions=fixed,load_description=args.load_description))
            return 0
        if args.command == 'inspect':
            emit(load_model_calibration_summary(directory))
            return 0

        config = require_file(args.config, 'Core YAML')
        if args.command == 'recompute':
            emit(recompute(config, directory, args.mode, args.locked_link, objective=args.objective))
        elif args.command == 'preview-save':
            source, selected = select_gravity_candidate(directory, args.source, args.candidate_path, args.run_id)
            emit(preview_gravity_correction(config, directory, candidate_source=source, candidate_path=selected))
        elif args.command == 'save':
            if not args.confirm:
                raise ValueError('保存会修改 Core YAML，请先执行 preview-save 检查差异，再使用 --confirm')
            source, selected = select_gravity_candidate(directory, args.source, args.candidate_path, args.run_id)
            emit(save_gravity_correction(config, sha256_file(config), directory, candidate_source=source, candidate_path=selected))
        elif args.command == 'restore-config':
            if not args.confirm:
                raise ValueError('恢复会替换当前 Core YAML，请使用 --confirm 明确确认')
            emit(restore_gravity_correction(config, directory))
        elif args.command == 'export-urdf':
            source, selected = select_gravity_candidate(directory, args.source, args.candidate_path, args.run_id)
            emit(export_verified(config, directory, args.destination, source, selected))
        else:
            raise ValueError('未知命令')
        return 0
    except Exception as error:
        print(f'操作失败: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
