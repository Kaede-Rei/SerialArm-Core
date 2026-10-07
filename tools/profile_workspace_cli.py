#!/usr/bin/env python3
"""Non-GUI entry points for the SerialArm profile workspace lifecycle."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / 'apps' / 'launcher' / 'backend'
sys.path.insert(0, str(BACKEND))

from profile_library import ProfileLibrary  # noqa: E402
from profiles import Inspector  # noqa: E402


def dump(value):
    print(json.dumps(value, ensure_ascii=False, indent=2))


def config_from_args(args):
    return {
        'profile': args.profile,
        'profile_file': getattr(args, 'profile_file', '') or '',
        'resource_paths': getattr(args, 'resource_paths', '') or '',
        'serial_port': '',
        'baudrate': '',
        'bus': '',
    }


def split_csv(value):
    return [item.strip() for item in (value or '').split(',') if item.strip()]


def main():
    parser = argparse.ArgumentParser(description='SerialArm Profile workspace command line tools')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('library', help='List registered Profiles and readiness')

    inspect_desc = sub.add_parser('inspect-description', help='Inspect a Description directory, ZIP, URDF, or Xacro')
    inspect_desc.add_argument('source')

    discover = sub.add_parser('discover-profile', help='Find Profiles in an external package or YAML')
    discover.add_argument('source')

    register = sub.add_parser('import-profile', help='Register an external Profile in Profile Library')
    register.add_argument('--profile-file', required=True)
    register.add_argument('--profile', required=True)
    register.add_argument('--resource-paths', default='')
    register.add_argument('--policy', choices=('error', 'replace', 'suffix'), default='error')

    remove = sub.add_parser('remove-profile', help='Remove a Profile Library reference without deleting files')
    remove.add_argument('entry_id')

    ready = sub.add_parser('readiness', help='Inspect lifecycle readiness for a Profile')
    ready.add_argument('--profile-file', default='')
    ready.add_argument('--profile', required=True)
    ready.add_argument('--resource-paths', default='')

    mark = sub.add_parser('mark-stage', help='Record operator-confirmed lifecycle evidence')
    mark.add_argument('--profile-file', default='')
    mark.add_argument('--profile', required=True)
    mark.add_argument('--resource-paths', default='')
    mark.add_argument('--stage', required=True, choices=('model_check','bringup','mapping_calibration','geometry_calibration','dynamics_calibration','validation'))
    mark.add_argument('--clear', action='store_true')

    create = sub.add_parser('create-profile', help='Create a draft Profile package from a Description')
    create.add_argument('--source', required=True)
    create.add_argument('--profile', required=True)
    create.add_argument('--destination', required=True)
    create.add_argument('--base-frame', required=True)
    create.add_argument('--tool-frame', required=True)
    create.add_argument('--joints', required=True, help='Comma-separated controlled joint names in order')
    create.add_argument('--hardware-plugin', default='')
    create.add_argument('--bus', default='main_can')
    create.add_argument('--device', default='/dev/ttyACM0')
    create.add_argument('--baudrate', type=int, default=921600)
    create.add_argument('--motor-ids', default='', help='Comma-separated IDs, aligned with --joints')
    create.add_argument('--master-ids', default='', help='Comma-separated master IDs, aligned with --joints')
    create.add_argument('--motor-types', default='', help='Comma-separated motor types, aligned with --joints')
    create.add_argument('--register', action='store_true', help='Add the generated Profile to Profile Library')

    args = parser.parse_args()
    library = ProfileLibrary(ROOT, Inspector(ROOT))

    if args.command == 'library':
        dump(library.list()); return 0
    if args.command == 'inspect-description':
        dump(library.inspect_description(args.source)); return 0
    if args.command == 'discover-profile':
        dump(library.discover_profile_source(args.source)); return 0
    if args.command == 'import-profile':
        dump(library.register(args.profile_file, args.profile, args.resource_paths, args.policy)); return 0
    if args.command == 'remove-profile':
        dump(library.remove(args.entry_id)); return 0
    if args.command == 'readiness':
        dump(library._readiness(config_from_args(args))); return 0
    if args.command == 'mark-stage':
        dump(library.mark_stage(config_from_args(args), args.stage, not args.clear)); return 0
    if args.command == 'create-profile':
        joints = split_csv(args.joints)
        motor_ids = split_csv(args.motor_ids)
        master_ids = split_csv(args.master_ids)
        motor_types = split_csv(args.motor_types)
        actuators = []
        if args.hardware_plugin:
            if not (len(motor_ids) == len(joints) == len(motor_types)):
                raise ValueError('--motor-ids and --motor-types must contain one value per controlled joint')
            if master_ids and len(master_ids) != len(joints):
                raise ValueError('--master-ids must contain one value per controlled joint when supplied')
            for index, joint in enumerate(joints):
                actuators.append({
                    'name': f'actuator{index + 1}',
                    'motor_id': int(motor_ids[index]),
                    'master_id': int(master_ids[index]) if master_ids else 0,
                    'motor_type': motor_types[index],
                })
        result = library.create_profile({
            'source': args.source,
            'profile': args.profile,
            'destination': args.destination,
            'base_frame': args.base_frame,
            'tool_frame': args.tool_frame,
            'joint_names': joints,
            'hardware_plugin': args.hardware_plugin,
            'bus': args.bus,
            'device': args.device,
            'baudrate': args.baudrate,
            'actuators': actuators,
        })
        if args.register:
            result['library'] = library.register(result['profile_file'], result['profile'], result['resource_paths'], 'suffix')
        dump(result); return 0
    return 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(2)
