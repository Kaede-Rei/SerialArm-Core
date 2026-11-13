#!/usr/bin/env python3
"""Replay archived online decisions read-only; all generated output stays in /tmp.

Uses the project's existing C++ compiler, Eigen, yaml-cpp and Python PyYAML.
No motor bus or robot object is constructed.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]


def json_values(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, list):
        return [json_values(item) for item in value]
    if isinstance(value, dict):
        return {key: json_values(item) for key, item in value.items()}
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--record')
    parser.add_argument('--event-index', type=int)
    parser.add_argument('--window-s', type=float)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    output = args.output.resolve()
    if Path('/tmp') not in output.parents:
        parser.error('--output must be within /tmp')
    with tempfile.TemporaryDirectory(prefix='serialarm-recorded-online-') as folder:
        executable = Path(folder) / 'replay'
        intermediate = Path(folder) / 'report.yaml'
        subprocess.run([args.compiler, '-std=c++17', '-O1', '-Wall', '-Wextra', '-Wpedantic',
                        '-I', str(ROOT / 'src/serial_arm/core/include'),
                        '-I', str(ROOT / 'src/serial_arm/core/third_lib/tl/include'),
                        '-I', '/usr/include/eigen3',
                        str(ROOT / 'tools/gain_recorded_online_replay.cpp'),
                        '-lyaml-cpp', '-o', str(executable)], check=True)
        command = [str(executable), '--task', str(args.task.resolve()), '--output', str(intermediate)]
        for key in ('record', 'event_index', 'window_s'):
            value = getattr(args, key)
            if value is not None:
                command.extend(['--' + key.replace('_', '-'), str(value)])
        subprocess.run(command, check=True)
        result = json_values(yaml.safe_load(intermediate.read_text()))
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
