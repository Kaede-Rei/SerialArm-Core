#!/usr/bin/env python3
"""Isolated Python process for optional native Pinocchio operations

A crashing NumPy/Pinocchio binary extension must never kill the Launcher IPC
or cause live robot controls to become unavailable
"""
import json
import sys


def main():
    if len(sys.argv) != 5:
        raise ValueError('请选择高级动力学辨识目标与可信 Link 后生成候选模型')
    from advanced_identification import run_advanced
    locked = json.loads(sys.argv[4])
    result = run_advanced(sys.argv[1], sys.argv[3], sys.argv[2] or None, locked)
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'{type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        sys.exit(2)
