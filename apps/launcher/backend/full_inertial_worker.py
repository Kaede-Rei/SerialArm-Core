#!/usr/bin/env python3
"""Isolated Python process for optional native Pinocchio operations

A crashing NumPy/Pinocchio binary extension must never kill the Launcher IPC
or cause live robot controls to become unavailable
"""
import json
import sys


def main():
    if len(sys.argv) != 3:
        raise ValueError('候选参数导出参数不完整')
    from full_inertial import export_full_inertial_candidate
    result = export_full_inertial_candidate(sys.argv[1], sys.argv[2] or None)
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'{type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        sys.exit(2)
